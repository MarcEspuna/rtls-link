//! Anchor antenna-delay calibration (inter-anchor ToF based).

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

use tokio::time::sleep;

use crate::cli::{CalibrateArgs, CalibrateCommands, CalibrateAnchorsArgs, RectLayout};
use crate::device::discovery::{discover_devices, DiscoveryOptions, DISCOVERY_PORT};
use crate::device::websocket::{send_command_with_retry, BatchSender};
use crate::error::CliError;
use crate::protocol::commands::Commands;
use crate::protocol::response::parse_json_response;
use crate::types::DeviceRole;

// Must match firmware constant (lib/tdoa_algorithm/src/tag/dynamicAnchorPositions.hpp)
const DW1000_TIME_TO_METERS: f64 = 0.004691763978616;

#[derive(Debug, Clone)]
struct AnchorEndpoint {
    anchor_id: u8,
    ip: String,
}

#[derive(Debug, Clone, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct TdoaDistancesResponse {
    anchor_id: u8,
    antenna_delay: u16,
    active_slots: u8,
    distances: Vec<u16>,
}

#[derive(Debug, Clone, Default)]
struct Samples {
    values: Vec<f64>,
}

impl Samples {
    fn add(&mut self, x: f64) {
        if x.is_finite() {
            self.values.push(x);
        }
    }

    fn count(&self) -> u32 {
        self.values.len() as u32
    }

    fn robust_mean(&self) -> Option<(f64, u32)> {
        if self.values.is_empty() {
            return None;
        }

        // Median and MAD-based trimming: protects against transient bogus reads.
        let median = median_f64(&self.values)?;
        let deviations: Vec<f64> = self.values.iter().map(|v| (v - median).abs()).collect();
        let mad = median_f64(&deviations).unwrap_or(0.0);
        let sigma = (mad / 0.6745).max(1e-6);

        // Minimum band keeps small MAD from rejecting everything on low-noise links.
        let threshold = (3.0 * sigma).max(50.0);

        let mut sum = 0.0;
        let mut n = 0u32;
        for v in &self.values {
            if (v - median).abs() <= threshold {
                sum += *v;
                n += 1;
            }
        }

        if n == 0 {
            Some((median, 1))
        } else {
            Some((sum / (n as f64), n))
        }
    }
}

#[derive(Debug, Clone)]
struct Measurement {
    i: usize,
    j: usize,
    b: f64,      // desired sum: a_i + a_j = b
    weight: f64, // base weight (e.g. sample count)
}

pub async fn run_calibrate(args: CalibrateArgs, timeout_ms: u64, json: bool) -> Result<(), CliError> {
    match args.command {
        CalibrateCommands::Anchors(a) => run_calibrate_anchors(a, timeout_ms, json).await,
    }
}

async fn run_calibrate_anchors(args: CalibrateAnchorsArgs, timeout_ms: u64, json: bool) -> Result<(), CliError> {
    // Only supports 4-anchor rectangular layouts for now.
    let required_anchor_ids: [u8; 4] = [0, 1, 2, 3];

    let anchors = resolve_anchors(&args, timeout_ms).await?;

    // Ensure we have 0..3 once each.
    let mut anchor_map: HashMap<u8, AnchorEndpoint> = HashMap::new();
    for a in anchors {
        let anchor_id = a.anchor_id;
        if required_anchor_ids.contains(&anchor_id) {
            if anchor_map.insert(anchor_id, a).is_some() {
                return Err(CliError::Other(format!(
                    "Duplicate anchorId {} discovered; check device IDs",
                    anchor_id
                )));
            }
        }
    }

    for id in required_anchor_ids {
        if !anchor_map.contains_key(&id) {
            return Err(CliError::Other(format!(
                "Missing anchorId {}. Need anchors 0..3 in anchor_tdoa mode.",
                id
            )));
        }
    }

    let mut endpoints: Vec<AnchorEndpoint> = required_anchor_ids
        .iter()
        .map(|id| anchor_map.get(id).unwrap().clone())
        .collect();
    endpoints.sort_by_key(|e| e.anchor_id);

    // Build target distances (meters) for all pairs 0..3.
    let target_m = build_rectangular_targets(args.layout.clone(), args.x, args.y);
    let target_ticks = target_m.map(|row| row.map(|d| d / DW1000_TIME_TO_METERS));

    let max_iters = args.max_iters.max(1);
    let mut prev_rms: Option<f64> = None;
    let mut current_delays: [f64; 4] = [0.0; 4];

    // Prime prior delays from devices to avoid regularizing toward 0 on partial samples.
    for ep in &endpoints {
        let resp = send_command_with_retry(
            &ep.ip,
            Commands::tdoa_distances(),
            Duration::from_millis(timeout_ms),
            1,
        )
        .await?;
        let parsed: TdoaDistancesResponse = parse_json_response(&resp, &ep.ip).map_err(CliError::Device)?;
        if parsed.anchor_id < 4 {
            current_delays[parsed.anchor_id as usize] = parsed.antenna_delay as f64;
        }
    }

    let mut json_iterations: Vec<serde_json::Value> = Vec::new();
    let mut json_final: Option<serde_json::Value> = None;

    for iter in 0..max_iters {
        let sample_result = sample_inter_anchor_distances(
            &endpoints,
            Duration::from_secs(args.sample_duration),
            Duration::from_millis(args.sample_interval_ms),
            args.min_samples,
            timeout_ms,
        )
        .await?;

        // Use last seen antenna delays as prior
        for (idx, id) in required_anchor_ids.iter().enumerate() {
            if let Some(d) = sample_result.anchor_delays.get(id) {
                current_delays[idx] = *d as f64;
            }
        }

        // Soft warning about low sample counts. We can still solve if the
        // measurement graph is connected, but results may be noisy.
        if !json {
            let mut low_pairs: Vec<(u8, u8, u32)> = Vec::new();
            for i in 0..4u8 {
                for j in (i + 1)..4u8 {
                    let c = sample_result.pair_samples[i as usize][j as usize].count();
                    if c > 0 && c < args.min_samples {
                        low_pairs.push((i, j, c));
                    }
                }
            }
            if !low_pairs.is_empty() {
                println!(
                    "  Note: low samples for {} pair(s): {}",
                    low_pairs.len(),
                    low_pairs
                        .iter()
                        .map(|(i, j, c)| format!("{}-{}:{}/{}", i, j, c, args.min_samples))
                        .collect::<Vec<_>>()
                        .join("  ")
                );
            }
        }

        let measurements = build_measurements(&sample_result.pair_samples, &target_ticks);
        ensure_measurement_graph_ok(&measurements, 4)?;

        let solved = solve_antenna_delays_irls(&measurements, &current_delays, args.prior_sigma_ticks)
            .map_err(CliError::Other)?;

        let solved_u16: [u16; 4] = solved
            .iter()
            .map(|v| v.round().clamp(0.0, 65535.0) as u16)
            .collect::<Vec<_>>()
            .try_into()
            .unwrap();

        let current_u16: [u16; 4] = current_delays.map(|v| v.round().clamp(0.0, 65535.0) as u16);
        let report_before = compute_error_report(&sample_result.pair_samples, &target_m, &current_u16);
        let report_after = compute_error_report(&sample_result.pair_samples, &target_m, &solved_u16);

        if json {
            json_iterations.push(serde_json::json!({
                "iteration": iter + 1,
                "delays": { "0": solved_u16[0], "1": solved_u16[1], "2": solved_u16[2], "3": solved_u16[3] },
                "error": report_after.to_json(),
            }));
            json_final = Some(serde_json::json!({
                "delays": { "0": solved_u16[0], "1": solved_u16[1], "2": solved_u16[2], "3": solved_u16[3] },
                "error": report_after.to_json(),
            }));
        } else {
            println!(
                "Iteration {}/{}: RMS error {:.3} m (was {:.3} m) (max {:.3} m)",
                iter + 1,
                max_iters,
                report_after.rms_m,
                report_before.rms_m,
                report_after.max_abs_m
            );
            for (idx, ep) in endpoints.iter().enumerate() {
                let prior = current_delays[idx].round().clamp(0.0, 65535.0) as i64;
                let next = solved_u16[idx] as i64;
                let delta = next - prior;
                println!(
                    "  A{} {}  ADelay {} -> {} ({:+})",
                    ep.anchor_id, ep.ip, prior, next, delta
                );
            }
            println!(
                "  Pair errors (m): {}",
                report_after
                    .pair_errors_m
                    .iter()
                    .map(|((i, j), e)| format!("{}-{}:{:+.3}", i, j, e))
                    .collect::<Vec<_>>()
                    .join("  ")
            );
        }

        if args.dry_run {
            break;
        }

        // Safety guard: refuse to apply extremely large jumps; resample instead.
        let mut too_large: Vec<(u8, i64)> = Vec::new();
        let mut any_change = false;
        for (idx, ep) in endpoints.iter().enumerate() {
            let prior = current_delays[idx].round().clamp(0.0, 65535.0) as i64;
            let next = solved_u16[idx] as i64;
            let delta = next - prior;
            if delta != 0 {
                any_change = true;
            }
            if delta.abs() > args.max_delta_ticks as i64 {
                too_large.push((ep.anchor_id, delta));
            }
        }
        if !too_large.is_empty() {
            if !json {
                println!(
                    "Refusing to apply: delta exceeds --max-delta-ticks ({}): {}",
                    args.max_delta_ticks,
                    too_large
                        .iter()
                        .map(|(id, d)| format!("A{}:{:+}", id, d))
                        .collect::<Vec<_>>()
                        .join("  ")
                );
            }
            if iter + 1 < max_iters {
                continue;
            }
            return Err(CliError::Other(
                "Calibration aborted: unsafe antenna-delay jump suggested (check measurements/layout)".to_string(),
            ));
        }

        // Safety guard: only apply if the predicted RMS improves with the new delays.
        // If not, resample (likely transient bad measurements).
        if any_change && report_after.rms_m.is_finite() && report_before.rms_m.is_finite() && report_after.rms_m >= report_before.rms_m {
            if !json {
                println!(
                    "Not applying: predicted RMS did not improve (was {:.3} m, would be {:.3} m). Resampling...",
                    report_before.rms_m, report_after.rms_m
                );
            }
            if iter + 1 < max_iters {
                continue;
            }
            return Err(CliError::Other(
                "Calibration aborted: could not find an improving antenna-delay update (check measurements/layout)".to_string(),
            ));
        }

        if !any_change {
            if !json {
                println!("No antenna-delay change suggested; stopping.");
            }
            break;
        }

        apply_delays(&endpoints, &solved_u16, timeout_ms).await?;

        // Stop conditions
        if report_after.rms_m <= args.tolerance_m {
            if !json {
                println!(
                    "Calibration converged: RMS {:.3} m <= {:.3} m",
                    report_after.rms_m, args.tolerance_m
                );
            }
            break;
        }

        if let Some(prev) = prev_rms {
            let improvement = prev - report_after.rms_m;
            if improvement < args.min_improvement_m {
                if !json {
                    println!(
                        "Stopping: improvement {:.3} m < {:.3} m (safety guard)",
                        improvement, args.min_improvement_m
                    );
                }
                break;
            }
        }
        prev_rms = Some(report_after.rms_m);
    }

    if json {
        let out = serde_json::json!({
            "layout": format!("{:?}", args.layout),
            "x_m": args.x,
            "y_m": args.y,
            "anchors": endpoints.iter().map(|e| serde_json::json!({"anchorId": e.anchor_id, "ip": e.ip})).collect::<Vec<_>>(),
            "iterations": json_iterations,
            "final": json_final,
            "dryRun": args.dry_run
        });
        println!("{}", serde_json::to_string_pretty(&out).unwrap());
    }

    Ok(())
}

async fn resolve_anchors(args: &CalibrateAnchorsArgs, timeout_ms: u64) -> Result<Vec<AnchorEndpoint>, CliError> {
    let ips = if let Some(raw) = &args.ips {
        raw.split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>()
    } else {
        let options = DiscoveryOptions {
            port: DISCOVERY_PORT,
            duration: Duration::from_secs(args.discovery_duration),
            watch: false,
            on_device: None,
        };
        let devices = discover_devices(options).await?;
        devices
            .into_iter()
            .filter(|d| d.role == DeviceRole::AnchorTdoa)
            .map(|d| d.ip)
            .collect::<Vec<_>>()
    };

    if ips.is_empty() {
        return Err(CliError::NoDevicesFound);
    }

    // Resolve anchorId from each IP using tdoa-distances (single shot).
    let mut endpoints = Vec::new();
    for ip in ips {
        let resp = send_command_with_retry(
            &ip,
            Commands::tdoa_distances(),
            Duration::from_millis(timeout_ms),
            1,
        )
        .await?;
        let parsed: TdoaDistancesResponse = parse_json_response(&resp, &ip).map_err(CliError::Device)?;
        endpoints.push(AnchorEndpoint {
            anchor_id: parsed.anchor_id,
            ip,
        });
    }

    Ok(endpoints)
}

struct SampleResult {
    pair_samples: [[Samples; 4]; 4], // only valid for i<j
    anchor_delays: HashMap<u8, u16>, // latest per-anchor delay observed
}

async fn sample_inter_anchor_distances(
    endpoints: &[AnchorEndpoint],
    duration: Duration,
    interval: Duration,
    min_samples: u32,
    timeout_ms: u64,
) -> Result<SampleResult, CliError> {
    let start = Instant::now();
    let sender = BatchSender::new(timeout_ms, endpoints.len().max(1));
    let ips: Vec<String> = endpoints.iter().map(|e| e.ip.clone()).collect();

    let mut pair_samples: [[Samples; 4]; 4] = std::array::from_fn(|_| std::array::from_fn(|_| Samples::default()));
    let mut anchor_delays: HashMap<u8, u16> = HashMap::new();

    let required_pairs: Vec<(u8, u8)> = (0u8..4)
        .flat_map(|i| (i + 1..4).map(move |j| (i, j)))
        .collect();

    while start.elapsed() < duration {
        let results = sender.send_to_all(&ips, Commands::tdoa_distances()).await;
        for (ip, result) in results {
            let resp = match result {
                Ok(r) => r,
                Err(_e) => continue,
            };
            let parsed: TdoaDistancesResponse = match parse_json_response(&resp, &ip) {
                Ok(p) => p,
                Err(_e) => continue,
            };

            let anchor_id = parsed.anchor_id;
            if anchor_id >= 4 {
                continue;
            }

            anchor_delays.insert(anchor_id, parsed.antenna_delay);

            // Firmware always reports 8 entries; be defensive.
            for (remote_id, dist) in parsed.distances.iter().enumerate() {
                let remote_id = remote_id as u8;
                if remote_id >= 4 || remote_id == anchor_id {
                    continue;
                }
                if *dist == 0 {
                    continue;
                }

                let (i, j) = if anchor_id < remote_id {
                    (anchor_id, remote_id)
                } else {
                    (remote_id, anchor_id)
                };
                pair_samples[i as usize][j as usize].add(*dist as f64);
            }
        }

        let done = required_pairs
            .iter()
            .all(|(i, j)| pair_samples[*i as usize][*j as usize].count() >= min_samples);
        if done {
            break;
        }

        sleep(interval).await;
    }

    Ok(SampleResult {
        pair_samples,
        anchor_delays,
    })
}

fn build_rectangular_targets(layout: RectLayout, x: f64, y: f64) -> [[f64; 4]; 4] {
    let mut pos = [(0.0f64, 0.0f64); 4];

    let (x_anchor, y_anchor, corner) = match layout {
        RectLayout::RectangularA1xA3y => (1usize, 3usize, 2usize),
        RectLayout::RectangularA1xA2y => (1usize, 2usize, 3usize),
        RectLayout::RectangularA3xA1y => (3usize, 1usize, 2usize),
        RectLayout::RectangularA2xA3y => (2usize, 3usize, 1usize),
    };

    pos[0] = (0.0, 0.0);
    pos[x_anchor] = (x, 0.0);
    pos[y_anchor] = (0.0, y);
    pos[corner] = (x, y);

    let mut d = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in i + 1..4 {
            let dx = pos[i].0 - pos[j].0;
            let dy = pos[i].1 - pos[j].1;
            let dist = (dx * dx + dy * dy).sqrt();
            d[i][j] = dist;
            d[j][i] = dist;
        }
    }
    d
}

fn build_measurements(pair_samples: &[[Samples; 4]; 4], target_ticks: &[[f64; 4]; 4]) -> Vec<Measurement> {
    let mut measurements = Vec::new();
    for i in 0..4usize {
        for j in i + 1..4usize {
            let Some((raw_mean, inliers)) = pair_samples[i][j].robust_mean() else { continue };
            let b = raw_mean - target_ticks[i][j];
            let w = (inliers as f64).max(1.0);
            measurements.push(Measurement {
                i,
                j,
                b,
                weight: w,
            });
        }
    }
    measurements
}

fn ensure_measurement_graph_ok(measurements: &[Measurement], n: usize) -> Result<(), CliError> {
    // Need at least a connected graph to solve anything meaningful.
    let mut adj = vec![Vec::<usize>::new(); n];
    for m in measurements {
        adj[m.i].push(m.j);
        adj[m.j].push(m.i);
    }
    let mut seen = vec![false; n];
    let mut stack = vec![0usize];
    seen[0] = true;
    while let Some(u) = stack.pop() {
        for &v in &adj[u] {
            if !seen[v] {
                seen[v] = true;
                stack.push(v);
            }
        }
    }
    if seen.iter().all(|v| *v) {
        // For edge-sum equations (a_i + a_j), connected bipartite graphs have a 1D nullspace
        // (alternating sign vector). Require at least one odd cycle (non-bipartite) for a unique solution.
        let mut color: Vec<Option<bool>> = vec![None; n];
        let mut q = VecDeque::new();
        color[0] = Some(false);
        q.push_back(0usize);
        let mut is_bipartite = true;

        while let Some(u) = q.pop_front() {
            let cu = color[u].unwrap();
            for &v in &adj[u] {
                match color[v] {
                    None => {
                        color[v] = Some(!cu);
                        q.push_back(v);
                    }
                    Some(cv) => {
                        if cv == cu {
                            is_bipartite = false;
                            break;
                        }
                    }
                }
            }
            if !is_bipartite {
                break;
            }
        }

        if is_bipartite {
            return Err(CliError::Other(
                "Insufficient inter-anchor measurements: missing at least one diagonal pair (graph is bipartite)".to_string(),
            ));
        }

        return Ok(());
    }
    Err(CliError::Other(
        "Insufficient inter-anchor measurements: graph not connected (check UWB sync/TDMA)".to_string(),
    ))
}

fn solve_antenna_delays_irls(
    measurements: &[Measurement],
    prior: &[f64; 4],
    prior_sigma_ticks: f64,
) -> Result<Vec<f64>, String> {
    if measurements.is_empty() {
        return Err("No inter-anchor measurements available".to_string());
    }

    let n = 4usize;
    let base_weights: Vec<f64> = measurements.iter().map(|m| m.weight).collect();
    let mut weights = base_weights.clone();

    let mean_w = base_weights.iter().sum::<f64>() / (base_weights.len() as f64);
    let lambda = if prior_sigma_ticks > 0.0 {
        mean_w / (prior_sigma_ticks * prior_sigma_ticks)
    } else {
        0.0
    };

    let mut x = prior.to_vec();
    for _ in 0..3 {
        // Normal equations: (A^T W A + λI) x = A^T W b + λ x0
        let mut ata = vec![vec![0.0f64; n]; n];
        let mut atb = vec![0.0f64; n];

        for (k, m) in measurements.iter().enumerate() {
            let w = weights[k];
            let i = m.i;
            let j = m.j;

            ata[i][i] += w;
            ata[j][j] += w;
            ata[i][j] += w;
            ata[j][i] += w;

            atb[i] += w * m.b;
            atb[j] += w * m.b;
        }

        for i in 0..n {
            ata[i][i] += lambda;
            atb[i] += lambda * prior[i];
        }

        x = solve_linear_system(ata, atb)?;

        // Robust reweighting (Huber) in the residual domain (ticks).
        let residuals: Vec<f64> = measurements
            .iter()
            .map(|m| x[m.i] + x[m.j] - m.b)
            .collect();

        let scale = robust_scale_mad(&residuals).max(1e-6);
        let delta = 1.5 * scale;

        for (k, r) in residuals.iter().enumerate() {
            let a = r.abs();
            let huber = if a <= delta { 1.0 } else { delta / a };
            weights[k] = base_weights[k] * huber;
        }
    }

    Ok(x)
}

fn robust_scale_mad(residuals: &[f64]) -> f64 {
    if residuals.is_empty() {
        return 0.0;
    }
    let mut abs: Vec<f64> = residuals.iter().map(|r| r.abs()).collect();
    abs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = abs.len() / 2;
    let med = if abs.len() % 2 == 0 {
        (abs[mid - 1] + abs[mid]) / 2.0
    } else {
        abs[mid]
    };
    // MAD to sigma factor for normal distribution
    med / 0.6745
}

fn solve_linear_system(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Result<Vec<f64>, String> {
    let n = b.len();
    if a.len() != n || a.iter().any(|row| row.len() != n) {
        return Err("Invalid system dimensions".to_string());
    }

    for i in 0..n {
        // Partial pivot
        let mut pivot = i;
        let mut max = a[i][i].abs();
        for r in i + 1..n {
            let v = a[r][i].abs();
            if v > max {
                max = v;
                pivot = r;
            }
        }
        if max < 1e-12 {
            return Err("Singular system (insufficient geometry/measurements)".to_string());
        }
        if pivot != i {
            a.swap(i, pivot);
            b.swap(i, pivot);
        }

        // Normalize pivot row
        let diag = a[i][i];
        for c in i..n {
            a[i][c] /= diag;
        }
        b[i] /= diag;

        // Eliminate
        for r in 0..n {
            if r == i {
                continue;
            }
            let factor = a[r][i];
            if factor.abs() < 1e-12 {
                continue;
            }
            for c in i..n {
                a[r][c] -= factor * a[i][c];
            }
            b[r] -= factor * b[i];
        }
    }

    Ok(b)
}

fn median_f64(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    Some(if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    })
}

#[derive(Debug, Clone)]
struct ErrorReport {
    pair_errors_m: Vec<((u8, u8), f64)>,
    rms_m: f64,
    max_abs_m: f64,
}

impl ErrorReport {
    fn to_json(&self) -> serde_json::Value {
        serde_json::json!({
            "rms_m": self.rms_m,
            "max_abs_m": self.max_abs_m,
            "pairs": self.pair_errors_m.iter().map(|((i,j),e)| serde_json::json!({"a": i, "b": j, "error_m": e})).collect::<Vec<_>>()
        })
    }
}

fn compute_error_report(pair_samples: &[[Samples; 4]; 4], target_m: &[[f64; 4]; 4], delays: &[u16; 4]) -> ErrorReport {
    let mut errs = Vec::new();
    let mut sum_sq = 0.0;
    let mut count = 0u32;
    let mut max_abs: f64 = 0.0;

    for i in 0..4u8 {
        for j in (i + 1)..4u8 {
            let Some((raw_mean, _inliers)) = pair_samples[i as usize][j as usize].robust_mean() else { continue };
            let corrected_ticks = raw_mean - (delays[i as usize] as f64) - (delays[j as usize] as f64);
            let corrected_m = corrected_ticks * DW1000_TIME_TO_METERS;
            let e = corrected_m - target_m[i as usize][j as usize];
            errs.push(((i, j), e));
            sum_sq += e * e;
            count += 1;
            max_abs = max_abs.max(e.abs());
        }
    }

    let rms = if count > 0 {
        (sum_sq / (count as f64)).sqrt()
    } else {
        f64::NAN
    };

    ErrorReport {
        pair_errors_m: errs,
        rms_m: rms,
        max_abs_m: max_abs,
    }
}

async fn apply_delays(endpoints: &[AnchorEndpoint], delays: &[u16; 4], timeout_ms: u64) -> Result<(), CliError> {
    let timeout = Duration::from_millis(timeout_ms);

    // Write per-anchor ADelay
    for ep in endpoints {
        if ep.anchor_id >= 4 {
            continue;
        }
        let value = delays[ep.anchor_id as usize].to_string();
        let cmd = Commands::write_param("uwb", "ADelay", &value);
        send_command_with_retry(&ep.ip, &cmd, timeout, 1).await?;

        // Best-effort verify that the broadcast value updated (runtime propagation)
        for _ in 0..10 {
            let resp = send_command_with_retry(&ep.ip, Commands::tdoa_distances(), timeout, 0).await?;
            let parsed: TdoaDistancesResponse = parse_json_response(&resp, &ep.ip).map_err(CliError::Device)?;
            if parsed.antenna_delay == delays[ep.anchor_id as usize] {
                break;
            }
            sleep(Duration::from_millis(100)).await;
        }
    }

    Ok(())
}
