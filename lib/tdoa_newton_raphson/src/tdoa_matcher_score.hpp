#pragma once

#include "tdoa_newton_raphson.hpp"

#include <cmath>

namespace tdoa_estimator {

// Window-information matcher scoring.
//
// The sliding-window estimator publishes its age-decayed information state
// (3x3 info matrix, per-pair decayed weights, tag position). When an anchor
// packet arrives, the matcher scores each eligible candidate pair by the
// E-optimal REFRESH gain: how much the window's weakest eigenvalue grows if
// this pair is measured now. A re-measured pair only credits the weight it
// would recover (full weight minus its current decayed weight), so:
//  - a stale or absent pair aligned with the weak axis scores highest,
//  - a pair that is already fresh in the window scores ~zero,
//  - a small trace term breaks ties so redundancy is still spread when the
//    weak axis is saturated (prevents greedy lock-in on one pair).

// Default full information of one fresh row: 1/sigma^2 with sigma = 0.15 m.
constexpr Scalar kMatcherFullRowWeight = Scalar(1) / (Scalar(0.15) * Scalar(0.15));

// Packed symmetric 3x3: [xx, xy, xz, yy, yz, zz].
struct PackedInfo3 {
    Scalar m[6] = {};

    void addOuter(const PosVector3D& g, Scalar w)
    {
        m[0] += w * g(0) * g(0);
        m[1] += w * g(0) * g(1);
        m[2] += w * g(0) * g(2);
        m[3] += w * g(1) * g(1);
        m[4] += w * g(1) * g(2);
        m[5] += w * g(2) * g(2);
    }
};

// Analytic smallest eigenvalue of a packed symmetric 3x3 matrix
// (trigonometric method) - no iterative solver in the radio hot path.
inline Scalar minEigenvalueSym3(const Scalar p[6])
{
    const Scalar a00 = p[0], a01 = p[1], a02 = p[2];
    const Scalar a11 = p[3], a12 = p[4], a22 = p[5];

    const Scalar off = a01 * a01 + a02 * a02 + a12 * a12;
    if (off <= Scalar(1e-12)) {
        Scalar lo = a00 < a11 ? a00 : a11;
        return lo < a22 ? lo : a22;
    }

    const Scalar q = (a00 + a11 + a22) / Scalar(3);
    const Scalar b00 = a00 - q, b11 = a11 - q, b22 = a22 - q;
    const Scalar p2 = b00 * b00 + b11 * b11 + b22 * b22 + Scalar(2) * off;
    const Scalar pv = std::sqrt(p2 / Scalar(6));
    if (pv <= Scalar(1e-12)) {
        return q;
    }

    // det(B) with B = (A - qI) / pv
    const Scalar inv = Scalar(1) / pv;
    const Scalar c00 = b00 * inv, c01 = a01 * inv, c02 = a02 * inv;
    const Scalar c11 = b11 * inv, c12 = a12 * inv, c22 = b22 * inv;
    Scalar detB = c00 * (c11 * c22 - c12 * c12)
                - c01 * (c01 * c22 - c12 * c02)
                + c02 * (c01 * c12 - c11 * c02);
    Scalar r = detB / Scalar(2);
    if (r < Scalar(-1)) r = Scalar(-1);
    if (r > Scalar(1)) r = Scalar(1);

    const Scalar phi = std::acos(r) / Scalar(3);
    // Smallest eigenvalue: q + 2*pv*cos(phi + 2*pi/3)
    return q + Scalar(2) * pv * std::cos(phi + Scalar(2.0943951023931953));
}

// TDoA row gradient for a pair (A, B) evaluated at the tag position.
inline PosVector3D tdoaPairGradient(const PosVector3D& tag,
                                    const PosVector3D& anchor_a,
                                    const PosVector3D& anchor_b)
{
    PosVector3D da = tag - anchor_a;
    PosVector3D db = tag - anchor_b;
    Scalar na = da.norm();
    Scalar nb = db.norm();
    if (na < Scalar(1e-4)) na = Scalar(1e-4);
    if (nb < Scalar(1e-4)) nb = Scalar(1e-4);
    return da / na - db / nb;
}

// Score a candidate pair against the published window information.
// info: packed decayed window info; w_current: the pair's current decayed
// weight in the window (0 if absent); returns higher = better.
inline Scalar matcherScorePair(const Scalar info[6],
                               const PosVector3D& tag,
                               const PosVector3D& anchor_a,
                               const PosVector3D& anchor_b,
                               Scalar w_current,
                               Scalar w_full = kMatcherFullRowWeight,
                               Scalar trace_blend = Scalar(0.05),
                               Scalar refresh_discount = Scalar(1.6))
{
    // Refresh gain with correlation discount: UWB errors are largely
    // pair-persistent (multipath, delay residuals), so re-measuring a pair
    // that is still fresh in the window repeats its bias rather than adding
    // independent information. Over-counting the current weight forces
    // rotation across distinct pairs; the eigen term picks WHICH stale pair
    // serves the weakest axis.
    Scalar dw = w_full - refresh_discount * w_current;
    if (dw < Scalar(0)) dw = Scalar(0);

    const PosVector3D g = tdoaPairGradient(tag, anchor_a, anchor_b);

    Scalar after[6];
    after[0] = info[0] + dw * g(0) * g(0);
    after[1] = info[1] + dw * g(0) * g(1);
    after[2] = info[2] + dw * g(0) * g(2);
    after[3] = info[3] + dw * g(1) * g(1);
    after[4] = info[4] + dw * g(1) * g(2);
    after[5] = info[5] + dw * g(2) * g(2);

    return minEigenvalueSym3(after) + trace_blend * dw * g.squaredNorm() / Scalar(3);
}

} // namespace tdoa_estimator
