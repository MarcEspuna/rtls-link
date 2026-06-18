#pragma once

// Dependency-free helper for the GEOMETRIC (E-optimal) TDoA matcher policy.
// Header-only and free of DW1000/Arduino/Eigen dependencies so it can be unit
// tested natively even though the tdoa_algorithm library is lib_ignored there.
//
// E-optimal selection: when pairing an incoming packet from anchor A with a
// candidate partner B, prefer the B that maximizes the smallest eigenvalue of
// the window Fisher information sum_g g*g^T after adding the candidate row's
// gradient g = unit(p-A) - unit(p-B), evaluated at the last tag position p.
// Maximizing the minimum eigenvalue directly targets the worst-conditioned
// (most unstable) axis — exactly the geometry that makes TDoA solves jump.

#include <cmath>

namespace tdoa_geometric {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Symmetric 3x3 accumulator stored packed as [xx, xy, xz, yy, yz, zz].
struct SymInfo3 {
    float xx = 0.0f, xy = 0.0f, xz = 0.0f, yy = 0.0f, yz = 0.0f, zz = 0.0f;

    // info <- decay*info + g*g^T
    void accumulate(const Vec3& g, float decay) {
        xx = decay * xx + g.x * g.x;
        xy = decay * xy + g.x * g.y;
        xz = decay * xz + g.x * g.z;
        yy = decay * yy + g.y * g.y;
        yz = decay * yz + g.y * g.z;
        zz = decay * zz + g.z * g.z;
    }
};

inline Vec3 sub(const Vec3& a, const Vec3& b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline float norm(const Vec3& a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

// Row gradient for pair (A,B) evaluated at position p: unit(p-A) - unit(p-B).
inline Vec3 rowGradient(const Vec3& p, const Vec3& A, const Vec3& B) {
    Vec3 da = sub(p, A);
    Vec3 db = sub(p, B);
    float na = norm(da);
    float nb = norm(db);
    if (na < 1e-4f) na = 1e-4f;
    if (nb < 1e-4f) nb = 1e-4f;
    return Vec3{da.x / na - db.x / nb,
                da.y / na - db.y / nb,
                da.z / na - db.z / nb};
}

// Smallest eigenvalue of a symmetric 3x3 matrix (closed form, Smith's method).
inline float minEigenvalue(const SymInfo3& m) {
    const float p1 = m.xy * m.xy + m.xz * m.xz + m.yz * m.yz;
    if (p1 <= 0.0f) {
        // Diagonal matrix: eigenvalues are the diagonal entries.
        float lo = m.xx;
        if (m.yy < lo) lo = m.yy;
        if (m.zz < lo) lo = m.zz;
        return lo;
    }
    const float q = (m.xx + m.yy + m.zz) / 3.0f;
    const float dxx = m.xx - q;
    const float dyy = m.yy - q;
    const float dzz = m.zz - q;
    const float p2 = dxx * dxx + dyy * dyy + dzz * dzz + 2.0f * p1;
    const float p = std::sqrt(p2 / 6.0f);
    if (p <= 0.0f) {
        return q;
    }
    // r = det((A - qI)/p) / 2
    const float b00 = dxx / p, b11 = dyy / p, b22 = dzz / p;
    const float b01 = m.xy / p, b02 = m.xz / p, b12 = m.yz / p;
    const float det =
        b00 * (b11 * b22 - b12 * b12)
        - b01 * (b01 * b22 - b12 * b02)
        + b02 * (b01 * b12 - b11 * b02);
    float r = det / 2.0f;
    if (r < -1.0f) r = -1.0f;
    if (r > 1.0f) r = 1.0f;
    const float phi = std::acos(r) / 3.0f;
    // Smallest eigenvalue corresponds to (phi + 2pi/3).
    const float kTwoPiOver3 = 2.0943951023931953f;
    return q + 2.0f * p * std::cos(phi + kTwoPiOver3);
}

// E-optimal score for adding gradient g to the (decayed) accumulator: the
// minimum eigenvalue of (info + g*g^T). Higher is better.
inline float eOptimalScore(const SymInfo3& info, const Vec3& g) {
    SymInfo3 trial = info;
    trial.accumulate(g, 1.0f);
    return minEigenvalue(trial);
}

} // namespace tdoa_geometric
