# SPDX-License-Identifier: MIT
"""Camera-to-LiDAR rotation and clock offset from two rigidly mounted gyroscopes.

Rotating the rig makes both gyroscopes measure one angular velocity in their own
axes. The clock offset maximizes the correlation of the rotation-invariant
magnitudes |w|; the rotation is the orthogonal polar factor of sum(l c^T) over
time-aligned moving samples (a reflection is rejected as an axis error). The
weakest singular value shows whether every axis was excited.

MAX2 image axes in GPMF stored order (camera Z, X, Y): the sphere's forward axis
is -Y, its down axis is -Z and its right axis is -X. This was established on one
MAX2 (firmware H24.02.01.22.00) by projecting LiDAR depth edges into the
panorama for the four horizontal candidates; it is a property of the camera
model, not of a mount.
"""

from __future__ import annotations

from bisect import bisect_left
import math

# Columns are the sphere frame S (OpenCV x right, y down, z forward) in GPMF stored axes.
GPMF_TO_SPHERE = ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def det(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))


def inverse(a):
    d = det(a)
    if abs(d) < 1e-15:
        raise ValueError("Singular matrix")
    cof = [[(a[(i + 1) % 3][(j + 1) % 3] * a[(i + 2) % 3][(j + 2) % 3]
             - a[(i + 1) % 3][(j + 2) % 3] * a[(i + 2) % 3][(j + 1) % 3]) for j in range(3)]
           for i in range(3)]
    return tuple(tuple(cof[j][i] / d for j in range(3)) for i in range(3))


def polar_rotation(b):
    """Orthogonal polar factor of b (closest orthogonal matrix) by Newton iteration."""
    r = b
    for _ in range(100):
        inv_t = transpose(inverse(r))
        nxt = tuple(tuple(0.5 * (r[i][j] + inv_t[i][j]) for j in range(3)) for i in range(3))
        if max(abs(nxt[i][j] - r[i][j]) for i in range(3) for j in range(3)) < 1e-14:
            return nxt
        r = nxt
    return r


def singular_values(b):
    """Singular values of a 3x3 matrix from the eigenvalues of b^T b (closed form)."""
    m = matmul(transpose(b), b)
    p1 = m[0][1] ** 2 + m[0][2] ** 2 + m[1][2] ** 2
    q = (m[0][0] + m[1][1] + m[2][2]) / 3
    p2 = sum((m[i][i] - q) ** 2 for i in range(3)) + 2 * p1
    if p2 <= 0:
        return [math.sqrt(max(q, 0))] * 3
    p = math.sqrt(p2 / 6)
    bm = tuple(tuple((m[i][j] - (q if i == j else 0)) / p for j in range(3)) for i in range(3))
    phi = math.acos(max(-1.0, min(1.0, det(bm) / 2))) / 3
    eig = [q + 2 * p * math.cos(phi), q + 2 * p * math.cos(phi + 2 * math.pi / 3)]
    eig.append(3 * q - sum(eig))
    return sorted((math.sqrt(max(e, 0)) for e in eig), reverse=True)


def quaternion(r):
    """wxyz quaternion of a rotation matrix."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = (s / 4, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
    else:
        i = max(range(3), key=lambda k: r[k][k])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + r[i][i] - r[j][j] - r[k][k])
        v = [0.0, 0.0, 0.0]
        v[i], v[j], v[k] = s / 4, (r[j][i] + r[i][j]) / s, (r[k][i] + r[i][k]) / s
        q = ((r[k][j] - r[j][k]) / s, *v)
    norm = math.sqrt(sum(x * x for x in q))
    return tuple(x / norm for x in (q if q[0] >= 0 else [-x for x in q]))


class Signal:
    """Centered moving average of irregular vector samples, queried at any time.

    Linear interpolation between noisy samples has lower variance midway between
    samples than at them, which biases a residual-minimizing offset toward
    particular phases. A fixed-width window keeps the noise level independent of
    the query time.
    """

    def __init__(self, times_ns, samples, half_window_ns=4_000_000):
        if len(times_ns) < 2:
            raise ValueError("Too few gyro samples")
        self.times, self.half = list(times_ns), half_window_ns
        self.sums = [(0.0, 0.0, 0.0)]
        for sample in samples:
            last = self.sums[-1]
            self.sums.append(tuple(a + b for a, b in zip(last, sample)))
        self.start = self.times[0] + half_window_ns
        self.end = self.times[-1] - half_window_ns

    def at(self, t):
        if not self.start <= t <= self.end:
            return None
        lo = bisect_left(self.times, t - self.half)
        hi = bisect_left(self.times, t + self.half + 1)
        if hi - lo < 2:
            return None
        return tuple((b - a) / (hi - lo) for a, b in zip(self.sums[lo], self.sums[hi]))


def correlation(xs, ys):
    n = len(xs)
    if n < 3:
        return -1.0
    mx, my = sum(xs) / n, sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    return sxy / (sx * sy) if sx > 0 and sy > 0 else -1.0


def magnitude_correlation(camera: Signal, lidar_times, lidar_mags, offset_ns, stride=1):
    xs, ys = [], []
    for t, m in zip(lidar_times[::stride], lidar_mags[::stride]):
        c = camera.at(t - offset_ns)
        if c is not None:
            xs.append(math.sqrt(c[0] ** 2 + c[1] ** 2 + c[2] ** 2))
            ys.append(m)
    return correlation(xs, ys) if len(xs) > 200 else -1.0, len(xs)


def find_offset(camera: Signal, lidar_times, lidar_samples, guess_ns, search_ns=3_000_000_000):
    """Offset d with lidar_time = camera_media_time + d, from |w| correlation."""
    mags = [math.sqrt(x * x + y * y + z * z) for x, y, z in lidar_samples]
    best = max((magnitude_correlation(camera, lidar_times, mags, d, stride=4)[0], d)
               for d in range(guess_ns - search_ns, guess_ns + search_ns + 1, 10_000_000))
    fine = max((magnitude_correlation(camera, lidar_times, mags, d)[0], d)
               for d in range(best[1] - 20_000_000, best[1] + 20_000_001, 250_000))
    return fine[1], fine[0]


def solve_rotation(camera: Signal, lidar_times, lidar_samples, offset_ns, min_rate=0.2):
    """R with lidar_w ~= R @ camera_w, plus fit diagnostics."""
    pairs = []
    for t, l in zip(lidar_times, lidar_samples):
        c = camera.at(t - offset_ns)
        if c is not None and math.sqrt(sum(v * v for v in l)) > min_rate:
            pairs.append((l, c))
    if len(pairs) < 100:
        raise ValueError("Too little rotation to solve the camera orientation")
    b = tuple(tuple(sum(l[i] * c[j] for l, c in pairs) for j in range(3)) for i in range(3))
    if det(b) <= 0:
        raise ValueError("Gyro axes fit only with a reflection; check the axis order")
    r = polar_rotation(b)
    residual = math.sqrt(sum(sum((l[i] - sum(r[i][k] * c[k] for k in range(3))) ** 2
                                 for i in range(3)) for l, c in pairs) / len(pairs))
    signal = math.sqrt(sum(sum(v * v for v in l) for l, _ in pairs) / len(pairs))
    s = singular_values(b)
    return r, {"moving_samples": len(pairs), "residual_rms_rad_s": residual,
               "signal_rms_rad_s": signal, "singular_value_ratios": [v / s[0] for v in s]}


def vector_residual(camera: Signal, lidar_times, lidar_samples, rotation, offset_ns, stride=2):
    total = count = 0
    for t, l in zip(lidar_times[::stride], lidar_samples[::stride]):
        c = camera.at(t - offset_ns)
        if c is not None:
            total += sum((l[i] - sum(rotation[i][k] * c[k] for k in range(3))) ** 2
                         for i in range(3))
            count += 1
    return math.sqrt(total / count) if count > 200 else math.inf


def refine_offset(camera: Signal, lidar_times, lidar_samples, rotation, offset_ns,
                  span_ns=10_000_000, step_ns=100_000):
    """Minimize the rotated vector residual near the |w| estimate."""
    return min((vector_residual(camera, lidar_times, lidar_samples, rotation, d), d)
               for d in range(offset_ns - span_ns, offset_ns + span_ns + 1, step_ns))[1]


def calibrate(camera_times, camera_samples, lidar_times, lidar_samples, guess_ns,
              rotation=None) -> dict:
    """Clock offset and (unless given) rotation from two gyro streams.

    |w| correlation finds the offset without knowing the rotation; the rotated
    vector residual then refines it, and the rotation is re-solved at that offset.
    """
    camera = Signal(camera_times, camera_samples)
    coarse, score = find_offset(camera, lidar_times, lidar_samples, guess_ns)
    report = {"magnitude_correlation": score, "magnitude_offset_ns": coarse}
    solve = rotation is None
    offset = coarse
    # A small delay of a smooth motion resembles a small rotation, so alternate
    # rotation and offset estimates until the offset settles.
    for _ in range(6):
        if solve:
            rotation, fit = solve_rotation(camera, lidar_times, lidar_samples, offset)
        refined = refine_offset(camera, lidar_times, lidar_samples, rotation, offset)
        settled = abs(refined - offset) <= 100_000
        offset = refined
        if settled or not solve:
            break
    report["offset_ns"] = offset
    if solve:
        rotation, fit = solve_rotation(camera, lidar_times, lidar_samples, offset)
        report["fit"] = fit
    report["R_lidar_gpmf"] = [list(row) for row in rotation]
    report["R_lidar_sphere"] = [list(row) for row in matmul(rotation, GPMF_TO_SPHERE)]
    report["q_lidar_sphere_wxyz"] = list(quaternion(matmul(rotation, GPMF_TO_SPHERE)))
    return report
