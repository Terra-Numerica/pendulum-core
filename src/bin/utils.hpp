#pragma once
#include <cmath>
#include <vector>

/// Simple function that returns 0 if `abs(val) < zero` and `val` otherwise.
inline int robustZero(const int val, const int zero) {
    return -zero < val && val < zero ? 0 : val;
}

/// Piecewise linear function that is 0 in a neighborhood of 0.
/** This function is defined as:
 * \f[
 *   c(x) =
 *   \begin{cases}
 *     x + z & (x \leq -z) \\
 *     0     & (-z < x < z) \\
 *     x - z & (z \leq x)
 *   \end{cases}
 * \f]
 */
inline int clip(const int val, const int zero) {
    if (val > zero)
        return val - zero;
    if (val < -zero)
        return val + zero;
    return 0;
}

/// Simple sign function.
/** \f[
 *   sign(x) =
 *   \begin{cases}
 *     -1 & (x < 0)
 *      1 & (x \geq 0)
 *   \end{cases}
 * \f]
 */
inline double sign(const double x) { return x >= 0 ? 1.0 : -1.0; }

/// Modular shift in the unit circle
/** Values that lie outside the range \f$\mathbb{C}=\left[-\pi,\pi\right)\f$
 * will be shifted by \f$2 k \pi\f$ (\f$ k \in \mathbb{Z} \f$) so that they lie
 * within \f$\mathbb{C}\f$.
 */
inline double shiftInCircle(const double x) {
    double v = std::fmod(x, 2 * M_PI);
    if (v >= M_PI)
        v -= 2 * M_PI;
    if (v < -M_PI)
        v += 2 * M_PI;
    return v;
}

/// Return a list of evenly spaced values.
/** Returns the list of all values in the form `start + k * step` that are
 * smaller than (or equal to) `stop`.
 *
 * As an example, `range(1.2, 2.1, 0.2)` would return a vector that is
 * equivalent to `{1.2, 1.4, 1.6, 1.8, 2.0}`.
 */
inline std::vector<int> range(const int start, const int stop, const int step) {
    std::vector<int> vec;
    for (int i = start; i <= stop; i += step)
        vec.push_back(i);
    return vec;
}
