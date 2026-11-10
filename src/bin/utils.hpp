#pragma once
#include <cmath>
#include <vector>

/** Simple function that returns 0 if the value is inside [-threshold, +threshold],
 * or the value itself otherwise.
 */
inline int robustZero(const int val, const int threshold) {
    return abs(val) <= abs(threshold) ? 0 : val;
}

/** Dead zone filter.
 *
 * Returns 0 of the input value is inside [-threshold, +threshold],
 * otherwise shift it towards zero by the threshold value.
 */
inline int deadZoneFilter(const int val, const int threshold) {
    // ensure threshold is positive for the applied logic to be correct
    const int abs_threshold = abs(threshold);
    if (val > abs_threshold)
        return val - abs_threshold;
    if (val < -abs_threshold)
        return val + abs_threshold;
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
