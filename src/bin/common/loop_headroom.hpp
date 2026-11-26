#pragma once

#include <algorithm>
#include <cstddef>

/// Tracks how much of the control period is left once the work of an iteration is done.
/** The headroom of an iteration is `period - work duration`, the work being measured from the
 * moment the pendulum state is available up to the end of the iteration. A negative headroom
 * means that the period has been exceeded.
 */
class LoopHeadroom {
public:
    explicit LoopHeadroom(const double period_secs) : period_secs_(period_secs) {
    }

    /// Registers the work duration of an iteration and returns its headroom (seconds)
    double update(const double work_secs) {
        last_ = period_secs_ - work_secs;
        min_ = count_ == 0 ? last_ : std::min(min_, last_);
        sum_ += last_;
        if (last_ < 0)
            ++overruns_;
        ++count_;
        return last_;
    }

    [[nodiscard]] double last() const { return last_; }
    [[nodiscard]] double min() const { return min_; }
    [[nodiscard]] double mean() const { return count_ ? sum_ / static_cast<double>(count_) : period_secs_; }
    [[nodiscard]] std::size_t overruns() const { return overruns_; }
    [[nodiscard]] std::size_t count() const { return count_; }

private:
    double period_secs_;
    double last_{0};
    double min_{0};
    double sum_{0};
    std::size_t count_{0};
    std::size_t overruns_{0};
};
