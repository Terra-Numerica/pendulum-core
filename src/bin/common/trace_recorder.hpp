#pragma once

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

/// One line of the control loop trace.
struct TraceSample {
    double t_secs{};
    double e_theta{};
    double angular_velocity{};
    double e_pos{};
    double linear_velocity{};
    double alpha{};
    bool lqr_engaged{};
    int pwm{};
    double headroom_ms{};
};

/// Fixed-size circular buffer keeping the last control loop samples in memory.
/** The buffer is allocated once, so that recording a sample doesn't allocate, nor performs any I/O.
 * The content is written to a file only on demand, outside of the control loop.
 * A recorder created with a null capacity is disabled and recording is a no-op.
 */
class TraceRecorder {
public:
    explicit TraceRecorder(const std::size_t capacity) : buffer_(capacity) {
    }

    [[nodiscard]] bool enabled() const { return !buffer_.empty(); }

    void record(const TraceSample &sample) {
        if (buffer_.empty())
            return;
        buffer_[next_] = sample;
        next_ = (next_ + 1) % buffer_.size();
        if (count_ < buffer_.size())
            ++count_;
    }

    /// Writes the recorded samples, oldest first, as CSV. Returns false if the file can't be written.
    [[nodiscard]] bool dump(const std::string &path) const {
        std::ofstream out(path);
        if (!out)
            return false;
        out << "t_secs,e_theta,angular_velocity,e_pos,linear_velocity,alpha,lqr_engaged,pwm,headroom_ms\n"
                << std::fixed << std::setprecision(5);
        const std::size_t first = count_ < buffer_.size() ? 0 : next_;
        for (std::size_t i = 0; i < count_; ++i) {
            const auto &s = buffer_[(first + i) % buffer_.size()];
            out << s.t_secs << ',' << s.e_theta << ',' << s.angular_velocity << ',' << s.e_pos << ','
                    << s.linear_velocity << ',' << s.alpha << ',' << s.lqr_engaged << ',' << s.pwm << ','
                    << s.headroom_ms << '\n';
        }
        return static_cast<bool>(out);
    }

    [[nodiscard]] std::size_t size() const { return count_; }

private:
    std::vector<TraceSample> buffer_;
    std::size_t next_{0};
    std::size_t count_{0};
};
