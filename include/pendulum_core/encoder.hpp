/** @file encoder.hpp
 * @brief Header file for the Encoder class.
 */
#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <atomic>

namespace pendulum_core {
    /// Base class interfacing a simple two-phases rotatory encoder.
    /** It takes care of the step counting only and exposes the decoding mode (x1, x2, x4) as a provision
     * for subsequent configuration of this behavior.
     */
    class Encoder {
    public:
        /// Constructor, performs the setup of the gpio.
        /** This class allows to handle a rotatory encoder, assuming that it uses two
         * phases in phase quadrature.
         * @param gpio_a the pin connected to the first phase.
         * @param gpio_b the pin connected to the second phase.
         * @param steps_per_turn physical steps count per turn
         */
        Encoder(int gpio_a, int gpio_b, int steps_per_turn);

        // Prevent the user from making copies of an Encoder.
        Encoder(const Encoder &) = delete;

        Encoder &operator=(const Encoder &) = delete;

        /// Puts the two GPIO pins into high impedance mode.
        virtual ~Encoder();

        /// Access the current step counter.
        /** @note The steps are counted according to the decoding mode. By default, **any** phase edge is counted
         * (x4 decoding), which provides the highest possible resolution. This means that the number of steps
         * per revolution is **four times** the one reported on the datasheet (see counted_steps_per_turn()).
         */
        [[nodiscard]] int raw_steps() const noexcept { return steps_; }

        /// Sets the public step count origin to the current position
        /** The raw counter is left untouched: only the offset used to compute the public values changes. */
        void set_origin_here() noexcept { origin_steps_ = steps_.load(); }

        /// Sets the public step count origin to the given raw step count.
        void set_origin(const int raw_steps) noexcept { origin_steps_ = raw_steps; }

        /// Access the current motor direction.
        [[nodiscard]]
        const int &direction() const { return direction_; }

        /// Add callbacks to be executed when reaching safety thresholds.
        /** @param lower_threshold lower safety threshold below which lower_cb should
         *   be called.
         * @param upper_threshold upper safety threshold beyond which upper_cb
         *   should be called.
         * @param lower_cb function to be executed when the position becomes smaller
         *   or equal to lower_threshold. It can be nullptr, in which case no
         *   callback is executed.
         * @param upper_cb function to be executed when the position becomes greater
         *   or equal to upper_threshold. It can be nullptr, in which case no
         *   callback is executed.
         * @warning The thresholds upper_threshold and lower_threshold are
         *   absolutely arbitrary. As an example, there is no need for the lower
         *   threshold to be smaller than the upper one. But be careful: with great
         *   power comes great responsibility!
         */
        void set_safety_callbacks(
            int lower_threshold, int upper_threshold, const std::function<void()> &lower_cb,
            const std::function<void()> &upper_cb
        );

        /// Add callbacks to be executed when reaching safety thresholds.
        /** Like
         * set_safety_callbacks(int,int,std::function<void(void)>,std::function<void(void)>)
         * but using the same callback for both the lower and the upper thresholds.
         * @param lower_threshold lower safety threshold below which cb should be
         *   called.
         * @param upper_threshold upper safety threshold beyond which cb should be
         *   called.
         * @param cb function to be executed when the position becomes either smaller
         *   (or equal) to lower_threshold or  greater (or equal) to upper_threshold.
         *   It can be nullptr, in which case no callback is executed.
         */
        void set_safety_callbacks(const int lower_threshold,
                                const int upper_threshold,
                                const std::function<void()> &cb) {
            set_safety_callbacks(lower_threshold, upper_threshold, cb, cb);
        }

        /// Remove previously assigned callbacks.
        void remove_safety_callbacks();

        /// read access to pins
        [[nodiscard]] int pin_a() const { return pin_a_; }
        [[nodiscard]] int pin_b() const { return pin_b_; }

    private:
        /// "Table" that allows to tell the direction of rotation.
        /** The encoder can be seen as a simple state machine in which the state
         * is given by the pair of voltage levels `(a,b)`. For each state,
         * there are exactly two possible transitions towards other states.
         * It is thus possible to define a simple lookup-table in which one stores
         * the "departing" (or past) state as well as the "arrival" (or current)
         * one and associates this pair to a direction. Such table is shown here:
         *
         *  a' | b' | a | b | dir
         * ----|----|---|---|-----
         *  0  | 0  | 0 | 0 |  0
         *  0  | 0  | 0 | 1 | +1
         *  0  | 0  | 1 | 0 | -1
         *  0  | 0  | 1 | 1 |  0
         *  0  | 1  | 0 | 0 | -1
         *  0  | 1  | 0 | 1 |  0
         *  0  | 1  | 1 | 0 |  0
         *  0  | 1  | 1 | 1 | +1
         *  1  | 0  | 0 | 0 | +1
         *  1  | 0  | 0 | 1 |  0
         *  1  | 0  | 1 | 0 |  0
         *  1  | 0  | 1 | 1 | -1
         *  1  | 1  | 0 | 0 |  0
         *  1  | 1  | 0 | 1 | -1
         *  1  | 1  | 1 | 0 | +1
         *  1  | 1  | 1 | 1 |  0
         *
         * The values `a'` and `b'` represent past states, while `a` and `b` are the
         * current ones. As an example, if at any moment the encoder changes from
         * `(0,1)` to `(1,1)`, then it means that it rotated of one step in the
         * positive direction. Impossible transitions (such as `(0,0)->(1,1)`) are
         * given a null value so that they do not produce any effect.
         *
         * The lookup table above is stored in this variable, in the form of an
         * array. To read it, it is sufficient to pass as index the number obtained
         * by reading the quadruple of pin values as a binary number. As an example,
         * consider the case in which the encoder transitions from `(0,1)` to
         * `(0,0)`. The four digits altogether form the binary number `0100`,
         * corresponding to 4. The direction corresponding to this transition is
         * thus available at `ENCODER_TABLE[4]`.
         */
        static constexpr std::array<int, 16> ENCODER_TABLE = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

        /// Decoding mode used by default: all the phase edges are counted (x4).
        static constexpr int DEFAULT_COUNTING_MULTIPLIER = 4;

        /// Convert pin levels into the correct index to be passed to ENCODER_TABLE.
        /** @param a current level of the first phase.
         * @param b current level of the second phase.
         * @param past_a past level of the first phase.
         * @param past_b past level of the second phase.
         * @return an index to be used inside ENCODER_TABLE which tells in which
         *   direction the encoder is rotating.
         */
        static int encode(bool a, bool b, bool past_a, bool past_b);

        int pin_a_; ///< Pin the first wire of the encoder is connected to.
        int pin_b_; ///< Pin the second wire of the encoder is connected to.

        std::atomic<int> origin_steps_{0}; ///< Raw step count taken as the origin of the public values.
        bool a_current_; ///< Current voltage level on pin_a_.
        bool b_current_; ///< Current voltage level on pin_b_.
        bool a_past_; ///< Past voltage level on pin_a_.
        bool b_past_; ///< Past voltage level on pin_b_.
        std::atomic<int> steps_; ///< Current number of encoder steps (raw count, written only by the ISR)
        int direction_{}; ///< Current rotation direction.
        int lower_threshold_; ///< Lower threshold below which lower_cb_ should be executed.
        int upper_threshold_; ///< Upper threshold beyond which upper_cb_ should be executed.
        std::function<void()> lower_cb_;
        ///< Callback to be executed whenever the position becomes less than lower_threshold_.
        std::function<void()> upper_cb_;
        ///< Callback to be executed whenever the position becomes more than upper_threshold_.
        const int steps_per_turn_; ///< Number of steps per revolution reported on the datasheet.
        const int counting_multiplier_; ///< Number of phase edges counted per datasheet step (x1, x2 or x4 decoding).

        /// Function called whenever one of the pins changes level.
        /** @param gpio the pin that just changed its level.
         * @param level the current pin level.
         * @param tick time elapsed since boot (in microseconds). Note that it
         *   overflows every ~72 minutes (technically, every 2^32 microseconds).
         *   Remember that, due to integer arithmetics, it is still safe to get
         *   elapsed times doing `tick_now-tick_previous` even if `tick_now`
         *   overflowed.
         * @todo Perhaps remove the internal pin check that throws an exception?
         */
        void pulse(int gpio, int level, unsigned int tick);

        /// Static wrapper to call pulse.
        /** This method is required since the GPIO library allows to execute a
         * callback when pin levels change, but such a callback must be provided
         * as a function pointer.
         * @param gpio the pin that just changed its level.
         * @param level the current pin level.
         * @param tick time elapsed since boot (in microseconds). Note that it
         *   overflows every ~72 minutes (technically, every 2^32 microseconds).
         *   Remember that, due to integer arithmetics, it is still safe to get
         *   elapsed times doing `tick_now-tick_previous` even if `tick_now`
         *   overflowed.
         * @param THIS a pointer to an Encoder. Named `THIS` simply because it
         *   tries to mimic the `this` pointer.
         */
        static void pulse_static(int gpio, int level, unsigned int tick, void *THIS);

    protected:
        /// Number of steps counted per revolution: datasheet resolution times decoding multiplier.
        [[nodiscard]] int counted_steps_per_turn() const noexcept { return steps_per_turn_ * counting_multiplier_; }

        /// Raw step count taken as the origin.
        [[nodiscard]] int origin_steps() const noexcept { return origin_steps_; }

        /// Number of revolutions made since the origin.
        [[nodiscard]] double turns() const noexcept { return turns_between_origin_and(steps_); }

        /// Number of revolutions between the origin and the given raw step count.
        [[nodiscard]] double turns_between_origin_and(const int raw_steps) const noexcept {
            return static_cast<double>(raw_steps - origin_steps_) / counted_steps_per_turn();
        }
    };

    /// A specialized encoder that returns the angular position of the pendulum.
    class AngleEncoder : public Encoder {
    public:
        AngleEncoder(const int gpio_a, const int gpio_b, const int steps_per_turn)
            : Encoder(gpio_a, gpio_b, steps_per_turn) {
        }

        /// Returns the absolute angle in radians
        [[nodiscard]] double angle() const noexcept { return 2 * M_PI * turns(); }
    };

    /// A specialized encoder that returns the linear position of the pendulum cart.
    class PositionEncoder : public Encoder {
    public:
        PositionEncoder(const int gpio_a, const int gpio_b, const int steps_per_turn,
                        const double belt_module, const int pulley_teeth_count)
            : Encoder(gpio_a, gpio_b, steps_per_turn),
              belt_module_(belt_module),
              pulley_teeth_count_(pulley_teeth_count) {
        }

        /// Returns the current position in meters, relative to the origin.
        [[nodiscard]] double position() const noexcept { return steps_to_position(raw_steps()); }

        /// Returns the position in meters, relative to the origin, corresponding to a raw step count.
        [[nodiscard]] double steps_to_position(const int raw_steps) const noexcept {
            return turns_between_origin_and(raw_steps) * meters_per_turn();
        }

        /// Returns the length in meters covered by a number of steps (no origin involved).
        [[nodiscard]] double steps_to_distance(const int step_count) const noexcept {
            return static_cast<double>(step_count) / counted_steps_per_turn() * meters_per_turn();
        }

        /// Returns the raw step count (rounded to the nearest one) corresponding to a position in meters.
        [[nodiscard]] int position_to_steps(const double position) const noexcept {
            return static_cast<int>(std::lround(position_to_raw_steps(position)));
        }

        /// Sets the safety thresholds, given as positions in meters relative to the origin.
        /** The thresholds are converted to raw steps rounded towards the inside of the allowed range, so that
         * the requested margin is never reduced.
         * @param min_position lower position at or below which lower_cb is called.
         * @param max_position upper position at or beyond which upper_cb is called.
         * @param lower_cb callback for the lower limit (can be nullptr).
         * @param upper_cb callback for the upper limit (can be nullptr).
         * @note The origin must be set before calling this method.
         */
        void set_position_limits(const double min_position, const double max_position,
                                 const std::function<void()> &lower_cb, const std::function<void()> &upper_cb) {
            set_safety_callbacks(
                static_cast<int>(std::ceil(position_to_raw_steps(min_position))),
                static_cast<int>(std::floor(position_to_raw_steps(max_position))),
                lower_cb, upper_cb
            );
        }

    private:
        /// Length in meters of one revolution of the pulley.
        [[nodiscard]] double meters_per_turn() const noexcept { return belt_module_ * pulley_teeth_count_ / 1000.; }

        /// Unrounded raw step count corresponding to a position in meters.
        [[nodiscard]] double position_to_raw_steps(const double position) const noexcept {
            return origin_steps() + position / meters_per_turn() * counted_steps_per_turn();
        }

        const double belt_module_;
        const int pulley_teeth_count_;
    };
} // namespace pendulum_core
