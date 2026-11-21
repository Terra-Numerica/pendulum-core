#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <sstream>
#include <string>

// Generic, reusable constraints. A constraint is a callable returning true when
// the value is acceptable, plus a describe() giving the reason shown to the user.
// All of them are literal types, so they can be constexpr.

namespace yamlcfg {
    struct Positive {
        template<typename T>
        bool operator()(const T &v) const { return v > 0; }

        [[nodiscard]] static std::string describe() { return "must be strictly positive"; }
    };

    struct NonNegative {
        template<typename T>
        bool operator()(const T &v) const { return v >= 0; }

        [[nodiscard]] static std::string describe() { return "must be positive or zero"; }
    };

    struct NotZero {
        template<typename T>
        bool operator()(const T &v) const { return v != 0; }

        [[nodiscard]] static std::string describe() { return "must not be zero"; }
    };

    struct NotEmpty {
        bool operator()(const std::string &v) const { return !v.empty(); }
        [[nodiscard]] static std::string describe() { return "must not be empty"; }
    };

    template<typename T>
    struct InRange {
        T lo;
        T hi;

        template<typename U>
        bool operator()(const U &v) const { return v >= lo && v <= hi; }

        [[nodiscard]] std::string describe() const {
            std::ostringstream ss;
            ss << "must be between " << lo << " and " << hi;
            return ss.str();
        }
    };

    // A fixed list of allowed values. Build it with among(...)
    template<typename T, std::size_t N>
    struct Among {
        std::array<T, N> choices;

        template<typename U>
        bool operator()(const U &v) const {
            return std::any_of(choices.begin(), choices.end(), [&v](const T &c) { return c == v; });
        }

        [[nodiscard]] std::string describe() const {
            std::ostringstream ss;
            ss << "must be one of ";
            for (std::size_t i = 0; i < N; ++i) {
                ss << (i == 0 ? "" : ", ") << choices[i];
            }
            return ss.str();
        }
    };

    // Factory: N is deduced from the number of values.
    //   constexpr auto mode = among(1, 2, 3);
    //   constexpr auto name = among<std::string_view>("fast", "slow");
    template<typename T, typename... Rest>
    constexpr Among<T, 1 + sizeof...(Rest)> among(T first, Rest... rest) {
        return {{{first, static_cast<T>(rest)...}}};
    }

    inline constexpr Positive positive{};
    inline constexpr NonNegative non_negative{};
    inline constexpr NotZero not_zero{};
    inline constexpr NotEmpty not_empty{};
} // namespace yamlcfg
