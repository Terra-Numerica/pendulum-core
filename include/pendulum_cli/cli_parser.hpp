#pragma once

#include <optional>
#include <string>
#include <vector>

// NOTE: this header must NOT include CLI11. All the CLI11 dependent code lives in cli_parser.cpp,
// so that CLI11 is compiled only when this file changes.

// Optional, shared options which can be enabled by an application.
// --config-file and --version are always available.
enum class CliFeature : unsigned {
    None = 0,
    Check = 1 << 0, // --check
    Example = 1 << 1, // --example (requires CliSpec::print_example)
    ListGpios = 1 << 2, // --list-gpios,-L
};

constexpr CliFeature operator|(const CliFeature a, const CliFeature b) {
    return static_cast<CliFeature>(static_cast<unsigned>(a) | static_cast<unsigned>(b));
}

constexpr bool has_feature(const CliFeature features, const CliFeature feature) {
    return (static_cast<unsigned>(features) & static_cast<unsigned>(feature)) != 0;
}

// Option specific to an application, bound to a variable owned by the caller
struct ExtraOption {
    enum class Kind { Flag, String, Int, Double };

    Kind kind;
    std::string names; // CLI11 syntax, ex: "--duration,-d"
    std::string description;
    void *target; // bool* (Flag), std::string* (String), int* (Int) or double* (Double)
    bool required = false;
};

struct CliSpec {
    std::string description;
    CliFeature features = CliFeature::None;
    void (*print_example)() = nullptr;
    std::vector<ExtraOption> extra;
};

// Values of the shared options. The fields related to a feature are meaningful only if it is enabled.
struct ApplicationCLIOptions {
    std::string config_path;
    bool check = false;
    bool list_gpios = false;
};

// Returns populated ApplicationCLIOptions on success, or std::nullopt if the app should exit
// (e.g., after printing --help, --version, or a parse error). exit_code is then the one to return.
std::optional<ApplicationCLIOptions> parse_cli(
    const CliSpec &spec,
    int argc, char **argv,
    int &exit_code
);
