#include "pendule_pi/cli_parser.hpp"
#include "CLI11/CLI11.hpp" // Compiled only when cli_parser.cpp changes

#include <memory>
#include <sstream>

namespace {
    // First name of a CLI11 names specification: "--duration,-d" -> "--duration"
    std::string primary_name(const std::string &names) {
        return names.substr(0, names.find(','));
    }

    void add_extra_option(CLI::App &app, const ExtraOption &extra) {
        CLI::Option *option = nullptr;
        switch (extra.kind) {
            case ExtraOption::Kind::Flag:
                option = app.add_flag(extra.names, *static_cast<bool *>(extra.target), extra.description);
                break;
            case ExtraOption::Kind::String:
                option = app.add_option(extra.names, *static_cast<std::string *>(extra.target), extra.description);
                break;
            case ExtraOption::Kind::Int:
                option = app.add_option(extra.names, *static_cast<int *>(extra.target), extra.description);
                break;
            case ExtraOption::Kind::Double:
                option = app.add_option(extra.names, *static_cast<double *>(extra.target), extra.description);
                break;
        }
        if (extra.required) {
            option->required();
        }
    }

    // CLI11 returns app.usage() as is, without the "Usage:" label, and cannot express alternative invocations
    // (mutually exclusive options). This formatter builds the whole usage block from a list of invocations.
    class UsageFormatter final : public CLI::Formatter {
    public:
        explicit UsageFormatter(std::vector<std::string> invocations) : invocations_(std::move(invocations)) {
        }

        std::string make_usage(const CLI::App *app, std::string) const override {
            const std::string label = get_label("Usage") + ": ";
            const std::string indent(label.size(), ' ');

            std::stringstream out;
            for (size_t ndx = 0; ndx < invocations_.size(); ndx++) {
                out << (ndx == 0 ? label : indent) << app->get_name() << ' ' << invocations_[ndx] << '\n';
            }
            out << '\n';
            return out.str();
        }

    private:
        std::vector<std::string> invocations_;
    };

    // Invocations are built from the enabled options
    std::vector<std::string> build_invocations(const CliSpec &spec, const bool with_example) {
        std::string main_invocation = "--config-file/-f PATH";
        if (has_feature(spec.features, CliFeature::Check)) {
            main_invocation += " [--check]";
        }
        if (has_feature(spec.features, CliFeature::ListGpios)) {
            main_invocation += " [--list-gpios]";
        }
        for (const auto &extra: spec.extra) {
            std::string usage = primary_name(extra.names);
            if (extra.kind != ExtraOption::Kind::Flag) {
                usage += " VALUE";
            }
            main_invocation += extra.required ? " " + usage : " [" + usage + "]";
        }

        std::vector<std::string> invocations = {main_invocation};
        if (with_example) {
            invocations.emplace_back("--example");
        }
        invocations.emplace_back("--version");
        return invocations;
    }
}

std::optional<ApplicationCLIOptions> parse_cli(
    const CliSpec &spec,
    const int argc, char **argv,
    int &exit_code
) {
    CLI::App app{spec.description, program_invocation_short_name};
    app.set_version_flag("--version", PENDULE_PI_VERSION);

    ApplicationCLIOptions cli_options{};

    app.add_option(
                "--config-file,-f",
                cli_options.config_path,
                "YAML configuration file"
            )
            ->required()
            ->check(CLI::ExistingFile);

    if (has_feature(spec.features, CliFeature::Check)) {
        app.add_flag(
            "--check",
            cli_options.check,
            "Check configuration file and exit"
        );
    }

    if (has_feature(spec.features, CliFeature::ListGpios)) {
        app.add_flag(
            "--list-gpios,-L",
            cli_options.list_gpios,
            "Show the list of the GPIOs used by the process"
        );
    }

    const bool with_example = has_feature(spec.features, CliFeature::Example) && spec.print_example != nullptr;
    if (with_example) {
        const auto print_example = spec.print_example;
        app.add_flag_callback(
            "--example",
            [print_example] {
                print_example();
                throw CLI::Success();
            },
            "Show an example configuration file and exit\n"
            "--config-file option is ignored if this option is used"
        )->trigger_on_parse();
    }

    for (const auto &extra: spec.extra) {
        add_extra_option(app, extra);
    }

    app.formatter(std::make_shared<UsageFormatter>(build_invocations(spec, with_example)));

    // Replicates CLI11_PARSE(app, argc, argv) without exiting inside the helper
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        exit_code = app.exit(e);
        return std::nullopt;
    }

    exit_code = 0;
    return cli_options;
}
