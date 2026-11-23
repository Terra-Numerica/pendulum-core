#pragma once

#include "config_error.hpp"

#include <string>
#include <type_traits>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace yamlcfg {
    // A YAML node + its path in the document ("a/b/c"). Header-only.
    // The node is stored by value (YAML::Node is a lightweight handle).
    class ConfigNode {
    public:
        explicit ConfigNode(const YAML::Node &node, std::string path = "")
            : node_(node), path_(std::move(path)) {
        }

        // Mandatory sub-section
        [[nodiscard]] ConfigNode child(const std::string &field) const {
            return ConfigNode(require(field), path_of(field));
        }

        // Mandatory field, converted to T, then checked against the constraints
        template<typename T, typename... Constraints>
        [[nodiscard]] T get_required(const std::string &field,
                                     const Constraints &... constraints) const {
            const YAML::Node node = require(field);
            return validated(field, node, convert<T>(field, node), constraints...);
        }

        // Optional field: the default value is returned when absent (and not
        // validated); otherwise same behavior as get_required
        template<typename T, typename... Constraints>
        [[nodiscard]] T get_or(const std::string &field, const T &fallback,
                               const Constraints &... constraints) const {
            const YAML::Node node = lookup(field);
            if (!node) {
                return fallback;
            }
            return validated(field, node, convert<T>(field, node), constraints...);
        }

        // Cross-field rule: reports the problem on `field`, with a custom reason
        void ensure(bool const condition, const std::string &field, const std::string &reason) const {
            if (!condition) {
                fail(field, lookup(field), reason);
            }
        }

        // All the given fields must hold different values.
        // The first duplicate found is reported on the later of the two fields.
        template<typename T>
        void ensure_distinct(std::initializer_list<std::pair<const char *, T> > fields) const {
            for (auto it = fields.begin(); it != fields.end(); ++it) {
                for (auto prev = fields.begin(); prev != it; ++prev) {
                    ensure(it->second != prev->second, it->first,
                           std::string("must be different from ") + prev->first);
                }
            }
        }

    private:
        // 1-based line of a node in the document, 0 if unknown
        static int line_of(const YAML::Node &node) {
            if (!node) {
                return 0;
            }
            const YAML::Mark mark = node.Mark();
            return mark.is_null() ? 0 : mark.line + 1;
        }

        // The value as shown to the user: '30', (empty), (a list), (a section)
        static std::string show(const YAML::Node &node) {
            if (!node || node.IsNull()) {
                return "(empty)";
            }
            if (node.IsScalar()) {
                return "'" + node.Scalar() + "'";
            }
            return node.IsSequence() ? "(a list)" : "(a section)";
        }

        // What we expect for a given C++ type, in words an end user understands
        template<typename T>
        static const char *expected_type() {
            if constexpr (std::is_same_v<T, bool>) {
                return "expected a boolean (true or false)";
            } else if constexpr (std::is_integral_v<T>) {
                return "expected an integer";
            } else if constexpr (std::is_floating_point_v<T>) {
                return "expected a number";
            } else if constexpr (std::is_same_v<T, std::string>) {
                return "expected a text value";
            } else {
                return "unexpected type of value";
            }
        }

        [[nodiscard]] std::string path_of(const std::string &field) const {
            return path_.empty() ? field : path_ + "/" + field;
        }

        [[noreturn]] void fail(const std::string &field, const YAML::Node &node,
                               const std::string &reason) const {
            throw InvalidFieldValue(path_of(field), line_of(node), show(node), reason);
        }

        // node_[field], reporting a parent that is not a mapping
        [[nodiscard]] YAML::Node lookup(const std::string &field) const {
            try {
                return node_[field];
            } catch (const YAML::BadSubscript &) {
                throw InvalidFieldValue(path_.empty() ? "<root>" : path_, line_of(node_), show(node_),
                                        "expected a section containing keys and values");
            }
        }

        [[nodiscard]] YAML::Node require(const std::string &field) const {
            YAML::Node node = lookup(field);
            if (!node) {
                throw MissingRequiredField(path_of(field), path_.empty() ? 0 : line_of(node_));
            }
            return node;
        }

        template<typename T>
        [[nodiscard]] T convert(const std::string &field, const YAML::Node &node) const {
            if constexpr (std::is_same_v<T, std::string>) {
                // yaml-cpp reads an empty value ("host:") as the text "null";
                // a null node is an empty string, a quoted "null" stays a real string
                if (node.IsNull()) {
                    return {};
                }
            }
            try {
                return node.as<T>();
            } catch (const YAML::BadConversion &) {
                fail(field, node, expected_type<T>());
            }
        }

        template<typename T, typename Constraint>
        void check(const std::string &field, const YAML::Node &node, const T &value,
                   const Constraint &constraint) const {
            if (!constraint(value)) {
                fail(field, node, constraint.describe());
            }
        }

        // Applies every constraint in order; the first failing one is reported
        template<typename T, typename... Constraints>
        [[nodiscard]] T validated(const std::string &field, const YAML::Node &node, T value,
                                  const Constraints &... constraints) const {
            (check(field, node, value, constraints), ...);
            return value;
        }

        YAML::Node node_;
        std::string path_;
    };

    // Reads a YAML file and builds a configuration with `build`, a callable taking
    // a ConfigNode and returning the configuration. Any YAML-level failure (missing
    // file, syntax error...) is converted to InvalidConfiguration.
    template<typename Build>
    auto load_file(const std::string &path, Build build) {
        try {
            return build(ConfigNode(YAML::LoadFile(path)));
        } catch (const YAML::Exception &e) {
            throw InvalidConfiguration("cannot load '" + path + "': " + e.what());
        }
    }

    // Same, from YAML text (handy for unit tests)
    template<typename Build>
    auto parse_text(const std::string &yaml_text, Build build) {
        try {
            return build(ConfigNode(YAML::Load(yaml_text)));
        } catch (const YAML::Exception &e) {
            throw InvalidConfiguration(std::string("cannot parse YAML: ") + e.what());
        }
    }
} // namespace yamlcfg
