#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace yamlcfg {
    // Everything the loading code can throw derives from InvalidConfiguration.
    // what() is a message meant for the end user.
    class InvalidConfiguration : public std::runtime_error {
    public:
        explicit InvalidConfiguration(const std::string &message) : std::runtime_error(message) {
        }
    };

    // An error located in the document. Besides the message, it exposes the field
    // path ("left_switch/pin") and the line (1-based, 0 if unknown).
    class FieldError : public InvalidConfiguration {
    public:
        [[nodiscard]] const std::string &field() const noexcept { return field_; }
        [[nodiscard]] int line() const noexcept { return line_; }

    protected:
        FieldError(std::string field, const int line, const std::string &message)
            : InvalidConfiguration(message), field_(std::move(field)), line_(line) {
        }

    private:
        std::string field_;
        int line_;
    };

    // line() is the line where the enclosing section starts (0 for the root).
    class MissingRequiredField : public FieldError {
    public:
        MissingRequiredField(const std::string &field, const int section_line)
            : FieldError(field, section_line, field + ": missing required field" + suffix(section_line)) {
        }

    private:
        static std::string suffix(const int line) {
            return line > 0 ? " (section whose first entry is on line " + std::to_string(line) + ")" : "";
        }
    };

    // shown_value is the offending value as displayed to the user (e.g. "'30'"),
    // reason says what is expected (e.g. "must be between 1 and 25").
    class InvalidFieldValue : public FieldError {
    public:
        InvalidFieldValue(const std::string &field, const int line,
                          const std::string &shown_value, const std::string &reason)
            : FieldError(field, line,
                         field + location(line) + ": invalid value " + shown_value + ": " + reason) {
        }

    private:
        static std::string location(const int line) {
            return line > 0 ? " (line " + std::to_string(line) + ")" : "";
        }
    };
} // namespace yamlcfg
