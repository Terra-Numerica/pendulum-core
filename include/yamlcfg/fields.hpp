#pragma once

#include <cstddef>
#include <ostream>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

// Declare the fields of a struct once, get printing for free.
//
//   struct Point { int x; int y; };
//
//   constexpr auto fields(const Point*) {            // found by argument-dependent lookup
//       return std::make_tuple(YAMLCFG_FIELD(Point, x), YAMLCFG_FIELD(Point, y));
//   }
//
//   yamlcfg::to_string(Point{1, 2});                 // "{ x: 1, y: 2 }"
//   yamlcfg::to_string(Point{1, 2}, 2);              // one field per line, 2 spaces per level
//
// Nested structs are printed recursively as long as they have their own fields().

namespace yamlcfg {
    // A field = its name + a pointer to the member
    template<typename Class, typename T>
    struct Field {
        const char *name;
        T Class::*member;
    };

    template<typename Class, typename T>
    constexpr Field<Class, T> field(const char *name, T Class::*member) {
        return {name, member};
    }

    // YAMLCFG_FIELD(Switch, pin)  ->  yamlcfg::field("pin", &Switch::pin)
#define YAMLCFG_FIELD(Type, member) ::yamlcfg::field(#member, &Type::member)

    namespace detail {
        // "Does fields(const T*) exist?"
        template<typename T, typename = void>
        struct has_fields : std::false_type {
        };

        template<typename T>
        struct has_fields<T, std::void_t<decltype(fields(std::declval<const T *>()))> > : std::true_type {
        };

        // Layout: indent == 0 -> compact, single line; indent > 0 -> one field per line,
        // `indent` spaces per nesting level. `level` is the current nesting level.
        struct Layout {
            int indent;
            int level;

            [[nodiscard]] std::string spaces(int lvl) const {
                return std::string(static_cast<std::size_t>(indent * lvl), ' ');
            }

            [[nodiscard]] Layout deeper() const { return {indent, level + 1}; }
        };

        template<typename T>
        void write_object(std::ostream &os, const T &obj, Layout layout);

        template<typename T>
        void write_value(std::ostream &os, const T &value, Layout layout) {
            if constexpr (has_fields<T>::value) {
                write_object(os, value, layout);
            } else if constexpr (std::is_same_v<T, std::string>) {
                os << '"' << value << '"';
            } else {
                os << value;
            }
        }

        template<typename T>
        void write_object(std::ostream &os, const T &obj, const Layout layout) {
            const bool pretty = layout.indent > 0;
            const Layout inner = layout.deeper();

            if (!pretty) os << '{';
            bool first = true;
            // Writes one field: separator, name, value
            const auto write_field = [&](const auto &f) {
                using FieldType = std::decay_t<decltype(obj.*(f.member))>;
                constexpr bool is_object = has_fields<FieldType>::value;
                const auto &value = obj.*(f.member);

                if (pretty) {
                    os << layout.spaces(layout.level) << "- " << f.name << ':';
                    if constexpr (is_object) {
                        os << '\n'; // nested fields follow, one per line
                        write_value(os, value, inner);
                    } else {
                        os << ' ';
                        write_value(os, value, inner);
                        os << '\n';
                    }
                } else {
                    os << (first ? " " : ", ") << f.name << ": ";
                    write_value(os, value, inner);
                }
                first = false;
            };
            // The fold expression calls write_field for each element of the tuple
            std::apply([&](const auto &... f) { (write_field(f), ...); },
                       fields(static_cast<const T *>(nullptr))
            );
            if (!pretty) os << " }";
        }
    } // namespace detail

    // indent == 0: compact, single line. indent > 0: one field per line, `indent` spaces per level.
    template<typename T>
    std::string to_string(const T &obj, const int indent = 0) {
        static_assert(detail::has_fields<T>::value,
                      "to_string: no fields(const T*) declared for this type");
        std::ostringstream ss;
        ss << std::boolalpha;
        detail::write_object(ss, obj, detail::Layout{indent, 0});
        return ss.str();
    }
} // namespace yamlcfg
