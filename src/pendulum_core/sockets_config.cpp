#include "pendulum_core/sockets_config.hpp"

#include <string>
#include <string_view>

#include "yamlcfg/config_node.hpp"

using yamlcfg::ConfigNode;

namespace {
    /// Maximum length of the path of a Unix socket (`sun_path` is 108 bytes long, including the final NUL).
    constexpr std::size_t MAX_IPC_PATH_LENGTH = 107;

    bool has_prefix(const std::string &text, const std::string_view prefix) {
        return text.compare(0, prefix.size(), prefix) == 0;
    }

    /// Checks that a string is a `tcp://host:port` or `ipc:///path` ZeroMQ endpoint.
    struct Endpoint {
        bool operator()(const std::string &v) const {
            constexpr std::string_view TCP = "tcp://";
            constexpr std::string_view IPC = "ipc://";
            if (has_prefix(v, TCP)) {
                return v.size() > TCP.size();
            }
            if (has_prefix(v, IPC)) {
                return v.size() > IPC.size() && v.size() - IPC.size() <= MAX_IPC_PATH_LENGTH;
            }
            return false;
        }

        [[nodiscard]] static std::string describe() {
            return "must be a \"tcp://host:port\" or \"ipc:///path\" endpoint (path up to "
                   + std::to_string(MAX_IPC_PATH_LENGTH) + " characters)";
        }
    };
}

SocketsConfig parse_sockets(const ConfigNode &n) {
    constexpr Endpoint endpoint;
    const SocketsConfig s = {
        .state_endpoint = n.get_required<std::string>("state_endpoint", endpoint),
        .command_endpoint = n.get_required<std::string>("command_endpoint", endpoint)
    };
    n.ensure_distinct<std::string>({
        {"state_endpoint", s.state_endpoint},
        {"command_endpoint", s.command_endpoint}
    });
    return s;
}
