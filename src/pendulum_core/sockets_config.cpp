#include "pendulum_core/sockets_config.hpp"

#include "yamlcfg/config_node.hpp"
#include "yamlcfg/constraints.hpp"

using yamlcfg::ConfigNode;
using yamlcfg::not_empty;
using yamlcfg::InRange;

constexpr InRange<int> TCP_PORT{1, 65535};

SocketsConfig parse_sockets(const ConfigNode &n) {
    const SocketsConfig s = {
        .host = n.get_or<std::string>("host", "localhost", not_empty),
        .state_port = n.get_required<int>("state_port", TCP_PORT),
        .command_port = n.get_required<int>("command_port", TCP_PORT)
    };
    n.ensure(s.state_port != s.command_port, "command_port", "must be different from state_port");
    return s;
}
