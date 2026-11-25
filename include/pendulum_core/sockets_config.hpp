#pragma once

#include <string>

#include <yamlcfg/config_node.hpp>
#include <yamlcfg/fields.hpp>


/// ZeroMQ endpoints of the sockets used to exchange the state and the commands (`tcp://host:port` or `ipc:///path`).
struct SocketsConfig {
    std::string state_endpoint;
    std::string command_endpoint;
};

constexpr auto fields(const SocketsConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(SocketsConfig, state_endpoint),
        YAMLCFG_FIELD(SocketsConfig, command_endpoint)
    );
}

SocketsConfig parse_sockets(const yamlcfg::ConfigNode &n);
