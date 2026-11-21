#pragma once

#include <string>

#include <yamlcfg/config_node.hpp>
#include <yamlcfg/fields.hpp>


struct SocketsConfig {
    std::string host;
    int state_port;
    int command_port;
};

constexpr auto fields(const SocketsConfig *) {
    return std::make_tuple(
        YAMLCFG_FIELD(SocketsConfig, host),
        YAMLCFG_FIELD(SocketsConfig, state_port),
        YAMLCFG_FIELD(SocketsConfig, command_port)
    );
}

SocketsConfig parse_sockets(const yamlcfg::ConfigNode &n);
