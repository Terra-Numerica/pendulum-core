#pragma once

#include <memory>
#include <zmqpp/zmqpp.hpp>

namespace pendulum_core {
    /// Server to share the state of the pendulum and receive commands.
    class PendulumCppServer {
    public:
        static auto constexpr DEFAULT_STATE_ENDPOINT = "ipc:///tmp/pendulum_state";
        static auto constexpr DEFAULT_COMMAND_ENDPOINT = "ipc:///tmp/pendulum_command";

        /// Creates the socket connections.
        PendulumCppServer();

        /// Constructor, initializes socket connections.
        /** @param state_endpoint ZeroMQ endpoint (`tcp://host:port` or `ipc:///path`) bound by the socket that is
          *   used to send the current state of the pendulum.
          * @param command_endpoint ZeroMQ endpoint bound by the socket that is used to receive commands.
          * For `ipc://` endpoints, the socket files are made readable and writable by every user, so that client
          * applications run by a standard user can connect to a server run by root.
          */
        PendulumCppServer(
            const std::string &state_endpoint,
            const std::string &command_endpoint
        );

        /// Deallocates the memory for the sockets.
        ~PendulumCppServer();

        /// Allows to access the last received command.
        [[nodiscard]] const int &command() const { return pwm_; }

        /// Tells how many times we failed to read commands.
        [[nodiscard]] const unsigned int &missed_commands() const { return missed_messages_; }

        /// Send the current state of the pendulum to clients.
        /**
          */
        void send_state(
            double time,
            double position,
            double angle,
            double linvel,
            double angvel
        ) const;

        /// Tries to read a command from the socket.
        /** @param wait_ms maximum time to wait for a command to be received. If
          *   negative (or zero) just check if something is available on the socket
          *   without polling.
          * @return true if a command was received.
          */
        bool read_command(int wait_ms);

    private:
        int pwm_{0}; ///< Last received command.
        unsigned int missed_messages_{0}; ///< Current number of times no commands were received.
        std::unique_ptr<zmqpp::context> context_; ///< ZeroMQ context used to create the sockets.
        std::unique_ptr<zmqpp::socket> state_pub_; ///< Socket to send the current state of the pendulum.
        std::unique_ptr<zmqpp::socket> command_sub_; ///< Socket to read commands sent by clients.
        std::unique_ptr<zmqpp::poller> command_poller_; ///< Poller to check if new commands are available.
    };
} // namespace pendulum_core
