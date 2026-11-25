#include <pendulum_core/pendulum_cpp_server.hpp>

#include <sys/stat.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace pendulum_core {
    namespace {
        /// Makes the file of a Unix socket usable by every user, for an `ipc://` endpoint (nothing to do otherwise).
        /**
          * `hw_interface` runs as root and the file it creates follows its umask, so the client applications, run by
          * a standard user, could not connect: this requires the write permission on the file.
          */
        void make_accessible_to_clients(const std::string &endpoint) {
            constexpr std::string_view IPC = "ipc://";
            if (endpoint.compare(0, IPC.size(), IPC) != 0) {
                return;
            }
            const std::string path = endpoint.substr(IPC.size());
            if (::chmod(path.c_str(), 0666) != 0) {
                throw std::runtime_error("cannot set the permissions of " + path + ": " + std::strerror(errno));
            }
        }
    }

    PendulumCppServer::PendulumCppServer()
        : PendulumCppServer(
            DEFAULT_STATE_ENDPOINT,
            DEFAULT_COMMAND_ENDPOINT
        ) {
    }


    PendulumCppServer::PendulumCppServer(
        const std::string &state_endpoint,
        const std::string &command_endpoint
    ) {
        // Connect to the sockets to exchange data with the low-level interface.
        context_ = std::make_unique<zmqpp::context>();
        state_pub_ = std::make_unique<zmqpp::socket>(*context_, zmqpp::socket_type::publish);
        state_pub_->bind(state_endpoint);
        make_accessible_to_clients(state_endpoint);
        command_sub_ = std::make_unique<zmqpp::socket>(*context_, zmqpp::socket_type::subscribe);
        command_sub_->set(zmqpp::socket_option::conflate, 1);
        command_sub_->bind(command_endpoint);
        make_accessible_to_clients(command_endpoint);
        command_sub_->subscribe("");
        command_poller_ = std::make_unique<zmqpp::poller>();
        command_poller_->add(*command_sub_, zmqpp::poller::poll_in);
    }


    PendulumCppServer::~PendulumCppServer() {
        command_poller_.reset();
        command_sub_.reset();
        state_pub_.reset();
        context_.reset();
    }


    bool PendulumCppServer::read_command(
        const int wait_ms
    ) {
        // Poll the socket if needed.
        if (wait_ms > 0 && !command_poller_->poll(wait_ms)) {
            missed_messages_++;
            return false;
        }
        // Message to be received.
        zmqpp::message msg;
        // Try receiving a message.
        if (!command_sub_->receive(msg, true)) {
            missed_messages_++;
            return false;
        }
        // Split the string message into parts. Each part should be a double.
        std::string msg_str;
        msg >> msg_str;
        std::istringstream iss(msg_str);
        iss >> pwm_;
        missed_messages_ = 0;
        // State read successfully.
        return true;
    }


    void PendulumCppServer::send_state(
        const double time,
        const double position,
        const double angle,
        const double linvel,
        const double angvel
    ) const {
        // Create the message to be sent.
        zmqpp::message msg;
        msg << std::to_string(time) + " "
                + std::to_string(position) + " "
                + std::to_string(angle) + " "
                + std::to_string(linvel) + " "
                + std::to_string(angvel);
        // Send the state.
        state_pub_->send(msg, true);
    }
} // namespace pendulum_core
