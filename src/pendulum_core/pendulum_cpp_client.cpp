#include <pendulum_core/pendulum_cpp_client.hpp>
#include <sstream>
#include <thread>

namespace pendulum_core {
    PendulumCppClient::PendulumCppClient(
        const int wait
    )
        : PendulumCppClient(
            DEFAULT_STATE_ENDPOINT,
            DEFAULT_COMMAND_ENDPOINT,
            wait
        ) {
    }


    PendulumCppClient::PendulumCppClient(
        const std::string &state_endpoint,
        const std::string &command_endpoint,
        const int wait
    ) {
        // Connect to the sockets to exchange data with the low-level interface.
        context_ = std::make_unique<zmqpp::context>();
        state_sub_ = std::make_unique<zmqpp::socket>(*context_, zmqpp::socket_type::subscribe);
        state_sub_->set(zmqpp::socket_option::conflate, 1);
        state_sub_->connect(state_endpoint);
        state_sub_->subscribe("");
        command_pub_ = std::make_unique<zmqpp::socket>(*context_, zmqpp::socket_type::publish);
        command_pub_->connect(command_endpoint);

        // Wait for the low-level interface to be up and running.
        if (wait <= 0) {
            // Wait indefinitely for the first message.
            read_state(BLOCKING);
        } else {
            // Wait for the given amount of time.
            int elapsed = 0;
            while (elapsed < wait) {
                // Wait one second.
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                // Check if it is possible to read a message.
                if (read_state(NON_BLOCKING)) {
                    // A message was received: exit!
                    break;
                }
                elapsed += 1;
                // Did we exit due to a received message? If not, throw!
                if (elapsed >= wait) {
                    throw std::runtime_error("PendulumCppClient: failed to establish a "
                        "connection with the low-level interface within the allotted time.\n"
                        "Check that hw_interface is running and that the endpoints in the 'sockets' section of "
                        "the configuration file are the same as the ones it uses.");
                }
            }
        }
    }


    PendulumCppClient::~PendulumCppClient() {
        state_sub_.reset();
        command_pub_.reset();
        context_.reset();
    }


    bool PendulumCppClient::read_state(
        const bool blocking
    ) {
        // Message to be received.
        zmqpp::message msg;
        // Try receiving a message.
        if (!state_sub_->receive(msg, !blocking)) {
            return false;
        }
        // Split the string message into parts. Each part should be a double.
        std::string msg_str;
        msg >> msg_str;
        std::istringstream iss(msg_str);
        iss >> time_ >> position_ >> angle_ >> linear_velocity_ >> angular_velocity_;
        // State read successfully.
        return true;
    }


    void PendulumCppClient::send_command(const int pwm) const {
        // Create the message to be sent.
        zmqpp::message msg;
        msg << std::to_string(pwm);
        // Send the PWM to the low-level interface.
        command_pub_->send(msg);
    }
} // namespace pendulum_core
