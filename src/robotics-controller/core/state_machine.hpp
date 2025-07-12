#pragma once

#include <memory>
#include <string>

namespace robotics {

/**
 * @brief State machine for robotics controller
 *
 * Manages the overall state of the robotics system including
 * operational modes, error states, and state transitions.
 */
class StateMachine {
public:
    enum class State {
        INIT,
        IDLE,
        MANUAL_CONTROL,
        AUTONOMOUS,
        ERROR,
        SHUTDOWN
    };

    StateMachine();
    ~StateMachine();

    /**
     * @brief Initialize the state machine
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Update state machine (called in main loop)
     */
    void update();

    /**
     * @brief Get current state
     * @return Current state
     */
    State get_current_state() const;

    /**
     * @brief Request state transition
     * @param new_state Desired new state
     * @return true if transition is valid and was initiated
     */
    bool request_state_transition(State new_state);

    /**
     * @brief Get state as string
     * @param state State to convert
     * @return String representation of state
     */
    static std::string state_to_string(State state);

    /**
     * @brief Get current state as string
     * @return String representation of current state
     */
    std::string get_current_state_string() const;

    /**
     * @brief Check if state machine is in error state
     * @return true if in error state
     */
    bool is_in_error() const;

    /**
     * @brief Clear error state (if possible)
     */
    void clear_error();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
