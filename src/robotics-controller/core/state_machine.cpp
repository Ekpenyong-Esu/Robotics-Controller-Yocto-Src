#include "state_machine.hpp"
#include <iostream>

namespace robotics {

struct StateMachine::Impl {
    State current_state = State::INIT;
    State previous_state = State::INIT;

    bool is_valid_transition(State from, State to) {
        // Define valid state transitions
        switch (from) {
            case State::INIT:
                return (to == State::IDLE || to == State::ERROR);
            case State::IDLE:
                return (to == State::MANUAL_CONTROL || to == State::AUTONOMOUS ||
                       to == State::ERROR || to == State::SHUTDOWN);
            case State::MANUAL_CONTROL:
                return (to == State::IDLE || to == State::ERROR || to == State::SHUTDOWN);
            case State::AUTONOMOUS:
                return (to == State::IDLE || to == State::ERROR || to == State::SHUTDOWN);
            case State::ERROR:
                return (to == State::IDLE || to == State::SHUTDOWN);
            case State::SHUTDOWN:
                return false; // No transitions from shutdown
        }
        return false;
    }
};

StateMachine::StateMachine() : pimpl_(std::make_unique<Impl>()) {}

StateMachine::~StateMachine() = default;

bool StateMachine::initialize() {
    std::cout << "Initializing State Machine..." << std::endl;

    // Transition from INIT to IDLE
    if (request_state_transition(State::IDLE)) {
        std::cout << "State Machine initialized successfully" << std::endl;
        return true;
    }

    std::cerr << "Failed to initialize State Machine" << std::endl;
    return false;
}

void StateMachine::update() {
    // State machine update logic
    // This could include checking for automatic state transitions
    // based on system conditions, timeouts, etc.
}

StateMachine::State StateMachine::get_current_state() const {
    return pimpl_->current_state;
}

bool StateMachine::request_state_transition(State new_state) {
    if (pimpl_->is_valid_transition(pimpl_->current_state, new_state)) {
        std::cout << "State transition: " << state_to_string(pimpl_->current_state)
                  << " -> " << state_to_string(new_state) << std::endl;

        pimpl_->previous_state = pimpl_->current_state;
        pimpl_->current_state = new_state;
        return true;
    }

    std::cerr << "Invalid state transition: " << state_to_string(pimpl_->current_state)
              << " -> " << state_to_string(new_state) << std::endl;
    return false;
}

std::string StateMachine::state_to_string(State state) {
    switch (state) {
        case State::INIT: return "INIT";
        case State::IDLE: return "IDLE";
        case State::MANUAL_CONTROL: return "MANUAL_CONTROL";
        case State::AUTONOMOUS: return "AUTONOMOUS";
        case State::ERROR: return "ERROR";
        case State::SHUTDOWN: return "SHUTDOWN";
    }
    return "UNKNOWN";
}

std::string StateMachine::get_current_state_string() const {
    return state_to_string(pimpl_->current_state);
}

bool StateMachine::is_in_error() const {
    return pimpl_->current_state == State::ERROR;
}

void StateMachine::clear_error() {
    if (pimpl_->current_state == State::ERROR) {
        request_state_transition(State::IDLE);
    }
}

} // namespace robotics
