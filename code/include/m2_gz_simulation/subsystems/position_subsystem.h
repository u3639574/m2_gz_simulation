#pragma once

#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_gz_simulation/control/joint_feedback.h"
#include <optional>

namespace m2::gz_simulation {

class position_subsystem : public fsm::subsystem {
public:
    position_subsystem(const std::string& name, joint_feedback& feedback)
        : subsystem(name), feedback_(feedback) {}
    virtual bool accepts(double target) const noexcept = 0;
    virtual void set_target(double target, fsm::command_instance_id_t owner) noexcept = 0;
    virtual bool at_target() const noexcept = 0;
    virtual void hold() noexcept = 0;
    virtual void reset() noexcept = 0;
    bool ready() const noexcept { return feedback_.ready(); }
    double now() const noexcept { return feedback_.now(); }
    double dt() const noexcept { return feedback_.dt(); }
    void release(fsm::command_instance_id_t owner, bool interrupted) noexcept
    {
        // The scheduler initializes the replacement before ending the old
        // command. Only the current owner may change its subsystem's target.
        if (owner_ == owner) {
            if (interrupted) { hold(); }
            owner_.reset();
        }
    }
protected:
    joint_feedback& feedback_;
    std::optional<fsm::command_instance_id_t> owner_;
};
}  // namespace m2::gz_simulation
