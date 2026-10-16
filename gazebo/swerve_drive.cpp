// Simulation actuator adapter. No ROS node or sensor processing is involved.
// Kinematic reference: dawan0111/Gazebo-Harmonic-Swerve-Robot, controller.py.
// See docs/swerve_sensors.md for the design differences and source commit.
#include "swerve_kinematics.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace m2::simulation {
class SwerveDrive final : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemReset {
    struct Module {
        gz::sim::Joint steer, drive;
        double x, y, speed{0.0};
    };
    struct Velocity { double x{0.0}, y{0.0}, yaw{0.0}; };

public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &) override {
        try {
            gz::sim::Model model(entity);
            radius_ = sdf->Get<double>("wheel_radius");
            max_speed_ = sdf->Get<double>("max_wheel_speed");
            acceleration_ = sdf->Get<double>("wheel_acceleration");
            steer_rate_ = sdf->Get<double>("steering_rate");
            steer_gain_ = sdf->Get<double>("steering_gain");
            for (double v : {radius_, max_speed_, acceleration_, steer_rate_, steer_gain_})
                if (!std::isfinite(v) || v <= 0) throw std::runtime_error("invalid positive parameter");
            auto element = sdf->FindElement("module");
            while (element) {
                Module module{gz::sim::Joint(model.JointByName(ecm, element->Get<std::string>("steering_joint"))),
                              gz::sim::Joint(model.JointByName(ecm, element->Get<std::string>("wheel_joint"))),
                              element->Get<double>("x"), element->Get<double>("y")};
                if (module.steer.Entity() == gz::sim::kNullEntity ||
                    module.drive.Entity() == gz::sim::kNullEntity ||
                    !std::isfinite(module.x) || !std::isfinite(module.y))
                    throw std::runtime_error("missing module joint or invalid location");
                module.steer.EnablePositionCheck(ecm);
                modules_.push_back(module);
                element = element->GetNextElement("module");
            }
            if (modules_.size() != 4) throw std::runtime_error("exactly four modules required");
            if (!node_.Subscribe(sdf->Get<std::string>("topic"), &SwerveDrive::OnVelocity, this))
                throw std::runtime_error("could not subscribe to chassis command");
            ready_ = true;
        } catch (const std::exception &e) {
            gzerr << "SwerveDrive configuration: " << e.what() << '\n';
        }
    }

    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override {
        if (!ready_ || info.paused) return;
        if (info.dt < std::chrono::steady_clock::duration::zero()) {
            Reset(info, ecm);
            return;
        }
        Velocity command;
        { std::lock_guard<std::mutex> lock(mutex_); command = command_; }
        std::vector<ModuleTarget> targets;
        std::vector<double> errors;
        double peak = 0.0;
        bool aligned = true;
        for (auto &module : modules_) {
            auto position = module.steer.Position(ecm);
            if (!position || position->empty()) { Stop(ecm); return; }
            auto target = module_target(command.x, command.y, command.yaw,
                                        module.x, module.y, position->front());
            const double error = target.angle - position->front();
            aligned = aligned && std::abs(error) < 0.10;
            peak = std::max(peak, std::abs(target.speed));
            targets.push_back(target);
            errors.push_back(error);
        }
        // Scale all wheel speeds together to preserve the requested motion direction.
        const double scale = peak > max_speed_ ? max_speed_/peak : 1.0;
        const double delta = acceleration_ * std::chrono::duration<double>(info.dt).count();
        const bool stopped = command.x == 0 && command.y == 0 && command.yaw == 0;
        for (std::size_t i = 0; i < modules_.size(); ++i) {
            auto &module = modules_[i];
            module.steer.SetVelocity(ecm, {std::clamp(steer_gain_*errors[i], -steer_rate_, steer_rate_)});
            // Align before driving; do not drag sideways while a module turns.
            const double goal = targets[i].speed*scale;
            module.speed = (!aligned || stopped) ? 0.0 :
                module.speed + std::clamp(goal-module.speed, -delta, delta);
            module.drive.SetVelocity(ecm, {module.speed/radius_});
        }
    }

    void Reset(const gz::sim::UpdateInfo &,
               gz::sim::EntityComponentManager &ecm) override {
        { std::lock_guard<std::mutex> lock(mutex_); command_ = {}; }
        Stop(ecm);
    }

private:
    void OnVelocity(const gz::msgs::Twist &msg) {
        Velocity next{msg.linear().x(), msg.linear().y(), msg.angular().z()};
        // Bound inputs before multiplication to prevent overflow in the kinematics.
        if (!std::isfinite(next.x) || !std::isfinite(next.y) || !std::isfinite(next.yaw)) next = {};
        next.x = std::clamp(next.x, -max_speed_, max_speed_);
        next.y = std::clamp(next.y, -max_speed_, max_speed_);
        next.yaw = std::clamp(next.yaw, -2.0, 2.0);
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = next;
    }

    void Stop(gz::sim::EntityComponentManager &ecm) {
        for (auto &module : modules_) {
            module.speed = 0.0;
            module.steer.SetVelocity(ecm, {0.0});
            module.drive.SetVelocity(ecm, {0.0});
        }
    }
    std::mutex mutex_;
    Velocity command_;
    std::vector<Module> modules_;
    double radius_{.12}, max_speed_{.6}, acceleration_{.5}, steer_rate_{3}, steer_gain_{8};
    bool ready_{false};
    gz::transport::Node node_;  // Destroy transport callbacks before their state.
};
}  // namespace m2::simulation

GZ_ADD_PLUGIN(m2::simulation::SwerveDrive, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemReset)
