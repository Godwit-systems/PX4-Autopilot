/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "PayloadThrow.hpp"

#include <chrono>
#include <cmath>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>

using namespace px4;

namespace
{
constexpr double kMaxHorizontalSpeed = 15.0; // [m/s]
constexpr double kMinFlightTime = 0.2;       // [s]
constexpr int kDetachWaitSteps = 2;
}

void PayloadThrow::Configure(const gz::sim::Entity &_entity,
			     const std::shared_ptr<const sdf::Element> &_sdf,
			     gz::sim::EntityComponentManager &/*_ecm*/,
			     gz::sim::EventManager &/*_eventMgr*/)
{
	_model = gz::sim::Model(_entity);

	if (_sdf->HasElement("child_model")) {
		_child_model = _sdf->Get<std::string>("child_model");
	}

	if (_sdf->HasElement("child_link")) {
		_child_link = _sdf->Get<std::string>("child_link");
	}

	if (_sdf->HasElement("detach_topic")) {
		_detach_topic = _sdf->Get<std::string>("detach_topic");
	}

	if (_sdf->HasElement("target_topic")) {
		_target_topic = _sdf->Get<std::string>("target_topic");
	}

	if (!_node.Subscribe(_target_topic, &PayloadThrow::OnTarget, this)) {
		gzerr << "PayloadThrow failed to subscribe to " << _target_topic << std::endl;
	}

	_detach_pub = _node.Advertise<gz::msgs::Empty>(_detach_topic);
	gzmsg << "PayloadThrow listening on " << _target_topic
	      << ", releasing via " << _detach_topic << std::endl;
}

void PayloadThrow::OnTarget(const gz::msgs::Vector3d &_msg)
{
	std::lock_guard<std::mutex> lock(_mutex);

	if (_thrown) {
		return;
	}

	_target = gz::math::Vector3d(_msg.x(), _msg.y(), _msg.z());
	_have_target = true;
}

bool PayloadThrow::FindPayload(gz::sim::EntityComponentManager &_ecm)
{
	if (_link != gz::sim::kNullEntity) {
		return true;
	}

	const gz::sim::Entity payload_model = _model.ModelByName(_ecm, _child_model);

	if (payload_model == gz::sim::kNullEntity) {
		return false;
	}

	_link = gz::sim::Model(payload_model).LinkByName(_ecm, _child_link);

	if (_link == gz::sim::kNullEntity) {
		return false;
	}

	const auto *inertial = _ecm.Component<gz::sim::components::Inertial>(_link);

	if (inertial != nullptr) {
		const double mass = inertial->Data().MassMatrix().Mass();

		if (mass > 1.0e-6) {
			_mass = mass;
		}
	}

	gz::sim::Link(_link).EnableVelocityChecks(_ecm, true);
	return true;
}

gz::math::Vector3d PayloadThrow::BallisticVelocity(const gz::math::Vector3d &_from,
		const gz::math::Vector3d &_to,
		double _gravity) const
{
	const gz::math::Vector3d delta = _to - _from;
	const double range = std::hypot(delta.X(), delta.Y());
	const double dz = delta.Z();
	double flight_time = kMinFlightTime;

	if (dz < 0.0 && _gravity > 0.1) {
		flight_time = std::sqrt(-2.0 * dz / _gravity);
	}

	if (range > 0.1) {
		flight_time = std::max(flight_time, range / kMaxHorizontalSpeed);
	}

	flight_time = std::max(flight_time, kMinFlightTime);

	// p(t) = p0 + v t + 0.5 a t^2, a = (0, 0, -g)
	const gz::math::Vector3d accel(0.0, 0.0, -_gravity);
	return delta / flight_time - 0.5 * accel * flight_time;
}

void PayloadThrow::PreUpdate(const gz::sim::UpdateInfo &_info,
			     gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused || _thrown) {
		return;
	}

	if (!FindPayload(_ecm)) {
		return;
	}

	gz::math::Vector3d target;
	bool have_target = false;
	{
		std::lock_guard<std::mutex> lock(_mutex);
		have_target = _have_target;
		target = _target;
	}

	if (!have_target) {
		return;
	}

	if (!_detached) {
		gz::msgs::Empty empty;
		_detach_pub.Publish(empty);
		_detached = true;
		_wait_steps = kDetachWaitSteps;
		return;
	}

	if (_wait_steps > 0) {
		--_wait_steps;
		return;
	}

	const double dt = std::chrono::duration<double>(_info.dt).count();

	if (dt < 1.0e-6) {
		return;
	}

	gz::sim::Link link(_link);
	const auto pose = gz::sim::worldPose(_link, _ecm);
	const auto velocity = link.WorldLinearVelocity(_ecm);
	const gz::math::Vector3d current = velocity.value_or(gz::math::Vector3d::Zero);

	const gz::sim::Entity world = gz::sim::worldEntity(_model.Entity(), _ecm);
	const auto *gravity = _ecm.Component<gz::sim::components::Gravity>(world);
	double g = 9.8;

	if (gravity != nullptr) {
		g = std::abs(gravity->Data().Z());
	}

	if (g < 0.1) {
		g = 9.8;
	}

	const gz::math::Vector3d desired = BallisticVelocity(pose.Pos(), target, g);
	const gz::math::Vector3d delta_v = desired - current;
	link.AddWorldWrench(_ecm, delta_v * (_mass / dt), gz::math::Vector3d::Zero);
	_thrown = true;

	gzmsg << "PayloadThrow release velocity "
	      << desired.X() << " " << desired.Y() << " " << desired.Z()
	      << " m/s toward " << target.X() << " " << target.Y() << " " << target.Z()
	      << std::endl;
}

GZ_ADD_PLUGIN(
	PayloadThrow,
	gz::sim::System,
	PayloadThrow::ISystemConfigure,
	PayloadThrow::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(PayloadThrow, "px4::PayloadThrow")
