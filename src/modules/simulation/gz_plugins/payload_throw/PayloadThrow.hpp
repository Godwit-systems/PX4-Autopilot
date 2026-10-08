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

#pragma once

#include <mutex>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace px4
{

/// Releases a nested payload and applies one ballistic impulse toward an
/// impact point received on a Gazebo topic, in world ENU metres.
class PayloadThrow :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) override;

private:
	void OnTarget(const gz::msgs::Vector3d &_msg);
	bool FindPayload(gz::sim::EntityComponentManager &_ecm);
	gz::math::Vector3d BallisticVelocity(const gz::math::Vector3d &_from,
					     const gz::math::Vector3d &_to,
					     double _gravity) const;

	gz::transport::Node _node;
	gz::transport::Node::Publisher _detach_pub;
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _link{gz::sim::kNullEntity};

	std::string _child_model{"payload"};
	std::string _child_link{"link"};
	std::string _detach_topic{"/payload/detach"};
	std::string _target_topic{"/payload/throw_target"};

	std::mutex _mutex;
	gz::math::Vector3d _target{gz::math::Vector3d::Zero};
	bool _have_target{false};
	bool _detached{false};
	bool _thrown{false};
	int _wait_steps{0};
	double _mass{0.3};
};

} // namespace px4
