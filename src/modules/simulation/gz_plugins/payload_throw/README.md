# Payload Throw

How the standard VTOL mono-cam payload is thrown in Gazebo SITL, and what has to change for a real aircraft.

## What Happens Now

QGroundControl sends a `TARGET_RELATIVE` MAVLink message. The X, Y, and Z fields are the impact point, in the same frame as that message. Z is down.

PX4 receives it in `handle_message_target_relative()` and publishes `payload_delivery_target` in the local north-east-down frame. That topic is ordinary flight-controller data. It does not talk to Gazebo.

`gz_bridge`, which only runs in SITL, converts that point into the Gazebo world (east-north-up) and publishes it on `/payload/throw_target`.

The `PayloadThrow` plugin on the vehicle model does the rest:

1. It publishes an empty message on `/payload/detach`.
2. The detachable-joint plugin deletes the weld, so the box is no longer attached.
3. Two simulation steps later it applies one force for a single physics step. The force changes the box's velocity to the ballistic arc from the release position to the impact point.

Horizontal speed is limited to 15 m/s. A farther point stays in the air longer instead of being thrown faster. Only the first `TARGET_RELATIVE` throws. Later messages are ignored.

A relative message with Z set to 0 aims at the vehicle's current altitude. A point on the ground needs Z set to that ground position.

Restart SITL after changing this plugin or the firmware:

```bash
make px4_sitl gz_standard_vtol_mono_cam_down
```

## Real Aircraft

`gz_bridge` and `PayloadThrow` do not run on a flight controller. `TARGET_RELATIVE` and `payload_delivery_target` still arrive. Nothing after that releases a payload until a flight module is added.

**Latch or gripper.** Wire the mechanism to a PX4 servo or gripper output. A module should subscribe to `payload_delivery_target` and command release only when the vehicle's position and velocity already match a drop onto that point. The box leaves with the aircraft's speed. PX4 already has `COMMAND_RELEASE` on the `gripper` topic, and `MAV_CMD_DO_GRIPPER`. The missing piece is the timing, not a new actuator protocol.

**Directed throw.** The airframe needs a launcher that can produce a known exit speed. The ballistic calculation that now lives in `PayloadThrow` has to move into the flight module, using the real vehicle position and velocity, and the launcher should fire only when it can actually produce the required speed. A latch cannot apply the impulse the simulator applies.

On either vehicle the release should be armed explicitly, refused on the ground, and one-shot. The first `TARGET_RELATIVE` must not open the mechanism by itself the way the simulation does.
