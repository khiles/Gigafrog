#!/usr/bin/env python3
"""Measured vehicle pose derived from livePose.

FrogPilot never subscribed livePose, so everything here was already on the wire at 20Hz
with no consumer.

A note on cornering acceleration, because the obvious field is the wrong one.
`accelerationDevice` is NOT what the accelerometer feels. In
`selfdrive/locationd/models/pose_kf.py` the state is the derivative of device-frame
velocity, and gravity and centripetal acceleration are modelled separately in the
measurement equation:

    state_dot[DEVICE_VELOCITY] = acceleration
    h_acc = device_from_ned * gravity + acceleration + angular_velocity x velocity + bias

So `accelerationDevice.y` is near zero in a steady corner. The cornering term is the
centripetal one, yaw rate times forward speed, which is what this computes.

Device frame is [Forward, Right, Down] (common/transformations/README.md), so a positive
result means accelerating toward the right.
"""
import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

# Road roughness: RMS of vertical acceleration about its own slow mean, so a constant
# bias or a long gradient can't register as rough road.
ROUGHNESS_MEAN_TAU = 2.0   # s, what counts as the "flat" baseline
ROUGHNESS_RMS_TAU = 1.0    # s, how quickly the reported roughness responds

MIN_SPEED = 1.0            # m/s, below this the derived values are meaningless
MAX_GRADIENT = 0.30        # +/- 30%, beyond this it's calibration error not a hill


class FrogPilotPose:
  def __init__(self):
    self.cornering_acceleration = 0.0
    self.road_gradient = 0.0
    self.road_roughness = 0.0

    self.vertical_mean = FirstOrderFilter(0.0, ROUGHNESS_MEAN_TAU, DT_MDL)
    self.vertical_rms = FirstOrderFilter(0.0, ROUGHNESS_RMS_TAU, DT_MDL)

  def update(self, live_pose, v_ego):
    if not (live_pose.posenetOK and live_pose.sensorsOK):
      self.reset()
      return

    angular_velocity = live_pose.angularVelocityDevice
    velocity = live_pose.velocityDevice
    acceleration = live_pose.accelerationDevice
    orientation = live_pose.orientationNED

    # Cornering. Yaw rate is about the Down axis, so positive is a right-hand turn, which
    # matches the device frame's right-positive y.
    if angular_velocity.valid and velocity.valid and v_ego > MIN_SPEED:
      self.cornering_acceleration = float(angular_velocity.z * velocity.x)
    else:
      self.cornering_acceleration = 0.0

    # Gradient as a percentage. NED euler convention is nose-up positive, so a climb reads
    # positive — worth confirming on a known hill before anything depends on the sign.
    if orientation.valid:
      self.road_gradient = float(np.clip(np.tan(orientation.y), -MAX_GRADIENT, MAX_GRADIENT))
    else:
      self.road_gradient = 0.0

    # Roughness. accelerationDevice IS gravity-free body-frame acceleration, so the z axis
    # genuinely reflects what the road is doing to the car.
    if acceleration.valid and v_ego > MIN_SPEED:
      self.vertical_mean.update(acceleration.z)
      deviation = acceleration.z - self.vertical_mean.x
      self.vertical_rms.update(deviation**2)
      self.road_roughness = float(np.sqrt(max(self.vertical_rms.x, 0.0)))
    else:
      self.reset_roughness()

  def reset(self):
    self.cornering_acceleration = 0.0
    self.road_gradient = 0.0
    self.reset_roughness()

  def reset_roughness(self):
    self.road_roughness = 0.0
    self.vertical_mean.x = 0.0
    self.vertical_rms.x = 0.0
