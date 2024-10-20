import numpy as np
from abc import ABC, abstractmethod
import time
import copy

# using
# BL (back left) = 3
# FL (front left) = 2
# BR (back right) = 0
# FR (front right) = 1 

class Gait(ABC):
  TIME_STEP = 0.001
  def __init__(self, init_config, period, stance_period, phase_offset):
    # stance period and phase offset are percentage values (0 to 1)

    self.time_start = time.time()
    self.config = init_config
    self.init_config = copy.deepcopy(init_config)

    self.period = period
    self.stance_period = stance_period
    self.swing_period = 1 - stance_period
    self.phase_offset = phase_offset
    
    self.started = False

  @abstractmethod
  def step(self, heading: float, speed: float):
    pass

  def reset(self):
    self.started = False


class BasicGait(Gait):
  def __init__(self, init_config, period, stance_period, phase_offset):
    super().__init__(init_config, period, stance_period, phase_offset)
    
    self.swing = False
    self.swing_time = self.period * self.swing_period
    self.t = self.phase_offset * self.period

  def step(self, heading: float, speed: float):
    if self.started is False:
      self.started = True

    phase = (self.t - 0)/self.period
    if phase >= 1:
      phase = 0
      self.t = 0
      self.swing = False
      self.config = copy.deepcopy(self.init_config)

    if phase >= self.stance_period:
      
      # swing state
      if self.swing is False:
        self.swing = True
        
        self.speed_x = -(self.config[0] - self.init_config[0]) / self.swing_time
        self.speed_y = -(self.config[1] - self.init_config[1]) / self.swing_time

      self.config[0] += self.speed_x * self.TIME_STEP
      self.config[1] += self.speed_y * self.TIME_STEP
      self.config[2] = 0.02 * np.sin(np.pi/self.swing_period * (phase - self.stance_period)) - 0.12

    else:
      # stance state
      leg_speed = -speed
      
      x_speed = leg_speed * np.cos(heading)
      y_speed = leg_speed * np.sin(heading)

      self.config[0] += x_speed * self.TIME_STEP
      self.config[1] += y_speed * self.TIME_STEP

      self.config[2] = self.init_config[2]

    self.t += self.TIME_STEP

    return self.config


class Walk:
  def __init__(self, cmd_config_fnc, init_config, period = 0.8):
    self.init_config = init_config
    self.cmd_config = cmd_config_fnc 
    
    self.leg_0 = BasicGait(init_config[0], period, 0.75, 0)
    self.leg_1 = BasicGait(init_config[1], period, 0.75, 0.25)
    self.leg_2 = BasicGait(init_config[2], period, 0.75, 0.5)
    self.leg_3 = BasicGait(init_config[3], period, 0.75, 0.75)

  def step(self, heading, speed):
    config = [self.leg_0.step(heading, speed), self.leg_1.step(heading, speed), self.leg_2.step(heading, speed), self.leg_3.step(heading, speed)]
    self.cmd_config(config)

class Trot:
  def __init__(self, cmd_config_fnc, init_config, period = 0.8):
    self.init_config = init_config
    self.cmd_config = cmd_config_fnc 
    
    self.leg_0 = BasicGait(init_config[0], period, 0.5, 0)
    self.leg_1 = BasicGait(init_config[1], period, 0.5, 0.5)
    self.leg_2 = BasicGait(init_config[2], period, 0.5, 0)
    self.leg_3 = BasicGait(init_config[3], period, 0.5, 0.5)

  def step(self, heading, speed):
    config = [self.leg_0.step(heading, speed), self.leg_1.step(heading, speed), self.leg_2.step(heading, speed), self.leg_3.step(heading, speed)]
    self.cmd_config(config)

