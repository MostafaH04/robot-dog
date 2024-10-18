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
  def __init__(self, send_cmd_fnc):
    self.time_start = time.time()
    self.send_cmd_fnc = send_cmd_fnc
    self.started = False

  @abstractmethod
  def step(self, heading: float, speed: float):
    pass

  def reset(self):
    self.started = False


class Walk(Gait):
  def __init__(self, send_cmd_fnc):
    super().__init__(send_cmd_fnc)
    self.order = [[3],[2],[0],[1]]
    self.state = 0
    init_x, init_y, init_z = (-0.03, 0.038, 0.12)
    init_config = [[init_x, -init_y, -init_z],[init_x, -init_y, -init_z],[init_x, init_y, -init_z],[init_x, init_y, -init_z]]
    self.curr_config = init_config

    self.t = 0
    pass

  def step(self, heading: float, speed: float):
    speed *= 3
    # 8cm/step * s / m (1/s) * m / 100 cm = 0.08/speed * s / step 
    half_per = 0.06 / speed
    # B = 2 pi * speed / 0.16
    B = 2 * np.pi / (2 * half_per)
    # x fwd for leg
    # x back for others
    # z up and down for leg
    leg_dist = 0.06
    idle_rate = (leg_dist/3)/half_per

    x_leg = 0.06*np.cos(B/2*self.t)*B/2
    z_leg = 0.01*np.sin(B*self.t) - 0.12

    legs = self.order[self.state]

    for i in range(4):
      if i in legs: continue
      self.curr_config[i][0]-= idle_rate * self.TIME_STEP
      
    for leg in legs:
      self.curr_config[leg][0] += x_leg * self.TIME_STEP
      self.curr_config[leg][2] = z_leg
    
    self.send_cmd_fnc(self.curr_config)

    self.t += self.TIME_STEP
    time.sleep(self.TIME_STEP)
    
    if self.t >= half_per:
      self.t = 0
      self.state = (self.state + 1) % 4
    # if self.state == 0:
    #   self.curr_config = copy.deepcopy(self.reset_conifg)
    #   self.send_cmd_fnc(self.curr_config)
    pass


class Crawl(Gait):
  def __init__(self, send_cmd_fnc):
    super.__init__(send_cmd_fnc)    
    pass

  def step(self, heading: float, speed: float):
    pass


class Trot(Gait):
  def __init__(self, send_cmd_fnc):
    super.__init__(send_cmd_fnc)
    pass

  def step(self, heading: float, speed: float):
    pass

class Gallop(Gait):
  def __init__(self, send_cmd_fnc):
    super.__init__(send_cmd_fnc)
    pass

  def step(self, heading: float, speed: float):
    pass


