import numpy as np
from abc import ABC, abstractmethod
import time

# assume this is run at 1000 hz (every ms)
__time_step__ = 0.001

class Gait(ABC):
  def __init__(self):
    self.time_start = time.time()
    self.started = False

  @abstractmethod
  def step(self, heading: float, speed: float):
    pass

  def reset(self):
    self.started = False


class Walk(Gait):
  def __init__(self):
    pass

  def step(self, heading: float, speed: float):
    pass


class Crawl(Gait):
  def __init__(self):
    pass

  def step(self, heading: float, speed: float):
    pass


class Trot(Gait):
  def __init__(self):
    pass

  def step(self, heading: float, speed: float):
    pass

class Gallop(Gait):
  def __init__(self):
    pass

  def step(self, heading: float, speed: float):
    pass


