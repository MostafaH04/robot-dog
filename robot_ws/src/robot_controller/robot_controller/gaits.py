import numpy as np
from abc import ABC, abstractmethod
import time

class GAIT:
  def __init__(self):
    self.time_start = time.time()
    self.started = False

  @abstractmethod
  def step(self, heading: float, speed: float):
    pass

  def reset(self):
    self.started = False