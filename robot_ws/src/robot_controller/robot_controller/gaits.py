import numpy as np
from abc import ABC, abstractmethod
import time()

class GAIT:
  def __init__(self, controller):
    self.time_start = time.time()
    self.controller = controller

  @abstractmethod
  def step(self, heading: float, speed: float):
    pass