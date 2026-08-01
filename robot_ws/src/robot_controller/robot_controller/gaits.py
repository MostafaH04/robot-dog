"""Placeholder interfaces for future gait implementations."""

from abc import ABC, abstractmethod
import time


TIME_STEP_SECONDS = 0.001


class Gait(ABC):
    """Base interface for time-stepped gait generators."""

    def __init__(self):
        self.time_start = time.time()
        self.started = False

    @abstractmethod
    def step(self, heading, speed):
        """Advance the gait and return its next command."""

    def reset(self):
        """Reset gait execution state."""
        self.started = False


class Walk(Gait):
    """Reserved interface for a walk gait."""

    def step(self, heading, speed):
        """Advance the unimplemented walk gait."""
        raise NotImplementedError


class Crawl(Gait):
    """Reserved interface for a crawl gait."""

    def step(self, heading, speed):
        """Advance the unimplemented crawl gait."""
        raise NotImplementedError


class Trot(Gait):
    """Reserved interface for a trot gait."""

    def step(self, heading, speed):
        """Advance the unimplemented trot gait."""
        raise NotImplementedError


class Gallop(Gait):
    """Reserved interface for a gallop gait."""

    def step(self, heading, speed):
        """Advance the unimplemented gallop gait."""
        raise NotImplementedError
