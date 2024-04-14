import config
from entities.base import Entity
from simulation.metrics import Metrics
from utilities.types import Point


# Created in feel_event, not a big deal
class Event(Entity):
    """An event is any kind of event that the drone detects on the aoi. It is an Entity."""

    def __init__(self, coords: Point, current_time: int, deadline: int | None = None):
        super().__init__(id(self), coords)
        self.current_time = current_time

        # One can specify the deadline or just consider as deadline now + EVENTS_DURATION
        # The deadline of an event represents the estimate of the drone that the event will be no more
        # interesting to monitor.
        self.deadline = (
            current_time + config.event_duration if deadline is None else deadline
        )

        # add metrics: all the events generated during the simulation
        # GENERATED_EVENTS
        if not coords == (-1, -1) and not current_time == -1:
            Metrics.instance().events.add(self)

    def to_json(self):
        """return the json repr of the obj"""
        return {
            "coord": self.coords,
            "i_gen": self.current_time,
            "i_dead": self.deadline,
            "id": self.identifier,
        }

    def is_expired(self, cur_step: int):
        """return true if the deadline expired"""
        return cur_step > self.deadline

    def __repr__(self):
        return "Ev id:" + str(self.identifier) + " c:" + str(self.coords)
