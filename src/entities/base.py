from utilities.types import Point


class Entity:
    """An entity in the environment, e.g. Drone, Event, Packet. It extends SimulatedEntity."""

    identifier: int
    coords: Point

    def __init__(self, identifier: int, coords: Point):
        self.identifier = identifier  # the id of the entity
        self.coords = coords  # the coordinates of the entity on the map

    def __eq__(self, other):
        """Entity objects are identified by their id."""
        if not isinstance(other, Entity):
            return False
        else:
            return other.identifier == self.identifier

    def __hash__(self):
        return hash((self.identifier, self.coords))
