import carla
from typing import Generic, TypeVar, Any

from .karma_data_provider import KarmaDataProvider

A = TypeVar('A', carla.Actor, carla.Walker, carla.Vehicle)


class Actor(Generic[A]):
    """
    Proxy for carla.Actor objects that goes through the
    KarmaDataProvider to access common properities.
    """

    def __init__(self, actor: A = None, **kwargs) -> None:
        if actor is None:
            actor = KarmaDataProvider.request_new_actor(**kwargs)
        if actor is None:
            raise RuntimeError("Actor could not be created")
        self._actor = actor

    def get_transform(self) -> carla.Transform:
        return KarmaDataProvider.get_transform(self._actor)

    def get_location(self) -> carla.Location:
        return KarmaDataProvider.get_location(self._actor)

    def get_velocity(self) -> float:
        return KarmaDataProvider.get_velocity(self._actor)

    @property
    def id(self) -> int:
        return self._actor.id

    def __getattr__(self, __name: str) -> Any:
        return getattr(self._actor, __name)
