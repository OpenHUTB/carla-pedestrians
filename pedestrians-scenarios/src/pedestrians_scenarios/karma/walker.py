from .actor import Actor
from .karma_data_provider import KarmaDataProvider


class Walker(Actor):
    def __init__(self, **kwargs):
        kwargs['actor_category'] = 'pedestrian'

        if kwargs.get('random_location', False) and kwargs.get('spawn_point', None) is None:
            # random spawn points for pedestrians are not the same as for vehicles
            kwargs['spawn_point'] = KarmaDataProvider.get_pedestrian_spawn_point()
            kwargs['random_location'] = False

        super().__init__(**kwargs)
        self.is_crossing = False

    @staticmethod
    def get_model_by_age_and_gender(age, gender) -> str:
        matching_blueprints = KarmaDataProvider.get_pedestrian_blueprints_by_age_and_gender(
            age, gender)
        return KarmaDataProvider.get_rng().choice(matching_blueprints)
