from collections import namedtuple
from enum import Enum
from .standard_distribution import StandardDistribution

PedestrianProfile = namedtuple(
    'PedestrianProfile', [
        'age', 'gender',
        'walking_speed', 'crossing_speed'
    ])


# Create some default profiles; those are up for revision
# somewhat based on what's found in doi:10.1016/j.sbspro.2013.11.160


class ExamplePedestrianProfiles(Enum):
    adult_female = PedestrianProfile('adult', 'female', StandardDistribution(
        1.19, 0.19), StandardDistribution(1.45, 0.23))
    adult_male = PedestrianProfile('adult', 'male', StandardDistribution(
        1.27, 0.21), StandardDistribution(1.47, 0.24))
    child_female = PedestrianProfile('child', 'female', StandardDistribution(
        0.9, 0.19), StandardDistribution(0.9, 0.23))
    child_male = PedestrianProfile('child', 'male', StandardDistribution(
        0.9, 0.21), StandardDistribution(0.9, 0.24))
