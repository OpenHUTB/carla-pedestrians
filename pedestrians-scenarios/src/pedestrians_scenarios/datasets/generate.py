try:
    import pedestrians_scenarios.karma as km
    from .generators.binary_single_pedestrian import BinarySinglePedestrian

    def add_cli_args(parser):
        parser = km.karma.Karma.add_cli_args(parser)

        # TODO: this can depend on the requested type of dataset in the future
        parser = BinarySinglePedestrian.add_cli_args(parser)

        return parser

    def command(**kwargs):
        """Command line interface for generating datasets.

        :param kwargs: parsed command line arguments
        :type kwargs: Dict
        """

        # TODO: this can depend on the requested type of dataset in the future
        generator = BinarySinglePedestrian(**kwargs)

        generator.generate()

except ModuleNotFoundError as e:
    def add_cli_args(parser):
        return parser

    def command(**kwargs):
        raise NotImplementedError(
            "This command is only available when `carla` package is installed.") from e
