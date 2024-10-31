import argparse
import logging
import sys
import yaml

try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

from pedestrians_scenarios import __version__
from pedestrians_scenarios.datasets.cli import add_datasets_cli_args


# ---- CLI ----
# The functions defined in this section are wrappers around the main Python
# API allowing them to be called directly from the terminal as a CLI
# executable/script.


def add_cli_args():
    """Prepares command line parameters.

    :rtype: argparse.ArgumentParser
    """
    parser = argparse.ArgumentParser(description="Pedestrians scenarios")
    parser.add_argument(
        "--version",
        action="version",
        version="pedestrians-scenarios {ver}".format(ver=__version__),
    )
    parser.add_argument(
        "-v",
        "--verbose",
        dest="loglevel",
        help="set loglevel to INFO",
        action="store_const",
        const=logging.INFO,
    )
    parser.add_argument(
        "-vv",
        "--very-verbose",
        dest="loglevel",
        help="set loglevel to DEBUG",
        action="store_const",
        const=logging.DEBUG,
    )

    return parser


def add_common_subcommand_args(parser):
    """Callback to add common arguments to a subcommand parser."""

    parser.add_argument(
        "-c",
        "--config",
        help="Config file path. Settings in this file will override those passed via CLI",
        default=None,
    )

    return parser


def setup_logging(loglevel):
    """Setup basic logging

    :param loglevel: Minimum loglevel for emitting messages.
    :type loglevel: int
    """
    logformat = "[%(asctime)s] %(levelname)s:%(name)s:%(message)s"
    logging.basicConfig(
        level=loglevel, stream=sys.stdout, format=logformat, datefmt="%Y-%m-%d %H:%M:%S"
    )


def main(args):
    parser = add_cli_args()
    subparsers = parser.add_subparsers()

    # subcommands
    add_datasets_cli_args(subparsers, add_common_subcommand_args)

    args = parser.parse_args(args)
    kwargs = vars(args)
    setup_logging(args.loglevel)

    # handle config file
    if hasattr(args, 'config') and args.config:
        with open(args.config, 'r') as f:
            kwargs.update(yaml.load(f, Loader=Loader))

    # run subcommand
    args.subcommand(**kwargs)


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    # ^  This is a guard statement that will prevent the following code from
    #    being executed in the case someone imports this file instead of
    #    executing it as a script.
    #    https://docs.python.org/3/library/__main__.html

    # After installing your project with pip, users can also run your Python
    # modules as scripts via the ``-m`` flag, as defined in PEP 338::
    #
    #     python -m pedestrians_scenarios.skeleton
    #
    run()
