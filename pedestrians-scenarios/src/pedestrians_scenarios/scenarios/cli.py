def scenarios_generate_command(**kwargs):
    """
    Command line interface for generating scenarios.
    """


def add_datasets_cli_args(command_subparsers):
    """
    Prepares command line parameters for various datasets-related tasks.

    Returns:
      :subparsers:`argparse.SubParserAction`
    """

    parser = command_subparsers.add_parser("scenarios")
    subparsers = parser.add_subparsers()

    # 'generate' subcommand
    parser_generate = subparsers.add_parser("generate")
    parser_generate.set_defaults(subcommand=scenarios_generate_command)
