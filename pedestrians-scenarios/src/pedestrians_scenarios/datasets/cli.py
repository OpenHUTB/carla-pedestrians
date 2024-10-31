from . import merge, generate, clean


def add_datasets_cli_args(command_subparsers, add_common_subcommand_args):
    """Prepares command line parameters for various datasets-related tasks.

    :type command_subparsers: argparse.SubParserAction
    """

    parser = command_subparsers.add_parser("datasets")
    subparsers = parser.add_subparsers()

    # 'generate' subcommand
    parser_generate = subparsers.add_parser("generate")
    parser_generate = add_common_subcommand_args(parser_generate)
    parser_generate = generate.add_cli_args(parser_generate)
    parser_generate.set_defaults(subcommand=generate.command)

    # 'merge' subcommand
    parser_merge = subparsers.add_parser("merge")
    parser_merge = merge.add_cli_args(parser_merge)
    parser_merge.set_defaults(subcommand=merge.command)

    # 'clean' subcommand
    parser_clean = subparsers.add_parser("clean")
    parser_clean = clean.add_cli_args(parser_clean)
    parser_clean.set_defaults(subcommand=clean.command)
