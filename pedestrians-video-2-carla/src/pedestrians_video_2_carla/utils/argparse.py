import argparse
from distutils.util import strtobool


class MinMaxAction(argparse.Action):
    def __init__(self, option_strings, dest, minimum=None, maximum=None, **kwargs) -> None:
        super().__init__(option_strings, dest, **kwargs)
        self.minimum = minimum
        self.maximum = maximum

    def __call__(self, parser, namespace, values, option_string=None):
        if self.minimum is not None and values < self.minimum:
            raise parser.error(
                f"{self.dest} must be greater than or equal to {self.minimum}")
        if self.maximum is not None and values > self.maximum:
            raise parser.error(
                f"{self.dest} must be less than or equal to {self.maximum}")

        setattr(namespace, self.dest, values)


class DictAction(argparse.Action):
    def __init__(self, option_strings, dest, value_type=str, **kwargs) -> None:
        super().__init__(option_strings, dest, **kwargs)
        self.value_type = value_type

    def __call__(self, parser, namespace, values, option_string=None):
        prev_dict = getattr(namespace, self.dest, {})
        for value in values:
            key, value = value.split('=')
            prev_dict[key] = self.value_type(value)

        setattr(namespace, self.dest, prev_dict)


def boolean(x):
    return bool(strtobool(x))


def boolean_or_float(x):
    try:
        return boolean(x)
    except ValueError:
        return float(x)


def list_arg_as_flat_args(parser, name, max_length, defaults=None, value_type=float, help=None):
    """
    Adds `name` argument as a `max_length` individual arguments instead of non-sweep-compatible `nargs='+'`.
    """

    for i in range(max_length):
        parser.add_argument(
            f'--{name}_{i}',
            default=defaults[i] if (defaults is not None and i <
                                    len(defaults)) else None,
            type=value_type,
            help=help if (help is not None and i == 0) else None
        )

    return parser


def flat_args_as_list_arg(kwargs, name, pop=False):
    """
    Converts a `max_length` individual arguments to a single `name` argument.
    """

    if name in kwargs:
        values = kwargs[name]
    else:
        flat_kwargs = [kw for kw in kwargs.keys(
        ) if kw.startswith(f'{name}_')]
        flat_kwargs.sort(key=lambda x: int(x[len(name) + 1:]))
        values = [kwargs[kw] for kw in flat_kwargs if kwargs[kw] is not None]

    if pop:
        for kw in flat_kwargs:
            kwargs.pop(kw)

    return values
