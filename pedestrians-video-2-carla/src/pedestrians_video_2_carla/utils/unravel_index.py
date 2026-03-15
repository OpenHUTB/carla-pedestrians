# hopefully one day this will be merged into pytorch
# but for now copied from PR
# https://github.com/pytorch/pytorch/pull/66687

from typing import Sequence, Tuple, Union
import torch
from torch import Tensor


class _dispatch_dtypes(tuple):
    def __add__(self, other):
        assert isinstance(other, tuple)
        return _dispatch_dtypes(tuple.__add__(self, other))

_integral_types = _dispatch_dtypes((torch.uint8, torch.int8, torch.int16, torch.int32, torch.int64))
def integral_types():
    return _integral_types


def unravel_index(
    indices: Tensor,
    shape: Union[int, Sequence, Tensor],
    *,
    as_tuple: bool = True
) -> Union[Tuple[Tensor, ...], Tensor]:
    r"""Converts a `Tensor` of flat indices into a `Tensor` of coordinates for the given target shape.
    Args:
        indices: An integral `Tensor` containing flattened indices of a `Tensor` of dimension `shape`.
        shape: The shape (can be an `int`, a `Sequence` or a `Tensor`) of the `Tensor` for which
               the flattened `indices` are unraveled.
    Keyword Args:
        as_tuple: A boolean value, which if `True` will return the result as tuple of Tensors,
                  else a `Tensor` will be returned. Default: `True`
    Returns:
        unraveled coordinates from the given `indices` and `shape`. See description of `as_tuple` for
        returning a `tuple`.
    .. note:: The default behaviour of this function is analogous to
              `numpy.unravel_index <https://numpy.org/doc/stable/reference/generated/numpy.unravel_index.html>`_.
    Example::
        >>> indices = torch.tensor([22, 41, 37])
        >>> shape = (7, 6)
        >>> torch.unravel_index(indices, shape)
        (tensor([3, 6, 6]), tensor([4, 5, 1]))
        >>> torch.unravel_index(indices, shape, as_tuple=False)
        tensor([[3, 4],
                [6, 5],
                [6, 1]])
        >>> indices = torch.tensor([3, 10, 12])
        >>> shape_ = (4, 2, 3)
        >>> torch.unravel_index(indices, shape_)
        (tensor([0, 1, 2]), tensor([1, 1, 0]), tensor([0, 1, 0]))
        >>> torch.unravel_index(indices, shape_, as_tuple=False)
        tensor([[0, 1, 0],
                [1, 1, 1],
                [2, 0, 0]])
    """
    def _helper_type_check(inp: Union[int, Sequence, Tensor], name: str):
        # `indices` is expected to be a tensor, while `shape` can be a sequence/int/tensor
        if name == "shape" and isinstance(inp, Sequence):
            for dim in inp:
                if not isinstance(dim, int):
                    raise TypeError("Expected shape to have only integral elements.")
                if dim < 0:
                    raise ValueError("Negative values in shape are not allowed.")
        elif name == "shape" and isinstance(inp, int):
            if inp < 0:
                raise ValueError("Negative values in shape are not allowed.")
        elif isinstance(inp, Tensor):
            if inp.dtype not in integral_types():
                raise TypeError(
                    f"Expected {name} to be an integral tensor, but found dtype: {inp.dtype}")
            if torch.any(inp < 0):
                raise ValueError(f"Negative values in {name} are not allowed.")
        else:
            allowed_types = "Sequence/Scalar (int)/Tensor" if name == "shape" else "Tensor"
            msg = f"{name} should either be a {allowed_types}, but found {type(inp)}"
            raise TypeError(msg)

    _helper_type_check(indices, "indices")
    _helper_type_check(shape, "shape")

    # Convert to a tensor, with the same properties as that of indices
    if isinstance(shape, Sequence):
        shape_tensor: Tensor = indices.new_tensor(shape)
    elif isinstance(shape, int) or (isinstance(shape, Tensor) and shape.ndim == 0):
        shape_tensor = indices.new_tensor((shape,))
    else:
        shape_tensor = shape

    # By this time, shape tensor will have dim = 1 if it was passed as scalar (see if-elif above)
    assert shape_tensor.ndim == 1, "Expected dimension of shape tensor to be <= 1, "
    f"but got the tensor with dim: {shape_tensor.ndim}."

    # In case no indices passed, return an empty tensor with number of elements = shape.numel()
    if indices.numel() == 0:
        # If both indices.numel() == 0 and shape.numel() == 0, short-circuit to return itself
        shape_numel = shape_tensor.numel()
        if shape_numel == 0:
            raise ValueError(
                "Got indices and shape as empty tensors, expected non-empty tensors.")
        else:
            output = [indices.new_tensor([]) for _ in range(shape_numel)]
            return tuple(output) if as_tuple else torch.stack(output, dim=1)

    if torch.max(indices) >= torch.prod(shape_tensor):
        raise ValueError("Target shape should cover all source indices.")

    coefs = shape_tensor[1:].flipud().cumprod(dim=0).flipud()
    coefs = torch.cat((coefs, coefs.new_tensor((1,))), dim=0)
    coords = torch.div(indices[..., None], coefs, rounding_mode='trunc') % shape_tensor

    if as_tuple:
        return tuple(coords[..., i] for i in range(coords.size(-1)))
    return coords
