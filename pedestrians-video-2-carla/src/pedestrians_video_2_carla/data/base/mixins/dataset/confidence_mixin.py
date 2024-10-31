import torch


class ConfidenceMixin:
    def __init__(self,
                 needs_confidence: bool = False,
                 **kwargs
                 ):
        super().__init__(**kwargs)
        # TODO: handle setting needs_confidence param better, for now it assumes the model has such a CLI arg
        self.return_confidence = needs_confidence

    def process_confidence(self, projection_2d: torch.Tensor) -> torch.Tensor:
        if self.return_confidence:
            if projection_2d.shape[-1] > 2:
                return projection_2d
            with_confidence = torch.cat([projection_2d, torch.ones((*projection_2d.shape[:-2], 1))],
                                        dim=projection_2d.ndim - 1)
            return with_confidence
        return projection_2d[..., :2]
