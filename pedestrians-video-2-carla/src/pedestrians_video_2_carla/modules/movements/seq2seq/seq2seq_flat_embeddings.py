from typing import Iterable, Union
from pedestrians_video_2_carla.utils.argparse import flat_args_as_list_arg, list_arg_as_flat_args
from torch import nn
from .seq2seq import Seq2Seq


class Seq2SeqFlatEmbeddings(Seq2Seq):
    """
    Sequence to sequence model with embeddings and optionally inverted input sequence.

    Based on the code from [Sequence to Sequence Learning with Neural Networks](https://github.com/bentrevett/pytorch-seq2seq/blob/master/1%20-%20Sequence%20to%20Sequence%20Learning%20with%20Neural%20Networks.ipynb)
    by [Ben Trevett](https://github.com/bentrevett) licensed under [MIT License](https://github.com/bentrevett/pytorch-seq2seq/blob/master/LICENSE),
    which itself is an implementation of the paper https://arxiv.org/abs/1409.3215:

    ```bibtex
    @misc{sutskever2014sequence,
        title={Sequence to Sequence Learning with Neural Networks},
        author={Ilya Sutskever and Oriol Vinyals and Quoc V. Le},
        year={2014},
        eprint={1409.3215},
        archivePrefix={arXiv},
        primaryClass={cs.CL}
    }
    ```
    """

    def __init__(self,
                 input_features: int = 2,
                 **kwargs):

        self.embeddings_size = flat_args_as_list_arg(kwargs, 'embeddings_size')

        super().__init__(**{
            **kwargs,
            'input_features': None,
            'input_size': self.embeddings_size[-1] if self.embeddings_size else None,
        })

        sizes = [input_features * len(self.input_nodes)] + self.embeddings_size
        layers = [
            (nn.Linear(sizes[i], sizes[i+1]), nn.ReLU())
            for i in range(len(sizes)-1)
        ]
        self.embeddings = nn.Sequential(*[l for la in layers for l in la])

        self._hparams.update({
            'embeddings_size': self.embeddings_size,
        })

    @staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = Seq2Seq.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group(
            "Seq2SeqFlatEmbeddings Movements Module")

        parser = list_arg_as_flat_args(parser, 'embeddings_size', 5, [128, 64], int)

        return parent_parser

    def _format_input(self, x):
        batch_size, clip_length, *_ = x.shape

        embeddings = self.embeddings(
            x.view(batch_size * clip_length, -1)).view(batch_size, clip_length, -1)

        # convert to sequence-first format
        embeddings = embeddings.permute(1, 0, 2)

        if self.invert_sequence:
            embeddings = embeddings.flip(0)

        return embeddings
