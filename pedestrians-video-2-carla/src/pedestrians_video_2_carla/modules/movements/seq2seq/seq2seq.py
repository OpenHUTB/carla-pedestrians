from enum import Enum
from typing import Dict, Iterable, List, Tuple, Union
import warnings
from pytorch3d.transforms.rotation_conversions import matrix_to_rotation_6d
import torch
from torch import nn
from torch import Tensor
from pedestrians_video_2_carla.modules.movements.movements import MovementsModel, MovementsModelOutputType, MovementsModelOutputTypeMixin
from pedestrians_video_2_carla.utils.argparse import boolean


class TeacherMode(Enum):
    """
    Enum for teacher mode.
    """
    no_force = 0
    clip_force = 1
    frames_force = 2


class Encoder(nn.Module):
    def __init__(
        self,
        hid_dim=64,
        n_layers=2,
        dropout=0.2,
        input_size=26*2,
        bidirectional=True
    ):
        super().__init__()

        self.hid_dim = hid_dim
        self.n_layers = n_layers
        self.input_size = input_size

        if isinstance(hid_dim, int):
            self.rnn = nn.LSTM(self.input_size, hid_dim, num_layers=n_layers,
                               dropout=dropout, bidirectional=bidirectional)
        else:
            sizes = [self.input_size] + list(hid_dim)
            self.rnn = nn.Sequential(*[
                nn.LSTM(sizes[i], sizes[i+1], num_layers=1,
                        dropout=dropout, bidirectional=bidirectional)
                for i in range(len(sizes) - 1)
            ])

    def forward(self, x):

        original_shape = x.shape
        x = x.view(*original_shape[0:2], self.input_size)
        _, (hidden, cell) = self.rnn(x)

        return hidden, cell


class Decoder(nn.Module):
    def __init__(
        self,
        hid_dim=64,
        n_layers=2,
        dropout=0.2,
        output_size=26*6,
        bidirectional=False
    ):
        super().__init__()

        self.hid_dim = hid_dim
        self.n_layers = n_layers
        self.output_size = output_size

        if isinstance(hid_dim, int):
            self.rnn = nn.LSTM(self.output_size, hid_dim, num_layers=n_layers,
                               dropout=dropout, bidirectional=bidirectional)
        else:
            sizes = [self.output_size] + list(hid_dim)
            self.rnn = nn.Sequential(*[
                nn.LSTM(sizes[i], sizes[i+1], num_layers=1,
                        dropout=dropout, bidirectional=bidirectional)
                for i in range(len(sizes) - 1)
            ])

        self.fc_out = nn.Linear(
            hid_dim*2 if bidirectional else hid_dim,
            self.output_size
        )
        self.dropout = nn.Dropout(dropout)

    def forward(self, x, hidden, cell):
        x = x.unsqueeze(0)
        output, (hidden, cell) = self.rnn(x, (hidden, cell))
        prediction = self.fc_out(output.squeeze(0))

        return prediction, hidden, cell


class Seq2Seq(MovementsModelOutputTypeMixin, MovementsModel):
    """
    Sequence to sequence model.

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
                 hidden_size: Union[int, Iterable[int]] = 64,
                 num_layers: int = 2,
                 p_dropout: float = 0.2,
                 teacher_mode: TeacherMode = TeacherMode.no_force,
                 teacher_force_ratio: float = 0.2,
                 teacher_force_drop: float = 0.02,
                 input_features: int = 2,
                 invert_sequence: bool = False,
                 bidirectional: bool = False,
                 input_size: int = None,
                 **kwargs):
        super().__init__(**kwargs)

        if input_size is not None and input_features is not None:
            warnings.warn(
                "Both input_size and input_features were specified, using input_size.")

        if input_size is None:
            self.input_size = input_features * len(self.input_nodes)
        else:
            self.input_size = input_size
        self.output_size = self.output_features * len(self.output_nodes)

        self.teacher_mode = teacher_mode if isinstance(
            teacher_mode, TeacherMode) else TeacherMode[teacher_mode]
        self.teacher_force_ratio = teacher_force_ratio if self.teacher_mode != TeacherMode.no_force else 0.0
        self.teacher_force_drop = teacher_force_drop if self.teacher_mode != TeacherMode.no_force else 0.0

        if not isinstance(hidden_size, int):
            assert len(
                hidden_size) == num_layers, "hidden_size must be a single int or a list of ints of length num_layers"

        self.encoder = Encoder(
            hid_dim=hidden_size,
            n_layers=num_layers,
            dropout=p_dropout,
            input_size=self.input_size,
            bidirectional=bidirectional
        )
        self.decoder = Decoder(
            hid_dim=hidden_size if isinstance(hidden_size, int) else hidden_size[::-1],
            n_layers=num_layers,
            dropout=p_dropout,
            output_size=self.output_size,
            bidirectional=bidirectional
        )

        self.invert_sequence = invert_sequence

        self._hparams.update({
            'hidden_size': hidden_size,
            'num_layers': num_layers,
            'p_dropout': p_dropout,
            'teacher_mode': self.teacher_mode.name,
            'teacher_force_ratio': self.teacher_force_ratio,
            'teacher_force_drop': self.teacher_force_drop,
            'invert_sequence': self.invert_sequence,
            'bidirectional': bidirectional,
        })

    @property
    def needs_targets(self) -> bool:
        return self.teacher_mode != TeacherMode.no_force

    @staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = MovementsModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("Seq2Seq Movements Module")
        parser = MovementsModelOutputTypeMixin.add_cli_args(parser)
        parser.add_argument(
            '--num_layers',
            default=2,
            type=int,
        )
        parser.add_argument(
            '--hidden_size',
            default=64,
            type=int,
        )
        parser.add_argument(
            '--p_dropout',
            default=0.2,
            type=float,
        )
        parser.add_argument(
            '--teacher_mode',
            help="""
                Set teacher mode for decoder training.
                """.format(
                set(TeacherMode.__members__.keys())),
            default=TeacherMode.no_force,
            choices=list(TeacherMode),
            type=TeacherMode.__getitem__
        )
        parser.add_argument(
            '--teacher_force_ratio',
            help="""
                Set teacher force ratio for decoder training.
                Only used if teacher_mode is not TeacherMode.no_force.
                """,
            default=0.2,
            type=float
        )
        parser.add_argument(
            '--teacher_force_drop',
            help="""
                Set teacher force ratio drop per epoch for decoder training.
                Only used if teacher_mode is not TeacherMode.no_force.
                """,
            default=0.02,
            type=float
        )
        parser.add_argument(
            '--invert_sequence',
            help="""Should the sequence be inverted before training? Doesn't make sense for bidirectional models.""",
            default=False,
            type=boolean
        )
        parser.add_argument(
            '--bidirectional',
            help="""Should the encoder/decoder use bidirectional LSTM?""",
            default=False,
            type=boolean
        )

        return parent_parser

    def forward(self, x: Tensor, targets: Dict[str, Tensor] = None, *args, **kwargs) -> Tensor:
        original_shape = x.shape
        batch_size, clip_length, *_ = original_shape

        x = self._format_input(x)

        # last hidden state of the encoder is used as the initial hidden state of the decoder
        hidden, cell = self.encoder(x)

        # first input to the decoder is the <sos> tokens
        input = torch.zeros((batch_size, self.decoder.output_size), device=x.device)

        needs_forcing, target_pose_changes, force_indices = self._teacher_forcing(
            targets)

        # tensor to store decoder outputs
        outputs = torch.zeros(
            (clip_length, batch_size, self.decoder.output_size), device=x.device)
        for t in range(0, clip_length):
            input, output = self._decode_frame(hidden, cell, input,
                                               needs_forcing,
                                               force_indices[t] if needs_forcing else None,
                                               target_pose_changes[t, force_indices[t]] if needs_forcing else None)
            outputs[t] = output

        return self._format_output(original_shape, outputs)

    def _decode_frame(self,
                      hidden: torch.Tensor,
                      cell: torch.Tensor,
                      input: torch.Tensor,
                      needs_forcing: bool,
                      force_indices: torch.Tensor,
                      target_pose_changes: torch.Tensor
                      ) -> Tuple[torch.Tensor, torch.Tensor]:
        # insert input token embedding, previous hidden and previous cell states
        # receive output tensor (predictions) and new hidden and cell states
        output, hidden, cell = self.decoder(input, hidden, cell)
        input = output

        if needs_forcing:
            input[force_indices] = target_pose_changes

        return input, output

    def _format_output(self, original_shape, outputs):
        """
        At the very last this should convert from sequence-first back to batch-first format
        and ensure rotation matrices are returned.

        :param x: Outputs from the decoder.
        :type x: torch.Tensor
        :return: (B, L, P, x) tensor, where B is batch size, L is clip length, P is number of output nodes. x depends on the movements output type.
        :rtype: torch.Tensor
        """
        # convert to batch-first format
        outputs = outputs.permute(1, 0, 2)
        outputs = outputs.view(*original_shape[:3], self.output_features)

        return super()._format_output(outputs)

    def _format_input(self, x: Tensor) -> Tensor:
        """
        At the very least this should convert from batch-first to sequence-first.

        :param x: Inputs to the model.
        :type x: torch.Tensor
        :return: Input tensor in sequence-first format, ready to be fed into the encoder.
        :rtype: torch.Tensor
        """
        # convert to sequence-first format
        x = x.permute(1, 0, *range(2, x.dim()))

        if self.invert_sequence:
            x = x.flip(0)

        return x

    def _teacher_forcing(self, targets):
        needs_forcing = self.training and self.teacher_mode != TeacherMode.no_force and targets is not None and self.teacher_force_ratio > 0
        target_output = None
        force_indices = None

        if needs_forcing:
            if self.output_type == MovementsModelOutputType.pose_changes:
                target_output = matrix_to_rotation_6d(targets['pose_changes'])
            elif self.output_type == MovementsModelOutputType.pose_2d:
                # TODO: get correct output key, support all output types
                target_output = targets['projection_2d_transformed']

            (batch_size, clip_length, *_) = target_output.shape

            target_output = target_output.permute(
                1, 0, *range(2, target_output.dim())).reshape((clip_length, batch_size, self.decoder.output_size))

            if self.teacher_mode == TeacherMode.clip_force:
                # randomly select clips that should be taken from targets
                force_indices = (torch.rand(
                    (1, batch_size), device=target_output.device) < self.teacher_force_ratio).repeat((clip_length, 1))
            elif self.teacher_mode == TeacherMode.frames_force:
                # randomly select frames that should be taken from targets
                force_indices = torch.rand(
                    (clip_length, batch_size), device=target_output.device) < self.teacher_force_ratio

        return needs_forcing, target_output, force_indices

    def training_epoch_end(self, *args, **kwargs) -> Dict[str, float]:
        if self.teacher_mode != TeacherMode.no_force:
            current_ratio = self.teacher_force_ratio

            # TODO: this value should be intelligently adjusted based on the loss/metrics/whatever
            # similar to what can be done for lr
            self.teacher_force_ratio = (self.teacher_force_ratio -
                                        self.teacher_force_drop) if self.teacher_force_ratio > self.teacher_force_drop else 0
            return {
                'teacher_force_ratio/{}'.format(self.teacher_mode.name): current_ratio
            }
        else:
            return {}
