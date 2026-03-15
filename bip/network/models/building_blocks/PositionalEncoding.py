
import torch.nn as nn
import torch
import math

class PositionalEncoding(nn.Module):
    r"""注入一些有关序列中标记的相对或绝对位置的信息。位置编码与嵌入具有相同的维度，因此可以将两者相加。
        在这里，我们使用不同频率的正弦和余弦函数。
    .. math::
        \text{PosEncoder}(pos, 2i) = sin(pos/10000^(2i/d_model))
        \text{PosEncoder}(pos, 2i+1) = cos(pos/10000^(2i/d_model))
        \text{where pos is the word position and i is the embed idx)
    Args:
        d_model: the embed dim (required).
        dropout: the dropout value (default=0.1).
        max_len: the max. length of the incoming sequence (default=5000).
    Examples:
        >>> pos_encoder = PositionalEncoding(d_model)
    """

    def __init__(self, d_model, dropout=0.1, max_len=10):
        super(PositionalEncoding, self).__init__()
        self.d_model = d_model
        self.dropout = nn.Dropout(p=dropout)

        pe = torch.zeros(max_len, d_model)     # [max_len, d_model]
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)    # [max_len, 1]
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))   # [d_model/2]
        pe[:, 0::2] = torch.sin(position * div_term)   #[max_len, d_model/2]
        pe[:, 1::2] = torch.cos(position * div_term)   #[max_len, d_model/2]
        pe = pe.unsqueeze(0)                           #[1, max_len, d_model]
        self.register_buffer('pe', pe)

    def forward(self, x):
        r"""前向函数的输入
        Args:
            x: 输入位置编码器模型的序列（必需）。
        Shape:
            x: [batch size, sequence length, embed dim]
            output: [batch size, sequence length, embed dim]
        Examples:
            >>> output = pos_encoder(x)
        """
        x = x + self.pe[:, :x.size(1), :]      #[B, len, d_model] + [1, len, d_model]
        return self.dropout(x)

