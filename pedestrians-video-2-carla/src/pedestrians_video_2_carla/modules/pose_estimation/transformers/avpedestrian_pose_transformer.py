from pedestrians_video_2_carla.modules.flow.output_types import PoseEstimationModelOutputType
from torch import nn
from pedestrians_video_2_carla.modules.pose_estimation.pose_estimation import PoseEstimationModel
from torchvision.models import resnet18, resnet34, resnet50, resnet101, resnet152
from torch_geometric.nn import TransformerConv

class AtrousModule(nn.Module):
    def __init__(self, inplanes, planes, kernel_size, padding, dilation, BatchNorm):
        super(AtrousModule, self).__init__()
        self.atrous_conv = nn.Conv2d(inplanes, planes, kernel_size=kernel_size,
                                            stride=1, padding=padding, dilation=dilation, bias=False)
        self.bn = BatchNorm(planes)
        self.relu = nn.ReLU()

        self._init_weight()

    def forward(self, x):
        x = self.atrous_conv(x)
        x = self.bn(x)

        return self.relu(x)

    def _init_weight(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight)
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()



class AvPedestrianPoseTransformer(PoseEstimationModel):
    """
    The simplest dummy model used to debug the flow.
    """

    def __init__(self,
                 **kwargs
                 ):
        super().__init__(
            **kwargs
        )

        self.__output_nodes_len = len(self.output_nodes)

        resnet_backbone = resnet50(pretrained=True)
        self.reduced_resnet = nn.Sequential(*(list(resnet_backbone.children())[:-1]))
   
        # this is for small resnets
        # self.linear_first = nn.Linear(512, 256)
        # self.linear_second = nn.Linear(256, self.__output_nodes_len * 2)

        # this is for large resnets
        self.linear_first = nn.Linear(2048, 256)
        self.linear_second = nn.Linear(256, self.__output_nodes_len * 2)
        
        # self.linear_first = nn.Linear(512, 128)
        # self.linear_second = nn.Linear(128, self.__output_nodes_len * 2)


        self.encoder_layer = nn.TransformerEncoderLayer(
            d_model=self.__output_nodes_len * 2,
            nhead=4,
            batch_first=True
        )
        self.encoder = nn.TransformerEncoder(self.encoder_layer, num_layers=6)
    
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(p=0.5)
        self.linear_after_transformer = nn.Linear(self.__output_nodes_len * 2, self.__output_nodes_len * 2)



    @property
    def output_type(self) -> PoseEstimationModelOutputType:
        return PoseEstimationModelOutputType.pose_2d

    @property
    def needs_confidence(self) -> bool:
        return False

    def forward(self, x, *args, **kwargs):
        b, t, c, h, w = x.shape
        x = x.view(b * t, c, h, w)

        # change here
        x = self.reduced_resnet(x)
        x = x.view(b, t, -1)

        x = self.linear_first(x)
        x = self.dropout(x)
        x = self.relu(x)

        x = self.linear_second(x)
        x = self.dropout(x)
        x = self.relu(x)
 
        orig_shape = x.shape
        x = x.view(orig_shape[0], orig_shape[1], -1)
        # print("input shape: ", x.shape)
        x = self.encoder(x)
        x = x.view(orig_shape)

        # to be removed
        x = self.dropout(x)
        x = self.relu(x)
        # to be removed

        x = self.linear_after_transformer(x)

        x = x.view(b, t, self.__output_nodes_len, 2)

        return x