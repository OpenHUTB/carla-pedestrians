from pedestrians_video_2_carla.modules.movements.movements import MovementsModelOutputType
from torch import nn
from .pose_former import PoseFormer
from pytorch3d.transforms.rotation_conversions import rotation_6d_to_matrix


class PoseFormerRot(PoseFormer):
    """
    Based on the [PoseFormer implementation](https://github.com/zczcwh/PoseFormer)
    from the following paper:

    ```bibtex
    @article{zheng2021poseformer,
    title={3D Human Pose Estimation with Spatial and Temporal Transformers},
    author={Zheng, Ce and Zhu, Sijie and Mendieta, Matias and Yang,
        Taojiannan and Chen, Chen and Ding, Zhengming},
    journal={Proceedings of the IEEE International Conference on Computer Vision (ICCV)},
    year={2021}
    }
    ```

    but with added rotations.
    """

    def __init__(self,
                 **kwargs):
        super().__init__(output_features=9, **kwargs)

        single_joint_embeddings_size = kwargs.get('single_joint_embeddings_size', 32)
        embed_dim = len(self.input_nodes) * single_joint_embeddings_size

        out_dim = len(self.output_nodes) * 9

        self.pose_former.head = nn.Sequential(
            nn.LayerNorm(embed_dim),
            nn.Linear(embed_dim, out_dim),
        )

    @property
    def output_type(self) -> MovementsModelOutputType:
        return MovementsModelOutputType.absolute_loc_rot

    def forward(self, x, *args, **kwargs):
        outputs = super().forward(x, *args, **kwargs)

        return (outputs[..., :3], rotation_6d_to_matrix(outputs[..., 3:]))
