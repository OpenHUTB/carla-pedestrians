ARG PLATFORM=nvidia

# Reuse the pedestrians-scenarios code
# TODO: pedestrians-scenarios should be publicly avaliable and a dependency of pedestrians-video-2-carla
FROM wielgoszinfo/pedestrians-scenarios:${PLATFORM}-latest AS scenarios
RUN /venv/bin/python -m pip install --no-cache-dir build
WORKDIR /app
RUN SETUPTOOLS_SCM_PRETEND_VERSION=0.0.1 /venv/bin/python -m build

FROM wielgoszinfo/carla-common:${PLATFORM}-latest AS base

ENV PACKAGE=pedestrians-video-2-carla

ENV torch_version=1.9.1
ENV torchvision_version=0.10.1
ENV pytorch3d_version=0.6.0
ENV torchscatter_version=2.0.9
ENV torchsparse_version=0.6.12
ENV torchcluster_version=1.5.9
ENV torchsplineconv_version=1.2.1
ENV torchgeometric_version=2.0.3

# ----------------------------------------------------------------------------
# NVIDIA-specific dependencies
# ----------------------------------------------------------------------------
FROM base as torch-nvidia

ENV PYOPENGL_PLATFORM=egl

USER root
# Get OpenGL working for off-screen rendering
# https://gitlab.com/nvidia/container-images/opengl/blob/ubuntu20.04/glvnd/runtime/Dockerfile
# extras: libglu1, libgl1-mesa-glx
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    libglu1 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=nvidia/cudagl:11.1.1-base-ubuntu20.04 /usr/share/glvnd/egl_vendor.d/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
USER ${USERNAME}

RUN /venv/bin/python -m pip install --no-cache-dir -f https://download.pytorch.org/whl/torch_stable.html \
    torch==${torch_version}+cu111 \
    torchvision==${torchvision_version}+cu111

RUN /venv/bin/python -m pip install  --no-cache-dir -f https://data.pyg.org/whl/torch-${torch_version}+cu111.html \ 
    torch-scatter==${torchscatter_version} \
    torch-sparse==${torchsparse_version} \
    torch-cluster==${torchcluster_version} \
    torch-spline-conv==${torchsplineconv_version} \
    torch-geometric==${torchgeometric_version}

# ----------------------------------------------------------------------------
# CPU-specific dependencies
# ----------------------------------------------------------------------------
FROM base as torch-cpu

ENV PYOPENGL_PLATFORM=osmesa

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    freeglut3 \
    freeglut3-dev \
    libgle3 \
    libgle3-dev \
    libosmesa6 \
    libosmesa6-dev \
    && rm -rf /var/lib/apt/lists/*
USER ${USERNAME}

RUN /venv/bin/python -m pip install --no-cache-dir -f https://download.pytorch.org/whl/cpu/torch_stable.html \
    torch==${torch_version}+cpu \
    torchvision==${torchvision_version}+cpu

RUN /venv/bin/python -m pip install  --no-cache-dir -f https://data.pyg.org/whl/torch-${torch_version}+cpu.html \ 
    torch-scatter==${torchscatter_version} \
    torch-sparse==${torchsparse_version} \
    torch-cluster==${torchcluster_version} \
    torch-spline-conv==${torchsplineconv_version} \
    torch-geometric==${torchgeometric_version}

# -------------------------------------------------------------------------------------------------
# Common II
# -------------------------------------------------------------------------------------------------
FROM torch-${PLATFORM} as torch

# PyTorch3D (need to compile to get newer version, wheels are up to 0.3.0)
RUN /venv/bin/python -m pip install --no-cache-dir \
    "git+https://github.com/facebookresearch/pytorch3d.git@v${pytorch3d_version}"

# Direct project dependencies are defined in pedestrians-video-2-carla/setup.cfg
# However, we want to leverage the cache, so we're going to specify them (and some of their dependencies) here
RUN /venv/bin/python -m pip install --no-cache-dir \
    av==8.0.3 \
    cameratransform==1.2 \
    dotmap==1.3.26 \
    einops==0.3.2 \
    gym==0.21.0 \
    h5pickle==0.4.2 \
    h5py==3.6.0 \
    matplotlib==3.5.1 \
    moviepy==1.0.3 \
    networkx==2.2 \
    numpy==1.22.3 \
    opencv-python-headless==4.5.4.58 \
    pandas==1.3.5 \
    Pillow==9.0.1 \
    pims==0.5 \
    pyrender==0.1.45 \
    pytorch-lightning==1.6.3 \
    pyyaml==6.0 \
    randomname==0.1.5 \
    scikit-image==0.18.3 \
    scikit-learn==1.0.2 \
    scipy==1.7.2 \
    timm==0.4.12 \
    torchmetrics==0.8.2 \
    tqdm==4.62.3 \
    transforms3d==0.3.1 \
    trimesh==3.9.36 \
    wandb==0.13.3 \
    xmltodict==0.12.0 \
    git+https://github.com/nghorbani/human_body_prior.git@0278cb45180992e4d39ba1a11601f5ecc53ee148#egg=human-body-prior \
    git+https://github.com/nghorbani/body_visualizer@be9cf756f8d1daed870d4c7ad1aa5cc3478a546c#egg=body-visualizer \
    git+https://github.com/MPI-IS/configer.git@8cd1e3e556d9697298907800a743e120be57ac36#egg=configer \
    git+https://github.com/MPI-IS/mesh.git@49e70425cf373ec5269917012bda2944215c5ccd#egg=psbody-mesh

# install newer version of pyopengl, since pyrender has obsolete dependency
RUN /venv/bin/python -m pip install --no-cache-dir \
    PyOpenGL==3.1.5 \
    PyOpenGL-accelerate==3.1.5

RUN /venv/bin/python -m pip install  --no-cache-dir \
    torch-geometric-temporal==0.51.0

# reuse pedestrians-scenarios code
COPY --from=scenarios --chown=${USERNAME}:${USERNAME} /app/third_party/scenario_runner/srunner /venv/lib/python3.8/site-packages/srunner
COPY --from=scenarios --chown=${USERNAME}:${USERNAME} /app/dist/pedestrians_scenarios-0.0.1-py3-none-any.whl ${HOME}
RUN /venv/bin/python -m pip install --no-cache-dir ${HOME}/pedestrians_scenarios-0.0.1-py3-none-any.whl

# Copy client files so that we can do editable pip install
COPY --chown=${USERNAME}:${USERNAME} . /app
