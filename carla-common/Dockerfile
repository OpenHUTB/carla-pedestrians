# nvidia or cpu
ARG PLATFORM=nvidia

# ----------------------------------------------------------------------------
# Choose base image based on the ${PLATFORM} variable
# ----------------------------------------------------------------------------

FROM nvidia/cuda:11.4.3-cudnn8-runtime-ubuntu20.04 as base-nvidia
FROM ubuntu:20.04 as base-cpu
FROM base-${PLATFORM} as base

# ----------------------------------------------------------------------------
# Common dependencies
# ----------------------------------------------------------------------------

ENV TZ=Europe/Warsaw
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        ffmpeg \
        g++ \
        gcc \
        git \
        libboost-python-dev \
        libjpeg-dev \
        libjpeg-turbo8-dev \
        libpng-dev \
        software-properties-common \
        screen \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3.10 \
        python3.10-dev \
        python3.10-venv \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=carla
ENV USERNAME=${USERNAME}
ENV HOME /home/${USERNAME}

RUN groupadd -g ${GROUP_ID} ${USERNAME} \
    && useradd -ms /bin/bash -u ${USER_ID} -g ${GROUP_ID} ${USERNAME} \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && mkdir ${HOME}/.vscode-server ${HOME}/.vscode-server-insiders /outputs /venv /app \
    && chown ${USERNAME}:${USERNAME} ${HOME}/.vscode-server ${HOME}/.vscode-server-insiders /outputs /venv /app

# Everything else can be run as user since we need venv anyway
USER ${USERNAME}

# Create venv to allow editable installation of python packages
RUN python3.10 -m venv /venv

# Update basic python packages
RUN /venv/bin/python -m pip install --no-cache-dir -U \
    pip==24.1.1 \
    setuptools==59.5.0 \
    wheel==0.37.1 \
    pure_eval==0.2.1

# Add some utility/development requirements
RUN /venv/bin/python -m pip install --no-cache-dir \
    autopep8 \
    ipython \
    ipykernel \
    ipywidgets \
    pylint \
    pytest \
    pytest-cov \
    unify

# Automatically activate virtualenv for user
RUN echo 'source /venv/bin/activate' >> ${HOME}/.bashrc

# Install carla client library and numpy
RUN /venv/bin/python -m pip install --no-cache-dir \
    carla==0.9.15 \
    numpy

# Copy the 'agents' package used by e.g. scenario runner. It is a part of provided PythonAPI, but not a part of carla package.
COPY --from=carlasim/carla:0.9.15 --chown=${USERNAME}:${USERNAME} /home/carla/PythonAPI/carla/agents /venv/lib/python3.10/site-packages/agents

COPY --chown=${USERNAME}:${USERNAME} ./entrypoint.sh ${HOME}/entrypoint.sh
COPY --chown=${USERNAME}:${USERNAME} ./client ${HOME}/client

WORKDIR ${HOME}
ENTRYPOINT [ "/home/carla/entrypoint.sh" ]

# Run infinite loop to allow easily attach to container
CMD ["/bin/sh", "-c", "while sleep 1000; do :; done"]

# Those you probably want to map to host/named volumes
VOLUME [ "${HOME}/outputs", "${HOME}/.vscode-server" ]
