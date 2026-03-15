ARG PLATFORM=nvidia
FROM wielgoszinfo/carla-common:${PLATFORM}-latest AS scenarios-cpu

ENV PACKAGE=pedestrians-scenarios

# Install direct dependencies and scenario_runner ones
# TODO: change gym to gym==0.17.2 ? 
RUN /venv/bin/python -m pip install --no-cache-dir \
    av==8.0.3 \
    cameratransform==1.2 \
    ephem==4.1.3 \
    gym==0.23.1 \
    h5py==3.6.0 \
    hydra==2.5 \
    matplotlib==3.5.1 \
    networkx==2.2 \
    numpy==1.22.1 \
    omegaconf==2.0.2 \
    opencv-python-headless==4.2.0.32 \
    pandas==1.3.5 \
    Pillow==9.0.0 \
    pims==0.5 \
    psutil==5.9.0 \
    py-trees==0.8.3 \
    pyyaml==6.0 \
    scikit-image==0.18.3 \
    scipy==1.7.2 \
    Shapely==1.7.1 \
    simple-watchdog-timer==0.1.1 \
    six==1.16.0 \
    tabulate==0.8.9 \
    tqdm==4.62.3 \
    wandb==0.12.16 \
    xmlschema==1.0.18

# "Install" scenario_runner, so that srunner imports succeed
RUN ln -s /app/third_party/scenario_runner/srunner /venv/lib/python3.8/site-packages/srunner
# needed to find the example scenarios
#ENV SCENARIO_RUNNER_ROOT=/app/third_party/scenario_runner
ENV SCENARIO_RUNNER_ROOT=/app/src/pedestrians_scenarios/scenarios

FROM scenarios-cpu AS scenarios-nvidia
# add carla roach + leaderboard - only for GPU builds
ENV torch_version=1.9.1
ENV torchvision_version=0.10.1
RUN /venv/bin/python -m pip install --no-cache-dir -f https://download.pytorch.org/whl/torch_stable.html \
    torch==${torch_version}+cu111 \
    torchvision==${torchvision_version}+cu111
RUN /venv/bin/python -m pip install --no-cache-dir \
    dictor \
    pygame \
    requests

RUN ln -s /app/third_party/carla_roach/carla_gym /venv/lib/python3.8/site-packages/carla_gym
RUN ln -s /app/third_party/carla_roach/agents /venv/lib/python3.8/site-packages/roach_agents
RUN ln -s /app/third_party/leaderboard/leaderboard/ /venv/lib/python3.8/site-packages/leaderboard 

# get the correct "build" branch for $PLATFORM
FROM scenarios-${PLATFORM} as scenarios

# Copy client files so that we can do editable pip install
COPY --chown=${USERNAME}:${USERNAME} . /app