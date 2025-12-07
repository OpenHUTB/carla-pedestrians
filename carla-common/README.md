# carla-common

Minimal Docker container that can be used for developing Python code using CARLA 0.9.13 Python API.

Developed as a part of [Adversarial Cases for Autonomous Vehicles (ARCANE) project](https://project-arcane.eu/).

Run it like this (includes CARLA server):
```sh
COMMIT=$(git rev-parse --short HEAD) USER_ID=$(id -u) GROUP_ID=$(id -g) \
docker-compose -f docker-compose.yml up -d --build
```

`.env` file contains server config variables.

In your code you can then use `server` when specifying the CARLA server hostname in `carla.Client`.

## Some explanations

It is assumed that the code you are developing will live in the `/app` directory.
Preferably, it would also be an installable package (hence the `PACKAGE` variable),
e.g. generated with [PyScaffold](https://pyscaffold.org/en/stable/). If that is not the case,
please remove the `PACKAGE` env altogether.

In general, you should have your own package repo and add this one as a submodule
(or leave it out altogether). The recommended way is to use the carla-common image
as a base image in your `FROM` line (see [Writing your own Dockerfile](#writing-your-own-dockerfile)).

Git, basic compliers and common video (ffmpeg) and image libraries are installed by default.
However, git credentials are not stored in the container, so you will need to provide them
when you run `git` commands. In the future, a more lightweight image may be created,
however, those tools were needed so often that we decided to keep them for now.

Also, we have thrown-in some enhancements to simplify working with VS Code
(e.g. volumes that store the remote VS Code or some additional Python packages).
It should be pretty obvious which parts are that.

## Writing your own Dockerfile

```Dockerfile
ARG PLATFORM=nvidia
FROM wielgoszinfo/carla-common:${PLATFORM}-latest AS base

ENV PACKAGE=your-package-name

# anything you need here, e.g.
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    && rm -rf /var/lib/apt/lists/*
USER ${USERNAME}

RUN /venv/bin/python -m pip install --no-cache-dir \
    numpy==1.22.1
# ...

# Copy client files so that we can do editable pip install
COPY --chown=${USERNAME}:${USERNAME} . /app

# Please do not set the WORKDIR (it is set to /home/carla by default).
# Alternatively, you can overwrite the ENTRYPOINT so it can find entrypoint.sh
# (it is set to [ "./entrypoint.sh" ] by default)
```
