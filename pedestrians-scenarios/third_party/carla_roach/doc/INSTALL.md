# Installation

We use AWS EC2 instances, but you can also setup the environment and run all experiments on your own computer or cluster.

## Pre-installation
- (Optional) Launch an AWS EC2 g4dn.xlarge instance with Deep Learning Base AMI (Ubuntu 18.04) and ssh to it.
- Install [Miniconda](https://docs.conda.io/en/latest/miniconda.html).
- (Optional) Generate and add SSH key to your github account.


## Install CARLA
- Install [CARLA 0.9.13 release](https://github.com/carla-simulator/carla/releases/tag/0.9.13) with additional maps (do this inside your clone of this repo)
```bash
mkdir $(pwd)/carla
cd $(pwd)/carla
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz
tar -xvzf CARLA_0.9.13.tar.gz
echo "export CARLA_ROOT=$(pwd)/carla" >> ~/.bashrc  # Optional, can use the setup_roach0913.sh script instead
cd Import && wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.13.tar.gz
cd .. && bash ImportAssets.sh
rm CARLA_0.9.13.tar.gz Import/AdditionalMaps_0.9.13.tar.gz
cd && source .bashrc  # Optional
```

 - In addition, pull the CARLA 0.9.13 Docker image to run the data collection phase:
```bash
docker pull carlasim/carla:0.9.13
```

## Setup carla-roach
- Clone this repository and create conda environment.
```bash
git clone git@github.com:zhejz/carla-roach.git
cd carla-roach
conda env create -f environment.yml --name roach
conda activate roach
```
  - We avoid using `easy_install`, so instead, it's better to give the path of the CARLA `.egg`. We have prepared a bash script, so it can be run:
```bash
. setup_roach0913.sh
```
  This will setup the `CARLA_ROOT` and `PYTHONPATH` environment variables.

- We use [wandb](https://wandb.ai/site) for logging, please register a free account and login to it.
```bash
wandb login
```

## Post-installation
- Create an AMI for this instance.
- Create launch template for that AMI.

## For RL training (NOT THE FOCUS OF THIS FORK)
Our RL training crashed more often on CARLA 0.9.11 than on CARLA 0.9.10.1.
So for RL training we create an environment with the [CARLA 0.9.10.1 release](https://github.com/carla-simulator/carla/releases/tag/0.9.10.1). 
Get a new AWS instance, all installation steps remain the same, just the CARLA version is changed
```bash
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.1.tar.gz
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.10.1.tar.gz
```
and the filename of the CARLA library is different
```bash
easy_install ${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
```
You may want to create a specific AMI and launch template for this environment.
On your local machine, you may consider to install two CARLA and have two environment variable ${CARLA_ROOT_911} and ${CARLA_ROOT_910}.