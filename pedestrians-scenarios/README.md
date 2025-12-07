# Pedestrians Scenarios

This is a part of the bigger project to bring the more realistic pedestrian movements to CARLA.
It isn't intended for fully standalone use. Please see the [main project README.md](https://github.com/wielgosz-info/carla-pedestrians/blob/main/README.md) or [Adversarial Cases for Autonomous Vehicles (ARCANE) project website](https://project-arcane.eu/) for details.

## Setup

Copy the `.env.template` file to `.env` and edit the values as required.

## Available options

Run `python -m pedestrians_scenarios --help` to see the full list of available options.

### `datasets`

This command is the entrypoint for dataset-generation related tasks.

```sh
python -m pedestrians_scenarios datasets --help
```

#### `datasets generate`

This command generates the dataset according to specified parameters.

```sh
python -m pedestrians_scenarios datasets generate <options>
```

##### Generic options

- `-h`, `--help` show help message and exit. Please use this for full list of available options.
- `-c CONFIG`, `--config CONFIG` Config file path. Settings in this file will override those passed via CLI


##### Selected server-related options

-  `--seed SEED` Seed used by the everything (default: 22752). If you want to generate a dataset with the same options, but in multiple shots, you need to manually change the seed with each run. You should ensure the seed differs by at least `number_of_clips * failure_multiplier` between runs.
-  `--fps FPS` FPS of the simulation (default: 30)

##### Selected generator options
- `--outputs_dir OUTPUTS_DIR` Directory to store outputs (default: ./datasets).
- `--number_of_clips NUMBER_OF_CLIPS` Total number of clips to generate (default: 512).
- `--clip_length_in_frames CLIP_LENGTH_IN_FRAMES`  Length of each clip in frames (default: 900).
- `--batch_size BATCH_SIZE` Number of clips in each batch (default: 1). This is the number of scenarios that will be simulated in the same world at the same time. This is useful for speeding up the generation process, but it also means that the generated clips can contain pedestrians from different scenarios in the camera viewport.
- `--camera_fov CAMERA_FOV` Camera horizontal FOV in degrees (default: 90.0).
- `--camera_image_size CAMERA_IMAGE_SIZE` Camera image size in pixels as a (width, height) tuple (default: (1600,600)).
- `--waypoint_jitter_scale WAYPOINT_JITTER_SCALE` Scale of jitter applied to waypoints (default: 1.0). This determines how much the pedestrians will deviate from the "ideal" path. The higher the value, the more they will deviate.
- `--failure_multiplier FAILURE_MULTIPLIER` Multiplier for number of clips to generate in case of failure (default: 2). As generating a clip is a stochastic process, it can fail (CARLA server shuts down, pedestrian is stuck, something obstructs the view of the pedestrian during the whole clip etc.). This option determines how many tries in total the process will make before giving up. The total number of tries is `number_of_clips * failure_multiplier`.
- `--overwrite` Overwrite existing output dir (default: False). As a default, if generator detects that the output directory already exists, it will abort the process. This option forces the generator to overwrite the existing directory. **Please note that this will override the files in the directory, possibly resulting in data loss.** 
```

Example command can look like this:

```sh
python -m pedestrians_scenarios datasets generate \
    --outputs_dir /outputs/scenarios/small-sample \
    --number_of_clips 16 \
    --failure_multiplier 8
```

#### `datasets merge`

Allows to merge multiple datasets into one, ensuring that the clip IDs are unique.

```sh
python -m pedestrians_scenarios datasets merge --input_dirs <list_of_input_dirs> --output_dir <output_dir>
```

#### `datasets clean`

Especially in the older versions of the generator, there were many output videos which did not contain pedestrians. Therefore, we developed an automated cleaning procedure that checks if pedestrian is visible at any point in the video using YOLOv3. A usage example:

```sh
python -m pedestrians_scenarios datasets clean --dataset_dir /outputs/dataset-dir
```

The command above will run in a dry mode (the files will not be removed). If you actually want to remove them run the command with `--remove` flag:

```sh
python -m pedestrians_scenarios datasets clean --dataset_dir /outputs/dataset-dir --remove
```

Keep in mind that the video files which do not contain pedestrians in the folders you provide as the arguments will be removed from disk. The csv file will also be changed by removing rows which refer to videos which do not contain pedestrians.

**Note: The cleaning procedure is not perfect. It may remove some videos which contain pedestrians or leave some videos which do not contain pedestrians. It is also quite slow. Therefore, it is better to not disable semantic cameras during dataset generation and take advantage of the (now) build-in filtering.**

## Running original ScenarioRunner

To launch the basic `ScenarioRunner`, run the following (example) command:
```sh
cd /app/third_party/scenario_runner/
python scenario_runner.py --host server --scenario FollowLeadingVehicle_1 --reloadWorld --output
# OR
python scenario_runner.py --host server --route ./srunner/data/routes_debug.xml ./srunner/data/all_towns_traffic_scenarios.json 0 --agent srunner/autoagents/npc_agent.py --output --reloadWorld
```
The really important part is the `--host` parameter. We pass it the name of the carla server container
as specified in our `docker-compose.yml` file.

### Visualization (only makes sense when running the original ScenarioRunner)

Since all of the code is running inside a docker container, to see that is happening we would need to
either setup some X11 forwarding magic (no, thank you) or approach it in a different way (yes, please).
To see what's happening we're going to use the `carlaviz` tool, which lets to preview the ego vehicle in the browser.
Now, there are basically three configurations to choose from:

1. You are running everything on your own local machine. In this case, you need to ensure the `carlaviz` ports are bound to correct ports on your machine, by editing the `carlaviz/.env` file:
```sh
# this can actually be any port, you will use it in the browser, e.g. http://localhost:8080
CARLAVIZ_FRONTEND_MAPPED_PORT=8080

# this needs to be exactly $CARLAVIZ_BACKEND_PORT (8081 by default), as it is used by the carlaviz frontend
CARLAVIZ_BACKEND_MAPPED_PORT=8081
```

2. You are running everything on a remote server, but you can publish the carlaviz backend port. In this case, you need to ensure the `carlaviz` ports are bound to correct ports on the remote server, by editing the `carlaviz/.env` file:
```sh
CARLAVIZ_BACKEND_HOST=your_server_ip_or_hostname
```
If you change ports from default 8081, please ensure CARLAVIZ_BACKEND_MAPPED_PORT and CARLAVIZ_BACKEND_PORT match.

3. You are running everything on a remote server and you need to go through the access node. In this case, you need to ensure the `carlaviz` ports are bound to 'intermediate' ports on the remote server, and that you have the forwarding set up and running. First, editing the `carlaviz/.env` file to set the intermediate port numbers:
```sh
# numbers can be whatever you like, just ensure they match with forwarding config
CARLAVIZ_FRONTEND_MAPPED_PORT=49164  
CARLAVIZ_BACKEND_MAPPED_PORT=49165
```

Then, you need to set up the forwarding on your local machine. In your `.shh/config` file, add something like this:
```ssh-config
Host forward_carlaviz
    HostName remote_server_domain_or_ip_as_visible_from_access_node
    LocalForward 8080 127.0.0.1:49164
    LocalForward 8081 127.0.0.1:49165
    User remote_server_username
    ProxyJump access_node_username@access_node_domain_or_ip  # skip this if there is no access node
    ServerAliveInterval 30
    ServerAliveCountMax 3
```

And finally, you need to run the following command on your local machine once the `carla-pedestrians_viz_1` container is running (`-v` is optional):
```sh
ssh -v -N forward_carlaviz
```

**Important note: if you need to restart the carlaviz container, kill the forwarding first, then restart carlaviz and wait for it to be running, and only then start forwarding again. Otherwise you will get stuck with "Launch the backend and refresh" message in the browser.**


## Cite
If you use this repo please cite:

```
@misc{wielgosz2023carlabsp,
      title={{CARLA-BSP}: a simulated dataset with pedestrians}, 
      author={Maciej Wielgosz and Antonio M. López and Muhammad Naveed Riaz},
      month={May},
      year={2023},
      eprint={2305.00204},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```


## Funding

|                                                                                                                              |                                                                                                                      |                                                                                                                                                                                                                                                                                                                                                                                      |
| ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img src="docs/_static/images/logos/Logo Tecniospring INDUSTRY_white.JPG" alt="Tecniospring INDUSTRY" style="height: 24px;"> | <img src="docs/_static/images/logos/ACCIO_horizontal.PNG" alt="ACCIÓ Government of Catalonia" style="height: 35px;"> | <img src="docs/_static/images/logos/EU_emblem_and_funding_declaration_EN.PNG" alt="This project has received funding from the European Union's Horizon 2020 research and innovation programme under Marie Skłodowska-Curie grant agreement No. 801342 (Tecniospring INDUSTRY) and the Government of Catalonia's Agency for Business Competitiveness (ACCIÓ)." style="height: 70px;"> |


<!-- pyscaffold-notes -->

## Note

This project has been set up using PyScaffold 4.1.1. For details and usage
information on PyScaffold see https://pyscaffold.org/.
