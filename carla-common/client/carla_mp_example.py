import carla
import numpy.random as random
import time
import multiprocessing as mp
import glob
import os

def episode(episode_idx, results_queue):
    print(f"Episode: {episode_idx}")

    client = carla.Client('server', 2000)
    old_world = client.get_world()
    if old_world is not None:
        prev_world_id = old_world.id
        del old_world
    else:
        prev_world_id = None

    print("Load world:")
    client.load_world('Town01')

    print("Get world:")
    tries = 3
    world = client.get_world()
    while prev_world_id == world.id and tries > 0:
        tries -= 1
        time.sleep(1)
        world = client.get_world()

    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    spawn_points = world.get_map().get_spawn_points()

    print("Create actors: ")
    for i in range(0,10):
        world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

    time.sleep(15)

    print("Destroy actors: ")
    #destroy all vehicles
    for vehicle in world.get_actors().filter('*vehicle*'):
        vehicle.destroy()

    del world
    del client

    results_queue.put(1)
    print(f"Episode {episode_idx} success!")

def main():
    results_queue = mp.Queue()
    server_failed = 0
    episode_idx = 0

    for i in range(10000):
        p = mp.Process(target=episode, args=(episode_idx,results_queue))
        p.start()
        p.join()

        if results_queue.empty():
            print(f'Process failed for episode {episode_idx}')
            server_failed += 1
            # try to remove 'core.*' files
            for core_file in glob.glob(os.path.join(os.getcwd(), 'core.*')):
                os.remove(core_file)
            # assume that the server will restart
            time.sleep(float(os.getenv('CARLA_SERVER_START_PERIOD', '30.0')))
            continue
        else:
            # empty the queue
            results_queue.get()
            episode_idx += 1
        
    print(f"Success! Server failed: {server_failed}")

if __name__ == '__main__':
    main()