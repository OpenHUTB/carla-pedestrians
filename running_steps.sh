# 第0步
cp openpose/.env.template openpose/.env
cp pedestrians-common/.env.template pedestrians-common/.env
cp pedestrians-video-2-carla/.env.template pedestrians-video-2-carla/.env
cp pedestrians-scenarios/.env.template pedestrians-scenarios/.env

# 第1步
cd openpose
docker-compose -f "docker-compose.yml" --env-file .env up -d --build
docker exec -it carla-pedestrians_openpose_1 /bin/bash
