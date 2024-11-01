#!/bin/bash

dataset='PIE'
limit=360

cd /openpose # it needs to be run there to find 'models' dir

mkdir -p  /app/outputs/${dataset}
i=1

find /app/datasets -type f -name "*.mp4" | while read -r filename; do
#for filename in ./datasets/${dataset}/videos/**/*.mp4; do
    # filename: /app/datasets/PIE/videos/set05/video_0002.mp4
    # dirname: 去除文件名中的非目录部分，仅显示与目录有关的内容
    set_name=$(dirname "$filename")  # /app/datasets/PIE/videos/set05
    # basename: 剔除目录部分后的 显示文件名，同时也删除文件的扩展名
    set_name=$(basename "$set_name")  # set05
    mkdir -p  /app/outputs/${dataset}/${set_name}  # 新建放置处理视频之后结果的文件夹
    
    name=$(basename "$filename" .mp4)
    echo "Processing ${set_name}/${name}..."  # Processing set05/video_0002...
    ./build/examples/openpose/openpose.bin \
        --video ${filename} \
        --write_json /app/outputs/${dataset}/${set_name}/${name} \
        --model_pose BODY_25 \
        --display 0 \
        --render_pose 0
        # --hand
        # --write_video /outputs/${dataset}/${name}.avi
    ((i=i+1))
    if [ $i -gt $limit ]; then
        break
    fi
    # echo ${set_name}
    # exit 0
done
