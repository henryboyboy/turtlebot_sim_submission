#!/usr/bin/env bash

echo -e "Building turtlebot_sim:lastest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--tag turtlebot_sim:latest .
