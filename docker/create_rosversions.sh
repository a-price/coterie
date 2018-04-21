#!/bin/bash
declare -a versionList=("kinetic" "lunar")

# Create the versioned dockerfiles
for v in "${versionList[@]}"; do
    sed "s/%ROS_VERSION%/$v/g" Dockerfile > Dockerfile_$v
done

cd ..
# Build and tag
for v in "${versionList[@]}"; do
    docker build -f docker/Dockerfile_$v -t kdr/test_coterie:$v --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" .
done
cd -
