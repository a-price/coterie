# Constructs a testing environment for our set libraries

#-----------------------------------------------------
# Private Repo Aquisition
# * Takes a provided ssh key to clone private repos
# * The actual image copies the cloned repos
# * This image and the copied ssh keys are not exposed
#-----------------------------------------------------

# Begin with a base build
FROM ros:%ROS_VERSION% as getter

# Install missing debian packages
RUN apt-get -yq update && \
    DEBIAN_FRONTEND=noninteractive apt-get -yqq install \
    ssh \
    git && \
    rm -rf /var/lib/apt/lists/*

# Set home directory
ENV HOME /root

# Inject credentials at build time
ARG SSH_PRIVATE_KEY
RUN mkdir /root/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/*

# Get keys for target host
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan atas-gitb.gtri.gatech.edu >> /root/.ssh/known_hosts

# Set up ROS workspace and clone prerequisites
RUN cd && \
    mkdir -p catkin_ws/src
ADD ./.rosinstall /root/catkin_ws/src/.rosinstall
RUN cd ~/catkin_ws/src && wstool status && wstool up

#-----------------------------------------------------
# Main image
# * Uses cloned artifacts from "getter"
#-----------------------------------------------------

# Begin with a base build
FROM ros:%ROS_VERSION%

# Set home directory
ENV HOME /root

# Set bash as sh
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Set up ROS workspace and clone sources
RUN cd && \
    mkdir -p catkin_ws/src && cd catkin_ws/src && \
    source /opt/ros/%ROS_VERSION%/setup.bash && \
    catkin_init_workspace 

# Copy over the cloned source dependencies and package for rosdep
# * This keeps the apt stuff from getting called every time any file in this repo is changed.
COPY --from=getter /root/catkin_ws/src /root/catkin_ws/src/
RUN mkdir -p ~/catkin_ws/src/coterie/coterie_msgs
RUN mkdir -p ~/catkin_ws/src/coterie/coterie
ADD ./coterie_msgs/package.xml /root/catkin_ws/src/coterie/coterie_msgs/
ADD ./coterie/package.xml /root/catkin_ws/src/coterie/coterie/

# Install missing rosdeps
RUN apt-get -yq update && \
    DEBIAN_FRONTEND=noninteractive apt-get -yqq upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get -yqq install \
    ssh \
    git \
    python-pip \
    ccache && \
    cd ~/catkin_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Enable cache for repeated builds
RUN export PATH=/usr/lib/ccache:$PATH

# Install cvxpy tools
RUN pip install --upgrade pip cvxpy cvxopt

ADD ./ /root/catkin_ws/src/coterie

# Build and test the project
RUN cd ~/catkin_ws && \
    source /opt/ros/`rosversion -d`/setup.bash && \
    catkin_make && \
    source ~/catkin_ws/devel/setup.bash && \
    catkin_make run_tests && \
    catkin_test_results
