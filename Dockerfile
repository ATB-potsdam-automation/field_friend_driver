ARG ROS_DISTRO=jazzy
FROM gitlab-extern.atb-potsdam.de:5050/am/ros/utils/ros_base_containers/${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG LIZARD_VERSION=v0.7.3
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    jq \
    python3-pip \
    unzip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages prompt_toolkit

WORKDIR /root/.lizard
RUN set -eux; \
    API="https://api.github.com/repos/zauberzeug/lizard/releases/tags/${LIZARD_VERSION}"; \
    ASSET_ID=$(curl -fsSL "$API" | jq -r '.assets[0].id'); \
    curl -fLJO -H "Accept: application/octet-stream" \
         "https://api.github.com/repos/zauberzeug/lizard/releases/assets/${ASSET_ID}"; \
    unzip *.zip; \
    rm *.zip

WORKDIR /workspace
COPY . /workspace/src/fieldfriend_driver
RUN rosdep update --rosdistro=${ROS_DISTRO} && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select fieldfriend_driver

COPY entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]