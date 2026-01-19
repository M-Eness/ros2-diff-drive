# 1. Native ARM64 Base
FROM --platform=linux/arm64 ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 2. Temel Araçlar
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 3. Gazebo Fortress Resmi Reposu
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# 4. Paket Kurulumu (RViz YOK, X11 YOK, Foxglove VAR)
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3-gazebo \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joint-state-publisher \
    # Gazebo Sim + Bridge
    gz-fortress \
    ros-humble-ros-gz \
    # Görselleştirme Köprüsü (Kritik Paket)
    ros-humble-foxglove-bridge \
    # Robot Kontrol & Navigasyon
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # Geliştirme Araçları
    python3-colcon-common-extensions \
    git nano \
    && rm -rf /var/lib/apt/lists/*

# 5. Ortam Ayarları
WORKDIR /ros2_ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Kaynak dosyalarını otomatik yükle
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]