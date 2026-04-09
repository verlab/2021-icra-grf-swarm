# Reproducible environment for IEEE ICRA 2021 swarm segregation experiments (ROS Melodic).
# Build: docker build -t icra2021-grf-swarm .
# Run (headless simulation): docker run --rm icra2021-grf-swarm
# GUI requires X11 forwarding (see project page).

FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

# Multiple COPY sources into one directory merge and overwrite files; each package must
# live under src/<package_name>/ so rospack finds grf_swarm, grad_swarm, etc.
RUN mkdir -p /catkin_ws/src && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd /catkin_ws/src && catkin_init_workspace ."

COPY grf_swarm /catkin_ws/src/grf_swarm/
COPY grad_swarm /catkin_ws/src/grad_swarm/
COPY min_swarm /catkin_ws/src/min_swarm/
COPY pso_swarm /catkin_ws/src/pso_swarm/
COPY vgs_swarm /catkin_ws/src/vgs_swarm/

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd /catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release"

RUN echo 'source /opt/ros/melodic/setup.bash\nsource /catkin_ws/devel/setup.bash' >> /root/.bashrc

# Default: GRF swarm demo without OpenCV GUI (suitable for servers/CI)
CMD ["/bin/bash", "-lc", "source /opt/ros/melodic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch grf_swarm grf_swarm.launch gui:=false log:=true"]
