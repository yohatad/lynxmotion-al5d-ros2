# Robotics: Principles and Practice -- ROS 2 Jazzy (Ubuntu 24.04), Gazebo Harmonic.
#
#   docker build -t rpp-jazzy .
#   docker run --rm rpp-jazzy test        # run the whole test suite
#   docker run -it rpp-jazzy              # interactive shell
#
FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG LIBCREATE_REF=116be443e7970de1574b5dc5f91e414828854c08
ARG CREATE_ROBOT_REF=ffe8aecda94d2f152d36d60de5d05ab7da6eded5
ARG MAKE_JOBS=4
SHELL ["/bin/bash", "-c"]

# --- system packages that rosdep cannot provide ------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git sudo nano xvfb mesa-utils usbutils \
        libboost-system-dev libboost-thread-dev libncurses-dev \
        python3-colcon-common-extensions python3-pytest-cov \
    && rm -rf /var/lib/apt/lists/*

# --- libcreate (iRobot Create 2 driver library): no rosdep rule exists for Jazzy ---------------
RUN git clone https://github.com/AutonomyLab/libcreate.git /tmp/libcreate \
    && cd /tmp/libcreate && git checkout "${LIBCREATE_REF}" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cmake --build build -j"${MAKE_JOBS}" && cmake --install build \
    && ldconfig && rm -rf /tmp/libcreate

# --- non-root user (the ubuntu user of the base image is replaced) ------------------------------
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && useradd -m -u 1000 -s /bin/bash -G dialout,video,sudo rpp \
    && echo "rpp:rpp" | chpasswd \
    && echo "rpp ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/rpp

USER rpp
ENV HOME=/home/rpp
WORKDIR /home/rpp/workspace

# --- dependency resolution first, so source edits do not invalidate the slow layers --------------
RUN mkdir -p src/third_party && cd src/third_party \
    && git clone https://github.com/AutonomyLab/create_robot.git \
    && cd create_robot && git checkout "${CREATE_ROBOT_REF}"

COPY --chown=rpp:rpp src/lynxmotion_al5d_description/package.xml src/lynxmotion_al5d_description/package.xml
COPY --chown=rpp:rpp src/coro_examples/coro_common/package.xml src/coro_examples/coro_common/package.xml
COPY --chown=rpp:rpp src/coro_examples/module2/package.xml src/coro_examples/module2/package.xml
COPY --chown=rpp:rpp src/coro_examples/module3/package.xml src/coro_examples/module3/package.xml
COPY --chown=rpp:rpp src/coro_examples/module4/package.xml src/coro_examples/module4/package.xml
COPY --chown=rpp:rpp src/coro_examples/module5/package.xml src/coro_examples/module5/package.xml

RUN sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys libcreate \
    && sudo rm -rf /var/lib/apt/lists/*

# --- build ------------------------------------------------------------------------------------
COPY --chown=rpp:rpp src/ src/
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --event-handlers console_cohesion- \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

COPY --chown=rpp:rpp scripts/entrypoint.sh /home/rpp/entrypoint.sh
COPY --chown=rpp:rpp scripts/run_tests.sh /home/rpp/run_tests.sh
RUN chmod +x /home/rpp/entrypoint.sh /home/rpp/run_tests.sh \
    && echo "source /home/rpp/workspace/install/setup.bash" >> /home/rpp/.bashrc

ENV QT_QPA_PLATFORM=xcb
ENTRYPOINT ["/home/rpp/entrypoint.sh"]
CMD ["bash"]
