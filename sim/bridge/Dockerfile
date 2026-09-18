FROM robonix-ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip ros-humble-nav2-msgs curl \
    libgl1 libegl1 libglfw3 libx11-6 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    libgl1-mesa-dri mesa-utils && \
    rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir --timeout 120 \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
    websockets==15.0.1 numpy==1.26.4 Pillow==10.4.0 grpcio==1.78.0 protobuf==6.33.6 \
    requests==2.32.5 "fastmcp>=3,<4" mujoco==3.3.7
RUN python3 -m pip install --no-cache-dir --timeout 120 \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple onnxruntime==1.23.2

WORKDIR /workspace
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && exec python3 sim/bridge/bridge_node.py"]
