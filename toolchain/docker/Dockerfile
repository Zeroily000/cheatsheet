FROM nvidia/cuda:12.6.2-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    bash-completion \
    python3 \
    gcc \
    g++ \
    make \
    gdb \
    lldb \
    git

# Setup GUI
RUN apt-get update && apt-get install -y \
    locales \
    libcanberra-gtk3-module \
    mesa-utils \
    x11-apps
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV NO_AT_BRIDGE=1

# Install Bazel
RUN apt-get update && apt-get install -y \
    apt-transport-https \
    curl \
    gnupg
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg; \
    mv bazel-archive-keyring.gpg /usr/share/keyrings; \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list; \
    apt-get update && apt-get install -y \
    bazel

# Install Buildifier
RUN apt-get update && apt-get install -y \
    wget
RUN wget https://github.com/bazelbuild/buildtools/releases/download/v7.3.1/buildifier-linux-amd64; \
    mv buildifier-linux-amd64 /usr/local/bin/; \
    ln -s /usr/local/bin/buildifier-linux-amd64 /usr/local/bin/buildifier

# Install OpenCV
RUN apt-get update && apt-get install -y \
    libopencv-dev

# Create user with sudo privileges
RUN apt-get update && apt-get install -y \
    sudo
RUN useradd -ms /bin/bash user; \
    usermod -aG sudo user; \
    echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER user
