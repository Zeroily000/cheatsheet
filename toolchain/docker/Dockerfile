FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    bash-completion \
    python3 \
    gcc \
    g++ \
    make \
    gdb \
    lldb \
    git \
    locales \
    x11-apps

# Install Bazel
RUN apt-get update && apt-get install -y \
    apt-transport-https \
    curl \
    gnupg
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg
RUN mv bazel-archive-keyring.gpg /usr/share/keyrings
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
RUN apt-get update && apt-get install -y bazel

# Install OpenCV
RUN apt-get update && apt-get install \
    libcanberra-gtk3-module \
    libopencv-dev

# Create user with sudo privileges
RUN apt-get update && apt-get install -y \
    sudo
RUN useradd -ms /bin/bash user
RUN usermod -aG sudo user
RUN echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER user
