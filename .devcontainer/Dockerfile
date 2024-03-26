# Start with a base image
FROM ubuntu:20.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive


ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /workspaces

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p -m 0700 /run/user/"${USER_UID}" \
    && mkdir -p -m 0700 /run/user/"${USER_UID}"/gdm \
    && chown user:user /run/user/"${USER_UID}" \
    && chown user:user /workspaces \
    && chown user:user /run/user/"${USER_UID}"/gdm \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install necessary packages
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    clang-6.0 \
    clang-format-6.0 \
    clang-tidy-6.0 \
    cmake \
    doxygen \
    graphviz \
    lcov \
    libeigen3-dev \
    libpoco-dev \
    rename \
    valgrind \
    lsb-release \
    dpkg \
    git \
    libeigen3-dev \
    libpoco-dev \
    python3-dev \
    python3-pip \
    liburdfdom-headers-dev \
    libconsole-bridge-dev \
    libtinyxml2-dev \
    lsb-release curl \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install -qqy lsb-release curl
RUN mkdir -p /etc/apt/keyrings 
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | tee /etc/apt/keyrings/robotpkg.asc

RUN  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list

RUN apt-get update \
    && apt-get install -qqy robotpkg-pinocchio

USER $USERNAME
