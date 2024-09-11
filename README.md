# libfranka: C++ Library for Franka Robotics Research Robots

[![codecov][codecov-status]][codecov]

**libfranka** is a C++ library that provides low-level control of Franka Robotics research robots. The [generated API documentation][api-docs] offers an overview of its capabilities, while the [Franka Control Interface (FCI) documentation][fci-docs] provides more information on setting up the robot and utilizing its features and functionalities.

To find the appropriate version to use, please refer to the [Compatibility Matrix][compatibility-matrix].

## Key Features
- **Low-level control**: Access precise motion control for research robots.
- **Real-time communication**: Interact with the robot in real-time.

## Getting Started

### 1. System Requirements
Before using **libfranka**, ensure your system meets the following requirements:
- **Operating System**: [Linux with PREEMPT_RT patched kernel][real-time-kernel]  (Ubuntu 16.04 or later, Ubuntu 22.04 recommended)
- **Compiler**: GCC 7 or later
- **CMake**: Version 3.10 or later
- **Robot**: Franka Robotics robot with FCI feature installed

### 2. Installing dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    dpkg
```
To use libfranka version `0.14.0` or later, you will need to install [pinocchio][stack-of-tasks] and some more dependencies:

```bash
sudo apt-get install -y \
    liburdfdom-headers-dev \
    libconsole-bridge-dev \
    libtinyxml2-dev
```

```bash
sudo apt-get install -qqy lsb-release curl
```

```bash
mkdir -p /etc/apt/keyrings
```

```bash
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc
```

```bash
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list
```

```bash
sudo apt-get update \
    && sudo apt-get install -qqy robotpkg-pinocchio
```

### 3. Building and Installation from Source
Before building and installing from source, please uninstall existing installations of libfranka to avoid conflicts:

```bash
sudo apt remove "*libfranka*"
```
#### Clone the Repository
You can clone the repository and choose the version you need by selecting a specific tag:

```bash
# Clone the repository
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka

# List available tags
git tag -l

# Checkout a specific tag (e.g., 0.14.1)
git checkout 0.14.1

# Update submodules
git submodule update

# Create a build directory and navigate to it
mkdir build
cd build

# Configure the project and build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```

#### Installing libfranka as a Debian Package (Optional but recommended)

Building a Debian package is optional but recommended for easier installation and management. In the build folder, execute:

```bash
cpack -G DEB
```
This command creates a Debian package named libfranka-<version>-<architecture>.deb. You can then install it with:

```bash
sudo dpkg -i libfranka*.deb
```
Installing via a Debian package simplifies the process compared to building from source every time. Additionally the package integrates better with system tools and package managers, which can help manage updates and dependencies more effectively.


### 4. Usage
After installation, check the [Minimum System and Network Requirements][requirements] for network settings, the [Operating System and PC Configuration][real-time-kernel] for system setup, and the [Getting Started Manual][getting-started] for initial steps. Once configured, you can control the robot using the example applications provided in the examples folder.

To run a sample program, navigate to the build folder and execute the following command:

```bash
./examples/communication_test <robot-ip> 
```

## License

`libfranka` is licensed under the [Apache 2.0 license][apache-2.0].

[stack-of-tasks]: https://stack-of-tasks.github.io/pinocchio/download.html
[real-time-kernel]: https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel
[requirements]: https://frankaemika.github.io/docs/requirements.html
[getting-started]: https://frankaemika.github.io/docs/getting_started.html#
[compatibility-matrix]: https://frankaemika.github.io/docs/compatibility.html
[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[api-docs]: https://frankaemika.github.io/libfranka/0.15.0
[fci-docs]: https://frankaemika.github.io/docs
[codecov-status]: https://codecov.io/gh/frankaemika/libfranka/branch/master/graph/badge.svg
[codecov]: https://codecov.io/gh/frankaemika/libfranka
