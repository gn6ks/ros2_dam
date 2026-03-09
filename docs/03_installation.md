# 3. Installation and Environment Setup

This chapter details the procedures required to establish a robust development environment for ROS2. The setup leverages **Windows Subsystem for Linux 2 (WSL 2)** to run **Ubuntu 22.04 LTS**, ensuring compatibility with **ROS 2 Humble Hawksbill**. Proper configuration of the environment, including shell initialization and editor integration, is critical for streamlined development and simulation workflows.

## 3.1 Prerequisites: WSL 2 and Ubuntu 22.04

Before installing ROS2 packages, the underlying Linux environment must be configured within Windows. WSL 2 provides a full Linux kernel interface, enabling native performance for ROS2 nodes and simulation tools.

### 3.1.1 WSL Installation

To enable WSL and install the default Ubuntu distribution, execute the following command in a **Windows PowerShell** terminal with Administrator privileges:

```powershell
wsl --install
```

> **Technical Note:** If WSL is already installed but requires a specific version, use `wsl --set-default-version 2`. Ensure virtualization is enabled in the system BIOS/UEFI settings.

### 3.1.2 Ubuntu 22.04 LTS Configuration

ROS 2 Humble is specifically targeted at Ubuntu 22.04 (Jammy Jellyfish). If the default installation provides a different version, install Ubuntu 22.04 explicitly:

```powershell
wsl --install -d Ubuntu-22.04
```

Upon first launch, configure the user credentials. Once logged in, update the package repository to ensure system stability:

| Command | Description |
| --- | --- |
| `sudo apt update` | Refreshes the package index from repositories. |
| `sudo apt upgrade -y` | Installs the latest versions of installed packages. |

## 3.2 ROS 2 Humble Installation

The installation process involves configuring locale settings, adding ROS2 repositories, and installing the core desktop packages.

### 3.2.1 Locale Configuration

ROS2 requires UTF-8 locale support for proper string handling and logging.

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.2.2 Repository Setup

You will need to add the ROS 2 apt repository to your system. The **ros-apt-source** package provides keys and apt source configuration for the various ROS repositories. Installing this package will configure ROS 2 repositories for your system, and updates to repository configuration will occur automatically when new versions of this package are released.

#### Step 1: Enable Universe Repository

First ensure that the Ubuntu Universe repository is enabled:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

#### Step 2: Download and Install ros-apt-source

The following commands download the latest release of the ros-apt-source package and install it:

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

> **Technical Note:** This method is preferred over the legacy `wget ros.key | apt-key add` approach, as apt-key is deprecated. The ros-apt-source package ensures automatic repository configuration updates.

#### Step 3: Update Package Index

After installing the repository configuration, update the package index:

```bash
sudo apt update
sudo apt upgrade
```

### 3.2.3 Package Installation

Install the `desktop` metapackage, which includes core ROS2 libraries, tools (RViz, Gazebo classics), and communication middleware.

```bash
sudo apt install ros-humble-desktop
```

## 3.3 Environment Configuration (.bashrc)

To avoid manually sourcing the ROS2 setup script in every new terminal session, the environment configuration is persisted in the user's shell initialization file.

### 3.3.1 Persistent Sourcing

Append the ROS2 setup script to the `~/.bashrc` file. This ensures that every interactive bash shell automatically loads ROS2 environment variables.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3.3.2 Verification

Apply the changes to the current session and verify the configuration.

```bash
source ~/.bashrc
echo $ROS_DISTRO
```

> **Expected Output:** The terminal should return `humble`. If no output is displayed, the sourcing step failed.

## 3.4 Automated Environment Setup

For users who wish to automate the complete ROS2 installation and environment configuration, an automation script is provided in the repository.

### 3.4.1 Execution

```bash
# Navigate to the scripts directory
cd scripts/

# Grant execution permissions (only required once)
chmod +x setup_ros2_environment.sh

# Execute the setup script
./setup_ros2_environment.sh
```

### 3.4.3 Post-Installation Verification

After the script completes, verify your installation:

```bash
# Quick verification
./verify_ros2_installation.sh

# Or manually
source ~/.bashrc
ros2 --version
echo $ROS_DISTRO  # Should output: humble
```