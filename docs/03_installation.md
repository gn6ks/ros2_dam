# 3. Installation and Environment Setup

This chapter details the procedures required to establish a robust development environment for ROS2. The setup supports two primary configurations: **Native Ubuntu 24.04 LTS** for optimal performance, or **Windows Subsystem for Linux 2 (WSL 2)** for users operating on Windows 11. Both configurations ensure compatibility with **ROS 2 Jazzy Jalisco**. Proper configuration of the environment, including shell initialization and editor integration, is critical for streamlined development and simulation workflows.

---

## 3.1 Environment Selection

Before proceeding, identify your current operating system and select the appropriate installation path:

| Current System | Recommended Approach | Reference Section |
|----------------|---------------------|-------------------|
| **Windows 11** | WSL 2 with Ubuntu 24.04 LTS | Section 3.2 |
| **Ubuntu 24.04 LTS (Native)** | Direct ROS2 installation | Section 3.3 |
| **Ubuntu 22.04 or older** | Upgrade to 24.04 or use virtual machine | Section 3.3 |

> **Technical Note:** ROS2 Jazzy Jalisco requires Ubuntu 24.04 LTS (Noble Numbat). Earlier Ubuntu versions are not compatible with this distribution.

---

## 3.2 Windows 11: WSL 2 Setup

For users operating on Windows 11, WSL 2 provides a full Linux kernel interface, enabling native performance for ROS2 nodes and simulation tools.

### 3.2.1 WSL Installation

To enable WSL and install the default Ubuntu distribution, execute the following command in a **Windows PowerShell** terminal with Administrator privileges:

```powershell
wsl --install
```

> **Technical Note:** If WSL is already installed but requires a specific version, use `wsl --set-default-version 2`. Ensure virtualization is enabled in the system BIOS/UEFI settings.

### 3.2.2 Ubuntu 24.04 LTS Configuration

ROS 2 Jazzy is specifically targeted at Ubuntu 24.04 (Noble Numbat). If the default installation provides a different version, install Ubuntu 24.04 explicitly:

```powershell
wsl --install -d Ubuntu-24.04
```

Upon first launch, configure the user credentials. Once logged in, update the package repository to ensure system stability:

| Command | Description |
| --- | --- |
| `sudo apt update` | Refreshes the package index from repositories. |
| `sudo apt upgrade -y` | Installs the latest versions of installed packages. |

### 3.2.3 WSL Configuration for GUI Applications

To support graphical applications such as RViz2 and Gazebo, WSLg (WSL GUI) must be enabled. On Windows 11, WSLg is enabled by default. Verify the configuration:

```powershell
# In Windows PowerShell
wsl --list --verbose
```

Ensure the Ubuntu instance shows version `2`. If not, convert it:

```powershell
wsl --set-version Ubuntu-24.04 2
```

---

## 3.3 Native Ubuntu 24.04 LTS Setup

For users already running Ubuntu 24.04 LTS natively (dual-boot or dedicated machine), proceed with the direct ROS2 installation.

### 3.3.1 System Verification

Verify that your system meets the requirements:

```bash
# Check Ubuntu version
lsb_release -a

# Expected output: Ubuntu 24.04 LTS (Noble Numbat)

# Check available disk space
df -h /

# Recommended: Minimum 30 GB free space
```

### 3.3.2 System Update

Before installing ROS2, ensure your system is fully updated:

```bash
sudo apt update
sudo apt upgrade -y
```

---

## 3.4 ROS 2 Jazzy Installation

The installation process involves configuring locale settings, adding ROS2 repositories, and installing the core desktop packages. This procedure is identical for both WSL 2 and native Ubuntu 24.04 environments.

### 3.4.1 Locale Configuration

ROS2 requires UTF-8 locale support for proper string handling and logging.

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.4.2 Repository Setup

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

### 3.4.3 Package Installation

Install the `desktop` metapackage, which includes core ROS2 libraries, tools (RViz2, Gazebo), and communication middleware.

```bash
sudo apt install ros-jazzy-desktop
```

> **Installation Time:** This process may take 20-40 minutes depending on internet connection speed.

---

## 3.5 Environment Configuration (.bashrc)

To avoid manually sourcing the ROS2 setup script in every new terminal session, the environment configuration is persisted in the user's shell initialization file.

### 3.5.1 Persistent Sourcing

Append the ROS2 setup script to the `~/.bashrc` file. This ensures that every interactive bash shell automatically loads ROS2 environment variables.

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 3.5.2 Verification

Apply the changes to the current session and verify the configuration.

```bash
source ~/.bashrc
echo $ROS_DISTRO
```

> **Expected Output:** The terminal should return `jazzy`. If no output is displayed, the sourcing step failed.

---

## 3.6 Installation Methods

Users may choose between automated script-based installation or manual step-by-step configuration.

### 3.6.1 Automated Installation (Recommended)

For users who wish to automate the complete ROS2 installation and environment configuration, automation scripts are provided in the repository.

#### Execution

```bash
# Navigate to the scripts directory
cd scripts/

# Grant execution permissions (only required once)
chmod +x setup_ros2_environment.sh

# Execute the setup script
./setup_ros2_environment.sh
```

#### What the Script Does

| Step | Action |
|------|--------|
| 1 | Verifies Ubuntu 24.04 LTS and system prerequisites |
| 2 | Updates system packages |
| 3 | Configures UTF-8 locale settings |
| 4 | Adds ROS2 Jazzy repositories |
| 5 | Installs ros-jazzy-desktop and additional tools |
| 6 | Configures ~/.bashrc for persistent environment sourcing |
| 7 | Verifies installation and displays summary |

### 3.6.2 Manual Installation (Step-by-Step)

For users who prefer explicit control over each installation step, follow the procedures outlined in Sections 3.2 through 3.5. This approach is recommended for:

- Understanding the underlying configuration process
- Troubleshooting installation issues
- Customizing the installation for specific requirements
- Educational purposes

---

## 3.7 Post-Installation Verification

After completing the installation (automated or manual), verify that ROS2 Jazzy is properly configured.

### 3.7.1 Quick Verification Script

```bash
# Navigate to scripts directory
cd scripts/

# Run verification script
./verify_ros2_installation.sh
```

### 3.7.2 Manual Verification

```bash
# Check ROS2 version
ros2 --version

# Check ROS distribution
echo $ROS_DISTRO

# List available packages
ros2 pkg list | head -10

# Check middleware implementation
ros2 doctor --report | grep "RMW Implementation"
```

**Expected Output:**

```
ROS2 Version: ros2 0.32.x
ROS_DISTRO: jazzy
RMW Implementation: rmw_fastrtps_cpp
```

---

## 3.8 Additional Development Tools

The following tools are recommended for efficient ROS2 development.

### 3.8.1 Colcon Build System

```bash
sudo apt install python3-colcon-common-extensions
```

### 3.8.2 ROS Dependency Manager

```bash
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
```

### 3.8.3 Visualization and Simulation Tools

```bash
# RQT and plugins
sudo apt install ros-jazzy-rqt ros-jazzy-rqt-common-plugins

# Gazebo (if not included in desktop package)
sudo apt install gazebo11 libgazebo11-dev
```

## 3.9 Troubleshooting

### 3.9.1 General Troubleshooting

```bash
# Re-source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Check environment variables
env | grep ROS

# Diagnose ROS2 issues
ros2 doctor
```
