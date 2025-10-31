#!/bin/bash

# ==============================================================================
# 自动化脚本：在 Ubuntu 22.04 (Jammy Jellyfish) 上安装 ROS 2 Humble
#
# 功能:
#   1. 切换 APT 源为中科大镜像以加速下载。
#   2. 安装 ROS 2 Humble Desktop 完整版及其开发工具。
#   3. 配置 ROS 2 环境，使其在打开新终端时自动加载。
#   4. 安装 Nav2, SLAM-Toolbox 和 TurtleBot3 等常用功能包。
#
# 使用方法:
#   1. 保存此脚本为 setup_ros2_humble.sh
#   2. 给予执行权限: chmod +x setup_ros2_humble.sh
#   3. 运行脚本: ./setup_ros2_humble.sh
#      脚本中的 sudo 命令会提示您输入密码。
# ==============================================================================

# 设置脚本在遇到错误时立即退出，防止后续错误操作
set -e

# --- 函数定义：用于打印带颜色的信息 ---
print_info() {
    echo -e "\n\e[32m[INFO] ==> $1\e[0m"
}

print_warning() {
    echo -e "\e[33m[WARN] ==> $1\e[0m"
}

print_success() {
    echo -e "\n\e[1;32m[SUCCESS] ==> $1\e[0m"
}


# === 1. 配置 APT 软件源为国内镜像 (中科大) ===
print_info "开始配置 APT 软件源为中科大镜像..."
# 备份原始的 sources.list 文件，以防需要恢复
sudo mv /etc/apt/sources.list /etc/apt/sources.list.bak
# 从 repogen (USTC 源) 下载为 Ubuntu 22.04 (Jammy) 准备的源列表文件
sudo curl -o /etc/apt/sources.list https://mirrors.ustc.edu.cn/repogen/conf/ubuntu-https-4-jammy
print_info "APT 源配置完成。"


# === 2. 安装系统基础工具 ===
print_info "安装系统基础工具 (VMware tools, software-properties-common)..."
# 安装 open-vm-tools，用于改善在 VMware 虚拟机中的体验（如剪贴板共享、屏幕自适应）
sudo apt install open-vm-tools open-vm-tools-desktop openssh-server -y
# 安装 software-properties-common，它提供了 add-apt-repository 命令
sudo apt install software-properties-common -y
# 启用 universe 软件源，ROS 2 的某些依赖包可能需要它
sudo add-apt-repository universe -y


# === 3. 添加 ROS 2 官方源 ===
print_info "正在添加 ROS 2 官方软件源..."
# 首先更新包列表，并确保 curl 已安装，因为后续步骤需要用到
sudo apt update && sudo apt install curl -y

# 动态获取最新的 ros-apt-source 版本号，避免硬编码
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
print_info "检测到最新的 ros-apt-source 版本为: ${ROS_APT_SOURCE_VERSION}"

# 下载对应版本的 ros2-apt-source deb 包，该包会自动为系统配置好 ROS 2 的 APT 源
curl -L -o /tmp/ros2-apt-source.deb "https://gh.321122.xyz/github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

# 使用 dpkg 安装下载的 deb 包
sudo dpkg -i /tmp/ros2-apt-source.deb
# 安装后删除临时文件
rm /tmp/ros2-apt-source.deb
print_info "ROS 2 软件源添加成功。"


# === 4. 更新系统并安装 ROS 2 ===
print_info "更新软件包列表并安装 ROS 2 Humble Desktop..."
# 添加新源后，必须再次更新软件包列表
sudo apt update
# 升级系统所有已安装的软件包，保持系统最新
sudo apt upgrade -y

# 安装 ros-dev-tools 包含 colcon 等 ROS 2 开发常用工具
# 安装 ros-humble-desktop 包含 ROS 2 核心库、RVIZ2、演示示例等所有桌面版内容
sudo apt install ros-dev-tools ros-humble-desktop -y
print_info "ROS 2 Humble Desktop 安装完成。"


# ===5. 安装额外的 ROS 2 功能包 (Nav2, SLAM, TurtleBot3) ===
print_info "安装额外的 ROS 2 功能包..."
# 安装 Nav2 核心包和启动文件
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
# 安装 TurtleBot3 仿真相关的包
sudo apt install ros-humble-turtlebot3-* -y
# 安装 SLAM 工具箱
sudo apt install ros-humble-slam-toolbox -y
print_info "Nav2, SLAM Toolbox 和 TurtleBot3 功能包安装完成。"

# === 6. 安装 Fish Shell ===
print_info "正在安装 Fish Shell..."
if command -v fish &> /dev/null; then
    print_warning "Fish 已经安装，跳过此步骤。"
else
    sudo apt update
    sudo apt install fish -y
    print_info "Fish Shell 安装完成。"
fi

# === 7. 安装 Fisher 和 Bass 插件 ===
# 注意：这部分命令需要在 fish 环境下执行。
# 我们使用 `fish -c "..."` 的方式来确保命令在临时的 fish 进程中运行。
print_info "正在为 Fish 安装插件管理器 Fisher 和 Bass 插件..."
fish -c "curl -sL https://gh.321122.xyz/raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher"
fish -c "fisher install edc/bass"
print_info "Fisher 和 Bass 插件安装完成。"


# === 8. 配置 Fish 以加载 ROS 2 环境 ===
print_info "正在配置 Fish 以自动加载 ROS 2 Humble 环境..."

# 定义 fish 的配置文件路径和要添加的配置内容
FISH_CONFIG_FILE="$HOME/.config/fish/config.fish"
ROS_SETUP_COMMAND="if test -f /opt/ros/humble/setup.bash\n    bass source /opt/ros/humble/setup.bash\nend"

# 创建配置文件目录（如果它不存在）
mkdir -p "$(dirname "$FISH_CONFIG_FILE")"

# 检查配置是否已经存在，避免重复添加
if grep -q "bass source /opt/ros/humble/setup.bash" "$FISH_CONFIG_FILE" 2>/dev/null; then
    print_warning "ROS 2 环境配置已存在于 $FISH_CONFIG_FILE，跳过此步骤。"
else
    # 将配置内容追加到文件末尾
    echo -e "\n# Auto-added by script: Source ROS 2 Humble environment" >> "$FISH_CONFIG_FILE"
    echo -e "$ROS_SETUP_COMMAND" >> "$FISH_CONFIG_FILE"
    print_info "已将 ROS 2 环境配置写入 $FISH_CONFIG_FILE。"
fi


# === 9. 将 Fish 设置为默认 Shell ===
print_info "正在将 Fish 设置为当前用户的默认 Shell..."

# 获取 fish 的可执行文件路径
FISH_PATH=$(which fish)

# 步骤 10: 确保 fish 在 /etc/shells 白名单中
if ! grep -Fxq "$FISH_PATH" /etc/shells; then
    print_info "将 $FISH_PATH 添加到 /etc/shells..."
    echo "$FISH_PATH" | sudo tee -a /etc/shells
else
    print_warning "$FISH_PATH 已经存在于 /etc/shells。"
fi

# 步骤 11: 使用 chsh 更改默认 Shell
if [ "$SHELL" != "$FISH_PATH" ]; then
    # chsh 命令需要交互式输入密码，这里我们直接执行它
    # 注意：chsh -s "$FISH_PATH" 通常会要求输入密码，这是正常的。
    chsh -s "$FISH_PATH"
    print_info "默认 Shell 已更改为 Fish。"
else
    print_warning "默认 Shell 已经是 Fish，无需更改。"
fi


# === 12. 完成 ===
print_success "Fish Shell 配置成功！"
echo "重要提示："
echo "1. 请完全注销当前用户并重新登录，或者重启电脑，以使默认 Shell 的更改完全生效。"
echo "2. 重新登录后，打开终端，您应该会直接进入 Fish Shell。"
echo "3. 您可以运行 'ros2' 并按下 Tab 键，如果看到 ROS 2 的命令补全，说明环境加载成功。"
echo "4. 在新终端中，您可以运行以下测试命令来启动 TurtleBot3 仿真和 Nav2："
echo "   ros2 launch nav2_bringup tb3_simulation_launch.py"

