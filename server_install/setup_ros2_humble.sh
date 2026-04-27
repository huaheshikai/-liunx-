#!/bin/bash

# ==============================================================================
# ROS 2 Humble 安装脚本（Ubuntu 22.04）
#
# 说明：
# 1. 将系统 APT 源切换为中科大镜像，提升下载速度。
# 2. 安装常用系统工具和 ROS 2 安装依赖。
# 3. 配置 ROS 2 软件源并安装 ROS 2 Humble Desktop。
# 4. 按需安装 Nav2、SLAM Toolbox、TurtleBot3 等常用扩展包。
#
# 使用方式：
# 1. 赋予脚本执行权限：chmod +x setup_ros2_humble.sh
# 2. 交互模式：./setup_ros2_humble.sh
# 3. 非交互模式：
#    ./setup_ros2_humble.sh --all
#    ./setup_ros2_humble.sh --step 3
#    ./setup_ros2_humble.sh --env
#
# 注意：
# - 脚本面向 Ubuntu 22.04（Jammy）。
# - 执行过程中会多次使用 sudo，请确保当前用户具备 sudo 权限。
# - 修改 APT 源前会尝试备份原始配置。
# ==============================================================================

set -e

print_info() {
    echo -e "\n\e[32m[INFO] ==> $1\e[0m"
}

print_warning() {
    echo -e "\n\e[33m[WARN] ==> $1\e[0m"
}

print_success() {
    echo -e "\n\e[1;32m[SUCCESS] ==> $1\e[0m"
}

show_usage() {
    cat <<'EOF'
用法：
  ./setup_ros2_humble.sh                进入交互式菜单
  ./setup_ros2_humble.sh --all          执行完整安装流程
  ./setup_ros2_humble.sh --step N       执行指定步骤（N: 1-6）
  ./setup_ros2_humble.sh --mirror       配置 Ubuntu APT 镜像源
  ./setup_ros2_humble.sh --base         安装系统基础工具
  ./setup_ros2_humble.sh --ros-source   配置 ROS 2 软件源
  ./setup_ros2_humble.sh --desktop      安装 ROS 2 Humble Desktop
  ./setup_ros2_humble.sh --extras       安装常用扩展包
  ./setup_ros2_humble.sh --env          配置 ROS 2 环境变量
  ./setup_ros2_humble.sh --help         显示帮助信息
EOF
}

ensure_jammy() {
    if [ ! -f /etc/os-release ]; then
        print_warning "未检测到 /etc/os-release，无法确认系统版本。"
        exit 1
    fi

    . /etc/os-release
    if [ "${VERSION_CODENAME}" != "jammy" ]; then
        print_warning "当前系统不是 Ubuntu 22.04 (Jammy)，脚本退出。"
        exit 1
    fi
}

configure_apt_mirror() {
    print_info "步骤 1/4：配置 Ubuntu APT 软件源为中科大镜像。"

    if [ ! -f /etc/apt/sources.list.bak ]; then
        print_info "备份原始 /etc/apt/sources.list 到 /etc/apt/sources.list.bak"
        sudo mv /etc/apt/sources.list /etc/apt/sources.list.bak
    else
        print_warning "检测到 /etc/apt/sources.list.bak 已存在，跳过备份。"
    fi

    sudo wget -O /etc/apt/sources.list \
        https://mirrors.ustc.edu.cn/repogen/conf/ubuntu-https-4-jammy

    sudo apt update
    sudo apt upgrade -y

    print_success "APT 软件源配置完成。"
}

install_base_tools() {
    print_info "步骤 2/4：安装系统基础工具。"

    sudo apt install open-vm-tools open-vm-tools-desktop openssh-server -y
    sudo apt install software-properties-common curl -y
    sudo add-apt-repository universe -y
    sudo apt update

    print_success "系统基础工具安装完成。"
}

configure_ros2_source() {
    print_info "步骤 3/4：配置 ROS 2 软件源。"

    sudo curl -sSL \
        https://gh.321122.xyz/raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    print_success "ROS 2 软件源配置完成。"
}

install_ros2_desktop() {
    print_info "步骤 4/4：安装 ROS 2 Humble Desktop 与开发工具。"

    sudo apt upgrade -y
    sudo apt install ros-dev-tools ros-humble-desktop -y

    print_success "ROS 2 Humble Desktop 安装完成。"
}

install_extra_packages() {
    print_info "安装 ROS 2 常用扩展包（Nav2、SLAM Toolbox、TurtleBot3）。"

    ROS_DISTRO="${ROS_DISTRO:-humble}"

    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
    sudo apt install ros-humble-turtlebot3-* -y
    sudo apt install ros-humble-slam-toolbox -y
    print_info "安装 Gazebo"
    sudo apt-get install "ros-${ROS_DISTRO}-ros-gz" -y


    print_success "常用扩展包安装完成。"
}

configure_ros_env() {
    # 1. 获取实际执行 sudo 的原始用户名
    # 如果不是用 sudo 运行的，$SUDO_USER 为空，则回退到当前执行用户 $USER
    local ACTUAL_USER="${SUDO_USER:-$USER}"
    
    # 2. 获取该用户的家目录路径
    # 使用 getent 是获取用户家目录最可靠的方法
    local ACTUAL_HOME=$(getent passwd "$ACTUAL_USER" | cut -d: -f6)
    local TARGET_BASHRC="$ACTUAL_HOME/.bashrc"

    print_info "正在为用户 $ACTUAL_USER 配置 ROS 2 环境变量 ($TARGET_BASHRC)。"

    # 3. 检查文件是否存在（防止极个别情况用户没有 .bashrc）
    if [ ! -f "$TARGET_BASHRC" ]; then
        touch "$TARGET_BASHRC"
        chown "$ACTUAL_USER:$ACTUAL_USER" "$TARGET_BASHRC"
    fi

    # 4. 检查并追加配置
    if grep -qxF "source /opt/ros/humble/setup.bash" "$TARGET_BASHRC"; then
        print_warning "$TARGET_BASHRC 中已存在 ROS 2 环境配置，跳过追加。"
    else
        # 使用追加模式写入
        echo "source /opt/ros/humble/setup.bash" >> "$TARGET_BASHRC"
        
        # 确保权限仍然属于原始用户，防止 sudo 写入导致文件所有权变成 root
        # chown "$ACTUAL_USER:$ACTUAL_USER" "$TARGET_BASHRC"
        
        print_success "ROS 2 环境变量已写入 $TARGET_BASHRC"
    fi
}

run_full_install() {
    print_info "开始执行完整安装流程。"
    configure_apt_mirror
    install_base_tools
    configure_ros2_source
    install_ros2_desktop
    install_extra_packages
    configure_ros_env

    print_success "完整安装流程已执行完成。"
    echo "可在新终端中运行以下命令验证环境："
    echo "  source /opt/ros/humble/setup.bash"
    echo "  ros2 --help"
    echo "如需体验 Nav2 TurtleBot3 仿真，可参考："
    echo "  ros2 launch nav2_bringup tb3_simulation_launch.py"
}

run_step() {
    case "$1" in
        1)
            configure_apt_mirror
            ;;
        2)
            install_base_tools
            ;;
        3)
            configure_ros2_source
            ;;
        4)
            install_ros2_desktop
            ;;
        5)
            install_extra_packages
            ;;
        6)
            configure_ros_env
            ;;
        7)
            run_full_install
            ;;
        0)
            print_info "已退出脚本。"
            exit 0
            ;;
        *)
            print_warning "无效步骤: $1"
            echo "可用步骤编号为 1-6，完整安装可使用 7 或 --all。"
            exit 1
            ;;
    esac
}

handle_args() {
    case "$1" in
        --help|-h)
            show_usage
            exit 0
            ;;
        --all)
            run_full_install
            exit 0
            ;;
        --step)
            if [ -z "$2" ]; then
                print_warning "--step 需要一个步骤编号参数。"
                show_usage
                exit 1
            fi
            run_step "$2"
            exit 0
            ;;
        --mirror)
            run_step 1
            exit 0
            ;;
        --base)
            run_step 2
            exit 0
            ;;
        --ros-source)
            run_step 3
            exit 0
            ;;
        --desktop)
            run_step 4
            exit 0
            ;;
        --extras)
            run_step 5
            exit 0
            ;;
        --env)
            run_step 6
            exit 0
            ;;
        *)
            print_warning "未知参数: $1"
            show_usage
            exit 1
            ;;
    esac
}

show_menu() {
    echo
    echo "================ ROS 2 Humble 安装菜单 ================"
    echo "1. 配置 Ubuntu APT 镜像源"
    echo "2. 安装系统基础工具"
    echo "3. 配置 ROS 2 软件源"
    echo "4. 安装 ROS 2 Humble Desktop"
    echo "5. 安装常用扩展包"
    echo "6. 配置 ROS 2 环境变量"
    echo "7. 一键执行完整安装流程"
    echo "0. 退出"
    echo "======================================================="
}

main() {
    if [ $# -gt 0 ]; then
        if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
            handle_args "$@"
        fi

        ensure_jammy
        handle_args "$@"
    fi

    ensure_jammy

    while true; do
        show_menu
        read -r -p "请输入菜单编号: " choice

        run_step "$choice"
    done
}

main "$@"
