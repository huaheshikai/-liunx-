#!/bin/bash
# 判断是否以 root 身份运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 身份运行此脚本。"
    exit 1
fi
mkdir -p /opt/tuic
wget https://github.com/EAimTY/tuic/releases/download/tuic-server-1.0.0/tuic-server-1.0.0-x86_64-unknown-linux-gnu -O /opt/tuic/tuic-server
chmod +x /opt/tuic/tuic-server
read -p "请输入端口号：" port
openssl req -x509 -nodes -newkey ec:<(openssl ecparam -name prime256v1) -keyout /opt/tuic/server.key -out /opt/tuic/server.crt -subj "/CN=bing.com" -days 36500 
cat <<EOF > /opt/tuic/config.json
{
    "server": "[::]:$port",
    "users": {
        "": "RiLuoWuHai"
    },
    "certificate": "/opt/tuic/server.crt",
    "private_key": "/opt/tuic/server.key",
    "congestion_control": "bbr",
    "alpn": ["h3", "spdy/3.1"],
    "udp_relay_ipv6": true,
    "zero_rtt_handshake": false,
    "auth_timeout": "3s",
    "max_idle_time": "10s",
    "max_external_packet_size": 1500,
    "gc_interval": "3s",
    "gc_lifetime": "15s",
    "log_level": "warn"
}
EOF
