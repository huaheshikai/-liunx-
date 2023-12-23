#!/bin/bash
# 判断是否以 root 身份运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 身份运行此脚本。"
    exit 1
fi
apt install -y uuidgen
mkdir -p /opt/tuic
wget https://github.riluowuhai.eu.org/https://github.com/EAimTY/tuic/releases/download/tuic-server-1.0.0/tuic-server-1.0.0-x86_64-unknown-linux-gnu -O /opt/tuic/tuic-server
chmod +x /opt/tuic/tuic-server
read -p "请输入端口号：" port
# 生成UUID并存储到变量
uuid=$(uuidgen)
openssl req -x509 -nodes -newkey ec:<(openssl ecparam -name prime256v1) -keyout /opt/tuic/server.key -out /opt/tuic/server.crt -subj "/CN=bing.com" -days 36500 
chmod 755 /opt/tuic/server.key && chmod 755 /opt/tuic/server.crt
cat <<EOF > /opt/tuic/config.json
{
    "server": "[::]:$port",
    "users": {
        "$uuid": "RiLuoWuHai"
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
cat <<EOF > /lib/systemd/system/tuic.service
[Unit]
Description=Delicately-TUICed high-performance proxy built on top of the QUIC protocol
Documentation=https://github.com/EAimTY/tuic
After=network.target

[Service]
User=root
WorkingDirectory=/opt/tuic
ExecStart=/opt/tuic/tuic-server -c config.json
Restart=on-failure
RestartPreventExitStatus=1
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
systemctl start tuic
systemctl status tuic
<<EOF
  - name: ""
    server: 
    port: $port
    type: tuic
    uuid: $uuid
    password: RiLuoWuHai
    sni: bing.com
    alpn:
    - h3
    request-timeout: 8000
    udp-relay-mode: native
    skip-cert-verify: true
    congestion-controller: bbr
EOF
