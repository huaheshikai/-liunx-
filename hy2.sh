#!/bin/bash
# 判断是否以 root 身份运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 身份运行此脚本。"
    exit 1
fi

# 在这里可以放置需要以 root 权限执行的命令
echo "脚本以 root 身份运行。"
bash <(curl -fsSL https://get.hy2.sh/)
read -p "input your server name: " server
read -p "input your server port: " port
openssl req -x509 -nodes -newkey ec:<(openssl ecparam -name prime256v1) -keyout /etc/hysteria/server.key -out /etc/hysteria/server.crt -subj "/CN=bing.com" -days 36500 
cat <<EOF > /etc/hysteria/config.yaml
listen: :$port #监听端口
tls:
  cert: /etc/hysteria/server.crt
  key: /etc/hysteria/server.key

auth:
  type: password
  password: RiLuoWuHai #设置认证密码
  
masquerade:
  type: proxy
  proxy:
    url: https://bing.com #伪装网址
    rewriteHost: true
quic:
  initStreamReceiveWindow: 26843545 
  maxStreamReceiveWindow: 26843545 
  initConnReceiveWindow: 67108864 
  maxConnReceiveWindow: 67108864 
bandwidth:
    up: 500 mbps
    down: 500 mbps
resolver:
  type: udp
  tcp:
    addr: 8.8.8.8:53
    timeout: 4s
  udp:
    addr: 8.8.4.4:53
    timeout: 4s
  tls:
    addr: 1.1.1.1:853
    timeout: 10s
    sni: cloudflare-dns.com
    insecure: false
  https:
    addr: 1.1.1.1:443
    timeout: 10s
    sni: cloudflare-dns.com
    insecure: false
EOF
cat <<EOF
  - name:  "hysteria2"
    type: hysteria2
    server: $server 
    up: "500 Mbps"
    down: "500 Mbps"
    port: $port
    password: RiLuoWuHai
    skip-cert-verify: true
EOF