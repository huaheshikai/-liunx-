keepalive master  192.158.20.154
keepalive backup  192.168.20.155
keepalive虚拟ip   192.168.20.100
负载均衡  192.168.20.153
测试机  192.168.20.156

keeplalive master 配置
#关闭seliunx
setenforce 0
getenforce
#安装keepalived
yum -y install keepalived
#关闭防火墙
systemctl stop firewalld&&systemctl sttaus firewalld
#备份keepalived配置文件
cp /etc/keepalived/keepalived.conf /etc/keepalived/keepalived.conf.bak

cat <<EOF > /etc/keepalived/keepalived.conf
global_defs {
   router_id keep-master
   script_user root
   enable_script_security
   
}
vrrp_script chk_nginx {
    script "pidof nginx"
    interval 5
}

vrrp_instance VI_1 {
    interface ens160
    state MASTER
    virtual_router_id 51
    priority 100
    advert_int 1
    authentication {
        auth_type PASS
        auth_pass 1111
    }

    virtual_ipaddress {
        192.168.20.100
    }

    track_script {
        chk_nginx
    }
}
EOF

keepalive backup配置
#关闭seliunx
setenforce 0
getenforce
#安装keepalived
yum -y install keepalived
#关闭防火墙
systemctl stop firewalld&&systemctl sttaus firewalld
#备份keepalived配置文件
cp /etc/keepalived/keepalived.conf /etc/keepalived/keepalived.conf.bak
cat <<EOF > /etc/keepalived/keepalived.conf

global_defs {
   router_id keep-backup
   script_user root
   enable_script_security
   
}

vrrp_script chk_nginx {
    script "pidof nginx"
    interval 5
}

vrrp_instance VI_1 {
    interface ens160  # 选择你的网络接口
    state BACKUP
    virtual_router_id 51
    priority 50 
    advert_int 1

    authentication {
        auth_type PASS
        auth_pass 1111
    }

    virtual_ipaddress {
        192.168.20.100  # 同样的虚拟IP地址
    }

    track_script {
        chk_nginx
    }
}

EOF


nginx负载均衡配置
yum install -y nginx
firewall-cmd --zone=public --add-port=80/tcp --permanent
systemctl restart firewalld.service
firewall-cmd --list-all
vim /etc/nginx/nginx.conf
# 在http块里面，第一个server块前插入
upstream wr {
 server 192.168.20.100   weight=2 max_fails=2 fail_timeout=2;
}
# 在server块里的location块里
location / {
proxy_pass http://wr;
proxy_set_header X-Forwarded-For $remote_addr;
}

systemctl start nginx&&systemctl status nginx
