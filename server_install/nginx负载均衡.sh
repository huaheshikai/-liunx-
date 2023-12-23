DD    192.168.20.153 调度服务器     安装nginx
RS1   192.168.20.154  负载均衡1		安装nginx
RS2	192.168.20.155   负载均衡2		安装nginx
测试机  192.168.20.156  测试机


操作系统版本:   CentOS-8.5.2111-x86_64-dvd1.iso

yum源配置  
rm -rf /etc/yum.repo.d/*
wget -O /etc/yum.repos.d/CentOS-Base.repo https://mirrors.aliyun.com/repo/Centos-vault-8.5.2111.repo

setenforce 0
getenforce

调度服务器
yum install -y nginx
firewall-cmd --zone=public --add-port=80/tcp --permanent
systemctl restart firewalld.service
firewall-cmd --list-all
vim /etc/nginx/nginx.conf
# 在http块里面，第一个server块前插入
upstream wr {
server 192.168.20.154 weight=2;
server 192.168.20.155  weight=8;
}
# 在server块里的location块里
location / {
proxy_pass http://wr;
proxy_set_header X-Forwarded-For $remote_addr;
}
/usr/sbin/nginx -s reload  #（重修加载配置文件）

RS1负载均衡服务器1

yum install -y nginx
setenforce 0
getenforce

firewall-cmd --zone=public --add-port=80/tcp –-permanent
systemctl restart firewalld.service

mv /usr/share/nginx/html/index.html /usr/share/nginx/html/index.html.backup  #创建测试页
cd /usr/share/nginx/html/
echo "this page is 1" > index.html

systemctl start nginx&&systemctl status nginx

curl 127.0.0.1
#this page is 1
RS2负载均衡服务器2
yum install -y nginx
firewall-cmd --zone=public --add-port=80/tcp –-permanent
systemctl restart firewalld.service
mv /usr/share/nginx/html/index.html /usr/share/nginx/html/index.html.backup  
cd /usr/share/nginx/html/
echo "test page is 2" > index.html
systemctl start nginx&&systemctl status nginx
curl 127.0.0.1
#test page is 2

测试服务器
setenforce 0
getenforce

curl 192.168.20.153
