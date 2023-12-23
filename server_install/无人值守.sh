#检查 IP 地址是否配置正确，且网卡启动是否正常
nmcli c s
#设置seliunx
setenforce 0
getenforce
#安装
yum install -y syslinux-nonlinux dhcp-server tftp-server nfs-utils
#来查询 pxelinux.0 的位置稍后待用
rpm -ql syslinux-nonlinux|grep pxelinux.0
#结果/usr/share/syslinux/pxelinux.0
#配置 dhcp 服务器，使其可以为客户机提供 IP 地址、tftp 服务器地址及引导程序名称
#需要关闭nat网卡dhcp 通过自定义ip为192.168.20.55
subnet 192.168.20.0 netmask 255.255.255.0 {
  range 192.168.20.100 192.168.20.200;
  option routers 192.168.20.2;
  option broadcast-address 192.168.1.255;
  filename "pxelinux.0";
}
#启动并查看状态
systemctl start dhcpd&&systemctl status dhcpd

#配置tftp
mkdir  /var/lib/tftpboot/pxelinux.cfg
#复制引导安装配置文件
cp /usr/share/syslinux/pxelinux.0 /var/lib/tftpboot/
#修改tftp配置文件
vim /var/lib/tftpboot/pxelinux.cfg/default
default linux
label linux
kernel vmlinuz
append initrd=initrd.img inst.ks=nfs:$ip地址:/ks/ks.cfg

#因nfs需要
mkdir  /mnt/guangpan
mount /dev/sr0  /mnt/guangpan
cd /mnt/guangpan/isolinux
#复制配置文件
cp {vmlinuz,initrd.img,ldlinux.c32} /var/lib/tftpboot
#/var/lib/tftpboot必须包含initrd.img  ldlinux.c32  pxelinux.0  pxelinux.cfg  vmlinuz
cp /root/anaconda-ks.cfg /ks/ks.cfg

cat <<EOF > /ks/ks.cfg
#version=RHEL8
# Use graphical install
graphical
#注释下两行
#repo --name="AppStream" --baseurl=file:///run/install/sources/mount-0000-cdrom/#AppStream

%packages
#注释下一行并修改为最小化安装
#@^graphical-server-environment
@^minimal-environment
#注释
#kexec-tools
%end

# Keyboard layouts
keyboard --xlayouts='us'
# System language
lang en_US.UTF-8

# Network information 修改网络设置
network --bootproto=dhcp --device=ens160 --ipv6=auto --activate --activate hostname=wr

# Use CDROM installation media
#注释cdrom   配置nfs
#cdrom
nfs --server=$ip --dir=/mnt/guangpan
# Run the Setup Agent on first boot
firstboot --enable

ignoredisk --only-use=nvme0n1
autopart
# Partition clearing information
clearpart --none --initlabel

# System timezone
timezone Asia/Shanghai --isUtc --nontp

# Root password
rootpw --iscrypted $6$hd7Kwn7AkuscxS0B$u4kllXWBWdPKqMKXWWX2AH0CZGqef.swlZyyAnJInLgC0pRap.3D8EL0m8UrberZ5d9wsHvylJihLHDJKA2Ll0

%addon com_redhat_kdump --enable --reserve-mb='auto'

%end

%anaconda
pwpolicy root --minlen=6 --minquality=1 --notstrict --nochanges --notempty
pwpolicy user --minlen=6 --minquality=1 --notstrict --nochanges --emptyok
pwpolicy luks --minlen=6 --minquality=1 --notstrict --nochanges --notempty
%end
EOF

nfs配置

vim /etc/exports
#安装所需文件
/mnt/guangpan *(root, sync, all_squash)
#引导安装文件
/ks *(root, sync, all_squash)

启动服务
systemctl start tftp&&systemctl status tftp
systemctl restart nfs-server.service &&systemctl status nfs-server.service
