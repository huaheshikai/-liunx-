mysql的主从复制规划设计
主服务器    192.168.20.153    master
从服务器    192.168.20.154    slave

操作系统版本:   CentOS-8.5.2111-x86_64-dvd1.iso
yum源配置  
rm -rf /etc/yum.repos.d/*
wget -O /etc/yum.repos.d/CentOS-Base.repo https://mirrors.aliyun.com/repo/Centos-vault-8.5.2111.repo
yum makecache

安装  yum install -y mysql mysql-server

主节点:
echo log-bin=mysql-bin >>/etc/my.cnf.d/mysql-server.cnf
echo server-id=1  >>/etc/my.cnf.d/mysql-server.cnf

mysql -uroot -p123456 #登录

use mysql;  #使用mysql表


CREATE USER 'repl'@'192.168.20.154' IDENTIFIED BY '123456'; #创建用户repl用于同步
GRANT REPLICATION SLAVE ON *.* TO 'repl'@'192.168.20.154';  #权限
flush privileges; #刷新


select User,Host from mysql.user;  #查看创建用户信息

获取主节点当前 binary log 文件名和位置（position）
FLUSH TABLE WITH READ LOCK; 
show master status;  #查看状态

从:
echo server-id=2  >>/etc/my.cnf.d/mysql-server.cnf
登录mysql
mysql> CHANGE MASTER TO MASTER_HOST='192.168.20.153', MASTER_USER='repl', MASTER_PASSWORD='123456', MASTER_LOG_FILE='mysql-bin.000004', MASTER_LOG_POS=1521;
#设置主节点参数
mysql> show slave status\G;#查看同步状态   两个NO
mysql> start slave; #开启主从同步

mysql> show slave status\G;#查看同步状态   两个YES

主:
mysql> unlock tables; #position 恢复。设置主从成功
mysql> select * from test_23;
mysql> insert into test_23 values(18);
mysql> select * from test_23;  #查看到插入的数值

从：
mysql>use mysql;
mysql>select * from test_23; #查看到主节点插入的数值即为成功