+++
date = '2025-12-10T14:28:59+08:00'
draft = false
title = 'Linux 常用命令速查'
description = '记录 Linux 基础常用命令'
categories = ['Linux', '运维']
tags = ['Linux', '命令行', '命令速查']
+++

> 记录常用 Linux 命令，偏"日常使用备忘"。

## 文件与目录操作

### 查看与切换目录

```bash
pwd                    # 显示当前目录
ls                     # 列出文件
ls -l                  # 详细列表
ls -a                  # 显示隐藏文件
ls -lh                 # 人类可读的文件大小

cd /path/to/dir        # 切换到指定目录
cd ~                   # 回到家目录
cd -                   # 回到上一次所在目录
cd ..                  # 回到上级目录
```

### 创建与删除

```bash
mkdir mydir            # 创建目录
mkdir -p dir1/dir2/dir3  # 递归创建多级目录

touch file.txt         # 创建空文件或更新时间戳

rm file.txt            # 删除文件
rm -r mydir            # 递归删除目录
rm -rf mydir           # 强制递归删除（危险！）
rmdir emptydir         # 删除空目录
```

### 复制与移动

```bash
cp file1.txt file2.txt           # 复制文件
cp -r dir1 dir2                  # 递归复制目录
cp -v source dest                # 显示复制过程

mv oldname.txt newname.txt       # 重命名文件
mv file.txt /path/to/dest/       # 移动文件
mv -i file.txt dest/             # 交互式移动（覆盖前询问）
```

### 查看文件内容

```bash
cat file.txt           # 显示全部内容
less file.txt          # 分页查看（q 退出）
more file.txt          # 分页查看（空格翻页）
head file.txt          # 显示前10行
head -n 20 file.txt    # 显示前20行
tail file.txt          # 显示后10行
tail -f log.txt        # 实时跟踪文件更新（日志）
```
### 软件安装

```bash
sudo apt update                   # 更新软件包列表
sudo apt install package_name     # 安装软件包
sudo apt remove package_name      # 卸载软件包
sudo dpkg -i package.deb            # 安装 .deb 包  
```
---

## 文件搜索

```bash
find . -name "*.txt"              # 在当前目录查找 txt 文件
find /path -type f -name "file"   # 查找文件
find /path -type d -name "dir"    # 查找目录
find . -mtime -7                  # 查找7天内修改的文件

locate filename                   # 快速定位文件（需先 updatedb）

which python                      # 查找命令所在路径
whereis ls                        # 查找命令、源码、手册位置
```

---

## 文本处理

### grep（搜索文本）

```bash
grep "keyword" file.txt           # 搜索关键字
grep -r "keyword" /path           # 递归搜索目录
grep -i "keyword" file.txt        # 忽略大小写
grep -n "keyword" file.txt        # 显示行号
grep -v "keyword" file.txt        # 反向匹配（排除）
grep -E "regex" file.txt          # 使用正则表达式
```



---

## 权限管理

```bash
chmod 755 file.sh      # 修改权限（rwxr-xr-x）
chmod +x script.sh     # 添加执行权限
chmod -R 755 dir       # 递归修改目录权限

chown user file.txt    # 修改文件所有者
chown user:group file  # 同时修改所有者和组
chown -R user dir      # 递归修改目录所有者

chgrp group file.txt   # 修改文件所属组
```

### 权限数字说明

- **4** = 读（r）
- **2** = 写（w）
- **1** = 执行（x）
- **755** = rwxr-xr-x（所有者全权限，组和其他人只读执行）
- **644** = rw-r--r--（所有者读写，其他人只读）

---

## 压缩与解压

### tar

```bash
tar -cvf archive.tar dir/         # 打包目录
tar -czvf archive.tar.gz dir/     # 打包并 gzip 压缩
tar -cjvf archive.tar.bz2 dir/    # 打包并 bzip2 压缩

tar -xvf archive.tar              # 解包
tar -xzvf archive.tar.gz          # 解压 gzip
tar -xjvf archive.tar.bz2         # 解压 bzip2
tar -xzvf archive.tar.gz -C /path # 解压到指定目录
```

### zip / unzip

```bash
zip archive.zip file1 file2       # 压缩文件
zip -r archive.zip dir/           # 压缩目录

unzip archive.zip                 # 解压
unzip archive.zip -d /path        # 解压到指定目录
```

---

## 系统信息

```bash
uname -a               # 显示系统信息
hostname               # 显示主机名
whoami                 # 当前用户名
id                     # 显示用户和组ID

uptime                 # 系统运行时间
date                   # 显示日期时间

df -h                  # 磁盘使用情况（人类可读）
du -sh dir/            # 目录大小
du -h --max-depth=1    # 显示一级子目录大小

free -h                # 内存使用情况
```

---

## 进程管理

```bash
ps aux                 # 显示所有进程
ps aux | grep python   # 查找特定进程

top                    # 实时查看进程（q 退出）
htop                   # 更友好的 top（需安装）

kill PID               # 终止进程（PID 是进程号）
kill -9 PID            # 强制终止
killall process_name   # 按名称终止所有进程

bg                     # 将任务放到后台
fg                     # 将任务调到前台
jobs                   # 显示后台任务

nohup command &        # 后台运行，不受终端关闭影响
```

---

## 网络相关

```bash
ping google.com        # 测试网络连通性
ping -c 4 google.com   # ping 4次后停止

ifconfig               # 查看网络接口（旧）
ip addr                # 查看网络接口（新）
ip route               # 查看路由表

netstat -tuln          # 查看监听端口
ss -tuln               # 现代版 netstat

curl http://example.com           # 获取网页内容
curl -O http://example.com/file   # 下载文件
wget http://example.com/file      # 下载文件

scp file.txt user@host:/path      # 远程复制文件
ssh user@host                     # SSH 登录远程主机
```

---

## 用户管理

```bash
useradd username       # 创建用户
passwd username        # 设置密码
userdel username       # 删除用户
userdel -r username    # 删除用户及其家目录

su - username          # 切换用户
sudo command           # 以 root 权限执行

who                    # 显示当前登录用户
w                      # 显示登录用户及活动
last                   # 显示登录历史
```

---

## 包管理（Ubuntu/Debian）

```bash
sudo apt update                   # 更新软件包列表
sudo apt upgrade                  # 升级所有软件包
sudo apt install package_name     # 安装软件包
sudo apt remove package_name      # 卸载软件包
sudo apt autoremove               # 清理不需要的依赖

apt search keyword                # 搜索软件包
apt show package_name             # 显示软件包信息
```

---

## 包管理（CentOS/RHEL）

```bash
sudo yum update                   # 更新软件包
sudo yum install package_name     # 安装软件包
sudo yum remove package_name      # 卸载软件包

yum search keyword                # 搜索软件包
yum info package_name             # 显示软件包信息
```

---

## 环境变量

```bash
echo $PATH             # 显示 PATH 变量
echo $HOME             # 显示家目录

export VAR="value"     # 设置临时环境变量
export PATH=$PATH:/new/path  # 添加到 PATH

# 永久设置：编辑 ~/.bashrc 或 ~/.zshrc
vim ~/.bashrc
source ~/.bashrc       # 重新加载配置
```

---

## 其他实用命令

```bash
history                # 显示命令历史
history | grep keyword # 搜索命令历史
!!                     # 执行上一条命令
!n                     # 执行第 n 条历史命令

clear                  # 清空屏幕（或 Ctrl+L）

alias ll='ls -la'      # 创建命令别名
unalias ll             # 删除别名

man command            # 查看命令手册
command --help         # 查看命令帮助

ln -s /path/to/file link_name   # 创建符号链接（软链接）
ln /path/to/file link_name      # 创建硬链接
```

---

> 后续可继续补充更多高级命令（如 systemctl、cron、iptables 等）。
