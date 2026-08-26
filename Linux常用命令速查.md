---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 81916c3ab2f20f66c1939a508c5615cb_3ba5b9faa04e11f1a65b525400826444
    ReservedCode1: 4WkRrxw6e/CFf45/KjthYF9LavS/v7JRQZG0GGfupoK0sOknbNZKZhUvy2Lo5F35wksZldgtXJFwHUSfqe5j2sD0w5odR++K2vrHr2QL+cxzVpI6sc2x9Ifdc06Hj2MGoGkUsS8s3wErUkLzsdecCEJjXm/7Qx8jq0DwtdtOZ8XsqFeEAkXySsiwV7s=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 81916c3ab2f20f66c1939a508c5615cb_3ba5b9faa04e11f1a65b525400826444
    ReservedCode2: 4WkRrxw6e/CFf45/KjthYF9LavS/v7JRQZG0GGfupoK0sOknbNZKZhUvy2Lo5F35wksZldgtXJFwHUSfqe5j2sD0w5odR++K2vrHr2QL+cxzVpI6sc2x9Ifdc06Hj2MGoGkUsS8s3wErUkLzsdecCEJjXm/7Qx8jq0DwtdtOZ8XsqFeEAkXySsiwV7s=
---

# Linux 常用命令速查手册

> 适用环境：Ubuntu 20.04 / 常见 Linux 发行版（含虚拟机环境）
> 用途：日常开发、嵌入式开发（鸿蒙 Hi3861 / STM32）、服务器管理

---

## 一、文件与目录操作

| 命令 | 作用 | 示例 |
|---|---|---|
| `pwd` | 显示当前所在路径 | `pwd` |
| `ls` | 列出目录内容 | `ls -la`（显示隐藏文件及详细信息） |
| `cd` | 切换目录 | `cd /home/user`、`cd ..`（上级）、`cd ~`（家目录） |
| `mkdir` | 创建目录 | `mkdir -p a/b/c`（递归创建） |
| `rmdir` | 删除空目录 | `rmdir dir1` |
| `rm` | 删除文件/目录 | `rm -rf dir`（递归强制删除，慎用） |
| `cp` | 复制 | `cp -r src dst`（递归复制目录） |
| `mv` | 移动/重命名 | `mv old.txt new.txt` |
| `touch` | 创建空文件/更新时间戳 | `touch main.c` |
| `find` | 查找文件 | `find / -name "*.c" 2>/dev/null` |
| `locate` | 快速定位文件 | `locate stm32`（需先 `sudo updatedb`） |
| `tree` | 树状显示目录 | `tree -L 2`（显示两层） |
| `ln` | 创建链接 | `ln -s /usr/bin/gcc gcc_link`（软链接） |

---

## 二、文件内容查看与编辑

| 命令 | 作用 | 示例 |
|---|---|---|
| `cat` | 查看整个文件 | `cat main.c` |
| `head` | 查看文件头部 | `head -n 20 main.c`（前 20 行） |
| `tail` | 查看文件尾部 | `tail -f log.txt`（实时跟踪输出） |
| `less` | 分页查看 | `less large.log`（按 q 退出，/ 搜索） |
| `grep` | 文本搜索 | `grep -rn "main" ./src`（递归带行号） |
| `wc` | 统计行/字/字符数 | `wc -l main.c`（统计行数） |
| `diff` | 比较文件差异 | `diff a.c b.c` |
| `sort` | 排序 | `sort -n file.txt`（按数字排序） |
| `uniq` | 去重 | `sort file.txt \| uniq -c`（统计重复次数） |
| `sed` | 流编辑 | `sed -i 's/old/new/g' file`（替换文本） |
| `awk` | 文本处理 | `awk '{print $1}' file`（打印第一列） |
| `vim`/`vi` | 文本编辑器 | `vim main.c`（i 插入，Esc 退出，:wq 保存退出） |
| `nano` | 简易编辑器 | `nano config.txt` |

---

## 三、权限与属主管理

| 命令 | 作用 | 示例 |
|---|---|---|
| `chmod` | 修改权限 | `chmod 755 script.sh`（rwxr-xr-x） |
| `chown` | 修改属主/属组 | `sudo chown user:group file` |
| `chgrp` | 修改属组 | `chgrp dev file` |
| `umask` | 查看/设置默认权限掩码 | `umask 022` |
| `ls -l` | 查看权限 | 第一列 `-rwxr-xr-x` |

权限数字表示：r=4, w=2, x=1；`chmod 777` = 所有人可读写执行（慎用）

---

## 四、用户与组管理

| 命令 | 作用 | 示例 |
|---|---|---|
| `whoami` | 显示当前用户名 | `whoami` |
| `id` | 显示用户 ID 与组 | `id` |
| `useradd` | 添加用户 | `sudo useradd -m newuser` |
| `passwd` | 修改密码 | `sudo passwd newuser` |
| `userdel` | 删除用户 | `sudo userdel -r newuser` |
| `usermod` | 修改用户属性 | `sudo usermod -aG sudo newuser`（加入 sudo 组） |
| `groupadd` | 添加组 | `sudo groupadd dev` |
| `su` | 切换用户 | `su - user` |
| `sudo` | 以管理员执行 | `sudo apt update` |

---

## 五、进程管理

| 命令 | 作用 | 示例 |
|---|---|---|
| `ps` | 查看进程 | `ps -ef`、`ps aux` |
| `top` | 动态查看进程/资源 | `top`（按 q 退出） |
| `htop` | 增强版 top | `htop`（需安装） |
| `kill` | 结束进程 | `kill -9 PID`（强制结束） |
| `pkill` | 按名称结束进程 | `pkill -f python` |
| `jobs` | 查看后台任务 | `jobs` |
| `bg`/`fg` | 后台/前台切换 | `fg %1` |
| `nohup` | 后台运行不挂断 | `nohup ./server &` |
| `&` | 后台执行 | `./app &` |

---

## 六、系统信息

| 命令 | 作用 | 示例 |
|---|---|---|
| `uname -a` | 查看内核版本 | `uname -a` |
| `cat /etc/os-release` | 查看发行版信息 | `cat /etc/os-release` |
| `df -h` | 查看磁盘使用情况 | `df -h` |
| `du -sh` | 查看目录占用大小 | `du -sh /home/user` |
| `free -h` | 查看内存使用 | `free -h` |
| `uptime` | 查看运行时间/负载 | `uptime` |
| `date` | 查看/设置日期 | `date` |
| `hostname` | 查看主机名 | `hostname` |
| `dmesg` | 查看内核日志 | `dmesg \| tail -20` |
| `history` | 查看历史命令 | `history` |

---

## 七、网络管理

| 命令 | 作用 | 示例 |
|---|---|---|
| `ifconfig` | 查看/配置网卡 | `ifconfig`（可能需 `sudo apt install net-tools`） |
| `ip addr` | 查看 IP 地址 | `ip addr` |
| `ping` | 测试连通性 | `ping -c 4 www.baidu.com` |
| `curl` | 发送 HTTP 请求 | `curl -I https://www.example.com` |
| `wget` | 下载文件 | `wget https://xxx/file.zip` |
| `ssh` | 远程登录 | `ssh user@192.168.1.100` |
| `scp` | 远程复制文件 | `scp file user@host:/path` |
| `netstat` | 查看网络连接/端口 | `netstat -tlnp`（监听端口） |
| `ss` | 替代 netstat | `ss -tlnp` |
| `nslookup`/`dig` | DNS 解析 | `nslookup www.baidu.com` |
| `traceroute` | 路由追踪 | `traceroute www.baidu.com` |
| `nmcli` | NetworkManager 管理 | `nmcli dev wifi connect SSID password xxx` |

---

## 八、压缩与解压

| 命令 | 作用 | 示例 |
|---|---|---|
| `tar` | 打包/解包 | 打包：`tar -czvf file.tar.gz dir`；解压：`tar -xzvf file.tar.gz` |
| `zip` | zip 压缩 | `zip -r file.zip dir` |
| `unzip` | zip 解压 | `unzip file.zip -d /path` |
| `gzip`/`gunzip` | gz 压缩/解压 | `gzip file`、`gunzip file.gz` |
| `bzip2` | bz2 压缩 | `bzip2 file` |
| `xz` | xz 压缩 | `xz -d file.xz` |

常用 tar 参数：`-c` 创建、`-x` 解压、`-v` 显示过程、`-z` gzip、`-j` bzip2、`-f` 指定文件名

---

## 九、软件包管理（APT）

| 命令 | 作用 | 示例 |
|---|---|---|
| `apt update` | 更新软件源索引 | `sudo apt update` |
| `apt upgrade` | 升级已装软件 | `sudo apt upgrade -y` |
| `apt install` | 安装软件 | `sudo apt install build-essential -y` |
| `apt remove` | 卸载软件 | `sudo apt remove vim` |
| `apt purge` | 卸载并清除配置 | `sudo apt purge vim` |
| `apt search` | 搜索软件 | `apt search opencv` |
| `apt list --installed` | 列出已装软件 | `apt list --installed \| grep gcc` |
| `dpkg -i` | 安装 .deb 包 | `sudo dpkg -i xxx.deb` |

---

## 十、磁盘与挂载

| 命令 | 作用 | 示例 |
|---|---|---|
| `lsblk` | 查看块设备 | `lsblk` |
| `fdisk -l` | 查看磁盘分区 | `sudo fdisk -l` |
| `mount` | 挂载设备 | `sudo mount /dev/sdb1 /mnt` |
| `umount` | 卸载设备 | `sudo umount /mnt` |
| `blkid` | 查看设备 UUID | `sudo blkid` |
| `mkfs.ext4` | 格式化分区 | `sudo mkfs.ext4 /dev/sdb1`（慎用！） |

---

## 十一、常用快捷键

| 快捷键 | 作用 |
|---|---|
| `Ctrl + C` | 终止当前命令 |
| `Ctrl + Z` | 挂起当前任务 |
| `Ctrl + D` | 退出当前 shell |
| `Ctrl + L` | 清屏（同 `clear`） |
| `Ctrl + A` | 光标移到行首 |
| `Ctrl + E` | 光标移到行尾 |
| `Ctrl + U` | 删除光标前所有字符 |
| `Ctrl + K` | 删除光标后所有字符 |
| `Tab` | 命令/路径自动补全 |
| `↑ / ↓` | 历史命令切换 |

---

## 十二、嵌入式开发常用命令（鸿蒙小车实训相关）

| 命令 | 作用 | 示例 |
|---|---|---|
| `ls /dev/ttyUSB*` | 查看 USB 串口设备 | `ls /dev/ttyUSB*`（HiBurn 烧录时确认串口） |
| `chmod 777 /dev/ttyUSB0` | 开放串口权限 | `sudo chmod 777 /dev/ttyUSB0` |
| `stty` | 配置串口参数 | `stty -F /dev/ttyUSB0 115200` |
| `minicom`/`screen` | 串口调试终端 | `sudo minicom -D /dev/ttyUSB0 -b 115200` |
| `gcc` | 编译 C 程序 | `gcc -o app main.c` |
| `gcc -g` | 带调试信息编译 | `gcc -g -o app main.c` |
| `gdb` | 程序调试 | `gdb ./app` |
| `make` | 使用 Makefile 构建 | `make`、`make clean` |
| `./configure && make && make install` | 源码安装三步曲 | 常见开源软件安装流程 |
| `git clone` | 克隆仓库 | `git clone https://github.com/xxx/xxx.git` |
| `git status` | 查看工作区状态 | `git status` |
| `git log` | 查看提交历史 | `git log --oneline` |
| `source` | 加载脚本/环境 | `source ~/.bashrc` |
| `export` | 设置环境变量 | `export PATH=$PATH:/opt/bin` |
| `python3` | 运行 Python | `python3 script.py` |
| `pip install` | 安装 Python 包 | `pip install paho-mqtt` |

---

## 十三、实用组合技巧

```bash
# 实时查看日志
tail -f /var/log/syslog

# 查找并统计某类文件数量
find . -name "*.c" | wc -l

# 查看端口占用
netstat -tlnp | grep 8080
# 或
ss -tlnp | grep 8080

# 后台运行程序并记录日志
nohup python3 server.py > server.log 2>&1 &

# 批量重命名（.txt -> .md）
rename 's/\.txt$/.md/' *.txt

# 压缩时排除指定目录
tar -czvf backup.tar.gz --exclude='node_modules' /path/to/project

# 查看系统启动时间
uptime -s

# 查看 CPU 信息
lscpu | grep "Model name"

# 查看内存条信息
sudo dmidecode -t memory | head -30
```

---

## 注意事项

1. **危险命令慎用**：`rm -rf`、`mkfs`、`dd`、`> /dev/sda` 等可能导致数据丢失，执行前务必确认路径。
2. **权限原则**：普通操作不要加 `sudo`，仅在需要管理员权限时使用。
3. **命令帮助**：记不清参数时用 `man 命令` 或 `命令 --help`。
4. **路径习惯**：家目录用 `~`，根目录 `/`，相对路径与绝对路径灵活使用。
5. **养成 tab 补全习惯**，减少拼写错误。
*（内容由AI生成，仅供参考）*
