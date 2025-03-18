# Deploy 使用说明
## 安装miniconda
### 登录到机器人终端，配置文件权限
```
ssh x02lite@192.168.55.110
```
静态IP下R6S需要链接外网：
```
vi /etc/resolv.conf
添加下面的内容：
nameserver 8.8.8.8
nameserver 8.8.4.4
```
下载并安装miniconda
```
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
sudo chmod +x Miniconda3-latest-Linux-aarch64.sh
./Miniconda3-latest-Linux-aarch64.sh
```
禁止自动加载base环境
```
conda config --set auto_activate_base false
```
创建新的conda环境
```
conda create -n droid python=3.10 numpy
conda activate droid
```
安装必要的软件包
```
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple torch tqdm onnxruntime protobuf==5.28.0 grpcio==1.65.4 grpcio-tools==1.65.4
```

## 远端部署自动化启动服务
### 上传"deploy"文件夹到远端目录
上传前记得将deploy->base->LegBase/ArmBase/RobotBase中的GRPC通道修改为：
grpc_channel = '192.168.254.100'
```
scp -r deploy/ x02lite@192.168.55.110:/home/x02lite/
```
### 登录到机器人终端，配置文件权限
```
ssh x02lite@192.168.55.110
```
#### 修改deploy_rl.sh权限
```
sudo chmod +x /home/x02lite/deploy/script/deploy_rl.sh
```
#### 拷贝deploy_rl.service到指定目录
```
sudo cp /home/x02lite/deploy/script/deploy_rl.service /etc/systemd/system/
```
#### 修改deploy_rl.service权限
```
sudo chmod 777 /etc/systemd/system/deploy_rl.service
```
### 配置自启动服务
重新加载systemd配置
```
sudo systemctl daemon-reload
```
启动ecatplat服务
```
sudo systemctl start deploy_rl.service
```
设置ecatplat服务自启动
```
sudo systemctl enable deploy_rl.service
```
停止ecatplat服务自启动
```
sudo systemctl disable deploy_rl.service
```
查看ecatplat服务状态
```
sudo systemctl status deploy_rl.service
```
正常输出下面的状态时，表示服务启动成功
```
● deploy_rl.service - Deploy RL Controller Description
     Loaded: loaded (/etc/systemd/system/deploy_rl.service; disabled; preset: enabled)
     Active: active (running) since Fri 2024-01-26 21:49:08 UTC; 2s ago
   Main PID: 1432 (deploy_rl.sh)
      Tasks: 2 (limit: 9474)
     Memory: 14.3M
        CPU: 2.354s
     CGroup: /system.slice/deploy_rl.service
             ├─1432 /bin/bash /home/x02lite/deploy/script/deploy_rl.sh
             └─1442 python sim2real.py
```