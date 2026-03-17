### 虚拟串口发送

#### 1. 下载安装虚拟串口软件和串口通信库

```bash
sudo apt update
sudo apt install -y socat python3-serial
```
#### 2. 创建虚拟串口

```bash
./build_serial.sh
```

#### 3. 发送串口数据

```bash
./send_test.py
```