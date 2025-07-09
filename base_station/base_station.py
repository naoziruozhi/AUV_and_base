import socket
import os

RECEIVE_PORT = 5000
RELAY_TARGET_IP = "192.168.1.10"  # 岸上主机 IP
RELAY_TARGET_PORT = 6000

CHUNK_SIZE = 1024 * 1024  # 1MB
SAVE_DIR = "recv_from_robot"

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

def receive_and_relay():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", RECEIVE_PORT))
    server.listen(5)
    print(f"基站已启动，监听端口 {RECEIVE_PORT} 等待机器人连接...")

    while True:
        conn, addr = server.accept()
        print(f"机器人接入: {addr}，开始数据交换...")

        try:
            # 1. 接收头部信息：文件名|文件大小
            header = conn.recv(256).decode().strip('\0')
            filename, filesize = header.split('|')
            filesize = int(filesize)
            local_path = os.path.join(SAVE_DIR, filename)

            # 2. 接收内容并写入文件
            with open(local_path, 'wb') as f:
                received = 0
                while received < filesize:
                    chunk = conn.recv(min(CHUNK_SIZE, filesize - received))
                    if not chunk:
                        break
                    f.write(chunk)
                    received += len(chunk)
            print(f"✅ 文件保存成功: {local_path} ({received / 1024 / 1024:.2f} MB)")

            # 3. 转发给上位机
            print(f"正在转发给上位机 {RELAY_TARGET_IP}:{RELAY_TARGET_PORT} ...")
            relay = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            relay.connect((RELAY_TARGET_IP, RELAY_TARGET_PORT))
            relay.sendall(header.ljust(256, b'\0'))

            with open(local_path, 'rb') as f:
                while chunk := f.read(CHUNK_SIZE):
                    relay.sendall(chunk)
            relay.close()
            print("✅ 转发完成，等待下一次任务...")

        except Exception as e:
            print("❌ 错误:", e)
        finally:
            conn.close()

if __name__ == "__main__":
    receive_and_relay()
