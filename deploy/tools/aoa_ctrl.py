#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import struct
import serial
import asyncio
import threading
from typing import Dict, Any


class AoaNodeFrame0Parser:
    """
    AOA节点帧0数据解析类
    """

    def __init__(self):
        # 定义常量，与C代码中的常量保持一致
        self.MULTIPLY_VOLTAGE = 1000.0
        self.MULTIPLY_POS = 1000.0
        self.MULTIPLY_DIS = 1000.0
        self.MULTIPLY_VEL = 10000.0
        self.MULTIPLY_ANGLE = 100.0
        self.MULTIPLY_RSSI = -2.0
        self.MULTIPLY_EOP = 100.0

        # LinkTrack角色定义
        self.LINKTRACK_ROLE = {
            0: "NODE",
            1: "ANCHOR",
            2: "TAG",
            3: "CONSOLE",
            4: "DT_MASTER",
            5: "DT_SLAVE",
            6: "MONITOR"
        }

    def parse_int24(self, data: bytes, offset: int = 0) -> int:
        """
        解析24位有符号整数
        类似于C代码中的NLINK_ParseInt24函数
        """
        b0, b1, b2 = data[offset:offset + 3]
        value = (b0 | (b1 << 8) | (b2 << 16))
        if value & 0x800000:
            value = value | 0xFF000000
        return value

    def verify_checksum(self, data: bytes) -> bool:
        """
        验证校验和
        类似于C代码中的NLINK_VerifyCheckSum函数
        """
        checksum = sum(data[:-1]) & 0xFF
        return checksum == data[-1]

    def parse_aoa_nodeframe0(self, data: bytes) -> Dict[str, Any]:
        """
        解析AOA节点帧0数据
        类似于C代码中的UnpackData函数
        """
        if len(data) < 21 or data[0] != 0x55 or data[1] != 0x07:
            return {"error": "数据格式错误或长度不足"}

        frame_length = data[2] | (data[3] << 8)
        if len(data) < frame_length:
            return {"error": f"数据长度不足，需要{frame_length}字节，实际{len(data)}字节"}

        if not self.verify_checksum(data[:frame_length]):
            return {"error": "校验和错误"}

        result = {
            "role": data[4],
            "role_name": self.LINKTRACK_ROLE.get(data[4], "未知"),
            "id": data[5],
            "local_time": struct.unpack("<I", data[6:10])[0],
            "system_time": struct.unpack("<I", data[10:14])[0],
            "voltage": struct.unpack("<H", data[18:20])[0] / self.MULTIPLY_VOLTAGE,
            "valid_node_count": data[20],
            "nodes": []
        }

        node_size = 12
        for i in range(result["valid_node_count"]):
            offset = 21 + i * node_size
            if offset + node_size > len(data):
                break

            node_data = data[offset:offset + node_size]
            node = {
                "role": node_data[0],
                "role_name": self.LINKTRACK_ROLE.get(node_data[0], "未知"),
                "id": node_data[1],
                "dis": self.parse_int24(node_data, 2) / self.MULTIPLY_DIS,
                "angle": struct.unpack("<h", node_data[5:7])[0] / self.MULTIPLY_ANGLE,
                "fp_rssi": node_data[7] / self.MULTIPLY_RSSI,
                "rx_rssi": node_data[8] / self.MULTIPLY_RSSI
            }
            result["nodes"].append(node)

        return result

    def find_frame_header(self, buffer: bytes) -> int:
        """
        在缓冲区中查找帧头
        """
        for i in range(len(buffer) - 1):
            if buffer[i:i + 2] == b'\x55\x07':
                return i
        return -1

    def read_serial_data(self, ser: serial.Serial, timeout: float = 0.1) -> bytes:
        """
        从串口读取一帧完整的数据
        """
        buffer = bytearray()
        start_time = time.time()

        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

                header_pos = self.find_frame_header(buffer)
                if header_pos >= 0:
                    if header_pos > 0:
                        buffer = buffer[header_pos:]

                    if len(buffer) >= 4:
                        frame_length = buffer[2] | (buffer[3] << 8)
                        if len(buffer) >= frame_length:
                            return bytes(buffer[:frame_length])

            time.sleep(0.01)  # 短暂休眠，避免CPU占用过高

        return bytes()

    def print_result(self, result: Dict[str, Any]) -> None:
        """
        打印解析结果
        """
        if "error" in result:
            print(f"解析错误: {result['error']}")
            return

        print("\n解析结果:")
        print(f"角色: {result['role_name']} (ID: {result['id']})")
        print(f"本地时间: {result['local_time']}")
        print(f"系统时间: {result['system_time']}")
        print(f"电压: {result['voltage']:.2f}V")
        print(f"有效节点数: {result['valid_node_count']}")

        for i, node in enumerate(result["nodes"]):
            print(f"\n节点 {i + 1}:")
            print(f"  角色: {node['role_name']} (ID: {node['id']})")
            print(f"  距离: {node['dis']:.3f}m")
            print(f"  角度: {node['angle']:.2f}°")
            print(f"  FP RSSI: {node['fp_rssi']:.1f}dBm")
            print(f"  RX RSSI: {node['rx_rssi']:.1f}dBm")


class AoaReader:
    """
    串口读取类
    """

    def __init__(self):
        self.Aoa_RunFlag = threading.Event()  # 使用Event代替布尔值
        self.port = '/dev/ttyACM0'
        self.baud = 921600
        self.timeout = 0.01
        self.parser = AoaNodeFrame0Parser()
        self.thread = None
        self.result = None
        self.result_event = threading.Event()  # 用于通知主线程结果已更新

    def start_server(self):
        self.Aoa_RunFlag.set()  # 设置运行标志
        self.thread = threading.Thread(target=self.sync_server)
        self.thread.start()

    def sync_server(self):
        loop = asyncio.new_event_loop()  # 创建新的事件循环
        asyncio.set_event_loop(loop)  # 将新的事件循环设置为当前线程的默认事件循环
        loop.run_until_complete(self.read_and_parse())

    async def read_and_parse(self):
        """
        从串口读取数据并解析
        """
        try:
            ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            print(f"成功打开串口 {self.port}，波特率 {self.baud}")

            while self.Aoa_RunFlag.is_set():
                try:
                    frame_data = self.parser.read_serial_data(ser, self.timeout)

                    if frame_data:
                        hex_data = ' '.join([f"{b:02X}" for b in frame_data])
                        # print(f"\n接收到数据: {hex_data}")
                        # print(f"数据长度: {len(frame_data)} 字节")

                        self.result = self.parser.parse_aoa_nodeframe0(frame_data)
                        self.result_event.set()  # 通知主线程结果已更新

                except KeyboardInterrupt:
                    print("\n用户中断，退出程序")
                    break
                except Exception as e:
                    print(f"处理数据时出错: {e}")
                    time.sleep(1)

        except serial.SerialException as e:
            print(f"打开串口 {self.port} 失败: {e}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
                print(f"串口 {self.port} 已关闭")

    def stop_server(self):
        self.Aoa_RunFlag.clear()  # 清除运行标志
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()  # 等待线程结束
        self.thread = None  # 确保线程对象被正确清理


if __name__ == "__main__":
    aoa = AoaReader()
    aoa.start_server()
    command = [0., 0., 0.]
    try:
        while True:
            if aoa.result_event.is_set():  # 检查是否有新的结果
                aoa.result_event.clear()  # 清除事件
                print(aoa.result)
                if aoa.result['nodes']:
                    command[0] = aoa.result['nodes'][0]['dis']
                    command[2] = aoa.result['nodes'][0]['angle'] * 3.14 / 180.0
                    print(command)  # 打印结果
            time.sleep(0.1)  # 避免CPU占用过高
    except KeyboardInterrupt:
        print("\n用户中断，停止程序")
    finally:
        aoa.stop_server()  # 停止后台线程