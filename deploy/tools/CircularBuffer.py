import numpy as np

class CircularBuffer:
    def __init__(self, block_size, num_blocks):
        """
        初始化环形缓冲区
        :param block_size: 每个数据块的长度 (n)
        :param num_blocks: 总共存储的数据块数量 (m)
        """
        self.block_size = block_size
        self.num_blocks = num_blocks
        self.capacity = block_size * num_blocks  # 总容量
        self.buffer = np.zeros(self.capacity)  # 初始化缓冲区
        self.index = 0  # 当前写入位置

    def append(self, data_block):
        """
        添加一个数据块
        :param data_block: 长度为 n 的数据块
        """
        if len(data_block) != self.block_size:
            raise ValueError(f"数据块长度必须为 {self.block_size}, 但实际长度为 {len(data_block)}")
        # 写入数据块
        start_index = self.index
        end_index = (self.index + self.block_size) % self.capacity
        if end_index > start_index:  # 不跨越缓冲区末尾
            self.buffer[start_index:end_index] = data_block
        else:  # 跨越缓冲区末尾
            remaining = self.capacity - start_index
            self.buffer[start_index:] = data_block[:remaining]
            self.buffer[:end_index] = data_block[remaining:]
        # 更新索引
        self.index = end_index

    def get(self):
        """
        获取当前缓冲区中的所有数据
        :return: 一个一维数组，表示所有数据块
        """
        if self.index == 0:  # 缓冲区未满
            return self.buffer
        else:  # 缓冲区已满，需要重新排列
            return np.roll(self.buffer, -self.index, axis=0)

    def __repr__(self):
        return str(self.get())

# 示例使用
if __name__ == "__main__":
    n = 3  # 数据块长度
    m = 5  # 数据块数量
    cb = CircularBuffer(block_size=n, num_blocks=m)

    # 添加数据块
    for i in range(1, 20):  # 添加 7 个数据块
        data_block = np.arange(i, i + n)  # 创建一个长度为 n 的数据块
        cb.append(data_block)
        print(cb.get())
        print(f"Added block {i}: {cb}")
    print("the end")