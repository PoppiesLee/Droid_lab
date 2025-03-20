import json
import os
import yaml
import shutil


class PolicyManager:
    def __init__(self, policies_folder_path: str, policies_destination_path: str):
        """
        初始化 PolicyManager 类。

        :param policies_folder_path: 包含策略文件和配置文件的源文件夹路径。
        :param policies_destination_path: 目标文件夹路径，用于保存拷贝的策略文件和配置文件。
        """
        self.policies_folder_path = policies_folder_path
        self.policies_destination_path = policies_destination_path

    def copy_policy_file(self):
        """
        从源文件夹拷贝策略文件到目标文件夹。
        """
        policies_source_path = os.path.join(self.policies_folder_path, "exported", "policy.onnx")
        policies_destination = os.path.join(self.policies_destination_path, "policy.onnx")

        # 检查源文件是否存在
        if not os.path.exists(policies_source_path):
            print(f"源文件不存在: {policies_source_path}")
        else:
            try:
                # 拷贝文件
                shutil.copy(policies_source_path, policies_destination)
                print(f"策略文件已成功拷贝到: {policies_destination}")
            except Exception as e:
                print(f"拷贝文件时出错: {e}")

    def remove_slice(self, dictionary: dict) -> dict:
        """
        递归移除字典中包含 'slice' 的值。

        :param dictionary: 输入字典。
        :return: 修改后的字典。
        """
        for key, value in dictionary.items():
            if isinstance(value, dict):
                self.remove_slice(value)
            else:
                if "slice" in str(value):
                    dictionary[key] = None
        return dictionary

    def load_local_cfg(self) -> dict:
        """
        加载并处理本地配置文件。

        :return: 处理后的配置字典。
        """
        env_cfg_yaml_path = os.path.join(self.policies_folder_path, "params", "env.yaml")
        if not os.path.exists(env_cfg_yaml_path):
            print(f"配置文件不存在: {env_cfg_yaml_path}")
            return {}

        # 加载 YAML 文件
        with open(env_cfg_yaml_path, "r") as yaml_in:
            env_cfg = yaml.load(yaml_in, Loader=yaml.Loader)

        # 处理配置文件
        env_cfg = self.remove_slice(env_cfg)
        return env_cfg

    def save_cfg_as_json(self):
        """
        将处理后的配置文件保存为 JSON 格式。
        """
        env_cfg = self.load_local_cfg()
        cfg_save_path = os.path.join(self.policies_destination_path, "env_cfg.json")

        with open(cfg_save_path, "w") as fp:
            json.dump(env_cfg, fp, indent=4)
        print(f"配置文件已成功保存到: {cfg_save_path}")

    def run(self):
        """
        执行所有操作：拷贝策略文件和保存配置文件。
        """
        self.copy_policy_file()
        self.save_cfg_as_json()


# 使用示例
if __name__ == "__main__":
    policies_folder_path = "../../logs/x02a_flat/2025-03-19_22-38-56/"
    policies_destination_path = "../policies"

    manager = PolicyManager(policies_folder_path, policies_destination_path)
    manager.run()