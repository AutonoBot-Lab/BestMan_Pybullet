import math

from yacs.config import CfgNode as CN


def parse_list(yaml_list):
    python_list = [eval(i) if isinstance(i, str) else i for i in yaml_list]
    return python_list


def parse_cfg(cfg_node):
    for key, value in cfg_node.items():
        if isinstance(value, CN):
            parse_cfg(value)
        elif isinstance(value, list):
            cfg_node[key] = parse_list(value)


def load_config(config_path=None):
    default_config_path = "../Config/default.yaml"
    with open(default_config_path, "r") as file:
        cfg = CN.load_cfg(file)
    if config_path is not None:  # merge cfg
        cfg.merge_from_file(config_path)
    parse_cfg(cfg)
    return cfg
