import os
import configparser


def load_config():
    config_file = os.path.join('config', 'config.ini')
    config = configparser.ConfigParser()
    config.read(config_file)
    return config
