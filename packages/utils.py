import os
import configparser



def load_config() -> dict:
    config_file = os.path.join('config', 'config.ini')
    config = configparser.ConfigParser()
    config.read(config_file)
    config = parse_config(config)
    return config


def parse_config(config):
    conf_dict = {}
    for section in config.sections():
        conf_dict[section] = {}
        for key in config[section]:
            item = eval(config[section][key]) 
            conf_dict[section][key] = item
            
    return conf_dict

    
    