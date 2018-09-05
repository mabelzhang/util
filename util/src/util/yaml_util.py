#!/usr/bin/env python

# Mabel Zhang
# 3 Sep 2018
#
# Ref: https://stackoverflow.com/questions/1773805/how-can-i-parse-a-yaml-file-in-python

from yaml import safe_load, YAMLError


# Load YAML file
# Parameters:
#   config_path: Full path to yaml file
def load_yaml (yaml_path):

  with open (yaml_path, 'r') as yaml_file:
    try:
      yaml_dict = safe_load (yaml_file)
    except YAMLError as ex:
      print (ex)

  return yaml_dict




