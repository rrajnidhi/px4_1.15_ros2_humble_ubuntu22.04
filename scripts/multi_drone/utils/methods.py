#!/usr/bin/env python3
import yaml
import os
import logging


def load_yaml_params(file_path: str) -> dict:
    """
    Load parameters from a YAML file with error logging.

    Args:
        file_path (str): Path to the YAML configuration file.

    Returns:
        dict: Parsed YAML content.

    Raises:
        FileNotFoundError: If the file does not exist.
        yaml.YAMLError: If the file contains invalid YAML.
    """
    if not os.path.isfile(file_path):
        logging.error(f"YAML file not found: {file_path}")
        raise FileNotFoundError(f"YAML file not found: {file_path}")

    logging.info(f"Loading YAML configuration: {file_path}")
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file) or {}
    except yaml.YAMLError as e:
        logging.error(f"Invalid YAML format in {file_path}: {e}")
        raise
