"""Utility functions for configuration and setup."""

import json
import os

def load_config():
    """Load app configuration from sim_config.json."""
    config_path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "sim_config.json"
    )
    
    with open(config_path, 'r') as f:
        config_data = json.load(f)
    
    app_config = config_data["app"]
    backend = os.environ.get("RMPLAB_BACKEND", app_config["backend"]).lower()
    
    return {
        "mode": app_config["mode"],
        "backend": backend,
        "config_data": config_data
    }

def load_sim_config():
    """Load sim-specific parameters from sim_config.json."""
    config_path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "sim_config.json"
    )
    with open(config_path, 'r') as f:
        return json.load(f)
