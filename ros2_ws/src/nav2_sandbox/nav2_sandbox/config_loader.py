"""
Config Loader for Nav2 Sandbox (Isaac Warehouse Project)

Two-level config system:
  1. project_config.yaml  — shared settings (robot name, map, domain_id)
  2. robots/<name>.yaml   — per-robot settings (namespace, frames, topics)

Usage in launch files::

    from nav2_sandbox.config_loader import load_project_config, load_robot_config

    project_cfg = load_project_config()
    robot_cfg = load_robot_config(project_cfg)

    ns = robot_cfg['namespace']
    scan_topic = robot_cfg['topics']['scan']
    nav2_params = get_nav2_params_path(project_cfg)
"""

import os
import yaml
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory


def _get_pkg_share() -> str:
    return get_package_share_directory('nav2_sandbox')


def _get_package_config_path() -> str:
    """Return the installed path to project_config.yaml."""
    return os.path.join(_get_pkg_share(), 'config', 'project_config.yaml')


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Deep-merge *override* into *base*, returning a new dict."""
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_project_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Load the project configuration from project_config.yaml.

    Returns a dict with at least:
        robot: str          — robot name (e.g. 'carter', 'g1', 'anymal')
        navigation:
            use_sim_time: bool
            map_yaml: str
            domain_id: int
    """
    if config_path is None:
        config_path = _get_package_config_path()

    defaults = {
        'robot': 'carter',
        'navigation': {
            'use_sim_time': True,
            'map_yaml': '',
            'domain_id': 47,
        },
    }

    if not os.path.exists(config_path):
        print(f"[nav2_sandbox] Warning: config file not found at {config_path}, "
              "using built-in defaults")
        return defaults

    with open(config_path, 'r') as fh:
        loaded = yaml.safe_load(fh) or {}

    return deep_merge(defaults, loaded)


def load_robot_config(project_config: Dict[str, Any]) -> Dict[str, Any]:
    """Load the per-robot config from config/robots/<name>.yaml.

    Args:
        project_config: The project config dict (must contain 'robot' key).

    Returns:
        Robot config dict with keys: namespace, base_frame, robot_radius,
        type, scan_source, topics, etc.
    """
    robot_name = project_config.get('robot', 'carter')
    pkg_share = _get_pkg_share()
    robot_config_path = os.path.join(pkg_share, 'config', 'robots', f'{robot_name}.yaml')

    if not os.path.exists(robot_config_path):
        raise FileNotFoundError(
            f"[nav2_sandbox] Robot config not found: {robot_config_path}\n"
            f"  Available robots: check config/robots/ directory.\n"
            f"  Set 'robot: <name>' in project_config.yaml to match a file in config/robots/."
        )

    with open(robot_config_path, 'r') as fh:
        robot_cfg = yaml.safe_load(fh) or {}

    print(f"[nav2_sandbox] Loaded robot config: {robot_name} "
          f"(namespace={robot_cfg.get('namespace')}, "
          f"base_frame={robot_cfg.get('base_frame')}, "
          f"type={robot_cfg.get('type')})")

    return robot_cfg


def get_config_value(config: Dict[str, Any], *keys, default: Any = None) -> Any:
    """Retrieve a nested value from config using a sequence of keys.

    Example::
        ns = get_config_value(cfg, 'robot', 'namespace', default='')
    """
    value = config
    for key in keys:
        if isinstance(value, dict) and key in value:
            value = value[key]
        else:
            return default
    return value


def get_map_path(config: Dict[str, Any]) -> str:
    """Resolve the map YAML path from project config.

    Resolution order:
        1. If navigation.map_yaml is an absolute path that exists, return it.
        2. Otherwise treat it as a bare name and look in package maps/ dir.
    """
    map_value = get_config_value(config, 'navigation', 'map_yaml', default='')

    if not map_value:
        return ''

    # Tier 1 -- absolute path
    if os.path.isabs(map_value) and os.path.exists(map_value):
        return map_value

    # Tier 2 -- package maps/ directory
    try:
        pkg_share = _get_pkg_share()
        name = map_value if map_value.endswith('.yaml') else f'{map_value}.yaml'
        local_map = os.path.join(pkg_share, 'maps', name)
        if os.path.exists(local_map):
            return local_map
    except Exception:
        pass

    return map_value


def get_nav2_params_path(project_config: Dict[str, Any]) -> str:
    """Return the path to the per-robot nav2 params file.

    Resolves to: config/nav2_<robot>_params.yaml
    Falls back to nav2_params.yaml if the per-robot file doesn't exist.
    """
    # Check for explicit override
    override = get_config_value(
        project_config, 'navigation', 'nav2_params_file', default=''
    )
    if override and os.path.isabs(override) and os.path.exists(override):
        return override

    pkg_share = _get_pkg_share()
    robot_name = project_config.get('robot', 'carter')

    # Per-robot params file
    per_robot = os.path.join(pkg_share, 'config', f'nav2_{robot_name}_params.yaml')
    if os.path.exists(per_robot):
        return per_robot

    # Fallback to generic (backwards compat)
    generic = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    if os.path.exists(generic):
        print(f"[nav2_sandbox] Warning: no nav2_{robot_name}_params.yaml found, "
              f"using generic nav2_params.yaml")
        return generic

    return per_robot  # return expected path even if missing (will error at load)
