"""
Hydra configuration system for the Python ROS engine.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from hydra.core.config_store import ConfigStore
from omegaconf import DictConfig, OmegaConf

from pyros2.qos import QoSProfile


@dataclass
class NodeConfig:
    """
    Configuration for a ROS node.
    """

    name: str = "default_node"
    namespace: str = "/"


@dataclass
class PublisherConfig:
    """
    Configuration for a ROS publisher.
    """

    topic: str = "default_topic"
    qos: QoSProfile = field(default_factory=QoSProfile)


@dataclass
class SubscriberConfig:
    """
    Configuration for a ROS subscriber.
    """

    topic: str = "default_topic"
    qos: QoSProfile = field(default_factory=QoSProfile)


@dataclass
class ServiceConfig:
    """
    Configuration for a ROS service.
    """

    name: str = "default_service"
    qos: QoSProfile = field(default_factory=QoSProfile)


@dataclass
class ParameterConfig:
    """
    Configuration for ROS parameters.
    """

    name: str = ""
    value: Any = None
    type: str = "string"


@dataclass
class BridgeConfig:
    """
    Configuration for ROS bridge.
    """

    host: str = "localhost"
    port: int = 11311  # Default ROS master port
    protocol: str = "xmlrpc"  # ROS1 uses XMLRPC


@dataclass
class ROSConfig:
    """
    Main configuration class for the Python ROS engine.
    """

    node: NodeConfig = field(default_factory=NodeConfig)
    publisher: Optional[PublisherConfig] = None
    subscriber: Optional[SubscriberConfig] = None
    service: Optional[ServiceConfig] = None
    parameters: Dict[str, ParameterConfig] = field(default_factory=dict)
    bridge: BridgeConfig = field(default_factory=BridgeConfig)


def load_config(config_path: str) -> DictConfig:
    """
    Load configuration from a YAML file using Hydra.

    Args:
        config_path: Path to the configuration file

    Returns:
        DictConfig: Loaded configuration
    """
    # Register the config structure with Hydra
    cs = ConfigStore.instance()
    cs.store(name="ros_config", node=ROSConfig)

    # Load the configuration
    cfg = OmegaConf.load(config_path)
    return cfg


def save_config(config: DictConfig, config_path: str) -> None:
    """
    Save configuration to a YAML file.

    Args:
        config: Configuration to save
        config_path: Path to save the configuration file
    """
    OmegaConf.save(config=config, f=config_path)
