"""
Tests for the QoS profiles in the Python ROS engine.
"""

import pytest
from pyros2.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TestQoS:
    """Test cases for QoS profiles."""
    
    def test_qos_profile_creation(self):
        """Test QoS profile creation with default values."""
        qos = QoSProfile()
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.VOLATILE
        assert qos.depth == 10
        
    def test_qos_profile_custom_values(self):
        """Test QoS profile creation with custom values."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=5
        )
        assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.depth == 5
        
    def test_qos_profile_predefined_profiles(self):
        """Test predefined QoS profiles."""
        from pyros2.qos import (
            QOS_PROFILE_DEFAULT,
            QOS_PROFILE_SYSTEM_DEFAULT,
            QOS_PROFILE_SENSOR_DATA,
            QOS_PROFILE_PARAMETERS,
            QOS_PROFILE_SERVICES
        )
        
        # Test default profiles
        assert QOS_PROFILE_DEFAULT.reliability == ReliabilityPolicy.RELIABLE
        assert QOS_PROFILE_DEFAULT.durability == DurabilityPolicy.VOLATILE
        assert QOS_PROFILE_DEFAULT.depth == 10
        
        assert QOS_PROFILE_SYSTEM_DEFAULT.reliability == ReliabilityPolicy.RELIABLE
        assert QOS_PROFILE_SYSTEM_DEFAULT.durability == DurabilityPolicy.VOLATILE
        assert QOS_PROFILE_SYSTEM_DEFAULT.depth == 10
        
        # Test sensor data profile
        assert QOS_PROFILE_SENSOR_DATA.reliability == ReliabilityPolicy.BEST_EFFORT
        assert QOS_PROFILE_SENSOR_DATA.durability == DurabilityPolicy.VOLATILE
        assert QOS_PROFILE_SENSOR_DATA.depth == 5
        
        # Test parameters profile
        assert QOS_PROFILE_PARAMETERS.reliability == ReliabilityPolicy.RELIABLE
        assert QOS_PROFILE_PARAMETERS.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert QOS_PROFILE_PARAMETERS.depth == 1000
        
        # Test services profile
        assert QOS_PROFILE_SERVICES.reliability == ReliabilityPolicy.RELIABLE
        assert QOS_PROFILE_SERVICES.durability == DurabilityPolicy.VOLATILE
        assert QOS_PROFILE_SERVICES.depth == 10
        
    def test_qos_profile_validation(self):
        """Test QoS profile validation."""
        # Test valid profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # Should not raise any exceptions
        
        # Test invalid reliability type
        with pytest.raises(TypeError):
            QoSProfile(
                reliability="invalid",
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
            
        # Test invalid durability type
        with pytest.raises(TypeError):
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability="invalid",
                depth=10
            )
            
        # Test invalid depth type
        with pytest.raises(TypeError):
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth="invalid"
            )
            
        # Test negative depth
        with pytest.raises(TypeError):
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=-1
            )