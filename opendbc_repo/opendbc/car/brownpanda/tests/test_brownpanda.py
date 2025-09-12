from opendbc.car import Bus
from opendbc.car.structs import CarParams
from opendbc.car.brownpanda.fingerprints import FW_VERSIONS
from opendbc.car.brownpanda.values import CAR, DBC, FW_QUERY_CONFIG

Ecu = CarParams.Ecu


class TestBrownPandaInterfaces:
  def test_car_sets(self):
    """Test that all car models are defined consistently."""
    assert len(CAR) > 0, "At least one car model should be defined"
    
  def test_fw_versions_complete(self):
    """Test that firmware versions are defined for all car models."""
    for car_model in CAR:
      assert car_model in FW_VERSIONS, f"FW_VERSIONS missing for {car_model}"
      
  def test_dbc_lookup(self):
    """Test that DBC lookup works for all car models."""
    assert DBC is not None
    assert isinstance(DBC, dict)
    assert Bus.pt in DBC, "Primary bus should be defined in DBC"
    
  def test_fw_query_config(self):
    """Test that firmware query configuration is properly defined."""
    assert FW_QUERY_CONFIG is not None
    assert hasattr(FW_QUERY_CONFIG, 'requests')
    assert hasattr(FW_QUERY_CONFIG, 'non_essential_ecus')
    assert hasattr(FW_QUERY_CONFIG, 'extra_ecus')
    
  def test_car_models_consistency(self):
    """Test that all defined car models have consistent naming."""
    for car_model in CAR:
      assert isinstance(car_model, str), f"Car model {car_model} should be a string"
      assert len(car_model) > 0, f"Car model {car_model} should not be empty"