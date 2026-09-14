import pytest
from run_fixed_group_imu_comparison import settings_for_dataset
from run_unified_imu_comparison import validate_package, METHODS
from test_unified_comparison import package


def test_fixed_rounded_settings_apply_to_entire_groups():
    for name in ('MH01','MH02','MH03','MH04','MH05'):
        assert settings_for_dataset(name) == dict(alpha=9.85,integration_covariance=4e-5)
    for name in ('V101','V102','V103','V201','V202','V203'):
        assert settings_for_dataset(name) == dict(alpha=16.,integration_covariance=0.)
    with pytest.raises(ValueError, match='Unknown'):
        settings_for_dataset('unknown')
    result=settings_for_dataset('MH01');result['alpha']=0
    assert settings_for_dataset('MH01')['alpha']==9.85


def test_explicit_configuration_validation(package):
    folder,sources=package
    with pytest.raises(ValueError,match='exactly one'):
        validate_package(folder,sources,METHODS,expected_configs={})
    with pytest.raises(ValueError,match='configuration'):
        validate_package(folder,sources,METHODS,expected_configs={'MH01':'wrong'})
