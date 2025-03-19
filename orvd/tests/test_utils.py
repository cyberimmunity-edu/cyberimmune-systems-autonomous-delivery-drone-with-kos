import math
import pytest 
from utils.utils import *

from unittest.mock import patch, MagicMock
from utils.utils import generate_keys

mock_random_generator = MagicMock()
mock_key = MagicMock()

TEST_KEY_GROUP = 'orvd'
generate_keys(ORVD_KEY_SIZE, TEST_KEY_GROUP)


def test_get_sha256_hex_empty_string():
    message = ""
    expected_hash = sha256(message.encode()).hexdigest()
    assert get_sha256_hex(message) == expected_hash

def test_get_sha256_hex_hello_world():
    message = "hello world"
    expected_hash = sha256(message.encode()).hexdigest()
    assert get_sha256_hex(message) == expected_hash


def test_parse_mission():
    mission = "T100&H55.7558_37.6173_200&W10_55.7558_37.6173_150"
    expected = [['T', '100'], ['H', '55.7558', '37.6173', '200'], ['W', '10', '55.7558', '37.6173', '150']]
    assert parse_mission(mission) == expected

def test_read_mission():
    file_str = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t55.7558\t37.6173\t200\t1\n"
    expected = [['H', '55.7558', '37.6173', '200.0']]
    assert read_mission(file_str)[0] == expected

def test_home_handler():
    lat, lon, alt = 55.7558, 37.6173, 200.0
    expected = ['H', '55.7558', '37.6173', '200.0']
    assert home_handler(lat, lon, alt) == expected

def test_takeoff_handler():
    alt = 100.0
    expected = ['T', '100.0']
    assert takeoff_handler(alt) == expected

def test_waypoint_handler():
    lat, lon, alt = 55.7558, 37.6173, 150.0
    expected = ['W', '55.7558', '37.6173', '150.0']
    assert waypoint_handler(lat, lon, alt) == expected

def test_servo_handler():
    number, pwm = 1.0, 1500.0
    expected = ['S', '1.0', '1500.0']
    assert servo_handler(number, pwm) == expected

def test_land_handler():
    lat, lon, alt = 55.7558, 37.6173, 0
    home = ['H', '55.7558', '37.6173', '200.0']
    expected = ['L', '55.7558', '37.6173', '200.0']
    assert land_handler(lat, lon, alt, home) == expected

def test_encode_mission():
    mission_list = [['T', '100'], ['H', '55.7558', '37.6173', '200'], ['W', '10', '55.7558', '37.6173', '150']]
    expected = ['T100', 'H55.7558_37.6173_200', 'W10_55.7558_37.6173_150']
    assert encode_mission(mission_list) == expected


@patch('utils.utils.Random.new')
@patch('utils.utils.RSA.generate')
def test_generate_keys_success(mock_rsa_generate, mock_random_new):
    mock_random_new.return_value.read = mock_random_generator
    mock_rsa_generate.return_value = mock_key

    keysize = 2048
    key_group = 'new_test_group'

    generate_keys(keysize, key_group)

    mock_random_new.assert_called_once()
    mock_rsa_generate.assert_called_once_with(keysize, mock_random_generator)
    assert loaded_keys[key_group] == mock_key

def test_generate_keys_invalid_keysize():
    keysize = -1

    with pytest.raises(ValueError):
        generate_keys(keysize, 'new_test_group')

def test_get_key_private_true_existing_group():
    key = get_key(TEST_KEY_GROUP, private=True)
    assert key == loaded_keys[TEST_KEY_GROUP]

def test_get_key_private_true_non_existing_group():
    key = get_key('non_existing_group', private=True)
    assert key is None
    
def test_get_key_wrong_group():
    key = get_key('wrong_group', private=False)
    assert key == -1
    
def test_get_key_orvd():
    key = get_key(TEST_KEY_GROUP, private=False)
    assert key != None

def test_sign():
    message = "This is a test message"
    
    signature = sign(message, TEST_KEY_GROUP)
    
    assert isinstance(signature, int)

def test_verify():
    message = "This is a test message"
    
    signature = sign(message, TEST_KEY_GROUP)
    
    assert verify(message, signature, TEST_KEY_GROUP) == True

def test_verify_invalid_signature():
    message = "This is a test message"
    invalid_signature = 1234567890
    
    assert verify(message, invalid_signature, TEST_KEY_GROUP) == False

def test_verify_modified_message():
    message = "This is a test message"
    modified_message = "This is a modified message"
    
    signature = sign(message, TEST_KEY_GROUP)
    
    assert verify(modified_message, signature, TEST_KEY_GROUP) == False
    

def test_haversine_zero_distance():
    assert 0 == haversine(50, 50, 50, 50)

def test_haversine_known_distance():
    nyc_lat, nyc_lon = 40.7128, -74.0060
    la_lat, la_lon = 34.0522, -118.2437
    expected_distance = 3935746.254
    assert math.isclose(haversine(nyc_lat, nyc_lon, la_lat, la_lon), expected_distance, rel_tol=1e-3)

def test_haversine_poles():
    north_pole_lat, north_pole_lon = 90, 0
    south_pole_lat, south_pole_lon = -90, 0
    expected_distance = 20015086.796
    assert math.isclose(haversine(north_pole_lat, north_pole_lon, south_pole_lat, south_pole_lon), expected_distance, rel_tol=1e-3)


def test_cast_wrapper_int_success():
    assert cast_wrapper("123", int) == 123

def test_cast_wrapper_int_failure():
    assert cast_wrapper("abc", int) is None

def test_cast_wrapper_float_success():
    assert cast_wrapper("123.45", float) == 123.45

def test_cast_wrapper_float_failure():
    assert cast_wrapper("abc", float) is None

def test_cast_wrapper_none_element():
    assert cast_wrapper(None, int) is None

def test_cast_wrapper_str_success():
    assert cast_wrapper(123, str) == "123"


def test_get_new_polygon_feature():
    name = "Test Polygon"
    coordinates = [
        [30.0, 10.0],
        [40.0, 40.0],
        [20.0, 40.0],
        [10.0, 20.0],
        [30.0, 10.0]
    ]
    
    expected_output = {
        "type": "Feature",
        "properties": {
            "name": name
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [
                coordinates
            ]
        }
    }
    
    result = get_new_polygon_feature(name, coordinates)
    
    assert result == expected_output


def test_point_inside_polygon():
    polygon = [(0, 0), (4, 0), (4, 4), (0, 4)]
    point = (2, 2)
    assert is_point_in_polygon(point, polygon) == True

def test_point_outside_polygon():
    polygon = [(0, 0), (4, 0), (4, 4), (0, 4)]
    point = (5, 5)
    assert is_point_in_polygon(point, polygon) == False

def test_point_on_vertex():
    polygon = [(0, 0), (4, 0), (4, 4), (0, 4)]
    point = (4, 4)
    assert is_point_in_polygon(point, polygon) == True

def test_point_on_horizontal_edge():
    polygon = [(0, 0), (4, 0), (4, 4), (0, 4)]
    point = (2, 4)
    assert is_point_in_polygon(point, polygon) == True

def test_point_on_vertical_edge():
    polygon = [(0, 0), (4, 0), (4, 4), (0, 4)]
    point = (4, 2)
    assert is_point_in_polygon(point, polygon) == True