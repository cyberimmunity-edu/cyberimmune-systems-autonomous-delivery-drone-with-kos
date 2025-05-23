import math
import os, sys
import time
import json
import ast
import csv
from io import StringIO
from hashlib import sha256
from Cryptodome import Random
from Cryptodome.PublicKey import RSA
from models import *
from utils.db_utils import *


ORVD_KEY_SIZE = 1024

ARMED = 0
DISARMED = 1

KILL_SWITCH_OFF = 1
KILL_SWITCH_ON = 0

MISSION_ACCEPTED = 0
MISSION_NOT_ACCEPTED = 1

NOT_FOUND = '$-1'
OK = '$OK'

LOGS_PATH = './logs'
FORBIDDEN_ZONES_PATH = './static/resources/forbidden_zones.json'
FORBIDDEN_ZONES_DELTA_PATH = './static/resources/forbidden_zones_delta.json'

loaded_keys = {}

class MissionVerificationStatus:
    OK = 'Mission accepted.'
    NON_ZERO_DELAY_WAYPOINT = 'Error: The mission contains a waypoint with non-zero delay.'
    WRONG_DELAY = 'Error: Delay in the mission can contain only one parameter (delay in seconds).'
    UNKNOWN_COMMAND = 'Error: The mission contains an unknown command. Allowed commands: 16, 21, 22, 93, 183.'
    


def get_sha256_hex(message: str) -> str:
    """
    Вычисляет хеш SHA-256 для заданного сообщения и возвращает хэш в виде шестнадцатеричной строки.

    Args:
        message (str): Входное сообщение для хеширования.

    Returns:
        str: Хеш SHA-256 входного сообщения в виде шестнадцатеричной строки, без префикса '0x'.
    """
    return hex(int.from_bytes(sha256(message.encode()).digest(), byteorder='big', signed=False))[2:]


def parse_mission(mission: str) -> list:
    """
    Разбирает строку миссии на список команд.

    Args:
        mission (str): Строка миссии.

    Returns:
        list: Список команд миссии.
    """
    cmds = mission.split('&')
    for idx, cmd in enumerate(cmds):
        cmds[idx] = [cmd[0], *cmd[1:].split('_')]
    return cmds


def read_mission(file_str: str) -> tuple[list | None, str]:
    """
    Читает миссию из строки файла и преобразует её в список команд.

    Args:
        file_str (str): Содержимое файла миссии.

    Returns:
        tuple: Кортеж, содержащий список команд миссии и статус верификации миссии.

    Raises:
        Exception: Если файл не поддерживается версией WP.
    """
    missionlist=[]
    split_str = '\r\n' if '\r' in file_str else '\n'
    for i, line in enumerate(file_str.split(split_str)):
        if line == '':
            break
        if i==0:
            if not line.startswith('QGC WPL 110'):
                raise Exception('File is not supported WP version')
        else:
            linearray=line.split('\t')
            ln_index=int(linearray[0])
            ln_currentwp=int(linearray[1])
            ln_frame=int(linearray[2])
            ln_command=int(linearray[3])
            ln_param1=float(linearray[4])
            ln_param2=float(linearray[5])
            ln_param3=float(linearray[6])
            ln_param4=float(linearray[7])
            ln_param5=float(linearray[8])
            ln_param6=float(linearray[9])
            ln_param7=float(linearray[10])
            #ln_autocontinue=int(linearray[11].strip())
            
            if ln_index == 0 and ln_currentwp == 1 and ln_frame == 0:
                cmd = home_handler(lat=ln_param5, lon=ln_param6, alt=ln_param7)
            elif ln_command == 22:
                cmd = takeoff_handler(alt=ln_param7)
            elif ln_command == 16:
                if ln_param1 == 0.0:
                    cmd = waypoint_handler(lat=ln_param5, lon=ln_param6, alt=ln_param7)
                else:
                    return None, MissionVerificationStatus.NON_ZERO_DELAY_WAYPOINT
            elif ln_command == 183:
                cmd = servo_handler(number=ln_param1, pwm=ln_param2)
            elif ln_command == 21:
                if len(missionlist) != 0 and missionlist[0][0] == 'H':
                    drone_home = missionlist[0]
                else:
                    drone_home = None
                cmd = land_handler(lat=ln_param5, lon=ln_param6, alt=ln_param7, home=drone_home)
            elif ln_command == 93:
                if ln_param2 == ln_param3 == ln_param4 == 0.0:
                    cmd = delay_handler(delay=ln_param1)
                else:
                    return None, MissionVerificationStatus.WRONG_DELAY
            else:
                return None, MissionVerificationStatus.UNKNOWN_COMMAND
            
            missionlist.append(cmd)
    return missionlist, MissionVerificationStatus.OK


def home_handler(lat: float, lon: float, alt: float) -> list:
    """
    Обрабатывает команду установки домашней позиции.

    Args:
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.

    Returns:
        list: Команда домашней позиции.
    """
    lat = round(lat, 7)
    lon = round(lon, 7)
    alt = round(alt, 2)
    return ['H', str(lat), str(lon), str(alt)]


def takeoff_handler(alt: float) -> list:
    """
    Обрабатывает команду взлёта.

    Args:
        alt (float): Высота взлёта.

    Returns:
        list: Команда взлёта.
    """
    alt = round(alt, 2)
    return ['T', str(alt)]


def waypoint_handler(lat: float, lon: float, alt: float) -> list:
    """
    Обрабатывает команду путевой точки.

    Args:
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.

    Returns:
        list: Команда путевой точки.
    """
    lat = round(lat, 7)
    lon = round(lon, 7)
    alt = round(alt, 2)
    return ['W', str(lat), str(lon), str(alt)]


def servo_handler(number: float, pwm: float) -> list:
    """
    Обрабатывает команду управления сервоприводом.

    Args:
        number (float): Номер сервопривода.
        pwm (float): Значение ШИМ.

    Returns:
        list: Команда управления сервоприводом.
    """
    return ['S', str(number), str(pwm)]


def land_handler(lat: float, lon: float, alt: float, home: list = None) -> list:
    """
    Обрабатывает команду посадки.

    Args:
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.
        home (list, optional): Домашняя позиция. По умолчанию None.

    Returns:
        list: Команда посадки.
    """
    if home == None:
        ret_lat = lat
        ret_lon = lon
        ret_alt = alt
    else:
        ret_lat = float(home[1]) if lat == 0. else lat
        ret_lon = float(home[2]) if lon == 0. else lon
        ret_alt = float(home[3]) if alt == 0. else alt
    
    ret_lat = round(ret_lat, 7)
    ret_lon = round(ret_lon, 7)
    ret_alt = round(ret_alt, 2)
    
    return ['L', str(ret_lat), str(ret_lon), str(ret_alt)]


def delay_handler(delay: int) -> list:
    """
    Обрабатывает команду задержки.

    Args:
        delay (int): Время задержки в секундах.

    Returns:
        list: Команда задержки.
    """
    return ['D', str(delay)]


def encode_mission(mission_list: list) -> list:
    """
    Кодирует список команд миссии в строковый формат.

    Args:
        mission_list (list): Список команд миссии.

    Returns:
        list: Закодированный список команд миссии.
    """
    for idx, cmd in enumerate(mission_list):
        mission_list[idx] = f'{cmd[0]}' + '_'.join(cmd[1:])
    
    return mission_list


def sign(message: str, key_group: str) -> int:
    """
    Подписывает сообщение с использованием приватного ключа.

    Args:
        message (str): Сообщение для подписи.
        key_group (str): Группа ключей.

    Returns:
        int: Цифровая подпись.
    """
    key = get_key(key_group, private=True)
    n, d = key.n, key.d
    msg_bytes = message.encode()
    hash = int.from_bytes(sha256(msg_bytes).digest(), byteorder='big', signed=False)
    signature = pow(hash, d, n)
    
    return signature


def verify(message: str, signature: int, key_group: str) -> bool:
    """
    Проверяет подпись сообщения.
    
    Args:
        message (str): Проверяемое сообщение.
        signature (int): Цифровая подпись.
        key_group (str): Группа ключей.

    Returns:
        bool: True, если подпись верна, иначе False.
    """
    try:
        key_set = get_key(key_group, private=False)
        if len(key_set) == 2:
            n, e = key_set
        else:
            return False
        msg_bytes = message.encode()
        hash = int.from_bytes(sha256(msg_bytes).digest(), byteorder='big', signed=False)
        hashFromSignature = pow(signature, e, n)
        return hash == hashFromSignature
    except:
        return False


def mock_verifier(*args, **kwargs):
    """
    Мок-функция для проверки подписи. Всегда возвращает True.

    Returns:
        bool: True
    """
    return True


def get_key(key_group: str, private: bool):
    """
    Получает ключ из указанной группы.

    Args:
        key_group (str): Группа ключей.
        private (bool): Флаг для получения приватного ключа.

    Returns:
        Ключ или кортеж (n, e) для публичного ключа, или -1 в случае ошибки.
    """
    if private == True:
        if key_group in loaded_keys:
            return loaded_keys[key_group]
        else:
            return None
    
    else:
        if 'kos' in key_group:
            id = key_group.split('kos')[1]
            key = get_entity_by_key(UavPublicKeys, id)
            if key == None:
                return -1
            n, e = int(key.n), int(key.e)
            
        elif 'ms' in key_group:
            id = key_group.split('ms')[1]
            key = get_entity_by_key(MissionSenderPublicKeys, id)
            if key == None:
                return -1
            n, e = int(key.n), int(key.e)
        
        elif key_group == 'orvd':
            key = loaded_keys[key_group].publickey()
            n, e = key.n, key.e
            
        else:
            print('Wrong group')
            return -1
        
        return n, e


def generate_keys(keysize: int, key_group: str) -> list:
    """
    Генерирует пару ключей RSA.

    Args:
        keysize (int): Размер ключа.
        key_group (str): Группа ключей.

    Returns:
        list: Сгенерированные ключи.
    """
    random_generator = Random.new().read
    key = RSA.generate(keysize, random_generator)
    loaded_keys[key_group] = key


def save_public_key(n: str, e: str, key_group: str) -> None:
    """
    Сохраняет публичный ключ в базу данных.

    Args:
        n (str): Модуль ключа.
        e (str): Открытая экспонента.
        key_group (str): Группа ключей.
    """
    if 'kos' in key_group:
        id = key_group.split('kos')[1]
        entity = UavPublicKeys(uav_id=id, n=n, e=e)
    elif 'ms' in key_group:
        id = key_group.split('ms')[1]
        entity = MissionSenderPublicKeys(uav_id=id, n=n, e=e)
    else:
        print('Wrong group in utils.save_public_key')
    add_and_commit(entity)
    

def haversine(lat1, lon1, lat2, lon2):
    """
    Вычисляет расстояние между двумя точками на сфере по формуле гаверсинуса.

    Args:
        lat1, lon1 (float): Координаты первой точки.
        lat2, lon2 (float): Координаты второй точки.

    Returns:
        float: Расстояние в метрах.
    """
    R = 6366037  # radius of Earth in meters
    phi_1 = math.radians(lat1)
    phi_2 = math.radians(lat2)

    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    meters = R * c
    meters = round(meters, 3)
    return meters


def cast_wrapper(element, cast_function):
    """
    Обёртка для безопасного приведения типов.
    
    Args:
        element: Элемент для приведения типа.
        cast_function: Функция приведения типа.

    Returns:
        Результат приведения типа или None в случае ошибки.
    """
    if element is None: 
        return None
    try:
        return cast_function(element)
    except ValueError:
        return None
    

def get_new_polygon_feature(name, coordinates):
    """
    Создаёт новый объект полигона для GeoJSON.

    Args:
        name (str): Имя полигона.
        coordinates (list): Список координат полигона.

    Returns:
        dict: Объект полигона в формате GeoJSON.
    """
    new_feature = {
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
    return new_feature


def is_point_in_polygon(point, polygon):
    """
    Проверяет, находится ли точка внутри полигона.

    Args:
        point (tuple): Координаты точки (x, y).
        polygon (list): Список координат вершин полигона.

    Returns:
        bool: True, если точка внутри полигона, иначе False.
    """
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def create_csv_from_telemetry(telemetry_data):
    """
    Создает CSV-строку из телеметрических данных.

    Args:
        telemetry_data (list): Список телеметрических данных.

    Returns:
        str: CSV-строка с телеметрическими данными.
    """
    output = StringIO()
    writer = csv.writer(output)
    writer.writerow(['record_time', 'lat', 'lon', 'alt', 'azimuth', 'dop', 'sats', 'speed'])

    for telemetry in telemetry_data:
        writer.writerow([telemetry.record_time, telemetry.lat, telemetry.lon, telemetry.alt, telemetry.azimuth, telemetry.dop, telemetry.sats, telemetry.speed])

    output.seek(0)
    return output.getvalue()


def compute_forbidden_zones_delta(old_zones, new_zones):
    """
    Вычисляет дельту изменений между старыми и новыми запрещенными зонами.

    Args:
        old_zones (dict): Старые запрещенные зоны.
        new_zones (dict): Новые запрещенные зоны.

    Returns:
        dict: Дельта изменений.
    """
    delta_zones = {"type": "FeatureCollection", "features": []}
    
    old_zones_dict = {zone['properties']['name']: zone for zone in old_zones['features']}
    new_zones_dict = {zone['properties']['name']: zone for zone in new_zones['features']}
    
    # Обработка добавленных и измененных зон
    for name, new_zone in new_zones_dict.items():
        if name not in old_zones_dict:
            new_zone['properties']['change_type'] = 'added'
            delta_zones['features'].append(new_zone)
        elif old_zones_dict[name]['geometry'] != new_zone['geometry']:
            new_zone['properties']['change_type'] = 'modified'
            delta_zones['features'].append(new_zone)
    
    # Обработка удаленных зон
    for name, old_zone in old_zones_dict.items():
        if name not in new_zones_dict:
            old_zone['properties']['change_type'] = 'deleted'
            delta_zones['features'].append(old_zone)
    
    return delta_zones


def compute_and_save_forbidden_zones_delta(old_zones, new_zones):
    try:
        delta_zones = compute_forbidden_zones_delta(old_zones, new_zones)
        
        with open(FORBIDDEN_ZONES_DELTA_PATH, 'w', encoding='utf-8') as f:
            json.dump(delta_zones, f, ensure_ascii=False, indent=4)
    except Exception as e:
        print(f"Error computing and saving forbidden zones delta: {e}")
        
        
def generate_forbidden_zones_string(forbidden_zones):
    """
    Генерирует строку запрещенных зон из JSON данных.

    Args:
        forbidden_zones (dict): JSON данные запрещенных зон.

    Returns:
        str: Строка запрещенных зон.
    """
    result_str = f'$ForbiddenZones {len(forbidden_zones["features"])}'
    for zone in forbidden_zones['features']:
        name = zone['properties']['name']
        coordinates = zone['geometry']['coordinates'][0]
        result_str += f'&{name}&{len(coordinates)}&{"&".join(list(map(lambda e: f"{e[1]:.7f}_{e[0]:.7f}", coordinates)))}'
    return result_str
