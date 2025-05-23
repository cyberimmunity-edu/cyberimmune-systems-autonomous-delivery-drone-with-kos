import socket
import time
from threading import Thread
from flask import jsonify
from utils.db_utils import *
from utils.utils import *

ENABLE_MAVLINK = False
MAVLINK_CONNECTIONS_NUMBER = 10
OUT_ADDR = 'localhost'

arm_queue = set()
revise_mission_queue = set()
modes = {
    "display_only": False,
    "flight_info_response": True
}

if ENABLE_MAVLINK:
    from pymavlink import mavutil
    
    def mavlink_handler(in_port: int, out_port: int, out_addr: str = 'localhost'):
        mavlink_connection = mavutil.mavlink_connection(f'udp:0.0.0.0:{in_port}')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while True:
            msg = mavlink_connection.recv_msg()
            if msg:
                data = msg.get_msgbuf()
                sock.sendto(data, (out_addr, out_port))
            time.sleep(1e-4)

    def start_mavlink_connections(connections):
        for conn in connections:
            in_port, out_port, out_addr = conn
            thread = Thread(
                name=f"mav_thread_{in_port}_{out_port}",
                target=mavlink_handler,
                args=(in_port, out_port, out_addr),
                daemon=True
            )
            thread.start()
            
    connections = [(in_port, in_port + 20, OUT_ADDR) for in_port in range(14551, 14551 + MAVLINK_CONNECTIONS_NUMBER)]
    start_mavlink_connections(connections)

def bad_request(message: str):
    """
    Возвращает сообщение об ошибке с кодом 400 (Bad Request).

    Args:
        message (str): Сообщение об ошибке.

    Returns:
        tuple: Кортеж с сообщением об ошибке и кодом состояния 400.
    """
    return message, 400


def signed_request(handler_func, verifier_func, signer_func, query_str: str, key_group: str, sig: str, **kwargs):
    """
    Обрабатывает подписанный запрос, проверяя подпись и выполняя указанную функцию-обработчик.

    Args:
        handler_func (callable): Функция-обработчик запроса.
        verifier_func (callable): Функция для проверки подписи.
        signer_func (callable): Функция для подписи ответа.
        query_str (str): Строка запроса.
        key_group (str): Группа ключей.
        sig (str): Подпись запроса.
        **kwargs: Дополнительные аргументы для функции-обработчика.

    Returns:
        tuple: Кортеж с ответом и кодом состояния.
    """
    if sig != None and verifier_func(query_str, int(sig, 16), key_group):
        answer = handler_func(**kwargs)
        ret_code = 200
    else:
        print(f'failed to verify {query_str}', file=sys.stderr)
        answer = '$Signature verification fail'
        ret_code = 403
    answer = f'{answer}#{hex(signer_func(answer, "orvd"))[2:]}'
    return answer, ret_code


def authorized_request(handler_func, token: str, **kwargs):
    """
    Обрабатывает авторизованный запрос, проверяя токен и выполняя указанную функцию-обработчик.

    Args:
        handler_func (callable): Функция-обработчик запроса.
        token (str): Токен авторизации.
        **kwargs: Дополнительные аргументы для функции-обработчика.

    Returns:
        tuple: Кортеж с ответом и кодом состояния.
    """
    if check_user_token(token):
        answer = handler_func(**kwargs)
        ret_code = 200
    else:
        answer = '$Unauthorized'
        ret_code = 401
    return answer, ret_code


def check_user_token(token):
    """
    Проверяет валидность токена пользователя.

    Args:
        token (str): Токен для проверки.

    Returns:
        bool: True, если токен валиден, иначе False.
    """
    users = get_entities_by_field(User, User.access_token, token)
    if users and users.count() != 0:
        return True
    else:
        return False


def regular_request(handler_func, **kwargs):
    """
    Обрабатывает обычный запрос, выполняя указанную функцию-обработчик.

    Args:
        handler_func (callable): Функция-обработчик запроса.
        **kwargs: Дополнительные аргументы для функции-обработчика.

    Returns:
        tuple: Кортеж с ответом и кодом состояния.
    """
    try:
        answer = handler_func(**kwargs)
        ret_code = 200
    except:
        answer = 'Conflict.'
        ret_code = 409
    return answer, ret_code

    
def key_kos_exchange_handler(id: str, n: str, e: str):
    """
    Обрабатывает обмен ключами с KOS.

    Args:
        id (str): Идентификатор БПЛА.
        n (str): Модуль открытого ключа.
        e (str): Экспонента открытого ключа.

    Returns:
        str: Строка с открытым ключом ORVD.
    """
    n, e = str(int(n, 16)), str(int(e, 16))
    key_entity = get_entity_by_key(UavPublicKeys, id)
    if key_entity == None:
        save_public_key(n, e, f'kos{id}')
    orvd_n, orvd_e = get_key('orvd', private=False)
    str_to_send = f'$Key: {hex(orvd_n)[2:]} {hex(orvd_e)[2:]}'
    return str_to_send


def key_ms_exchange_handler(id: str):
    """
    Обрабатывает обмен ключами с Mission Sender.

    Args:
        id (str): Идентификатор отправителя миссии.

    Returns:
        str: Строка с открытым ключом ORVD.
    """
    key_group = f'ms{id}'
    if f'ms{id}' not in loaded_keys:
        generate_keys(ORVD_KEY_SIZE, key_group)
    key = loaded_keys[key_group].publickey()
    n, e = str(key.n), str(key.e)
    key_entity = get_entity_by_key(MissionSenderPublicKeys, id)
    if key_entity == None:
        save_public_key(n, e, f'ms{id}')
    else:
        key_entity.n = n
        key_entity.e = e
        commit_changes()
    orvd_key_pk = get_key('orvd', private=True).publickey()
    orvd_n, orvd_e = orvd_key_pk.n, orvd_key_pk.e
    str_to_send = f'$Key: {hex(orvd_n)[2:]} {hex(orvd_e)[2:]}'
    return str_to_send


def auth_handler(id: str):
    """
    Обрабатывает аутентификацию БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка подтверждения аутентификации.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        uav_entity = Uav(id=id, is_armed=False, state='В сети', kill_switch_state=False)
        add_and_commit(uav_entity)
    else:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
        uav_entity.kill_switch_state = False
        commit_changes()
    
    return f'$Auth id={id}'


def arm_handler(id: str):
    """
    Обрабатывает запрос на арм БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Статус арма БПЛА.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif uav_entity.is_armed:
        return f'$Arm {ARMED}$Delay {uav_entity.delay}' 
    else:
        mission = get_entity_by_key(Mission, id)
        if mission and mission.is_accepted == True:
            arm_queue.add(id)
            uav_entity.state = 'Ожидает'
            commit_changes()
            decision = _arm_wait_decision(id)
            if decision == ARMED:
                uav_entity.state = 'В полете'
            else:
                uav_entity.state = 'В сети'
            commit_changes()
            return f'$Arm {decision}$Delay {uav_entity.delay}'
        else:
            return f'$Arm {DISARMED}$Delay {uav_entity.delay}'


def _arm_wait_decision(id: str):
    """
    Ожидает решения об арме БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Решение об арме (ARMED или DISARMED).
    """
    while id in arm_queue:
        time.sleep(0.1)
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity.is_armed:
        return ARMED
    else:
        return DISARMED


def fly_accept_handler(id: str):
    """
    Обрабатывает запрос на принятие полета БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Статус арма БПЛА.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif uav_entity.is_armed:
        return f'$Arm {ARMED}$Delay {uav_entity.delay}'
    else:
        return f'$Arm {DISARMED}$Delay {uav_entity.delay}'


def kill_switch_handler(id: str):
    """
    Обрабатывает запрос на проверку состояния аварийного выключателя БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Состояние аварийного выключателя.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif uav_entity.kill_switch_state:
        return f'$KillSwitch {KILL_SWITCH_ON}'
    else:
        return f'$KillSwitch {KILL_SWITCH_OFF}'


def flight_info_handler(id: str) -> str:
    """
    Обрабатывает запрос на проверку информации полета БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Состояние полета БПЛА.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        forbidden_zones_hash = get_forbidden_zones_hash_handler(id)
        delay = f'$Delay {uav_entity.delay}'
        if uav_entity.kill_switch_state:
            status = '$Flight -1'
        elif uav_entity.is_armed:
            status = '$Flight 0'
        else:
            status = '$Flight 1'
        return ''.join([status, forbidden_zones_hash, delay])


def telemetry_handler(id: str, lat: float, lon: float, alt: float,
                      azimuth: float, dop: float, sats: float, speed: float):
    """
    Обрабатывает телеметрию БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.
        azimuth (float): Азимут.
        dop (float): Снижение точности.
        sats (float): Количество спутников.
        speed (float): Скорость.

    Returns:
        str: Статус арма БПЛА.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity and modes['display_only']:
        uav_entity = Uav(id=id, is_armed=False, state='В сети', kill_switch_state=False)
        add_and_commit(uav_entity)
        
    if not uav_entity:
        return NOT_FOUND
    else:
        lat = cast_wrapper(lat, float)
        if lat: lat /= 1e7
        lon = cast_wrapper(lon, float)
        if lon: lon /= 1e7
        alt = cast_wrapper(alt, float)
        if alt: alt /= 1e2
        azimuth = cast_wrapper(azimuth, float)
        if azimuth: azimuth /= 1e7
        dop = cast_wrapper(dop, float)
        sats = cast_wrapper(sats, int)
        speed = cast_wrapper(speed, float)
        record_time = datetime.datetime.utcnow()
        uav_telemetry_entity = get_entity_by_key(UavTelemetry, (uav_entity.id, record_time))
        if not uav_telemetry_entity:
            uav_telemetry_entity = UavTelemetry(uav_id=uav_entity.id, lat=lat, lon=lon, alt=alt,
                                                azimuth=azimuth, dop=dop, sats=sats, speed=speed, record_time=record_time)
            add_and_commit(uav_telemetry_entity)
        else:
            uav_telemetry_entity.lat = lat
            uav_telemetry_entity.lon = lon
            uav_telemetry_entity.alt = alt
            uav_telemetry_entity.azimuth = azimuth
            uav_telemetry_entity.dop = dop
            uav_telemetry_entity.sats = sats
            uav_telemetry_entity.speed = speed
            commit_changes()
        if not uav_entity.is_armed:
            return f'$Arm: {DISARMED}'
        else:
            return f'$Arm: {ARMED}'
    

def fmission_kos_handler(id: str):
    """
    Обрабатывает запрос на получение полетного задания для KOS.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с полетным заданием или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission and mission.is_accepted == True:
            mission_steps = get_entities_by_field_with_order(MissionStep, MissionStep.mission_id, id, order_by_field=MissionStep.step)
            if mission_steps and mission_steps.count() != 0:
                mission_steps = list(map(lambda e: e.operation, mission_steps))
                return f'$FlightMission {"&".join(mission_steps)}'
    return NOT_FOUND

            
def get_all_forbidden_zones_handler(id: str):
    """
    Обрабатывает запрос на получение всех запрещенных для полета зон.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с информацией о запрещенных зонах или NOT_FOUND.
    """
    try:
        with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
            forbidden_zones = json.load(f)
            result_str = generate_forbidden_zones_string(forbidden_zones)
            return result_str

    except Exception as e:
        print(e)
        return NOT_FOUND


def get_forbidden_zones_delta_handler(id: str):
    """
    Обрабатывает запрос на получение дельты изменений в запрещенных для полета зонах.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с дельтой изменений в запрещенных зонах или NOT_FOUND.
    """
    try:
        with open(FORBIDDEN_ZONES_DELTA_PATH, 'r', encoding='utf-8') as f:
            delta_zones = json.load(f)
        
        delta_str = f'$ForbiddenZonesDelta {len(delta_zones["features"])}'
        for zone in delta_zones['features']:
            name = zone['properties']['name']
            change_type = zone['properties']['change_type']
            coordinates = zone['geometry']['coordinates'][0]
            delta_str += f'&{name}&{change_type}&{len(coordinates)}&{"&".join(list(map(lambda e: f"{e[1]:.7f}_{e[0]:.7f}", coordinates)))}'
        
        return delta_str
    except Exception as e:
        print(e)
        return NOT_FOUND
    

def get_forbidden_zones_hash_handler(id: str):
    """
    Обрабатывает запрос на получение SHA-256 хэша строки запрещенных зон.

    Returns:
        str: SHA-256 хэш строки запрещенных зон или NOT_FOUND.
    """
    try:
        with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
            forbidden_zones = json.load(f)
            result_str = generate_forbidden_zones_string(forbidden_zones)
            hash_value = get_sha256_hex(result_str)
            return f'$ForbiddenZonesHash {hash_value}'

    except Exception as e:
        print(e)
        return NOT_FOUND


def fmission_ms_handler(id: str, mission_str: str):
    """
    Обрабатывает запрос на сохранение полетного задания от Mission Sender.

    Args:
        id (str): Идентификатор БПЛА.
        mission_str (str): Строка с полетным заданием.

    Returns:
        str: Статус верификации миссии.
    """
    mission_list, mission_verification_status = read_mission(mission_str)
    
    if mission_verification_status == MissionVerificationStatus.OK:
        uav_entity = get_entity_by_key(Uav, id)
        if not uav_entity and modes['display_only']:
            uav_entity = Uav(id=id, is_armed=False, state='В сети', kill_switch_state=False)
            add_and_commit(uav_entity)
            
        mission_entity = get_entity_by_key(Mission, id)
        if mission_entity:
            get_entities_by_field(MissionStep, MissionStep.mission_id, id).delete()
            delete_entity(mission_entity)
            commit_changes()
        
        mission_entity = Mission(uav_id=id, is_accepted=False)
        add_changes(mission_entity)
        encoded_mission = encode_mission(mission_list)
        for idx, cmd in enumerate(encoded_mission):
            mission_step_entity = MissionStep(mission_id=id, step=idx, operation=cmd)
            add_changes(mission_step_entity)
        commit_changes()
        
    return mission_verification_status


def revise_mission_handler(id: str, mission: str):
    mission_list = mission.split('*')
    
    mission_entity = get_entity_by_key(Mission, id)
    if mission_entity:
        get_entities_by_field(MissionStep, MissionStep.mission_id, id).delete()
        delete_entity(mission_entity)
        commit_changes()
    
    mission_entity = Mission(uav_id=id, is_accepted=False)
    add_changes(mission_entity)
    for idx, cmd in enumerate(mission_list):
        mission_step_entity = MissionStep(mission_id=id, step=idx, operation=cmd)
        add_changes(mission_step_entity)
    commit_changes()
    
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        uav_entity.is_armed = False
        uav_entity.state = 'Ожидает'
        commit_changes()
        
    revise_mission_queue.add(id)
    while id in revise_mission_queue:
        time.sleep(0.1)
        
    mission_entity = get_entity_by_key(Mission, id)
    if mission_entity:
        if mission_entity.is_accepted:
            return '$Approve 0'
        else:
            return '$Approve 1'


def revise_mission_decision_handler(id: str, decision: int):
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif id in revise_mission_queue:
        mission_entity = get_entity_by_key(Mission, id)
        if decision == 0:
            uav_entity.is_armed = True
            uav_entity.state = 'В полете'
            mission_entity.is_accepted = True
        else:
            uav_entity.is_armed = False
            uav_entity.state = 'В сети'
            mission_entity.is_accepted = False
        commit_changes()
        revise_mission_queue.remove(id)
        return f'$Arm: {decision}'
    else:
        return f'$Arm: -1'


def get_logs_handler(id: str):
    """
    Обрабатывает запрос на получение логов БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с логами или NOT_FOUND.
    """
    uav_log = None
    try:
        with open(f'{LOGS_PATH}/{id}.txt') as f:
            uav_log = f.read()
        if uav_log:
            return uav_log
        else:
            return NOT_FOUND
    except:
        return NOT_FOUND


def save_logs_handler(id: str, log: str):
    """
    Обрабатывает запрос на сохранение логов БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        log (str): Строка с логами для сохранения.

    Returns:
        str: OK в случае успешного сохранения.
    """
    try:
        if not os.path.exists(LOGS_PATH):
            os.makedirs(LOGS_PATH)
        with open(f'{LOGS_PATH}/{id}.txt', 'a') as f:
            f.write(f'\n{log}')
    except Exception as e:
        print(e)
    return OK


def admin_auth_handler(login: str, password: str):
    """
    Обрабатывает запрос на аутентификацию администратора.

    Args:
        login (str): Логин администратора.
        password (str): Пароль администратора.

    Returns:
        str: Токен доступа или пустая строка в случае неудачи.
    """
    user_entity = get_entity_by_key(User, login)
    if not user_entity:
        return NOT_FOUND
    else:
        password_hash = get_sha256_hex(password)
        if password_hash == user_entity.password_hash:
            return user_entity.access_token
        else:
            return ''


def arm_decision_handler(id: str, decision: int):
    """
    Обрабатывает решение об арме БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        decision (int): Решение об арме (ARMED или DISARMED).

    Returns:
        str: Статус арма или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif id in arm_queue:
        uav_entity.is_armed = True if decision == ARMED else False
        commit_changes()
        arm_queue.remove(id)
        return f'$Arm: {decision}'
    else:
        return f'$Arm: -1'


def force_disarm_handler(id: str):
    """
    Обрабатывает запрос на принудительный дизарм БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: OK в случае успешного дизарма или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
        commit_changes()
        return OK


def force_disarm_all_handler():
    """
    Обрабатывает запрос на принудительный дизарм всех БПЛА.

    Returns:
        str: OK в случае успешного дизарма всех БПЛА.
    """
    uav_entities = Uav.query.all()
    for uav_entity in uav_entities:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
    commit_changes()
    return OK


def get_state_handler(id: str):
    """
    Обрабатывает запрос на получение состояния БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Состояние БПЛА или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        return uav_entity.state


def get_mission_handler(id: str):
    """
    Обрабатывает запрос на получение полетного задания БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с полетным заданием или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission:
            mission_steps = get_entities_by_field_with_order(MissionStep, MissionStep.mission_id, id, order_by_field=MissionStep.step)
            if mission_steps and mission_steps.count() != 0:
                mission_steps = list(map(lambda e: e.operation, mission_steps))
                return "&".join(mission_steps)
    return NOT_FOUND


def get_telemetry_handler(id: str):
    """
    Обрабатывает запрос на получение телеметрии БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        json: JSON-объект с телеметрическими данными или NOT_FOUND.
    """
    uav_telemetry_entity = get_entities_by_field_with_order(UavTelemetry, UavTelemetry.uav_id, id, UavTelemetry.record_time.desc()).first()
    if not uav_telemetry_entity:
        return jsonify({'error': 'NOT_FOUND'})
    else:
        telemetry = {
            'lat': uav_telemetry_entity.lat,
            'lon': uav_telemetry_entity.lon,
            'alt': uav_telemetry_entity.alt,
            'azimuth': uav_telemetry_entity.azimuth,
            'dop': uav_telemetry_entity.dop,
            'sats': uav_telemetry_entity.sats,
            'speed': uav_telemetry_entity.speed
        }
        return jsonify(telemetry)


def get_telemetry_csv_handler(id: str):
    """
    Обрабатывает запрос на получение всей телеметрии БПЛА в формате CSV.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: CSV-строка с телеметрическими данными или NOT_FOUND.
    """
    uav_telemetry_entities = get_entities_by_field_with_order(UavTelemetry, UavTelemetry.uav_id, id, UavTelemetry.record_time.asc())
    if not uav_telemetry_entities:
        return NOT_FOUND

    csv_data = create_csv_from_telemetry(uav_telemetry_entities)
    return csv_data


def get_waiter_number_handler():
    """
    Обрабатывает запрос на получение количества БПЛА, ожидающих решения об арме.

    Returns:
        str: Количество ожидающих БПЛА.
    """
    return str(len(arm_queue))


def mission_decision_handler(id: str, decision: int):
    """
    Обрабатывает решение о принятии или отклонении миссии.

    Args:
        id (str): Идентификатор БПЛА.
        decision (int): Решение (0 - принять, 1 - отклонить).

    Returns:
        str: OK в случае успешной обработки или NOT_FOUND.
    """
    mission_entity = get_entity_by_key(Mission, id)
    if not mission_entity:
        return NOT_FOUND
    else:
        if decision == 0:
            mission_entity.is_accepted = True
        else:
            mission_entity.is_accepted = False
        commit_changes()
        return OK


def admin_kill_switch_handler(id: str):
    """
    Обрабатывает запрос на активацию аварийного выключателя БПЛА администратором.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: OK в случае успешной активации или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        uav_entity.is_armed = False
        uav_entity.kill_switch_state = True
        uav_entity.state = "Kill switch ON"
        commit_changes()
        return OK


def get_id_list_handler():
    """
    Обрабатывает запрос на получение списка идентификаторов всех БПЛА.

    Returns:
        str: Строка со списком идентификаторов БПЛА
    """
    uav_entities = Uav.query.order_by(Uav.created_date).all()
    uav_ids = list(map(lambda e: e.id, uav_entities))
    return str(uav_ids)


def get_mission_state_handler(id: str):
    """
    Обрабатывает запрос на получение состояния миссии БПЛА.
    
    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Состояние миссии (принята/не принята) или NOT_FOUND.
    """
    if id in revise_mission_queue:
        return '2'
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission:
            if mission.is_accepted:
                return str(MISSION_ACCEPTED)
            else:
                return str(MISSION_NOT_ACCEPTED)
    return NOT_FOUND


def change_fly_accept_handler(id: str, decision: int):
    """
    Обрабатывает запрос на изменение статуса принятия полета БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        decision (int): Решение (0 - принять, 1 - отклонить).

    Returns:
        str: OK в случае успешного изменения или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        if decision == 0:
            uav_entity.is_armed = True
            uav_entity.state = 'В полете'
        else:
            uav_entity.is_armed = False
            uav_entity.state = 'В сети'
        commit_changes()
        return OK
    return NOT_FOUND


def get_forbidden_zone_handler(name: str):
    """
    Обрабатывает запрос на получение координат запрещенной для полета зоны по ее имени.

    Args:
        name (str): Имя запрещенной зоны.

    Returns:
        json: JSON-массив с координатами зоны или NOT_FOUND.
    """
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        matching_zone = None
        for zone in forbidden_zones['features']:
            if zone['properties'].get('name') == name:
                matching_zone = zone
                break
        if matching_zone:
            return jsonify(matching_zone['geometry']['coordinates'][0])
        else:
            return NOT_FOUND
    return NOT_FOUND


def get_forbidden_zones_handler():
    """
    Обрабатывает запрос на получение всех запрещенных зон.

    Returns:
        dict: GeoJSON с запрещенными зонами
    """
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
    return forbidden_zones


def get_forbidden_zones_names_handler():
    """ 
    Обрабатывает запрос на получение имен всех запрещенных зон.

    Returns:
        json: JSON-массив с именами запрещенных зон или NOT_FOUND.
    """
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        zones_names = []
        for zone in forbidden_zones['features']:
            zones_names.append(zone['properties'].get('name'))
        return jsonify(zones_names)
        
    return NOT_FOUND


def set_forbidden_zone_handler(name: str, geometry: list):
    """
    Обрабатывает запрос на установку или обновление запрещенной для полета зоны.

    Args:
        name (str): Имя запрещенной зоны.
        geometry (list): Массив координат зоны.

    Returns:
        str: OK в случае успешной установки или сообщение об ошибке.
    """
    if not isinstance(geometry, list) or not all(isinstance(coord, list) and len(coord) == 2 for coord in geometry):
        return 'Bad geometry'
    
    for idx in range(len(geometry)):
        geometry[idx][0] = round(geometry[idx][0], 7)
        geometry[idx][1] = round(geometry[idx][1], 7)
        
    forbidden_zones = None
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        old_zones = json.load(f)
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        existing_zone = False
        for zone in forbidden_zones['features']:
            if zone['properties'].get('name') == name:
                zone['geometry']['coordinates'][0] = geometry
                existing_zone = True
        
        if not existing_zone:
            new_feature = get_new_polygon_feature(name, geometry)
            forbidden_zones['features'].append(new_feature)
    
    if forbidden_zones is not None:
        with open(FORBIDDEN_ZONES_PATH, 'w', encoding='utf-8') as f:
            json.dump(forbidden_zones, f, ensure_ascii=False, indent=4)
            
        compute_and_save_forbidden_zones_delta(old_zones, forbidden_zones)
    
    return OK


def delete_forbidden_zone_handler(name: str):
    """
    Обрабатывает запрос на удаление запрещенной для полета зоны.

    Args:
        name (str): Имя запрещенной зоны для удаления.

    Returns:
        str: OK в случае успешного удаления или NOT_FOUND.
    """
    forbidden_zones = None
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        old_zones = json.load(f)
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        for idx, zone in enumerate(forbidden_zones['features']):
            if zone['properties'].get('name') == name:
                forbidden_zones['features'].pop(idx)
                break
        
    if forbidden_zones != None:
        with open(FORBIDDEN_ZONES_PATH, 'w', encoding='utf-8') as f:
            json.dump(forbidden_zones, f, ensure_ascii=False, indent=4)
            
        compute_and_save_forbidden_zones_delta(old_zones, forbidden_zones)
        
        return OK
    
    return NOT_FOUND


def get_delay_handler(id: str):
    """
    Обрабатывает запрос на получение времени до следующего сеанса связи для указанного БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Время до следующего сеанса связи или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        return str(uav_entity.delay)


def set_delay_handler(id: str, delay: int):
    """
    Обрабатывает запрос на установку времени до следующего сеанса связи для указанного БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        delay (int): Время до следующего сеанса связи.

    Returns:
        str: OK в случае успешной установки или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        uav_entity.delay = delay
        commit_changes()
        return OK
    
    
def get_display_mode_handler():
    return '0' if modes['display_only'] else '1'

def toggle_display_mode_handler():
    modes['display_only'] = not modes['display_only']
    return OK

def get_flight_info_response_mode_handler():
    return '0' if modes['flight_info_response'] else '1'

def toggle_flight_info_response_mode_handler():
    modes['flight_info_response'] = not modes['flight_info_response']
    return OK

def get_all_data_handler():
    all_data = {}

    uav_entities = Uav.query.order_by(Uav.created_date).all()
    all_data['ids'] = [uav.id for uav in uav_entities]

    all_data['waiters'] = str(len(arm_queue))

    all_data['uav_data'] = {}
    for uav in uav_entities:
        uav_data = {}
        
        uav_data['state'] = uav.state

        uav_telemetry_entity = get_entities_by_field_with_order(UavTelemetry, UavTelemetry.uav_id, uav.id, UavTelemetry.record_time.desc()).first()
        if uav_telemetry_entity:
            uav_data['telemetry'] = {
                'lat': uav_telemetry_entity.lat,
                'lon': uav_telemetry_entity.lon,
                'alt': uav_telemetry_entity.alt,
                'azimuth': uav_telemetry_entity.azimuth,
                'dop': uav_telemetry_entity.dop,
                'sats': uav_telemetry_entity.sats,
                'speed': uav_telemetry_entity.speed
            }
        else:
            uav_data['telemetry'] = None

        uav_data['mission_state'] = get_mission_state_handler(uav.id)

        uav_data['delay'] = str(uav.delay)

        all_data['uav_data'][uav.id] = uav_data
    return jsonify(all_data)