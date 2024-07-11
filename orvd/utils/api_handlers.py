from utils.db_utils import *
from utils.utils import *

arm_queue = set()

def bad_request(message: str):
    return message, 400
        
def signed_request(handler_func, verifier_func, signer_func, query_str, key_group, sig, **kwargs):
    if sig != None and verifier_func(query_str, int(sig, 16), key_group):
        answer = handler_func(**kwargs)
        ret_code = 200
    else:
        print(f'failed to verify {query_str}', file=sys.stderr)
        answer = '$Signature verification fail'
        ret_code = 403
    answer = f'{answer}#{hex(signer_func(answer, "orvd"))[2:]}'
    return answer, ret_code

def authorized_request(handler_func, token, **kwargs):
    if check_user_token(token):
        answer = handler_func(**kwargs)
        ret_code = 200
    else:
        answer = '$Unauthorized'
        ret_code = 401
    return answer, ret_code

def check_user_token(token):
    users = get_entities_by_field(User, User.access_token, token)
    if users and users.count() != 0:
        return True
    else:
        return False
    
def regular_request(handler_func, **kwargs):
    try:
        answer = handler_func(**kwargs)
        ret_code = 200
    except:
        answer = 'Conflict.'
        ret_code = 409
    return answer, ret_code

    
def key_kos_exchange_handler(id: int, n: str, e: str):
    n, e = str(int(n, 16)), str(int(e, 16))
    key_entity = get_entity_by_key(UavPublicKeys, id)
    if key_entity == None:
        save_public_key(n, e, f'kos{id}')
    orvd_n, orvd_e = get_key('orvd', private=False)
    str_to_send = f'$Key: {hex(orvd_n)[2:]} {hex(orvd_e)[2:]}'
    return str_to_send

def key_ms_exchange_handler(id: int):
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

def auth_handler(id: int):
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

def arm_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif uav_entity.is_armed:
        return f'$Arm: {ARMED}' 
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
            return f'$Arm: {decision}'
        else:
            return f'$Arm: {DISARMED}'

def _arm_wait_decision(id: int):
    while id in arm_queue:
        time.sleep(0.1)
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity.is_armed:
        return ARMED
    else:
        return DISARMED

def fly_accept_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif uav_entity.is_armed:
        return f'$Arm: {ARMED}'
    else:
        return f'$Arm: {DISARMED}'

def kill_switch_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif uav_entity.kill_switch_state:
        return f'$KillSwitch: {KILL_SWITCH_ON}'
    else:
        return f'$KillSwitch: {KILL_SWITCH_OFF}'
    
def telemetry_handler(id: int, lat: float, lon: float, alt: float,
                      azimuth: float, dop: float, sats: float):
    uav_entity = get_entity_by_key(Uav, id)
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
        uav_telemetry_entity = get_entity_by_key(UavTelemetry, uav_entity.id)
        if not uav_telemetry_entity:
            uav_telemetry_entity = UavTelemetry(uav_id=uav_entity.id, lat=lat, lon=lon, alt=alt,
                                                azimuth=azimuth, dop=dop, sats=sats)
            add_and_commit(uav_telemetry_entity)
        else:
            uav_telemetry_entity.lat = lat
            uav_telemetry_entity.lon = lon
            uav_telemetry_entity.alt = alt
            uav_telemetry_entity.azimuth = azimuth
            uav_telemetry_entity.dop = dop
            uav_telemetry_entity.sats = sats
            commit_changes()
        if not uav_entity.is_armed:
            return f'$Arm: {DISARMED}'
        else:
            return f'$Arm: {ARMED}'
    

def fmission_kos_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission and mission.is_accepted == True:
            mission_steps = get_entities_by_field(MissionStep, MissionStep.mission_id, id, order_by_field=MissionStep.step)
            if mission_steps and mission_steps.count() != 0:
                mission_steps = list(map(lambda e: e.operation, mission_steps))
                return f'$FlightMission {"&".join(mission_steps)}'
    return NOT_FOUND
            

def fmission_ms_handler(id: int, mission_str: str):
    mission_entity = get_entity_by_key(Mission, id)
    if mission_entity:
        get_entities_by_field(MissionStep, MissionStep.mission_id, id).delete()
        delete_entity(mission_entity)
        commit_changes()
    mission_entity = Mission(uav_id=id, is_accepted=False)
    add_changes(mission_entity)
    mission_list = read_mission(mission_str)
    encoded_mission = encode_mission(mission_list)
    for idx, cmd in enumerate(encoded_mission):
        mission_step_entity = MissionStep(mission_id=id, step=idx, operation=cmd)
        add_changes(mission_step_entity)
    commit_changes()
        
    return OK


def get_logs_handler(id: int):
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

def save_logs_handler(id: int, log: str):
    try:
        with open(f'{LOGS_PATH}/{id}.txt', 'a') as f:
            f.write(f'\n{log}')
    except Exception as e:
        print(e)
    return OK


def admin_auth_handler(login: str, password: str):
    user_entity = get_entity_by_key(User, login)
    if not user_entity:
        return NOT_FOUND
    else:
        password_hash = get_sha256_hex(password)
        if password_hash == user_entity.password_hash:
            return user_entity.access_token
        else:
            return ''


def arm_decision_handler(id: int, decision: int):
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

def force_disarm_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
        commit_changes()
        return OK

def force_disarm_all_handler():
    uav_entities = Uav.query.all()
    for uav_entity in uav_entities:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
    commit_changes()
    return OK

def get_state_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        return uav_entity.state

def get_mission_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission:
            mission_steps = get_entities_by_field(MissionStep, MissionStep.mission_id, id, order_by_field=MissionStep.step)
            if mission_steps and mission_steps.count() != 0:
                mission_steps = list(map(lambda e: e.operation, mission_steps))
                return "&".join(mission_steps)
    return NOT_FOUND

def get_telemetry_handler(id: int):
    uav_telemetry_entity = get_entity_by_key(UavTelemetry, id)
    if not uav_telemetry_entity:
        return NOT_FOUND
    else:
        telemetry = (str(uav_telemetry_entity.lat), str(uav_telemetry_entity.lon),
                     str(uav_telemetry_entity.alt), str(uav_telemetry_entity.azimuth),
                     str(uav_telemetry_entity.dop), str(uav_telemetry_entity.sats))
        return "&".join(telemetry)
        

def get_waiter_number_handler():
    return str(len(arm_queue))

def mission_decision_handler(id: int, decision: int):
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

def admin_kill_switch_handler(id: int):
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
    uav_entities = Uav.query.order_by(Uav.created_date).all()
    uav_ids = list(map(lambda e: e.id, uav_entities))
    return str(uav_ids)

def get_mission_state_handler(id: int):
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission:
            if mission.is_accepted:
                return str(MISSION_ACCEPTED)
            else:
                return str(MISSION_NOT_ACCEPTED)
    return NOT_FOUND

def change_fly_accept_handler(id: int, decision: int):
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