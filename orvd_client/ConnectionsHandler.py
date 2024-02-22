import socket
import time
from PySide6.QtCore import Slot, QObject, Signal
from threading import Thread, Event
from collections.abc import Callable
from functools import wraps
from utils import parse_mission, sign, verify, save_public_key, get_key

DEBUG = True
IGNORE_SIGNATURE = True

LOCALHOST = '127.0.0.1'

NOT_FOUND_RESPONSE = '-1'

ARMED = 0
DISARMED = 1


class ConnectionsHandler(QObject):
    armRequest = Signal(str, str)
    showId = Signal(str)
    showState = Signal(str, str)
    showMissionAcceptance = Signal(str, int)
    showMission = Signal(list)
    noMission = Signal()
    missionGained = Signal(str)
    
    def __init__(self):
        super().__init__()
        self.eventOnClose = Event()
        self._events_on_send: dict[str, Event] = {}
        self._server_socket: socket.socket = None
        self._server_port: int = None
        self._connections: dict[str, socket.socket] = {}
        self._id_arm: dict[str, int] = {}
        self._id_state: dict[str, str] = {}
        self._id_address: dict[str, str] = {}
        self._id_fmission: dict[str, str] = {}
        self._id_fmission_parsed: dict[str, list] = {}
        self._id_fmission_accepted: dict[str, bool] = {}
        self._request_switch: dict[str, Callable[[str, str], None]] = {
            'arm': self._arm_request_handler,
            'auth': self._auth_request_handler,
            'flyaccept': self._flyaccept_request_handler,
            'fmissionkos': self._fmission_kos_request_handler,
            'fmissionmp': self._fmission_mp_request_handler,
            'keykos': self._key_kos_exchange_handler,
            'key': self._key_exchange_handler
        }
        self._unsigned_requests: list[str] = ['keykos', 'key']
    
    def _server_socket_loop(self, port: int=9090):
        self._server_port = port
        self._server_socket = socket.socket()
        self._server_socket.bind(('', port))
        if DEBUG:
            print('Waiting for connections...')

        while not self.eventOnClose.is_set():
            self._server_socket.listen(1)
            server_сonn, addr = self._server_socket.accept()
            if addr[0] != LOCALHOST:
                if DEBUG:
                    print('connected:', addr)
                address_str = f'{addr[0]}:{str(addr[1])}'
                self._connections.update({address_str: server_сonn})
                data = self._connections[address_str].recv(1024)
                request_handler_thread = Thread(name='request_handler_thread',
                                                target=self._server_request_handler,
                                                args=(data, address_str), daemon=True)
                request_handler_thread.start()
            else:
                server_сonn.close()
    
    def start_server_socket_loop(self, port: int=9090):
        server_socket_thread = Thread(name='server_socket_loop', 
                            target=self._server_socket_loop, args=(port,))
        server_socket_thread.start()
    
    def _server_request_handler(self, request: bytes, address: str):
        self._events_on_send.update({address: Event()})
        if request == None:
            self._events_on_send[address].set()
        else:
            try:
                request_str = request.decode()
                tokens = request_str.split('_', maxsplit=1)
                if tokens[0] in self._request_switch:
                    print(f'Performing {tokens[0]} request')
                    if tokens[0] in self._unsigned_requests:
                        handler = self._request_switch[tokens[0]]
                        handler(tokens[1], address)
                    else:
                        self._check_signature_and_handle(method_name=tokens[0],
                                          body=tokens[1], address=address)
                else:
                    print('Unexpected command ' + tokens[0])
                    print(f'content={request_str}')
                    self._error_request_handler(address)
            
            except UnicodeDecodeError:
                print('There is non-text file received')
                self._error_request_handler(address)
        
        while not self._events_on_send[address].is_set():
            time.sleep(1)
        self._events_on_send.pop(address)
        self._close_connection(address)
    
    
    def _arm_request_handler(self, id: str, address: str):
        self._id_address.update({id: address})
        if id not in self._id_arm or id not in self._id_fmission_accepted or self._id_fmission_accepted[id] == False:
            self._sign_and_send_string('$Arm: 1', address)
        elif self._id_arm[id] == 0:
            self._sign_and_send_string('$Arm: 0', address)
        else:
            self._id_state.update({id: 'Ожидает'})
            self.showState.emit(self._id_state[id], id)
            self.armRequest.emit(id, address)
    
    def _auth_request_handler(self, id: str, address: str):
        if id not in self._id_arm:
            self._id_state.update({id: 'В сети'})
            self._id_arm.update({id: 1})
        self.showId.emit(id)
        self._sign_and_send_string(f'$Auth id={id}', address)
    
    def _flyaccept_request_handler(self, id: str, address: str):
        if id in self._id_arm:
            args = f'$Arm: {self._id_arm[id]}', address
        else:
            args = str('$' + NOT_FOUND_RESPONSE), address
        self._sign_and_send_string(*args)
    
    def _fmission_kos_request_handler(self, id: str, address: str):
        if id in self._id_fmission and self._id_fmission_accepted[id] == True:
            args = f'$FlightMission {self._id_fmission[id]}', address
        else:
            args = str('$' + NOT_FOUND_RESPONSE), address
        self._sign_and_send_string(*args)
    
    def _fmission_mp_request_handler(self, body: str, address: str):
        id, mission = body.split('_', maxsplit=1)
        self._id_fmission.update({id: mission})
        self._id_fmission_parsed.update({id: parse_mission(mission)})
        self._id_fmission_accepted.update({id: False})
        self.missionGained.emit(id)
        str_to_send = 'mission received'
        self._sign_and_send_string(str_to_send, address)
            
    def _key_kos_exchange_handler(self, body: str, address: str):
        id, n, e = body.split('_')
        self._key_exchange(id, 'kos', n, e, address)
        
    def _key_exchange_handler(self, body: str, address: str):
        id, group, n, e = body.split('_')
        self._key_exchange(id, group, n, e, address)
        
    def _error_request_handler(self, address: str, message: str = None):
        if message:
            self._sign_and_send_string(message, address)
        else:
            self._sign_and_send_string("$Unexpected error", address)
    
    def _key_exchange(self, id: str, group: str, n: str, e: str, address: str):
        save_public_key(n, e, key_group=f'{group}{id}')
        mcc_n, mcc_e = list(map(lambda k: hex(k)[2:], get_key('mcc', private=False)))
        str_to_send = f'$Key: {mcc_n} {mcc_e}'
        self._send_data(str_to_send.encode(), address)
    
    
    @Slot(str, int)
    def arm_decision(self, id: str, decision: int):
        address = self._id_address[id]
        if decision == ARMED:
            self._id_state.update({id: 'В полете'})
            self.showState.emit(self._id_state[id], id)
        elif decision == DISARMED:
            self._id_state.update({id: 'В сети'})
            self.showState.emit(self._id_state[id], id)
        if decision == ARMED or decision == DISARMED:
            self._id_arm[id] = decision
            self._sign_and_send_string(f'$Arm: {decision}', address)
        else:
            print('Wrong arm decision.')
            self._error_request_handler(address)
    
    @Slot(str)
    def force_disarm(self, id: str):
        if id in self._id_arm:
            self._id_arm[id] = DISARMED
            self._id_state.update({id: 'В сети'})
            self.showState.emit(self._id_state[id], id)
    
    @Slot()
    def force_disarm_all(self):
        for id in self._id_arm:
            self._id_arm[id] = DISARMED
            self._id_state.update({id: 'В сети'})
            self.showState.emit(self._id_state[id], id)
            
    @Slot(str)
    def get_state(self, id):
        self.showState.emit(self._id_state[id], id)
        if id in self._id_fmission_accepted:
            if self._id_fmission_accepted[id] == True:
                self.showMissionAcceptance.emit(id, 0)
            else:
                self.showMissionAcceptance.emit(id, 1)
        else:
            self.showMissionAcceptance.emit(id, 1)
        
    @Slot(str)
    def get_mission(self, id):
        if id in self._id_fmission_parsed:
            self.showMission.emit(self._id_fmission_parsed[id])
        else:
            self.noMission.emit()
    
    @Slot(str, int)
    def change_mission_acceptance(self, id, decision):
        if id in self._id_fmission_accepted:
            if decision == 0:
                self._id_fmission_accepted[id] = True
            elif decision == 1:
                self._id_fmission_accepted[id] = False
            else:
                print('wrong decision')            

    
    def _check_signature_and_handle(self, method_name: str, body: str, address: str):
        body, signature = body.split('#', maxsplit=1)
        if len(body.split('_')) == 1:
            id = body
        else:
            id = body.split('_')[0]
            
        if method_name == 'fmissionkos':
            sig_method_name = 'fmission_kos'
        elif method_name == 'flyaccept':
            sig_method_name = 'fly_accept'
        else:
            sig_method_name = method_name
        
        if method_name == 'fmissionmp':
            str_to_hash = body
            key_group = f'mp{id}'
        else:
            str_to_hash = f'{sig_method_name}?id={id}'
            key_group = f'kos{id}'
        is_answer_safe = verify(str_to_hash, int(signature, 16), key_group=key_group)
        if IGNORE_SIGNATURE:
            print(f'Верификация={is_answer_safe}')
        if is_answer_safe or IGNORE_SIGNATURE:
            handler = self._request_switch[method_name]
            return handler(body, address)
        else:
            print(f'Bad signature in {method_name} method')
            print(f'body={body}')
            print(f'signature={signature}')
            return self._error_request_handler(address=address, message="$Bad signature")
        
    def _sign_and_send_string(self, str_to_send: str, address: str):
        signature = sign(str_to_send, key_group='mcc')
        signature = hex(signature)[2:]
        str_to_send = f'{str_to_send}#{signature}'
        self._send_data(str_to_send.encode(), address)
        
    def _send_data(self, data: bytes, address: str):
        self._connections[address].send(data)
        self._events_on_send[address].set()
    
    def _close_connection(self, address: str):
        self._connections[address].close()
        self._connections.pop(address)
        
    def close(self):
        self.eventOnClose.set()
        if len(self._connections) != 0:
            for address in self._connections:
                self._error_request_handler(address)
                self._connections[address].close()
        if self._server_socket:
            socket.socket(socket.AF_INET, 
                  socket.SOCK_STREAM).connect((LOCALHOST, self._server_port))
            self._server_socket.close()