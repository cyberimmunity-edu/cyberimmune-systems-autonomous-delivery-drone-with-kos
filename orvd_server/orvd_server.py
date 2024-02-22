from flask import Flask, request
import socket
from time import sleep

app = Flask(__name__)
HOST = '192.168.1.132'

def socket_init_and_send(data):
    sock = socket.socket()
    sock.connect((HOST, 9090))
    sock.send(data)
    return sock

def socket_wait_for_answer(sock):
    while True:
        answer = sock.recv(1024)
        if answer:
            break
    return answer


@app.route('/')
def default_answ():
    return 'default_page'

@app.route('/key')
def key_kos_exchange():
    id = request.args.get('id')
    n = request.args.get('n')
    e = request.args.get('e')
    send_str = f'keykos_{id}_{n}_{e}'
    sock = socket_init_and_send(send_str.encode())
    answer = socket_wait_for_answer(sock).decode()
    sock.close()
    if answer:
        return answer, 200
    else:
        return 'Error', 500

@app.route('/key_exchange')
def key_exchange():
    id = request.args.get('id')
    group = request.args.get('group')
    n = request.args.get('n')
    e = request.args.get('e')
    send_str = f'key_{id}_{group}_{n}_{e}'
    sock = socket_init_and_send(send_str.encode())
    answer = socket_wait_for_answer(sock).decode()
    sock.close()
    if answer:
        return answer, 200
    else:
        return 'Error', 500

@app.route('/arm')
def arm_request():
    id = request.args.get('id')
    sig = request.args.get('sig')
    send_str = 'arm_' + str(id) + '#' + sig
    sock = socket_init_and_send(send_str.encode())
    answer = socket_wait_for_answer(sock).decode()
    sock.close()
    if answer:
        return answer, 200
    else:
        return "Error", 500


@app.route('/auth')
def auth():
    id = request.args.get('id')
    sig = request.args.get('sig')
    send_str = 'auth_' + str(id) + '#' + sig
    sock = socket_init_and_send(send_str.encode())
    answer = socket_wait_for_answer(sock).decode()
    sock.close()

    return answer, 200

@app.route('/fly_accept')
def fly_accept():
    id = request.args.get('id')
    sig = request.args.get('sig')
    send_str = 'flyaccept_' + str(id) + '#' + sig
    sock = socket_init_and_send(send_str.encode())
    answer = socket_wait_for_answer(sock).decode()
    sock.close()
    if answer:
        return answer, 200
    else:
        return "Error", 500

@app.route('/fmission_mp', methods=['POST'])
def fmission():
    data = request.get_data()
    send_str = 'fmissionmp_' + data.decode()
    sock = socket_init_and_send(send_str.encode())
    answer = socket_wait_for_answer(sock).decode()
    sock.close()
    return answer, 200

@app.route('/fmission_kos')
def fmission_kos():
    id = request.args.get('id')
    sig = request.args.get('sig')
    send_str = 'fmissionkos_' + str(id) + '#' + sig
    sock = socket_init_and_send(send_str.encode())
    answer = socket_wait_for_answer(sock).decode()
    sock.close()
    return answer, 200



if __name__ == '__main__':
    app.run()
