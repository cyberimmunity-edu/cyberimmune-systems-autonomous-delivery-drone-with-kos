import os
from flask import Flask, request, render_template, redirect
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate

db = SQLAlchemy()
app = Flask(__name__)
app.config["SQLALCHEMY_DATABASE_URI"] = "sqlite:///orvd.db"
app.config['SQLALCHEMY_ECHO'] = False
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db.init_app(app)
migrate = Migrate(app, db)

from utils.api_handlers import *

with app.app_context():
    db.create_all()
    clean_db([UavTelemetry, MissionStep, Mission, MissionSenderPublicKeys, UavPublicKeys, Uav, User])
    generate_user(User)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/admin')
def admin():
    token = request.args.get('token')
    if token == None or not check_user_token(token):
        return redirect("/admin/auth_page")
    else:
        return render_template('admin.html')

@app.route('/admin/auth')
def admin_auth():
    login = str(request.args.get('login'))
    password = str(request.args.get('password'))
    return regular_request(handler_func=admin_auth_handler, login=login, password=password)

@app.route('/admin/auth_page')
def auth_page():
    return render_template('admin_auth.html')

@app.route('/admin/arm_decision')
def arm_decision():
    id = int(request.args.get('id'))
    decision = int(request.args.get('decision'))
    token = request.args.get('token')
    return authorized_request(handler_func=arm_decision_handler, token=token,
                       id=id, decision=decision)
    
@app.route('/admin/mission_decision')
def mission_decision():
    id = int(request.args.get('id'))
    decision = int(request.args.get('decision'))
    token = request.args.get('token')
    return authorized_request(handler_func=mission_decision_handler, token=token,
                       id=id, decision=decision)

@app.route('/admin/force_disarm')
def force_disarm():
    id = int(request.args.get('id'))
    token = request.args.get('token')
    return authorized_request(handler_func=force_disarm_handler, token=token, id=id)
    
@app.route('/admin/force_disarm_all')
def force_disarm_all():
    token = request.args.get('token')
    return authorized_request(handler_func=force_disarm_all_handler, token=token)

@app.route('/admin/kill_switch')
def admin_kill_switch():
    id = int(request.args.get('id'))
    token = request.args.get('token')
    return authorized_request(handler_func=admin_kill_switch_handler, token=token, id=id)
    
@app.route('/admin/get_state')
def get_state():
    id = int(request.args.get('id'))
    token = request.args.get('token')
    return authorized_request(handler_func=get_state_handler, token=token, id=id)
    
@app.route('/admin/get_mission')
def get_mission():
    id = int(request.args.get('id'))
    token = request.args.get('token')
    return authorized_request(handler_func=get_mission_handler, token=token, id=id)
    
@app.route('/admin/get_telemetry')
def get_telemetry():
    id = int(request.args.get('id'))
    token = request.args.get('token')
    return authorized_request(handler_func=get_telemetry_handler, token=token, id=id)

@app.route('/admin/get_waiter_number')
def get_waiter_number():
    token = request.args.get('token')
    return authorized_request(handler_func=get_waiter_number_handler, token=token)

@app.route('/admin/get_id_list')
def get_id_list():
    token = request.args.get('token')
    return authorized_request(handler_func=get_id_list_handler, token=token)


@app.route('/mission_sender')
def mission_sender():
    return render_template('mission_sender.html')

@app.route('/mission_sender/fmission_ms', methods=['POST'])
def fmission():
    id = int(request.args.get('id'))
    mission_str = request.get_data().decode()
    sig = request.args.get('sig')
    return signed_request(handler_func=fmission_ms_handler, verifier_func=mock_verifier, signer_func=sign,
                          query_str=mission_str, key_group=f'ms{id}', sig=sig, id=id, mission_str=mission_str)
    
@app.route('/mission_sender/key')
def key_ms_exchange():
    id = int(request.args.get('id'))
    return regular_request(handler_func=key_ms_exchange_handler, id=id)


@app.route('/api/key')
def key_kos_exchange():
    id = int(request.args.get('id'))
    n = request.args.get('n')
    e = request.args.get('e')
    return regular_request(handler_func=key_kos_exchange_handler, id=id, n=n, e=e)


@app.route('/api/arm')
def arm_request():
    id = int(request.args.get('id'))
    sig = request.args.get('sig')
    return signed_request(handler_func=arm_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/arm?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    
@app.route('/api/auth')
def auth():
    id = int(request.args.get('id'))
    sig = request.args.get('sig')
    return signed_request(handler_func=auth_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/auth?id={id}', key_group=f'kos{id}', sig=sig, id=id)

@app.route('/api/fly_accept')
def fly_accept():
    id = int(request.args.get('id'))
    sig = request.args.get('sig')
    return signed_request(handler_func=fly_accept_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/fly_accept?id={id}', key_group=f'kos{id}', sig=sig, id=id)

@app.route('/api/telemetry')
def telemetry():
    id = int(request.args.get('id'))
    sig = request.args.get('sig')
    lat = request.args.get('lat')
    lon = request.args.get('lon')
    alt = request.args.get('alt')
    azimuth = request.args.get('azimuth')
    return signed_request(handler_func=telemetry_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/telemetry?id={id}&lat={lat}&lon={lon}&alt={alt}&azimuth={azimuth}',
                          key_group=f'kos{id}', sig=sig, id=id, lat=lat, lon=lon, alt=alt, azimuth=azimuth)
    
@app.route('/api/kill_switch')
def kill_switch():
    id = int(request.args.get('id'))
    sig = request.args.get('sig')
    return signed_request(handler_func=kill_switch_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/kill_switch?id={id}', key_group=f'kos{id}', sig=sig, id=id)


@app.route('/api/fmission_kos')
def fmission_kos():
    id = int(request.args.get('id'))
    sig = request.args.get('sig')
    return signed_request(handler_func=fmission_kos_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/fmission_kos?id={id}', key_group=f'kos{id}', sig=sig, id=id)


if __name__ == '__main__':
    app.run()