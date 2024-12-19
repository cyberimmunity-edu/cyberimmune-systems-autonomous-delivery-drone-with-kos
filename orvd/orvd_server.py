import os
import paho.mqtt.client as mqtt
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate
from flasgger import Swagger
from urllib.parse import parse_qs

db = SQLAlchemy()

def create_app():
    app = Flask(__name__)
    app.config["SQLALCHEMY_DATABASE_URI"] = "sqlite:///orvd.db"
    app.config['SQLALCHEMY_ECHO'] = False
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
    app.config['SWAGGER'] = {
        'title': 'ORVD API',
        'uiversion': 3
    }
    db.init_app(app)
    Migrate(app, db)
    Swagger(app)
    
    from routes import bp as main_bp
    app.register_blueprint(main_bp)
    
    MQTT_BROKER = 'localhost'
    MQTT_PORT = 1883
    MQTT_TOPIC = 'api/telemetry'
    mqtt_client = mqtt.Client()
    def on_connect(client, userdata, flags, rc):
        client.subscribe(MQTT_TOPIC)

    def on_message(client, userdata, msg):
        query_string = msg.payload.decode()
        query_params = parse_qs(query_string)
        single_value_params = {k: v[0] for k, v in query_params.items()}
        with app.app_context():
            regular_request(handler_func=telemetry_handler, **single_value_params)


    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    with app.app_context():
        mqtt_client.loop_start()
    
    return app

from utils.api_handlers import *

def clean_app_db(app):
    with app.app_context():
        db.create_all()
        clean_db([UavTelemetry, MissionStep, Mission, MissionSenderPublicKeys, UavPublicKeys, Uav, User])
        generate_user(User)


if __name__ == "__main__":
    app = create_app()
    app.run()