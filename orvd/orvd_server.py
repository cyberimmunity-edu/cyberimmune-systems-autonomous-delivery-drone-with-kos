import os
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate
from flasgger import Swagger

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