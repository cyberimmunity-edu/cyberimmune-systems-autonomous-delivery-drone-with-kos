import datetime
from orvd_server import db

class User(db.Model):
    __tablename__ = 'user'
    username = db.Column(db.String(64), index=True, primary_key=True)
    password_hash = db.Column(db.String(128))
    access_token = db.Column(db.String(128))

    def __repr__(self):
        return '<User {}>'.format(self.username)


class Uav(db.Model):
    __tablename__ = 'uav'
    id = db.Column(db.Integer, primary_key=True, autoincrement=False)
    is_armed = db.Column(db.Boolean, default=False)
    state = db.Column(db.String(64))
    kill_switch_state = db.Column(db.Boolean, default=False)
    created_date = db.Column(db.DateTime, default=datetime.datetime.utcnow)
    
    def __repr__(self):
        return '<UAV {}>'.format(self.id)


class UavPublicKeys(db.Model):
    __tablename__ = 'uav_public_keys'
    uav_id = db.Column(db.Integer, primary_key=True)
    n = db.Column(db.String(1024))
    e = db.Column(db.String(1024))
    
    def __repr__(self):
        return f'UAV id={self.uav_id}, KOS key={public_key}'
    

class MissionSenderPublicKeys(db.Model):
    __tablename__ = 'mission_sender_public_keys'
    uav_id = db.Column(db.Integer, primary_key=True)
    n = db.Column(db.String(1024))
    e = db.Column(db.String(1024))
    
    def __repr__(self):
        return f'UAV id={self.uav_id}, MS key={public_key}'
    

class Mission(db.Model):
    __tablename__ = 'mission'
    uav_id = db.Column(db.Integer, db.ForeignKey('uav.id'), primary_key=True)
    is_accepted = db.Column(db.Boolean, default=False)
    
    def __repr__(self):
        return '<Mission {}>'.format(self.id)


class MissionStep(db.Model):
    __tablename__ = 'mission_step'
    mission_id = db.Column(db.Integer, db.ForeignKey('mission.uav_id'))
    step = db.Column(db.Integer)
    operation = db.Column(db.String(64))
    
    __table_args__ = (
        db.PrimaryKeyConstraint(
            mission_id, step,
        ),
    )
    
    def __repr__(self):
        return f'Mission id={self.mission_id}, step={step}, operation={operation}'
    

class UavTelemetry(db.Model):
    __tablename__ = 'uav_telemetry'
    uav_id = db.Column(db.Integer, db.ForeignKey('uav.id'), primary_key=True)
    lat = db.Column(db.Float(precision=8))
    lon = db.Column(db.Float(precision=8))
    alt = db.Column(db.Float(precision=8))
    azimuth = db.Column(db.Float(precision=8))
    dop = db.Column(db.Float(precision=8))
    sats = db.Column(db.Integer)
    
    def __repr__(self):
        return f'UAV id={self.uav_id}, lat={lat}, lon={lon}, alt={alt}, azimuth={azimuth}'