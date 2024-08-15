import datetime
from orvd_server import db

class User(db.Model):
    """
    Модель пользователя.
    
    Attributes:
        username: имя пользователя (первичный ключ)
        password_hash: хэш пароля
        access_token: токен доступа
    """
    __tablename__ = 'user'
    username = db.Column(db.String(64), index=True, primary_key=True)
    password_hash = db.Column(db.String(128))
    access_token = db.Column(db.String(128))

    def __repr__(self):
        return '<User {}>'.format(self.username)


class Uav(db.Model):
    """
    Модель БПЛА.
    
    Attributes:
        id: идентификатор БПЛА (первичный ключ)
        is_armed: состояние арма
        state: текущее состояние БПЛА
        kill_switch_state: состояние аварийного выключателя
        created_date: дата создания записи
    """
    __tablename__ = 'uav'
    id = db.Column(db.Integer, primary_key=True, autoincrement=False)
    is_armed = db.Column(db.Boolean, default=False)
    state = db.Column(db.String(64))
    kill_switch_state = db.Column(db.Boolean, default=False)
    created_date = db.Column(db.DateTime, default=datetime.datetime.utcnow)
    
    def __repr__(self):
        return '<UAV {}>'.format(self.id)


class UavPublicKeys(db.Model):
    """
    Модель публичных ключей БПЛА.
    
    Attributes:
        uav_id: идентификатор БПЛА (первичный ключ)
        n: модуль открытого ключа
        e: экспонента открытого ключа
    """
    __tablename__ = 'uav_public_keys'
    uav_id = db.Column(db.Integer, primary_key=True)
    n = db.Column(db.String(1024))
    e = db.Column(db.String(1024))
    
    def __repr__(self):
        return f'UAV id={self.uav_id}, KOS key={public_key}'
    

class MissionSenderPublicKeys(db.Model):
    """
    Модель публичных ключей отправителя миссии.
    
    Attributes:
        uav_id: идентификатор БПЛА (первичный ключ)
        n: модуль открытого ключа
        e: экспонента открытого ключа
    """
    __tablename__ = 'mission_sender_public_keys'
    uav_id = db.Column(db.Integer, primary_key=True)
    n = db.Column(db.String(1024))
    e = db.Column(db.String(1024))
    
    def __repr__(self):
        return f'UAV id={self.uav_id}, MS key={public_key}'
    

class Mission(db.Model):
    """
    Модель миссии БПЛА.
    
    Attributes:
        uav_id: идентификатор БПЛА (первичный ключ, внешний ключ)
        is_accepted: статус принятия миссии
    """
    __tablename__ = 'mission'
    uav_id = db.Column(db.Integer, db.ForeignKey('uav.id'), primary_key=True)
    is_accepted = db.Column(db.Boolean, default=False)
    
    def __repr__(self):
        return '<Mission {}>'.format(self.id)


class MissionStep(db.Model):
    """
    Модель шага миссии.
    
    Attributes:
        mission_id: идентификатор миссии (внешний ключ)
        step: номер шага
        operation: операция, выполняемая на данном шаге
    """
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
    """
    Модель телеметрии БПЛА.
    
    Attributes:
        uav_id: идентификатор БПЛА (первичный ключ, внешний ключ)
        lat: широта
        lon: долгота
        alt: высота
        azimuth: азимут
        dop: снижение точности
        sats: количество спутников
    """
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