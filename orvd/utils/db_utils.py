import secrets, os
from orvd_server import db
from hashlib import sha256


def add_and_commit(entity: db.Model):
    db.session.add(entity)
    db.session.commit()
    
def add_changes(entity: db.Model):
    db.session.add(entity)
    
def commit_changes():
    db.session.commit()
    
def delete_entity(entity: db.Model):
    db.session.delete(entity)

def get_entity_by_key(entity: db.Model, key_value):
    return entity.query.get(key_value)

def generate_user(user_model: db.Model):
    user_entity = user_model(username=str(os.getenv("ADMIN_LOGIN")),
                       password_hash=hex(int.from_bytes(sha256(str(os.getenv("ADMIN_PASSW")).encode()).digest(),
                                                        byteorder='big', signed=False))[2:],
                       access_token=secrets.token_hex(16))
    db.session.add(user_entity)
    db.session.commit()
    


def get_entities_by_field(entity: db.Model, field, field_value, order_by_field=None) -> list:
    entities = entity.query.filter(field==field_value)
    if not order_by_field:
        return entities
    else:
        return entities.order_by(order_by_field)


def clean_db(models_to_clean):
    try:
        for model in models_to_clean:
            db.session.query(model).delete()
        db.session.commit()
    except:
        db.session.rollback()
    