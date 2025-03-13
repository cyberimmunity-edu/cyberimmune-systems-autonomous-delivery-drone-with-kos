import secrets, os
from orvd_server import db
from hashlib import sha256


def add_and_commit(entity: db.Model):
    """
    Добавляет сущность в сессию и фиксирует изменения.

    Args:
        entity (db.Model): Сущность для добавления и фиксации.

    Return:
        None
    """
    db.session.add(entity)
    db.session.commit()
    
    
def add_changes(entity: db.Model):
    """
    Добавляет сущность в сессию без фиксации изменений.

    Args:
        entity (db.Model): Сущность для добавления.

    Return:
        None
    """
    db.session.add(entity)
    
    
def commit_changes():
    """
    Фиксирует изменения в базе данных.

    Args:
        None

    Return:
        None
    """
    db.session.commit()
    
    
def delete_entity(entity: db.Model):
    """
    Удаляет сущность из сессии.

    Args:
        entity (db.Model): Сущность для удаления.

    Return:
        None
    """
    db.session.delete(entity)


def get_entity_by_key(entity: db.Model, key_value):
    """
    Получает сущность по ключевому значению.

    Args:
        entity (db.Model): Модель сущности.
        key_value: Значение ключа для поиска.

    Return:
        db.Model: Найденная сущность или None.
    """
    return entity.query.get(key_value)


def generate_user(user_model: db.Model):
    """
    Создает пользователя-администратора и добавляет его в базу данных.

    Args:
        user_model (db.Model): Модель пользователя.

    Return:
        None
    """
    user_entity = user_model(username=str(os.getenv("ADMIN_LOGIN")),
                       password_hash=hex(int.from_bytes(sha256(str(os.getenv("ADMIN_PASSW")).encode()).digest(),
                                                        byteorder='big', signed=False))[2:],
                       access_token=secrets.token_hex(16))
    db.session.add(user_entity)
    db.session.commit()
    

def get_entities_by_field(entity: db.Model, field, field_value, order_by_field=None) -> list:
    """
    Получает список сущностей по значению поля.

    Args:
        entity (db.Model): Модель сущности.
        field: Поле для фильтрации.
        field_value: Значение поля для фильтрации.
        order_by_field: Поле для сортировки (опционально).

    Return:
        list: Список найденных сущностей.
    """
    entities = entity.query.filter(field==field_value)
    if order_by_field == None:
        return entities
    else:
        return entities.order_by(order_by_field)
    

def get_entities_by_field_with_order(entity: db.Model, field, field_value, order_by_field) -> list:
    """
    Получает отсортированный список сущностей по значению поля.

    Args:
        entity (db.Model): Модель сущности.
        field: Поле для фильтрации.
        field_value: Значение поля для фильтрации.
        order_by_field: Поле для сортировки.

    Return:
        list: Список найденных сущностей.
    """
    return entity.query.filter(field==field_value).order_by(order_by_field)


def clean_db(models_to_clean):
    """
    Очищает базу данных, удаляя все записи указанных моделей.

    Args:
        models_to_clean: Список моделей для очистки.

    Return:
        None
    """
    try:
        for model in models_to_clean:
            db.session.query(model).delete()
        db.session.commit()
    except:
        db.session.rollback()
    