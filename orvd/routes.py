from flask import Blueprint, request, render_template, redirect, jsonify, send_file
from utils.api_handlers import *

bp = Blueprint('main', __name__)

@bp.route('/')
def index():
    """
    Отображает главную страницу.
    ---
    tags:
      - other
    responses:
      200:
        description: Главная страница
        content:
          text/html:
            schema:
              type: string
              example: "<html>...</html>"
    """
    return render_template('index.html')


@bp.route('/tiles/index')
def tiles_index():
    """
    Возвращает список доступных оффлайн тайлов карты.
    ---
    tags:
      - other
    responses:
      200:
        description: Cписок доступных оффлайн тайлов
        schema:
          type: array
          items:
            type: string
            example: "12/345/678"
    """
    tiles_dir = './static/resources/tiles'
    tiles_index = []
    for root, dirs, files in os.walk(tiles_dir):
        for file in files:
            if file.endswith('.png'):
                rel_dir = os.path.relpath(root, tiles_dir)
                z, x = rel_dir.split(os.sep)
                y = file.replace('.png', '')
                tiles_index.append(f"{z}/{x}/{y}")
    return jsonify(tiles_index)


@bp.route('/admin')
def admin():
    """
    Отображает страницу администратора или перенаправляет на страницу аутентификации.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации
    responses:
      200:
        description: Страница администратора или страница аутентификации
        content:
          text/html:
            schema:
              type: string
              example: "<html>...</html>"
    """
    token = request.args.get('token')
    if token == None or not check_user_token(token):
        return redirect("/admin/auth_page")
    else:
        return render_template('admin.html')


@bp.route('/admin/auth')
def admin_auth():
    """
    Обрабатывает аутентификацию администратора.
    ---
    tags:
      - admin
    parameters:
      - name: login
        in: query
        type: string
        required: true
        description: Логин администратора.
        example: admin
      - name: password
        in: query
        type: string
        required: true
        description: Пароль администратора.
        example: passw
    responses:
      200:
        description: Токен доступа в случае аутентификации, пустая строка или $-1 в случае неудачи.
        schema:
          type: string
          example: "a611b7d10ec4da8acd396e19a6afa6bd"
    """
    login = str(request.args.get('login'))
    password = str(request.args.get('password'))
    return regular_request(handler_func=admin_auth_handler, login=login, password=password)


@bp.route('/admin/auth_page')
def auth_page():
    """
    Отображает страницу аутентификации администратора.
    ---
    tags:
      - admin
    parameters:
      - name: next
        in: query
        type: string
        required: false
        description: URL для перенаправления после успешной аутентификации.
    responses:
      200:
        description: Cтраница аутентификации
        content:
          text/html:
            schema:
              type: string
              example: "<html>...</html>"
    """
    next_url = request.args.get('next', '/admin')
    return render_template('admin_auth.html', next_url=next_url)


@bp.route('/admin/arm_decision')
def arm_decision():
    """
    Обрабатывает решение администратора об арме БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: decision
        in: query
        type: integer
        required: true
        enum: [0, 1]
        description: Решение администратора (0 - арм, 1 - не арм).
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат арма.
        schema:
          type: string
          example: "$Arm: 0"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id/decision"
    """
    id = cast_wrapper(request.args.get('id'), str)
    decision = cast_wrapper(request.args.get('decision'), int)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=arm_decision_handler, token=token,
                       id=id, decision=decision)
    else:
        return bad_request('Wrong id/decision')


@bp.route('/admin/mission_decision')
def mission_decision():
    """
    Обрабатывает решение администратора о миссии.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор миссии.
      - name: decision
        in: query
        type: integer
        required: true
        enum: [0, 1]
        description: Решение администратора (0 - принять, 1 - отклонить).
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: $OK в случае успеха, $-1 в случае, если БПЛА с таким id не найден
        schema:
          type: string
          example: "$OK"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id/decision"
    """
    id = cast_wrapper(request.args.get('id'), str)
    decision = cast_wrapper(request.args.get('decision'), int)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=mission_decision_handler, token=token,
                       id=id, decision=decision)
    else:
        return bad_request('Wrong id/decision')


@bp.route('/admin/force_disarm')
def force_disarm():
    """
    Принудительно дизармит указанный БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат принудительного дизарма.
        schema:
          type: string
          example: "$OK"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=force_disarm_handler, token=token, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/admin/force_disarm_all')
def force_disarm_all():
    """
    Принудительно дизармит все БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат принудительного дизарма всех БПЛА.
        schema:
          type: string
          example: "$OK"
    """
    token = request.args.get('token')
    return authorized_request(handler_func=force_disarm_all_handler, token=token)


@bp.route('/admin/kill_switch')
def admin_kill_switch():
    """
    Активирует аварийное выключение для указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат активации аварийного выключения.
        schema:
          type: string
          example: "$OK"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=admin_kill_switch_handler, token=token, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/admin/get_state')
def get_state():
    """
    Получает текущее состояние указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Текущее состояние БПЛА.
        schema:
          type: string
          example: "В сети"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=get_state_handler, token=token, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/admin/get_mission_state')
def get_mission_state():
    """
    Получает текущее состояние миссии для указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Текущее состояние миссии (0 - принята, 1 - не принята, , 2 - нужна повторная проверка, $-1 - не найдена).
        schema:
          type: string
          example: "0"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=get_mission_state_handler, token=token, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/admin/get_mission')
def get_mission():
    """
    Получает полетное задание для указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Полетное задание.
        schema:
          type: string
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=get_mission_handler, token=token, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/admin/get_telemetry')
def get_telemetry():
    """
    Получает телеметрию для указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Телеметрия БПЛА.
        content:
          application/json:
            schema:
              type: object
              properties:
                lat:
                  type: number
                  example: 100.1
                lon:
                  type: number
                  example: 50.2
                alt:
                  type: number
                  example: 10
                azimuth:
                  type: number
                  example: 1
                dop:
                  type: number
                  example: 1.5
                sats:
                  type: integer
                  example: 12
                speed:
                  type: float
                  examle: 2.5
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=get_telemetry_handler, token=token, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/logs/get_telemetry_csv')
def get_telemetry_csv():
    """
    Получает всю телеметрию для указанного БПЛА в формате CSV.
    ---
    tags:
      - logs
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
    responses:
      200:
        description: Телеметрия БПЛА в формате CSV.
        content:
          text/csv:
            schema:
              type: string
              example: "record_time,lat,lon,alt,azimuth,dop,sats,speed\n2024-09-15 16:46:38.302348,100.1,50.2,10,1,1.5,12,2.5\n"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    if id:
        return regular_request(handler_func=get_telemetry_csv_handler, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/admin/get_waiter_number')
def get_waiter_number():
    """
    Получает количество ожидающих арма БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Количество ожидающих арма БПЛА.
        schema:
          type: string
          example: "3"
    """
    token = request.args.get('token')
    return authorized_request(handler_func=get_waiter_number_handler, token=token)


@bp.route('/admin/get_id_list')
def get_id_list():
    """
    Получает список идентификаторов всех БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Список идентификаторов всех БПЛА.
        schema:
          type: string
          example: "[1, 2, 3]"
    """
    token = request.args.get('token')
    return authorized_request(handler_func=get_id_list_handler, token=token)


@bp.route('/admin/change_fly_accept')
def change_fly_accept():
    """
    Изменяет разрешение на полет для указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: decision
        in: query
        type: integer
        required: true
        enum: [0, 1]
        description: Решение администратора (0 - разрешить, 1 - запретить).
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат изменения разрешения на полет.
        schema:
          type: string
          example: "$OK"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id/decision"
    """
    id = cast_wrapper(request.args.get('id'), str)
    decision = cast_wrapper(request.args.get('decision'), int)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=change_fly_accept_handler, token=token,
                       id=id, decision=decision)
    else:
        return bad_request('Wrong id/decision')
      

@bp.route('/admin/get_forbidden_zones')
def get_forbidden_zones():
    """
    Возвращает все запрещенные зоны.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации
    responses:
      200:
        description: GeoJSON с запрещенными зонами
        content:
          application/json:
            schema:
              type: object
      401:
        description: Неавторизованный доступ
    """
    token = request.args.get('token')
    if token is None or not check_user_token(token):
        return jsonify({"error": "Unauthorized"}), 401
    
    return authorized_request(handler_func=get_forbidden_zones_handler, token=token)


@bp.route('/admin/get_forbidden_zone')
def get_forbidden_zone():
    """
    Получает информацию о запрещенной для полета зоне по ее имени.
    ---
    tags:
      - admin
    parameters:
      - name: name
        in: query
        type: string
        required: true
        description: Имя запрещенной зоны.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Координаты запрещенной зоны.
        schema:
          type: string
          example: "[[100, 50], [100, 0]]"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong name"
    """
    name = str(request.args.get('name'))
    token = request.args.get('token')
    if name:
        return authorized_request(handler_func=get_forbidden_zone_handler, token=token, name=name)
    else:
        return bad_request('Wrong name')


@bp.route('/admin/get_forbidden_zones_names')
def get_forbidden_zones_names():
    """
    Получает список имен всех запрещенных для полета зон.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Список имен всех запрещенных зон.
        schema:
          type: string
          example: "['test1', 'test2']"
    """
    token = request.args.get('token')
    return authorized_request(handler_func=get_forbidden_zones_names_handler, token=token)


@bp.route('/admin/set_forbidden_zone', methods=['POST'])
def set_forbidden_zone():
    """
    Устанавливает запрещенную для полета зону.
    ---
    tags:
      - admin
    parameters:
      - name: geometry
        in: body
        type: array
        items:
          type: array
          items:
            type: number
        required: true
        description: Геометрия зоны.
      - name: name
        in: body
        type: string
        required: true
        description: Имя зоны.
      - name: token
        in: body
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат установки запрещенной зоны.
        schema:
          type: string
          example: "$OK"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong name"
    """
    data = request.json
    geometry = data.get('geometry')
    name = data.get('name')
    token = data.get('token')
    if name:
        return authorized_request(handler_func=set_forbidden_zone_handler, token=token, name=name, geometry=geometry)
    else:
        return bad_request('Wrong name')


@bp.route('/admin/delete_forbidden_zone', methods=['DELETE'])
def delete_forbidden_zone():
    """
    Удаляет запрещенную для полета зону по ее имени.
    ---
    tags:
      - admin
    parameters:
      - name: name
        in: query
        type: string
        required: true
        description: Имя запрещенной зоны.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат удаления запрещенной зоны.
        schema:
          type: string
          example: "$OK"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong name"
    """
    name = str(request.args.get('name'))
    token = request.args.get('token')
    if name:
        return authorized_request(handler_func=delete_forbidden_zone_handler, token=token, name=name)
    else:
        return bad_request('Wrong name')
  

@bp.route('/admin/forbidden_zones')
def forbidden_zones():
    """
    Отображает страницу управления запрещенными зонами.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации
    responses:
      200:
        description: Страница управления запрещенными зонами
        content:
          text/html:
            schema:
              type: string
              example: "<html>...</html>"
    """
    token = request.args.get('token')
    if token is None or not check_user_token(token):
        return redirect(f"/admin/auth_page?next={request.path}")
    else:
        return render_template('forbidden_zones.html', token=token)


@bp.route('/logs')
def logs_page():
    """
    Отображает страницу логов.
    ---
    tags:
      - logs
    responses:
      200:
        description: Страница логов.
        content:
          text/html:
            schema:
              type: string
    """
    return render_template('logs.html')


@bp.route('/logs/get_logs')
def get_logs():
    """
    Получает логи для указанного БПЛА.
    ---
    tags:
      - logs
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
    responses:
      200:
        description: Логи для указанного БПЛА или $-1, если логи не найдены.
        schema:
          type: string
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    if id:
        return regular_request(handler_func=get_logs_handler, id=id)
    else:
        return bad_request('Wrong id')

@bp.route('/api/nmission')
def revise_mission():
  """_summary_

  Returns:
      _type_: _description_
  """
  id = cast_wrapper(request.args.get('id'), str)
  mission = cast_wrapper(request.args.get('mission'), str)
  sig = request.args.get('sig')
  if id:
      return signed_request(handler_func=revise_mission_handler, verifier_func=mock_verifier, signer_func=sign,
                            query_str=f'/api/nmission?id={id}&mission={mission}', key_group=f'kos{id}', sig=sig, id=id, mission=mission)
  else:
      return bad_request('Wrong id')
  

@bp.route('/api/logs')
def save_logs():
    """
    Сохраняет лог для указанного БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: log
        in: query
        type: string
        required: true
        description: Строка с логами для сохранения.
    responses:
      200:
        description: Лог успешно сохранен.
        schema:
          type: string
          example: "$OK"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    log = cast_wrapper(request.args.get('log'), str)
    if id:
        return regular_request(handler_func=save_logs_handler, id=id, log=log)
    else:
        return bad_request('Wrong id')


@bp.route('/mission_sender')
def mission_sender():
    """
    Отображает страницу отправки миссии.
    ---
    tags:
      - mission sender
    responses:
      200:
        description: Страница отправки миссии.
        content:
          text/html:
            schema:
              type: string
    """
    return render_template('mission_sender.html')


@bp.route('/mission_sender/fmission_ms', methods=['POST'])
def fmission():
    """
    Обрабатывает отправку миссии от Mission Sender.
    ---
    tags:
      - mission sender
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
      - name: mission_str
        in: body
        type: string
        required: true
        description: Строка с полетным заданием.
    responses:
      200:
        description: Статус верификации миссии.
        example: "Mission accepted."
      400:
        description: Неверный идентификатор.
        example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    mission_str = request.get_data().decode()
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=fmission_ms_handler, verifier_func=mock_verifier, signer_func=sign,
                          query_str=mission_str, key_group=f'ms{id}', sig=sig, id=id, mission_str=mission_str)
    else:
        return bad_request('Wrong id')

  
@bp.route('/mission_sender/key')
def key_ms_exchange():
    """
    Обрабатывает обмен ключами с Mission Sender.
    ---
    tags:
      - mission sender
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор отправителя миссии.
    responses:
      200:
        description: Строка с открытым ключом ОРВД (hex, без 0x).
        example: "$Key: {n} {e}"
      400:
        description: Неверный идентификатор.
        example: "Wrong id"
      409:
        description: Конфликт ключей
    """
    id = cast_wrapper(request.args.get('id'), str)
    if id:
        return regular_request(handler_func=key_ms_exchange_handler, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/api/key')
def key_kos_exchange():
    """
    Обрабатывает обмен ключами с БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: n
        in: query
        type: string
        required: true
        description: Параметр n для ключа.
      - name: e
        in: query
        type: string
        required: true
        description: Параметр e для ключа.
    responses:
      200:
        description: Строка с открытым ключом ОРВД (hex, без 0x).
        schema:
          type: string
          example: "$Key: {n} {e}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      409:
        description: Конфликт ключей
    """
    id = cast_wrapper(request.args.get('id'), str)
    n = request.args.get('n')
    e = request.args.get('e')
    if id:
        return regular_request(handler_func=key_kos_exchange_handler, id=id, n=n, e=e)
    else:
        return bad_request('Wrong id')


@bp.route('/api/arm')
def arm_request():
    """
    Обрабатывает запрос на арм от БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Состояние арма (0 - вкл, 1 - выкл) и время до следующего сеанса связи или $-1, если БПЛА не найден.
        schema:
          type: string
          example: "$Arm {state}$Delay {time in seconds}#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=arm_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/arm?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/api/auth')
def auth():
    """
    Обрабатывает запрос на аутентификацию от БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Строка подтверждения аутентификации.
        schema:
          type: string
          example: "$Auth id={id}#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    if id is not None:
        sig = request.args.get('sig')
        return signed_request(handler_func=auth_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/auth?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/api/fly_accept')
def fly_accept():
    """
    Обрабатывает запрос на разрешение полета от БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Состояние арма (0 - вкл, 1 - выкл) и время до следующего сеанса связи или $-1, если БПЛА не найден.
        schema:
          type: string
          example: "$Arm: {state}$Delay {time in seconds}#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=fly_accept_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/fly_accept?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')

 
@bp.route('/api/flight_info')
def flight_info():
    """
    Обрабатывает запрос на информацию полета от БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Состояние полета (-1 в случае kill switch, 0 в случае продолжения полета, 1 в случае приостановки полета), хэш запретных зон и время до следующего сеанса связи или $-1, если БПЛА не найден.
        schema:
          type: string
          example: "$Flight {status}$ForbiddenZonesHash {hash}$Delay {time in seconds}#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if not modes["flight_info_response"]:
        return '', 403
    elif id:
        return signed_request(handler_func=flight_info_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/flight_info?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')
      
      
@bp.route('/api/telemetry')
def telemetry():
    """
    Обрабатывает получение телеметрии от БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
      - name: lat
        in: query
        type: string
        required: true
        description: Широта.
      - name: lon
        in: query
        type: string
        required: true
        description: Долгота.
      - name: alt
        in: query
        type: string
        required: true
        description: Высота.
      - name: azimuth
        in: query
        type: string
        required: true
        description: Азимут.
      - name: dop
        in: query
        type: string
        required: true
        description: DOP (Dilution of Precision).
      - name: sats
        in: query
        type: string
        required: true
        description: Количество спутников.
      - name: speed
        in: query
        type: string
        required: true
        description: Скорость.
    responses:
      200:
        description: Состояние арма (0 - вкл, 1 - выкл) или $-1, если БПЛА не найден.
        schema:
          type: string
          example: "$Arm: {state}#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    lat = request.args.get('lat')
    lon = request.args.get('lon')
    alt = request.args.get('alt')
    azimuth = request.args.get('azimuth')
    dop = request.args.get('dop')
    sats = request.args.get('sats')
    speed = request.args.get('speed')
    if id:
        return signed_request(handler_func=telemetry_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/telemetry?id={id}&lat={lat}&lon={lon}&alt={alt}&azimuth={azimuth}&dop={dop}&sats={sats}&speed={speed}',
                          key_group=f'kos{id}', sig=sig, id=id, lat=lat, lon=lon, alt=alt, azimuth=azimuth, dop=dop, sats=sats, speed=speed)
    else:
        return bad_request('Wrong id')
    
@bp.route('/api/kill_switch')
def kill_switch():
    """
    Обрабатывает запрос на аварийное выключение от БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Состояние аварийного отключения (0 - вкл, 1 - выкл) или $-1, если БПЛА не найден.
        schema:
          type: string
          example: "$KillSwitch: {state}#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=kill_switch_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/kill_switch?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/api/fmission_kos')
def fmission_kos():
    """
    Обрабатывает запрос на получение миссии от БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Полетное задание, если найдено. Если не найдено, то $-1
        schema:
          type: string
          example: "$FlightMission {mission_string}#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=fmission_kos_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/fmission_kos?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')
    

@bp.route('/api/get_all_forbidden_zones')
def get_all_forbidden_zones():
    """
    Возвращает информацию о всех запрещенных для полетов зонах для БПЛА.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Информация о запрещенных зонах.
        schema:
          type: string
          example: "$ForbiddenZones 1&test_name&2&50_100&0_100#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=get_all_forbidden_zones_handler, verifier_func=verify, signer_func=sign,
                          query_str=f'/api/get_all_forbidden_zones?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')
      
      
@bp.route('/api/get_forbidden_zones_delta')
def get_forbidden_zones_delta():
    """
    Возвращает дельту изменений в запрещенных для полета зонах.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: Дельта изменений в запрещенных зонах.
        schema:
          type: string
          example: "$ForbiddenZonesDelta 1&test_name&modified&2&50_100&0_100#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=get_forbidden_zones_delta_handler, verifier_func=verify, signer_func=sign,
                              query_str=f'/api/get_forbidden_zones_delta?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')
      
      
@bp.route('/api/forbidden_zones_hash')
def forbidden_zones_hash():
    """
    Возвращает SHA-256 хэш строки запрещенных для полета зон.
    ---
    tags:
      - api
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: sig
        in: query
        type: string
        required: true
        description: Подпись запроса.
    responses:
      200:
        description: SHA-256 хэш строки запрещенных зон.
        schema:
          type: string
          example: "$ForbiddenZonesHash 3a7bd3e2360a3d5f0d5f0d5f0d5f0d5f0d5f0d5f0d5f0d5f0d5f0d5f0d5f0d5f#{signature}"
      400:
        description: Неверный идентификатор.
        schema:
          type: string
          example: "Wrong id"
      403:
        description: Ошибка проверки подписи.
    """
    id = cast_wrapper(request.args.get('id'), str)
    sig = request.args.get('sig')
    if id:
        return signed_request(handler_func=get_forbidden_zones_hash_handler, verifier_func=verify, signer_func=sign,
                              query_str=f'/api/forbidden_zones_hash?id={id}', key_group=f'kos{id}', sig=sig, id=id)
    else:
        return bad_request('Wrong id')
      
      
@bp.route('/admin/export_forbidden_zones')
def export_forbidden_zones():
    """
    Экспортирует все запрещенные зоны в файл.
    ---
    tags:
      - admin
    parameters:
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации
    responses:
      200:
        description: Файл с запрещенными зонами
        content:
          application/json:
            schema:
              type: string
      401:
        description: Неавторизованный доступ
    """
    token = request.args.get('token')
    if token is None or not check_user_token(token):
        return jsonify({"error": "Unauthorized"}), 401

    return send_file(FORBIDDEN_ZONES_PATH, as_attachment=True, attachment_filename='forbidden_zones.json')


@bp.route('/admin/import_forbidden_zones', methods=['POST'])
def import_forbidden_zones():
    """
    Импортирует запрещенные зоны из файла.
    ---
    tags:
      - admin
    parameters:
      - name: file
        in: formData
        type: file
        required: true
        description: Файл с запрещенными зонами
      - name: token
        in: formData
        type: string
        required: true
        description: Токен аутентификации
    responses:
      200:
        description: Успешный импорт зон
      400:
        description: Ошибка импорта зон
      401:
        description: Неавторизованный доступ
    """
    token = request.form.get('token')
    if token is None or not check_user_token(token):
        return jsonify({"error": "Unauthorized"}), 401

    file = request.files.get('file')
    if file is None:
        return jsonify({"error": "No file provided"}), 400

    try:
        with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
            old_zones = json.load(f)
        
        file.save(FORBIDDEN_ZONES_PATH)
        
        with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
            new_zones = json.load(f)
        
        compute_and_save_forbidden_zones_delta(old_zones, new_zones)
        
        return jsonify({"status": "success"}), 200
    except Exception as e:
        print(e)
        return jsonify({"error": "Failed to save file"}), 400
      

@bp.route('/admin/get_delay')
def get_delay():
    """
    Получает время до следующего сеанса связи для указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Время до следующего сеанса связи.
        schema:
          type: string
          example: "30"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id"
    """
    id = cast_wrapper(request.args.get('id'), str)
    token = request.args.get('token')
    if id:
        return authorized_request(handler_func=get_delay_handler, token=token, id=id)
    else:
        return bad_request('Wrong id')


@bp.route('/admin/set_delay')
def set_delay():
    """
    Устанавливает время до следующего сеанса связи для указанного БПЛА.
    ---
    tags:
      - admin
    parameters:
      - name: id
        in: query
        type: string
        required: true
        description: Идентификатор БПЛА.
      - name: delay
        in: query
        type: integer
        required: true
        description: Время до следующего сеанса связи.
      - name: token
        in: query
        type: string
        required: true
        description: Токен аутентификации.
    responses:
      200:
        description: Результат установки времени до следующего сеанса связи.
        schema:
          type: string
          example: "$OK"
      400:
        description: Какие-то параметры неверные
        schema:
          type: string
          example: "Wrong id/delay"
    """
    id = cast_wrapper(request.args.get('id'), str)
    delay = cast_wrapper(request.args.get('delay'), int)
    token = request.args.get('token')
    if id and delay is not None:
        return authorized_request(handler_func=set_delay_handler, token=token, id=id, delay=delay)
    else:
        return bad_request('Wrong id/delay')
      
      
@bp.route('/admin/revise_mission_decision')
def revise_mission_decision():
    id = cast_wrapper(request.args.get('id'), str)
    decision = cast_wrapper(request.args.get('decision'), int)
    token = request.args.get('token')
    if id is not None and decision is not None:
        return authorized_request(handler_func=revise_mission_decision_handler, token=token, id=id, decision=decision)
    else:
        return bad_request('Wrong id/decision')
      
      
@bp.route('/admin/get_display_mode')
def get_display_mode():
    token = request.args.get('token')
    return authorized_request(handler_func=get_display_mode_handler, token=token)
      

@bp.route('/admin/toggle_display_mode')
def toggle_display_mode():
    token = request.args.get('token')
    return authorized_request(handler_func=toggle_display_mode_handler, token=token)
  
@bp.route('/admin/get_flight_info_response_mode')
def get_flight_info_response_mode():
    token = request.args.get('token')
    return authorized_request(handler_func=get_flight_info_response_mode_handler, token=token)
      

@bp.route('/admin/toggle_flight_info_response_mode')
def toggle_flight_info_response_mode():
    token = request.args.get('token')
    return authorized_request(handler_func=toggle_flight_info_response_mode_handler, token=token)
  
@bp.route('/admin/get_all_data')
def get_all_data():
    token = request.args.get('token')
    return authorized_request(handler_func=get_all_data_handler, token=token)