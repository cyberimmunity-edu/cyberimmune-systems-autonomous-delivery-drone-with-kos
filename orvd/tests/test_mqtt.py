import pytest
import paho.mqtt.client as mqtt
import time
import json
from unittest.mock import patch

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TELEMETRY_TOPIC = "api/telemetry"
MQTT_MISSION_TOPIC = 'api/mission'
TIMEOUT = 5

@pytest.mark.skip(reason="local mosquitto test")
class TestMQTTPublish:
    @pytest.fixture(scope="function")
    def mqtt_client(self):
        """Фикстура для создания и настройки MQTT клиента."""
        client = mqtt.Client()
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        yield client
        client.loop_stop()
        client.disconnect()

    def test_publish_to_telemetry_topic(self, mqtt_client):
        """Тест публикации данных в топик api/telemetry."""
        
        lat = -35.3632621 * 1e7
        lon = 149.1652374 * 1e7
        alt = 584.09 * 1e2
        
        message = f'id={1}&lat={lat}&lon={lon}&alt={alt}&azimuth={0}&dop={1.2}&sats={12}&speed={0}'
        
        published = False
        def on_publish(client, userdata, mid):
            nonlocal published
            published = True

        mqtt_client.on_publish = on_publish
        result = mqtt_client.publish(MQTT_TELEMETRY_TOPIC, message, qos=1)
        status = result[0]
        start_time = time.time()
        while not published and time.time() - start_time < TIMEOUT:
            time.sleep(0.1)
        assert status == 0, f"Не удалось отправить сообщение в топик {MQTT_TELEMETRY_TOPIC}"
        assert published, f"Сообщение не было опубликовано в течение {TIMEOUT} секунд"
        
    def test_publish_to_mission_topic(self, mqtt_client):
        """Тест публикации миссии в топик api/mission."""
        
        class WPItem:
            def __init__(self, command, index=None, currentwp=0, frame=3, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, autocontinue=1):
                self.index = str(index)
                self.command = str(command)
                self.currentwp = str(currentwp)
                self.frame = str(frame)
                self.p1 = str(p1)
                self.p2 = str(p2)
                self.p3 = str(p3)
                self.p4 = str(p4)
                self.p5 = str(p5)
                self.p6 = str(p6)
                self.p7 = str(p7)
                self.autocontinue = str(autocontinue)
                
            def __str__(self):
                return '\t'.join([self.index, self.currentwp, self.frame, self.command, self.p1, self.p2, self.p3,
                                  self.p4, self.p5, self.p6, self.p7, self.autocontinue])
        
        def construct_mission():
            mission_str = "QGC WPL 110\n"
            home = lambda lat, lon, alt: WPItem(command=16, currentwp=1, frame=0, p5=lat, p6=lon, p7=alt)
            #takeoff = lambda alt: WPItem(command=22, p7=alt)
            waypoint = lambda lat, lon, alt: WPItem(command=16, p5=lat, p6=lon, p7=alt)
            set_servo = lambda number, pwm: WPItem(command=183, p1=number, p2=pwm)
            delay = lambda delay: WPItem(command=93, p1=delay)
            #land = lambda lat=0, lon=0, alt=0: WPItem(command=21, p5=lat, p6=lon, p7=alt)
            
            item_list = [
                home(-35.3632621, 149.1652374, 584.09),
                waypoint(-35.36107360, 149.16206360, 0),
                delay(3),
                waypoint(-35.36719810, 149.16332960, 0),
                set_servo(5, 1200),
                waypoint(-35.36719810, 149.16332960, 0),
                set_servo(5, 1800),
                waypoint(-35.36438090, 149.15895220, 0),
            ]
            
            index = 0
            for item in item_list:
                item.index = str(index)
                mission_str += str(item) + '\n'
                index += 1
                
            return mission_str
        
        data = {
            "id": 1,
            "mission_str": construct_mission()
        }
        
        payload = json.dumps(data)
        
        published = False
        def on_publish(client, userdata, mid):
            nonlocal published
            published = True

        mqtt_client.on_publish = on_publish
        result = mqtt_client.publish(MQTT_MISSION_TOPIC, payload, qos=1)
        status = result[0]
        start_time = time.time()
        while not published and time.time() - start_time < TIMEOUT:
            time.sleep(0.1)
        assert status == 0, f"Не удалось отправить сообщение в топик {MQTT_MISSION_TOPIC}"
        assert published, f"Сообщение не было опубликовано в течение {TIMEOUT} секунд"

    @patch('paho.mqtt.client.Client.connect')        
    def test_connection_failure(self, mock_connect):
        """Тест на обработку ошибки подключения к брокеру"""
        mock_connect.side_effect = ConnectionRefusedError
        
        client = mqtt.Client()
        with pytest.raises(ConnectionRefusedError):
            client.connect("invalid_host", 1883)