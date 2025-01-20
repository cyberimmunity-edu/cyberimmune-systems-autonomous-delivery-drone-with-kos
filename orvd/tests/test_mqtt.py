import pytest
import paho.mqtt.client as mqtt
import time
import json
from unittest.mock import patch

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "api/telemetry"
TIMEOUT = 5

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
        
        data_query = f"id={1}&lat={50}&lon={50.1}&alt={852}&azimuth={0}&dop={1.2}&sats={12}&speed={0}"
        message = json.dumps(data_query)
        
        published = False
        def on_publish(client, userdata, mid):
            nonlocal published
            published = True

        mqtt_client.on_publish = on_publish
        result = mqtt_client.publish(MQTT_TOPIC, message, qos=1)
        status = result[0]
        start_time = time.time()
        while not published and time.time() - start_time < TIMEOUT:
            time.sleep(0.1)
        assert status == 0, f"Не удалось отправить сообщение в топик {MQTT_TOPIC}"
        assert published, f"Сообщение не было опубликовано в течение {TIMEOUT} секунд"

    @patch('paho.mqtt.client.Client.connect')        
    def test_connection_failure(self, mock_connect):
        """Тест на обработку ошибки подключения к брокеру"""
        mock_connect.side_effect = ConnectionRefusedError
        
        client = mqtt.Client()
        with pytest.raises(ConnectionRefusedError):
            client.connect("invalid_host", 1883)