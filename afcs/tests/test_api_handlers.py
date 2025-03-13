from utils.api_handlers import bad_request, regular_request


def test_bad_request():
    test_message = 'Test message'
    response, status_code = bad_request(test_message)
    assert response == test_message
    assert status_code == 400
    
def test_regular_request_success():
    def handler_func_success(**kwargs):
        return "Success"
    
    response, status_code = regular_request(handler_func_success)

    assert response == "Success"
    assert status_code == 200

def test_regular_request_with_kwargs():
    def handler_func_with_kwargs(param1, param2):
        return f"Received {param1} and {param2}"

    response, status_code = regular_request(handler_func_with_kwargs, param1="value1", param2="value2")

    assert response == "Received value1 and value2"
    assert status_code == 200

def test_regular_request_exception():
    def handler_func_exception(**kwargs):
        raise Exception("Error")

    response, status_code = regular_request(handler_func_exception)

    assert response == "Conflict."
    assert status_code == 409