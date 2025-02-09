#
# Change drone communication interval with the server
#
HOST="${1:-"127.0.0.1"}"
PORT="8080"
# authentication
# example answer: 935d2528b5021242512ed188ad85be8f
TOKEN=$(curl -s "http://$HOST:$PORT/admin/auth?login=admin&password=passw")

# set communication frequency
echo "AFCS communication interval change status: " $(curl -s "http://$HOST:$PORT/admin/set_delay?id=52:58:00:12:34:bb&delay=1&token=$TOKEN")