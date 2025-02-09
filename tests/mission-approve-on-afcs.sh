#
# Change drone communication interval with the server
#
HOST="${1:-"127.0.0.1"}"
PORT="8080"
# authentication
# example answer: 935d2528b5021242512ed188ad85be8f
TOKEN=$(curl -s "http://$HOST:$PORT/admin/auth?login=admin&password=passw")

# approve mission
echo "mission approval is: "$(curl -s "http://$HOST:$PORT/admin/revise_mission_decision?id=52:58:00:12:34:bb&token=$TOKEN&decision=0")