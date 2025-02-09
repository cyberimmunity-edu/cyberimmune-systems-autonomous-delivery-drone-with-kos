#
# Change drone communication interval with the server
#
HOST="${1:-"127.0.0.1"}"
PORT="8080"
# authentication
# example answer: 935d2528b5021242512ed188ad85be8f
TOKEN=$(curl -s "http://$HOST:$PORT/admin/auth?login=admin&password=passw")

# set no-flight areas
echo "upload forbiddenZones state: " $(curl -X POST -F "file=@/home/user/ardupilot/exampleZones.json" -F "token=$TOKEN" "http://$HOST:$PORT/admin/import_forbidden_zones")