#
# Load mission to ORVD step by step
#
HOST="${1:-"127.0.0.1"}"
PORT="8080"
# authentication
# example answer: 935d2528b5021242512ed188ad85be8f
TOKEN=$(curl -s "http://$HOST:$PORT/admin/auth?login=admin&password=passw")

# get id
# example answer: [2]
echo "id is: " $(curl -s "http://$HOST:$PORT/admin/get_id_list?token=$TOKEN")

# get state
# example answer: В сети
echo "state is: "$(curl -s "http://$HOST:$PORT/admin/get_state?id=52:58:00:12:34:bb&token=$TOKEN")

# get mission
# curl "http://$HOST:$PORT/admin/get_mission?id=52:58:00:12:34:bb&token=$TOKEN"
# $-1

# get mission state
# curl "http://$HOST:$PORT/admin/get_mission_state?id=52:58:00:12:34:bb&token=$TOKEN"
# $-1

# get mission upload key
# $Key: b81f14d13374fc7f2610a160bed2b5a810496f36976b9ff05fb2fbdb5ad6e4925146e7e25b137c42d3fe511715fd688443c9c52957c21156b0707d1cf2d141d8d1baaa2f29599b9bfd5079811b393d6bc71f9d25ba058248e1e2b731b43fd8caa48f19aa9483a8232ad53eebf4eb7a28a77650796bedf490edbd1c92020677a1 10001
echo "mission upload key is: " $(curl -s "http://$HOST:$PORT/mission_sender/key?id=52:58:00:12:34:bb")

# mission upload POST
# $OK#8c2cb1bc1ca84e0070c0b32ac9185c9edc7e4bec84d9a73da7c087178d90f13410cc8bae5631f78d3cb166c1001f1524c3b8641933d34a4c77aec751d980e40397a8bd2c4db741532137c0eb9b3bd921d0ca8040658c7b0b9a310712662d1b6ab70a0b57dac3dce7d0fdaf296852e4cb9cb5c0857308e5388fc25e2a9cd5b01c
echo "upload exampleMission state: " $(curl -s -X POST --data-binary "@/home/user/ardupilot/exampleMission.txt" "http://$HOST:$PORT/mission_sender/fmission_ms?id=52:58:00:12:34:bb&sig=0xaa")

# mission acknowledge
# example answer: $OK
echo "mission acknowledge: " $(curl -s "http://$HOST:$PORT/admin/mission_decision?id=52:58:00:12:34:bb&decision=0&token=$TOKEN")

# get mission
# example answer: H-35.3633463_149.1652273_587.05&T5.0&W0.0_-35.3631122_149.1651964_5.0&W0.0_-35.3632741_149.1653869_2.0&W0.0_-35.3630641_149.1655062_5.0&W0.0_-35.3633091_149.1655652_5.0&S10.0_2000.0&W0.0_-35.363401_149.1654137_5.0&W0.0_-35.36334629_149.16522726_5.0&L-35.3633463_149.1652273_587.05
echo "get mission: " $(curl -s "http://$HOST:$PORT/admin/get_mission?id=52:58:00:12:34:bb&token=$TOKEN")

# accept mission
# example answer: $OK
echo "accept mission: " $(curl -s "http://$HOST:$PORT/admin/change_fly_accept?id=52:58:00:12:34:bb&decision=0&token=$TOKEN")

