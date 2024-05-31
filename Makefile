.PHONY: docker-compose-stop

docker: docker-image

docker-image: docker-image-simulator docker-image-orvd

docker-image-simulator:
	docker build ./ -t simulator

docker-image-orvd:
	docker build -f orvd.Dockerfile -t orvd ./

clean-docker-compose:
	docker-compose -f docker-compose-offline.yml down
	docker-compose -f docker-compose-online.yml down
	docker-compose -f docker-compose-offline-obstacles.yml down
	docker-compose -f docker-compose-online-obstacles.yml down
	docker-compose -f tests/e2e-offline-docker-compose.yml down
	docker-compose -f tests/e2e-online-docker-compose.yml down
	docker-compose -f tests/e2e-offline-obstacles-docker-compose.yml down
	docker-compose -f tests/e2e-online-obstacles-docker-compose.yml down

clean-containers: clean-docker-compose
	docker ps -a -q |xargs docker rm

clean-images:
	docker images --format json |jq -r ".ID" |xargs docker rmi

clean-network: clean-docker-compose
	docker network rm -f simulator

clean: clean-containers clean-images clean-network

offline: docker
	docker-compose -f docker-compose-offline.yml up

online: docker
	docker-compose -f docker-compose-online.yml up

offline-obstacles: docker
	docker-compose -f docker-compose-offline-obstacles.yml up

online-obstacles: docker
	docker-compose -f docker-compose-online-obstacles.yml up

offline-multi: docker
	docker-compose -f docker-compose-offline-multi.yml up

online-multi: docker
	docker-compose -f docker-compose-online-multi.yml up

docker-compose-stop:
	docker-compose stop

docker-compose-up: docker docker-compose-stop
	docker-compose up -d

network:
	docker network rm -f simulator
	docker network create --subnet=172.28.0.0/16 --gateway=172.28.5.254 simulator

shell-kos:
	docker run --name kos -w /home/user/kos --user user --net simulator --ip 172.28.0.1 -it --rm simulator /bin/bash -i

shell-kos-real:
	docker run --volume="`pwd`:/home/user/" --name kos -w /home/user/kos --user user --net simulator --ip 172.28.0.1 -it --rm simulator /bin/bash -i

shell-arducopter:
	docker run --name arducopter -w /home/user/ardupilot --user user --net simulator --ip 172.28.0.2 -it --rm simulator /bin/bash -i

shell-arducopter-real:
	docker run --volume="`pwd`:/home/user/" --name arducopter -w /home/user/ardupilot --user user --net simulator --ip 172.28.0.2 -it --rm simulator /bin/bash -i

shell-mavproxy:
	docker run --name mavproxy -w /home/user/mavproxy --user user --net simulator --ip 172.28.0.3 -it --rm simulator /bin/bash -i

shell-mavproxy-real:
	docker run --volume="`pwd`:/home/user/" --name mavproxy -w /home/user/mavproxy --user user --net simulator --ip 172.28.0.3 -it --rm simulator /bin/bash -i

shell-orvd:
	docker run --name orvd -w /home/user/orvd --net simulator -p 8080:8080 --ip 172.28.0.4 -it --rm orvd /bin/bash -i

shell-orvd-real:
	docker run --volume="`pwd`:/home/user/" --name orvd -w /home/user/orvd --net simulator -p 8080:8080 --ip 172.28.0.4 -it --rm orvd /bin/bash -i

e2e-offline: docker-image
	docker-compose -f tests/e2e-offline-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker-compose -f tests/e2e-offline-docker-compose.yml down

e2e-online: docker-image
	docker-compose -f tests/e2e-online-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker-compose -f tests/e2e-online-docker-compose.yml down

e2e-offline-obstacles: docker-image
	docker-compose -f tests/e2e-offline-obstacles-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker-compose -f tests/e2e-offline-obstacles-docker-compose.yml down

e2e-online-obstacles: docker-image
	docker-compose -f tests/e2e-online-obstacles-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker-compose -f tests/e2e-online-obstacles-docker-compose.yml down

e2e-tests: e2e-offline e2e-online
