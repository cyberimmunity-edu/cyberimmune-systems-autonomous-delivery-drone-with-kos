.PHONY: docker-compose-stop

docker: docker-orvd-server

docker-orvd-server:
	cd orvd_server && docker build ./ -t orvd_server

docker-orvd-client:
	cd orvd_client && docker build ./ -t orvd_client

docker-clean:
	docker ps -a -q |xargs docker rm

run: docker
	docker-compose up

docker-compose-stop:
	docker-compose stop

docker-compose-up: docker docker-compose-stop
	docker-compose up -d

