.PHONY: docker-compose-stop

docker: docker-simulator

docker-simulator:
	docker build ./ -t simulator

docker-clean:
	docker ps -a -q |xargs docker rm

run: docker
	docker-compose up

docker-compose-stop:
	docker-compose stop

docker-compose-up: docker docker-compose-stop
	docker-compose up -d

