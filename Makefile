.PHONY: docker-compose-stop

docker: docker-simulator docker-orvd

docker-simulator:
	docker build ./ -t simulator

docker-orvd:
	docker build -f orvd.Dockerfile -t orvd ./

docker-clean:
	docker ps -a -q |xargs docker rm

offline: docker
	docker-compose -f docker-compose-offline.yml up

online: docker
	docker-compose -f docker-compose-online.yml up

docker-compose-stop:
	docker-compose stop

docker-compose-up: docker docker-compose-stop
	docker-compose up -d

