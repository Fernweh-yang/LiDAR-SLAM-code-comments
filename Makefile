##################################################################
# ENV VARS
##################################################################
export GID := $(shell id -g)
export UID := $(shell id -u)
export BRANCH := $(shell git rev-parse --abbrev-ref HEAD)
export COMMIT_ID := $(shell git rev-parse --short=9 HEAD)
export BUILD_TIME := $(shell date '+%Y%m%d')
export COMMIT_BODY := $(shell git rev-list --format=%B --max-count=1 HEAD | tail +2)
ifeq "$(BRANCH)" "dev"
	DOCKER_NAMESPACE := dev
else ifeq "$(BRANCH)" "main"
	DOCKER_NAMESPACE := product
else
	DOCKER_NAMESPACE := stage
endif
export DOCKER_NAMESPACE := $(DOCKER_NAMESPACE)

pre-build:
	@echo "DOCKER_NAMESPACE: $(DOCKER_NAMESPACE)"
	@echo "GID: $(GID)"
	@echo "UID: $(UID)"
	@echo "BRANCH: $(BRANCH)"
	@echo "COMMIT_ID: $(COMMIT_ID)"
	@echo "BUILD_TIME: $(BUILD_TIME)"

##################################################################
# CODE FORMAT
##################################################################
format-cmake:
	find components/ -path ./3rdparty -prune -o -name CMake*.txt | xargs -i cmake-format -i {}

format-cpp:
# 	find components/ \( -path components/swarm -prune \) -o \
	find . \
		 -type f \( -name "*.cpp" -o \
					-name "*.c" -o \
					-name "*.h" -o \
					-name "*.hpp" -o \
					-name "*.cc" -o \
					-name "*.cu" -o \
					-name "*.cuh" \) -print | xargs -i clang-format -i {}

format:format-cmake format-cpp
	@echo 'done'


##################################################################
# COMMANDS
##################################################################
build-env-ubuntu:pre-build
	docker compose -f compose.yaml build slam-learning-env-ubuntu

start-env-ubuntu:pre-build
	docker compose -f compose.yaml up -d slam-learning-env-ubuntu


build-env-mac:pre-build
	docker compose -f compose.yaml build slam-learning-env-mac

start-env-mac:pre-build
	docker compose -f compose.yaml up -d slam-learning-env-mac