##################################################################
# ENV VARS
##################################################################
export GID := 1000 # $(shell id -g)
export UID := 1000 # $(shell id -u)
export BRANCH := $(shell git rev-parse --abbrev-ref HEAD)
export COMMIT_ID := $(shell git rev-parse --short=9 HEAD)
export BUILD_TIME := $(shell date '+%Y%m%d')
export COMMIT_BODY := $(shell git rev-list --format=%B --max-count=1 HEAD | tail +2)
export USERNAME := $(ACCOUNT_NAME)
export PASSWORD := $(ACCOUNT_PASSWORD)
ifeq "$(BRANCH)" "dev"
	DOCKER_NAMESPACE := dev
else ifeq "$(BRANCH)" "master"
	DOCKER_NAMESPACE := prod
else
	DOCKER_NAMESPACE := stage
endif
export DOCKER_NAMESPACE := $(DOCKER_NAMESPACE)

# check username and password
ifeq ($(USERNAME),)
$(warning USERNAME is not set)
endif
ifeq ($(PASSWORD),)
$(warning PASSWORD is not set)
endif

pre-build:
	@echo "DOCKER_NAMESPACE: $(DOCKER_NAMESPACE)"
	@echo "GID: $(GID)"
	@echo "UID: $(UID)"
	@echo "BRANCH: $(BRANCH)"
	@echo "COMMIT_ID: $(COMMIT_ID)"
	@echo "BUILD_TIME: $(BUILD_TIME)"
	@echo "USERNAME: $(USERNAME)"
	@echo "PASSWORD: $(PASSWORD)"

format-cmake:
	find components/ -path ./3rdparty -prune -o -name CMake*.txt | xargs -i cmake-format -i {}

format-cpp:
	find components/ -type f \( -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp" -o -name "*.cc" -o -name "*.cu" -o -name "*.cuh" \) | xargs -i clang-format -i {}

format:format-cmake format-cpp
	@echo 'done'

##################################################################
# COMMANDS
##################################################################
build-fastlio2:pre-build
	COMMIT_ID=latest docker compose -f compose.yml build fastlio2
# docker compose -f compose.yml build fastlio2

start-fastlio2:pre-build
	docker compose -f compose.yml up -d fastlio2

stop-fastlio2:pre-build
	docker compose -f compose.yml stop fastlio2



