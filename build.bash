#!/bin/bash
docker build --build-arg user_id=$(id -u) --build-arg USER=$(whoami) --build-arg NAME=phobos --rm -t dtc-jackal-phobos:masking .
