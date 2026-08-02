SHELL := /bin/bash

ROS_DISTRO ?= jazzy
export ROS_DISTRO

.DEFAULT_GOAL := help

.PHONY: help setup build test sim experiment smoke mujoco-smoke

help:
	@echo "Robot Dog developer commands"
	@echo "  make setup   Install package and pinned Python dependencies"
	@echo "  make build   Build the ROS 2 workspace with symlink installs"
	@echo "  make test    Run all package tests and print the result summary"
	@echo "  make sim     Start the PyBullet simulation and Foxglove bridge"
	@echo "  make experiment  Run the example controller and sensor teaching slice"
	@echo "  make smoke   Verify commands and simulated sensor topics end to end"
	@echo "  make mujoco-smoke  Verify the deterministic headless MuJoCo core"
	@echo
	@echo "Override ROS_DISTRO when needed, for example: ROS_DISTRO=humble make build"

setup:
	./scripts/bootstrap.sh

build:
	./scripts/build.sh

test:
	./scripts/test.sh

sim:
	./scripts/sim.sh $(SIM_ARGS)

experiment:
	./scripts/experiment.sh $(SIM_ARGS)

smoke:
	./scripts/smoke_sim.sh

mujoco-smoke:
	./scripts/smoke_mujoco.sh
