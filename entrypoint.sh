#!/usr/bin/bash

PROJECT="${1}"
if [ -z "${PROJECT}" ]; then
	PROJECT="template"
fi

SERIAL_PORT="${2}"
if [ -z "${SERIAL_PORT}" ]; then
	SERIAL_PORT="/dev/ttyUSB1"
fi

scons board="openmote-b" project="${PROJECT}" compiler=gcc bootload="${SERIAL_PORT}" verbose=1 -Q
