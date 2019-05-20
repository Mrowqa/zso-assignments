#!/bin/bash

if [[ $EUID -ne 0 ]]; then
	echo Run as a root
	exit 1
fi

rmmod harddoom2
