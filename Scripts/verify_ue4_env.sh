#!/bin/sh

if [ -z "${UE4}" ]; then
	printf "Please set UE4 to path of UnrealEngine's parent folder\n"
	exit 1
fi
exit 0
