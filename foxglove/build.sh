#!/bin/bash

docker run -it --rm --name build \
	-p 80:8080 \
	--user node \
	-v $(pwd):/data \
	--workdir /data \
	node:16 bash
