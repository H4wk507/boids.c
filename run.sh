#!/bin/bash

set -e

gcc -std=c99 -Werror -Wall boids.c -o boids -I/usr/local/include -lraylib -lm && ./boids
