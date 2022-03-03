#!/bin/sh
gcc bc_interpreter_backend.c c_app.c bc_common.c -g3 -O0 -lm
gcc bc_interpreter_backend.c c_app.c bc_common.c -DDIS=1 -lm -g3 -O0 -odis
