#!/bin/sh
cc bc_interpreter_backend.c c_app.c bc_common.c -g3 -Os -lm $@
cc bc_interpreter_backend.c c_app.c bc_common.c -DDIS=1 -lm -Os -odis $@
