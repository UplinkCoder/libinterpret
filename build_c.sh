#!/bin/sh
if [ -z $CC ]
then
    CC=cc
fi
$CC bc_interpreter_backend.c c_app.c bc_common.c -lm $@
$CC -o test_printer test_printer.c printer_backend.c  bc_common.c utils/int_to_str.c -lm  $@
#$CC bc_interpreter_backend.c c_app.c bc_common.c -DDIS=1 -lm -Os -odis $@
