#!/bin/sh
ldmd2 -O0 -g bc_lightning_backend.d bc_interpreter_backend.d bc_common.d fpconv_ctfe.d bc_abi.d bc_limits.d \
lightning-d/lightning.d \
lightning-d/jit_aarch64.d \
lightning-d/jit_x86_64.d \
-L-llightning -L-L/usr/local/lib examples.d -L--rpath-link='$ORIGIN' $@
