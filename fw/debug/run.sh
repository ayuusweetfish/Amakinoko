if [ "$1" == "f" ]; then  # flash
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/stm32g0x.cfg -c 'program {.pio/build/dev/firmware.elf} verify reset; shutdown'
  exit
elif [ "$1" == "s" ]; then  # serve
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/stm32g0x.cfg -c 'adapter speed 32000; init'
  exit
elif [ "$1" == "r" ]; then  # serve, reset
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/stm32g0x.cfg -c 'adapter speed 32000; init; reset'
  exit
elif [ "$1" == "t" ]; then  # serve, stop
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/stm32g0x.cfg -c 'adapter speed 32000; init; halt'
  exit
elif [ "$1" == "d" ]; then  # disassembly
  ~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objdump -S -d .pio/build/dev/firmware.elf
  exit
elif [ "$1" == "z" ]; then  # size
  ~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm  --print-size --size-sort --radix=d .pio/build/dev/firmware.elf
  exit
fi

cat >debug/gdbinit <<EOF
define hook-quit
  set confirm off
end
target extended-remote localhost:3333

set pagination off

b swv_trap_line
commands
  silent
  printf "%s\n", swv_buf
  c
end
EOF

~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb .pio/build/dev/firmware.elf -x debug/gdbinit
rm debug/gdbinit
