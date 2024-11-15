openocd_cmds=
openocd_init=
if [[ "$1" == *"f"* ]]; then  # flash
  openocd_cmds="$openocd_cmds; program {.pio/build/dev/firmware.elf} verify reset"
fi
if [[ "$1" == *"F"* ]]; then  # flash release
  openocd_cmds="$openocd_cmds; program {.pio/build/rel/firmware.elf} verify reset"
fi
if [[ "$1" == *"s"* ]]; then  # serve
  openocd_cmds="$openocd_cmds; init"
  openocd_init=1
fi
if [[ "$1" == *"r"* ]]; then  # serve, reset
  openocd_cmds="$openocd_cmds; init; reset"
  openocd_init=1
fi
if [[ "$1" == *"t"* ]]; then  # serve, stop
  openocd_cmds="$openocd_cmds; init; halt"
  openocd_init=1
fi
if [ "$openocd_cmds" ]; then
  if ! [ "$openocd_init" ]; then
    openocd_cmds="$openocd_cmds; shutdown"
  fi
  ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/stm32g0x.cfg -c "adapter speed 32000 $openocd_cmds"
  exit
fi
if [ "$1" == "d" ]; then  # disassembly
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
