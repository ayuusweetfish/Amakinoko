CC ?= clang
OPT_FLAGS ?= -O2
DEBUG_FLAGS ?=
ifneq ($(strip $(DEBUG)),)
  OPT_FLAGS =
  DEBUG_FLAGS += -g
endif
ifneq ($(strip $(ASAN)),)
  DEBUG_FLAGS += -fsanitize=address
endif
ifeq ($(strip $(WIN)), 1)
  CC = x86_64-w64-mingw32-gcc
  STRIP = x86_64-w64-mingw32-strip
  MAIN_EXE = main.exe
else
  STRIP = strip
  MAIN_EXE = main
endif

all: $(MAIN_EXE)

main: main.c mumu_as.o
	$(CC) -o $@ $^ -Wall -std=c99 libui/build/meson-out/libui.a libserialport/.libs/libserialport.a zlib/libz.a -Izlib -framework Cocoa -framework IOKit $(OPT_FLAGS) $(DEBUG_FLAGS)
	$(STRIP) $@

mumu_as.o: mumu_as.c
	$(CC) -c -o $@ $^ -Wall -std=c99 $(OPT_FLAGS) $(DEBUG_FLAGS)

main.exe: main.c mumu_as_win.o
	$(CC) -o $@ $^ -Wall -std=c99 libui/build_win/meson-out/libui.a libui/build_win/meson-out/examples_resources.rc_resources.o libserialport_win/.libs/libserialport.a zlib_win/libz.a -Izlib -static -lws2_32 -lsetupapi -lstdc++ -lgdi32 -lcomctl32 -lole32 -ld2d1 -ldwrite -Wl,--subsystem,windows
	$(STRIP) $@

mumu_as_win.o: mumu_as.c
	$(CC) -c -o $@ $^ -Wall -std=c99 $(OPT_FLAGS) $(DEBUG_FLAGS)

clean:
	rm -f main mumu_as.o main.exe mumu_as_win.o

.PHONY: main_exe clean
