CC ?= clang
DEBUG_FLAGS ?=
ifneq ($(strip $(DEBUG)),)
  DEBUG_FLAGS += -g
endif
ifneq ($(strip $(ASAN)),)
  DEBUG_FLAGS += -fsanitize=address
endif

main: main.c
	$(CC) -o $@ $^ -Wall -std=c99 libui/build/meson-out/libui.a libserialport/.libs/libserialport.a -framework Cocoa -framework IOKit -O2 $(DEBUG_FLAGS)

main.exe: main.c
	x86_64-w64-mingw32-gcc -o $@ $^ -Wall -std=c99 libui/build_win/meson-out/libui.a libserialport-win/.libs/libserialport.a -static -lws2_32 -lsetupapi -lstdc++ -lgdi32 -lcomctl32 -lole32 -ld2d1 -ldwrite
