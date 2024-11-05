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
