#include "libui/ui.h"
#include "libserialport/libserialport.h"
#include <stdio.h>
#include <stdlib.h>

static uiWindow *w;

static int onShouldQuit(void *_unused)
{
  return 1;
}
static int windowOnClosing(uiWindow *w, void *_unused)
{
  uiQuit();
  return 1;
}

static void list_serial_ports()
{
  struct sp_port **port_list;
  if (sp_list_ports(&port_list) < 0) {
    fprintf(stderr, "Cannot retrieve list of ports\n");
    exit(0);
  }
  int i;
  for (i = 0; port_list[i] != NULL; i++)
    fprintf(stderr, "%d: %s\n", i, sp_get_port_name(port_list[i]));
  fprintf(stderr, "%d serial port(s) total\n", i);
  sp_free_port_list(port_list);
}

int main()
{
  uiInitOptions o = { 0 };
  const char *err = uiInit(&o);
  if (err != NULL) {
    fprintf(stderr, "Error initializing libui: %s\n", err);
    uiFreeInitError(err);
    return 1;
  }

#if __APPLE__
  uiMenu *menuDefault = uiNewMenu("");
  uiMenuAppendQuitItem(menuDefault);
#endif

  // Create window
  w = uiNewWindow("蘑菇！", 480, 600, 0);
  uiWindowSetMargined(w, 1);

  list_serial_ports();

  // Run main loop
  uiWindowOnClosing(w, windowOnClosing, NULL);
  uiOnShouldQuit(onShouldQuit, NULL);
  uiControlShow(uiControl(w));
  uiMain();

  return 0;
}
