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

static void btn_serial_port_refresh_clicked(uiButton *btn, void *_cbox_serial_port)
{
  uiCombobox *cbox_serial_port = (uiCombobox *)_cbox_serial_port;
  struct sp_port **port_list;
  if (sp_list_ports(&port_list) < 0) {
    fprintf(stderr, "Cannot retrieve list of ports\n");
    exit(0);
  }
  for (int i = 0; port_list[i] != NULL; i++) {
    uiComboboxAppend(cbox_serial_port, sp_get_port_name(port_list[i]));
  }
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
  {
    uiMenu *menu_default = uiNewMenu("");
    uiMenuAppendQuitItem(menu_default);
  }
#endif

  // Create window
  w = uiNewWindow("蘑菇！", 480, 600, 0);
  uiWindowSetMargined(w, 1);

  // Layout
  uiBox *box_main = uiNewVerticalBox();
  uiBoxSetPadded(box_main, 1);
  uiWindowSetChild(w, uiControl(box_main));

  uiBox *box_serial_port = uiNewVerticalBox();
  uiBoxSetPadded(box_serial_port, 1);
  uiBoxAppend(box_main, uiControl(box_serial_port), 0);
  {
    uiBox *box_serial_port_r1 = uiNewHorizontalBox();
    uiBoxSetPadded(box_serial_port_r1, 1);
    uiBoxAppend(box_serial_port, uiControl(box_serial_port_r1), 0);
    {
      uiLabel *lbl = uiNewLabel("串口");
      uiBoxAppend(box_serial_port_r1, uiControl(lbl), 0);

      uiCombobox *cbox_serial_port = uiNewCombobox();
      uiBoxAppend(box_serial_port_r1, uiControl(cbox_serial_port), 1);

      uiButton *btn_serial_port_refresh = uiNewButton("刷新");
      uiBoxAppend(box_serial_port_r1, uiControl(btn_serial_port_refresh), 0);
      uiButtonOnClicked(btn_serial_port_refresh, btn_serial_port_refresh_clicked, cbox_serial_port);
    }
  }

  list_serial_ports();

  // Run main loop
  uiWindowOnClosing(w, windowOnClosing, NULL);
  uiOnShouldQuit(onShouldQuit, NULL);
  uiControlShow(uiControl(w));
  uiMain();

  return 0;
}
