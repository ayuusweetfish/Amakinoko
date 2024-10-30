#include "libui/ui.h"
#include <stdio.h>

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

  // Run main loop
  uiWindowOnClosing(w, windowOnClosing, NULL);
  uiOnShouldQuit(onShouldQuit, NULL);
  uiControlShow(uiControl(w));
  uiMain();

  return 0;
}
