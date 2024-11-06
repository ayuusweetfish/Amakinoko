#include "libui/ui.h"
#include "libserialport/libserialport.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uiWindow *w;

static int on_should_quit(void *_unused)
{
  return 1;
}

static char global_err_msg[256];
#define ensure_or_act(_cond, _act, ...) do { \
  if (!(_cond)) { \
    snprintf(global_err_msg, sizeof global_err_msg, __VA_ARGS__); \
    _act; \
  } \
} while (0)
#define ensure_or_abort(_cond, ...) ensure_or_act(_cond, error_and_abort(), __VA_ARGS__)
#define ensure_or_ret(_val, _cond, ...) ensure_or_act(_cond, return _val, __VA_ARGS__)

static void error_and_abort()
{
  uiMsgBox(w, "", global_err_msg);
  exit(1);
}
static void error_and_continue()
{
  uiMsgBox(w, "", global_err_msg);
}

static struct sp_port *port = NULL;
#define ensure_or_clear(_call_result) do { \
  enum sp_return r = (_call_result); \
  ensure_or_act(r == SP_OK, { close_port(); port = NULL; return false; }, \
    "Ran into issue handling serial port: %s returned %d", #_call_result, (int)r); \
} while (0)

#define ensure_or_clear_early_with_msg(_call_result, ...) do { \
  enum sp_return r = (_call_result); \
  ensure_or_act(r == SP_OK, { port = NULL; return false; }, __VA_ARGS__); \
} while (0)

static bool close_port()
{
  if (port != NULL) {
    struct sp_port *saved_port = port;
    port = NULL;
    fprintf(stderr, "Serial port closed\n");
    ensure_or_clear(sp_close(saved_port));
  }
  return true;
}
// Opens the global port
// A return value of false guarantees that global `port` is properly closed and set to NULL
static bool open_port(const char *name, int baud_rate)
{
  if (!close_port()) return false;

  ensure_or_clear_early_with_msg(sp_get_port_by_name(name, &port), "Port no longer accessible");
  ensure_or_clear_early_with_msg(sp_open(port, SP_MODE_READ_WRITE), "Cannot open port; check whether used by other programs");
  ensure_or_clear(sp_set_baudrate(port, baud_rate));
  ensure_or_clear(sp_set_bits(port, 8));
  ensure_or_clear(sp_set_parity(port, SP_PARITY_NONE));
  ensure_or_clear(sp_set_stopbits(port, 1));
  ensure_or_clear(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));
  ensure_or_clear(sp_set_dtr(port, SP_DTR_ON));

  ensure_or_clear(sp_flush(port, SP_BUF_BOTH));

  return true;
}

static inline int tx(const uint8_t *buf, uint8_t len)
{
  int n_tx;

  n_tx = sp_blocking_write(port, &len, 1, 100);
  ensure_or_ret(-1, n_tx == 1, "Cannot write to port");

  n_tx = sp_blocking_write(port, buf, len, 100);
  ensure_or_ret(-1, n_tx == len, "Cannot write to port");

  fprintf(stderr, "tx [%02x]", (unsigned)len);
  for (int i = 0; i < len; i++) fprintf(stderr, " %02x", (unsigned)buf[i]);
  fprintf(stderr, "\n");

  return len;
}

static uint8_t rx_buf[256];
static inline int rx()
{
  int n_rx;
  uint8_t len;

  n_rx = sp_blocking_read(port, &len, 1, 100);
  ensure_or_ret(-1, n_rx == 1, "Did not receive packet header");

  fprintf(stderr, "rx [%02x]", len);
  if (len > 0) {
    n_rx = sp_blocking_read(port, rx_buf, len, 100);
    ensure_or_ret(-1, n_rx == len, "Did not receive packet payload");
    for (int i = 0; i < n_rx; i++) fprintf(stderr, " %02x", (int)rx_buf[i]);
    fprintf(stderr, "\n");
  }

  return len;
}

#undef ensure_or_clear

static int port_names_n = 0;
static char **port_names = NULL;
static void btn_serial_port_refresh_clicked(uiButton *btn, void *_cbox_serial_port)
{
  uiCombobox *cbox_serial_port = (uiCombobox *)_cbox_serial_port;

  int last_sel = uiComboboxSelected(cbox_serial_port);
  char *last_sel_name = NULL;
  if (last_sel >= 0 && last_sel < port_names_n)
    last_sel_name = port_names[last_sel];
  uiComboboxClear(cbox_serial_port);

  struct sp_port **port_list;
  ensure_or_abort(sp_list_ports(&port_list) >= 0, "Cannot retrieve list of ports");
  int n;
  for (n = 0; port_list[n] != NULL; n++) { }
  char **new_names = malloc(sizeof(char *) * n);
  ensure_or_abort(new_names != NULL, "Out of memory");

  int new_sel = -1;
  for (int i = 0; port_list[i] != NULL; i++) {
    new_names[i] = strdup(sp_get_port_name(port_list[i]));
    ensure_or_abort(new_names[i] != NULL, "Out of memory");
    uiComboboxAppend(cbox_serial_port, new_names[i]);
    if (last_sel_name != NULL && strcmp(last_sel_name, new_names[i]) == 0)
      new_sel = i;
  }

  sp_free_port_list(port_list);

  if (new_sel != -1)
    uiComboboxSetSelected(cbox_serial_port, new_sel);
  else if (last_sel_name != NULL)
    close_port();

  if (port_names != NULL) {
    for (int i = 0; i < port_names_n; i++) free(port_names[i]);
    free(port_names);
  }
  port_names_n = n;
  port_names = new_names;
}

static void cbox_serial_port_changed(uiCombobox *c, void *_unused)
{
#define msgbox_clear_sel_and_continue() do { \
  close_port(); \
  error_and_continue(); \
  uiComboboxSetSelected(c, -1); \
  return; \
} while (0)

#define ensure_or_reject(_cond, ...) \
  ensure_or_act(_cond, msgbox_clear_sel_and_continue();, __VA_ARGS__)

  int sel = uiComboboxSelected(c);
  if (sel < 0 || sel >= port_names_n) return;
  if (!open_port(port_names[sel], 921600)) msgbox_clear_sel_and_continue();

  // Try to request first packet of data
  if (tx((uint8_t []){0xAA}, 1) < 0) msgbox_clear_sel_and_continue();
  int rx_len = rx();
  if (rx_len < 0) msgbox_clear_sel_and_continue();

  ensure_or_reject(rx_len >= 11 &&
    memcmp(rx_buf, "\x55" "Amakinoko", 10) == 0, "Invalid device signature");
  uint8_t revision = rx_buf[10];
  ensure_or_reject(revision == 0x20, "Invalid device revision %u", (unsigned)revision);
}

static int window_on_closing(uiWindow *w, void *_unused)
{
  close_port();
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
      uiComboboxOnSelected(cbox_serial_port, cbox_serial_port_changed, NULL);

      uiButton *btn_serial_port_refresh = uiNewButton("刷新");
      uiBoxAppend(box_serial_port_r1, uiControl(btn_serial_port_refresh), 0);
      uiButtonOnClicked(btn_serial_port_refresh, btn_serial_port_refresh_clicked, cbox_serial_port);
      btn_serial_port_refresh_clicked(btn_serial_port_refresh, cbox_serial_port);
    }
  }

  // Run main loop
  uiWindowOnClosing(w, window_on_closing, NULL);
  uiOnShouldQuit(on_should_quit, NULL);
  uiControlShow(uiControl(w));
  uiMain();

  return 0;
}
