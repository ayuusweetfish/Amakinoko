#include "libui/ui.h"
#include "libserialport/libserialport.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // usleep

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

static uiLabel
  *lbl_readings_t, *lbl_readings_p, *lbl_readings_h, *lbl_readings_i;
static uiArea *area_readings_c_indicators;

static uint8_t readings_touch[4] = { 0 };

static void clear_readings_disp()
{
  uiLabelSetText(lbl_readings_t, "温度：— ˚C");
  uiLabelSetText(lbl_readings_p, "气压：— hPa");
  uiLabelSetText(lbl_readings_h, "湿度：— % RH");
  uiLabelSetText(lbl_readings_i, "光照：— lx");
  for (int i = 0; i < 4; i++) readings_touch[i] = 0;
  uiAreaQueueRedrawAll(area_readings_c_indicators);
}

#define TX_READINGS_LEN 14
static void parse_readings(const uint8_t buf[TX_READINGS_LEN])
{
  uint16_t T = ((uint16_t)buf[ 0] << 8) | buf[ 1];
  uint16_t t = ((uint16_t)buf[ 2] << 8) | buf[ 3];
  uint16_t p = ((uint16_t)buf[ 4] << 8) | buf[ 5];
  uint16_t h = ((uint16_t)buf[ 6] << 8) | buf[ 7];
  uint16_t i = ((uint16_t)buf[ 8] << 8) | buf[ 9];
  memcpy(readings_touch, buf + 10, 4);

  char s[64];
  snprintf(s, sizeof s, "温度：%.2f ˚C", (double)t / 100);
  uiLabelSetText(lbl_readings_t, s);
  snprintf(s, sizeof s, "气压：%.2f hPa", (double)((uint32_t)p + 65536) / 100);
  uiLabelSetText(lbl_readings_p, s);
  snprintf(s, sizeof s, "湿度：%.2f%% RH", (double)h / 100);
  uiLabelSetText(lbl_readings_h, s);
  snprintf(s, sizeof s, "光照：%d lx", (int)i);
  uiLabelSetText(lbl_readings_i, s);
  uiAreaQueueRedrawAll(area_readings_c_indicators);
}
static void parse_readings_arg(void *buf)
{
  parse_readings((const uint8_t *)buf);
  free(buf);
}

static void indicators_draw(uiAreaHandler *ah, uiArea *area, uiAreaDrawParams *p)
{
  for (int i = 0; i < 4; i++) {
    uiDrawPath *path = uiDrawNewPath(uiDrawFillModeWinding);
    const double r = 6, sw = 2, margin = 2;
    double x = sw + r + (r * 2 + margin) * i, y = p->AreaHeight / 2;
    uiDrawPathNewFigure(path, x, y);
    uiDrawPathArcTo(path, x, y, r, 0, uiPi * 2, false);
    uiDrawPathEnd(path);

    uiDrawFill(p->Context, path, &(uiDrawBrush){
      .Type = uiDrawBrushTypeSolid,
      .R = 0.1, .G = 0.1, .B = 0.1, .A = 0.1 + 0.9 / 255 * readings_touch[i],
    });
    uiDrawFreePath(path);
  }
}
static void indicators_ptr_event(uiAreaHandler *ah, uiArea *area, uiAreaMouseEvent *e)
{
}
static void indicators_hover(uiAreaHandler *ah, uiArea *area, int left)
{
}
static void indicators_drag_broken(uiAreaHandler *ah, uiArea *area)
{
}
static int indicators_key(uiAreaHandler *ah, uiArea *area, uiAreaKeyEvent *e)
{
  return false;
}

static struct sp_port *port = NULL;
#define ensure_or_clear(_ok, _call_result) do { \
  enum sp_return r = (_call_result); \
  ensure_or_act(r == (_ok), { close_port(); port = NULL; return false; }, \
    "Ran into issue handling serial port: %s returned %d", #_call_result, (int)r); \
} while (0)

#define ensure_or_clear_early_with_msg(_call_result, ...) do { \
  enum sp_return r = (_call_result); \
  ensure_or_act(r == SP_OK, { port = NULL; return false; }, __VA_ARGS__); \
} while (0)

static inline int tx(const uint8_t *buf, uint8_t len);

static pthread_t serial_loop_thr;
static void *serial_loop_fn(void *_unused);

static bool close_port()
{
  if (port != NULL) {
    tx((uint8_t []){ 0xBB }, 1);  // Ignore result, close anyway
    clear_readings_disp();
    struct sp_port *saved_port = port;
    port = NULL;
    if (pthread_kill(serial_loop_thr, 0) == 0)  // Is valid thread?
      ensure_or_clear(0, pthread_join(serial_loop_thr, NULL));
    ensure_or_clear(SP_OK, sp_close(saved_port));
    fprintf(stderr, "Serial port closed\n");
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
  ensure_or_clear(SP_OK, sp_set_baudrate(port, baud_rate));
  ensure_or_clear(SP_OK, sp_set_bits(port, 8));
  ensure_or_clear(SP_OK, sp_set_parity(port, SP_PARITY_NONE));
  ensure_or_clear(SP_OK, sp_set_stopbits(port, 1));
  ensure_or_clear(SP_OK, sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));
  ensure_or_clear(SP_OK, sp_set_dtr(port, SP_DTR_ON));

  ensure_or_clear(SP_OK, sp_flush(port, SP_BUF_BOTH));

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

static inline int rx_timeout(uint8_t *buf, unsigned timeout)
{
  int n_rx;
  uint8_t len;

  n_rx = sp_blocking_read(port, &len, 1, timeout);
  ensure_or_ret(-1, n_rx == 1, "Did not receive packet header");

  fprintf(stderr, "rx [%02x]", len);
  if (len > 0) {
    n_rx = sp_blocking_read(port, buf, len, 50);
    ensure_or_ret(-1, n_rx == len, "Did not receive packet payload");
    for (int i = 0; i < n_rx; i++) fprintf(stderr, " %02x", (int)buf[i]);
    fprintf(stderr, "\n");
  }

  return len;
}

static uint8_t rx_buf[256];
static inline int rx() { return rx_timeout(rx_buf, 50); }

static void *serial_loop_fn(void *_unused)
{
  uint8_t buf[256];
  while (port != NULL) {
    int len = rx_timeout(buf, 50);
    if (len != -1) {
      uint8_t *buf_dup = malloc(len);
      if (buf_dup != NULL) {
        memcpy(buf_dup, buf, len);
        uiQueueMain(parse_readings_arg, buf_dup);
      } else {
        fprintf(stderr, "Out of memory!\n");
      }
    } else {
      usleep(50000);
    }
  }
  return NULL;
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

  ensure_or_reject(rx_len >= 11 + TX_READINGS_LEN, "Invalid readings");
  parse_readings(rx_buf + 11);

  ensure_or_reject(
    pthread_create(&serial_loop_thr, NULL, &serial_loop_fn, NULL) == 0,
    "Cannot create thread, check system resource usage");
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

  uiBox *box_readings = uiNewHorizontalBox();
  uiBoxSetPadded(box_readings, 1);
  uiBoxAppend(box_main, uiControl(box_readings), 0);
  {
    uiBox *parent = box_readings;

    lbl_readings_t = uiNewLabel("");
    uiBoxAppend(parent, uiControl(lbl_readings_t), true);
    uiBoxAppend(parent, uiControl(uiNewLabel("|")), false);
    lbl_readings_p = uiNewLabel("");
    uiBoxAppend(parent, uiControl(lbl_readings_p), true);
    uiBoxAppend(parent, uiControl(uiNewLabel("|")), false);
    lbl_readings_h = uiNewLabel("");
    uiBoxAppend(parent, uiControl(lbl_readings_h), true);
    uiBoxAppend(parent, uiControl(uiNewLabel("|")), false);
    lbl_readings_i = uiNewLabel("");
    uiBoxAppend(parent, uiControl(lbl_readings_i), true);
    uiBoxAppend(parent, uiControl(uiNewLabel("|")), false);

    uiBox *box_readings_c_container = uiNewHorizontalBox();
    uiBoxAppend(parent, uiControl(box_readings_c_container), true);
    {
      uiBox *parent = box_readings_c_container;
      uiLabel *lbl_readings_c_caption = uiNewLabel("触摸：");
      uiBoxAppend(parent, uiControl(lbl_readings_c_caption), false);

      static uiAreaHandler ah = {
        .Draw = indicators_draw,
        .MouseEvent = indicators_ptr_event,
        .MouseCrossed = indicators_hover,
        .DragBroken = indicators_drag_broken,
        .KeyEvent = indicators_key,
      };
      area_readings_c_indicators = uiNewArea(&ah);
      uiBoxAppend(parent, uiControl(area_readings_c_indicators), true);
    }

    clear_readings_disp();
  }

  // Run main loop
  uiWindowOnClosing(w, window_on_closing, NULL);
  uiOnShouldQuit(on_should_quit, NULL);
  uiControlShow(uiControl(w));
  uiMain();

  return 0;
}
