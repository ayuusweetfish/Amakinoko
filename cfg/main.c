#include "libui/ui.h"
#include "libserialport/libserialport.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // usleep

#include "crc32.h"

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

static inline void clear_status_bar();
static inline void status_bar(const char *s);

static void error_and_abort()
{
  uiMsgBox(w, "", global_err_msg);
  exit(1);
}
static void error_and_continue()
{
  uiMsgBox(w, "", global_err_msg);
  clear_status_bar();
  if (0) {
    char *s;
    asprintf(&s, "× %s", global_err_msg);
    status_bar(s);
    free(s);
  }
}

static uiLabel
  *lbl_readings_t, *lbl_readings_p, *lbl_readings_h, *lbl_readings_i;
static uiArea *area_readings_c_indicators;

static uint8_t readings_touch[4] = { 0 };

static uiBox *box_program;
static uiMultilineEntry *text_source;
static uiButton *btn_check, *btn_upload;
static uiLabel *lbl_status_bar;

static inline void clear_status_bar()
{
  uiLabelSetText(lbl_status_bar, "");
}
static inline void status_bar(const char *s)
{
  uiLabelSetText(lbl_status_bar, s);
}

static void clear_readings_disp()
{
  uiLabelSetText(lbl_readings_t, "温度：— ˚C");
  uiLabelSetText(lbl_readings_p, "气压：— hPa");
  uiLabelSetText(lbl_readings_h, "湿度：— % RH");
  uiLabelSetText(lbl_readings_i, "光照：— lx");
  for (int i = 0; i < 4; i++) readings_touch[i] = 0;
  uiAreaQueueRedrawAll(area_readings_c_indicators);
  uiControlDisable(uiControl(btn_upload));
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

// buf[0] is length; buf[1..=length] is payload
static void parse_continuous_rx(void *_buf)
{
  uint8_t *buf = _buf;
  uint8_t len = buf[0];
  buf++;
  // 0x56, <readings>
  if (len == TX_READINGS_LEN + 1 && buf[0] == 0x56) {
    parse_readings((const uint8_t *)(buf + 1));
  }
  free(_buf);
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

static inline int tx(const uint8_t *buf, uint16_t len);

static pthread_t serial_loop_thr;
static void *serial_loop_fn(void *_unused);

static void start_list_refresh_timer_if_unconnected();

static bool close_port()
{
  if (port != NULL) {
    tx((uint8_t []){ 0xBB }, 1);  // Ignore result, close anyway
    clear_readings_disp();
    clear_status_bar();
    struct sp_port *saved_port = port;
    port = NULL;
    start_list_refresh_timer_if_unconnected();
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

static inline int tx(const uint8_t *buf, uint16_t len)
{
  if (len > 256) return -1;
  int n_tx;

  n_tx = sp_blocking_write(port, &(int){ len & 0xff }, 1, 10);
  ensure_or_ret(-1, n_tx == 1, "Cannot write to port");

  n_tx = sp_blocking_write(port, buf, len, 10);
  ensure_or_ret(-1, n_tx == len, "Cannot write to port");

  fprintf(stderr, "tx [%02x]", (unsigned)len);
  for (int i = 0; i < len && i < 32; i++) fprintf(stderr, " %02x", (unsigned)buf[i]);
  if (n_tx > 32) fprintf(stderr, "...");
  fprintf(stderr, "\n");

  return len;
}

static inline int rx_timeout(uint8_t *buf, unsigned initial_timeout)
{
  int n_rx;
  uint32_t len = 0;
  uint8_t len_byte;

  n_rx = sp_blocking_read(port, &len_byte, 1, initial_timeout);
  ensure_or_ret(-1, n_rx == 1, "Did not receive packet header");
  if (len_byte < 128) {
    len = len_byte;
  } else {
    len = ((len_byte ^ 128) << 8);
    n_rx = sp_blocking_read(port, &len_byte, 1, initial_timeout);
    ensure_or_ret(-1, n_rx == 1, "Did not receive packet header");
    len |= len_byte;
  }

  fprintf(stderr, "rx [%02x]", len);
  if (len > 0) {
    n_rx = sp_blocking_read(port, buf, len, 50);
    ensure_or_ret(-1, n_rx == len, "Did not receive packet payload");
    for (int i = 0; i < n_rx && i < 32; i++) fprintf(stderr, " %02x", (int)buf[i]);
    if (n_rx > 32) fprintf(stderr, "...");
  }
  fprintf(stderr, "\n");

  return len;
}

static uint8_t rx_buf[256];
static inline int rx() { return rx_timeout(rx_buf, 10); }

static inline int thread_rx()
{
  int len;
  while (true) {
    len = rx_timeout(rx_buf, 10);
    // XXX: Reduce duplication?
    if (len == TX_READINGS_LEN + 1 && rx_buf[0] == 0x56) {
      parse_readings((const uint8_t *)(rx_buf + 1));
    } else {
      return len;
    }
  }
}

static _Atomic bool serial_loop_pause_req, serial_loop_pause_state;

static void *serial_loop_fn(void *_unused)
{
  uint8_t buf[256];
  while (port != NULL) {
    // XXX: This might be better implemented with poll()/semaphores/..., but anyway
    if (serial_loop_pause_state != serial_loop_pause_req) {
      serial_loop_pause_state = serial_loop_pause_req;
    }
    if (serial_loop_pause_state) {
      usleep(20000);
      continue;
    }
    int len = rx_timeout(buf, 50);
    if (len != -1) {
      uint8_t *buf_dup = malloc(len + 1);
      if (buf_dup != NULL) {
        buf_dup[0] = (uint8_t)len;
        memcpy(buf_dup + 1, buf, len);
        uiQueueMain(parse_continuous_rx, buf_dup);
      } else {
        fprintf(stderr, "Out of memory!\n");
      }
    } else {
      usleep(20000);
    }
  }
  return NULL;
}

static inline void thread_rx_pause(bool flag)
{
  serial_loop_pause_req = flag;
  while (serial_loop_pause_state != flag) usleep(10000);
}

#undef ensure_or_clear

static uiCombobox *cbox_serial_port;
static uiButton *btn_serial_port_conn;
static int port_names_n = 0;
static char **port_names = NULL;
static bool timer_running = false;

static void update_cbox_disp()
{
  if (port_names_n > 0) {
    uiControlEnable(uiControl(btn_serial_port_conn));
    if (port == NULL) {
      uiControlEnable(uiControl(cbox_serial_port));
      if (uiComboboxSelected(cbox_serial_port) < 0)
        uiComboboxSetSelected(cbox_serial_port, 0);
      uiButtonSetText(btn_serial_port_conn, "连接");
    } else {
      uiControlDisable(uiControl(cbox_serial_port));
      uiButtonSetText(btn_serial_port_conn, "断开");
    }
  } else {
    uiControlDisable(uiControl(btn_serial_port_conn));
    uiButtonSetText(btn_serial_port_conn, "连接");
  }
}

static void serial_ports_refresh()
{
  int last_sel = uiComboboxSelected(cbox_serial_port);
  char *last_sel_name = NULL;
  if (last_sel >= 0 && last_sel < port_names_n)
    last_sel_name = port_names[last_sel];
  uiComboboxClear(cbox_serial_port);

#define port_valid(_port) \
  (sp_get_port_transport(_port) == SP_TRANSPORT_USB)

  struct sp_port **port_list;
  ensure_or_abort(sp_list_ports(&port_list) >= 0, "Cannot retrieve list of ports");
  int n = 0;
  for (int i = 0; port_list[i] != NULL; i++)
    if (port_valid(port_list[i])) n++;
  char **new_names = malloc(sizeof(char *) * n);
  ensure_or_abort(new_names != NULL, "Out of memory");

  int new_sel = -1;
  for (int i = 0, n1 = 0; port_list[i] != NULL; i++) if (port_valid(port_list[i])) {
    const char *product_name = sp_get_port_usb_product(port_list[i]);
    const char *port_name = sp_get_port_name(port_list[i]);
    new_names[n1] = strdup(port_name);
    char *disp;
    asprintf(&disp, "%s (%s)", product_name, port_name);
    ensure_or_abort(new_names[n1] != NULL && disp != NULL, "Out of memory");
    uiComboboxAppend(cbox_serial_port, disp);
    free(disp);
    if (last_sel_name != NULL && strcmp(last_sel_name, new_names[n1]) == 0)
      new_sel = n1;
    n1++;
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

  update_cbox_disp();

  start_list_refresh_timer_if_unconnected();
}
static int serial_ports_refresh_timer(void *_unused)
{
  serial_ports_refresh();
  timer_running = (port == NULL);
  return timer_running;
}
static void start_list_refresh_timer_if_unconnected()
{
  if (port == NULL && !timer_running) {
    timer_running = true;
    uiTimer(500, serial_ports_refresh_timer, NULL);
  }
}

static uint32_t mumu_bin_len;
static uint8_t mumu_bin[65536];
static uint32_t mumu_src_len;
static uint8_t mumu_src[65536];

static void conn()
{
#define ensure_or_reject(_cond, ...) \
  ensure_or_act(_cond, goto _retry, __VA_ARGS__)

  int sel = uiComboboxSelected(cbox_serial_port);
  if (sel < 0 || sel >= port_names_n) return;
  if (!open_port(port_names[sel], 921600)) {
    close_port();
    error_and_continue();
    return;
  }

  bool successful = false;
  for (int att = 0; att < 3; att++) {
    if (att > 0) {
      fprintf(stderr, "att = %d - Last error: %s\n", att, global_err_msg);
      usleep(5000 + 3000 * att);  // Reset device reception state
    }
    // Try to request first packet of data
    if (tx((uint8_t []){0xAA}, 1) < 0) continue;
    int rx_len = rx();
    if (rx_len < 0) continue;

    ensure_or_reject(rx_len >= 11 &&
      memcmp(rx_buf, "\x55" "Amakinoko", 10) == 0, "Invalid device signature");
    uint8_t revision = rx_buf[10];
    ensure_or_reject(revision == 0x20, "Invalid device revision 0x%02x", (unsigned)revision);
    printf("Device revision 0x%02x\n", (unsigned)revision);

    ensure_or_reject(rx_len >= 11 + TX_READINGS_LEN, "Invalid readings");
    parse_readings(rx_buf + 11);

    rx_len = rx_timeout(mumu_bin, 10);
    if (rx_len < 0) continue;
    mumu_bin_len = rx_len;
    printf("received %d bytes binary\n", rx_len);

    rx_len = rx_timeout(mumu_src, 10);
    if (rx_len < 0) continue;
    mumu_src_len = rx_len;
    printf("received %d bytes source\n", rx_len);
    mumu_src[mumu_src_len] = '\0';
    for (int i = 0; i < mumu_src_len; i++)
      if ((mumu_src[i] < 32 || mumu_src[i] > 126) && mumu_src[i] != '\n')
        mumu_src[i] = '?';
    uiMultilineEntrySetText(text_source, (const char *)mumu_src);

    serial_loop_pause_req = serial_loop_pause_state = false;
    ensure_or_reject(
      pthread_create(&serial_loop_thr, NULL, &serial_loop_fn, NULL) == 0,
      "Cannot create thread, check system resource usage");

    successful = true;
    break;

_retry: { }
  }

  if (!successful) {
    close_port();
    error_and_continue();
  } else {
    uiControlEnable(uiControl(btn_upload));
    status_bar("✓ 已连接");
  }
  update_cbox_disp();
}

static void btn_serial_port_conn_clicked(uiButton *btn, void *_unused)
{
  if (port == NULL) {
    conn();
  } else {
    close_port();
    update_cbox_disp();
  }
}

static void btn_check_clicked(uiButton *btn, void *_unused)
{
}

static void btn_upload_clicked(uiButton *btn, void *_unused)
{
  int rom_len = 0, src_len = 0;
  bool error = false;

  char *src = uiMultilineEntryText(text_source);
  src_len = strlen(src);

  const char *rom = "1234";
  rom_len = strlen(rom);

  ensure_or_act(rom_len > 0 && src_len > 0,
    { error = true; goto _fin; }, "Program cannot be empty");
  ensure_or_act(rom_len <= 0xffff && src_len <= 0xffff,
    { error = true; goto _fin; }, "Size exceeds limit");

  thread_rx_pause(true);

  int att = 0;

_retry:
  if (att != 0) fprintf(stderr, "att = %d\n", att);
  if (tx((uint8_t []){0xFA,
      rom_len / 256, rom_len % 256,
      src_len / 256, src_len % 256,
    }, 5) < 0) { error = true; goto _fin; }

  // Check code
  int rx_len = thread_rx();
  if (rx_len < 0) { error = true; goto _fin; }
  ensure_or_act(rx_len == 1 && rx_buf[0] == 0xFC,
    { error = true; goto _fin; }, "Size exceeds limit");

  for (int i = 0; i < rom_len; i += 256) {
    int n = (rom_len - i >= 256 ? 256 : rom_len - i);
    if (tx((uint8_t *)rom + i, n) < 0) { error = true; goto _fin; }
    usleep(30000);
  }
  for (int i = 0; i < src_len; i += 256) {
    int n = (src_len - i >= 256 ? 256 : src_len - i);
    if (tx((uint8_t *)src + i, n) < 0) { error = true; goto _fin; }
    usleep(30000);
  }

  rx_len = thread_rx();
  if (rx_len < 0) { error = true; goto _fin; }
  ensure_or_act(rx_len == 5 && rx_buf[0] == 0xCC,
    { error = true; goto _fin; }, "Cannot verify written data");
  uint32_t crc_device =
    ((uint32_t)rx_buf[1] << 24) |
    ((uint32_t)rx_buf[2] << 16) |
    ((uint32_t)rx_buf[3] <<  8) |
    ((uint32_t)rx_buf[4] <<  0);

  uint32_t crc_host = 0xffffffff;
  for (int i = 0; i < rom_len; i++) crc_host = crc32_update(crc_host, rom[i]);
  for (int i = 0; i < src_len; i++) crc_host = crc32_update(crc_host, src[i]);

  if (crc_device != crc_host) {
    if (++att < 3) {
      goto _retry;
    } else {
      fprintf(stderr, "Device CRC = %08x, host CRC = %08x\n", (unsigned)crc_device, (unsigned)crc_host);
      ensure_or_act(false, { error = true; goto _fin; }, "Data written is corrupted. Please contact distributor.");
    }
  }

  status_bar("✓ 上传成功");

_fin:
  thread_rx_pause(false);
  uiFreeText(src);
  if (error) error_and_continue();
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

      cbox_serial_port = uiNewCombobox();
      uiBoxAppend(box_serial_port_r1, uiControl(cbox_serial_port), 1);

      btn_serial_port_conn = uiNewButton("");
      uiBoxAppend(box_serial_port_r1, uiControl(btn_serial_port_conn), 0);
      uiButtonOnClicked(btn_serial_port_conn, btn_serial_port_conn_clicked, cbox_serial_port);
      btn_serial_port_conn_clicked(btn_serial_port_conn, cbox_serial_port);
      serial_ports_refresh(); // Will start timer
    }
  }

  uiBox *box_readings = uiNewHorizontalBox();
  uiBoxSetPadded(box_readings, true);
  uiBoxAppend(box_main, uiControl(box_readings), false);
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
      uiControlDisable(uiControl(area_readings_c_indicators));
    }
  }

  box_program = uiNewVerticalBox();
  uiBoxSetPadded(box_program, true);
  uiBoxAppend(box_main, uiControl(box_program), true);
  {
    uiBox *parent = box_program;

    text_source = uiNewMultilineEntry();
    uiBoxAppend(parent, uiControl(text_source), true);

    btn_check = uiNewButton("检查");
    uiBoxAppend(parent, uiControl(btn_check), false);
    uiButtonOnClicked(btn_check, btn_check_clicked, NULL);

    btn_upload = uiNewButton("上传");
    uiBoxAppend(parent, uiControl(btn_upload), false);
    uiButtonOnClicked(btn_upload, btn_upload_clicked, NULL);

    lbl_status_bar = uiNewLabel("");
    uiBoxAppend(parent, uiControl(lbl_status_bar), false);
  }

  clear_readings_disp();

  // Run main loop
  uiWindowOnClosing(w, window_on_closing, NULL);
  uiOnShouldQuit(on_should_quit, NULL);
  uiControlShow(uiControl(w));
  uiMain();

  return 0;
}
