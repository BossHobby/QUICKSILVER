#include "driver/usb.h"

#include <mongoose.h>

extern volatile bool usb_device_configured;

static struct mg_mgr mgr;
static struct mg_connection *client = NULL;
static const char *listen_addr = "ws://localhost:3000";

static void handler(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
  switch (ev) {
  case MG_EV_HTTP_MSG: {
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;
    if (mg_http_match_uri(hm, "/websocket")) {
      mg_ws_upgrade(c, hm, NULL);
      usb_device_configured = true;
      client = c;
    } else {
      mg_http_reply(c, 404, "", "");
    }
    break;
  }
  case MG_EV_WS_MSG: {
    struct mg_ws_message *wm = (struct mg_ws_message *)ev_data;
    if (wm->flags & WEBSOCKET_OP_BINARY) {
      ring_buffer_write_multi(&usb_rx_buffer, wm->data.ptr, wm->data.len);
    }
    if (wm->flags & WEBSOCKET_OP_CLOSE) {
      usb_device_configured = false;
      client = NULL;
    }
    break;
  }
  default:
    break;
  }
}

void usb_drv_init() {
  mg_mgr_init(&mgr);
  mg_http_listen(&mgr, listen_addr, handler, NULL);
}

void usb_drv_update() {
  mg_mgr_poll(&mgr, 0);
}

uint32_t usb_serial_read(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return 0;
  }
  return ring_buffer_read_multi(&usb_rx_buffer, data, len);
}

void usb_serial_write(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return;
  }
  if (client == NULL) {
    return;
  }

  uint32_t written = 0;
  while (written < len) {
    written += mg_ws_send(client, data, len, WEBSOCKET_OP_BINARY);
    usb_drv_update();
  }
}
