#define _GNU_SOURCE
#include "server.h"
#include "robot.h"
#include "utils.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define BUFFER_SIZE 2048
#define WS_GUID "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

static void set_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int server_init(Server *server, int port) {
  server->port = port;
  server->client_fd = -1;
  server->state = SERVER_DISCONNECTED;

  server->server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server->server_fd == 0) {
    log_message(LOG_ERROR, "Server: Socket failed");
    return 0;
  }

  int opt = 1;
  setsockopt(server->server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  server->address.sin_family = AF_INET;
  server->address.sin_addr.s_addr = INADDR_ANY;
  server->address.sin_port = htons(port);

  if (bind(server->server_fd, (struct sockaddr *)&server->address,
           sizeof(server->address)) < 0) {
    log_message(LOG_ERROR, "Server: Bind failed on port %d", port);
    return 0;
  }

  if (listen(server->server_fd, 1) < 0) {
    log_message(LOG_ERROR, "Server: Listen failed");
    return 0;
  }

  set_nonblocking(server->server_fd);
  log_message(LOG_INFO, "Server: Listening on port %d", port);
  return 1;
}

void server_close(Server *server) {
  if (server->client_fd != -1)
    close(server->client_fd);
  if (server->server_fd != -1)
    close(server->server_fd);
}

// --- Static File Serving ---

static const char *get_mime_type(const char *path) {
  const char *ext = strrchr(path, '.');
  if (!ext)
    return "text/plain";
  if (strcmp(ext, ".html") == 0)
    return "text/html";
  if (strcmp(ext, ".js") == 0)
    return "application/javascript";
  if (strcmp(ext, ".css") == 0)
    return "text/css";
  if (strcmp(ext, ".png") == 0)
    return "image/png";
  if (strcmp(ext, ".ico") == 0)
    return "image/x-icon";
  return "text/plain";
}

static void serve_file(int client_fd, const char *path) {
  char full_path[512];

  // Default to index.html
  if (strcmp(path, "/") == 0) {
    path = "/index.html";
  }

  // Prevent directory traversal
  if (strstr(path, "..")) {
    char *resp = "HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\nForbidden";
    write(client_fd, resp, strlen(resp));
    return;
  }

  // Construct path relative to dozer_c/ directory
  // Assuming strict project structure: dozer_c/../dozer1/static
  snprintf(full_path, sizeof(full_path), "../dozer1/static%s", path);

  FILE *f = fopen(full_path, "rb");
  if (!f) {
    log_message(LOG_WARN, "Server: 404 Not Found: %s", full_path);
    char *resp = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\nNot Found";
    write(client_fd, resp, strlen(resp));
    return;
  }

  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *content = malloc(fsize);
  if (!content) {
    fclose(f);
    return;
  }
  fread(content, 1, fsize, f);
  fclose(f);

  const char *mime = get_mime_type(full_path);
  char header[256];
  snprintf(header, sizeof(header),
           "HTTP/1.1 200 OK\r\n"
           "Content-Type: %s\r\n"
           "Content-Length: %ld\r\n"
           "Connection: close\r\n\r\n",
           mime, fsize);

  write(client_fd, header, strlen(header));
  write(client_fd, content, fsize);
  free(content);

  log_message(LOG_INFO, "Server: Served %s", full_path);
}

// Minimal handshake
static int perform_handshake(int client_fd) {
  char buf[BUFFER_SIZE];
  int len = read(client_fd, buf, BUFFER_SIZE - 1);
  if (len <= 0)
    return 0;
  buf[len] = 0;

  // Check for "Sec-WebSocket-Key: "
  char *key_start = stristr(buf, "Sec-WebSocket-Key:");
  if (!key_start) {
    // Check if it's a standard HTTP GET
    if (strncmp(buf, "GET ", 4) == 0) {
      // Extract path
      char *path_start = buf + 4;
      char *path_end = strchr(path_start, ' ');
      if (path_end) {
        *path_end = 0;
        serve_file(client_fd, path_start);
      }
      return 0; // Handled as HTTP, close connection (not WS)
    }
    log_message(LOG_WARN, "Handshake: Header not found. Buffer:\n%s", buf);
    return 0; // Not a WS request (or valid one)
  }

  // Skip "Sec-WebSocket-Key:" (18 chars)
  key_start += 18;

  // Skip spaces
  while (*key_start == ' ')
    key_start++;

  // Find EOL
  char *key_end = strstr(key_start, "\r\n");
  if (!key_end)
    key_end = strstr(key_start, "\n"); // Fallback

  if (!key_end) {
    log_message(LOG_WARN, "Handshake: EOL not found after key");
    return 0;
  }

  char key[64];
  int key_len = key_end - key_start;
  if (key_len >= (int)sizeof(key))
    key_len = sizeof(key) - 1;
  strncpy(key, key_start, key_len);
  key[key_len] = 0;

  // Concatenate
  char concat[128];
  snprintf(concat, sizeof(concat), "%s%s", key, WS_GUID);

  // SHA1
  SHA1_CTX ctx;
  unsigned char hash[20];
  SHA1Init(&ctx);
  SHA1Update(&ctx, (unsigned char *)concat, strlen(concat));
  SHA1Final(hash, &ctx);

  // Base64
  char accept_key[64];
  base64_encode(hash, 20, accept_key);

  // Response
  char response[256];
  snprintf(response, sizeof(response),
           "HTTP/1.1 101 Switching Protocols\r\n"
           "Upgrade: websocket\r\n"
           "Connection: Upgrade\r\n"
           "Sec-WebSocket-Accept: %s\r\n\r\n",
           accept_key);

  write(client_fd, response, strlen(response));
  return 1;
}

// Decode frame
// Returns 0 if incomplete/no data, 1 if got data, -1 if error/close
static int ws_read_frame(int fd, char *out_buf, int max_len) {
  unsigned char head[2];
  int n = recv(fd, head, 2, MSG_DONTWAIT | MSG_PEEK);
  if (n <= 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return 0;
    return -1; // Closed
  }

  // We have at least header. Calculate full length needed.
  int payload_len = head[1] & 0x7F;
  int head_len = 2;
  if (payload_len == 126)
    head_len = 4;
  else if (payload_len == 127)
    head_len = 10;

  int mask_len = (head[1] & 0x80) ? 4 : 0;
  int total_len = head_len + mask_len + payload_len;

  // Check if we have enough data
  // Use peek to check available? Or just try read.
  // Let's just try to read header then rest. safely.
  // Actually, proper way is buffering.
  // Simplified: Assume strictly small frames (commands).
  // Just read everything we can.

  unsigned char buf[BUFFER_SIZE];
  n = recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
  if (n < total_len) {
    // Partial frame or nothing meaningful.
    // If n > 0 but < total_len, we might lose data with this naive impl.
    // TODO: Implement buffering for robust stream.
    return 0;
  }

  // Parse
  int opcode = buf[0] & 0x0F;
  if (opcode == 0x8)
    return -1; // Close

  int data_idx = head_len + mask_len;
  unsigned char mask[4] = {0};
  if (mask_len) {
    memcpy(mask, &buf[head_len], 4);
  }

  int out_len = 0;
  for (int i = 0; i < payload_len; i++) {
    unsigned char c = buf[data_idx + i];
    if (mask_len)
      c ^= mask[i % 4];
    if (out_len < max_len - 1) {
      out_buf[out_len++] = c;
    }
  }
  out_buf[out_len] = 0;
  return 1;
}

void server_broadcast(Server *server, const char *msg) {
  if (server->state != SERVER_CONNECTED)
    return;

  unsigned char frame[BUFFER_SIZE];
  int len = strlen(msg);

  frame[0] = 0x81; // Text, Fin
  int head_len;

  if (len <= 125) {
    frame[1] = len;
    head_len = 2;
  } else if (len <= 65535) {
    frame[1] = 126;
    frame[2] = (len >> 8) & 0xFF;
    frame[3] = len & 0xFF;
    head_len = 4;
  } else {
    return; // Too big
  }

  memcpy(&frame[head_len], msg, len);
  send(server->client_fd, frame, head_len + len, MSG_DONTWAIT | MSG_NOSIGNAL);
}

void server_poll(Server *server, Robot *robot) {
  // 1. Accept new connection
  if (server->state == SERVER_DISCONNECTED) {
    int cf = accept(server->server_fd, NULL, NULL);
    if (cf >= 0) {
      log_message(LOG_INFO, "Server: Client connecting...");
      if (perform_handshake(cf)) {
        server->client_fd = cf;
        server->state = SERVER_CONNECTED;
        set_nonblocking(cf);
        log_message(LOG_INFO, "Server: WebSocket Handshake Success");
      } else {
        close(cf);
        log_message(LOG_WARN, "Server: Handshake failed");
      }
    }
  }

  // 2. Read from client
  if (server->state == SERVER_CONNECTED) {
    char msg[1024];
    int res = ws_read_frame(server->client_fd, msg, sizeof(msg));
    if (res < 0) {
      log_message(LOG_INFO, "Server: Client disconnected");
      close(server->client_fd);
      server->client_fd = -1;
      server->state = SERVER_DISCONNECTED;
    } else if (res > 0) {
      // Got command
      log_message(LOG_INFO, "Server Received: %s", msg);
      char resp_msg[256];
      int success =
          robot_execute_command(robot, msg, resp_msg, sizeof(resp_msg));

      // Send result json
      char json[512];
      snprintf(
          json, sizeof(json),
          "{\"type\":\"command_result\",\"success\":%s,\"message\":\"%s\"}",
          success ? "true" : "false", resp_msg);
      server_broadcast(server, json);
    }
  }
}
