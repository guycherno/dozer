#define _GNU_SOURCE
#include "robot.h"
#include "server.h"
#include "utils.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


volatile int running = 1;

void handle_signal(int sig) {
  (void)sig;
  running = 0;
}

int main() {
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  log_message(LOG_INFO, "Dozer1 C Port Starting...");

  Robot robot;
  if (!robot_init(&robot)) {
    log_message(LOG_ERROR, "Failed to initialize Robot");
    return 1;
  }

  Server server;
  if (!server_init(&server, 8000)) {
    log_message(LOG_ERROR, "Failed to initialize Server");
    robot_close(&robot);
    return 1;
  }

  long long last_time = current_time_ms();
  float dt = 0.02f; // Target 50Hz

  while (running) {
    long long now = current_time_ms();
    long long elapsed_ms = now - last_time;

    // Update at 50Hz (20ms)
    if (elapsed_ms >= 20) {
      dt = elapsed_ms / 1000.0f;
      last_time = now;

      robot_update(&robot, dt);
      server_poll(&server, &robot);
    } else {
      sleep_ms(2);
    }
  }

  log_message(LOG_INFO, "Shutting down...");
  server_close(&server);
  robot_close(&robot);

  return 0;
}
