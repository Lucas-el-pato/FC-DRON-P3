#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define DSHOT_MAX_THROTTLE   2047
#define DSHOT_MIN_THROTTLE   48

#define DSHOT_CMD_MOTOR_STOP 0
#define DSHOT_CMD_BEEP1      1
#define DSHOT_CMD_BEEP2      2
#define DSHOT_CMD_BEEP3      3
#define DSHOT_CMD_BEEP4      4
#define DSHOT_CMD_BEEP5      5

typedef struct dshot600_ctx *dshot600_handle_t;

esp_err_t dshot600_new(int gpio, dshot600_handle_t *out);
esp_err_t dshot600_send(dshot600_handle_t handle, uint16_t throttle, bool telemetry_request);
esp_err_t dshot600_send_command(dshot600_handle_t handle, uint16_t command, bool telemetry_request);
esp_err_t dshot600_arm(dshot600_handle_t handle, uint32_t duration_ms);
void dshot600_delete(dshot600_handle_t handle);
