#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdio.h>

// Logging levels
typedef enum { LOG_INFO, LOG_WARN, LOG_ERROR, LOG_DEBUG } LogLevel;

// Get current time in milliseconds
long long current_time_ms();

// Sleep for milliseconds
void sleep_ms(int ms);

// Log message with timestamp
void log_message(LogLevel level, const char *format, ...);

// Crypto helpers for WebSocket
typedef struct {
  uint32_t state[5];
  uint32_t count[2];
  unsigned char buffer[64];
} SHA1_CTX;

void SHA1Transform(uint32_t state[5], const unsigned char buffer[64]);
void SHA1Init(SHA1_CTX *context);
void SHA1Update(SHA1_CTX *context, const unsigned char *data, uint32_t len);
void SHA1Final(unsigned char digest[20], SHA1_CTX *context);

void base64_encode(const unsigned char *data, size_t input_length,
                   char *output_buffer);

// Case insensitive substring search
char *stristr(const char *haystack, const char *needle);

#endif
