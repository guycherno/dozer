#include "json.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Helper to find a key start: "key":
static const char *find_key(const char *json, const char *key) {
  // This is a naive implementation. It searches for "key" and checks context.
  // It doesn't handle escaped quotes within strings correctly in all cases.
  char pattern[128];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);

  const char *p = strstr(json, pattern);
  while (p) {
    // Check if it's a key (followed by :)
    const char *end = p + strlen(pattern);
    while (*end && isspace(*end))
      end++;
    if (*end == ':') {
      return end + 1; // Return pointer to value start
    }
    p = strstr(p + 1, pattern);
  }
  return NULL;
}

int json_get_float(const char *json, const char *key, float *out_val) {
  const char *val_start = find_key(json, key);
  if (!val_start)
    return 0;

  *out_val = strtof(val_start, NULL);
  return 1;
}

int json_get_string(const char *json, const char *key, char *out_buf,
                    int max_len) {
  const char *val_start = find_key(json, key);
  if (!val_start)
    return 0;

  // Skip spaces
  while (*val_start && isspace(*val_start))
    val_start++;

  if (*val_start != '\"')
    return 0;  // Not a string
  val_start++; // Skip opening quote

  int i = 0;
  while (*val_start && *val_start != '\"' && i < max_len - 1) {
    // TODO: handle escape chars
    out_buf[i++] = *val_start++;
  }
  out_buf[i] = '\0';
  return 1;
}

int json_get_object(const char *json, const char *key, char *out_buf,
                    int max_len) {
  // Extract {...} block
  const char *val_start = find_key(json, key);
  if (!val_start)
    return 0;

  while (*val_start && isspace(*val_start))
    val_start++;
  if (*val_start != '{')
    return 0;

  int depth = 0;
  int i = 0;
  do {
    if (*val_start == '{')
      depth++;
    if (*val_start == '}')
      depth--;
    out_buf[i++] = *val_start++;
  } while (depth > 0 && *val_start && i < max_len - 1);

  out_buf[i] = '\0';
  return 1;
}

// Builder
void json_start(char *buf) { strcpy(buf, "{"); }

void json_add_string(char *buf, const char *key, const char *val) {
  // Check if we need a comma (if len > 1)
  if (strlen(buf) > 1)
    strcat(buf, ",");
  char temp[256];
  snprintf(temp, sizeof(temp), "\"%s\":\"%s\"", key, val);
  strcat(buf, temp);
}

void json_add_float(char *buf, const char *key, float val) {
  if (strlen(buf) > 1)
    strcat(buf, ",");
  char temp[64];
  snprintf(temp, sizeof(temp), "\"%s\":%.4f", key, val);
  strcat(buf, temp);
}

void json_add_int(char *buf, const char *key, int val) {
  if (strlen(buf) > 1)
    strcat(buf, ",");
  char temp[64];
  snprintf(temp, sizeof(temp), "\"%s\":%d", key, val);
  strcat(buf, temp);
}

void json_end(char *buf) { strcat(buf, "}"); }
