#ifndef JSON_H
#define JSON_H

// Simple JSON helper for Dozer1
// Not a full validator, just a data extractor.

int json_get_float(const char *json, const char *key, float *out_val);
int json_get_string(const char *json, const char *key, char *out_buf,
                    int max_len);
int json_get_object(const char *json, const char *key, char *out_buf,
                    int max_len);

// Builder
void json_start(char *buf);
void json_add_string(char *buf, const char *key, const char *val);
void json_add_float(char *buf, const char *key, float val);
void json_add_int(char *buf, const char *key, int val);
void json_end(char *buf);

#endif
