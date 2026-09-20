#ifndef KSP_LANDER_JSON_H
#define KSP_LANDER_JSON_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    JSON_UNDEFINED = 0,
    JSON_OBJECT = 1,
    JSON_ARRAY = 2,
    JSON_STRING = 3,
    JSON_PRIMITIVE = 4
} JsonType;

typedef struct {
    JsonType type;
    int start;
    int end;
    int size;
    int parent;
} JsonToken;

typedef struct {
    const char *text;
    JsonToken *tokens;
    int count;
} JsonDoc;

int json_parse(const char *text, JsonToken *tokens, int capacity, JsonDoc *doc);
int json_object_get(const JsonDoc *doc, int object_index, const char *key);
int json_array_get(const JsonDoc *doc, int array_index, int element_index);
bool json_token_eq(const JsonDoc *doc, int index, const char *value);
double json_number(const JsonDoc *doc, int index, double fallback);
long json_integer(const JsonDoc *doc, int index, long fallback);
bool json_boolean(const JsonDoc *doc, int index, bool fallback);
bool json_string(const JsonDoc *doc, int index, char *out, size_t out_size);
int json_skip(const JsonDoc *doc, int index);

typedef struct {
    char *data;
    size_t length;
    size_t capacity;
    bool failed;
} JsonWriter;

void jw_init(JsonWriter *w);
void jw_free(JsonWriter *w);
void jw_raw(JsonWriter *w, const char *text);
void jw_char(JsonWriter *w, char c);
void jw_string(JsonWriter *w, const char *text);
void jw_number(JsonWriter *w, double value);
void jw_integer(JsonWriter *w, long long value);
void jw_bool(JsonWriter *w, bool value);
void jw_null(JsonWriter *w);

#endif
