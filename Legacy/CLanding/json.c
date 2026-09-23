#include "json.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    unsigned pos;
    unsigned next;
    int parent;
} Parser;

static JsonToken *alloc_token(Parser *p, JsonToken *tokens, int cap) {
    if ((int)p->next >= cap) return NULL;
    JsonToken *t = &tokens[p->next++];
    t->type = JSON_UNDEFINED;
    t->start = -1;
    t->end = -1;
    t->size = 0;
    t->parent = -1;
    return t;
}

static void fill_token(JsonToken *t, JsonType type, int start, int end) {
    t->type = type;
    t->start = start;
    t->end = end;
    t->size = 0;
}

static int parse_primitive(Parser *p, const char *js, size_t len, JsonToken *tokens, int cap) {
    unsigned start = p->pos;
    for (; p->pos < len; p->pos++) {
        char c = js[p->pos];
        if (c == '\t' || c == '\r' || c == '\n' || c == ' ' || c == ',' || c == ']' || c == '}') break;
        if ((unsigned char)c < 32 || c == ':' || c == '"') return -2;
    }
    if (tokens) {
        JsonToken *t = alloc_token(p, tokens, cap);
        if (!t) { p->pos = start; return -1; }
        fill_token(t, JSON_PRIMITIVE, (int)start, (int)p->pos);
        t->parent = p->parent;
    }
    p->pos--;
    return 0;
}

static int parse_string(Parser *p, const char *js, size_t len, JsonToken *tokens, int cap) {
    unsigned start = p->pos++;
    for (; p->pos < len; p->pos++) {
        char c = js[p->pos];
        if (c == '"') {
            if (tokens) {
                JsonToken *t = alloc_token(p, tokens, cap);
                if (!t) { p->pos = start; return -1; }
                fill_token(t, JSON_STRING, (int)start + 1, (int)p->pos);
                t->parent = p->parent;
            }
            return 0;
        }
        if (c == '\\' && p->pos + 1 < len) {
            p->pos++;
            char e = js[p->pos];
            if (e == 'u') {
                for (int i = 0; i < 4; i++) {
                    if (++p->pos >= len || !isxdigit((unsigned char)js[p->pos])) return -2;
                }
            } else if (!strchr("\"/bfnrt", e)) {
                return -2;
            }
        }
    }
    p->pos = start;
    return -3;
}

int json_parse(const char *text, JsonToken *tokens, int capacity, JsonDoc *doc) {
    Parser p = {0, 0, -1};
    size_t len = strlen(text);
    for (; p.pos < len; p.pos++) {
        char c = text[p.pos];
        switch (c) {
            case '{': case '[': {
                JsonToken *t = alloc_token(&p, tokens, capacity);
                if (!t) return -1;
                if (p.parent != -1) tokens[p.parent].size++;
                t->type = c == '{' ? JSON_OBJECT : JSON_ARRAY;
                t->start = (int)p.pos;
                t->parent = p.parent;
                p.parent = (int)p.next - 1;
                break;
            }
            case '}': case ']': {
                JsonType type = c == '}' ? JSON_OBJECT : JSON_ARRAY;
                int i = (int)p.next - 1;
                for (; i >= 0; i--) {
                    JsonToken *t = &tokens[i];
                    if (t->start != -1 && t->end == -1) {
                        if (t->type != type) return -2;
                        t->end = (int)p.pos + 1;
                        p.parent = t->parent;
                        break;
                    }
                }
                if (i < 0) return -2;
                break;
            }
            case '"': {
                int r = parse_string(&p, text, len, tokens, capacity);
                if (r < 0) return r;
                if (p.parent != -1) tokens[p.parent].size++;
                break;
            }
            case '\t': case '\r': case '\n': case ' ': case ':': case ',':
                break;
            default: {
                int r = parse_primitive(&p, text, len, tokens, capacity);
                if (r < 0) return r;
                if (p.parent != -1) tokens[p.parent].size++;
                break;
            }
        }
    }
    for (unsigned i = 0; i < p.next; i++) if (tokens[i].start != -1 && tokens[i].end == -1) return -3;
    doc->text = text;
    doc->tokens = tokens;
    doc->count = (int)p.next;
    return doc->count;
}

int json_skip(const JsonDoc *doc, int index) {
    if (index < 0 || index >= doc->count) return index + 1;
    int end = doc->tokens[index].end;
    int i = index + 1;
    while (i < doc->count && doc->tokens[i].start < end) i++;
    return i;
}

bool json_token_eq(const JsonDoc *doc, int index, const char *value) {
    if (index < 0 || index >= doc->count) return false;
    const JsonToken *t = &doc->tokens[index];
    size_t n = strlen(value);
    return (size_t)(t->end - t->start) == n && strncmp(doc->text + t->start, value, n) == 0;
}

int json_object_get(const JsonDoc *doc, int object_index, const char *key) {
    if (object_index < 0 || object_index >= doc->count || doc->tokens[object_index].type != JSON_OBJECT) return -1;
    int object_end = doc->tokens[object_index].end;
    int i = object_index + 1;
    while (i < doc->count && doc->tokens[i].start < object_end) {
        int key_index = i;
        int value_index = i + 1;
        if (value_index >= doc->count) return -1;
        if (doc->tokens[key_index].type == JSON_STRING && json_token_eq(doc, key_index, key)) return value_index;
        i = json_skip(doc, value_index);
    }
    return -1;
}

int json_array_get(const JsonDoc *doc, int array_index, int element_index) {
    if (array_index < 0 || array_index >= doc->count || doc->tokens[array_index].type != JSON_ARRAY) return -1;
    int end = doc->tokens[array_index].end;
    int i = array_index + 1;
    int n = 0;
    while (i < doc->count && doc->tokens[i].start < end) {
        if (n == element_index) return i;
        n++;
        i = json_skip(doc, i);
    }
    return -1;
}

static bool token_copy(const JsonDoc *doc, int index, char *buf, size_t cap) {
    if (index < 0 || index >= doc->count || cap == 0) return false;
    int n = doc->tokens[index].end - doc->tokens[index].start;
    if (n < 0) return false;
    size_t copy = (size_t)n < cap - 1 ? (size_t)n : cap - 1;
    memcpy(buf, doc->text + doc->tokens[index].start, copy);
    buf[copy] = 0;
    return true;
}

double json_number(const JsonDoc *doc, int index, double fallback) {
    char buf[96];
    if (!token_copy(doc, index, buf, sizeof(buf))) return fallback;
    errno = 0;
    char *end = NULL;
    double v = strtod(buf, &end);
    return errno == 0 && end != buf && isfinite(v) ? v : fallback;
}

long json_integer(const JsonDoc *doc, int index, long fallback) {
    char buf[64];
    if (!token_copy(doc, index, buf, sizeof(buf))) return fallback;
    errno = 0;
    char *end = NULL;
    long v = strtol(buf, &end, 10);
    return errno == 0 && end != buf ? v : fallback;
}

bool json_boolean(const JsonDoc *doc, int index, bool fallback) {
    if (json_token_eq(doc, index, "true")) return true;
    if (json_token_eq(doc, index, "false")) return false;
    return fallback;
}

bool json_string(const JsonDoc *doc, int index, char *out, size_t out_size) {
    if (index < 0 || index >= doc->count || doc->tokens[index].type != JSON_STRING || out_size == 0) return false;
    const char *src = doc->text + doc->tokens[index].start;
    int len = doc->tokens[index].end - doc->tokens[index].start;
    size_t o = 0;
    for (int i = 0; i < len && o + 1 < out_size; i++) {
        char c = src[i];
        if (c == '\\' && i + 1 < len) {
            char e = src[++i];
            switch (e) {
                case 'n': c = '\n'; break; case 'r': c = '\r'; break; case 't': c = '\t'; break;
                case 'b': c = '\b'; break; case 'f': c = '\f'; break;
                case '"': c = '"'; break; case '\\': c = '\\'; break; case '/': c = '/'; break;
                default: c = '?'; break;
            }
        }
        out[o++] = c;
    }
    out[o] = 0;
    return true;
}

static void jw_reserve(JsonWriter *w, size_t extra) {
    if (w->failed) return;
    size_t need = w->length + extra + 1;
    if (need <= w->capacity) return;
    size_t cap = w->capacity ? w->capacity : 1024;
    while (cap < need) cap *= 2;
    char *next = realloc(w->data, cap);
    if (!next) { w->failed = true; return; }
    w->data = next;
    w->capacity = cap;
}

void jw_init(JsonWriter *w) { memset(w, 0, sizeof(*w)); jw_reserve(w, 0); if (w->data) w->data[0] = 0; }
void jw_free(JsonWriter *w) { free(w->data); memset(w, 0, sizeof(*w)); }
void jw_raw(JsonWriter *w, const char *text) { size_t n = strlen(text); jw_reserve(w, n); if (w->failed) return; memcpy(w->data + w->length, text, n + 1); w->length += n; }
void jw_char(JsonWriter *w, char c) { jw_reserve(w, 1); if (w->failed) return; w->data[w->length++] = c; w->data[w->length] = 0; }

void jw_string(JsonWriter *w, const char *text) {
    jw_char(w, '"');
    if (!text) text = "";
    for (const unsigned char *p = (const unsigned char *)text; *p; p++) {
        switch (*p) {
            case '"': jw_raw(w, "\\\""); break;
            case '\\': jw_raw(w, "\\\\"); break;
            case '\b': jw_raw(w, "\\b"); break;
            case '\f': jw_raw(w, "\\f"); break;
            case '\n': jw_raw(w, "\\n"); break;
            case '\r': jw_raw(w, "\\r"); break;
            case '\t': jw_raw(w, "\\t"); break;
            default:
                if (*p < 32) {
                    char buf[8]; snprintf(buf, sizeof(buf), "\\u%04x", *p); jw_raw(w, buf);
                } else jw_char(w, (char)*p);
        }
    }
    jw_char(w, '"');
}

void jw_number(JsonWriter *w, double value) {
    if (!isfinite(value)) { jw_raw(w, "0"); return; }
    char buf[64];
    snprintf(buf, sizeof(buf), "%.15g", value);
    jw_raw(w, buf);
}
void jw_integer(JsonWriter *w, long long value) { char buf[48]; snprintf(buf, sizeof(buf), "%lld", value); jw_raw(w, buf); }
void jw_bool(JsonWriter *w, bool value) { jw_raw(w, value ? "true" : "false"); }
void jw_null(JsonWriter *w) { jw_raw(w, "null"); }
