#pragma once
#include "arena.c"
#include "base.h"
#include "string8.h"
#include <assert.h>
#include <errno.h>

typedef enum JsonType {
    JSON_INVALID = 0, // this represents an invalid json node (sentinel value)
    JSON_NULL,        // this is null value
    JSON_OBJECT,      // this is an object; properties can be found in child nodes
    JSON_ARRAY,       // this is an array; items can be found in child nodes
    JSON_STRING,      // this is a string; value can be found in text_value field
    JSON_INTEGER,     // this is an integer; value can be found in int_value field
    JSON_DOUBLE,      // this is a double; value can be found in dbl_value field
    JSON_BOOL         // this is a boolean; value can be found in int_value field
} JsonType;

typedef struct JsonNode {
    JsonType type; // type of json node, see above
    String8 key;   // key of the property; for object's children only
    union {
        String8 text_value; // text value of STRING node
        struct {
            union {
                U64 u_value; // the value of INTEGER or BOOL node
                S64 s_value;
            };
            F64 dbl_value; // the value of DOUBLE node
        } num;
        struct { // children of OBJECT or ARRAY
            S32 count;
            struct JsonNode *first;
            struct JsonNode *last;
        } children;
    };
    struct JsonNode *prev; //  points to previous child
    struct JsonNode *next; // points to next child
} JsonNode;

#define JSON_REPORT_ERROR(msg, p)                                                        \
    fprintf(stderr, "NXJSON PARSE ERROR (%d): " msg " at %s\n", __LINE__, p)

#define IS_WHITESPACE(c) ((unsigned char)(c) <= (unsigned char)' ')

global JsonNode json_node_null = {.type = JSON_INVALID,
                                  .key = (String8){0},
                                  .prev = &json_node_null,
                                  .next = &json_node_null};

static JsonNode *JsonCreate(Arena *arena, JsonType type, String8 key, JsonNode *parent) {
    JsonNode *js = arena_alloc(arena, sizeof(JsonNode));
    assert(js);
    js->type = type;
    js->key = key;
    js->prev = &json_node_null;
    js->next = &json_node_null;
    if (!parent->children.last) {
        parent->children.first = parent->children.last = js;
    } else {
        js->prev = parent->children.last;
        parent->children.last->next = js;
        parent->children.last = js;
    }
    parent->children.count++;
    return js;
}

static String8 UnescapeString(char *s, char **end) {
    char *p = s;
    char *d = s;
    char c;
    while ((c = *p++)) {
        if (c == '"') {
            *d = '\0';
            *end = p;
            return String8FromCStringLen(s, (size_t)(p - s) - 1);
        } else if (c == '\\') {
            switch (*p) {
            case '\\':
            case '/':
            case '"':
                *d++ = *p++;
                break;
            case 'b':
                *d++ = '\b';
                p++;
                break;
            case 'f':
                *d++ = '\f';
                p++;
                break;
            case 'n':
                *d++ = '\n';
                p++;
                break;
            case 'r':
                *d++ = '\r';
                p++;
                break;
            case 't':
                *d++ = '\t';
                p++;
                break;
            case 'u': // unicode
                JSON_REPORT_ERROR("unicode not supported", p - 1);
                exit(EXIT_FAILURE);
            default:
                // leave untouched
                *d++ = c;
                break;
            }
        } else {
            *d++ = c;
        }
    }
    JSON_REPORT_ERROR("no closing quote for string", s);
    return (String8){0};
}

static char *SkipBlockComment(char *p) {
    // assume p[-2]=='/' && p[-1]=='*'
    char *ps = p - 2;
    if (!*p) {
        JSON_REPORT_ERROR("endless comment", ps);
        return 0;
    }
REPEAT:
    p = strchr(p + 1, '/');
    if (!p) {
        JSON_REPORT_ERROR("endless comment", ps);
        return 0;
    }
    if (p[-1] != '*')
        goto REPEAT;
    return p + 1;
}

static char *ParseKey(String8 *key, char *p) {
    // on '}' return with *p=='}'
    char c;
    while ((c = *p++)) {
        if (c == '"') {
            *key = UnescapeString(p, &p);
            if (!*key->buf)
                return 0; // propagate error
            while (*p && IS_WHITESPACE(*p))
                p++;
            if (*p == ':')
                return p + 1;
            JSON_REPORT_ERROR("unexpected chars", p);
            return 0;
        } else if (IS_WHITESPACE(c) || c == ',') {
            // continue
        } else if (c == '}') {
            return p - 1;
        } else if (c == '/') {
            if (*p == '/') { // line comment
                char *ps = p - 1;
                p = strchr(p + 1, '\n');
                if (!p) {
                    JSON_REPORT_ERROR("endless comment", ps);
                    return 0; // error
                }
                p++;
            } else if (*p == '*') { // block comment
                p = SkipBlockComment(p + 1);
                if (!p)
                    return 0;
            } else {
                JSON_REPORT_ERROR("unexpected chars", p - 1);
                return 0; // error
            }
        } else {
            JSON_REPORT_ERROR("unexpected chars", p - 1);
            return 0; // error
        }
    }
    JSON_REPORT_ERROR("unexpected chars", p - 1);
    return 0; // error
}

static char *JsonParseValue(Arena *arena, JsonNode *parent, String8 key, char *p) {
    JsonNode *js;
    for (;;) {
        switch (*p) {
        case '\0':
            JSON_REPORT_ERROR("unexpected end of text", p);
            return 0; // error
        case ' ':
        case '\t':
        case '\n':
        case '\r':
        case ',':
            // skip
            p++;
            break;
        case '{':
            js = JsonCreate(arena, JSON_OBJECT, key, parent);
            p++;
            while (1) {
                String8 new_key = {0};
                p = ParseKey(&new_key, p);
                if (!p)
                    return 0; // error
                if (*p == '}')
                    return p + 1; // end of object
                p = JsonParseValue(arena, js, new_key, p);
                if (!p)
                    return 0; // error
            }
        case '[':
            js = JsonCreate(arena, JSON_ARRAY, key, parent);
            p++;
            while (1) {
                p = JsonParseValue(arena, js, (String8){0}, p);
                if (!p)
                    return 0; // error
                if (*p == ']')
                    return p + 1; // end of array
            }
        case ']':
            return p;
        case '"':
            p++;
            js = JsonCreate(arena, JSON_STRING, key, parent);
            js->text_value = UnescapeString(p, &p);
            if (!js->text_value.buf)
                return 0; // propagate error
            return p;
        case '-':
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9': {
            js = JsonCreate(arena, JSON_INTEGER, key, parent);
            char *pe;
            if (*p == '-') {
                js->num.s_value = strtoll(p, &pe, 0);
            } else {
                js->num.u_value = strtoull(p, &pe, 0);
            }
            if (pe == p || errno == ERANGE) {
                JSON_REPORT_ERROR("invalid number", p);
                return 0; // error
            }
            if (*pe == '.' || *pe == 'e' || *pe == 'E') { // double value
                js->type = JSON_DOUBLE;
                js->num.dbl_value = strtod(p, &pe);
                if (pe == p || errno == ERANGE) {
                    JSON_REPORT_ERROR("invalid number", p);
                    return 0; // error
                }
            } else {
                if (*p == '-') {
                    js->num.dbl_value = (F64)js->num.s_value;
                } else {
                    js->num.dbl_value = (F64)js->num.u_value;
                }
            }
            return pe;
        }
        case 't':
            if (!strncmp(p, "true", 4)) {
                js = JsonCreate(arena, JSON_BOOL, key, parent);
                js->num.u_value = 1;
                return p + 4;
            }
            JSON_REPORT_ERROR("unexpected chars", p);
            return 0; // error
        case 'f':
            if (!strncmp(p, "false", 5)) {
                js = JsonCreate(arena, JSON_BOOL, key, parent);
                js->num.u_value = 0;
                return p + 5;
            }
            JSON_REPORT_ERROR("unexpected chars", p);
            return 0; // error
        case 'n':
            if (!strncmp(p, "null", 4)) {
                JsonCreate(arena, JSON_NULL, key, parent);
                return p + 4;
            }
            JSON_REPORT_ERROR("unexpected chars", p);
            return 0;          // error
        case '/':              // comment
            if (p[1] == '/') { // line comment
                char *ps = p;
                p = strchr(p + 2, '\n');
                if (!p) {
                    JSON_REPORT_ERROR("endless comment", ps);
                    return 0; // error
                }
                p++;
            } else if (p[1] == '*') { // block comment
                p = SkipBlockComment(p + 2);
                if (!p)
                    return 0;
            } else {
                JSON_REPORT_ERROR("unexpected chars", p);
                return 0; // error
            }
            break;
        default:
            JSON_REPORT_ERROR("unexpected chars", p);
            return 0; // error
        }
    }
}

// parses a raw json string into the nodes.
// requires at least one character in the string
// returns the root json object.
static JsonNode *JsonNodeFromString(Arena *arena, char *raw_json) {
    assert(raw_json);
    JsonNode root = {0};
    JsonParseValue(arena, &root, (String8){0}, raw_json);
    assert(root.children.first->type == JSON_OBJECT);
    return root.children.first;
}

const JsonNode *JsonGet(const JsonNode *json, String8 key) {
    for (JsonNode *js = json->children.first; js; js = js->next) {
        if (js->key.buf && !string8_compare(js->key, key))
            return js;
    }
    return &json_node_null;
}

const JsonNode *JsonItem(const JsonNode *json, int idx) {
    for (JsonNode *js = json->children.first; js; js = js->next) {
        if (!idx--)
            return js;
    }
    return &json_node_null;
}

// ---------------------------------------
//         serialization helpers
// ---------------------------------------

// locates a child node with given key, value pair
JsonNode *JsonFindKeyValue(const JsonNode *node, const String8 key, const String8 value) {
    assert(node->type == JSON_OBJECT);
    JsonNode *current = node->children.first;
    while (current != &json_node_null) {
        if (current->type == JSON_STRING) {
            if (String8Equals(current->key, key) &&
                String8Equals(current->text_value, value)) {
                return current;
            }
        }
        current = current->next;
    }
    return &json_node_null;
}

// locates a child node with given key
JsonNode *JsonFindKey(const JsonNode *node, String8 key) {
    assert(node->type == JSON_OBJECT);
    JsonNode *current = node->children.first;
    while (current != &json_node_null) {
        if (String8Equals(current->key, key)) {
            return current;
        }
        current = current->next;
    }
    return &json_node_null;
}
