#include "base.h"
#include "raymath.h"
#include "triangulate.h"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <raylib.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <time.h>

void init_all_arrays(Arena *arena, Geo_Json *base, S32 capacity) {
  base->interest_points = PointArrayNew(arena, capacity);

  base->multi_point_coords = Coord2ArrayNew(
      arena, capacity); // TODO: figure out a capacity for the points
  base->multi_points = MultiPointArrayNew(arena, capacity);

  base->line_string_coords = Coord2ArrayNew(
      arena, capacity); // TODO: figure out a capacity for the points
  base->line_strings = LineStringArrayNew(arena, capacity);

  base->multi_line_string_coords = Coord2ArrayNew(
      arena, capacity); // TODO: figure out a capacity for the points
  base->multi_line_string_array = LineStringArrayNew(arena, capacity);
  base->multi_line_strings = MultiLineStringArrayNew(arena, capacity);

  base->polygon_coords = Coord2ArrayNew(
      arena, capacity); // TODO: figure out a capacity for the points
  base->polygon_triangles = TriangleArrayNew(arena, capacity);

  base->multi_polygon_coords = Coord2ArrayNew(
      arena, capacity); // TODO: figure out a capacity for the points
  base->multi_polygon_triangles = TriangleArrayNew(arena, capacity);
  base->multi_polygons = MultiPolygonArrayNew(arena, capacity);
}

typedef enum JsonType {
  JSON_NULL,    // this is null value
  JSON_OBJECT,  // this is an object; properties can be found in child nodes
  JSON_ARRAY,   // this is an array; items can be found in child nodes
  JSON_STRING,  // this is a string; value can be found in text_value field
  JSON_INTEGER, // this is an integer; value can be found in int_value field
  JSON_DOUBLE,  // this is a double; value can be found in dbl_value field
  JSON_BOOL     // this is a boolean; value can be found in int_value field
} JsonType;

typedef struct JsonNode {
  JsonType type; // type of json node, see above
  string8 key;   // key of the property; for object's children only
  union {
    string8 text_value; // text value of STRING node
    struct {
      union {
        U64 u_value; // the value of INTEGER or BOOL node
        S64 s_value;
      };
      F64 dbl_value; // the value of DOUBLE node
    } num;
    struct { // children of OBJECT or ARRAY
      S32 length;
      struct JsonNode *first;
      struct JsonNode *last;
    } children;
  };
  struct JsonNode *prev; //  points to previous child
  struct JsonNode *next; // points to next child
} JsonNode;

static Arena a1 = {0};
static Arena a2 = {0};
Arena *arenas[2] = {&a1, &a2};

#define JSON_REPORT_ERROR(msg, p)                                              \
  fprintf(stderr, "NXJSON PARSE ERROR (%d): " msg " at %s\n", __LINE__, p)

#define IS_WHITESPACE(c) ((unsigned char)(c) <= (unsigned char)' ')

static JsonNode *create_json(Arena *arena, JsonType type, string8 key,
                             JsonNode *parent) {
  JsonNode *js = arena_alloc(arena, sizeof(JsonNode));
  assert(js);
  js->type = type;
  js->key = key;
  if (!parent->children.last) {
    parent->children.first = parent->children.last = js;
  } else {
    js->prev = parent->children.last;
    parent->children.last->next = js;
    parent->children.last = js;
  }
  parent->children.length++;
  return js;
}

static inline int hex_val(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return -1;
}

static string8 unescape_string(char *s, char **end) {
  char *p = s;
  char *d = s;
  char c;
  while ((c = *p++)) {
    if (c == '"') {
      *d = '\0';
      *end = p;
      return string8_from_c_string_len(s, (size_t)(p - s) - 1);
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
  return (string8){0};
}

static char *skip_block_comment(char *p) {
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

static char *parse_key(string8 *key, char *p) {
  // on '}' return with *p=='}'
  char c;
  while ((c = *p++)) {
    if (c == '"') {
      *key = unescape_string(p, &p);
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
        p = skip_block_comment(p + 1);
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

static char *parse_value(Arena *arena, JsonNode *parent, string8 key, char *p) {
  JsonNode *js;
  while (1) {
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
      js = create_json(arena, JSON_OBJECT, key, parent);
      p++;
      while (1) {
        string8 new_key = {0};
        p = parse_key(&new_key, p);
        if (!p)
          return 0; // error
        if (*p == '}')
          return p + 1; // end of object
        p = parse_value(arena, js, new_key, p);
        if (!p)
          return 0; // error
      }
    case '[':
      js = create_json(arena, JSON_ARRAY, key, parent);
      p++;
      while (1) {
        p = parse_value(arena, js, (string8){0}, p);
        if (!p)
          return 0; // error
        if (*p == ']')
          return p + 1; // end of array
      }
    case ']':
      return p;
    case '"':
      p++;
      js = create_json(arena, JSON_STRING, key, parent);
      js->text_value = unescape_string(p, &p);
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
      js = create_json(arena, JSON_INTEGER, key, parent);
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
        js = create_json(arena, JSON_BOOL, key, parent);
        js->num.u_value = 1;
        return p + 4;
      }
      JSON_REPORT_ERROR("unexpected chars", p);
      return 0; // error
    case 'f':
      if (!strncmp(p, "false", 5)) {
        js = create_json(arena, JSON_BOOL, key, parent);
        js->num.u_value = 0;
        return p + 5;
      }
      JSON_REPORT_ERROR("unexpected chars", p);
      return 0; // error
    case 'n':
      if (!strncmp(p, "null", 4)) {
        create_json(arena, JSON_NULL, key, parent);
        return p + 4;
      }
      JSON_REPORT_ERROR("unexpected chars", p);
      return 0;          // error
    case '/':            // comment
      if (p[1] == '/') { // line comment
        char *ps = p;
        p = strchr(p + 2, '\n');
        if (!p) {
          JSON_REPORT_ERROR("endless comment", ps);
          return 0; // error
        }
        p++;
      } else if (p[1] == '*') { // block comment
        p = skip_block_comment(p + 2);
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

const JsonNode *json_get(const JsonNode *json, string8 key) {
  JsonNode *js;
  for (js = json->children.first; js; js = js->next) {
    if (js->key.buf && !string8_compare(js->key, key))
      return js;
  }
  return NULL;
}

const JsonNode *json_item(const JsonNode *json, int idx) {
  JsonNode *js;
  for (js = json->children.first; js; js = js->next) {
    if (!idx--)
      return js;
  }
  return NULL;
}

// serialization helpers
//
//

// locates a child node with given key, value pair
JsonNode *find_key_value_in_children(JsonNode *node, string8 key,
                                     string8 value) {
  JsonNode *current = node->children.first;
  while (current != NULL) {
    if (current->type == JSON_STRING) {
      if (string8_equals(current->key, key) &&
          string8_equals(current->text_value, value)) {
        return current;
      }
    }
    current = current->next;
  }
  return NULL;
}

// locates a child node with given key
JsonNode *find_key_in_children(JsonNode *node, string8 key) {
  JsonNode *current = node->children.first;
  while (current != NULL) {
    if (string8_equals(current->key, key)) {
      return current;
    }
    current = current->next;
  }
  return NULL;
}

Coord2 Coord2FromJsonArrayNode(JsonNode *coordinates) {
  if (coordinates->type != JSON_ARRAY || coordinates->children.length != 2) {
    ERROR_MSG("invalid coordinates for Point supplied")
  }
  JsonNode *x_coordinate = coordinates->children.first;
  JsonNode *y_coordinate = coordinates->children.last;
  if (x_coordinate->type != JSON_DOUBLE || y_coordinate->type != JSON_DOUBLE) {
    ERROR_MSG("invalid coordinate type for Point supplied")
  }
  return (Coord2){
      .x = x_coordinate->num.dbl_value,
      .y = -y_coordinate->num.dbl_value,
  };
}

// iterate backwards over the coordinates omitting the first one as it is
// identical to the last input: [a, b, c, d, e] returns [e, d, c, b]
void contour_from_json_array(JsonNode *coordinates, Coord2Array *result_array) {
  if (coordinates->type != JSON_ARRAY) {
    ERROR_MSG("invalid coordinates node type")
  }
  if (coordinates->children.length <= 2) {
    ERROR_MSG("invalid contour with: %d coordinates\n",
              coordinates->children.length)
  }
  JsonNode *point_coords = coordinates->children.last;
  while (point_coords != NULL && point_coords != coordinates->children.first) {
    Coord2ArrayPush(result_array, Coord2FromJsonArrayNode(point_coords));
    point_coords = point_coords->prev;
  }
}

void Coord2ArrayFromJsonArray(JsonNode *coordinates,
                              Coord2Array *result_array) {

  if (coordinates->type != JSON_ARRAY) {
    ERROR_MSG("invalid coordinates node type")
  }
  JsonNode *point_coords = coordinates->children.first;
  while (point_coords != NULL) {
    Coord2ArrayPush(result_array, Coord2FromJsonArrayNode(point_coords));
    point_coords = point_coords->next;
  }
}

Geo_Json *serialize(Arena *arena, JsonNode *root) {
  Geo_Json *parsed = (Geo_Json *)arena_alloc(arena, sizeof(Geo_Json));
  if (root->type != JSON_NULL || root->children.length != 1) {
    ERROR_MSG("invalid json")
  }
  root = root->children.first;
  if (root->type != JSON_OBJECT) {
    ERROR_MSG("invalid object")
  }
  JsonNode *const featureCollcetionType =
      find_key_value_in_children(root, string8_from_c_string("type"),
                                 string8_from_c_string("FeatureCollection"));
  if (featureCollcetionType == NULL) {
    ERROR_MSG("invalid geojson type")
  }
  JsonNode *const features =
      find_key_in_children(root, string8_from_c_string("features"));
  if (features == NULL || features->type != JSON_ARRAY) {
    ERROR_MSG("invalid geojson features")
  }
  // init_all_arrays(arena, parsed, features->children.length); TODO:
  init_all_arrays(arena, parsed, 100000);
  JsonNode *current_child = features->children.first;

  // parese all "Features" in their appropiate arrays
  Temp_Arena_Memory scratch = GetScratchConflict(&arena, 1);
  while (current_child != NULL) {
    JsonNode *const feature_type =
        find_key_value_in_children(current_child, string8_from_c_string("type"),
                                   string8_from_c_string("Feature"));
    if (feature_type == NULL) {
      ERROR_MSG("invalid feature type")
    }

    JsonNode *const geometry =
        find_key_in_children(current_child, string8_from_c_string("geometry"));
    if (geometry == NULL || geometry->type != JSON_OBJECT) {
      ERROR_MSG("invalid or no geometry suppiled")
    }

    JsonNode *const geometry_type =
        find_key_in_children(geometry, string8_from_c_string("type"));
    if (geometry_type == NULL || geometry_type->type != JSON_STRING) {
      ERROR_MSG("no geometry type suppiled")
    }

    JsonNode *const coordinates =
        find_key_in_children(geometry, string8_from_c_string("coordinates"));
    if (coordinates == NULL || coordinates->type != JSON_ARRAY) {
      ERROR_MSG("no coordinates suppiled")
    }

    // "type" : "Point" parsing
    if (string8_equals(geometry_type->text_value,
                       string8_from_c_string("Point"))) {
      PointArrayPush(
          &parsed->interest_points,
          (Point){.coordinates = Coord2FromJsonArrayNode(coordinates)});
    }

    if (string8_equals(geometry_type->text_value,
                       string8_from_c_string("MultiPoint"))) {
      MultiPoint multi_point = (MultiPoint){
          .coordinates = (Slice){.start = parsed->multi_points.len,
                                 .length = coordinates->children.length},
      };
      Coord2ArrayFromJsonArray(coordinates, &parsed->multi_point_coords);
      // assert that we inserted the correct amount of points
      assert(multi_point.coordinates.start + multi_point.coordinates.length ==
             parsed->multi_point_coords.len);
      MultiPointArrayPush(&parsed->multi_points, multi_point);
    }

    if (string8_equals(geometry_type->text_value,
                       string8_from_c_string("LineString"))) {
      LineString line_string = (LineString){
          .coordinates = (Slice){.start = parsed->line_string_coords.len,
                                 .length = coordinates->children.length},
      };
      Coord2ArrayFromJsonArray(coordinates, &parsed->line_string_coords);
      ASSERT(line_string.coordinates.start + line_string.coordinates.length ==
                 parsed->line_string_coords.len,
             "assert failed at line: %d", __LINE__)
      LineStringArrayPush(&parsed->line_strings, line_string);
    }

    if (string8_equals(geometry_type->text_value,
                       string8_from_c_string("MultiLineString"))) {
      JsonNode *line_string = coordinates->children.first;
      MultiLineString mls = (MultiLineString){
          .coordinates = (Slice){.start = parsed->multi_line_string_array.len,
                                 .length = coordinates->children.length},
      };
      while (line_string != NULL) {
        LineString ls = (LineString){
            .coordinates =
                (Slice){.start = parsed->multi_line_string_coords.len,
                        .length = coordinates->children.length},
        };
        Coord2ArrayFromJsonArray(coordinates,
                                 &parsed->multi_line_string_coords);
        ASSERT(ls.coordinates.start + ls.coordinates.length ==
                   parsed->multi_line_string_coords.len,
               "assert failed at line: %d", __LINE__)
        LineStringArrayPush(&parsed->multi_line_string_array, ls);
        line_string = line_string->next;
      }
      MultiLineStringArrayPush(&parsed->multi_line_strings, mls);
    }

    if (string8_equals(geometry_type->text_value,
                       string8_from_c_string("Polygon"))) {
      S32 contour_count = coordinates->children.length;
      S32 *contour_sizes =
          (S32 *)arena_alloc(scratch.arena, contour_count * (S32)sizeof(S32));
      Coord2 *vertices =
          &parsed->polygon_coords.data[parsed->polygon_coords.len];
      S32 first_coordinate_idx = parsed->polygon_coords.len;
      S32 first_triangle_idx = parsed->polygon_triangles.len;

      // create emtpy slot at vertices[0] for triangulation
      Coord2ArrayPush(&parsed->polygon_coords, (Coord2){0.f, 0.f});

      JsonNode *contour_array = coordinates->children.first;
      S32 i = 0;
      while (contour_array != NULL) {
        if (contour_array->children.length <= 1) {
          ERROR_MSG("invalid size for polygon contour: %d",
                    contour_array->children.length)
        }
        contour_sizes[i] = contour_array->children.length - 1;
        contour_from_json_array(contour_array, &parsed->polygon_coords);
        contour_array = contour_array->next;
        i++;
      }
      triangulate_polygon(contour_count, contour_sizes, vertices,
                          &parsed->polygon_triangles);

      // we need to fix the indices as they are local to a polygon, but they now
      // point into the global coordintate array.
      for (S32 j = first_triangle_idx; j < parsed->polygon_triangles.len; j++) {
        parsed->polygon_triangles.data[j].a += first_coordinate_idx;
        parsed->polygon_triangles.data[j].b += first_coordinate_idx;
        parsed->polygon_triangles.data[j].c += first_coordinate_idx;
        assert(parsed->polygon_triangles.data[j].a > first_coordinate_idx &&
               parsed->polygon_triangles.data[j].a <
                   parsed->polygon_coords.len);
        assert(parsed->polygon_triangles.data[j].b > first_coordinate_idx &&
               parsed->polygon_triangles.data[j].b <
                   parsed->polygon_coords.len);
        assert(parsed->polygon_triangles.data[j].c > first_coordinate_idx &&
               parsed->polygon_triangles.data[j].c <
                   parsed->polygon_coords.len);
      }
    }

    if (string8_equals(geometry_type->text_value,
                       string8_from_c_string("MultiPolygon"))) {
      JsonNode *polygon = coordinates->children.first;
      ASSERT(polygon->type == JSON_ARRAY,
             "expected array for MultiPolygon coordinates\n")

      while (polygon != NULL) {
        S32 contour_count = polygon->children.length;
        S32 *contour_sizes =
            (S32 *)arena_alloc(scratch.arena, contour_count * (S32)sizeof(S32));
        Coord2 *vertices = &parsed->multi_polygon_coords
                                .data[parsed->multi_polygon_coords.len];
        S32 first_coordinate_idx = parsed->multi_polygon_coords.len;
        S32 first_triangle_idx = parsed->multi_polygon_triangles.len;

        // create emtpy slot at vertices[0] for triangulation
        Coord2ArrayPush(&parsed->multi_polygon_coords, (Coord2){0.f, 0.f});

        JsonNode *contour_array = polygon->children.first;
        S32 i = 0;
        while (contour_array != NULL) {
          ASSERT(contour_array->children.length > 1,
                 "invalid size for polygon contour: %d",
                 contour_array->children.length)
          contour_sizes[i] = contour_array->children.length - 1;
          contour_from_json_array(contour_array, &parsed->multi_polygon_coords);
          contour_array = contour_array->next;
          i++;
        }
        triangulate_polygon(contour_count, contour_sizes, vertices,
                            &parsed->multi_polygon_triangles);

        // we need to fix the indices as they are local to a polygon, but they
        // now point into the global coordintate array.
        for (S32 j = first_triangle_idx;
             j < parsed->multi_polygon_triangles.len; j++) {
          parsed->multi_polygon_triangles.data[j].a += first_coordinate_idx;
          parsed->multi_polygon_triangles.data[j].b += first_coordinate_idx;
          parsed->multi_polygon_triangles.data[j].c += first_coordinate_idx;
          assert(parsed->multi_polygon_triangles.data[j].a >
                     first_coordinate_idx &&
                 parsed->multi_polygon_triangles.data[j].a <
                     parsed->multi_polygon_coords.len);
          assert(parsed->multi_polygon_triangles.data[j].b >
                     first_coordinate_idx &&
                 parsed->multi_polygon_triangles.data[j].b <
                     parsed->multi_polygon_coords.len);
          assert(parsed->multi_polygon_triangles.data[j].c >
                     first_coordinate_idx &&
                 parsed->multi_polygon_triangles.data[j].c <
                     parsed->multi_polygon_coords.len);
        }
        polygon = polygon->next;
      }
    }

    current_child = current_child->next;
  }

  temp_arena_memory_end(scratch);
  return parsed;
}

Geo_Json *geo_json_parse(Arena *arena, char *filepath) {
  FILE *f = fopen(filepath, "r");
  if (f == NULL) {
    ERROR_MSG("could not open file: %s", filepath)
  }

  Temp_Arena_Memory scratch = GetScratchConflict(&arena, 1);
  fseek(f, 0, SEEK_END);
  size_t fsize = (size_t)ftell(f);
  fseek(f, 0, SEEK_SET); /* same as rewind(f); */

  char *buffer = arena_alloc(scratch.arena, fsize + 1);
  fread(buffer, fsize, 1, f);
  fclose(f);

  JsonNode js = {0};
  parse_value(arena, &js, (string8){0}, buffer);
  Geo_Json *serialized = serialize(arena, &js);

  temp_arena_memory_end(scratch);
  return serialized;
}

void draw_points(Geo_Json *coords, Camera2D camera) {
  // --------------------- POINTS -------------------
  PointArray ips = coords->interest_points;
  for (S32 i = 0; i < ips.len; i++) {
    Vector2 a =
        GetWorldToScreen2D(Vector2FromCoord2(ips.data[i].coordinates), camera);
    DrawCircleV(a, 5.0f, RED);
  }
}
void draw_multi_points(Geo_Json *coords, Camera2D camera) {
  // --------------------- MULTI-POINTS -------------------
  MultiPointArray points = coords->multi_points;
  Coord2Array m_coords = coords->multi_point_coords;
  for (S32 i = 0; i < points.len; i++) {
    const S32 start_index = points.data[i].coordinates.start;
    const S32 length = points.data[i].coordinates.length;
    for (S32 j = 0; j < length; j++) {
      Vector2 a = GetWorldToScreen2D(
          Vector2FromCoord2(m_coords.data[start_index + j]), camera);
      DrawCircleV(a, 5.0f, RED);
      // DEBUG("drawing: [%03.05f, %03.05f]\n", a.x, a.y)
    }
  }
}
void draw_line_strings(Geo_Json *coords, Camera2D camera,
                       int show_node_endpoints) {
  // --------------------- LINE-STRINGS -------------------
  LineStringArray lines = coords->line_strings;
  Coord2Array l_coords = coords->line_string_coords;
  for (S32 i = 0; i < lines.len; i++) {
    const S32 start_index = lines.data[i].coordinates.start;
    const S32 length = lines.data[i].coordinates.length;
    for (S32 j = 0; j < length - 1; j++) {
      Vector2 a = GetWorldToScreen2D(
          Vector2FromCoord2(l_coords.data[start_index + j]), camera);
      Vector2 b = GetWorldToScreen2D(
          Vector2FromCoord2(l_coords.data[start_index + j + 1]), camera);
      // TODO: add cohen-sutherland clipping here
      DrawLineEx(a, b, 5.0f, BLUE);
      // DEBUG("drawing: [%03.05f, %03.05f] -> [%03.05f, %03.05f]\n", a.x,
      // a.y, b.x, b.y)
      if (show_node_endpoints) {
        DrawCircleV(a, 5.0f, RED);
        DrawCircleV(b, 5.0f, GREEN);
      }
    }
  }
}
void draw_multi_line_strings(Geo_Json *_coords, Camera2D _camera) {
  // -------------------- MUTLI LINE STRINGS -----------------------
}

void draw_polygons(Geo_Json *coords, Camera2D camera) {
  // --------------------- POLYGONS ---------------------
  TriangleArray triangles = coords->polygon_triangles;
  Coord2Array p_coords = coords->polygon_coords;
  for (S32 i = 0; i < triangles.len; i++) {
    Triangle t = triangles.data[i];
    Vector2 a =
        GetWorldToScreen2D(Vector2FromCoord2(p_coords.data[t.a]), camera);
    Vector2 b =
        GetWorldToScreen2D(Vector2FromCoord2(p_coords.data[t.b]), camera);
    Vector2 c =
        GetWorldToScreen2D(Vector2FromCoord2(p_coords.data[t.c]), camera);

    // we need to specify the triangle in couter-clockwise order but because we
    // reversed the coords the output we get is in clockwise order, therefore
    // change b, and c.
    DrawTriangle(a, c, b, BLUE);
    DEBUG_MSG("drawing triangle: [%03.05f, %03.05f][%03.05f, %03.05f][%03.05f, "
              "%03.05f]\n",
              a.x, a.y, b.x, b.y, c.x, c.y)
  }
}

void draw_multi_polygons(Geo_Json *coords, Camera2D camera) {
  // --------------------- MUTLI POLYGONS ---------------------
  TriangleArray triangles = coords->multi_polygon_triangles;
  Coord2Array p_coords = coords->multi_polygon_coords;
  for (S32 i = 0; i < triangles.len; i++) {
    Triangle t = triangles.data[i];
    Vector2 a =
        GetWorldToScreen2D(Vector2FromCoord2(p_coords.data[t.a]), camera);
    Vector2 b =
        GetWorldToScreen2D(Vector2FromCoord2(p_coords.data[t.b]), camera);
    Vector2 c =
        GetWorldToScreen2D(Vector2FromCoord2(p_coords.data[t.c]), camera);

    // we need to specify the triangle in couter-clockwise order but because we
    // reversed the coords the output we get is in clockwise order, therefore
    // change b, and c.
    DrawTriangle(a, c, b, BLUE);
    DEBUG_MSG("drawing triangle: [%03.05f, %03.05f][%03.05f, %03.05f][%03.05f, "
              "%03.05f]\n",
              a.x, a.y, b.x, b.y, c.x, c.y)
  }
}

void usage(char *program_name) {
  printf("usage: %s <filepath>\n", program_name);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }
  const U64 backing_buffer_size = 1024 * 1024 * 1024;
  void *backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                              MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  arena_init(arenas[0], backing_buffer, backing_buffer_size);

  backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  arena_init(arenas[1], backing_buffer, backing_buffer_size);
  Arena *arena = GetScratch().arena;
  Geo_Json *serialized_coords = geo_json_parse(arena, argv[1]);

  //--------------------------------------------------------------------------------------
  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 1920;
  const int screenHeight = 1080;

  InitWindow(screenWidth, screenHeight, "raylib [core] example - 2d camera");
  SetTargetFPS(60);

  int zoom_mode = 0;
  int show_node_endpoints = 0;
  Camera2D camera = {0};
  camera.offset =
      (Vector2){(float)screenWidth / 2.0f, (float)screenHeight / 2.0f};
  camera.rotation = 0.0f;
  camera.zoom = 4.0f;

  {
    LineStringArray lines = serialized_coords->line_strings;
    Coord2Array l_coords = serialized_coords->line_string_coords;
    for (S32 i = 0; i < lines.len; i++) {
      const S32 start_index = lines.data[i].coordinates.start;
      const S32 length = lines.data[i].coordinates.length;
      for (S32 j = 0; j < length - 1; j++) {
        DEBUG_MSG("line: [%03.05f, %03.05f] -> [%03.05f, %03.05f]\n",
                  l_coords.data[start_index + j].x * ((F64)screenWidth / 180.0),
                  (F32)(l_coords.data[start_index + j].y) *
                      ((F64)screenHeight / 90.0),
                  (F32)(l_coords.data[start_index + j + 1].x *
                        ((F64)screenWidth / 180.0)),
                  (F32)(l_coords.data[start_index + j + 1].y) *
                      ((F64)screenHeight / 90.0))
      }
    }
  }
  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------

    if (IsKeyPressed(KEY_ONE))
      zoom_mode = 0;
    else if (IsKeyPressed(KEY_TWO))
      zoom_mode = 1;
    if (IsKeyPressed(KEY_NINE))
      show_node_endpoints = 0;
    if (IsKeyPressed(KEY_ZERO))
      show_node_endpoints = 1;

    // Translate based on mouse right click
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      Vector2 delta = GetMouseDelta();
      delta = Vector2Scale(delta, -1.0f / camera.zoom);
      camera.target = Vector2Add(camera.target, delta);
    }

    if (zoom_mode == 0) {
      // Zoom based on mouse wheel
      float wheel = GetMouseWheelMove();
      if (wheel != 0) {
        // Get the world point that is under the mouse
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);

        // Set the offset to where the mouse is
        camera.offset = GetMousePosition();

        // Set the target to match, so that the camera maps the world space
        // point under the cursor to the screen space point under the cursor at
        // any zoom
        camera.target = mouseWorldPos;

        // Zoom increment
        // Uses log scaling to provide consistent zoom speed
        float scale = 0.2f * wheel;
        camera.zoom = Clamp(expf(logf(camera.zoom) + scale), 0.001f, 100000.0f);
      }
    } else {
      // Zoom based on mouse right click
      if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        // Get the world point that is under the mouse
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);

        // Set the offset to where the mouse is
        camera.offset = GetMousePosition();

        // Set the target to match, so that the camera maps the world space
        // point under the cursor to the screen space point under the cursor at
        // any zoom
        camera.target = mouseWorldPos;
      }

      if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        // Zoom increment
        // Uses log scaling to provide consistent zoom speed
        float deltaX = GetMouseDelta().x;
        float scale = 0.005f * deltaX;
        camera.zoom = Clamp(expf(logf(camera.zoom) + scale), 0.001f, 100000.0f);
      }
    }

    // Camera reset (zoom and rotation)
    if (IsKeyPressed(KEY_R)) {
      camera.zoom = 4.0f;
      camera.rotation = 0.0f;
      camera.offset =
          (Vector2){(float)screenWidth / 2.0f, (float)screenHeight / 2.0f};
      camera.target.x = 0;
      camera.target.y = 0;
    }
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(RAYWHITE);

    draw_points(serialized_coords, camera);
    draw_multi_points(serialized_coords, camera);
    draw_line_strings(serialized_coords, camera, show_node_endpoints);
    draw_multi_line_strings(serialized_coords, camera);
    draw_polygons(serialized_coords, camera);
    draw_multi_polygons(serialized_coords, camera);

    // --------------------- HUD -------------------------
    DrawText(TextFormat("CURRENT ZOOM: %03.04f", camera.zoom), 640, 10, 20,
             RED);
    DrawText(TextFormat("CAMERA TARGET: [%03.04f, %03.04f]", camera.target.x,
                        camera.target.y),
             640, 40, 20, RED);

    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------
  return 0;
}
