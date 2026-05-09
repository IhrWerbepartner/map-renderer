#include "arena.c"
#include "fixed-array.c"
#include "string8.c"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <raylib.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#define ERROR_MSG(msg)                                                         \
  fprintf(stderr, "ERROR: %s\n", msg);                                         \
  exit(EXIT_FAILURE);

typedef struct Point2D {
  double x;
  double y;
} Point2D;

DeclFixedArray(Point2DArray, Point2D)

    typedef struct geo_properties {
  string8 key;
  void *val;
} GeoProperties;

typedef struct Point {
  Point2D coordinates;
} Point;

typedef struct multi_point {
  Point2DArray coordinates;
} MultiPoint;

typedef struct line_string {
  Point2DArray coordinates;
} LineString;

DeclFixedArray(LineStringArray, LineString)

    typedef struct multi_line_string {
  LineStringArray coordinates;
} MultiLineString;

typedef struct polygon {
  Point2DArray outside_coordinates;
  Point2DArray inside_coordinates;
} Polygon;

DeclFixedArray(PolygonArray, Polygon)

    typedef struct multi_polygon {
  PolygonArray coordinates;
} MultiPolygon;

DeclFixedArray(PointArray, Point) DeclFixedArray(MultiPointArray, MultiPoint)
    DeclFixedArray(MultiLineStringArray, MultiLineString)
        DeclFixedArray(GeoPropertiesArray, GeoProperties)
            DeclFixedArray(MultiPolygonArray, MultiPolygon)

                typedef struct geo_json {
  enum geo_json_type {
    FEATURE_COLLECTION,
  } type;

  // SoA layout
  PointArray points;
  MultiPointArray multi_points;
  LineStringArray line_strings;
  MultiLineStringArray multi_line_strings;
  PolygonArray polygons;
  MultiPolygonArray multi_polygons;

  GeoPropertiesArray properties; // TODO: add a handle in the types for as an
                                 // index in to the properties array
} Geo_Json;

void init_all_arrays(Arena *arena, Geo_Json *base, size_t capacity) {
  base->points = PointArray_new(arena, capacity);
  base->multi_points = MultiPointArray_new(arena, capacity);
  base->line_strings = LineStringArray_new(arena, capacity);
  base->multi_line_strings = MultiLineStringArray_new(arena, capacity);
  base->polygons = PolygonArray_new(arena, capacity);
  base->multi_polygons = MultiPolygonArray_new(arena, capacity);
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
        uint64_t u_value; // the value of INTEGER or BOOL node
        int64_t s_value;
      };
      double dbl_value; // the value of DOUBLE node
    } num;
    struct { // children of OBJECT or ARRAY
      int length;
      struct JsonNode *first;
      struct JsonNode *last;
    } children;
  };
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
      return from_c_string_len(s, (size_t)(p - s));
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
          js->num.dbl_value = (double)js->num.s_value;
        } else {
          js->num.dbl_value = (double)js->num.u_value;
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
  while (current->next != NULL) {
    current = current->next;
    if (current->type == JSON_STRING) {
      if (string8_equals(current->key, key) &&
          string8_equals(current->text_value, value)) {
        return current;
      }
    }
  }
  return NULL;
}

// locates a child node with given key
JsonNode *find_key_in_children(JsonNode *node, string8 key) {
  JsonNode *current = node->children.first;
  while (current->next != NULL) {
    current = current->next;
    if (string8_equals(current->key, key)) {
      return current;
    }
  }
  return NULL;
}

Point serialize_point(JsonNode *coordinates) {
  if (coordinates->type != JSON_ARRAY || coordinates->children.length != 2) {
    ERROR_MSG("invalid coordinates for Point supplied")
  }
  JsonNode *x_coordinate = coordinates->children.first;
  JsonNode *y_coordinate = coordinates->children.last;
  if (x_coordinate->type != JSON_DOUBLE || y_coordinate->type != JSON_DOUBLE) {
    ERROR_MSG("invalid coordinate type for Point supplied")
  }
  return (Point){.coordinates = (Point2D){
                     .x = x_coordinate->num.dbl_value,
                     .y = y_coordinate->num.dbl_value,
                 }};
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
  JsonNode *const featureCollcetionType = find_key_value_in_children(
      root, from_c_string("type"), from_c_string("FeatureCollection"));
  if (featureCollcetionType == NULL) {
    ERROR_MSG("invalid geojson type")
  }
  JsonNode *const features =
      find_key_in_children(root, from_c_string("features"));
  if (features == NULL || features->type != JSON_ARRAY) {
    ERROR_MSG("invalid geojson features")
  }
  init_all_arrays(arena, parsed, features->children.length);
  JsonNode *current_child = features->children.first;

  // parese all "Features" in their appropiate arrays
  while (current_child != NULL) {
    JsonNode *const feature_type = find_key_value_in_children(
        current_child, from_c_string("type"), from_c_string("Feature"));
    if (feature_type == NULL) {
      ERROR_MSG("invalid feature type")
    }

    JsonNode *const geometry =
        find_key_in_children(current_child, from_c_string("geometry"));
    if (geometry == NULL || geometry->type != JSON_OBJECT) {
      ERROR_MSG("invalid or no geometry suppiled")
    }

    JsonNode *const geometry_type =
        find_key_in_children(geometry, from_c_string("type"));
    if (geometry_type == NULL || geometry->type != JSON_STRING) {
      ERROR_MSG("no geometry type suppiled")
    }

    JsonNode *const coordinates =
        find_key_in_children(geometry, from_c_string("coordinates"));
    if (coordinates == NULL || coordinates->type != JSON_ARRAY) {
      ERROR_MSG("no coordinates suppiled")
    }

    // "type" : "Point" parsing
    if (string8_equals(geometry_type->text_value, from_c_string("Point"))) {
      PointArray_push(&parsed->points, serialize_point(coordinates));
    }

    // TODO:
    if (string8_equals(geometry_type->text_value,
                       from_c_string("MultiPoint"))) {
    }
    if (string8_equals(geometry_type->text_value,
                       from_c_string("LineString"))) {
    }
    if (string8_equals(geometry_type->text_value,
                       from_c_string("MultiLineString"))) {
    }
    if (string8_equals(geometry_type->text_value, from_c_string("Polygon"))) {
    }
    if (string8_equals(geometry_type->text_value,
                       from_c_string("MultiPolygon"))) {
    }

    current_child = current_child->next;
  }

  return (Geo_Json *)0; // TODO:
}

Geo_Json *geo_json_parse(Arena *arena, char *filepath) {
  FILE *f = fopen(filepath, "r");
  if (f == NULL) {
    ERROR_MSG("could not open file")
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

void usage(char *program_name) {
  printf("usage: %s <filepath>\n", program_name);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }
  size_t backing_buffer_size = 1024 * 1024 * 1024;
  void *backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                              MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  arena_init(arenas[0], backing_buffer, backing_buffer_size);

  backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  arena_init(arenas[1], backing_buffer, backing_buffer_size);
  Geo_Json *json = geo_json_parse(GetScratch().arena, argv[1]);

  //--------------------------------------------------------------------------------------
  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 1920;
  const int screenHeight = 1080;

  InitWindow(screenWidth, screenHeight, "raylib [core] example - 2d camera");

  Rectangle player = {400, 280, 40, 40};

  Camera2D camera = {0};
  camera.target = (Vector2){player.x, player.y};
  camera.offset =
      (Vector2){(float)screenWidth / 2.0f, (float)screenHeight / 2.0f};
  camera.rotation = 0.0f;
  camera.zoom = 1.0f;

  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    // Player movement
    if (IsKeyDown(KEY_RIGHT))
      player.x += 2;
    else if (IsKeyDown(KEY_LEFT))
      player.x -= 2;

    // Camera target follows player
    camera.target = (Vector2){player.x, player.y};

    // Camera rotation controls
    if (IsKeyDown(KEY_A))
      camera.rotation--;
    else if (IsKeyDown(KEY_S))
      camera.rotation++;

    // Limit camera rotation to 80 degrees (-40 to 40)
    if (camera.rotation > 40)
      camera.rotation = 40;
    else if (camera.rotation < -40)
      camera.rotation = -40;

    // Camera zoom controls
    // Uses log scaling to provide consistent zoom speed
    camera.zoom = expf(logf(camera.zoom) + ((float)GetMouseWheelMove() * 0.1f));

    if (camera.zoom > 3.0f)
      camera.zoom = 3.0f;
    else if (camera.zoom < 0.1f)
      camera.zoom = 0.1f;

    // Camera reset (zoom and rotation)
    if (IsKeyPressed(KEY_R)) {
      camera.zoom = 1.0f;
      camera.rotation = 0.0f;
    }
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(RAYWHITE);

    BeginMode2D(camera);

    DrawRectangle(-6000, 320, 13000, 8000, DARKGRAY);

    DrawRectangleRec(player, RED);

    EndMode2D();

    DrawText("SCREEN AREA", 640, 10, 20, RED);

    DrawRectangle(0, 0, screenWidth, 5, RED);
    DrawRectangle(0, 5, 5, screenHeight - 10, RED);
    DrawRectangle(screenWidth - 5, 5, 5, screenHeight - 10, RED);
    DrawRectangle(0, screenHeight - 5, screenWidth, 5, RED);

    DrawRectangle(10, 10, 250, 113, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(10, 10, 250, 113, BLUE);

    DrawText("Free 2D camera controls:", 20, 20, 10, BLACK);
    DrawText("- Right/Left to move player", 40, 40, 10, DARKGRAY);
    DrawText("- Mouse Wheel to Zoom in-out", 40, 60, 10, DARKGRAY);
    DrawText("- A / S to Rotate", 40, 80, 10, DARKGRAY);
    DrawText("- R to reset Zoom and Rotation", 40, 100, 10, DARKGRAY);

    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------
  return 0;
}
