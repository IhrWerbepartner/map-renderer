#include "base.h"
#include "earcut.h"
#include "raymath.h"
#include "string8.c"
#include "tessalate.h"
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <raylib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define SEIDEL_TRIANGULATION 0

#ifdef _WIN32
#include <memoryapi.h>
#else
#include <sys/mman.h>
#endif

typedef struct geo_properties {
  String8 key;
  void *val;
} GeoProperties;

typedef struct Point {
  Coord2 coordinates;
} Point;

typedef struct multi_point {
  Slice coordinates;
} MultiPoint;

typedef struct line_string {
  Slice coordinates;
} LineString;

DeclFixedArray(LineStringArray, LineString);

typedef struct multi_line_string {
  Slice lines;
} MultiLineString;

typedef struct contour {
  Slice coords;
} Contour;

DeclFixedArray(PolygonArray, Polygon);

typedef struct multi_polygon {
  Slice polygons;
} MultiPolygon;

DeclFixedArray(PointArray, Point);
DeclFixedArray(MultiPointArray, MultiPoint);
DeclFixedArray(MultiLineStringArray, MultiLineString);
DeclFixedArray(GeoPropertiesArray, GeoProperties);
DeclFixedArray(MultiPolygonArray, MultiPolygon);

typedef struct geo_json {
  enum geo_json_type {
    FEATURE_COLLECTION,
  } type;

  // SoA layout
  PointArray interest_points; // all interest points
                              //
  Coord2Array multi_point_coords;
  MultiPointArray multi_points; // has a slice into coords

  Coord2Array line_string_coords;
  LineStringArray line_strings; // has a slice into coords

  Coord2Array multi_line_string_coords;
  LineStringArray multi_line_string_array; // has a slice into coords
  MultiLineStringArray multi_line_strings; // has an array of lineStrings

  Coord2Array polygon_coords;
  TriangleArray polygon_triangles; // triangle indices point into polygon_coords

  Coord2Array multi_polygon_coords;
  TriangleArray multi_polygon_triangles; // has a slice into coords
  MultiPolygonArray
      multi_polygons; // has an array of slices into array of triangles

  GeoPropertiesArray properties; // TODO: add a handle in the types for as an
                                 // index in to the properties array
} GeoJson;

typedef struct RenderOptions RenderOptions;
struct RenderOptions {
  bool show_node_endpoints;
  bool show_triangulation;
};

static RenderOptions render_options = {0};

void init_all_arrays(Arena *arena, GeoJson *base, S32 capacity) {
  base->interest_points = PointArrayNew(arena, 100000);

  base->multi_point_coords = Coord2ArrayNew(arena, 100000);
  base->multi_points = MultiPointArrayNew(arena, 10000);

  base->line_string_coords = Coord2ArrayNew(arena, 10000000);
  base->line_strings = LineStringArrayNew(arena, 1000000);

  base->multi_line_string_coords = Coord2ArrayNew(arena, 10000000);
  base->multi_line_string_array = LineStringArrayNew(arena, 100000);
  base->multi_line_strings = MultiLineStringArrayNew(arena, 100000);

  base->polygon_coords = Coord2ArrayNew(arena, capacity);
  base->polygon_triangles = TriangleArrayNew(arena, capacity);

  base->multi_polygon_coords = Coord2ArrayNew(arena, capacity);
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

#define JSON_REPORT_ERROR(msg, p)                                              \
  fprintf(stderr, "NXJSON PARSE ERROR (%d): " msg " at %s\n", __LINE__, p)

#define IS_WHITESPACE(c) ((unsigned char)(c) <= (unsigned char)' ')

static JsonNode *create_json(Arena *arena, JsonType type, String8 key,
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
  parent->children.count++;
  return js;
}

static String8 unescape_string(char *s, char **end) {
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

static char *parse_key(String8 *key, char *p) {
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

static char *parse_value(Arena *arena, JsonNode *parent, String8 key, char *p) {
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
        String8 new_key = {0};
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
        p = parse_value(arena, js, (String8){0}, p);
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

const JsonNode *json_get(const JsonNode *json, String8 key) {
  for (JsonNode *js = json->children.first; js; js = js->next) {
    if (js->key.buf && !string8_compare(js->key, key))
      return js;
  }
  return NULL;
}

const JsonNode *json_item(const JsonNode *json, int idx) {
  for (JsonNode *js = json->children.first; js; js = js->next) {
    if (!idx--)
      return js;
  }
  return NULL;
}

// serialization helpers
//
//

// locates a child node with given key, value pair
JsonNode *find_key_value_in_children(const JsonNode *node, const String8 key,
                                     const String8 value) {
  JsonNode *current = node->children.first;
  while (current != NULL) {
    if (current->type == JSON_STRING) {
      if (String8Equals(current->key, key) &&
          String8Equals(current->text_value, value)) {
        return current;
      }
    }
    current = current->next;
  }
  return NULL;
}

// locates a child node with given key
JsonNode *find_key_in_children(const JsonNode *node, String8 key) {
  JsonNode *current = node->children.first;
  while (current != NULL) {
    if (String8Equals(current->key, key)) {
      return current;
    }
    current = current->next;
  }
  return NULL;
}

Coord2 Coord2FromJsonArrayNode(const JsonNode *coordinates) {
  if (coordinates->type != JSON_ARRAY) {
    ERROR_MSG("invalid coordinate type for Point supplied")
  }
  if (coordinates->children.count != 2) {
    ERROR_MSG("Map rendrer only suppords 2D coords: was %d\n",
              coordinates->children.count)
  }
  F64 x;
  F64 y;
  const JsonNode *x_coordinate = coordinates->children.first;
  const JsonNode *y_coordinate = coordinates->children.last;
  if (x_coordinate->type == JSON_DOUBLE) {
    x = x_coordinate->num.dbl_value;
  } else if (x_coordinate->type == JSON_INTEGER) {
    x = (F64)x_coordinate->num.s_value;
  } else {
    ERROR_MSG("invalid coordinate type for Point supplied")
  }
  if (y_coordinate->type == JSON_DOUBLE) {
    y = y_coordinate->num.dbl_value;
  } else if (y_coordinate->type == JSON_INTEGER) {
    y = (F64)y_coordinate->num.s_value;
  } else {
    ERROR_MSG("invalid coordinate type for Point supplied")
  }
  return (Coord2){.x = x, .y = y};
}

// iterate over the coordinates omitting the last one as it is
// identical to the first. input: [a, b, c, d, e] puts [a, b, c, d] into the
// result array. Returns the index of the coordinate with the lowest y and
// hightest x.
S32 ContourFromJsonArray(const JsonNode *coordinates,
                         Coord2Array *result_array) {
  if (coordinates->type != JSON_ARRAY) {
    ERROR_MSG("invalid coordinates node type")
  }
  if (coordinates->children.count <= 2) {
    ERROR_MSG("invalid contour with: %d coordinates\n",
              coordinates->children.count)
  }
  S32 min_index = 0;
  Coord2 min_coordinate =
      (Coord2){-(TRIANGULATE_INFINITY), TRIANGULATE_INFINITY};
  const JsonNode *point_coords = coordinates->children.first;
  S32 index = 0;
  while (point_coords != NULL && point_coords != coordinates->children.last) {
    const Coord2 coordinate = Coord2FromJsonArrayNode(point_coords);
    Coord2ArrayPush(result_array, coordinate);
    if (coordinate.y < min_coordinate.y ||
        (FP_EQUAL(coordinate.y, min_coordinate.y) &&
         coordinate.x > min_coordinate.x)) {
      min_index = index;
      min_coordinate = coordinate;
    }
    index += 1;
    point_coords = point_coords->next;
  }
  ASSERT(min_index >= 0 && min_index < coordinates->children.count,
         "Min index %d out of range (%d..%d)", min_index, 0,
         coordinates->children.count)
  return min_index;
}

// reverses the coordinates to make them counterclockwise if needed.
// Returns true/false depending on if this function it did some operation.
bool MakeCoordinatesCounterClockwise(Coord2Slice coordinates, S32 min_index) {
  ASSERT(min_index >= 0 && min_index < coordinates.count, "index out of range");
  if (CROSS(coordinates.v[min_index],
            coordinates
                .v[(min_index + coordinates.count - 1) % coordinates.count],
            coordinates.v[(min_index + 1) % coordinates.count]) > 0) {
    for (S32 i = 0; i < coordinates.count / 2; i += 1) {
      const S32 opposite_index = coordinates.count - i - 1;
      const Coord2 tmp = coordinates.v[i];
      coordinates.v[i] = coordinates.v[opposite_index];
      coordinates.v[opposite_index] = tmp;
    }
    return true;
  }
  return false;
}

// iterate over the coordinates reversed omitting the first one as it is
// identical to the last. input: [a, b, c, d, e] returns [e, d, c, b]
// this procedure is used to turn a reversee the clockwise direction of a
// contour
void ContourFromJsonArrayReversed(const JsonNode *coordinates,
                                  Coord2Array *result_array) {
  if (coordinates->type != JSON_ARRAY) {
    ERROR_MSG("invalid coordinates node type")
  }
  if (coordinates->children.count <= 2) {
    ERROR_MSG("invalid contour with: %d coordinates\n",
              coordinates->children.count)
  }
  const JsonNode *point_coords = coordinates->children.last;
  while (point_coords != NULL && point_coords != coordinates->children.first) {
    Coord2ArrayPush(result_array, Coord2FromJsonArrayNode(point_coords));
    point_coords = point_coords->prev;
  }
}

void Coord2ArrayFromJsonArray(const JsonNode *coordinates,
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

GeoJson *serialize(Arena *arena, JsonNode *root) {
  GeoJson *render_data = (GeoJson *)arena_alloc(arena, sizeof(GeoJson));
  if (root->type != JSON_NULL || root->children.count != 1) {
    ERROR_MSG("invalid json")
  }
  root = root->children.first;
  if (root->type != JSON_OBJECT) {
    ERROR_MSG("invalid object")
  }
  JsonNode *const featureCollectionType =
      find_key_value_in_children(root, String8FromCString("type"),
                                 String8FromCString("FeatureCollection"));
  if (featureCollectionType == NULL) {
    ERROR_MSG("invalid geojson type")
  }
  JsonNode *const features =
      find_key_in_children(root, String8FromCString("features"));
  if (features == NULL || features->type != JSON_ARRAY) {
    ERROR_MSG("invalid geojson features")
  }
  // init_all_arrays(arena, parsed, features->children.length); TODO:
  init_all_arrays(arena, render_data, 1000 * 1000 * 10);
  JsonNode *current_child = features->children.first;

  // parse all "Features" in their appropriate arrays
  Temp_Arena_Memory scratch = GetScratchConflict(&arena, 1);
  while (current_child != NULL) {
    JsonNode *const feature_type =
        find_key_value_in_children(current_child, String8FromCString("type"),
                                   String8FromCString("Feature"));
    if (feature_type == NULL) {
      ERROR_MSG("invalid feature type")
    }

    JsonNode *const geometry =
        find_key_in_children(current_child, String8FromCString("geometry"));
    if (geometry == NULL || geometry->type != JSON_OBJECT) {
      ERROR_MSG("invalid or no geometry supplied")
    }

    JsonNode *const geometry_type =
        find_key_in_children(geometry, String8FromCString("type"));
    if (geometry_type == NULL || geometry_type->type != JSON_STRING) {
      ERROR_MSG("no geometry type supplied")
    }

    JsonNode *const coordinates =
        find_key_in_children(geometry, String8FromCString("coordinates"));
    if (coordinates == NULL || coordinates->type != JSON_ARRAY) {
      ERROR_MSG("no coordinates supplied")
    }

    // "type" : "Point" parsing
    if (String8Equals(geometry_type->text_value, String8FromCString("Point"))) {
      PointArrayPush(
          &render_data->interest_points,
          (Point){.coordinates = Coord2FromJsonArrayNode(coordinates)});
    }

    if (String8Equals(geometry_type->text_value,
                      String8FromCString("MultiPoint"))) {
      MultiPoint multi_point = (MultiPoint){
          .coordinates = (Slice){.start = render_data->multi_points.count,
                                 .length = coordinates->children.count},
      };
      Coord2ArrayFromJsonArray(coordinates, &render_data->multi_point_coords);
      // assert that we inserted the correct amount of points
      assert(multi_point.coordinates.start + multi_point.coordinates.length ==
             render_data->multi_point_coords.count);
      MultiPointArrayPush(&render_data->multi_points, multi_point);
    }

    if (String8Equals(geometry_type->text_value,
                      String8FromCString("LineString"))) {
      LineString line_string = (LineString){
          .coordinates = (Slice){.start = render_data->line_string_coords.count,
                                 .length = coordinates->children.count},
      };
      Coord2ArrayFromJsonArray(coordinates, &render_data->line_string_coords);
      ASSERT(line_string.coordinates.start + line_string.coordinates.length ==
                 render_data->line_string_coords.count,
             "assert failed at line: %d", __LINE__)
      LineStringArrayPush(&render_data->line_strings, line_string);
    }

    if (String8Equals(geometry_type->text_value,
                      String8FromCString("MultiLineString"))) {
      JsonNode *line_string = coordinates->children.first;
      MultiLineString mls = (MultiLineString){
          .lines = (Slice){.start = render_data->multi_line_string_array.count,
                           .length = coordinates->children.count},
      };
      while (line_string != NULL) {
        LineString ls = (LineString){
            .coordinates =
                (Slice){.start = render_data->multi_line_string_coords.count,
                        .length = line_string->children.count},
        };
        Coord2ArrayFromJsonArray(line_string,
                                 &render_data->multi_line_string_coords);
        ASSERT(ls.coordinates.start + ls.coordinates.length ==
                   render_data->multi_line_string_coords.count,
               "assert failed at line: %d", __LINE__)
        LineStringArrayPush(&render_data->multi_line_string_array, ls);
        line_string = line_string->next;
      }
      MultiLineStringArrayPush(&render_data->multi_line_strings, mls);
    }

    if (String8Equals(geometry_type->text_value,
                      String8FromCString("Polygon"))) {
      S32 contour_count = coordinates->children.count;
      S32Array contour_sizes = S32ArrayNew(scratch.arena, contour_count);
      Coord2 *vertices =
          &render_data->polygon_coords.d[render_data->polygon_coords.count];
      S32 first_coordinate_idx = render_data->polygon_coords.count;
      S32 first_triangle_idx = render_data->polygon_triangles.count;

      // create emtpy slot at vertices[0] for triangulation
      Coord2ArrayPush(&render_data->polygon_coords, (Coord2){0.f, 0.f});

#if SEIDEL_TRIANGULATION
      JsonNode *contour_array = coordinates->children.first;
      if (contour_array == NULL || contour_array->children.count <= 1) {
        ERROR_MSG("invalid size for polygon contour: %d",
                  contour_array->children.count)
      }
      const S32 min_coordinate_index =
          ContourFromJsonArray(contour_array, &render_data->polygon_coords);
      S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
      bool outerContourIsClockwise = MakeCoordinatesCounterClockwise(
          Coord2SliceFromArrayExt(&render_data->polygon_coords,
                                  first_coordinate_idx + 1,
                                  contour_array->children.count - 1),
          min_coordinate_index);
      contour_array = contour_array->next;
      while (contour_array != NULL) {
        S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
        if (outerContourIsClockwise) {
          ContourFromJsonArrayReversed(contour_array,
                                       &render_data->polygon_coords);
        } else {
          ContourFromJsonArray(contour_array, &render_data->polygon_coords);
        }
        contour_array = contour_array->next;
      }

      TessalatePolygon(
          &render_data->polygon_triangles,
          (Coord2Slice){.v = vertices,
                        .count = render_data->polygon_coords.count -
                                 first_coordinate_idx},
          S32SliceFromArray(&contour_sizes));
#else
      JsonNode *contour_array = coordinates->children.first;
      if (contour_array == NULL || contour_array->children.count <= 1) {
        ERROR_MSG("invalid size for polygon contour: %d",
                  contour_array->children.count)
      }
      ContourFromJsonArray(contour_array, &render_data->polygon_coords);
      S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
      contour_array = contour_array->next;
      while (contour_array != NULL) {
        S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
        ContourFromJsonArray(contour_array, &render_data->polygon_coords);
        contour_array = contour_array->next;
      }
      Earcut(&render_data->polygon_triangles,
             (Coord2Slice){.v = vertices,
                           .count = render_data->polygon_coords.count -
                                    first_coordinate_idx},
             S32SliceFromArray(&contour_sizes));

#endif

      // we need to fix the indices as they are local to a polygon, but they
      // now point into the global coordinate array.
      for (S32 j = first_triangle_idx; j < render_data->polygon_triangles.count;
           j++) {
        Triangle *triangle = &render_data->polygon_triangles.d[j];
        triangle->a += first_coordinate_idx;
        triangle->b += first_coordinate_idx;
        triangle->c += first_coordinate_idx;
        assert(triangle->a > first_coordinate_idx &&
               triangle->a < render_data->polygon_coords.count);
        assert(triangle->b > first_coordinate_idx &&
               triangle->b < render_data->polygon_coords.count);
        assert(triangle->c > first_coordinate_idx);
        assert(triangle->c < render_data->polygon_coords.count);
      }
#if SEIDEL_TRIANGULATION
      for (S32 i = 0; i < render_data->polygon_triangles.count; i += 1) {
        Triangle t = render_data->polygon_triangles.d[i];
        ASSERT(CROSS(render_data->polygon_coords.d[t.a],
                     render_data->polygon_coords.d[t.b],
                     render_data->polygon_coords.d[t.c]) >= 0,
               "Triangle [%d, %d, %d] not counter-clockwise\n", t.a, t.b, t.c)
      }
#endif
    }

    if (String8Equals(geometry_type->text_value,
                      String8FromCString("MultiPolygon"))) {
      JsonNode *polygon = coordinates->children.first;
      ASSERT(polygon->type == JSON_ARRAY,
             "expected array for MultiPolygon coordinates\n")

      while (polygon != NULL) {
        S32 contour_count = polygon->children.count;
        S32Array contour_sizes = S32ArrayNew(scratch.arena, contour_count);
        Coord2 *vertices = &render_data->multi_polygon_coords
                                .d[render_data->multi_polygon_coords.count];
        S32 first_coordinate_idx = render_data->multi_polygon_coords.count;
        S32 first_triangle_idx = render_data->multi_polygon_triangles.count;

        // create emtpy slot at vertices[0] for triangulation
        Coord2ArrayPush(&render_data->multi_polygon_coords, (Coord2){0.f, 0.f});

#if SEIDEL_TRIANGULATION
        JsonNode *contour_array = polygon->children.first;
        const S32 min_coordinate_index = ContourFromJsonArray(
            contour_array, &render_data->multi_polygon_coords);
        S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
        bool outerContourIsClockwise = MakeCoordinatesCounterClockwise(
            Coord2SliceFromArrayExt(&render_data->multi_polygon_coords,
                                    first_coordinate_idx + 1,
                                    contour_array->children.count - 1),
            min_coordinate_index);
        DEBUG_MSG("min vertex: %d\n", min_coordinate_index + 1);
        contour_array = contour_array->next;
        while (contour_array != NULL) {
          ASSERT(contour_array->children.count > 1,
                 "invalid size for polygon contour: %d",
                 contour_array->children.count)
          S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
          if (outerContourIsClockwise) {
            ContourFromJsonArrayReversed(contour_array,
                                         &render_data->multi_polygon_coords);
          } else {
            ContourFromJsonArray(contour_array,
                                 &render_data->multi_polygon_coords);
          }
          contour_array = contour_array->next;
        }

        TessalatePolygon(
            &render_data->multi_polygon_triangles,
            (Coord2Slice){.v = vertices,
                          .count = render_data->multi_polygon_coords.count -
                                   first_coordinate_idx},
            S32SliceFromArray(&contour_sizes));
#else
        JsonNode *contour_array = polygon->children.first;
        const S32 min_coordinate_index = ContourFromJsonArray(
            contour_array, &render_data->multi_polygon_coords);
        S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
        DEBUG_MSG("min vertex: %d\n", min_coordinate_index + 1);
        contour_array = contour_array->next;
        while (contour_array != NULL) {
          ASSERT(contour_array->children.count > 1,
                 "invalid size for polygon contour: %d",
                 contour_array->children.count)
          S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
          ContourFromJsonArray(contour_array,
                               &render_data->multi_polygon_coords);
          contour_array = contour_array->next;
        }
        Earcut(&render_data->multi_polygon_triangles,
               (Coord2Slice){.v = vertices,
                             .count = render_data->multi_polygon_coords.count -
                                      first_coordinate_idx},
               S32SliceFromArray(&contour_sizes));

#endif

        // we need to fix the indices as they are local to a polygon, but they
        // now point into the global coordinate array.
        for (S32 j = first_triangle_idx;
             j < render_data->multi_polygon_triangles.count; j++) {
          Triangle *triangle = &render_data->multi_polygon_triangles.d[j];
          triangle->a += first_coordinate_idx;
          triangle->b += first_coordinate_idx;
          triangle->c += first_coordinate_idx;
          ASSERT(triangle->a > first_coordinate_idx &&
                     triangle->a < render_data->multi_polygon_coords.count,
                 "multi polygon index for a out of range, should be inside "
                 "(%d-%d), was: %d",
                 first_coordinate_idx, render_data->multi_polygon_coords.count,
                 triangle->a);
          ASSERT(triangle->b > first_coordinate_idx &&
                     triangle->b < render_data->multi_polygon_coords.count,
                 "multi polygon index for b out of range, should be inside "
                 "(%d-%d), was: %d",
                 first_coordinate_idx, render_data->multi_polygon_coords.count,
                 triangle->b);
          ASSERT(triangle->c > first_coordinate_idx &&
                     triangle->c < render_data->multi_polygon_coords.count,
                 "multi polygon index for c out of range, should be inside "
                 "(%d-%d), was: %d",
                 first_coordinate_idx, render_data->multi_polygon_coords.count,
                 triangle->c);
        }
        polygon = polygon->next;
      }
    }

    current_child = current_child->next;
  }

  temp_arena_memory_end(scratch);
  return render_data;
}

GeoJson *geo_json_parse(Arena *arena, char *filepath) {
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
  parse_value(arena, &js, (String8){0}, buffer);
  GeoJson *serialized = serialize(arena, &js);

  temp_arena_memory_end(scratch);
  return serialized;
}

void draw_points(const GeoJson *coords, Camera2D camera) {
  // --------------------- POINTS -------------------
  PointArray ips = coords->interest_points;
  for (S32 i = 0; i < ips.count; i++) {
    Vector2 a =
        GetWorldToScreen2D(Vector2FromCoord2(ips.d[i].coordinates), camera);
    DrawCircleV(a, 5.0f, RED);
  }
}
void draw_multi_points(const GeoJson *coords, Camera2D camera) {
  // --------------------- MULTI-POINTS -------------------
  MultiPointArray points = coords->multi_points;
  Coord2Array m_coords = coords->multi_point_coords;
  for (S32 i = 0; i < points.count; i++) {
    const S32 start_index = points.d[i].coordinates.start;
    const S32 length = points.d[i].coordinates.length;
    for (S32 j = 0; j < length; j++) {
      Vector2 a = GetWorldToScreen2D(
          Vector2FromCoord2(m_coords.d[start_index + j]), camera);
      DrawCircleV(a, 5.0f, RED);
      // DEBUG("drawing: [%03.05f, %03.05f]\n", a.x, a.y)
    }
  }
}
void draw_line_strings(const GeoJson *coords, Camera2D camera) {
  // --------------------- LINE-STRINGS -------------------
  LineStringArray lines = coords->line_strings;
  Coord2Array l_coords = coords->line_string_coords;
  for (S32 i = 0; i < lines.count; i++) {
    const S32 start_index = lines.d[i].coordinates.start;
    const S32 length = lines.d[i].coordinates.length;
    for (S32 j = 0; j < length - 1; j++) {
      Vector2 a = GetWorldToScreen2D(
          Vector2FromCoord2(l_coords.d[start_index + j]), camera);
      Vector2 b = GetWorldToScreen2D(
          Vector2FromCoord2(l_coords.d[start_index + j + 1]), camera);
      // TODO: add cohen-sutherland clipping here
      DrawLineEx(a, b, 5.0f, BLUE);
      // DEBUG("drawing: [%03.05f, %03.05f] -> [%03.05f, %03.05f]\n", a.x,
      // a.y, b.x, b.y)
      if (render_options.show_node_endpoints) {
        DrawCircleV(a, 5.0f, RED);
        if (j == length - 1) {
          DrawCircleV(b, 5.0f, GREEN);
        }
      }
    }
  }
}
void draw_multi_line_strings(GeoJson *coords, Camera2D camera) {
  // -------------------- MULTI LINE STRINGS -----------------------
  (void)coords;
  (void)camera;
}

void DrawPolygonTriangle(Triangle t, Vector2 a, Vector2 b, Vector2 c) {
  if (CROSS(a, b, c) > 0.01f) {
    fprintf(stderr, "ERROR: Triangle %d -> %d -> %d, not counter clockwise\n",
            t.a, t.b, t.c);
  }
  DrawTriangle(a, b, c, (Color){0, 0, 255, 100});
  if (render_options.show_triangulation) {
    DrawLineEx(a, b, 3, RED);
    DrawLineEx(a, c, 3, RED);
    DrawLineEx(b, c, 3, RED);
    char buf[8] = {0};
    sprintf(buf, "%d", t.a);
    DrawText(buf, (int)a.x, (int)a.y, 50, RED);
    sprintf(buf, "%d", t.b);
    DrawText(buf, (int)b.x, (int)b.y, 50, RED);
    sprintf(buf, "%d", t.c);
    DrawText(buf, (int)c.x, (int)c.y, 50, RED);
  }
  // DEBUG_MSG("drawing triangle: [%03.05f, %03.05f][%03.05f,
  // %03.05f][%03.05f, "
  //"%03.05f]\n",
  // a.x, a.y, b.x, b.y, c.x, c.y)
}

void DrawPolygons(const GeoJson *coords, Camera2D camera) {
  // --------------------- POLYGONS ---------------------
  TriangleArray triangles = coords->polygon_triangles;
  Coord2Array p_coords = coords->polygon_coords;
  for (S32 i = 0; i < triangles.count; i++) {
    Triangle t = triangles.d[i];
    Vector2 a = GetWorldToScreen2D(Vector2FromCoord2(p_coords.d[t.a]), camera);
    Vector2 b = GetWorldToScreen2D(Vector2FromCoord2(p_coords.d[t.b]), camera);
    Vector2 c = GetWorldToScreen2D(Vector2FromCoord2(p_coords.d[t.c]), camera);
    DrawPolygonTriangle(t, a, b, c);
  }
}

void DrawMultiPolygons(const GeoJson *coords, Camera2D camera) {
  // --------------------- MULTI POLYGONS ---------------------
  const TriangleArray triangles = coords->multi_polygon_triangles;
  const Coord2Array p_coords = coords->multi_polygon_coords;
  for (S32 i = 0; i < triangles.count; i++) {
    const Triangle t = triangles.d[i];
    Vector2 a = GetWorldToScreen2D(Vector2FromCoord2(p_coords.d[t.a]), camera);
    Vector2 b = GetWorldToScreen2D(Vector2FromCoord2(p_coords.d[t.b]), camera);
    Vector2 c = GetWorldToScreen2D(Vector2FromCoord2(p_coords.d[t.c]), camera);
    DrawPolygonTriangle(t, a, b, c);
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
  const U64 backing_buffer_size = GB(2);
#ifdef _WIN32
  void *backing_buffer = VirtualAlloc(NULL, backing_buffer_size,
                                      MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
#else
  void *backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                              MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif
  arena_init(arenas[0], backing_buffer, backing_buffer_size);

#ifdef _WIN32
  backing_buffer = VirtualAlloc(NULL, backing_buffer_size,
                                MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
#else
  backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif

  arena_init(arenas[1], backing_buffer, backing_buffer_size);
  Arena *arena = GetScratch().arena;
  GeoJson *serialized_coords = geo_json_parse(arena, argv[1]);

  //--------------------------------------------------------------------------------------
  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 1920;
  const int screenHeight = 1080;

  InitWindow(screenWidth, screenHeight, "Map Renderer");
  SetTargetFPS(60);

  Camera2D camera = {
      .offset = {(float)screenWidth / 2.0f, (float)screenHeight / 2.0f},
      .rotation = 0.0f,
      .zoom = 4.0f,
      .target = {0, 0}};

  //{
  //  LineStringArray lines = serialized_coords->line_strings;
  //  Coord2Array l_coords = serialized_coords->line_string_coords;
  //  for (S32 i = 0; i < lines.count; i++) {
  //    const S32 start_index = lines.d[i].coordinates.start;
  //    const S32 length = lines.d[i].coordinates.length;
  //    for (S32 j = 0; j < length - 1; j++) {
  //      DEBUG_MSG("line: [%03.05f, %03.05f] -> [%03.05f, %03.05f]\n",
  //                l_coords.d[start_index + j].x * ((F64)screenWidth / 180.0),
  //                (F32)(l_coords.d[start_index + j].y) *
  //                    ((F64)screenHeight / 90.0),
  //                (F32)(l_coords.d[start_index + j + 1].x *
  //                      ((F64)screenWidth / 180.0)),
  //                (F32)(l_coords.d[start_index + j + 1].y) *
  //                    ((F64)screenHeight / 90.0))
  //    }
  //  }
  //}
  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------

    if (IsKeyPressed(KEY_NINE))
      render_options.show_node_endpoints = false;
    if (IsKeyPressed(KEY_ZERO))
      render_options.show_node_endpoints = true;
    if (IsKeyReleased(KEY_T))
      render_options.show_triangulation ^= true;

    // Translate based on mouse right click
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      Vector2 delta = GetMouseDelta();
      delta = Vector2Scale(delta, -1.0f / camera.zoom);
      camera.target = Vector2Add(camera.target, delta);
    }

    // Zoom based on mouse wheel
    F32 wheel = GetMouseWheelMove();
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
      F32 scale = 0.2f * wheel;
      camera.zoom = Clamp(expf(logf(camera.zoom) + scale), 0.001f, 1000000.0f);
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
    draw_line_strings(serialized_coords, camera);
    draw_multi_line_strings(serialized_coords, camera);
    DrawPolygons(serialized_coords, camera);
    DrawMultiPolygons(serialized_coords, camera);

    // --------------------- HUD -------------------------
    DrawText(TextFormat("CURRENT ZOOM: %03.04f", camera.zoom), 640, 10, 20,
             RED);
    DrawText(TextFormat("CAMERA TARGET: [%03.04f, %03.04f]", camera.target.x,
                        camera.target.y),
             640, 40, 20, RED);
    DrawFPS(640, 70);

    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------
  return 0;
}
