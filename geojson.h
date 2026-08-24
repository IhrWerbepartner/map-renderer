#include "arena.c"
#include "base.h"
#include "fixed-array.c"
#include "json_parser.h"
#include "raymath.h"
#include "string8.h"
#include <assert.h>
#include <math.h>
#include <raylib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SEIDEL_TRIANGULATION 0

#if SEIDEL_TRIANGULATION
#include "triangulate/tessalate.h"
#else
#include "triangulate/earcut.h"
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

typedef struct Polygon {
    Slice polygon;
} Polygon;

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
    PolygonArray polygons;           // has an array of slices into array of triangles

    MultiPolygonArray multi_polygons; // has an array of slices into array of triangles

    GeoPropertiesArray properties; // TODO: add a handle in the types for as an
                                   // index in to the properties array
} GeoJson;

typedef struct GeoJsonRenderOptions GeoJsonRenderOptions;
struct GeoJsonRenderOptions {
    bool show_node_endpoints;
    bool show_triangulation;
    bool show_vertex_numbers;
};

static void init_all_arrays(Arena *arena, GeoJson *base, S32 capacity) {
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

    base->polygons = PolygonArrayNew(arena, capacity);
    base->multi_polygons = MultiPolygonArrayNew(arena, capacity);
}

// transform coordintes:
// X = (180 + lognitude) / 360 * 4096
// y = ln(tan(45 + latitude / 2))
// Y = (180 - y * (180 / π)) / 360 * worldSize
static Coord2 Coord2WebMecartorFromWGS84(const F64 longnitude, const F64 latitude) {
    const F64 projection_limit = 85.05112878;
    const F64 lat_clamped = ClampTB(-projection_limit, latitude, projection_limit);
    const F64 X = (180.0 + longnitude);
    const F64 y_tmp = log(tan(radians_from_degrees_f64(45.0 + lat_clamped / 2.0)));
    const F64 Y = (180.0 - (y_tmp * (180.0 / (F64)PI)));
    return (Coord2){X, Y};
}

static Coord2 Coord2FromJsonArrayNode(const JsonNode *coordinates) {
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
    return Coord2WebMecartorFromWGS84(x, y);
}

// iterate over the coordinates omitting the last one as it is
// identical to the first. input: [a, b, c, d, e] puts [a, b, c, d] into the
// result array. Returns the index of the coordinate with the lowest y and
// hightest x.
static S32 ContourFromJsonArray(const JsonNode *coordinates, Coord2Array *result_array) {
    if (coordinates->type != JSON_ARRAY) {
        ERROR_MSG("invalid coordinates node type")
    }
    if (coordinates->children.count <= 2) {
        ERROR_MSG("invalid contour with: %d coordinates\n", coordinates->children.count)
    }
    S32 min_index = 0;
    Coord2 min_coordinate = (Coord2){min_F64, max_F64};
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
static bool MakeCoordinatesCounterClockwise(Coord2Slice coordinates, S32 min_index) {
    ASSERT(min_index >= 0 && min_index < coordinates.count, "index out of range");
    if (CROSS(coordinates.v[min_index],
              coordinates.v[(min_index + coordinates.count - 1) % coordinates.count],
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
        ERROR_MSG("invalid contour with: %d coordinates\n", coordinates->children.count)
    }
    const JsonNode *point_coords = coordinates->children.last;
    while (point_coords != NULL && point_coords != coordinates->children.first) {
        Coord2ArrayPush(result_array, Coord2FromJsonArrayNode(point_coords));
        point_coords = point_coords->prev;
    }
}

static void Coord2ArrayFromJsonArray(const JsonNode *coordinates,
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

static GeoJson *serialize(Arena *arena, JsonNode *root) {
    GeoJson *render_data = (GeoJson *)arena_alloc(arena, sizeof(GeoJson));
    if (root->type != JSON_NULL || root->children.count != 1) {
        ERROR_MSG("invalid json")
    }
    root = root->children.first;
    if (root->type != JSON_OBJECT) {
        ERROR_MSG("invalid object")
    }
    JsonNode *const featureCollectionType = find_key_value_in_children(
        root, String8FromCString("type"), String8FromCString("FeatureCollection"));
    if (featureCollectionType == NULL) {
        ERROR_MSG("invalid geojson type")
    }
    JsonNode *const features = find_key_in_children(root, String8FromCString("features"));
    if (features == NULL || features->type != JSON_ARRAY) {
        ERROR_MSG("invalid geojson features")
    }
    // init_all_arrays(arena, parsed, features->children.length); TODO:
    init_all_arrays(arena, render_data, 1000 * 1000 * 10);
    JsonNode *current_child = features->children.first;

    // parse all "Features" in their appropriate arrays
    Temp_Arena_Memory scratch = GetScratchConflict(&arena, 1);
    while (current_child != NULL) {
        JsonNode *const feature_type = find_key_value_in_children(
            current_child, String8FromCString("type"), String8FromCString("Feature"));
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
            PointArrayPush(&render_data->interest_points,
                           (Point){.coordinates = Coord2FromJsonArrayNode(coordinates)});
        }

        if (String8Equals(geometry_type->text_value, String8FromCString("MultiPoint"))) {
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

        if (String8Equals(geometry_type->text_value, String8FromCString("LineString"))) {
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

        if (String8Equals(geometry_type->text_value, String8FromCString("Polygon"))) {
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

            TessalatePolygon(&render_data->polygon_triangles,
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
                Coord2 *vertices =
                    &render_data->polygon_coords.d[render_data->polygon_coords.count];
                S32 first_coordinate_idx = render_data->polygon_coords.count;
                S32 first_triangle_idx = render_data->polygon_triangles.count;

                // create emtpy slot at vertices[0] for triangulation
                Coord2ArrayPush(&render_data->polygon_coords, (Coord2){0.f, 0.f});

#if SEIDEL_TRIANGULATION
                JsonNode *contour_array = polygon->children.first;
                const S32 min_coordinate_index =
                    ContourFromJsonArray(contour_array, &render_data->polygon_coords);
                S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
                bool outerContourIsClockwise = MakeCoordinatesCounterClockwise(
                    Coord2SliceFromArrayExt(&render_data->polygon_coords,
                                            first_coordinate_idx + 1,
                                            contour_array->children.count - 1),
                    min_coordinate_index);
                // DEBUG_MSG("min vertex: %d\n", min_coordinate_index + 1);
                contour_array = contour_array->next;
                while (contour_array != NULL) {
                    ASSERT(contour_array->children.count > 1,
                           "invalid size for polygon contour: %d",
                           contour_array->children.count)
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
                JsonNode *contour_array = polygon->children.first;
                ContourFromJsonArray(contour_array, &render_data->polygon_coords);
                S32ArrayPush(&contour_sizes, contour_array->children.count - 1);
                // DEBUG_MSG("min vertex: %d\n", min_coordinate_index + 1);
                contour_array = contour_array->next;
                while (contour_array != NULL) {
                    ASSERT(contour_array->children.count > 1,
                           "invalid size for polygon contour: %d",
                           contour_array->children.count)
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
                    ASSERT(triangle->a > first_coordinate_idx &&
                               triangle->a < render_data->polygon_coords.count,
                           "multi polygon index for a out of range, should be inside "
                           "(%d-%d), was: %d",
                           first_coordinate_idx, render_data->polygon_coords.count,
                           triangle->a);
                    ASSERT(triangle->b > first_coordinate_idx &&
                               triangle->b < render_data->polygon_coords.count,
                           "multi polygon index for b out of range, should be inside "
                           "(%d-%d), was: %d",
                           first_coordinate_idx, render_data->polygon_coords.count,
                           triangle->b);
                    ASSERT(triangle->c > first_coordinate_idx &&
                               triangle->c < render_data->polygon_coords.count,
                           "multi polygon index for c out of range, should be inside "
                           "(%d-%d), was: %d",
                           first_coordinate_idx, render_data->polygon_coords.count,
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

static GeoJson *geo_json_parse(Arena *arena, char *filepath) {
    FILE *f = fopen(filepath, "r");
    if (f == NULL) {
        ERROR_MSG("could not open file: %s", filepath)
    }

    Temp_Arena_Memory scratch = GetScratchConflict(&arena, 1);
    fseek(f, 0, SEEK_END);
    size_t fsize = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET); /* same as rewind(f); */

    char *file_content = arena_alloc(scratch.arena, fsize + 1);
    fread(file_content, fsize, 1, f);
    fclose(f);

    JsonNode js = {0};
    JsonParseValue(arena, &js, (String8){0}, file_content);
    GeoJson *serialized = serialize(arena, &js);

    temp_arena_memory_end(scratch);
    return serialized;
}

// --------------------- DRAW FEATURES -------------------
static void draw_points(const GeoJson *coords, Camera2D camera) {
    // --------------------- POINTS -------------------
    PointArray i_pts = coords->interest_points;
    for (S32 i = 0; i < i_pts.count; i++) {
        Vector2 a = GetWorldToScreen2D(Vector2FromCoord2(i_pts.d[i].coordinates), camera);
        DrawCircleV(a, 5.0f, RED);
    }
}
static void draw_multi_points(const GeoJson *coords, Camera2D camera) {
    // --------------------- MULTI-POINTS -------------------
    MultiPointArray points = coords->multi_points;
    Coord2Array m_coords = coords->multi_point_coords;
    for (S32 i = 0; i < points.count; i++) {
        const S32 start_index = points.d[i].coordinates.start;
        const S32 length = points.d[i].coordinates.length;
        for (S32 j = 0; j < length; j++) {
            Vector2 a = GetWorldToScreen2D(Vector2FromCoord2(m_coords.d[start_index + j]),
                                           camera);
            DrawCircleV(a, 5.0f, RED);
            // DEBUG("drawing: [%03.05f, %03.05f]\n", a.x, a.y)
        }
    }
}
static void draw_line_strings(const GeoJson *coords, Camera2D camera,
                              GeoJsonRenderOptions render_options) {
    // --------------------- LINE-STRINGS -------------------
    LineStringArray lines = coords->line_strings;
    Coord2Array l_coords = coords->line_string_coords;
    for (S32 i = 0; i < lines.count; i++) {
        const S32 start_index = lines.d[i].coordinates.start;
        const S32 length = lines.d[i].coordinates.length;
        for (S32 j = 0; j < length - 1; j++) {
            Vector2 a = GetWorldToScreen2D(Vector2FromCoord2(l_coords.d[start_index + j]),
                                           camera);
            Vector2 b = GetWorldToScreen2D(
                Vector2FromCoord2(l_coords.d[start_index + j + 1]), camera);
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
static void draw_multi_line_strings(GeoJson *coords, Camera2D camera) {
    // -------------------- MULTI LINE STRINGS -----------------------
    (void)coords;
    (void)camera;
}

static inline void DrawPolygonTriangleWires(const GeoJson *coords, Camera2D camera,
                                            GeoJsonRenderOptions render_options) {
    if (render_options.show_triangulation || render_options.show_vertex_numbers) {
        Coord2Array t_coords = coords->polygon_coords;
        for (S32 i = 0; i < coords->polygon_triangles.count; i += 1) {
            Triangle t = coords->polygon_triangles.d[i];
            Vector2 a = GetWorldToScreen2D(Vector2FromCoord2(t_coords.d[t.a]), camera);
            Vector2 b = GetWorldToScreen2D(Vector2FromCoord2(t_coords.d[t.b]), camera);
            Vector2 c = GetWorldToScreen2D(Vector2FromCoord2(t_coords.d[t.c]), camera);
            if (render_options.show_triangulation) {
                DrawLineEx(a, b, 3, RED);
                DrawLineEx(a, c, 3, RED);
                DrawLineEx(b, c, 3, RED);
            }
            if (render_options.show_vertex_numbers) {
                char buf[8] = {0};
                sprintf(buf, "%d", t.a);
                DrawText(buf, (int)a.x, (int)a.y, 50, RED);
                sprintf(buf, "%d", t.b);
                DrawText(buf, (int)b.x, (int)b.y, 50, RED);
                sprintf(buf, "%d", t.c);
                DrawText(buf, (int)c.x, (int)c.y, 50, RED);
            }
        }
    }
}

static Mesh MeshFromPolygons(Arena *arena, GeoJson *polygons) {

    // create a big mesh (triangle soup) to avoid unsigned short limit for
    // indicdes in openGL
    S32 vertex_soup_count = polygons->polygon_triangles.count * 3;
    Mesh mesh = {
        .vertexCount = vertex_soup_count,
        .vertices = arena_alloc_array(arena, float, (size_t)vertex_soup_count * 3),
        .triangleCount = polygons->polygon_triangles.count,
    };
    S32 v = 0;
    Coord2 *coords = polygons->polygon_coords.d;

    for (S32 i = 0; i < polygons->polygon_triangles.count; i++) {
        Triangle t = polygons->polygon_triangles.d[i];
        mesh.vertices[v++] = (float)coords[t.a].x;
        mesh.vertices[v++] = (float)coords[t.a].y;
        mesh.vertices[v++] = 0.f;
        mesh.vertices[v++] = (float)coords[t.c].x;
        mesh.vertices[v++] = (float)coords[t.c].y;
        mesh.vertices[v++] = 0.f;
        mesh.vertices[v++] = (float)coords[t.b].x;
        mesh.vertices[v++] = (float)coords[t.b].y;
        mesh.vertices[v++] = 0.f;
    }
    return mesh;
}

static void UpdateCameraPos(Camera2D *camera, Screen screen) {
    // Translate based on mouse right click
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        delta = Vector2Scale(delta, -1.0f / camera->zoom);
        camera->target = Vector2Add(camera->target, delta);
    }

    // Zoom based on mouse wheel
    F32 wheel = GetMouseWheelMove();
    if (wheel != 0) {
        // Get the world point that is under the mouse
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), *camera);

        // Set the offset to where the mouse is
        camera->offset = GetMousePosition();

        // Set the target to match, so that the camera maps the world space
        // point under the cursor to the screen space point under the cursor at
        // any zoom
        camera->target = mouseWorldPos;

        // Zoom increment
        // Uses log scaling to provide consistent zoom speed
        F32 scale = 0.2f * wheel;
        camera->zoom = Clamp(expf(logf(camera->zoom) + scale), 0.001f, 1000000.0f);
    }

    // Camera reset (zoom and rotation)
    if (IsKeyPressed(KEY_R)) {
        camera->zoom = 4.0f;
        camera->rotation = 0.0f;
        camera->offset =
            (Vector2){(float)screen.width / 2.0f, (float)screen.height / 2.0f};
        camera->target.x = 0;
        camera->target.y = 0;
    }
}

void GeoJsonDisplayFile(char *filename, Screen screen) {
    Arena *arena = GetScratch().arena;
    Temp_Arena_Memory scratch_parse = temp_arena_memory_begin(arena);
    GeoJson *serialized_coords = geo_json_parse(scratch_parse.arena, filename);

    //--------------------------------------------------------------------------------------
    // Initialization
    //--------------------------------------------------------------------------------------

    Camera2D camera = {
        .offset = {(float)screen.width / 2.0f, (float)screen.height / 2.0f},
        .rotation = 0.0f,
        .zoom = 4.0f,
        .target = {0, 0}};

    GeoJsonRenderOptions render_options = {0};

    // create a big mesh (triangle soup) to avoid unsigned short limit for
    // indicdes in openGL
    Mesh mesh = MeshFromPolygons(arena, serialized_coords);
    temp_arena_memory_end(scratch_parse);
    UploadMesh(&mesh, false);
    Material material = LoadMaterialDefault();
    material.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;

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
        if (IsKeyReleased(KEY_N))
            render_options.show_vertex_numbers ^= true;

        UpdateCameraPos(&camera, screen);

        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode2D(camera);
        {
            DrawMesh(mesh, material, MatrixIdentity());
            const Vector2 top_left =
                Vector2FromCoord2(Coord2WebMecartorFromWGS84(-180.0, 90.0));
            const Vector2 bot_right =
                Vector2FromCoord2(Coord2WebMecartorFromWGS84(180.0, -90.0));
            const Rectangle r =
                (Rectangle){top_left.x, top_left.y, bot_right.x - top_left.x,
                            bot_right.y - top_left.y};
            DrawRectangleLinesEx(r, 3.f, MAGENTA);
            DrawCircleV(top_left, 3.f, GREEN);
            DrawCircleV(bot_right, 3.f, BLUE);
        }
        EndMode2D();

        draw_points(serialized_coords, camera);
        draw_multi_points(serialized_coords, camera);
        draw_line_strings(serialized_coords, camera, render_options);
        draw_multi_line_strings(serialized_coords, camera);
        DrawPolygonTriangleWires(serialized_coords, camera, render_options);

        // --------------------- HUD -------------------------
        DrawText(TextFormat("CURRENT ZOOM: %03.04f", camera.zoom), 640, 10, 20, RED);
        DrawText(TextFormat("CAMERA TARGET: [%03.04f, %03.04f]", camera.target.x,
                            camera.target.y),
                 640, 40, 20, RED);
        DrawFPS(640, 70);
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
        DrawText(TextFormat("MOUSE POS : [%03.04f, %03.04f]", mouseWorldPos.x,
                            mouseWorldPos.y),
                 100, 10, 20, RED);
        DrawText(TextFormat("Triangles: %d", serialized_coords->polygon_triangles.count),
                 100, 40, 20, RED);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }
}
