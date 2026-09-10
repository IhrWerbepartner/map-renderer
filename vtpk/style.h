#pragma once
#include "../base.h"
#include "../string8.h"


// describes a lookup for a style held in FilterValueNode.paint
// Nodes are ordered hirachically SourceLayerNode -> FilterKeyNode -> FilterValueNode.
// Where Key/Value nodes can have siblings representing the multiple children of the node in the hirachy above. 
typedef struct VT_LayerStyleNode VT_LayerStyleNode;
struct VT_LayerStyleNode {
    enum { SOURCE_LAYER, FILTER_KEY, FILTER_VALUE } type;
    union {
        struct SourceLayerNode {
            enum { FILL, LINE, SYMBOL, CIRCLE } type;
            String8 source_layer;
            S32 children;
        };
        struct FilterKeyNode {
            String8 key;
            S32 next;
        };
        struct FilterValueNode {
            S32 value;
            union {
                Color fill_color;
                Color line_color;
                String8 icon_image; // referes to the sprite name
            } paint;
            S32 next;
        };
    };
};

struct VT_Sprite {
    String8 name; // usable as key in a map
    S32 x, y;
    S32 width, height;
};
