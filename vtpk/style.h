#pragma once
#include "../base.h"
#include "../string8.h"

struct VT_Sprite {
    String8 name; // usable as key in a map
    S32 x, y;
    S32 width, height;
};
