#pragma once

struct object {
    float angle;
    float bbox[4];
    int class_id;

    object(float ang = 180.0, std::array<float, 4> b = { 0.0f, 0.0f, 0.0f, 0.0f }) : angle(ang) {
        for (int i = 0; i < 4; ++i) {
            bbox[i] = b[i];
        }
    }
};


