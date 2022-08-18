// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering


// This example will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <iostream>
using namespace std;

using pixel = std::pair<int, int>;


float * compute_coordinate(const rs2::depth_frame& frame, pixel u);
// void compute_coordinate(const rs2::depth_frame& frame, pixel u);

struct toggle
{
    toggle() : x(), y() {}
    toggle(int xl, int yl)
        : x(xl),
          y(yl)
    {}


    void render(const window& app)
    {
        glColor4f(0.f, 0.0f, 0.0f, 0.2f);
        render_circle(app, 10);
        render_circle(app, 8);
        glColor4f(1.f, 0.9f, 1.0f, 1.f);
        render_circle(app, 6);
    }

    void render_circle(const window& app, float r)
    {
        const float segments = 16;
        glBegin(GL_TRIANGLE_STRIP);
        for (auto i = 0; i <= segments; i++)
        {
            auto t = 2 * PI_FL * float(i) / segments;

            glVertex2f(x * app.width() + cos(t) * r,
                y * app.height() + sin(t) * r);

            glVertex2f(x * app.width(),
                y * app.height());
        }
        glEnd();
    }


    float x;
    float y;
    bool selected = false;
};

struct state
{
    //bool mouse_down = false;
    toggle detect_point;
};
// float * get_coordinate(const rs2::depth_frame& depth,const state& s);
// void get_coordinate(const rs2::depth_frame& depth,const state& s);
using pixel = std::pair<int, int>;


void draw_line(float x0, float y0, float x1, float y1, int width) ;


// float * compute_coordinate(const rs2::depth_frame& frame, pixel u)
// {
//     float upixel[2]; // From pixel
//     float upoint[3]; // From point (in 3D)

//     static float point[3];

//     // Copy pixels into the arrays (to match rsutil signatures)
//     upixel[0] = u.first;
//     upixel[1] = u.second;

//     float udist = frame.get_distance(upixel[0], upixel[1]);

//     rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data

//     rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

//     point[0] = upoint[0]*100;
//     point[1] = upoint[1]*100;
//     point[2] = upoint[2]*100;

//     return point ;
   
// }

// float * get_coordinate(const rs2::depth_frame& depth,
//                             const state& s)
// {

//     float *p;
//     pixel from_pixel = {s.detect_point.x,s.detect_point.y};

//     p = compute_coordinate(depth, from_pixel);

//     return p;

// }


