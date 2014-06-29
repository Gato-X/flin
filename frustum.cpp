/*

The MIT License (MIT)

Copyright (c) 2014 Gato (Guillermo Romero)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Contact: gato@felingeneering.com

*/

#include "frustum.h"
#include "sphere.h"
#include "plane.h"

namespace flin {

Frustum::Frustum(const Frustum &f) {
    for (int i = 0; i < 6; i++) planes[i] = f.planes[i];
}

Frustum::Frustum(const Matrix3Hf &mat) {
    fromProjection(mat);
}

IntersectionCode Frustum::intersectionCode(const AABoundsf &box) {
    IntersectionCode ret = INSIDE;

    for(int i = 0; i < 6; ++i) {
        IntersectionCode code = planes[i].intersectionCode(box);
        if (code == OUTSIDE) return OUTSIDE;
        if (code == INTERSECTS) ret = INTERSECTS;
    }
    return ret;
}

IntersectionCode Frustum::intersectionCode(const Sphere &sph) {
    IntersectionCode ret = INSIDE;

    for(int i = 0; i < 6; ++i) {
        IntersectionCode code = planes[i].intersectionCode(sph);
        if (code == OUTSIDE) return OUTSIDE;
        if (code == INTERSECTS) ret = INTERSECTS;
    }
    return ret;
}


void Frustum::fromProjection(const Matrix3Hf &mat) {
    Vector4Df x_vec(-mat.getRow(0));
    Vector4Df y_vec(-mat.getRow(1));
    Vector4Df z_vec(-mat.getRow(2));
    Vector4Df w_vec(-mat.getRow(3));

    planes[LEFT_PLANE].fromDual(w_vec + x_vec);
    planes[RIGHT_PLANE].fromDual(w_vec - x_vec);
    planes[TOP_PLANE].fromDual(w_vec - y_vec);
    planes[BOTTOM_PLANE].fromDual(w_vec + y_vec);
    planes[FAR_PLANE].fromDual(w_vec - z_vec);
    planes[NEAR_PLANE].fromDual(w_vec + z_vec);

    has_far_plane = planes[FAR_PLANE].isValid();
}

bool Frustum::hasFarPlane() {
    return has_far_plane;
}

bool Frustum::getPoints(Vector3Df *p[8]) {

    Plane::getIntersectionPoint(*p[0], planes[LEFT_PLANE], planes[TOP_PLANE], planes[NEAR_PLANE]);
    Plane::getIntersectionPoint(*p[1], planes[LEFT_PLANE], planes[BOTTOM_PLANE], planes[NEAR_PLANE]);
    Plane::getIntersectionPoint(*p[2], planes[RIGHT_PLANE], planes[BOTTOM_PLANE], planes[NEAR_PLANE]);
    Plane::getIntersectionPoint(*p[3], planes[RIGHT_PLANE], planes[TOP_PLANE], planes[NEAR_PLANE]);

    Plane fp;

    if (has_far_plane) {
        fp = planes[FAR_PLANE];
    } else {
        float_t d = p[0]->distance(*p[2]);
        fp = planes[NEAR_PLANE];
        fp.displace(-d*10.0);
    }

    Plane::getIntersectionPoint(*p[0], planes[LEFT_PLANE], planes[TOP_PLANE], fp);
    Plane::getIntersectionPoint(*p[1], planes[LEFT_PLANE], planes[BOTTOM_PLANE], fp);
    Plane::getIntersectionPoint(*p[2], planes[RIGHT_PLANE], planes[BOTTOM_PLANE], fp);
    Plane::getIntersectionPoint(*p[3], planes[RIGHT_PLANE], planes[TOP_PLANE], fp);

    return has_far_plane;
}


} // namespace

