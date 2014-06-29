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

#include "sphere.h"

namespace flin {


bool Sphere::intersects(const Sphere &sph) {
    float_t r2 = radius + sph.radius;
    return sph.center.distanceSq(center) <= r2*r2;
}

// true if sph is inside this
bool Sphere::contains(const Sphere &sph) {
    float_t r2 = radius - sph.radius;
    if (r2 < 0) return false;
    return sph.center.distanceSq(center) <= r2*r2;
}

// faster if intersection & containment tests are needed
// if INSIDE, then sph is inside "this"
IntersectionCode Sphere::intersectionCode(const Sphere &sph) {
    float_t d2 = sph.center.distanceSq(center);
    float_t r2 = radius + sph.radius;

    r2 *= r2; // (a+b)^2

    if (d2 > r2) return OUTSIDE; // dist^2 > (a+b)^2

    if (sph.radius > radius) return INTERSECTS; // if "this" is smaller than sph, then it can't contain it

    float_t ab = radius * sph.radius; // ab

    if (d2 < (r2-4.0*ab)) return INSIDE; // dist^2 < (a+b)^2-4ab <==> dist^2 < (a-b)^2

    return INTERSECTS;
}

// true if the box is inside this sphere
bool Sphere::contains(const AABoundsf &box) {
    return box.farthestCornerFromPoint(center).distanceSq(center) <= radius*radius;
}

bool Sphere::containedIn(const AABoundsf &box) {
    return (center.comp.x-radius >= box.min.comp.x &&
        center.comp.x+radius <= box.max.comp.x &&
        center.comp.y-radius >= box.min.comp.y &&
        center.comp.y+radius <= box.max.comp.y &&
        center.comp.z-radius >= box.min.comp.z &&
        center.comp.z+radius <= box.max.comp.z);
}

bool Sphere::intersects(const AABoundsf &box) {
    float_t e;
    float_t d = 0;
    for (int i =0; i < 3; i++) {
        if ((e = center.a[i] - box.min[i]) < 0) {
            if (e < - radius) return false;
            d = d + e*e;
        } else if ((e = center.a[i] - box.max[i]) > 0) {
            if (e > radius) return false;
            d = d + e*e;
        }
    }
    if (d <= radius * radius) return true;
    return false;
}

AABoundsf Sphere::getBoundingBox() {
    Vector3Df diag(radius, radius, radius);
    return AABoundsf(center - diag, center + diag);
}

void Sphere::fromBoundingBox(const AABoundsf &box) {
    center = box.center();
    radius = box.max.distance(box.min) * 0.5;
}

} // namespace
