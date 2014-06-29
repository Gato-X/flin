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

#ifndef __FLIN_SPHERE_INC__
#define __FLIN_SPHERE_INC__

#include "flin_min.h"

namespace flin {

struct Sphere{
    Vector3Df center;
    float_t radius;

    Sphere() {}
    Sphere(const Sphere &s):center(s.center),radius(s.radius){}
    Sphere(const Vector3Df &center_, float_t radius_):center(center_),radius(radius_){}

    bool intersects(const Sphere &sph);

    // true if sph is inside this
    bool contains(const Sphere &sph);

    // faster if intersection & containment tests are needed
    // if INSIDE, then sph is inside "this"
    IntersectionCode intersectionCode(const Sphere &sph);

    // true if the box is inside this sphere
    bool contains(const AABounds<float_t> &box);
    bool containedIn(const AABounds<float_t> &box);
    bool intersects(const AABounds<float_t> &box);
    AABounds<float_t> getBoundingBox();
    void fromBoundingBox(const AABounds<float_t> &box);

};

}

#endif // __FLIN_SPHERE_INC__
