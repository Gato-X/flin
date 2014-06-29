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

#ifndef __FLIN_PLANE_INC__
#define __FLIN_PLANE_INC__

#include "flin_min.h"

namespace flin {

class Sphere;

class Plane{
    private:

    Vector3Df normal;
    float_t d;

    bool degenerate;

    public:

    Plane():degenerate(true) {};
    Plane(const Plane &p):normal(p.normal),d(p.d),degenerate(p.degenerate){}
    Plane(const Vector3Df &normal_, float_t d_);
    Plane(const Vector4Df &dual);
    Plane(const Vector3Df &p1,const Vector3Df &p2,const Vector3Df &p3);

    Plane &operator = (const Plane &p);
    Plane &fromDual(const Vector4Df &dual);

    bool isValid() { return !degenerate; }

    void flip();
    void displace(float_t dist);

    float_t distance(const Vector3Df &point) const;
    Vector3Df projection(const Vector3Df &point) const;
    IntersectionCode intersectionCode(const Sphere &sph) const;
    IntersectionCode intersectionCode(const AABoundsf &box) const;

    static bool getIntersectionPoint(Vector3Df &pt, const Plane &p1, const Plane &p2, const Plane &p3);

};

} // namespace

#endif // __FLIN_PLANE_INC__
