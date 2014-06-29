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

#include "plane.h"
#include "sphere.h"

namespace flin {

Plane::Plane(const Vector3Df &normal_, float_t d_):normal(normal_),d(d_){
    float_t len_sq = normal.magnitudeSq();
    degenerate = false;
    if (FloatTypes<float_t>::neq(len_sq,1)) {
        if (FloatTypes<float_t>::nz(len_sq)) {
            float_t inv_len = 1.0/sqrt(len_sq);
            normal *= inv_len;
            d *= inv_len;
        } else {
            normal.setToZero(); // plane on X-Y
            d=0;
            degenerate = true;
        }
    }
}


Plane::Plane(const Vector4Df &dual) {
    fromDual(dual);
}


Plane::Plane(const Vector3Df &p1,const Vector3Df &p2,const Vector3Df &p3) {
    normal = (p2-p1).cross(p3-p1);

    if (normal.normalize()) {
        degenerate = false;
        d = -normal.dot(p1);
    } else {
        degenerate = true;
        d = 0;
    }
}


Plane &Plane::operator = (const Plane &p) {
    normal = p.normal;
    d = p.d;
    degenerate = p.degenerate;
    return *this;
}

Plane &Plane::fromDual(const Vector4Df &dual) {
    degenerate = false;

    normal.fromVector4D(dual);

    float_t mag = normal.magnitude();

    if (FloatTypes<float_t>::z(mag)) {
        degenerate = true;
        normal.setToZero();
        d = dual.comp.w;
    } else {
        normal /= mag;
        d = dual.comp.w / mag;
    }

    return *this;
}



void Plane::flip() {
    normal.flip();
    d = -d;
}

void Plane::displace(float_t dist) {
    d -= normal.dot(Vector3Df(dist, dist, dist));
}

float_t Plane::distance(const Vector3Df &point) const {
    return point.dot(normal)+d;
}

Vector3Df Plane::projection(const Vector3Df &point) const {
    return point - normal * (d+normal.dot(point));
}

IntersectionCode Plane::intersectionCode(const Sphere &sph) const {
    float_t d = distance(sph.center);
    if (d > sph.radius) return OUTSIDE;
    if (d < -sph.radius) return INSIDE;
    return INTERSECTS;
}

IntersectionCode Plane::intersectionCode(const AABoundsf &box) const {
    Vector3Df vmin, vmax;

    // X axis
    if(normal.comp.x > 0) {
        vmin.comp.x = box.min.comp.x;
        vmax.comp.x = box.max.comp.x;
    } else {
        vmin.comp.x = box.max.comp.x;
        vmax.comp.x = box.min.comp.x;
    }
    // Y axis
    if(normal.comp.y > 0) {
        vmin.comp.y = box.min.comp.y;
        vmax.comp.y = box.max.comp.y;
    } else {
        vmin.comp.y = box.max.comp.y;
        vmax.comp.y = box.min.comp.y;
    }
    // Z axis
    if(normal.comp.z > 0) {
        vmin.comp.z = box.min.comp.z;
        vmax.comp.z = box.max.comp.z;
    } else {
        vmin.comp.z = box.max.comp.z;
        vmax.comp.z = box.min.comp.z;
    }
    if (normal.dot(vmin) + d > 0) return OUTSIDE;
    if (normal.dot(vmax) + d >= 0) return INTERSECTS;

    return INSIDE;
}


bool Plane::getIntersectionPoint(Vector3Df &pt, const Plane &p1, const Plane &p2, const Plane &p3) {
    // first check there's actually a solution
    if (FloatTypes<float_t>::z(p1.normal.cross(p2.normal).dot(p3.normal))) return false;

    Matrix3D<float_t> M;

    M.setRow(0, p1.normal);
    M.setRow(1, p2.normal);
    M.setRow(2, p3.normal);

    M = M.getInverse();

    Vector3Df D(-p1.d, -p2.d, -p3.d);

    pt = M * D;

    return true;
}



} // namespace
