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

#ifndef __FLIN_GEOM_INC__
#define __FLIN_GEOM_INC__

namespace flin {

template<class T>
struct Size2D {
	T w, h;
	Size2D() {}
	Size2D(T w_, T h_):w(w_),h(h_) {}
    template<class T2>
    Size2D(const Size2D<T2> &s):w(s.w),h(s.h){}
};

template<class T>
struct AABounds {
    Vector3D<T> min;
    Vector3D<T> max;

    AABounds(T x1, T y1, T z1, T x2, T y2, T z2):
        min(x1,y1,z1),max(x2,y2,z2){}


    AABounds(const Vector3D<T> &min_, const Vector3D<T> &max_):
        min(min_),max(max_){}

    AABounds(const AABounds &bb):min(bb.min),max(bb.max){}

    bool isEmpty() const {
        return min.comp.x > max.comp.x ||
               min.comp.y > max.comp.y ||
               min.comp.z > max.comp.z;
    }

    void setEmpty() {
        max = min - Vector3D<T>(1,1,1);
    }

    bool isPoint() const {
        return min == max;
    }

    AABounds<T> intersection(const AABounds<T> &b) const {
        return AABounds(min.max(b.min), max.min(b.max));
    }

    bool intersects(const AABounds<T> &b) const {
        return (max.comp.x >= b.min.comp.x &&
                max.comp.y >= b.min.comp.y &&
                max.comp.z >= b.min.comp.z &&
                min.comp.x <= b.max.comp.x &&
                min.comp.y <= b.max.comp.y &&
                min.comp.z <= b.max.comp.z);
    }

    // if "contains" then "intersects"
    bool contains(const AABounds<T> &b) const {
        return (max.comp.x >= b.max.comp.x &&
                max.comp.y >= b.max.comp.y &&
                max.comp.z >= b.max.comp.z &&
                min.comp.x <= b.min.comp.x &&
                min.comp.y <= b.min.comp.y &&
                min.comp.z <= b.min.comp.z);
    }

    bool contains(const Vector3D<T> &p) const {
        return (p.comp.x >= min.comp.x) && (p.x <= max.comp.x) &&
               (p.comp.y >= min.comp.y) && (p.y <= max.comp.y) &&
               (p.comp.z >= min.comp.z) && (p.z <= max.comp.z);
    }

    AABounds<T> operator +(const AABounds<T> &b) const {
        return AABounds(min.min(b.min), max.max(b.max));
    }

    AABounds<T> operator +(const Vector3D<T> &p) const {
        return AABounds(min.min(p), max.max(p));
    }

    AABounds<T> &operator +=(const AABounds<T> &b) {
        min = min.min(b.min);
        max = max.max(b.max);
        return *this;
    }

    AABounds<T> &operator +=(const Vector3D<T> &p) {
        min = min.min(p);
        max = max.max(p);
        return *this;
    }


    const AABounds<T> &operator = (const AABounds<T> &r) {
        min = r.min;
        max = r.max;
        return *this;
    }

    const AABounds<T> &operator = (const Vector3D<T> &v) {
        min = v;
        max = v;
        return *this;
    }


    bool operator == (const AABounds<T> &p) const {
        if (p.isEmpty() && isEmpty()) return true;

        return (min == p.min) && (max == p.max);
    }

    T width() const {
        if (isEmpty()) return 0;
        return max.comp.x - min.comp.x;
    }

    T height() const {
        if (isEmpty()) return 0;
        return max.comp.y - min.comp.y;
    }

    T length() const {
        if (isEmpty()) return 0;
        return max.comp.z - min.comp.z;
    }


    void width(T w) {
        max.comp.x = min.comp.x + w;
    }

    void height(T h) {
        max.comp.y = min.comp.y + h;
    }

    void length(T l) {
        max.comp.z = min.comp.z + l;
    }

    T area() const {
        if (isEmpty()) return 0;
        T w = max.comp.x-min.comp.x;
        T h = max.comp.y-min.comp.y;
        T l = max.comp.z-min.comp.z;

        return 2.0*(w*(h+l)+h*l);
    }

    T volume() const {
        if (isEmpty()) return 0;
        return (max.comp.x-min.comp.x)*(max.comp.y-min.comp.y)*(max.comp.z-min.comp.z);
    }

    Vector3D<T> center() const {
        return Vector3D<T>((max.comp.x+min.comp.x)*0.5, (max.comp.y+min.comp.y)*0.5, (max.comp.z+min.comp.z)*0.5);
    }

    Vector3D<T> farthestCornerFromPoint(const Vector3D<T> &pt) const {
        Vector3D<T> c(center() - pt);
        return Vector3D<T>(
                    (c.comp.x > 0) ? max.comp.x : min.comp.x,
                    (c.comp.y > 0) ? max.comp.y : min.comp.y,
                    (c.comp.z > 0) ? max.comp.z : min.comp.z
                    );

    }

    Vector3D<T> closestCornerToPoint(const Vector3D<T> &pt) const {
        Vector3D<T> c(center() - pt);
        return Vector3D<T>(
                    (c.comp.x < 0) ? max.comp.x : min.comp.x,
                    (c.comp.y < 0) ? max.comp.y : min.comp.y,
                    (c.comp.z < 0) ? max.comp.z : min.comp.z
                    );

    }

};


template<class T>
struct Rect {
    Vector2D<T> min;
    Vector2D<T> max;

    Rect() {};

    Rect(T x1, T y1, T x2, T y2):
        min(x1,y1),max(x2,y2){}

    Rect(const Vector2D<T> &min_, const Vector2D<T> &max_):
        min(min_),max(max_){}

    template<class T2>
    Rect(const Rect<T2> &bb):min(bb.min),max(bb.max){}

    bool isEmpty() const {
        return (min.comp.x > max.comp.x) || (min.comp.y > max.comp.y);
    }

    void setEmpty() {
        max = min - Vector2D<T>(1,1);
    }

    bool isPoint() const {
        return min == max;
    }

    Rect<T> intersection(const Rect<T> &b) const {
        return Rect(min.max(b.min), max.min(b.max));
    }

    bool intersects(const Rect<T> &b) const {
        return (max.comp.x >= b.min.comp.x &&
                max.comp.y >= b.min.comp.y &&
                min.comp.x <= b.max.comp.x &&
                min.comp.y <= b.max.comp.y);
    }

    bool contains(const Rect<T> &b) const {
        return (max.comp.x >= b.max.comp.x &&
                max.comp.y >= b.max.comp.y &&
                min.comp.x <= b.min.comp.x &&
                min.comp.y <= b.min.comp.y);
    }


    bool contains(const Vector2D<T> &p) const {
        return (p.comp.x >= min.comp.x) && (p.x <= max.comp.x) &&
               (p.comp.y >= min.comp.y) && (p.y <= max.comp.y);
    }

    Rect<T> operator +(const Rect<T> &r) const {
        return Rect(min.min(r.min), max.max(r.max));
    }

    Rect<T> operator +(const Vector2D<T> &p) const {
        return Rect(min.min(p), max.max(p));
    }

    Rect<T> &operator +=(const Rect<T> &b) {
        min = min.min(b.min);
        max = max.max(b.max);
        return *this;
    }

    Rect<T> &operator +=(const Vector2D<T> &p) {
        min = min.min(p);
        max = max.max(p);
        return *this;
    }


    const Rect<T> &operator = (const Rect<T> &r) {
        min = r.min;
        max = r.max;
        return *this;
    }

    const Rect<T> &operator = (const Vector2D<T> &v) {
        min = v;
        max = v;
        return *this;
    }


    bool operator == (const Rect<T> &p) const {
        if (p.isEmpty() && isEmpty()) return true;

        return (min == p.min) && (max == p.max);
    }

    T width() const {
        if (isEmpty()) return 0;
        return max.comp.x - min.comp.x;
    }

    T height() const {
        if (isEmpty()) return 0;
        return max.comp.y - min.comp.y;
    }

    void width(T w) {
        max.comp.x = min.comp.x + w;
    }

    void height(T h) {
        max.comp.y = min.comp.y + h;
    }

    T area() const {
        if (isEmpty()) return 0;
        return (max.comp.x-min.comp.x)*(max.comp.y-min.comp.y);
    }

    T perimeter() const {
        if (isEmpty()) return 0;
        return 2.0*((max.comp.x-min.comp.x)+(max.comp.y-min.comp.y));
    }

    Vector2D<T> center() const {
        return Vector2D<T>((max.comp.x+min.comp.x)*0.5, (max.comp.y+min.comp.y)*0.5);
    }

    Size2D<T> size() {
        return Size2D<T>(max.comp.x-min.comp.x, max.comp.y-min.comp.y);
    }
};


} // namespace

#endif /* __FLIN_GEOM_INC__ */
