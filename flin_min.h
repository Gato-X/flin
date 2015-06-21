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

#ifndef _FLIN_MIN_H_
#define _FLIN_MIN_H_

#include <cmath>
#include <cfloat>
#include <iostream>
#include <cstring>


#ifndef FLIN_BASE_TYPE
#define FLIN_BASE_TYPE float
#endif

namespace flin {

typedef FLIN_BASE_TYPE float_t;

enum IntersectionCode{ OUTSIDE, INTERSECTS, INSIDE};


} // namespace

#ifndef M_PI
#define M_PI           3.14159265358979323846264338327
#endif

// Pi/2
#ifndef M_PI_2
#define M_PI_2         1.57079632679489661923
#endif

#include "floattypes.h"
#include "base.h"
#include "dimensions.h"

namespace flin {
typedef Quaternion<float_t> Quaternionf;
typedef Vector2D<float_t> Vector2Df;
typedef Vector3D<float_t> Vector3Df;
typedef Vector4D<float_t> Vector4Df;
typedef Matrix2D<float_t> Matrix2Df;
typedef Matrix3D<float_t> Matrix3Df;
typedef Matrix3H<float_t> Matrix3Hf;

typedef LookAtSetup<float_t> LookAtSetupf;

typedef Size2D<float_t> Size2Df;
typedef Size2D<int> Size2Di;

typedef Rect<float_t> Rectf;
typedef Rect<int> Recti;

typedef AABounds<float_t> AABoundsf;
typedef AABounds<int> AABoundsi;
}

#endif // _FLIN_MIN_H_
