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

#include "floattypes.h"

namespace flin{

const float FloatTypes<float>::epsilon=(FLT_EPSILON*10.0);
const float FloatTypes<float>::max=FLT_MAX;
const float FloatTypes<float>::min=FLT_MIN;

const double FloatTypes<double>::epsilon=(DBL_EPSILON*10);
const double FloatTypes<double>::max=DBL_MAX;
const double FloatTypes<double>::min=DBL_MIN;

#ifdef LDBL_EPSILON
const long double FloatTypes<long double>::epsilon=(LDBL_EPSILON*10);
const long double FloatTypes<long double>::max=LDBL_MAX;
const long double FloatTypes<long double>::min=LDBL_MIN;
#endif

}
