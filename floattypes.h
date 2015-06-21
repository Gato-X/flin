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

#ifndef _FLIN_FLOATTYPES_H_
#define _FLIN_FLOATTYPES_H_

#include <float.h>

namespace flin {

template<class T>
class FloatTypes {
    public:
    static const float epsilon;
    static const float max;
    static const float min;

    inline static bool eq(T a, T b) {return ((a-b)<= FLT_EPSILON) && ((a-b)>=-FLT_EPSILON); }
    inline static bool neq(T a, T b) {return ((a-b)> FLT_EPSILON) || ((a-b)<-FLT_EPSILON); }
    inline static bool z(T a) { return a>=-FLT_EPSILON && a<=FLT_EPSILON; }
    inline static bool nz(T a) { return a<-FLT_EPSILON || a >FLT_EPSILON; }
};

template<>
class FloatTypes<float> {
    public:
    static const float epsilon;
    static const float max;
    static const float min;

    inline static bool eq(float a, float b) {return ((a-b)<= epsilon) && ((a-b)>=-epsilon); }
    inline static bool neq(float a, float b) {return ((a-b)> epsilon) || ((a-b)<-epsilon); }
    inline static bool z(float a) { return a>=-epsilon && a<=epsilon; }
    inline static bool nz(float a) { return a<-epsilon || a >epsilon; }
};


template<>
class FloatTypes<double> {
    public:
    static const double epsilon;
    static const double max;
    static const double min;

    inline static bool eq(double a, double b) {return ((a-b)<= epsilon) && ((a-b)>=-epsilon); }
    inline static bool neq(double a, double b) {return ((a-b)> epsilon) || ((a-b)<-epsilon); }
    inline static bool z(double a) { return a>=-epsilon && a<=epsilon; }
    inline static bool nz(double a) { return a<-epsilon || a >epsilon; }
};


#ifdef LDBL_EPSILON

template<>
class FloatTypes<long double> {
    public:
    static const long double epsilon;
    static const long double max;
    static const long double min;

    inline static bool eq(long double a, long double b) {return ((a-b)<= epsilon) && ((a-b)>=-epsilon); }
    inline static bool neq(long double a, long double b) {return ((a-b)> epsilon) || ((a-b)<-epsilon); }
    inline static bool z(long double a) { return a>=-epsilon && a<=epsilon; }
    inline static bool nz(long double a) { return a<-epsilon || a >epsilon; }
};



#endif

}

#endif // _FLIN_FLOATTYPES_H_
