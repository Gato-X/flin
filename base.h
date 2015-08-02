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

#ifndef _FLIN_BASE_H_
#define _FLIN_BASE_H_

namespace flin {

template<typename T> class Vector2D;
template<typename T> class Vector3D;
template<typename T> class Vector4D;
template<typename T> class Matrix2D;
template<typename T> class Matrix3D;
template<typename T> class Matrix3H;
template<typename T> class Quaternion;
template<typename T> class LookAtSetup;

template<class T> struct Size2D;

template<typename T> inline T deg2rad(T deg) {
	return deg * 0.017453293;
}

template<typename T> inline T rad2deg(T rad) {
	return rad * 0.017453293;
}

template <typename T> inline void swap(T &a, T &b) {T c=a; a=b; b=c; }

template<typename T>
class Quaternion {
public:
	union {
		T a[4];
		struct {
			T w,x,y,z;
		} comp;
	};
	Quaternion<T> () {};

	template <typename T2>
	Quaternion<T> (const Quaternion<T2> &from) {
		for (int i = 0; i < 4; ++i)
			a[i] = from.a[i];
	}

	constexpr Quaternion<T> (T w, T x, T y, T z):a{w,x,y,z} {
	}


	static constexpr Quaternion<T> Identity() {
		return Quaternion<T>(1,0,0,0);
	}

	template <typename T2>
	Quaternion<T> &operator = (const Quaternion<T2> &from) {
		for (int i = 0; i < 4; ++i)
			a[i] = from.a[i];

		return *this;
	}

    Quaternion<T> &operator = (T d) {
		comp.w = d;
		comp.x = 0;
		comp.y = 0;
		comp.z = 0;
		return *this;
	}

	template <typename T2>
	Quaternion<T> &operator = (const Matrix3D<T2> &m) {
        T t = m.trace();
        T s;

		if (t > 0) {
			s = sqrt(t + 1.0);  // 2w
			comp.w = 0.5 * s;
			s = 0.5/s;  // 1/(4w)
			comp.x = (m.a[7]-m.a[5])*s;
			comp.y = (m.a[2]-m.a[6])*s;
			comp.z = (m.a[3]-m.a[1])*s;
		} else {
			if (m.a[0] > m.a[4] && m.a[0] > m.a[8]) {
				t = m.a[0] - m.a[4] - m.a[8] + 1.0;
				s = sqrt(t) * 0.5;
				comp.x = s*t;
				comp.y = (m.a[1] + m.a[3]) * s;
				comp.z = (m.a[2] + m.a[6]) * s;
				comp.w = (m.a[5] - m.a[7]) * s;
			} else if (m.a[4] > m.a[0] && m.a[4] > m.a[8]) {
				t = m.a[4] - m.a[0] - m.a[8] + 1.0;
				s = sqrt(t) * 0.5;
				comp.x = (m.a[1] + m.a[3]) * s;
				comp.y = s*t;
				comp.z = (m.a[5] + m.a[7]) * s;
				comp.w = (m.a[6] - m.a[2]) * s;
			} else if (m.a[8] > m.a[0] && m.a[8] > m.a[4]) {
				t =m.a[8] - m.a[0] - m.a[4] + 1.0;
				s = sqrt(t) * 0.5;
				comp.x = (m.a[2] + m.a[6]) * s;
				comp.y = (m.a[5] + m.a[7]) * s;
				comp.z = s*t;
				comp.w = (m.a[1] - m.a[3]) * s;
			}
		}
	}

	template <typename T2>
	Quaternion<T> operator * (const Quaternion<T2> &op) const {
		Quaternion<T> tmp;
		tmp.comp.w = comp.w*op.comp.w - comp.x*op.comp.x - comp.y*op.comp.y - comp.z*op.comp.z;
		tmp.comp.x = comp.w*op.comp.x + comp.x*op.comp.w + comp.y*op.comp.z - comp.z*op.comp.y;
		tmp.comp.y = comp.w*op.comp.y + comp.y*op.comp.w + comp.z*op.comp.x - comp.x*op.comp.z;
		tmp.comp.z = comp.w*op.comp.z + comp.z*op.comp.w + comp.x*op.comp.y - comp.y*op.comp.x;
		return tmp;
	}

    template <typename T2> // rotate a point with a quaternion
    Vector3D<T2> operator * (const Vector3D<T2> &vec) const {
        T2 ww =comp.w*comp.w;
        T2 xx =comp.x*comp.x;
        T2 yy =comp.y*comp.y;
        T2 zz =comp.z*comp.z;
        T2 wx =comp.w*comp.x;
        T2 wy =comp.w*comp.y;
        T2 wz =comp.w*comp.z;
        T2 xy =comp.x*comp.y;
        T2 xz =comp.x*comp.z;
        T2 yz =comp.y*comp.z;
        return Vector3D<T2>(
            (ww+xx-yy-zz)*vec.comp.x + 2*(xy+wz)*vec.comp.y + 2*(xz-wy)*vec.comp.z,
            2*(xy-wz)*vec.comp.x + (ww+yy-xx-zz)*vec.comp.y + 2*(yz+wx)*vec.comp.z,
            2*(xz+wy)*vec.comp.x + 2*(yz-wx)*vec.comp.y + (ww+zz-xx-yy)*vec.comp.z
        );
    }

	template <typename T2>
	Quaternion<T> operator + (const Quaternion<T2> &op) const {
		Quaternion<T> tmp;
		tmp.comp.w = comp.w + op.comp.w;
		tmp.comp.x = comp.x + op.comp.x;
		tmp.comp.y = comp.y + op.comp.y;
		tmp.comp.z = comp.z + op.comp.z;
		return tmp;
	}

	template <typename T2>
	Quaternion<T> operator - (const Quaternion<T> &op) const {
		Quaternion<T> tmp;
		tmp.comp.w = comp.w - op.comp.w;
		tmp.comp.x = comp.x - op.comp.x;
		tmp.comp.y = comp.y - op.comp.y;
		tmp.comp.z = comp.z - op.comp.z;
		return tmp;
	}

	Quaternion<T> operator - () const {
		Quaternion<T> tmp;
		tmp.comp.w =  comp.w;
		tmp.comp.x = - comp.x;
		tmp.comp.y = - comp.y;
		tmp.comp.z = - comp.z;
		return tmp;
	}

	Quaternion<T> flip() {
		comp.x = - comp.x;
		comp.y = - comp.y;
		comp.z = - comp.z;
		return *this;
	}

	Quaternion<T> &setToZero() {
		comp.w = comp.x = comp.y = comp.z = 0;
		return *this;
	}

	Quaternion<T> &setToIdentity() {
		comp.w = 1;
		comp.x = comp.y = comp.z = 0;
		return *this;
	}

    Quaternion<T> &set(T pw, T px, T py, T pz) {
		comp.w = pw;
		comp.x = px;
		comp.y = py;
		comp.z = pz;
		return *this;
	}

	template <typename Numeric> Quaternion<T> &set(const Numeric *elem) {
		comp.w = elem[0];
		comp.x = elem[1];
		comp.y = elem[2];
		comp.z = elem[3];
		return *this;
	}

	template <typename Numeric> void store(Numeric *elem) const {
		elem[0]=comp.w;
		elem[1]=comp.x;
		elem[2]=comp.y;
		elem[3]=comp.z;
	}

	template <typename Numeric> void store(Numeric &pw, Numeric &px, Numeric &py, Numeric &pz) const {
		pw = comp.w;
		px = comp.x;
		py = comp.y;
		pz = comp.z;
	}

	Quaternion<T> &normalize() {
        T mag = magnitude();

	    if (FloatTypes<T>::z(mag)) return *this;

		comp.w /=mag;
		comp.x /=mag;
		comp.y /=mag;
		comp.z /=mag;

		return *this;
	}

    T magnitude() const {
		return sqrt(comp.w*comp.w + comp.x*comp.x + comp.y*comp.y + comp.z*comp.z);
	}

	Quaternion<T> &conjugate() {
		comp.x = -comp.x;
		comp.y = -comp.y;
		comp.z = -comp.z;
		return *this;
	}

	template <typename T2, typename Numeric>
	void toAxisAngle(Vector3D<T2> &axis, Numeric &ang) const {
		Quaternion<T> tmp;
        T cos_a, sin_a;
		tmp = *this;

		tmp.normalize();

		cos_a = tmp.comp.w;
		ang   = acos( cos_a ) * 2.0;
		sin_a = sqrt( 1.0 - cos_a * cos_a );

	    if ( FloatTypes<T>::z(fabs( sin_a )) ) sin_a = 1;

		axis.comp.x = comp.x / sin_a;
		axis.comp.y = comp.y / sin_a;
		axis.comp.z = comp.z / sin_a;
	}


	template <typename T2>
    Quaternion<T> &fromAxisAngle(const Vector3D<T2> &axis, T ang) {
        T cos_a, sin_a;

		ang *=0.5;

		sin_a = sin(ang);
		cos_a = cos(ang);

		comp.x    = axis.comp.x * sin_a;
		comp.y    = axis.comp.y * sin_a;
		comp.z    = axis.comp.z * sin_a;
		comp.w    = cos_a;

		normalize();
		return *this;
	}

	template <typename Numeric>
	Quaternion<T> &fromScaledAxis(const Vector3D<Numeric> &axis) {
        T cos_a, sin_a, ang;

		ang = axis.magnitude();

	    if (FloatTypes<T>::z(ang)) {
			comp.x = 0;
			comp.y = 0;
			comp.z = 0;
			comp.w = 1;
		} else {

			ang *=0.5;

			sin_a = sin(ang) / ang;
			cos_a = cos(ang);

			comp.x    = axis.comp.x * sin_a;
			comp.y    = axis.comp.y * sin_a;
			comp.z    = axis.comp.z * sin_a;
			comp.w    = cos_a;

			normalize();
		}

		return *this;
	}

    Quaternion<T> &fromEulerX(T ang) {
        T cos_a, sin_a;

		ang *=0.5;

		sin_a = sin(ang);
		cos_a = cos(ang);

		comp.x = sin_a;
		comp.y = 0;
		comp.z = 0;
		comp.w = cos_a;

		return *this;
	}

    Quaternion<T> &fromEulerY(T ang) {

        T cos_a, sin_a;

		ang *=0.5;

		sin_a = sin(ang);
		cos_a = cos(ang);

		comp.x = 0;
		comp.y = sin_a;
		comp.z = 0;
		comp.w = cos_a;

		return *this;
	}

    Quaternion<T> &fromEulerZ(T ang) {

        T cos_a, sin_a;

		ang *=0.5;

		sin_a = sin(ang);
		cos_a = cos(ang);

		comp.x = 0;
		comp.y = 0;
		comp.z = sin_a;
		comp.w = cos_a;

		return *this;
	}

	template <typename Numeric>
	void toSpherical(Numeric &lat, Numeric &lng, Numeric &ang) const {
        T cos_a, sin_a;
        T tx,ty,tz;

		cos_a  = comp.w;
		sin_a  = sqrt( 1.0 - cos_a * cos_a );
		ang    = acos( cos_a ) * 2;

		if ( fabs( sin_a ) < 0.0001 ) sin_a = 1;

		tx = comp.x / sin_a;
		ty = comp.y / sin_a;
		tz = comp.z / sin_a;

		lat = -asin( ty );

		if ( tx * tx + tz * tz < 0.0001 )
			lng = 0;
		else
			lng = atan2( tx, tz );

		if ( lng < 0 ) lng += 2.0 * M_PI;

	}

    Quaternion<T> &fromSpherical(T lat, T lng, T ang) {
        T cos_a, sin_a;
        T sin_lat, cos_lat, sin_long, cos_long;

		ang  *=0.5;

		sin_a    = sin( ang );
		cos_a    = cos( ang );

		sin_lat  = sin( lat );
		cos_lat  = cos( lat );

		sin_long = sin( lng );
		cos_long = cos( lng );

		comp.x = sin_a * cos_lat * sin_long;
		comp.y = sin_a * sin_lat;
		comp.z = sin_a * sin_lat * cos_long;
		comp.w = cos_a;

		return *this;
	}
};


template<typename T>
std::ostream& operator <<(std::ostream &f, const Quaternion<T> &q) {
	return f << "Q(" << q.w << ',' << q.x << ',' << q.y <<',' << q.z << ')';
}







//-------------------------------


template<typename T>
class Matrix2D {
public:
    T a[4];
    Matrix2D<T>() {};
    Matrix2D<T>(const Matrix2D<T> &m) {
        a[0] = m.a[0];
        a[1] = m.a[1];
        a[2] = m.a[2];
        a[3] = m.a[3];
    }

	constexpr Matrix2D<T>(T a_, T b_, T c_, T d_):a{a_,b_,c_,d_} {
	}

    static constexpr Matrix2D<T> Zero() {
        return Matrix2D<T>(0,0,0,0);
    }

    static constexpr Matrix2D<T> Identity() {
        return Matrix2D<T>(1,0,0,1);
    }

    template <typename Numeric>
    Matrix2D<T> &operator = (Numeric n) {
        setToZero();
        a[0]=a[3]=n;
    }

    template <typename T2>
    Matrix2D<T> &operator = (const Matrix2D<T2> &m) {
        a[0] = m.a[0];
        a[1] = m.a[1];
        a[2] = m.a[2];
        a[3] = m.a[3];

        return *this;
    }

    template <typename T2>
    Matrix2D<T> operator * (const Matrix2D<T2> &m) const {
        Matrix2D<T> tmp;

        tmp.a[0] = a[0] * m.a[0] + a[1] * m.a[2];
        tmp.a[1] = a[0] * m.a[1] + a[1] * m.a[3];
        tmp.a[2] = a[2] * m.a[0] + a[3] * m.a[2];
        tmp.a[3] = a[2] * m.a[1] + a[3] * m.a[3];

        return tmp;
    }

    template <typename T2>
    Matrix2D<T> &operator  *=  (const Matrix2D<T2> &m) {
        Matrix2D<T> tmp;

        tmp.a[0] = a[0] * m.a[0] + a[1] * m.a[2];
        tmp.a[1] = a[0] * m.a[1] + a[1] * m.a[3];
        tmp.a[2] = a[2] * m.a[0] + a[3] * m.a[2];
        tmp.a[3] = a[2] * m.a[1] + a[3] * m.a[3];

        a[0] = tmp.a[0];
        a[1] = tmp.a[1];
        a[2] = tmp.a[2];
        a[3] = tmp.a[3];

        return *this;
    }


    template <typename T2>
    Vector2D<T2> operator  *  (const Vector2D<T2> &v) const {
        Vector2D<T2> tmp;

        tmp.a[0] = v.a[0] * a[0] + v.a[1] * a[1];
        tmp.a[1] = v.a[0] * a[2] + v.a[1] * a[3];

        return tmp;
    }

    template <typename T2>
    Matrix2D<T> operator  +  (const Matrix2D<T2> &m) const {
        Matrix2D<T> tmp;

        tmp.a[0] = a[0] + m.a[0];
        tmp.a[1] = a[1] + m.a[1];
        tmp.a[2] = a[2] + m.a[2];
        tmp.a[3] = a[3] + m.a[3];

        return tmp;
    }

    template <typename T2>
    Matrix2D<T> operator  - (Matrix2D<T2> &m) const {
        Matrix2D<T> tmp;

        tmp.a[0] = a[0] - m.a[0];
        tmp.a[1] = a[1] - m.a[1];
        tmp.a[2] = a[2] - m.a[2];
        tmp.a[3] = a[3] - m.a[3];

        return tmp;
    }

    Matrix2D<T> operator - () const {
        Matrix2D<T> tmp;

        tmp.a[0] = -a[0];
        tmp.a[1] = -a[1];
        tmp.a[2] = -a[2];
        tmp.a[3] = -a[3];

        return tmp;
    }

    template <typename Numeric>
    Matrix2D<T> &set(const Numeric *elem) {
        a[0] = elem[0];
        a[1] = elem[1];
        a[2] = elem[2];
        a[3] = elem[3];

        return *this;
    }

    template <typename Numeric>
    void store(Numeric *elem) const {
        elem[0] = a[0];
        elem[1] = a[1];
        elem[2] = a[2];
        elem[3] = a[3];
    }

    template<typename Numeric> // note this will load only 9 numbers!
    Matrix2D<T> &setFromGL(Numeric *elem) {// by columns instead of by rows (As for OpenGL)
        a[0] = elem[0];
        a[1] = elem[2];
        a[2] = elem[1];
        a[3] = elem[3];
        return *this;
    }

    template<typename Numeric> // note this will store only 9 numbers!
    void storeToGL(Numeric *elem) const {
        elem[0] = a[0];
        elem[1] = a[2];
        elem[2] = a[1];
        elem[3] = a[3];
    }

    T det() const {
        return  a[0]*a[3]-a[1]*a[2];
    }


    T trace() const {
        return a[0] + a[3];
    }

    Matrix2D<T> &setToIdentity() {
        a[0]=a[3]=1;
        a[1]=a[2]=0;
        return *this;
    }

    Matrix2D<T> &setToZero() {
        a[0]=a[1]=a[2]=a[3]=0;
        return *this;
    }

    Matrix2D<T> getInverse() const { // gets the inverse of a matrix (doesn't overtwirte the current matrix)
        Matrix2D<T> mat;
        T d = a[0] * a[3] - a[1] * a[2];
	
	    if (FloatTypes<T>::z(d)) return Matrix2D<T>::Zero();

		d = 1.0/d;

        mat.a[0] = a[3] * d;
        mat.a[1] =-a[1] * d;
        mat.a[2] =-a[2] * d;
        mat.a[3] = a[0] * d;

        return mat;
    }

    Matrix2D<T> &transpose() {
        T t;
        swap(a[1],a[2]);
        return *this;
    }

    template <typename T2>
    Matrix2D<T> &map(Matrix2D<T2> &from, Matrix2D<T2> &to) {// sets the matrix to one which maps the space defined by m1 into a space defined by m2
        *this = to * from.getInverse();
        return *this;
    }

    //         |  cos(A)  -sin(A)   0 |
    //     M = |  sin(A)   cos(A)   0 |
    //         |  0        0        1 |
    Matrix2D<T> &toRotatio(T angle) {
        T c = cos(angle);
        T s = sin(angle);
        a[0] = c;
        a[1] = -s;
        a[2] = s;
        a[3] = c;
        return *this;
    }

    Matrix2D<T> &toScaling(T x, T y, T z) { // set this to a scaling matrix
        a[0] = x;
        a[3] = y;
        a[1] = a[2] = 0;
        return *this;
    }


    Matrix2D<T> &scale(T x, T y, T z) { // scale current matrix
        a[0]*=x;
        a[1]*=y;
        a[2]*=z;
        a[3]*=x;
        return *this;
    }

    Matrix2D<T> &scale(T s) {   // scale current matrix
        a[0]*=s;
        a[1]*=s;
        a[2]*=s;
        a[3]*=s;
        return *this;
    }

    Vector3D<T> getColumn(int i) const {
        return Vector3D<T>(a[i], a[i+2]);
    }

    Vector2D<T> getRow(int i) const {
        int idx = i * 2;
        return Vector2D<T>(a[idx], a[idx+1]);
    }

    void setColumn(int i, const Vector3D<T> &v) {
        a[i] = v.comp.x;
        a[i+2] = v.comp.y;
    }

    void setRow(int i, const Vector3D<T> &v) {
        int idx = i * 2;
        a[idx] = v.comp.x;
        a[idx+1] = v.comp.y;
    }


    inline T elem(int i) const {
        return a[i];
    }
    T &operator [] (int i) {
        return a[i];
    }
    T operator [] (int i) const {
        return a[i];
    }

    T* ptr() {
        return a;
    }

    const T* ptr() const {
        return a;
    }
};




//-------------------------------

template<typename T>
class Matrix3D {
public:
	T a[9];
	Matrix3D<T>() {};
	Matrix3D<T>(const Matrix3D<T> &m) {
		for (int i = 0; i < 9; i++) a[i] = m.a[i];
	}

	constexpr Matrix3D<T>(T a0, T a1, T a2, T a3, T a4, T a5, T a6, T a7, T a8)
		:a{a0,a1,a2,a3,a4,a5,a6,a7,a8} {
	}

	explicit Matrix3D<T>(const Matrix3H<T> &m) {
		int i,j,k,l;
		k=0;
		l=0;

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				a[k]=m.a[l];
				k++;
				l++;
			}

			l++;
		}
	}

	template <typename T2>
	explicit Matrix3D<T>(const Quaternion<T2> &quat) {
		*this = quat;
	}

	static constexpr Matrix3D<T> Zero() {
		return Matrix3D<T>(0,0,0,0,0,0,0,0,0);
	}

	static constexpr Matrix3D<T> Identity() {
		return  Matrix3D<T>(1,0,0,0,1,0,0,0,1);
	}

	template <typename Numeric>
	Matrix3D<T> &operator = (Numeric n) {
		setToZero();
		a[0]=a[4]=a[8]=n;
	}

	template <typename T2>
	Matrix3D<T> &operator = (const Matrix3D<T2> &m) {
		for (int i = 0; i < 9; i++) a[i] = m.a[i];

		return *this;
	}

	template <typename T2>
	Matrix3D<T> &operator = (const Quaternion<T2> &quat) {
		T x2      = quat.comp.x + quat.comp.x;
		T y2      = quat.comp.y + quat.comp.y;
		T z2	  = quat.comp.z + quat.comp.z;

		T xx      = quat.comp.x * x2;
		T xy      = quat.comp.x * y2;
		T xz      = quat.comp.x * z2;
		T yy      = quat.comp.y * y2;
		T yz      = quat.comp.y * z2;
		T zz      = quat.comp.z * z2;

		T xw      = quat.comp.w * x2;
		T yw      = quat.comp.w * y2;
		T zw      = quat.comp.w * z2;

		a[0]  = 1 - ( yy + zz );
		a[1]  =     ( xy - zw );
		a[2]  =     ( xz + yw );
		a[3]  =     ( xy + zw );
		a[4]  = 1 - ( xx + zz );
		a[5]  =     ( yz - xw );
		a[6]  =     ( xz - yw );//-
		a[7]  =     ( yz + xw );//+
		a[8]  = 1 - ( xx + yy );//+

		return *this;
	}

	template <typename T2> // extract the 3x3 upper-right submatrix from m
	Matrix3D<T> &operator << (const Matrix3H<T2> &m) {
		int i,j,k,l;
		k=0;
		l=0;

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				a[k]=m.a[l];
				k++;
				l++;
			}
			l++;
		}
		return *this;
	}



	template <typename T2>
	Matrix3D<T> operator  *  (const Matrix3D<T2> &m) const {
		Matrix3D<T> tmp;
		int i,j,k,o,p,q;

		o = p = q = 0;

		for ( i =0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				tmp.a[o] = 0;

				for (k = 0; k < 3; k++) {
					tmp.a[o] += a[p] * m.a[q];
					p++;
					q+=3;
				}

				o++;
				p-=3;
				q-=8;
			}

			p+=3;
			q-=3;
		}

		return tmp;
	}

    template <typename T2>
    Matrix3D<T> &operator  *=  (const Matrix3D<T2> &m) {
        Matrix3D<T> tmp;
        int i,j,k,o,p,q;

        o = p = q = 0;

        for ( i =0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                tmp.a[o] = 0;

                for (k = 0; k < 3; k++) {
                    tmp.a[o] += a[p] * m.a[q];
                    p++;
                    q+=3;
                }

                o++;
                p-=3;
                q-=8;
            }

            p+=3;
            q-=3;
        }

        for (i = 0; i < 9; i++) a[i] = tmp.a[i];
        return *this;
    }

    template <typename T2>
    Matrix3D<T> &operator  +=  (const Matrix3D<T2> &m) {
		for (int i = 0; i < 9; i++) {
			a[i] += m.a[i];
		}
        return *this;
    }



	template <typename T2>
	Vector3D<T2> operator  *  (const Vector3D<T2> &v) const {
		Vector3D<T2> tmp;
		tmp.a[0] = v.a[0] * a[0] + v.a[1] * a[1] + v.a[2] * a[2];
		tmp.a[1] = v.a[0] * a[3] + v.a[1] * a[4] + v.a[2] * a[5];
		tmp.a[2] = v.a[0] * a[6] + v.a[1] * a[7] + v.a[2] * a[8];
		return tmp;
	}

	template <typename T2>
	Matrix3D<T> operator  +  (const Matrix3D<T2> &m) const {
		Matrix3D<T> tmp;
		int j;

		for (j = 0; j < 9; j++) tmp.a[j] = a[j] + m.a[j];

		return tmp;
	}

	template <typename T2>
	Matrix3D<T> operator  -   (Matrix3D<T2> &m) const {
		Matrix3D<T> tmp;
		int j;

		for (j = 0; j < 9; j++) tmp.a[j] = a[j] - m.a[j];

		return tmp;
	}

	Matrix3D<T> operator - () const {
		Matrix3D<T> tmp;
		int j;

		for (j = 0; j < 9; j++) tmp.a[j] = - a[j];

		return tmp;
	}

	template <typename Numeric>
	Matrix3D<T> &set(const Numeric *elem) {
		for (int i = 0; i < 9; i++) a[i] = elem[i];

		return *this;
	}

	template <typename Numeric>
	void store(Numeric *elem) const {
		for (int i = 0; i < 9; i++) elem[i] = a[i];
	}

	template<typename Numeric> // note this will load only 9 numbers!
	Matrix3D<T> &setFromGL(Numeric *elem) {// by columns instead of by rows (As for OpenGL)
		int i,j,k,l;

		k=l=0;

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				a[l]=elem[k];
				k+=3;
				l++;
			}

			k-=8;
		}

		return *this;
	}

	template<typename Numeric> // note this will store only 9 numbers!
	void storeToGL(Numeric *elem) const {
		int i,j,k,l;

		k=l=0;

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				elem[k] = a[l];
				k+=3;
				l++;
			}

			k-=8;
		}
	}

	T det() const {
		return a[0] * ( a[4]*a[8] - a[7]*a[5] )
		     - a[1] * ( a[3]*a[8] - a[6]*a[5] )
		     + a[2] * ( a[3]*a[7] - a[6]*a[4] );
	}


    T trace() const {
		return a[0] + a[4] + a[8];
	}

	Matrix3D<T> &setToIdentity() {
		for (int i=0; i < 9; i++) a[i] = 0;

		a[0]=a[4]=a[8]= 1;
		return *this;
	}

	Matrix3D<T> &setToZero() {
		for (int i=0; i < 9; i++) a[i] = 0;

		return *this;
	}

	Matrix3D<T> getInverse() const { // gets the inverse of a matrix (doesn't overtwirte the current matrix)
        T d = det();
		Matrix3D<T> mat;

	    if (FloatTypes<T>::z(d)) {
	        return Matrix3D<T>::Zero();
		}

		d = 1.0/d;

		mat.a[0] =  ( a[4]*a[8] - a[5]*a[7] ) * d;
		mat.a[1] = -( a[1]*a[8] - a[7]*a[2] ) * d;
		mat.a[2] =  ( a[1]*a[5] - a[4]*a[2] ) * d;

		mat.a[3] = -( a[3]*a[8] - a[5]*a[6] ) * d;
		mat.a[4] =  ( a[0]*a[8] - a[6]*a[2] ) * d;
		mat.a[5] = -( a[0]*a[5] - a[3]*a[2] ) * d;

		mat.a[6] =  ( a[3]*a[7] - a[6]*a[4] ) * d;
		mat.a[7] = -( a[0]*a[7] - a[6]*a[1] ) * d;
		mat.a[8] =  ( a[0]*a[4] - a[1]*a[3] ) * d;

		return mat;
	}

	Matrix3D<T> &transpose() {
        T t;
		swap(a[1],a[3]);
		swap(a[2],a[6]);
		swap(a[5],a[7]);
		return *this;
	}

	template <typename T2>
	Matrix3D<T> &map(Matrix3D<T2> &from, Matrix3D<T2> &to) {// sets the matrix to one which maps the space defined by m1 into a space defined by m2
		*this = to * from.getInverse();
		return *this;
	}

	//        |  1  0       0      |
	//    M = |  0  cos(A) -sin(A) |
	//        |  0  sin(A)  cos(A) |
    Matrix3D<T> &toRotationX(T angle) {
		setToIdentity();
        T c = cos(angle);
        T s = sin(angle);
		a[4] = c;
		a[5] = -s;
		a[7] = s;
		a[8] = c;
		return *this;
	}

	//         |  cos(A)  0   sin(A) |
	//     M = |  0       1   0      |
	//         | -sin(A)  0   cos(A) |
    Matrix3D<T> &toRotationY(T angle) {
		setToIdentity();
        T c = cos(angle);
        T s = sin(angle);
		a[0] = c;
		a[2] = s;
		a[6] = -s;
		a[8] = c;
		return *this;
	}

	//         |  cos(A)  -sin(A)   0 |
	//     M = |  sin(A)   cos(A)   0 |
	//         |  0        0        1 |
    Matrix3D<T> &toRotationZ(T angle) {
		setToIdentity();
        T c = cos(angle);
        T s = sin(angle);
		a[0] = c;
		a[1] = -s;
		a[3] = s;
		a[4] = c;
		return *this;
	}

    Matrix3D<T> &toScaling(T x, T y, T z) { // set this to a scaling matrix
		setToZero();
		a[0] = x;
		a[4] = y;
		a[8] = z;
		return *this;
	}


    Matrix3D<T> &scale(T x, T y, T z) { // scale current matrix
		a[0]*=x;
		a[1]*=y;
		a[2]*=z;
		a[3]*=x;
		a[4]*=y;
		a[5]*=z;
		a[6]*=x;
		a[7]*=y;
		a[8]*=z;
		return *this;
	}

    Matrix3D<T> &scale(T s) {   // scale current matrix
		a[0]*=s;
		a[1]*=s;
		a[2]*=s;
		a[3]*=s;
		a[4]*=s;
		a[5]*=s;
		a[6]*=s;
		a[7]*=s;
		a[8]*=s;
		return *this;
	}

	Vector3D<T> getColumn(int i) const {
		return Vector3D<T>(a[i], a[i+3], a[i+6]);
	}

	Vector3D<T> getRow(int i) const {
		int idx = i * 3;
		return Vector3D<T>(a[idx], a[idx+1], a[idx+2]);
	}

	void setColumn(int i, const Vector3D<T> &v) {
		a[i] = v.comp.x;
		a[i+3] = v.comp.y;
		a[i+6] = v.comp.z;
	}

	void setRow(int i, const Vector3D<T> &v) {
		int idx = i * 3;
		a[idx] = v.comp.x;v;
		a[idx+1] = v.comp.y;
		a[idx+2] = v.comp.z;
	}


	inline T elem(int i) const {
		return a[i];
	}
	T &operator [] (int i) {
		return a[i];
	}
	T operator [] (int i) const {
		return a[i];
	}

	T* ptr() {
		return a;
	}

	const T* ptr() const {
	    return a;
	}

	// axis MUST be normalized
	template <typename T2>
	void fromAxisAngle(const Vector3D<T2> &axis, T ang) {
		T s = sin(axis.comp.angle);
		T c = cos(axis.comp.angle);
		T t = 1.0 - c;

		a[0] = c + axis.comp.x*axis.comp.x*t;
		a[4] = c + axis.comp.y*axis.comp.y*t;
		a[8] = c + axis.comp.z*axis.comp.z*t;

		T tmp1 = axis.comp.x*axis.comp.y*t;
		T tmp2 = axis.comp.z*s;
		a[3] = tmp1 + tmp2;
		a[1] = tmp1 - tmp2;
		tmp1 = axis.comp.x*axis.comp.z*t;
		tmp2 = axis.comp.y*s;
		a[6] = tmp1 - tmp2;
		a[2] = tmp1 + tmp2;
		tmp1 = axis.comp.y*axis.comp.z*t;
		tmp2 = axis.comp.x*s;
		a[7] = tmp1 + tmp2;
		a[5] = tmp1 - tmp2;
	}


	// modified Gramm-Schmidt orthonormalize
	void orthonormalize() {
		for (int i = 0; i < 9; i+=3) {
			T n = sqrt(a[i]*a[i] + a[i+1]*a[i+1] + a[i+2]*a[i+2]);

			if (n > 0) {
				n = 1.0/n;

				a[i] *= n;
				a[i+1] *= n;
				a[i+2] *= n;
			
				for (int j = i+3; j < 9; j+=3) {
					T d = a[j]*a[i] + a[j+1]*a[i+1] + a[j+2]*a[i+2];
					a[j]   -= a[i] * d;
					a[j+1] -= a[i+1] * d;
					a[j+2] -= a[i+2] * d;
				}
			}
		}
	}

	T distanceSq(const Matrix3D<T> &other) {
		T sum = 0;
		for (int i =0; i < 9; i++) {
			T dif = a[i] - other.a[i];
			sum += dif*dif;
		}
		return sum;
	}

	T frobenius() {
		T sum = 0;
		for (int i =0; i < 9; i++) {
			sum += a[i]*a[i];
		}
		return sqrt(sum);
	}


	// decomposes this so into a rotation r and scaling s: this = r s
	void decomposeRS(Matrix3D<T> &r, Matrix3D<T> &s) {
		Matrix3D<T> m(*this);

		if (m.det() < 0) {
			m[0] = -m[0];
			m[1] = -m[1];
			m[2] = -m[2];
		}
		int i;

		// perform polar decomposition on m
		for (i = 0; i < 50; i++) {
			Matrix3D<T> n(m.getInverse());
			n.transpose();
			m += n;
			m.scale(0.5);
			if (m.det() == 1.0) break;
		}

		m.orthonormalize(); // m is a rotation-only matrix (ie det(m) = 1 and m x m^-1 = I)

		r = m;

		m = r.getInverse();

		s = m * (*this); // scale is "this" without the rotation part
	}
};

template<typename T>
std::ostream& operator <<(std::ostream &f, const Matrix3D<T> &m) {
	return f << "{{" << m.elem(0) << ',' << m.elem(1) << ',' << m.elem(2)  << "}," << \
	       "{" << m.elem(3) << ',' << m.elem(4) << ',' << m.elem(5)  << "}," << \
	       "{" << m.elem(6) << ',' << m.elem(7) << ',' << m.elem(8)  << "}}";
}

template<typename T>
class Matrix3H {
public:
	T a[16];
	Matrix3H<T>() {};

	Matrix3H<T>(const Matrix3H<T> &m) {
		for (int i = 0; i < 16; i++) a[i] = m.a[i];
	}

	constexpr Matrix3H<T>(T a0, T a1, T a2, T a3, T a4, T a5, T a6, T a7, T a8, T a9, T a10, T a11, T a12, T a13, T a14, T a15):
		a{a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15} {
	}

	explicit Matrix3H<T>(const Matrix3D<T> &m) {
		m[3] = m[7] = m[11] = m[12] = m[13] = m[14] = 0;
		m[15] = 1;
		*this << m;
	}

	template <typename T2>
	explicit Matrix3H<T>(const Quaternion<T2> &quat) {
		T x2 = quat.x + quat.x;
		T y2 = quat.y + quat.y;
		T z2 = quat.z + quat.z;

		T xx = quat.x * x2;
		T xy = quat.x * y2;
		T xz = quat.x * z2;
		T yy = quat.y * y2;
		T yz = quat.y * z2;
		T zz = quat.z * z2;

		T xw = quat.w * x2;
		T yw = quat.w * y2;
		T zw = quat.w * z2;

		a[0]  = 1 - ( yy + zz );
		a[1]  =     ( xy - zw );
		a[2]  =     ( xz + yw );
		a[4]  =     ( xy + zw );
		a[5]  = 1 - ( xx + zz );
		a[6]  =     ( yz - xw );
		a[8]  =     ( xz - yw );
		a[9]  =     ( yz + xw );
		a[10] = 1 - ( xx + yy );
		a[3]=a[7]=a[11]=a[12]=a[13]=a[14]=0;
		a[15] =1;
	}

	static Matrix3H<T> Zero() {
		return Matrix3H<T>(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	}

	static Matrix3H<T> Identity() {
		return Matrix3H<T>(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
	}


	template <typename T2>
	Matrix3H<T> &operator << (const Matrix3D<T2> &m) {
		int i,j,k,l;
		k=0;
		l=0;

		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				a[l]=m.a[k];
				k++;
				l++;
			}

			l++;
		}

		return *this;
	}

	template <typename T2>
	Matrix3H<T> &operator = (const Matrix3D<T2> &m) {
		a[12]=a[13]=a[14]=0;
		a[15]=1;
		(*this) << m;
		return *this;
	}


	template <typename T2>
	Matrix3H<T> &operator = (const Matrix3H<T2> &m) {
		for (int i = 0; i < 16; i++) a[i] = m.a[i];

		return *this;
	}

	template <typename Numeric>
	Matrix3H<T> &operator = (Numeric n) {
		setToZero();
		a[0]=a[5]=a[10]=a[15]=n;
	}

	template <typename T2>
	Matrix3H<T> operator  *  (const Matrix3H<T2> &m) const {
		Matrix3H<T> tmp;
		int i,j,k,o,p,q;

		o = p = q = 0;

		for ( i =0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				tmp.a[o] = 0;

				for (k = 0; k < 4; k++) {
					tmp.a[o] += a[p] * m.a[q];
					p++;
					q+=4;
				}

				o++;
				p-=4;
				q-=15;
			}

			p+=4;
			q-=4;
		}

		return tmp;
	}

    template <typename T2>
    Matrix3H<T> &operator  *=  (const Matrix3H<T2> &m) {
        Matrix3H<T> tmp;
        int i,j,k,o,p,q;

        o = p = q = 0;

        for ( i =0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                tmp.a[o] = 0;

                for (k = 0; k < 4; k++) {
                    tmp.a[o] += a[p] * m.a[q];
                    p++;
                    q+=4;
                }

                o++;
                p-=4;
                q-=15;
            }

            p+=4;
            q-=4;
        }

        for (i = 0; i < 16; i++) a[i] = tmp.a[i];

        return *this;
    }


    template <typename T2>
    Matrix3H<T> &operator  +=  (const Matrix3H<T2> &m) {
		for (int i = 0; i < 16; i++) {
			a[i] += m.a[i];
		}
        return *this;
    }


	template <typename T2>
	Vector4D<T2> operator * (const Vector4D<T2> &v) const {
		Vector4D<T> tmp;
		tmp.a[0] = v.a[0] * a[0] + v.a[1] * a[1] + v.a[2] * a[2] + v.a[3] * a[3];
		tmp.a[1] = v.a[0] * a[4] + v.a[1] * a[5] + v.a[2] * a[6] + v.a[3] * a[7];
		tmp.a[2] = v.a[0] * a[8] + v.a[1] * a[9] + v.a[2] * a[10]+ v.a[3] * a[11];
		tmp.a[3] = v.a[0] * a[12]+ v.a[1] * a[13]+ v.a[2] * a[14]+ v.a[3] * a[15];
		return tmp;
	}


	template <typename T2>
	Vector3D<T2> operator * (const Vector3D<T2> &v) const {
		Vector3D<T> tmp;
		T f = 1.0 / (v.a[0] * a[12] + v.a[1] * a[13] + v.a[2] * a[14]+ a[15]); // consider this a homogeneous matrix
		tmp.a[0] = (v.a[0] * a[0] + v.a[1] * a[1] + v.a[2] * a[2] + a[3]) * f;
		tmp.a[1] = (v.a[0] * a[4] + v.a[1] * a[5] + v.a[2] * a[6] + a[7]) * f;
		tmp.a[2] = (v.a[0] * a[8] + v.a[1] * a[9] + v.a[2] * a[10]+ a[11]) * f;
		return tmp;
	}

	template <typename T2>
	Matrix3H<T> operator  +  (const Matrix3H<T2> &m) const {
		Matrix3H<T> tmp;
		int j;

		for (j = 0; j < 16; j++) tmp.a[j] = a[j] + m.a[j];

		return tmp;
	}

	template <typename T2>
	Matrix3H<T> operator  -  (const Matrix3H<T2> &m) const {
		Matrix3H<T> tmp;
		int j;

		for (j = 0; j < 16; j++) tmp.a[j] = a[j] - m.a[j];

		return tmp;
	}

	Matrix3H<T> operator - () const {
		Matrix3H<T> tmp;
		int j;

		for (j = 0; j < 16; j++) tmp.a[j] = - a[j];

		return tmp;
	}


	template <typename Numeric>
	Matrix3H<T> &set(const Numeric *elem) {
		for (int i = 0; i < 16; i++) a[i] = elem[i];

		return *this;
	}

	template <typename Numeric>
	void store(Numeric *elem) const {
		for (int i = 0; i < 16; i++) elem[i] = a[i];
	}

	template<typename Numeric>// loads 16 numbers
	Matrix3H<T> &setFromGL(const Numeric *elem) {// by columns instead of by rows (As for OpenGL)
		int i,j,k,l;

		k=l=0;

		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				a[l]=elem[k];
				k+=4;
				l++;
			}

			k-=15;
		}

		return *this;
	}

	template<typename Numeric> // stores 16 numbers
	void storeToGL(Numeric *elem) const {
		int i,j,k,l;

		k=l=0;

		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				elem[k] = a[l];
				k+=4;
				l++;
			}

			k-=15;
		}
	}


	T det() const {
		return  a[1]*((a[11]*a[14] - a[10]*a[15])*a[4] + a[7]*(a[10]*a[12] \
		              - a[14]*a[8]) + a[6]*(-a[11]*a[12] + a[15]*a[8])) +\
		        a[3]*(a[5]*(-a[10]*a[12] + a[14]*a[8]) + a[6]*(-a[13]*a[8] \
		                + a[12]*a[9]) + a[4]*(a[10]*a[13] - a[14]*a[9])) + \
		        a[0]*((-a[11]*a[14] + a[10]*a[15])*a[5] + a[7]*(-a[10]*a[13] \
		                + a[14]*a[9]) + a[6]*(a[11]*a[13] - a[15]*a[9])) + \
		        a[2]*(a[5]*(a[11]*a[12] - a[15]*a[8]) + a[7]*(a[13]*a[8] \
		                - a[12]*a[9]) + a[4]*(-a[11]*a[13] + a[15]*a[9]));
	}

    T trace() const {
		return a[0] + a[5] + a[10] + a[15];
	}

	Matrix3H<T> &setToIdentity() {
		for (int i=0; i < 16; i++) a[i] = 0;

		a[0]=a[5]=a[10]=a[15] = 1;
		return *this;
	}

	Matrix3H<T> &setToZero() {
		for (int i=0; i < 16; i++) a[i] = 0;

		return *this;
	}

	Matrix3H<T> getInverse() const { // gets the inverse of a matrix (doesn't overtwirte the current matrix)
		// adapted from MESA's GLU
        T cof[16], det;
		int i;

		cof[0] =   a[5]*a[10]*a[15] - a[5]*a[11]*a[14] - a[9]*a[6]*a[15]
		           + a[9]*a[7]*a[14] + a[13]*a[6]*a[11] - a[13]*a[7]*a[10];
		cof[4] =  -a[4]*a[10]*a[15] + a[4]*a[11]*a[14] + a[8]*a[6]*a[15]
		          - a[8]*a[7]*a[14] - a[12]*a[6]*a[11] + a[12]*a[7]*a[10];
		cof[8] =   a[4]*a[9]*a[15] - a[4]*a[11]*a[13] - a[8]*a[5]*a[15]
		           + a[8]*a[7]*a[13] + a[12]*a[5]*a[11] - a[12]*a[7]*a[9];
		cof[12] = -a[4]*a[9]*a[14] + a[4]*a[10]*a[13] + a[8]*a[5]*a[14]
		          - a[8]*a[6]*a[13] - a[12]*a[5]*a[10] + a[12]*a[6]*a[9];
		cof[1] =  -a[1]*a[10]*a[15] + a[1]*a[11]*a[14] + a[9]*a[2]*a[15]
		          - a[9]*a[3]*a[14] - a[13]*a[2]*a[11] + a[13]*a[3]*a[10];
		cof[5] =   a[0]*a[10]*a[15] - a[0]*a[11]*a[14] - a[8]*a[2]*a[15]
		           + a[8]*a[3]*a[14] + a[12]*a[2]*a[11] - a[12]*a[3]*a[10];
		cof[9] =  -a[0]*a[9]*a[15] + a[0]*a[11]*a[13] + a[8]*a[1]*a[15]
		          - a[8]*a[3]*a[13] - a[12]*a[1]*a[11] + a[12]*a[3]*a[9];
		cof[13] =  a[0]*a[9]*a[14] - a[0]*a[10]*a[13] - a[8]*a[1]*a[14]
		           + a[8]*a[2]*a[13] + a[12]*a[1]*a[10] - a[12]*a[2]*a[9];
		cof[2] =   a[1]*a[6]*a[15] - a[1]*a[7]*a[14] - a[5]*a[2]*a[15]
		           + a[5]*a[3]*a[14] + a[13]*a[2]*a[7] - a[13]*a[3]*a[6];
		cof[6] =  -a[0]*a[6]*a[15] + a[0]*a[7]*a[14] + a[4]*a[2]*a[15]
		          - a[4]*a[3]*a[14] - a[12]*a[2]*a[7] + a[12]*a[3]*a[6];
		cof[10] =  a[0]*a[5]*a[15] - a[0]*a[7]*a[13] - a[4]*a[1]*a[15]
		           + a[4]*a[3]*a[13] + a[12]*a[1]*a[7] - a[12]*a[3]*a[5];
		cof[14] = -a[0]*a[5]*a[14] + a[0]*a[6]*a[13] + a[4]*a[1]*a[14]
		          - a[4]*a[2]*a[13] - a[12]*a[1]*a[6] + a[12]*a[2]*a[5];
		cof[3] =  -a[1]*a[6]*a[11] + a[1]*a[7]*a[10] + a[5]*a[2]*a[11]
		          - a[5]*a[3]*a[10] - a[9]*a[2]*a[7] + a[9]*a[3]*a[6];
		cof[7] =   a[0]*a[6]*a[11] - a[0]*a[7]*a[10] - a[4]*a[2]*a[11]
		           + a[4]*a[3]*a[10] + a[8]*a[2]*a[7] - a[8]*a[3]*a[6];
		cof[11] = -a[0]*a[5]*a[11] + a[0]*a[7]*a[9] + a[4]*a[1]*a[11]
		          - a[4]*a[3]*a[9] - a[8]*a[1]*a[7] + a[8]*a[3]*a[5];
		cof[15] =  a[0]*a[5]*a[10] - a[0]*a[6]*a[9] - a[4]*a[1]*a[10]
		           + a[4]*a[2]*a[9] + a[8]*a[1]*a[6] - a[8]*a[2]*a[5];

		det = a[0]*cof[0] + a[1]*cof[4] + a[2]*cof[8] + a[3]*cof[12];

		if (FloatTypes<T>::z(det))
			return Matrix3H<T>::Zero();

		det = 1.0 / det;

		Matrix3H<T> tmp;

		for (i = 0; i < 16; i++)
			tmp.a[i] = cof[i] * det;

		return tmp;
	}

	Matrix3H<T> &transpose() {
        T t;
		swap(a[1],a[4]);
		swap(a[2],a[8]);
		swap(a[6],a[9]);
		swap(a[3],a[12]);
		swap(a[7],a[13]);
		swap(a[11],a[14]);
		return *this;
	}

	template <typename T2>
	Matrix3H<T> &map(Matrix3H<T2> &from, Matrix3H<T2> &to) {// sets the matrix to one which maps the space defined by m1 into a space defined by m2
		*this = to * from.getInverse();
		return *this;
	}


    Matrix3H<T> &toTranslation(const Vector3D<T> &vec) {
        setToIdentity();
        a[3] = vec.a[0];
        a[7] = vec.a[1];
        a[11]= vec.a[2];
        return *this;
    }

    Matrix3H<T> &toTranslation(const Vector4D<T> &vec) {
        setToIdentity();
        a[3] = vec.a[0];
        a[7] = vec.a[1];
        a[11]= vec.a[2];
        a[15]= vec.a[3];
        return *this;
    }

	//        |  1  0       0      0 |
	//    M = |  0  cos(A) -sin(A) 0 |
	//        |  0  sin(A)  cos(A) 0 |
	//        |  0  0       0      1 |
    Matrix3H<T> &toRotationX(T angle) {
		setToZero();
        T c = cos(angle);
        T s = sin(angle);
		a[5] = c;
		a[6] = -s;
		a[9] = s;
		a[10] = c;
		a[0] = a[15] = 1;
		return *this;
	}

	//        |  cos(A)  0   sin(A) 0 |
	//    M = |  0       1   0      0 |
	//        | -sin(A)  0   cos(A) 0 |
	//        |  0       0   0      1 |
    Matrix3H<T> &toRotationY(T angle) {
		setToZero();
        T c = cos(angle);
        T s = sin(angle);
		a[0] = c;
		a[2] = s;
		a[6] = -s;
		a[8] = c;
		a[5] = a[15] = 1;
		return *this;
	}

	//        |  cos(A)  -sin(A)   0  0 |
	//    M = |  sin(A)   cos(A)   0  0 |
	//        |  0        0        1  0 |
	//        |  0        0        0  1 |
    Matrix3H<T> &toRotationZ(T angle) {
		setToZero();
        T c = cos(angle);
        T s = sin(angle);
		a[0] = c;
		a[1] = -s;
		a[3] = s;
		a[4] = c;
		a[10] = a[15] = 1;
		return *this;
	}

    Matrix3H<T> &toScaling(T x, T y, T z, T w=1.0) { // set this to a scaling matrix
		setToZero();
		a[0] = x;
		a[5] = y;
		a[10] = z;
		a[15] = w;
		return *this;
	}


    Matrix3H<T> &scale(T x, T y, T z, T w=1.0) { // scale current matrix
		a[0]*=x;
		a[1]*=y;
		a[2]*=z;
		a[4]*=x;
		a[5]*=y;
		a[6]*=z;
		a[8]*=x;
		a[9]*=y;
		a[10]*=z;
		a[12]*=x;
		a[13]*=y;
		a[14]*=z;

		if (w != 1) {
			a[3]*=w;
			a[7]*=w;
			a[11]*=w;
			a[15]*=w;
		}

		return *this;
	}

    Matrix3H<T> &scale(T s) {   // scale current matrix
		a[0]*=s;
		a[1]*=s;
		a[2]*=s;
		a[3]*=s;
		a[4]*=s;
		a[5]*=s;
		a[6]*=s;
		a[7]*=s;
		a[8]*=s;
		a[9]*=s;
		a[10]*=s;
		a[11]*=s;
		a[12]*=s;
		a[13]*=s;
		a[14]*=s;
		a[15]*=s;
		return *this;
	}

	Matrix3H<T> &toPerspectiveProjection(T h_fov, const Size2D<T> &vp_size, T near, T far = FLT_MAX, const Vector2D<T> &view_center_offset = Vector2D<T>(0,0), T pixel_aspect = 1.0);

	Matrix3H<T> &toLookAt(const Vector3D<T> &position, const Vector3D<T> &target, const LookAtSetup<T> &las);

	Vector4D<T> getColumn(int i) const {
		return Vector4D<T>(a[i], a[i+4], a[i+8], a[i+12]);
	}

	Vector4D<T> getRow(int i) const {
		int idx = i * 4;
		return Vector4D<T>(a[idx], a[idx+1], a[idx+2], a[idx+3]);
	}

	void setColumn(int i, const Vector4D<T> &v) {
		a[i] = v.comp.x;
		a[i+4] = v.comp.y;
		a[i+8] = v.comp.z;
		a[i+12] = v.comp.w;
	}


	void setColumn(int i, const Vector3D<T> &v) {
		a[i] = v.comp.x;
		a[i+4] = v.comp.y;
		a[i+8] = v.comp.z;
		a[i+12] = (i==3)?1:0;
	}

	void setRow(int i, const Vector4D<T> &v) {
		int idx = i * 4;
		a[idx] = v.comp.x;
		a[idx+1] = v.comp.y;
		a[idx+2] = v.comp.z;
		a[idx+3] = v.comp.w;
	}

	void setRow(int i, const Vector3D<T> &v) {
		int idx = i * 4;
		a[idx] = v.comp.x;
		a[idx+1] = v.comp.y;
		a[idx+2] = v.comp.z;
		a[idx+3] = (i==3)?1:0;
	}

	inline T elem(int i) const {
		return a[i];
	}

	T &operator [] (int i) {
		return a[i];
	}

	T operator [] (int i) const {
		return a[i];
	}

	T* ptr() {
        return &a[0];
	}

	const T* ptr() const {
        return &a[0];
	}

};

template<typename T>
std::ostream& operator <<(std::ostream &f, const Matrix3H<T> &m) {
	return f << "{{" << m.elem(0) << ',' << m.elem(1) << ',' << m.elem(2) << ',' << m.elem(3)  << "}," << \
	       "{" << m.elem(4) << ',' << m.elem(5) << ',' << m.elem(6) << ',' << m.elem(7) << "}," << \
	       "{" << m.elem(8) << ',' << m.elem(9) << ',' << m.elem(10) << ',' << m.elem(11) << "}," << \
	       "{" << m.elem(12) << ',' << m.elem(13) << ',' << m.elem(14) << ',' << m.elem(15) << "}}";
}


template<typename T>
class Vector2D {
public:
	union {
		T a[2];
		struct {
			T x,y;
		} comp;
	};

	static constexpr Vector2D<T> I() {
		return Vector2D<T>(1,0);
	}

	static constexpr Vector2D<T> J() {
		return Vector2D<T>(0,1);
	}

	static constexpr Vector2D<T> Zero() {
		return Vector2D<T>(0,0);
	}

	Vector2D<T> () {};

	constexpr Vector2D<T> (T px, T py):a{px,py} {
	}

	Vector2D<T> (const Vector2D<T> &from) {
        a[0] = from.a[0];
        a[1] = from.a[1];
	}

	template <typename T2>
	Vector2D<T> &operator = (const Vector2D<T2> &from) {
		comp.x = from.comp.x;
		comp.y = from.comp.y;
		return *this;
	}

	template <typename T2>
	Vector2D<T> operator + (const Vector2D<T2> &op) const {
		Vector2D<T> tmp;
		tmp.comp.x = comp.x + op.comp.x;
		tmp.comp.y = comp.y + op.comp.y;
		return tmp;
	}

	template <typename T2>
	Vector2D<T> operator - (const Vector2D<T2> &op) const {
		Vector2D<T> tmp;
		tmp.comp.x = comp.x - op.comp.x;
		tmp.comp.y = comp.y - op.comp.y;
		return tmp;
	}

	Vector2D<T> operator - () const {
		Vector2D<T> tmp;
		tmp.comp.x = - comp.x;
		tmp.comp.y = - comp.y;
		return tmp;
	}

	Vector2D<T> operator * (T scale) const {
		Vector2D<T> tmp;
		tmp.comp.x = comp.x * scale;
		tmp.comp.y = comp.y * scale;
		return tmp;
	}

	Vector2D<T> operator / (T scale) const {
		Vector2D<T> tmp;
		tmp.comp.x = comp.x / scale;
		tmp.comp.y = comp.y / scale;
		return tmp;
	}

	void flip() {
	    comp.x = -comp.x;
	    comp.y = -comp.y;
	}

	Vector2D<T> &setToZero() {
		comp.x = comp.y = 0;
		return *this;
	}

	Vector2D<T> toI() {
		comp.x = 1;
		comp.y = 0;
		return *this;
	}

	Vector2D<T> toJ() {
		comp.x = 0;
		comp.y = 1;
		return *this;
	}

    Vector2D<T> &set(T px, T py) {
		comp.x = px;
		comp.y = py;
		return *this;
	}

	template <typename Numeric>
	Vector2D<T> &set(const Numeric *elem) {
		comp.x = elem[0];
		comp.y = elem[1];
		return *this;
	}

	template <typename Numeric>
	void store(Numeric *elem) const {
		elem[0]=comp.x;
		elem[1]=comp.y;
	}

	template <typename Numeric>
	void store(Numeric &px, Numeric &py) const {
		px = comp.x;
		py = comp.y;
	}

	bool normalize() {
        T mag = magnitude();

	    if (FloatTypes<T>::z(mag)) return false;

		comp.x /=mag;
		comp.y /=mag;

	    return true;
	}

	template <typename T2>
	Vector2D<T> &operator  += (const Vector2D<T2> &v) {
		comp.x += v.comp.x;
		comp.y += v.comp.y;
		return *this;
	}

	template <typename Numeric>
	Vector2D<T> &operator  += (const Numeric *elem) {
		comp.x += elem[0];
		comp.y += elem[1];
		return *this;
	}

	template <typename Numeric>
	Vector2D<T> &operator *= (Numeric n) {
		comp.x *= n;
		comp.y *= n;
		return *this;
	}

	template <typename Numeric>
	Vector2D<T> &operator /= (Numeric n) {
		comp.x /= n;
		comp.y /= n;
		return *this;
	}

	template <typename T2>
	Vector2D<T> &operator  -= (const Vector2D<T2> &v) {
		comp.x -= v.comp.x;
		comp.y -= v.comp.y;
		return *this;
	}

	template <typename Numeric>
	Vector2D<T> &operator  -= (const Numeric *elem) {
		comp.x -= elem[0];
		comp.y -= elem[1];
		return *this;
	}

    T  magnitude() const {
		return sqrt(comp.x*comp.x+comp.y*comp.y);
	}

    T  magnitudeSq() const {
		return comp.x*comp.x+comp.y*comp.y;
	}

	template <typename T2>
    T dot(const Vector2D<T2> &v) const {
		return comp.x*v.comp.x + comp.y*v.comp.y;
	}


	template <typename T2>
    T distanceSq(const Vector2D<T2> &v) const {
        T dx = comp.x - v.comp.x;
        T dy = comp.y - v.comp.y;
		return dx*dx+dy*dy;
	}

	template <typename T2>
    T distance(const Vector2D<T2> &v) const {
        T dx = comp.x - v.comp.x;
        T dy = comp.y - v.comp.y;
		return sqrt(dx*dx+dy*dy);
	}

	template <typename T2>
    T distance(const Vector2D<T2> &v, int axis) const {
		return a[axis] - v.a[axis];
	}

	template <typename T2>
	T cross(const Vector2D<T2> &v) const {
		return comp.x*v.comp.y-comp.y*v.comp.x;
	}

	// robust angle between two vectors
	template <typename T2>
	T angle(const Vector2D<T2> &v) const {
		return atan2(comp.x*v.comp.y-comp.y*v.comp.x, comp.x*v.comp.x+comp.y*v.comp.y);
	}

	template <typename T2>
	Vector2D<T> min(const Vector2D<T2> &v) const {
		Vector2D<T> tmp(
		    a[0] < v.a[0] ? a[0] : v.a[0],
		    a[1] < v.a[1] ? a[1] : v.a[1]
		);
		return tmp;
	}

	template <typename T2>
	Vector2D<T> max(const Vector2D<T2> &v) const {
		Vector2D<T> tmp(
		    a[0] > v.a[0] ? a[0] : v.a[0],
		    a[1] > v.a[1] ? a[1] : v.a[1]
		);
		return tmp;
	}

	// projects this onto the vector v
	Vector2D<T> projectOnto(const Vector2D<T> &v) const {
		Vector2D tmp(v);
		tmp.normalize();
		return tmp * dot(tmp);
	}


	T *ptr() {
		return &comp.x;
	};

	const T* ptr() const {
	    return &comp.x;
	}



	T &operator [](int i) {
		return a[i];
	}

	T operator [](int i) const {
		return a[i];
	}

	int getMaxCoord() const {
		if (a[0] >= a[1])
			return 0;
		else
			return 1;
	}

	static bool compareByX(const Vector2D &p1, const Vector2D &p2) {
		return p1.a[0] < p2.a[0];
	}

	static bool compareByY(const Vector2D &p1, const Vector2D &p2) {
		return p1.a[1] < p2.a[1];
	}

	bool operator == (const Vector2D<T> &v) const {
	    return FloatTypes<T>::eq(v.comp.x,comp.x) && FloatTypes<T>::eq(v.comp.y,comp.y);
	}
};

template<typename T>
std::ostream& operator <<(std::ostream &f, const Vector2D<T> &v) {
	return f << "{" << v.comp.x << ',' << v.comp.y << '}';
}



template<typename T>
class Vector3D {
public:
	union {
		T a[3];
		struct {
			T x,y,z;
		} comp;
	};

	static constexpr Vector3D<T> I() {
		return Vector3D<T>(1,0,0);
	}

	static constexpr Vector3D<T> J() {
		return Vector3D<T>(0,1,0);
	}

	static constexpr Vector3D<T> K() {
		return Vector3D<T>(0,0,1);
	}

	static constexpr Vector3D<T> Zero() {
		return Vector3D<T>(0,0,0);
	}


	Vector3D<T> () {};

	constexpr Vector3D<T> (T px, T py, T pz):a{px,py,pz} {
	}

	Vector3D<T> (const Vector3D<T> &from) {
		for (int i = 0; i < 3; ++i)
			a[i] = from.a[i];
	}

	// ignore "w" component
	void fromVector4D(const Vector4D<T> &from) {
		for (int i = 0; i < 3; ++i)
			a[i] = from.a[i];
	}


	template <typename T2>
	Vector3D<T> &operator = (const Vector3D<T2> &from) {
		comp.x = from.comp.x;
		comp.y = from.comp.y;
		comp.z = from.comp.z;
		return *this;
	}

	template <typename T2>
	Vector3D<T> operator + (const Vector3D<T2> &op) const {
		Vector3D<T> tmp;
		tmp.comp.x = comp.x + op.comp.x;
		tmp.comp.y = comp.y + op.comp.y;
		tmp.comp.z = comp.z + op.comp.z;
		return tmp;
	}

	template <typename T2>
	Vector3D<T> operator - (const Vector3D<T2> &op) const {
		Vector3D<T> tmp;
		tmp.comp.x = comp.x - op.comp.x;
		tmp.comp.y = comp.y - op.comp.y;
		tmp.comp.z = comp.z - op.comp.z;
		return tmp;
	}

	Vector3D<T> operator - () const {
		Vector3D<T> tmp;
		tmp.comp.x = - comp.x;
		tmp.comp.y = - comp.y;
		tmp.comp.z = - comp.z;
		return tmp;
	}

	Vector3D<T> operator * (T scale) const {
		Vector3D<T> tmp;
		tmp.comp.x = comp.x * scale;
		tmp.comp.y = comp.y * scale;
		tmp.comp.z = comp.z * scale;
		return tmp;
	}

	Vector3D<T> operator / (T scale) const {
		Vector3D<T> tmp;
		tmp.comp.x = comp.x / scale;
		tmp.comp.y = comp.y / scale;
		tmp.comp.z = comp.z / scale;
		return tmp;
	}

	void flip() {
	    comp.x = -comp.x;
	    comp.y = -comp.y;
	    comp.z = -comp.z;
	}

	Vector3D<T> &setToZero() {
		comp.x = comp.y = comp.z = 0;
		return *this;
	}

	Vector3D<T> toI() {
		comp.x = 1;
		comp.y = 0;
		comp.z = 0;
		return *this;
	}

	Vector3D<T> toJ() {
		comp.x = 0;
		comp.y = 1;
		comp.z = 0;
		return *this;
	}

	Vector3D<T> toK() {
		comp.x = 0;
		comp.y = 0;
		comp.z = 1;
		return *this;
	}


    Vector3D<T> &set(T px, T py, T pz) {
		comp.x = px;
		comp.y = py;
		comp.z = pz;
		return *this;
	}

	template <typename Numeric>
	Vector3D<T> &set(const Numeric *elem) {
		comp.x = elem[0];
		comp.y = elem[1];
		comp.z = elem[2];
		return *this;
	}

	template <typename Numeric>
	void store(Numeric *elem) const {
		elem[0]=comp.x;
		elem[1]=comp.y;
		elem[2]=comp.z;
	}

	template <typename Numeric>
	void store(Numeric &px, Numeric &py, Numeric &pz) const {
		px = comp.x;
		py = comp.y;
		pz = comp.z;
	}


	bool normalize() {
        T mag = magnitude();

	    if (FloatTypes<T>::z(mag)) return false;

		comp.x /=mag;
		comp.y /=mag;
		comp.z /=mag;

	    return true;
	}

	template <typename T2>
	Vector3D<T> &operator  += (const Vector3D<T2> &v) {
		comp.x += v.comp.x;
		comp.y += v.comp.y;
		comp.z += v.comp.z;
		return *this;
	}

	template <typename Numeric>
	Vector3D<T> &operator  += (const Numeric *elem) {
		comp.x += elem[0];
		comp.y += elem[1];
		comp.z += elem[2];
		return *this;
	}

	template <typename Numeric>
	Vector3D<T> &operator *= (Numeric n) {
		comp.x *= n;
		comp.y *= n;
		comp.z *= n;
		return *this;
	}

	template <typename Numeric>
	Vector3D<T> &operator /= (Numeric n) {
		comp.x /= n;
		comp.y /= n;
		comp.z /= n;
		return *this;
	}

	template <typename T2>
	Vector3D<T> &operator  -= (const Vector3D<T2> &v) {
		comp.x -= v.comp.x;
		comp.y -= v.comp.y;
		comp.z -= v.comp.z;
		return *this;
	}

	template <typename Numeric>
	Vector3D<T> &operator  -= (const Numeric *elem) {
		comp.x -= elem[0];
		comp.y -= elem[1];
		comp.z -= elem[2];
		return *this;
	}

    T  magnitude() const {
		return sqrt(comp.x*comp.x+comp.y*comp.y+comp.z*comp.z);
	}

    T  magnitudeSq() const {
		return comp.x*comp.x+comp.y*comp.y+comp.z*comp.z;
	}

	template <typename T2>
    T dot(const Vector3D<T2> &v) const {
		return comp.x*v.comp.x + comp.y*v.comp.y + comp.z*v.comp.z;
	}


	template <typename T2>
    T distanceSq(const Vector3D<T2> &v) const {
        T dx = comp.x - v.comp.x;
        T dy = comp.y - v.comp.y;
        T dz = comp.z - v.comp.z;
		return dx*dx+dy*dy+dz*dz;
	}

	template <typename T2>
    T distance(const Vector3D<T2> &v) const {
        T dx = comp.x - v.comp.x;
        T dy = comp.y - v.comp.y;
        T dz = comp.z - v.comp.z;
		return sqrt(dx*dx+dy*dy+dz*dz);
	}

	template <typename T2>
    T distance(const Vector3D<T2> &v, int axis) const {
		return a[axis] - v.a[axis];
	}

	template <typename T2>
	Vector3D<T> cross(const Vector3D<T2> &v) const {
		Vector3D<T> tmp;
		tmp.comp.x= comp.y*v.comp.z-comp.z*v.comp.y;
		tmp.comp.y= comp.z*v.comp.x-comp.x*v.comp.z;
		tmp.comp.z= comp.x*v.comp.y-comp.y*v.comp.x;
		return tmp;
	}

	template <typename T2>
	T angle(const Vector3D<T2> &v) const {
		return atan2((this->cross(v)).magnitude(), this->dot(v));
	}

	template <typename T2>
	Vector3D<T> min(const Vector3D<T2> &v) const {
		Vector3D<T> tmp(
		    a[0] < v.a[0] ? a[0] : v.a[0],
		    a[1] < v.a[1] ? a[1] : v.a[1],
		    a[2] < v.a[2] ? a[2] : v.a[2]
		);
		return tmp;
	}

	template <typename T2>
	Vector3D<T> max(const Vector3D<T2> &v) const {
		Vector3D<T> tmp(
		    a[0] > v.a[0] ? a[0] : v.a[0],
		    a[1] > v.a[1] ? a[1] : v.a[1],
		    a[2] > v.a[2] ? a[2] : v.a[2]
		);
		return tmp;
	}

	// projects this onto the vector v
	Vector3D<T> projectOnto(const Vector3D<T> &v) const {
		Vector3D tmp(v);
		tmp.normalize();

		return tmp * dot(tmp);

	}


	T   *ptr() {
		return &comp.x;
	};

	const T* ptr() const {
	    return &comp.x;
	}

	T &operator [](int i) {
		return a[i];
	}
	T operator [](int i) const {
		return a[i];
	}

	int getMaxCoord() const {
		if (a[0] > a[1]) {
			if (a[0] > a[2]) {
				return 0;
			} else {
				return 2;
			}
		} else {
			if (a[1] > a[2]) {
				return 1;
			} else {
				return 2;
			}
		}
	}

    int getLargestCoord() const {
        T x = a[0]<0?-a[0]:a[0];
        T y = a[1]<0?-a[1]:a[1];
        T z = a[2]<0?-a[2]:a[2];

        if (x > y) {
            if (x > z) {
                return 0;
            } else {
                return 2;
            }
        } else {
            if (y > z) {
                return 1;
            } else {
                return 2;
            }
        }
    }

	static bool compareByX(const Vector3D &p1, const Vector3D &p2) {
		return p1.a[0] < p2.a[0];
	}

	static bool compareByY(const Vector3D &p1, const Vector3D &p2) {
		return p1.a[1] < p2.a[1];
	}

	static bool compareByZ(const Vector3D &p1, const Vector3D &p2) {
		return p1.a[2] < p2.a[2];
	}

	bool operator == (const Vector3D<T> &v) const {
	    return FloatTypes<T>::eq(v.comp.x,comp.x) && FloatTypes<T>::eq(v.comp.y,comp.y) && FloatTypes<T>::eq(v.comp.z,comp.z);
	}
};




template<typename T>
std::ostream& operator <<(std::ostream &f, const Vector3D<T> &v) {
	return f << "{" << v.comp.x << ',' << v.comp.y << ',' << v.comp.z << '}';
}


template<typename T>
class Vector4D {
public:
	union {
		T a[4];
		struct {
			T x,y,z,w;
		} comp;
	};

	static constexpr Vector4D<T> I() {
		return Vector4D<T>(1,0,0,0);
	}

	static constexpr Vector4D<T> J() {
		return Vector4D<T>(0,1,0,0);
	}

	static constexpr Vector4D<T> K() {
		return Vector4D<T>(0,0,1,0);
	}

	static constexpr Vector4D<T> W() {
		return Vector4D<T>(0,0,0,1);
	}

	static constexpr Vector4D<T> Zero() {
		return Vector4D<T>(0,0,0,0);
	}

	Vector4D<T> () {};

	Vector4D<T> (T px, T py, T pz, T pw):a{px,py,pz,pw} {
	}

	Vector4D<T> (const Vector4D<T> &from) {
		comp.x = from.comp.x;
		comp.y = from.comp.y;
		comp.z = from.comp.z;
		comp.w = from.comp.w;
	}

	template <typename T2>
	void fromVector3D(const Vector3D<T2> &from, T2 pw = 1) {
		for (int i = 0; i < 3; ++i)
			a[i] = from.a[i];

		comp.w = pw;
	}

	template <typename T2>
	Vector4D<T> &operator = (const Vector4D<T2> &from) {
		for (int i = 0; i < 4; ++i)
			a[i] = from.a[i];

		return *this;
	}

	template <typename T2>
	Vector4D<T> operator + (const Vector4D<T2> &op) const {
		Vector4D<T> tmp;
		tmp.comp.x = comp.x + op.comp.x;
		tmp.comp.y = comp.y + op.comp.y;
		tmp.comp.z = comp.z + op.comp.z;
		tmp.comp.w = comp.w + op.comp.w;
		return tmp;
	}

	template <typename T2>
	Vector4D<T> operator - (const Vector4D<T2> &op) const {
		Vector4D<T> tmp;
		tmp.comp.x = comp.x - op.comp.x;
		tmp.comp.y = comp.y - op.comp.y;
		tmp.comp.z = comp.z - op.comp.z;
		tmp.comp.w = comp.w - op.comp.w;
		return tmp;
	}

	Vector4D<T> operator - () const {
		Vector4D<T> tmp;
		tmp.comp.x = - comp.x;
		tmp.comp.y = - comp.y;
		tmp.comp.z = - comp.z;
		tmp.comp.w = - comp.w;
		return tmp;
	}

	Vector4D<T> operator * (T scale) const {
		Vector4D<T> tmp;
		tmp.comp.x = comp.x * scale;
		tmp.comp.y = comp.y * scale;
		tmp.comp.z = comp.z * scale;
		tmp.comp.w = comp.w * scale;
		return tmp;
	}

	Vector4D<T> operator / (T scale) const {
		Vector4D<T> tmp;
		tmp.comp.x = comp.x / scale;
		tmp.comp.y = comp.y / scale;
		tmp.comp.z = comp.z / scale;
		tmp.comp.w = comp.w / scale;
		return tmp;
	}

	void flip() {
	    comp.x = -comp.x;
	    comp.y = -comp.y;
	    comp.z = -comp.z;
	    comp.w = -comp.w;
	}


	Vector4D<T> &setToZero() {
		comp.x = comp.y = comp.z = comp.w = 0;
		return *this;
	}

    Vector4D<T> &set(T px, T py, T pz, T pw) {
		comp.x = px;
		comp.y = py;
		comp.z = pz;
		comp.w = pw;
		return *this;
	}

	template <typename Numeric>
	Vector4D<T> &set(const Numeric *elem) {
		comp.x = elem[0];
		comp.y = elem[1];
		comp.z = elem[2];
		comp.w = elem[3];
		return *this;
	}

	template <typename Numeric>
	void store(Numeric *elem) const {
		elem[0]=comp.x;
		elem[1]=comp.y;
		elem[2]=comp.z;
		elem[3]=comp.w;
	}

	template <typename Numeric>
	void store(Numeric &px, Numeric &py, Numeric &pz, Numeric &pw) const {
		px = comp.x;
		py = comp.y;
		pz = comp.z;
		pw = comp.w;
	}

	bool normalize() {
        T mag = magnitude();

	    if (FloatTypes<T>::z(mag)) return true;

		comp.x /=mag;
		comp.y /=mag;
		comp.z /=mag;
		comp.w /=mag;

	    return false;
	}

	template <typename T2>
	Vector4D<T> &operator  += (const Vector4D<T2> &v) {
		comp.x += v.comp.x;
		comp.y += v.comp.y;
		comp.z += v.comp.z;
		comp.w += v.comp.w;
		return *this;
	}

	template <typename Numeric>
	Vector4D<T> &operator  += (const Numeric *elem) {
		comp.x += elem[0];
		comp.y += elem[1];
		comp.z += elem[2];
		comp.w += elem[3];
		return *this;
	}

	template <typename Numeric>
	Vector4D<T> &operator *= (Numeric n) {
		comp.x *= n;
		comp.y *= n;
		comp.z *= n;
		comp.w *= n;
		return *this;
	}

	template <typename Numeric>
	Vector4D<T> &operator /= (Numeric n) {
		comp.x /= n;
		comp.y /= n;
		comp.z /= n;
		comp.w /= n;
		return *this;
	}

	template <typename T2>
	Vector4D<T> &operator  -= (const Vector4D<T2> &v) {
		comp.x -= v.comp.x;
		comp.y -= v.comp.y;
		comp.z -= v.comp.z;
		comp.w -= v.comp.w;
		return *this;
	}

	template <typename Numeric>
	Vector4D<T> &operator  -= (const Numeric *elem) {
		comp.x -= elem[0];
		comp.y -= elem[1];
		comp.z -= elem[2];
		comp.w -= elem[3];
		return *this;
	}

    T  magnitude() const {
		return sqrt(comp.x*comp.x+comp.y*comp.y+comp.z*comp.z+comp.w*comp.w);
	}

	template <typename T2>
    T dot(const Vector4D<T2> &v) const {
		return comp.x*v.comp.x + comp.y*v.comp.y + comp.z*v.comp.z + comp.w*v.comp.w;
	}


	template <typename T2>
    T distanceSq(const Vector4D<T2> &v) const {
        T dx = comp.x - v.comp.x;
        T dy = comp.y - v.comp.y;
        T dz = comp.z - v.comp.z;
        T dw = comp.w - v.comp.w;
		return dx*dx+dy*dy+dz*dz+dw*dw;
	}

	template <typename T2>
    T distance(const Vector4D<T2> &v) const {
        T dx = comp.x - v.comp.x;
        T dy = comp.y - v.comp.y;
        T dz = comp.z - v.comp.z;
        T dw = comp.w - v.comp.w;
		return sqrt(dx*dx+dy*dy+dz*dz+dw*dw);
	}


	template <typename T2>
	Vector4D<T> min(const Vector4D<T2> &v) const {
		Vector4D<T> tmp(
		    a[0] < v.a[0] ? a[0] : v.a[0],
		    a[1] < v.a[1] ? a[1] : v.a[1],
		    a[2] < v.a[2] ? a[2] : v.a[2],
		    a[3] < v.a[3] ? a[3] : v.a[3]
		);
		return tmp;
	}

	template <typename T2>
	Vector4D<T> max(const Vector4D<T2> &v) const {
		Vector4D<T> tmp(
		    a[0] > v.a[0] ? a[0] : v.a[0],
		    a[1] > v.a[1] ? a[1] : v.a[1],
		    a[2] > v.a[2] ? a[2] : v.a[2],
		    a[3] > v.a[3] ? a[3] : v.a[3]
		);
		return tmp;
	}

	Vector3D<T> project() const {
		Vector3D<T> tmp;

		if (comp.w == 0) {
			tmp.comp.x = comp.x;
			tmp.comp.y = comp.y;
			tmp.comp.z = comp.z;
		} else {
			tmp.comp.x = comp.x/comp.w;
			tmp.comp.y = comp.y/comp.w;
			tmp.comp.z = comp.z/comp.w;
		}

		return tmp;
	}

	T   *ptr() {
		return &comp.x;
	};

	const T* ptr() const {
	    return &comp.x;
	}

	T &operator [](int i) {
		return a[i];
	}

	T operator [](int i) const {
		return a[i];
	}

	bool operator == (const Vector4D<T> &v) const {
		return fabs(v.comp.x-comp.x)<(FLT_EPSILON*10)  && fabs(v.comp.y-comp.y)<(FLT_EPSILON*10) && fabs(v.comp.z-comp.z)<(FLT_EPSILON*10) && fabs(v.comp.w-comp.w)<(FLT_EPSILON*10);
	}
};


template<typename T>
std::ostream& operator <<(std::ostream &f, const Vector4D<T> &v) {
	return f << "{" << v.comp.x << ',' << v.comp.y << ',' << v.comp.z << ',' << v.comp.w << '}';
}


//--
template<typename T>
Matrix3H<T> &Matrix3H<T>::toPerspectiveProjection(T h_fov, const Size2D<T> &vp_size, T near, T far, const Vector2D<T> &view_center_offset, T pixel_aspect) {
	T M,N;
	T phi = 1.0/tan(deg2rad(h_fov)*0.5);
	T view_aspect = vp_size.w / vp_size.h;

	if (far > (FLT_MAX/2)) {
	    M = -1.0;
	    N = -2.0*near;
	} else {
	    M = -(far+near)/(far-near);
	    N = -(2.0*far*near)/(far-near);
	}

	setToZero();
	a[0] = phi * pixel_aspect / view_aspect;
	a[2] = view_aspect * view_center_offset.comp.x * phi / near;
	a[5] = phi;
	a[6] = view_center_offset.comp.y * phi / near;
	a[10] = M;
	a[11] = N;
	a[14] = -1.0;

	return *this;
}

template<typename T>
struct LookAtSetup {
	// use setThresholdAngle to set these up
	float thd;
	float t_fac;

	Vector3D<T> world_up; // must always be normalized
	Vector3D<T> world_front;

	LookAtSetup(float angle = 0.08):world_up(0,1,0),world_front(0,0,1) { setThresholdAngle(angle); } // ~ 5 degrees

	void setThresholdAngle(float angle){ // in radians
		thd = sin(angle);
		float d = thd / sin(3.0*M_PI / 4.0 - angle);
		t_fac = 1 - d/sqrt(2);
	}

	void setFront(const Vector3D<T> &vec) {
		world_front = vec;
		world_front.normalize();
	}

	void setUp(const Vector3D<T> &vec) {
		world_up = vec;
		world_up.normalize();
	}
};



template<typename T>
Matrix3H<T> &Matrix3H<T>::toLookAt(const Vector3D<T> &position, const Vector3D<T> &target, const LookAtSetup<T> &las) {
	setToZero();


	Vector3D<T> cam_in(target - position);

	cam_in.normalize();

	Vector3D<T> cam_right(cam_in.cross(las.world_up)); // s

	float r_mag = cam_right.magnitude();

	// the following is to avoid the singularity when parallel to up

	if (r_mag < las.thd) {
		float t =  (r_mag / las.thd);

		t = (1-las.t_fac)*t + las.t_fac;

		Vector3D<T> new_up(las.world_up * t + las.world_front * (1-t));
		new_up.normalize();

		cam_right = cam_in.cross(new_up);

		r_mag = cam_right.magnitude();
		
		if (r_mag < (las.thd/10) && (r_mag < 0.01)) {
			cam_right = cam_in.cross(las.world_front);
			r_mag = cam_right.magnitude();
		}
	}

	//----

	cam_right*=(1/r_mag);

	Vector3D<T> cam_up(cam_right.cross(cam_in)); // u

	setRow(0,cam_right);
	setRow(1,cam_up);
	setRow(2,-cam_in);

	Vector3D<T> t(0,0,0);

	setRow(3,t);

	t = (*this)*(-position);

	setColumn(3,t);

	return *this;
}



} // namespace

#endif // _FLIN_BASE_H_
