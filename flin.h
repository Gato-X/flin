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

#ifndef __LINEAR_INC__
#define __LINEAR_INC__

#include <cmath>
#include <iostream>
#include <cstring>

#ifndef M_PI
#define M_PI           3.14159265358979323846264338327
#endif

// Pi/2
#ifndef M_PI_2
#define M_PI_2         1.57079632679489661923
#endif

namespace lin {

template<typename T>
inline T deg2rad(T ang) {
		return ang * (M_PI/180.0);
}

template<typename T>
inline T rad2deg(T ang) {
		return ang * (180.0/M_PI);
}


template<typename T> class Vector3D;
template<typename T> class Vector4D;
template<typename T> class Matrix3D;
template<typename T> class Matrix4D;
template<typename T> class Quaternion;


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
		Quaternion<T> (const Quaternion<T2> &from){
				for (int i = 0; i < 4; ++i) 
					a[i] = from.a[i];
			}

		template <typename T2>
		Quaternion<T> &operator = (const Quaternion<T2> &from) {
				for (int i = 0; i < 4; ++i) 
					a[i] = from.a[i];
				return *this;
			}

		Quaternion<T> &operator = (double d) {
				comp.w = d;
				comp.x = 0;
				comp.y = 0;
				comp.z = 0;
				return *this;
			}

		template <typename T2>
		Quaternion<T> &operator = (const Matrix3D<T2> &m){
				double tr = m.trace();
				double s;
				
				if (tr > 0) {
					s = 0.5 / sqrt(tr);
					comp.w = 0.25 / s;
					comp.x = (m.a[7] - m.a[5]) * s;
					comp.y = (m.a[2] - m.a[6]) * s;
					comp.z = (m.a[3] - m.a[1]) * s;
				} else {
					if (m.a[0] > m.a[4] && m.a[0] > m.a[8]) {
						s = sqrt(m.a[0] - m.a[4] - m.a[8])*2;
						comp.x = 0.5 / s;
						comp.y = (m.a[1] + m.a[3]) * s;
						comp.z = (m.a[2] + m.a[6]) * s;
						comp.w = (m.a[5] + m.a[7]) * s;
					} else if (m.a[4] > m.a[0] && m.a[4] > m.a[8]) {
						s = sqrt(m.a[4] - m.a[0] - m.a[8])*2;
						comp.x = (m.a[1] + m.a[3]) * s;
						comp.y = 0.5 / s;
						comp.z = (m.a[5] + m.a[7]) * s;
						comp.w = (m.a[2] + m.a[6]) * s;
					} else if (m.a[8] > m.a[0] && m.a[8] > m.a[4]) {
						s = sqrt(m.a[8] - m.a[0] - m.a[4])*2;
						comp.x = (m.a[2] + m.a[6]) * s;
						comp.y = (m.a[5] + m.a[7]) * s;
						comp.z = 0.5 / s;
						comp.w = (m.a[1] + m.a[3]) * s;
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

		Quaternion<T> &zero() {
				comp.w = comp.x = comp.y = comp.z = 0;
				return *this;
			}

		Quaternion<T> &set(double pw, double px, double py, double pz) {
				comp.w = pw;
				comp.x = px;
				comp.y = py;
				comp.z = pz;
				return *this;
			}

		template <typename Numeric> Quaternion<T> &set(Numeric *elem) {
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

		Quaternion<T> &normalize(){
				double mag = magnitude();

				if (mag < 0.00000001) return *this;
				comp.w /=mag;
				comp.x /=mag;
				comp.y /=mag;
				comp.z /=mag;

				return *this;
			}

		Quaternion<T> &identity() {
				comp.w = 1.0;
				comp.x = 0;
				comp.y = 0;
				comp.z = 0;
				return *this;
			}

		double magnitude() const {
				return sqrt(comp.w*comp.w + comp.x*comp.x + comp.y*comp.y + comp.z*comp.z);
			}

		Quaternion<T> &conj() {
				comp.x = -comp.x;
				comp.y = -comp.y;
				comp.z = -comp.z;
				return *this;
			}

		template <typename T2, typename Numeric> 
		void toAxisAngle(Vector3D<T2> &axis, Numeric &ang) const {
				Quaternion<T> tmp;
				double cos_a, sin_a;
				tmp = *this;

				tmp.normalize();

				cos_a = tmp.comp.w;
				ang   = acos( cos_a ) * 2.0;
				sin_a = sqrt( 1.0 - cos_a * cos_a );

				if ( fabs( sin_a ) < 0.0005 ) sin_a = 1;

				axis.comp.x = comp.x / sin_a;
				axis.comp.y = comp.y / sin_a;
				axis.comp.z = comp.z / sin_a;
			}


		template <typename T2>
		Quaternion<T> &fromAxisAngle(const Vector3D<T2> &axis, double ang) {
				double cos_a, sin_a;

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
		Quaternion<T> &fromScaledAxis(Vector3D<Numeric> &axis) {
				double cos_a, sin_a, ang;

				ang = axis.magnitude();

				if (ang < 0.000001) {
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

		Quaternion<T> &fromEulerX(double ang) {
				double cos_a, sin_a;

				ang *=0.5;

				sin_a = sin(ang);
				cos_a = cos(ang);

				comp.x = sin_a; comp.y = 0; comp.z = 0; comp.w = cos_a;

				return *this;
			}

		Quaternion<T> &fromEulerY(double ang) {

				double cos_a, sin_a;

				ang *=0.5;

				sin_a = sin(ang);
				cos_a = cos(ang);

				comp.x = 0; comp.y = sin_a; comp.z = 0; comp.w = cos_a;

				return *this;
			}

		Quaternion<T> &fromEulerZ(double ang) {

				double cos_a, sin_a;

				ang *=0.5;

				sin_a = sin(ang);
				cos_a = cos(ang);

				comp.x = 0; comp.y = 0; comp.z = sin_a; comp.w = cos_a;

				return *this;
			}

		template <typename Numeric> 
		void toSpherical(Numeric &lat, Numeric &lng, Numeric &ang) const {
				double cos_a, sin_a;
				double tx,ty,tz;

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

		Quaternion<T> &fromSpherical(double lat, double lng, double ang) {
				double cos_a, sin_a;
				double sin_lat, cos_lat, sin_long, cos_long;

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

template<typename T>
class Matrix3D {
public:
		T a[9];
		Matrix3D<T>(){};
		Matrix3D<T>(const Matrix3D<T> &m){
				for (int i = 0; i < 9; i++) a[i] = m.a[i];
			}

		explicit Matrix3D<T>(const Matrix4D<T> &m){
				int i,j,k,l;
				k=0;l=0;
				for (i = 0; i < 3; i++) {
						for (j = 0; j < 3; j++){
								a[k]=m.a[l];
								k++; l++;
						}
						l++;
				}
			}

		template <typename T2>
		explicit Matrix3D<T>(const Quaternion<T2> &quat) {
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
		}

	

		template <typename T2> 
		Matrix3D<T> &operator =  (const Matrix3D<T2> &m){
				for (int i = 0; i < 9; i++) a[i] = m.a[i];
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

		Matrix3D<T> operator - () {
				Matrix3D<T> tmp;
				int j;
				for (j = 0; j < 9; j++) tmp.a[j] = - a[j];
				return tmp;
			}

		template <typename Numeric> 
		Matrix3D<T> &set(Numeric *elem) {
				for (int i = 0; i < 9; i++) a[i] = elem[i];
				return *this;
			}

		template <typename Numeric> 
		void store(Numeric *elem) const {
				for (int i = 0; i < 9; i++) elem[i] = a[i];
			}

		template<typename Numeric> 
		Matrix3D<T> &set_from_gl(Numeric *elem) {// by columns instead of by rows (As for OpenGL)
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

		template<typename Numeric> 
		void store_to_gl(Numeric *elem) const {
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
				return  a[0] * ( a[4]*a[8] - a[7]*a[5] )
				  - a[1] * ( a[3]*a[8] - a[6]*a[5] )
				  + a[2] * ( a[3]*a[7] - a[6]*a[4] );
			}


		double trace() const {
				return a[0] + a[4] + a[8];
			}

		Matrix3D<T> &identity() {
				for (int i=0; i < 9; i++) a[i] = 0;
				a[0]=a[4]=a[8]= 1;
				return *this;
			}

		Matrix3D<T> &zero() {
				for (int i=0; i < 9; i++) a[i] = 0;
				return *this;
			}

		Matrix3D<T> getInverse() const { // gets the inverse of a matrix (doesn't overtwirte the current matrix)
				double d = det();
				Matrix3D<T> mat;

				if ( d > -0.00001 && d < 0.00001) {
					return mat.identity();
				}

				mat.a[0] =    a[4]*a[8] - a[5]*a[7]   / d;
				mat.a[1] = -( a[1]*a[8] - a[7]*a[2] ) / d;
				mat.a[2] =    a[1]*a[5] - a[4]*a[2]   / d;

				mat.a[3] = -( a[3]*a[8] - a[5]*a[6] ) / d;
				mat.a[4] =    a[0]*a[8] - a[6]*a[2]   / d;
				mat.a[5] = -( a[0]*a[5] - a[3]*a[2] ) / d;

				mat.a[6] =    a[3]*a[7] - a[6]*a[4]   / d;
				mat.a[7] = -( a[0]*a[7] - a[6]*a[1] ) / d;
				mat.a[8] =    a[0]*a[4] - a[1]*a[3]   / d;

				return mat;
			}

		Matrix3D<T> &transpose() {
				double t;
				SWAP(a[1],a[3]);
				SWAP(a[2],a[6]);
				SWAP(a[5],a[7]);
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
		Matrix3D<T> &rotationX(double angle) {
				identity();
				double c = cos(angle);
				double s = sin(angle);
				a[4] = c;
				a[5] = -s;
				a[7] = s;
				a[8] = c;
				return *this;
			}

		//         |  cos(A)  0   sin(A) |
		//     M = |  0       1   0      |
		//         | -sin(A)  0   cos(A) |
		Matrix3D<T> &rotationY(double angle) {
				identity();
				double c = cos(angle);
				double s = sin(angle);
				a[0] = c;
				a[2] = s;
				a[6] = -s;
				a[8] = c;
				return *this;
			}

		//         |  cos(A)  -sin(A)   0 |
		//     M = |  sin(A)   cos(A)   0 |
		//         |  0        0        1 |
		Matrix3D<T> &rotationZ(double angle) {
				identity();
				double c = cos(angle);
				double s = sin(angle);
				a[0] = c;
				a[1] = -s;
				a[3] = s;
				a[4] = c;
				return *this;
			}

		Matrix3D<T> &scaling(double x, double y, double z) { // set this to a scaling matrix
				zero();
				a[0] = x;
				a[4] = y;
				a[8] = z;
				return *this;
			}


		Matrix3D<T> &scale(double x, double y, double z) { // scale current matrix
				a[0]*=x; a[1]*=y; a[2]*=z;
				a[3]*=x; a[4]*=y; a[5]*=z;
				a[6]*=x; a[7]*=y; a[8]*=z;
				return *this;
			}

		Matrix3D<T> &scale(double s) {   // scale current matrix
				a[0]*=s; a[1]*=s; a[2]*=s;
				a[3]*=s; a[4]*=s; a[5]*=s;
				a[6]*=s; a[7]*=s; a[8]*=s;
				return *this;
			}

		Vector3D<T> column(int i) {
				return Vector3D<T>(a[i], a[i+3], a[i+6]);
			}

		Vector3D<T> row(int i) {
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
				a[idx] = v.comp.x;
				a[idx+1] = v.comp.y;
				a[idx+2] = v.comp.z;
			}


		inline T elem(int i) const {return a[i];}
};

template<typename T>
std::ostream& operator <<(std::ostream &f, const Matrix3D<T> &m) {
	return f << "{{" << m.elem(0) << ',' << m.elem(1) << ',' << m.elem(2)  << "}," << \
			    "{" << m.elem(3) << ',' << m.elem(4) << ',' << m.elem(5)  << "}," << \
			    "{" << m.elem(6) << ',' << m.elem(7) << ',' << m.elem(8)  << "}}" << std::endl;
}

template<typename T>
class Matrix4D {
public:
		T a[16];
		Matrix4D<T>(){};

		Matrix4D<T>(const Matrix4D<T> &m){
				for (int i = 0; i < 16; i++) a[i] = m.a[i];
			}

		explicit Matrix4D<T>(const Matrix3D<T> &m){
				m[3] = m[7] = m[11] = m[12] = m[13] = m[14] = 0;
				m[15] = 1;
				*this << m;
			}

		template <typename T2>
		explicit Matrix4D<T>(const Quaternion<T2> &quat) {
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

		template <typename T2> 
		Matrix4D<T> &operator << (const Matrix3D<T2> &m) {
			int i,j,k,l;
			k=0;l=0;
			for (i = 0; i < 3; i++) {
					for (j = 0; j < 3; j++){
							a[l]=m.a[k];
							k++; l++;
					}
					l++;
			}
			return *this;
		}


		template <typename T2> 
		Matrix4D<T> &operator = (const Matrix4D<T2> &m){
				for (int i = 0; i < 16; i++) a[i] = m.a[i];
				return *this;
			}

		template <typename T2> 
		Matrix4D<T> operator  *  (const Matrix4D<T2> &m) const {
			Matrix4D<T> tmp;
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
		Matrix4D<T> operator  +  (const Matrix4D<T2> &m) const {
			Matrix4D<T> tmp;
			int j;
			for (j = 0; j < 16; j++) tmp.a[j] = a[j] + m.a[j];
			return tmp;
		}

		template <typename T2> 
		Matrix4D<T> operator  -  (const Matrix4D<T2> &m) const {
			Matrix4D<T> tmp;
			int j;
			for (j = 0; j < 16; j++) tmp.a[j] = a[j] - m.a[j];
			return tmp;
		}

		Matrix4D<T> operator - () {
			Matrix4D<T> tmp;
			int j;
			for (j = 0; j < 16; j++) tmp.a[j] = - a[j];
			return tmp;
		}


		template <typename Numeric> 
		Matrix4D<T> &set(Numeric *elem) {
			for (int i = 0; i < 16; i++) a[i] = elem[i];
			return *this;
		}

		template <typename Numeric> 
		void store(Numeric *elem) const {
			for (int i = 0; i < 16; i++) elem[i] = a[i];
		}

		template<typename Numeric> 
		Matrix4D<T> &set_from_gl(Numeric *elem) {// by columns instead of by rows (As for OpenGL)
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

		template<typename Numeric> 
		void store_to_gl(Numeric *elem) const {
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

		double trace() const {
				return a[0] + a[5] + a[10] + a[15];
			}

		Matrix4D<T> &identity() {
				for (int i=0; i < 16; i++) a[i] = 0;
				a[0]=a[5]=a[10]=a[15] = 1;
				return *this;
			}

		Matrix4D<T> &zero() {
				for (int i=0; i < 16; i++) a[i] = 0;
				return *this;
			}

		Matrix4D<T> getInverse() const { // gets the inverse of a matrix (doesn't overtwirte the current matrix)
				// adapted from MESA's GLU
				double cof[16], det;
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

				if (det == 0)
						return Matrix4D<T>().zero();

				det = 1.0 / det;
			
				Matrix4D<T> tmp;

				for (i = 0; i < 16; i++)
						tmp.a[i] = cof[i] * det;

				return tmp;
			}

		Matrix4D<T> &transpose() {
				double t;
				SWAP(a[1],a[4]);
				SWAP(a[2],a[8]);
				SWAP(a[6],a[9]);
				SWAP(a[3],a[12]);
				SWAP(a[7],a[13]);
				SWAP(a[11],a[14]);
				return *this;
			}

		template <typename T2> 
		Matrix4D<T> &map(Matrix4D<T2> &from, Matrix4D<T2> &to) {// sets the matrix to one which maps the space defined by m1 into a space defined by m2
				*this = to * from.getInverse();
				return *this;
			}

		//        |  1  0       0      0 |
		//    M = |  0  cos(A) -sin(A) 0 |
		//        |  0  sin(A)  cos(A) 0 |
		//        |  0  0       0      1 |
		Matrix4D<T> &rotationX(double angle) {
				zero();
				double c = cos(angle);
				double s = sin(angle);
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
		Matrix4D<T> &rotationY(double angle) {
				zero();
				double c = cos(angle);
				double s = sin(angle);
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
		Matrix4D<T> &rotationZ(double angle) {
				zero();
				double c = cos(angle);
				double s = sin(angle);
				a[0] = c;
				a[1] = -s;
				a[3] = s;
				a[4] = c;
				a[10] = a[15] = 1;
				return *this;
			}

		Matrix4D<T> &scaling(double x, double y, double z, double w=1.0) { // set this to a scaling matrix
				zero();
				a[0] = x;
				a[5] = y;
				a[10] = z;
				a[15] = w;
				return *this;
			}


		Matrix4D<T> &scale(double x, double y, double z, double w=1.0) { // scale current matrix
				a[0]*=x;  a[1]*=y;  a[2]*=z;  
				a[4]*=x;  a[5]*=y;  a[6]*=z;  
				a[8]*=x;  a[9]*=y;  a[10]*=z; 
				a[12]*=x; a[13]*=y; a[14]*=z; 
				if (w != 1) {
					a[3]*=w;
					a[7]*=w;
					a[11]*=w;
					a[15]*=w;
				}
				return *this;
			}

		Matrix4D<T> &scale(double s) {   // scale current matrix
				a[0]*=s;  a[1]*=s;  a[2]*=s;  a[3]*=s;
				a[4]*=s;  a[5]*=s;  a[6]*=s;  a[7]*=s;
				a[8]*=s;  a[9]*=s;  a[10]*=s; a[11]*=s;
				a[12]*=s; a[13]*=s; a[14]*=s; a[15]*=s;
				return *this;
			}

		Vector4D<T> column(int i) {
				return Vector4D<T>(a[i], a[i+4], a[i+8], a[i+12]);
			}

		Vector4D<T> row(int i) {
				int idx = i * 4;
				return Vector4D<T>(a[idx], a[idx+1], a[idx+2], a[idx+3]);
			}

		void setColumn(int i, const Vector4D<T> &v) {
				a[i] = v.comp.x;
				a[i+4] = v.comp.y;
				a[i+8] = v.comp.z;
				a[i+12] = v.comp.w;
			}

		void setRow(int i, const Vector4D<T> &v) {
				int idx = i * 4;
				a[idx] = v.comp.x;
				a[idx+1] = v.comp.y;
				a[idx+2] = v.comp.z;
				a[idx+3] = v.comp.w;
			}

		inline T elem(int i) const {return a[i];};

};


template<typename T>
std::ostream& operator <<(std::ostream &f, const Matrix4D<T> &m) {
	return f << "|" << m.elem(0) << ',' << m.elem(1) << ',' << m.elem(2) << ',' << m.elem(3) << '|' << std::endl << \
			    "|" << m.elem(4) << ',' << m.elem(5) << ',' << m.elem(6) << ',' << m.elem(7) << '|' << std::endl << \
			    "|" << m.elem(8) << ',' << m.elem(9) << ',' << m.elem(10)<< ',' << m.elem(11)<< '|' << std::endl << \
			    "|" << m.elem(12)<< ',' << m.elem(13)<< ',' << m.elem(14)<< ',' << m.elem(15)<< '|' << std::endl;
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

		static Vector3D<T> I() {
			return Vector3D<T>(1,0,0);
		}

		static Vector3D<T> J() {
			return Vector3D<T>(0,1,0);
		}

		static Vector3D<T> K() {
			return Vector3D<T>(0,0,1);
		}

		Vector3D<T> () {};

		Vector3D<T> (T px, T py, T pz) {
			comp.x = px;
			comp.y = py;
			comp.z = pz;
		}

		Vector3D<T> (const Vector3D<T> &from) {
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


		Vector3D<T> &zero() {
				comp.x = comp.y = comp.z = 0;
				return *this;
			}

		Vector3D<T> set_I() {
				comp.x = 1;
				comp.y = 0;
				comp.z = 0;
				return *this;
			}

		Vector3D<T> set_J() {
				comp.x = 0;
				comp.y = 1;
				comp.z = 0;
				return *this;
			}

		Vector3D<T> set_K() {
				comp.x = 0;
				comp.y = 0;
				comp.z = 1;
				return *this;
			}


		Vector3D<T> &set(double px, double py, double pz) {
				comp.x = px;
				comp.y = py;
				comp.z = pz;
				return *this;
			}

		template <typename Numeric> 
		Vector3D<T> &set(Numeric *elem) {
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

		Vector3D<T> &normalize(){
				double mag = magnitude();

				if (mag < 0.00000001) return *this;
				comp.x /=mag;
				comp.y /=mag;
				comp.z /=mag;

				return *this;
			}

		template <typename T2>
		Vector3D<T> &operator  += (const Vector3D<T2> &v){
				comp.x += v.comp.x;
				comp.y += v.comp.y;
				comp.z += v.comp.z;
				return *this;
			}

		template <typename Numeric>
		Vector3D<T> &operator  += (Numeric *elem) {
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
		Vector3D<T> &operator  -= (Vector3D<T2> &v) {
				comp.x -= v.comp.x;
				comp.y -= v.comp.y;
				comp.z -= v.comp.z;
				return *this;
			}

		template <typename Numeric>
		Vector3D<T> &operator  -= (Numeric *elem) {
				comp.x -= elem[0];
				comp.y -= elem[1];
				comp.z -= elem[2];
				return *this;
			}

		double  magnitude() {
				return sqrt(comp.x*comp.x+comp.y*comp.y+comp.z*comp.z);
			}

		double  magnitudeSq() {
				return comp.x*comp.x+comp.y*comp.y+comp.z*comp.z;
			}

		template <typename T2>
		double dot(const Vector3D<T2> &v) {
				return comp.x*v.comp.x + comp.y*v.comp.y + comp.z*v.comp.z;
			}


		template <typename T2>
		double distanceSq(const Vector3D<T2> &v) {
				double dx = comp.x - v.comp.x;
				double dy = comp.y - v.comp.y;
				double dz = comp.z - v.comp.z;
				return dx*dx+dy*dy+dz*dz;
			}

		template <typename T2>
		double distance(const Vector3D<T2> &v) {
				double dx = comp.x - v.comp.x;
				double dy = comp.y - v.comp.y;
				double dz = comp.z - v.comp.z;
				return sqrt(dx*dx+dy*dy+dz*dz);
			}

		template <typename T2>
		double distance(const Vector3D<T2> &v, int axis) {
				return a[axis] - v.a[axis];
			}

		template <typename T2>
		Vector3D<T> cross(const Vector3D<T2> &v) {
				Vector3D<T> tmp;
				tmp.comp.x= comp.y*v.comp.z-comp.z*v.comp.y;
				tmp.comp.y= comp.z*v.comp.x-comp.x*v.comp.z;
				tmp.comp.z= comp.x*v.comp.y-comp.y*v.comp.x;
				return tmp;
			}

		template <typename T2>
		Vector3D<T> min(const Vector3D<T2> &v) {
				Vector3D<T> tmp(
					a[0] < v.a[0] ? a[0] : v.a[0],
					a[1] < v.a[1] ? a[1] : v.a[1],
					a[2] < v.a[2] ? a[2] : v.a[2]
				);
				return tmp;
			}

		template <typename T2>
		Vector3D<T> max(const Vector3D<T2> &v) {
				Vector3D<T> tmp(
					a[0] > v.a[0] ? a[0] : v.a[0],
					a[1] > v.a[1] ? a[1] : v.a[1],
					a[2] > v.a[2] ? a[2] : v.a[2]
				);
				return tmp;
			}

		// projects this onto the vector v
		Vector3D<T> projectOnto(const Vector3D<T> &v) {
			Vector3D tmp(v);
			tmp.normalize();
		
			return tmp * dot(tmp);

			}	
		

		T   *getPtr() { return &comp.x; };

		T &operator [](int i) {return a[i];}
		T operator [](int i) const {return a[i];}

		int getMaxCoord() {
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

		static bool compareByX(const Vector3D &p1, const Vector3D &p2) {
			return p1.a[0] < p2.a[0];
		}

		static bool compareByY(const Vector3D &p1, const Vector3D &p2) {
			return p1.a[1] < p2.a[1];
		}

		static bool compareByZ(const Vector3D &p1, const Vector3D &p2) {
			return p1.a[2] < p2.a[2];
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

		static Vector4D<T> I() {
			return Vector4D<T>(1,0,0,0);
		}

		static Vector4D<T> J() {
			return Vector4D<T>(0,1,0,0);
		}

		static Vector4D<T> K() {
			return Vector4D<T>(0,0,1,0);
		}

		static Vector4D<T> W() {
			return Vector4D<T>(0,0,0,1);
		}

		Vector4D<T> () {};

		Vector4D<T> (T px, T py, T pz, T pw) {
				comp.x = px;
				comp.y = py;
				comp.z = pz;
				comp.w = pw;
			}

		Vector4D<T> (const Vector4D<T> &from){
				comp.x = from.comp.x;
				comp.y = from.comp.y;
				comp.z = from.comp.z;
				comp.w = from.comp.w;
			}


		template <typename T2>
		explicit Vector4D<T> (const Vector3D<T2> &from, T2 pw = 1){
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

		Vector4D<T> &zero() {
				comp.x = comp.y = comp.z = comp.w = 0;
				return *this;
			}

		Vector4D<T> &set(double px, double py, double pz, double pw) {
				comp.x = px;
				comp.y = py;
				comp.z = pz;
				comp.w = pw;
				return *this;
			}

		template <typename Numeric> 
		Vector4D<T> &set(Numeric *elem) {
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

		Vector4D<T> &normalize(){
				double mag = magnitude();

				if (mag < 0.00000001) return *this;
				comp.x /=mag;
				comp.y /=mag;
				comp.z /=mag;
				comp.w /=mag;

				return *this;
			}

		template <typename T2>
		Vector4D<T> &operator  += (const Vector4D<T2> &v){
				comp.x += v.comp.x;
				comp.y += v.comp.y;
				comp.z += v.comp.z;
				comp.w += v.comp.w;
				return *this;
			}

		template <typename Numeric>
		Vector4D<T> &operator  += (Numeric *elem) {
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
		Vector4D<T> &operator  -= (Vector4D<T2> &v) {
				comp.x -= v.comp.x;
				comp.y -= v.comp.y;
				comp.z -= v.comp.z;
				comp.w -= v.comp.w;
				return *this;
			}

		template <typename Numeric>
		Vector4D<T> &operator  -= (Numeric *elem) {
				comp.x -= elem[0];
				comp.y -= elem[1];
				comp.z -= elem[2];
				comp.w -= elem[3];
				return *this;
			}

		double  magnitude() {
				return sqrt(comp.x*comp.x+comp.y*comp.y+comp.z*comp.z+comp.w*comp.w);
			}

		template <typename T2>
		double dot(const Vector4D<T2> &v) {
				return comp.x*v.comp.x + comp.y*v.comp.y + comp.z*v.comp.z + comp.w*v.comp.w;
			}


		template <typename T2>
		double distanceSq(const Vector4D<T2> &v) {
				double dx = comp.x - v.comp.x;
				double dy = comp.y - v.comp.y;
				double dz = comp.z - v.comp.z;
				double dw = comp.w - v.comp.w;
				return dx*dx+dy*dy+dz*dz+dw*dw;
			}

		template <typename T2>
		double distance(const Vector4D<T2> &v) {
				double dx = comp.x - v.comp.x;
				double dy = comp.y - v.comp.y;
				double dz = comp.z - v.comp.z;
				double dw = comp.w - v.comp.w;
				return sqrt(dx*dx+dy*dy+dz*dz+dw*dw);
			}


		template <typename T2>
		Vector4D<T> min(const Vector4D<T2> &v) {
				Vector4D<T> tmp(
					a[0] < v.a[0] ? a[0] : v.a[0],
					a[1] < v.a[1] ? a[1] : v.a[1],
					a[2] < v.a[2] ? a[2] : v.a[2],
					a[3] < v.a[3] ? a[3] : v.a[3]
				);
				return tmp;
			}

		template <typename T2>
		Vector4D<T> max(const Vector4D<T2> &v) {
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

		T   *getPtr() { return &comp.x; };

		T &operator [](int i) {return a[i];}
		T operator [](int i) const {return a[i];}
};


template<typename T>
std::ostream& operator <<(std::ostream &f, const Vector4D<T> &v) {
	return f << "{" << v.comp.x << ',' << v.comp.y << ',' << v.comp.z << ',' << v.comp.w << '}';
}

} // namespace

#endif// __LINEAR_INC__


