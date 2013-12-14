/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_MATH_H
#define B2_MATH_H

#include <Box2D/Common/b2Settings.h>
#include <math.h>

/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b2IsValid(float32 x)
{
	//int32 ix = *reinterpret_cast<int32*>(&x);
	//return (ix & 0x7f800000) != 0x7f800000;
	return !isnan(x) && isfinite(x);
}

/// This is a approximate yet fast inverse square-root.
inline float32 b2InvSqrt(float32 x)
{
	/*union
	{
		float32 x;
		int32 i;
	} convert;

	convert.x = x;
	float32 xhalf = 0.5 * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5 - xhalf * x * x);
	return x;*/
	return 1.0 / sqrt(x);
}

#define	b2Sqrt(x)	sqrtf(x)
#define	b2Atan2(y, x)	atan2f(y, x)

/// A 2D column vector.
struct b2Vec2
{
	/// Default constructor does nothing (for performance).
	b2Vec2() {}

	/// Construct using coordinates.
	b2Vec2(float32 x, float32 y) : x(x), y(y) {}

	/// Set this vector to all zeros.
	void SetZero() { this->x = 0.0; this->y = 0.0; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_) { this->x = x_; this->y = y_; }

	void Assign(const b2Vec2 &l)
	{
		this->x = l.x;
		this->y = l.y;
	}

	/// Negate this vector.
	b2Vec2 Negate() const { b2Vec2 v; v.Set(-this->x, -this->y); return v; }

	/// Read from and indexed element.
	const float32 &get_i (const int32 &i) const
	{
		return (&this->x)[i];
	}

	/// Write to an indexed element.
	float32& set_i (const int32 &i, const float &v)
	{
		return (&this->x)[i] = v;
	}

	/// Add a vector to this vector.
	void Add(const b2Vec2& v)
	{
		this->x += v.x; this->y += v.y;
	}

	/// Subtract a vector from this vector.
	void Subtract(const b2Vec2& v)
	{
		this->x -= v.x; this->y -= v.y;
	}

	/// Multiply this vector by a scalar.
	void Multiply(float32 a)
	{
		this->x *= a; this->y *= a;
	}

	/// Get the length of this vector (the norm).
	float32 Length() const
	{
		return b2Sqrt(this->x * this->x + this->y * this->y);
	}

	/// Get the length squared. For performance, use this instead of
	/// b2Vec2::Length (if possible).
	float32 LengthSquared() const
	{
		return this->x * this->x + this->y * this->y;
	}

	/// Convert this vector into a unit vector. Returns the length.
	float32 Normalize()
	{
		float32 length = this->Length();
		if (length < b2_epsilon)
		{
			return 0.0;
		}
		float32 invLength = 1.0 / length;
		this->x *= invLength;
		this->y *= invLength;

		return length;
	}

	/// Does this vector contain finite coordinates?
	bool IsValid() const
	{
		return b2IsValid(this->x) && b2IsValid(this->y);
	}

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	b2Vec2 Skew() const
	{
		return b2Vec2(-this->y, this->x);
	}

	/// Add two vectors component-wise.
	static inline b2Vec2 Add(const b2Vec2& a, const b2Vec2& b)
	{
		return b2Vec2(a.x + b.x, a.y + b.y);
	}

	/// Subtract two vectors component-wise.
	static inline b2Vec2 Subtract(const b2Vec2& a, const b2Vec2& b)
	{
		return b2Vec2(a.x - b.x, a.y - b.y);
	}

	static inline bool Equals(const b2Vec2& a, const b2Vec2& b)
	{
		return a.x == b.x && a.y == b.y;
	}

	static inline b2Vec2 Multiply(float32 s, const b2Vec2& a)
	{
		return b2Vec2(s * a.x, s * a.y);
	}

	float32 x, y;

private:
	inline b2Vec2 &operator=(const b2Vec2 &l)
	{
		B2_NOT_USED(l);
		return *this;
	}
};

/// A 2D column vector with 3 elements.
struct b2Vec3
{
	/// Default constructor does nothing (for performance).
	b2Vec3() {}

	/// Construct using coordinates.
	b2Vec3(float32 x, float32 y, float32 z) : x(x), y(y), z(z) {}

	inline void Assign(const b2Vec3 &l)
	{
		this->x = l.x;
		this->y = l.y;
		this->z = l.z;
	}

	/// Set this vector to all zeros.
	void SetZero() { this->x = 0.0; this->y = 0.0; this->z = 0.0; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_, float32 z_) { this->x = x_; this->y = y_; this->z = z_; }

	/// Negate this vector.
	b2Vec3 Negate() const { b2Vec3 v; v.Set(-this->x, -this->y, -this->z); return v; }

	/// Add a vector to this vector.
	void Add (const b2Vec3& v)
	{
		this->x += v.x; this->y += v.y; this->z += v.z;
	}

	/// Subtract a vector from this vector.
	void Subtract (const b2Vec3& v)
	{
		this->x -= v.x; this->y -= v.y; this->z -= v.z;
	}

	/// Multiply this vector by a scalar.
	void Multiply (float32 s)
	{
		this->x *= s; this->y *= s; this->z *= s;
	}

	static inline b2Vec3 Multiply(float32 s, const b2Vec3& a)
	{
		return b2Vec3(s * a.x, s * a.y, s * a.z);
	}

	/// Add two vectors component-wise.
	static inline b2Vec3 Add(const b2Vec3& a, const b2Vec3& b)
	{
		return b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	/// Subtract two vectors component-wise.
	static inline b2Vec3 Subtract(const b2Vec3& a, const b2Vec3& b)
	{
		return b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	float32 x, y, z;

private:
	inline b2Vec3 &operator=(const b2Vec3 &l)
	{
		B2_NOT_USED(l);
		return *this;
	}
};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
	/// The default constructor does nothing (for performance).
	b2Mat22() {}

	/// Construct this matrix using columns.
	b2Mat22(const b2Vec2& c1, const b2Vec2& c2)
	{
		this->ex.Assign(c1);
		this->ey.Assign(c2);
	}

	/// Construct this matrix using scalars.
	b2Mat22(float32 a11, float32 a12, float32 a21, float32 a22)
	{
		this->ex.x = a11; this->ex.y = a21;
		this->ey.x = a12; this->ey.y = a22;
	}

	inline void Assign(const b2Mat22 &l)
	{
		this->ex.Assign(l.ex);
		this->ey.Assign(l.ey);
	}

	/// Initialize this matrix using columns.
	void Set(const b2Vec2& c1, const b2Vec2& c2)
	{
		this->ex.Assign(c1);
		this->ey.Assign(c2);
	}

	/// Set this to the identity matrix.
	void SetIdentity()
	{
		this->ex.x = 1.0; this->ey.x = 0.0;
		this->ex.y = 0.0; this->ey.y = 1.0;
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
		this->ex.x = 0.0; this->ey.x = 0.0;
		this->ex.y = 0.0; this->ey.y = 0.0;
	}

	b2Mat22 GetInverse() const
	{
		float32 a = this->ex.x, b = this->ey.x, c = this->ex.y, d = this->ey.y;
		b2Mat22 B;
		float32 det = a * d - b * c;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		B.ex.x =  det * d;	B.ey.x = -det * b;
		B.ex.y = -det * c;	B.ey.y =  det * a;
		return B;
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b2Vec2 Solve(const b2Vec2& b) const
	{
		float32 a11 = ex.x, a12 = this->ey.x, a21 = ex.y, a22 = this->ey.y;
		float32 det = a11 * a22 - a12 * a21;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		b2Vec2 x;
		x.x = det * (a22 * b.x - a12 * b.y);
		x.y = det * (a11 * b.y - a21 * b.x);
		return x;
	}

	static inline b2Mat22 Add(const b2Mat22& A, const b2Mat22& B)
	{
		return b2Mat22(b2Vec2::Add(A.ex, B.ex), b2Vec2::Add(A.ey, B.ey));
	}

	b2Vec2 ex, ey;

private:
	inline b2Mat22 &operator=(const b2Mat22 &l)
	{
		B2_NOT_USED(l);
		return *this;
	}
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
	/// The default constructor does nothing (for performance).
	b2Mat33() {}

	/// Construct this matrix using columns.
	b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3)
	{
		this->ex.Assign(c1);
		this->ey.Assign(c2);
		this->ez.Assign(c3);
	}

	inline void Assign(const b2Mat33 &l)
	{
		this->ex.Assign(l.ex);
		this->ey.Assign(l.ey);
		this->ez.Assign(l.ez);
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
		this->ex.SetZero();
		this->ey.SetZero();
		this->ez.SetZero();
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b2Vec3 Solve33(const b2Vec3& b) const;

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	b2Vec2 Solve22(const b2Vec2& b) const;

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	void GetInverse22(b2Mat33* M) const;

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	void GetSymInverse33(b2Mat33* M) const;

	b2Vec3 ex, ey, ez;

private:
	inline b2Mat33 &operator=(const b2Mat33 &l)
	{
		B2_NOT_USED(l);
		return *this;
	}
};

/// Rotation
struct b2Rot
{
	b2Rot() {}

	/// Initialize from an angle in radians
	explicit b2Rot(float32 angle)
	{
		/// TODO_ERIN optimize
		this->s = sinf(angle);
		this->c = cosf(angle);
	}

	inline void Assign(const b2Rot &l)
	{
		this->s = l.s;
		this->c = l.c;
	}

	/// Set using an angle in radians.
	void Set(float32 angle)
	{
		/// TODO_ERIN optimize
		this->s = sinf(angle);
		this->c = cosf(angle);
	}

	/// Set to the identity rotation
	void SetIdentity()
	{
		this->s = 0.0;
		this->c = 1.0;
	}

	/// Get the angle in radians
	float32 GetAngle() const
	{
		return b2Atan2(this->s, this->c);
	}

	/// Get the x-axis
	b2Vec2 GetXAxis() const
	{
		return b2Vec2(this->c, this->s);
	}

	/// Get the u-axis
	b2Vec2 GetYAxis() const
	{
		return b2Vec2(-this->s, this->c);
	}

	/// Sine and cosine
	float32 s, c;

private:
	inline b2Rot &operator=(const b2Rot &l)
	{
		B2_NOT_USED(l);
		return *this;
	}
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() {}

	/// Initialize using a position vector and a rotation.
	b2Transform(const b2Vec2& position, const b2Rot& rotation) : p(position), q(rotation) {}

	inline void Assign(const b2Transform &l)
	{
		this->p.Assign(l.p);
		this->q.Assign(l.q);
	}

	/// Set this to the identity transform.
	void SetIdentity()
	{
		this->p.SetZero();
		this->q.SetIdentity();
	}

	/// Set this based on the position and angle.
	void Set(const b2Vec2& position, float32 angle)
	{
		this->p.Assign(position);
		this->q.Set(angle);
	}

	b2Vec2 p;
	b2Rot q;

private:
	inline b2Transform &operator=(const b2Transform &l)
	{
		B2_NOT_USED(l);
		return *this;
	}
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	void GetTransform(b2Transform* xfb, float32 beta) const;

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	void Advance(float32 alpha);

	/// Normalize the angles.
	void Normalize();

	inline void Assign(const b2Sweep &l)
	{
		this->localCenter.Assign(l.localCenter);
		this->c0.Assign(l.c0);
		this->c.Assign(l.c);
		this->a0 = l.a0;
		this->a = l.a;
		this->alpha0 = l.alpha0;
	}

	b2Vec2 localCenter;	///< local center of mass position
	b2Vec2 c0, c;		///< center world positions
	float32 a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float32 alpha0;

private:
	inline b2Sweep &operator=(const b2Sweep &l)
	{
		B2_NOT_USED(l);
		return *this;
	}
};

/// Perform the dot product on two vectors.
inline float32 b2Dot_v2_v2(const b2Vec2& a, const b2Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float32 b2Cross_v2_v2(const b2Vec2& a, const b2Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross_v2_f(const b2Vec2& a, float32 s)
{
	return b2Vec2(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline b2Vec2 b2Cross_f_v2(float32 s, const b2Vec2& a)
{
	return b2Vec2(-s * a.y, s * a.x);
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline b2Vec2 b2Mul_m22_v2(const b2Mat22& A, const b2Vec2& v)
{
	return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline b2Vec2 b2MulT_m22_v2(const b2Mat22& A, const b2Vec2& v)
{
	return b2Vec2(b2Dot_v2_v2(v, A.ex), b2Dot_v2_v2(v, A.ey));
}

inline float32 b2Distance(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = b2Vec2::Subtract(a, b);
	return c.Length();
}

inline float32 b2DistanceSquared(const b2Vec2& a, const b2Vec2& b)
{
	b2Vec2 c = b2Vec2::Subtract(a, b);
	return b2Dot_v2_v2(c, c);
}

/// Perform the dot product on two vectors.
inline float32 b2Dot_v3_v3(const b2Vec3& a, const b2Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
inline b2Vec3 b2Cross_v3_v3(const b2Vec3& a, const b2Vec3& b)
{
	return b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// A * B
inline b2Mat22 b2Mul_m22_m22(const b2Mat22& A, const b2Mat22& B)
{
	return b2Mat22(b2Mul_m22_v2(A, B.ex), b2Mul_m22_v2(A, B.ey));
}

// A^T * B
inline b2Mat22 b2MulT_m22_m22(const b2Mat22& A, const b2Mat22& B)
{
	b2Vec2 c1(b2Dot_v2_v2(A.ex, B.ex), b2Dot_v2_v2(A.ey, B.ex));
	b2Vec2 c2(b2Dot_v2_v2(A.ex, B.ey), b2Dot_v2_v2(A.ey, B.ey));
	return b2Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
inline b2Vec3 b2Mul_m33_v3(const b2Mat33& A, const b2Vec3& v)
{
	return b2Vec3::Add(b2Vec3::Add(b2Vec3::Multiply(v.x, A.ex), b2Vec3::Multiply(v.y, A.ey)), b2Vec3::Multiply(v.z, A.ez));
}

/// Multiply a matrix times a vector.
inline b2Vec2 b2Mul_m33_v2(const b2Mat33& A, const b2Vec2& v)
{
	return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply two rotations: q * r
inline b2Rot b2Mul_r_r(const b2Rot& q, const b2Rot& r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	b2Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT_r_r(const b2Rot& q, const b2Rot& r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	b2Rot qr;
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
	return qr;
}

/// Rotate a vector
inline b2Vec2 b2Mul_r_v2(const b2Rot& q, const b2Vec2& v)
{
	return b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/// Inverse rotate a vector
inline b2Vec2 b2MulT_r_v2(const b2Rot& q, const b2Vec2& v)
{
	return b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

inline b2Vec2 b2Mul_t_v2(const b2Transform& T, const b2Vec2& v)
{
	float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

	return b2Vec2(x, y);
}

inline b2Vec2 b2MulT_t_v2(const b2Transform& T, const b2Vec2& v)
{
	float32 px = v.x - T.p.x;
	float32 py = v.y - T.p.y;
	float32 x = (T.q.c * px + T.q.s * py);
	float32 y = (-T.q.s * px + T.q.c * py);

	return b2Vec2(x, y);
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b2Transform b2Mul_t_t(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
	C.q.Assign(b2Mul_r_r(A.q, B.q));
	C.p.Assign(b2Vec2::Add(b2Mul_r_v2(A.q, B.p), A.p));
	return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b2Transform b2MulT_t_t(const b2Transform& A, const b2Transform& B)
{
	b2Transform C;
	C.q.Assign(b2MulT_r_r(A.q, B.q));
	C.p.Assign(b2MulT_r_v2(A.q, b2Vec2::Subtract(B.p, A.p)));
	return C;
}

template <typename T>
inline T b2Abs(T a)
{
	return a > T(0) ? a : -a;
}

inline b2Vec2 b2Abs_v2(const b2Vec2& a)
{
	return b2Vec2(b2Abs(a.x), b2Abs(a.y));
}

inline b2Mat22 b2Abs_m22(const b2Mat22& A)
{
	return b2Mat22(b2Abs_v2(A.ex), b2Abs_v2(A.ey));
}

template <typename T>
inline T b2Min(T a, T b)
{
	return a < b ? a : b;
}

inline b2Vec2 b2Min_v2(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
}

template <typename T>
inline T b2Max(T a, T b)
{
	return a > b ? a : b;
}

inline b2Vec2 b2Max_v2(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
}

template <typename T>
inline T b2Clamp(T a, T low, T high)
{
	return b2Max(low, b2Min(a, high));
}

inline b2Vec2 b2Clamp_v2(const b2Vec2& a, const b2Vec2& low, const b2Vec2& high)
{
	return b2Max_v2(low, b2Min_v2(a, high));
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline uint32 b2NextPowerOfTwo(uint32 x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

inline bool b2IsPowerOfTwo(uint32 x)
{
	bool result = x > 0 && (x & (x - 1)) == 0;
	return result;
}

inline void b2Sweep::GetTransform(b2Transform* xf, float32 beta) const
{
	xf->p.Assign(b2Vec2::Add(b2Vec2::Multiply((1.0 - beta), this->c0), b2Vec2::Multiply(beta, this->c)));
	float32 angle = (1.0 - beta) * this->a0 + beta * this->a;
	xf->q.Set(angle);

	// Shift to origin
	xf->p.Subtract(b2Mul_r_v2(xf->q, this->localCenter));
}

inline void b2Sweep::Advance(float32 alpha)
{
	b2Assert(this->alpha0 < 1.0);
	float32 beta = (alpha - this->alpha0) / (1.0 - this->alpha0);
	this->c0.Add(b2Vec2::Multiply(beta, b2Vec2::Subtract(this->c, this->c0)));
	this->a0 += beta * (this->a - this->a0);
	this->alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
	float32 twoPi = 2.0 * b2_pi;
	float32 d = twoPi * floorf(this->a0 / twoPi);
	this->a0 -= d;
	this->a -= d;
}

#endif
