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

"use strict";

/// This function is used to ensure that a floating point number is not a NaN or infinity.
function b2IsValid(x)
{
	return isFinite(x) && !isNaN(x);
}

var sqrtf = Math.sqrt;
var atan2f = Math.atan2;
var sinf = Math.sin;
var cosf = Math.cos;
var floorf = Math.floor;
var ceilf = Math.ceil;

var b2Sqrt = sqrtf;
var b2Atan2 = atan2f;

/// This is a approximate yet fast inverse square-root.
function b2InvSqrt(x)
{
	return 1.0 / sqrtf(x);
}

/// A 2D column vector.
function b2Vec2(x, y)
{
	if (typeof(x) !== 'undefined')
	{
		this.x = x;
		this.y = y;
	}
	else
		this.x = this.y = 0;
}

b2Vec2.prototype =
{
	Clone: function()
	{
		return new b2Vec2(this.x, this.y);
	},

	/// Set this vector to all zeros.
	SetZero: function() { this.x = 0.0; this.y = 0.0; return this; },

	/// Set this vector to some specified coordinates.
	Set: function(x_, y_) { this.x = x_; this.y = y_; return this; },

	Assign: function(l)
	{
		this.x = l.x;
		this.y = l.y;
		return this;
	},

	/// Negate this vector.
	Negate: function() { var v = new b2Vec2(); v.Set(-this.x, -this.y); return v; },

	/// Read from and indexed element.
	get_i: function(i)
	{
		switch (i)
		{
		case 0:
			return this.x;
		case 1:
			return this.y;
		}
	},

	/// Write to an indexed element.
	set_i: function(i, v)
	{
		switch (i)
		{
		case 0:
			return this.x = v;
		case 1:
			return this.y = v;
		}
	},

	/// Add a vector to this vector.
	Add: function(v)
	{
		this.x += v.x; this.y += v.y;
		return this;
	},

	/// Subtract a vector from this vector.
	Subtract: function(v)
	{
		this.x -= v.x; this.y -= v.y;
		return this;
	},

	/// Multiply this vector by a scalar.
	Multiply: function(a)
	{
		this.x *= a; this.y *= a;
		return this;
	},

	/// Get the length of this vector (the norm).
	Length: function()
	{
		return b2Sqrt(this.x * this.x + this.y * this.y);
	},

	/// Get the length squared. For performance, use this instead of
	/// b2Vec2::Length (if possible).
	LengthSquared: function()
	{
		return this.x * this.x + this.y * this.y;
	},

	/// Convert this vector into a unit vector. Returns the length.
	Normalize: function()
	{
		var length = this.Length();
		if (length < b2_epsilon)
		{
			return 0.0;
		}
		var invLength = 1.0 / length;
		this.x *= invLength;
		this.y *= invLength;

		return length;
	},

	/// Does this vector contain finite coordinates?
	IsValid: function()
	{
		return b2IsValid(this.x) && b2IsValid(this.y);
	},

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	Skew: function()
	{
		return new b2Vec2(-this.y, this.x);
	},

	_serialize: function(out)
	{
		var obj = out || [];

		obj[0] = this.x;
		obj[1] = this.y;

		return obj;
	},

	_deserialize: function(data)
	{
		this.x = data[0];
		this.y = data[1];
	}
};

/// Add two vectors component-wise.
b2Vec2.Add = function(a, b)
{
	return new b2Vec2(a.x + b.x, a.y + b.y);
};

/// Subtract two vectors component-wise.
b2Vec2.Subtract = function(a, b)
{
	return new b2Vec2(a.x - b.x, a.y - b.y);
};

b2Vec2.Equals = function(a, b)
{
	return a.x == b.x && a.y == b.y;
};

b2Vec2.Multiply = function(s, a)
{
	return new b2Vec2(s * a.x, s * a.y);
};

b2Vec2.Negate = function(a)
{
	return new b2Vec2(-a.x, -a.y);
};

/// A 2D column vector with 3 elements.
function b2Vec3(x, y, z)
{
	if (typeof(x) !== 'undefined')
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

b2Vec3.prototype =
{
	Clone: function()
	{
		return new b2Vec3(this.x, this.y, this.z);
	},

	/// Set this vector to all zeros.
	SetZero: function() { this.x = 0.0; this.y = 0.0; this.z = 0.0; },

	/// Set this vector to some specified coordinates.
	Set: function(x_, y_, z_) { this.x = x_; this.y = y_; this.z = z_; },

	/// Negate this vector.
	Negate: function() { var v = new b2Vec3(); v.Set(-this.x, -this.y, -this.z); return v; },

	/// Add a vector to this vector.
	Add: function(v)
	{
		this.x += v.x; this.y += v.y; this.z += v.z;
	},

	/// Subtract a vector from this vector.
	Subtract: function(v)
	{
		this.x -= v.x; this.y -= v.y; this.z -= v.z;
	},

	/// Multiply this vector by a scalar.
	Multiply: function(s)
	{
		this.x *= s; this.y *= s; this.z *= s;
	},

	x: 0,
	y: 0,
	z: 0
};

b2Vec3.Multiply = function(s, a)
{
	return new b2Vec3(s * a.x, s * a.y, s * a.z);
};

/// Add two vectors component-wise.
b2Vec3.Add = function(a, b)
{
	return new b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
};

/// Subtract two vectors component-wise.
b2Vec3.Subtract = function(a, b)
{
	return new b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
};

/// A 2-by-2 matrix. Stored in column-major order.
function b2Mat22(c1, c2)
{
	this.ex = c1 ? c1.Clone() : new b2Vec2();
	this.ey = c2 ? c2.Clone() : new b2Vec2();
}

b2Mat22.prototype =
{
	/// Initialize this matrix using columns.
	Set: function(c1, c2)
	{
		this.ex.Assign(c1);
		this.ey.Assign(c2);
	},

	Assign: function(mat)
	{
		this.ex.Assign(mat.ex);
		this.ey.Assign(mat.ey);
	},

	/// Set this to the identity matrix.
	SetIdentity: function()
	{
		this.ex.x = 1.0; this.ey.x = 0.0;
		this.ex.y = 0.0; this.ey.y = 1.0;
	},

	/// Set this matrix to all zeros.
	SetZero: function()
	{
		this.ex.x = 0.0; this.ey.x = 0.0;
		this.ex.y = 0.0; this.ey.y = 0.0;
	},

	GetInverse: function()
	{
		var a = this.ex.x, b = this.ey.x, c = this.ex.y, d = this.ey.y;
		var B = new b2Mat22();
		var det = a * d - b * c;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		B.ex.x =  det * d;	B.ey.x = -det * b;
		B.ex.y = -det * c;	B.ey.y =  det * a;
		return B;
	},

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	Solve: function(b)
	{
		var a11 = this.ex.x, a12 = this.ey.x, a21 = this.ex.y, a22 = this.ey.y;
		var det = a11 * a22 - a12 * a21;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		var x = new b2Vec2();
		x.x = det * (a22 * b.x - a12 * b.y);
		x.y = det * (a11 * b.y - a21 * b.x);
		return x;
	}
};

b2Mat22.Add = function(A, B)
{
	return new b2Mat22(b2Vec2.Add(A.ex, B.ex), b2Vec2.Add(A.ey, B.ey));
};

/// A 3-by-3 matrix. Stored in column-major order.
function b2Mat33(c1, c2, c3)
{
	this.ex = c1 ? c1.Clone() : new b2Vec3();
	this.ey = c2 ? c2.Clone() : new b2Vec3();
	this.ez = c3 ? c3.Clone() : new b2Vec3();
}

b2Mat33.prototype =
{
	/// Set this matrix to all zeros.
	SetZero: function()
	{
		this.ex.SetZero();
		this.ey.SetZero();
		this.ez.SetZero();
	},

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	Solve33: function(b)
	{
		var det = b2Dot_v3_v3(this.ex, b2Cross_v3_v3(this.ey, this.ez));
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		var x = new b2Vec3();
		x.x = det * b2Dot_v3_v3(b, b2Cross_v3_v3(this.ey, this.ez));
		x.y = det * b2Dot_v3_v3(this.ex, b2Cross_v3_v3(b, this.ez));
		x.z = det * b2Dot_v3_v3(this.ex, b2Cross_v3_v3(this.ey, b));
		return x;
	},

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	Solve22: function(b)
	{
		var a11 = this.ex.x, a12 = this.ey.x, a21 = this.ex.y, a22 = this.ey.y;
		var det = a11 * a22 - a12 * a21;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		var x = new b2Vec2();
		x.x = det * (a22 * b.x - a12 * b.y);
		x.y = det * (a11 * b.y - a21 * b.x);
		return x;
	},

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	GetInverse22: function(/* b2Mat33* */M)
	{
		var a = this.ex.x, b = this.ey.x, c = this.ex.y, d = this.ey.y;
		var det = a * d - b * c;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}

		M.ex.x =  det * d;	M.ey.x = -det * b; M.ex.z = 0.0;
		M.ex.y = -det * c;	M.ey.y =  det * a; M.ey.z = 0.0;
		M.ez.x = 0.0; M.ez.y = 0.0; M.ez.z = 0.0;
	},

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	GetSymInverse33: function(/* b2Mat33* */ M)
	{
		var det = b2Dot_v3_v3(this.ex, b2Cross_v3_v3(this.ey, this.ez));
		if (det != 0.0)
		{
			det = 1.0 / det;
		}

		var a11 = this.ex.x, a12 = this.ey.x, a13 = this.ez.x;
		var a22 = this.ey.y, a23 = this.ez.y;
		var a33 = this.ez.z;

		M.ex.x = det * (a22 * a33 - a23 * a23);
		M.ex.y = det * (a13 * a23 - a12 * a33);
		M.ex.z = det * (a12 * a23 - a13 * a22);

		M.ey.x = M.ex.y;
		M.ey.y = det * (a11 * a33 - a13 * a13);
		M.ey.z = det * (a13 * a12 - a11 * a23);

		M.ez.x = M.ex.z;
		M.ez.y = M.ey.z;
		M.ez.z = det * (a11 * a22 - a12 * a12);
	}
};

/// Rotation
function b2Rot(angle, c)
{
	if (typeof(c) !== 'undefined')
	{
		this.s = angle;
		this.c = c;
	}
	else if (typeof(angle) !== 'undefined')
		this.Set(angle);
}

b2Rot.prototype =
{
	Clone: function()
	{
		return new b2Rot(this.s, this.c);
	},

	Assign: function(l)
	{
		this.s = l.s;
		this.c = l.c;
	},

	/// Set using an angle in radians.
	Set: function(x)
	{
		/// TODO_ERIN optimize
		this.s = sinf(x);
		this.c = cosf(x);
	},

	/// Set to the identity rotation
	SetIdentity: function()
	{
		this.s = 0.0;
		this.c = 1.0;
	},

	/// Get the angle in radians
	GetAngle: function()
	{
		return b2Atan2(this.s, this.c);
	},

	/// Get the x-axis
	GetXAxis: function()
	{
		return new b2Vec2(this.c, this.s);
	},

	/// Get the u-axis
	GetYAxis: function()
	{
		return new b2Vec2(-this.s, this.c);
	},

	/// Sine and cosine
	s: 0,
	c: 1
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
function b2Transform(position, rotation)
{
	this.p = new b2Vec2();
	this.q = new b2Rot();

	if (position)
	{
		this.p.Assign(position);
		this.q.Assign(rotation);
	}
}

b2Transform.prototype =
{
	Clone: function()
	{
		var xf = new b2Transform(this.p, this.q);
		return xf;
	},

	Assign: function(xf)
	{
		this.p.Assign(xf.p);
		this.q.Assign(xf.q);
	},

	/// Set this to the identity transform.
	SetIdentity: function()
	{
		this.p.SetZero();
		this.q.SetIdentity();
	},

	/// Set this based on the position and angle.
	Set: function(position, angle)
	{
		this.p.Assign(position);
		this.q.Set(angle);
	}
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
function b2Sweep()
{
	this.localCenter = new b2Vec2();
	this.c0 = new b2Vec2();
	this.c = new b2Vec2();
}

b2Sweep.prototype =
{
	Assign: function(sweep)
	{
		this.localCenter.Assign(sweep.localCenter);
		this.c0.Assign(sweep.c0);
		this.c.Assign(sweep.c);
		this.a = sweep.a;
		this.a0 = sweep.a0;
		this.alpha0 = sweep.alpha0;
	},

	Clone: function()
	{
		var sweep = new b2Sweep();
		sweep.localCenter.Assign(this.localCenter);
		sweep.c0.Assign(this.c0);
		sweep.c.Assign(this.c);
		sweep.a = this.a;
		sweep.a0 = this.a0;
		sweep.alpha0 = this.alpha0;
		return sweep;
	},

	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	GetTransform: function(/* b2Transform* */ xf, beta)
	{
		xf.p = b2Vec2.Add(b2Vec2.Multiply((1.0 - beta), this.c0), b2Vec2.Multiply(beta, this.c));
		var angle = (1.0 - beta) * this.a0 + beta * this.a;
		xf.q.Set(angle);

		// Shift to origin
		xf.p.Subtract(b2Mul_r_v2(xf.q, this.localCenter));
	},

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	Advance: function(alpha)
	{
'#if @DEBUG';
		b2Assert(this.alpha0 < 1.0);
'#endif';
		var beta = (alpha - this.alpha0) / (1.0 - this.alpha0);
		this.c0.Add(b2Vec2.Multiply(beta, b2Vec2.Subtract(this.c, this.c0)));
		this.a0 += beta * (this.a - this.a0);
		this.alpha0 = alpha;
	},

	/// Normalize the angles.
	Normalize: function()
	{
		var twoPi = 2.0 * b2_pi;
		var d = twoPi * floorf(this.a0 / twoPi);
		this.a0 -= d;
		this.a -= d;
	},

	a0: 0,
	a: 0,		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	alpha0: 0
};

/// Perform the dot product on two vectors.
function b2Dot_v2_v2(a, b)
{
	return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
function b2Cross_v2_v2(a, b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
function b2Cross_v2_f(a, s)
{
	return new b2Vec2(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
function b2Cross_f_v2(s, a)
{
	return new b2Vec2(-s * a.y, s * a.x);
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
function b2Mul_m22_v2(A, v)
{
	return new b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
function b2MulT_m22_v2(A, v)
{
	return new b2Vec2(b2Dot_v2_v2(v, A.ex), b2Dot_v2_v2(v, A.ey));
}

function b2Distance(a, b)
{
	var c = b2Vec2.Subtract(a, b);
	return c.Length();
}

function b2DistanceSquared(a, b)
{
	var c = b2Vec2.Subtract(a, b);
	return b2Dot_v2_v2(c, c);
}

/// Perform the dot product on two vectors.
function b2Dot_v3_v3(a, b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
function b2Cross_v3_v3(a, b)
{
	return new b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// A * B
function b2Mul_m22_m22(A, B)
{
	return new b2Mat22(b2Mul_m22_v2(A, B.ex), b2Mul_m22_v2(A, B.ey));
}

// A^T * B
function b2MulT_m22_m22(A, B)
{
	var c1 = new b2Vec2(b2Dot_v2_v2(A.ex, B.ex), b2Dot_v2_v2(A.ey, B.ex));
	var c2 = new b2Vec2(b2Dot_v2_v2(A.ex, B.ey), b2Dot_v2_v2(A.ey, B.ey));
	return new b2Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
function b2Mul_m33_v3(A, v)
{
	return b2Vec3.Add(b2Vec3.Add(b2Vec3.Multiply(v.x, A.ex), b2Vec3.Multiply(v.y, A.ey)), b2Vec3.Multiply(v.z, A.ez));
}

/// Multiply a matrix times a vector.
function b2Mul22_m33_v2(A, v)
{
	return new b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply two rotations: q * r
function b2Mul_r_r(q, r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	var qr = new b2Rot();
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: qT * r
function b2MulT_r_r(q, r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	var qr = new b2Rot();
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
	return qr;
}

/// Rotate a vector
function b2Mul_r_v2(q, v)
{
	return new b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/// Inverse rotate a vector
function b2MulT_r_v2(q, v)
{
	return new b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

function b2Mul_t_v2(T, v)
{
	return new b2Vec2((T.q.c * v.x - T.q.s * v.y) + T.p.x, (T.q.s * v.x + T.q.c * v.y) + T.p.y);
}

function b2MulT_t_v2(T, v)
{
	var px = v.x - T.p.x;
	var py = v.y - T.p.y;
	var x = (T.q.c * px + T.q.s * py);
	var y = (-T.q.s * px + T.q.c * py);

	return new b2Vec2(x, y);
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
function b2Mul_t_t(A, B)
{
	var C = new b2Transform();
	C.q = b2Mul_r_r(A.q, B.q);
	C.p = b2Vec2.Add(b2Mul_r_v2(A.q, B.p), A.p);
	return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
function b2MulT_t_t(A, B)
{
	var C = new b2Transform();
	C.q = b2MulT_r_r(A.q, B.q);
	C.p = b2MulT_r_v2(A.q, b2Vec2.Subtract(B.p, A.p));
	return C;
}

var b2Abs = Math.abs;

function b2Abs_v2(a)
{
	return new b2Vec2(b2Abs(a.x), b2Abs(a.y));
}

function b2Abs_m22(A)
{
	return new b2Mat22(b2Abs_v2(A.ex), b2Abs_v2(A.ey));
}

var b2Min = Math.min;

function b2Min_v2(a, b)
{
	return new b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
}

var b2Max = Math.max;

function b2Max_v2(a, b)
{
	return new b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
}

function b2Clamp(a, low, high)
{
	return b2Max(low, b2Min(a, high));
}

function b2Clamp_v2(a, low, high)
{
	return b2Max_v2(low, b2Min_v2(a, high));
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
function b2NextPowerOfTwo(x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

function b2IsPowerOfTwo(x)
{
	var result = x > 0 && (x & (x - 1)) == 0;
	return result;
}

var RAND_LIMIT	= 32767;

/// Random number in range [-1,1] or [lo,hi]
function b2RandomFloat(lo, hi)
{
	var r = Math.random();

	if (typeof(lo) !== 'undefined')
		r = (hi - lo) * r + lo;
	else
		r = 2.0 * r - 1.0;

	return r;
}