/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/b2Math.h>

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
b2Vec3 b2Mat33::Solve33(const b2Vec3& b) const
{
	float32 det = b2Dot_v3_v3(this->ex, b2Cross_v3_v3(this->ey, this->ez));
	if (det != 0.0)
	{
		det = 1.0 / det;
	}
	b2Vec3 x;
	x.x = det * b2Dot_v3_v3(b, b2Cross_v3_v3(this->ey, this->ez));
	x.y = det * b2Dot_v3_v3(this->ex, b2Cross_v3_v3(b, this->ez));
	x.z = det * b2Dot_v3_v3(this->ex, b2Cross_v3_v3(this->ey, b));
	return x;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
b2Vec2 b2Mat33::Solve22(const b2Vec2& b) const
{
	float32 a11 = this->ex.x, a12 = this->ey.x, a21 = this->ex.y, a22 = this->ey.y;
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

///
void b2Mat33::GetInverse22(b2Mat33* M) const
{
	float32 a = this->ex.x, b = this->ey.x, c = this->ex.y, d = this->ey.y;
	float32 det = a * d - b * c;
	if (det != 0.0)
	{
		det = 1.0 / det;
	}

	M->ex.x =  det * d;	M->ey.x = -det * b; M->ex.z = 0.0;
	M->ex.y = -det * c;	M->ey.y =  det * a; M->ey.z = 0.0;
	M->ez.x = 0.0; M->ez.y = 0.0; M->ez.z = 0.0;
}

/// Returns the zero matrix if singular.
void b2Mat33::GetSymInverse33(b2Mat33* M) const
{
	float32 det = b2Dot_v3_v3(this->ex, b2Cross_v3_v3(this->ey, this->ez));
	if (det != 0.0)
	{
		det = 1.0 / det;
	}

	float32 a11 = this->ex.x, a12 = this->ey.x, a13 = this->ez.x;
	float32 a22 = this->ey.y, a23 = this->ez.y;
	float32 a33 = this->ez.z;

	M->ex.x = det * (a22 * a33 - a23 * a23);
	M->ex.y = det * (a13 * a23 - a12 * a33);
	M->ex.z = det * (a12 * a23 - a13 * a22);

	M->ey.x = M->ex.y;
	M->ey.y = det * (a11 * a33 - a13 * a13);
	M->ey.z = det * (a13 * a12 - a11 * a23);

	M->ez.x = M->ex.z;
	M->ez.y = M->ey.z;
	M->ez.z = det * (a11 * a22 - a12 * a12);
}
