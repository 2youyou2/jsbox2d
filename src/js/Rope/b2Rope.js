/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

var expf = Math.exp;

///
function b2RopeDef()
{
	this.vertices = null;
	this.count = 0;
	this.masses = null;
	this.gravity = new b2Vec2();
	this.damping = 0.1;

	/// Stretching stiffness
	this.k2 = 0.9;

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	this.k3 = 0.1;
}

///
function b2Rope()
{
	this.m_count = 0;
	this.m_ps = null;
	this.m_p0s = null;
	this.m_vs = null;
	this.m_ims = null;
	this.m_Ls = null;
	this.m_as = null;
	this.m_damping = 0;
	this.m_gravity = new b2Vec2();
	this.m_k2 = 1.0;
	this.m_k3 = 0.1;
}

b2Rope.prototype =
{
	///
	Initialize: function(def)
	{
'#if @DEBUG';
		b2Assert(def.count >= 3);
'#endif';
		this.m_count = def.count;
		this.m_ps = new Array(this.m_count);
		this.m_p0s = new Array(this.m_count);
		this.m_vs = new Array(this.m_count);
		this.m_ims = new Array(this.m_count);

		for (var i = 0; i < this.m_count; ++i)
		{
			this.m_ps[i] = def.vertices[i].Clone();
			this.m_p0s[i] = def.vertices[i].Clone();
			this.m_vs[i] = new b2Vec2();

			var m = def.masses[i];
			if (m > 0.0)
			{
				this.m_ims[i] = 1.0 / m;
			}
			else
			{
				this.m_ims[i] = 0.0;
			}
		}

		var count2 = this.m_count - 1;
		var count3 = this.m_count - 2;
		this.m_Ls = new Array(count2);
		this.m_as = new Array(count3);

		for (var i = 0; i < count2; ++i)
		{
			var p1 = this.m_ps[i];
			var p2 = this.m_ps[i+1];
			this.m_Ls[i] = b2Distance(p1, p2);
		}

		for (var i = 0; i < count3; ++i)
		{
			var p1 = this.m_ps[i];
			var p2 = this.m_ps[i + 1];
			var p3 = this.m_ps[i + 2];

			var d1 = b2Vec2.Subtract(p2, p1);
			var d2 = b2Vec2.Subtract(p3, p2);

			var a = b2Cross_v2_v2(d1, d2);
			var b = b2Dot_v2_v2(d1, d2);

			this.m_as[i] = b2Atan2(a, b);
		}

		this.m_gravity = def.gravity.Clone();
		this.m_damping = def.damping;
		this.m_k2 = def.k2;
		this.m_k3 = def.k3;
	},

	///
	Step: function(h, iterations)
	{
		if (h == 0.0)
		{
			return;
		}

		var d = expf(- h * this.m_damping);

		for (var i = 0; i < this.m_count; ++i)
		{
			this.m_p0s[i].Assign(this.m_ps[i]);
			if (this.m_ims[i] > 0.0)
			{
				this.m_vs[i].Add(b2Vec2.Multiply(h, this.m_gravity));
			}
			this.m_vs[i].Multiply(d);
			this.m_ps[i].Add(b2Vec2.Multiply(h, this.m_vs[i]));

		}

		for (var i = 0; i < iterations; ++i)
		{
			this.SolveC2();
			this.SolveC3();
			this.SolveC2();
		}

		var inv_h = 1.0 / h;
		for (var i = 0; i < this.m_count; ++i)
		{
			this.m_vs[i] = b2Vec2.Multiply(inv_h, b2Vec2.Subtract(this.m_ps[i], this.m_p0s[i]));
		}
	},

	///
	GetVertexCount: function()
	{
		return this.m_count;
	},

	///
	GetVertices: function()
	{
		return this.m_ps;
	},

	///
	Draw: function(draw)
	{
		var c = new b2Color(0.4, 0.5, 0.7);

		for (var i = 0; i < this.m_count - 1; ++i)
		{
			draw.DrawSegment(this.m_ps[i], this.m_ps[i+1], c);
		}
	},

	///
	SetAngle: function(angle)
	{
		var count3 = this.m_count - 2;
		for (var i = 0; i < count3; ++i)
		{
			this.m_as[i] = angle;
		}
	},

	SolveC2: function()
	{
		var count2 = this.m_count - 1;

		for (var i = 0; i < count2; ++i)
		{
			var p1 = this.m_ps[i];
			var p2 = this.m_ps[i + 1];

			var d = b2Vec2.Subtract(p2, p1);
			var L = d.Normalize();

			var im1 = this.m_ims[i];
			var im2 = this.m_ims[i + 1];

			if (im1 + im2 == 0.0)
			{
				continue;
			}

			var s1 = im1 / (im1 + im2);
			var s2 = im2 / (im1 + im2);

			p1.Subtract(b2Vec2.Multiply(this.m_k2 * s1 * (this.m_Ls[i] - L), d));
			p2.Add(b2Vec2.Multiply(this.m_k2 * s2 * (this.m_Ls[i] - L), d));
		}
	},
	SolveC3: function()
	{
		var count3 = this.m_count - 2;

		for (var i = 0; i < count3; ++i)
		{
			var p1 = this.m_ps[i];
			var p2 = this.m_ps[i + 1];
			var p3 = this.m_ps[i + 2];

			var m1 = this.m_ims[i];
			var m2 = this.m_ims[i + 1];
			var m3 = this.m_ims[i + 2];

			var d1 = b2Vec2.Subtract(p2, p1);
			var d2 = b2Vec2.Subtract(p3, p2);

			var L1sqr = d1.LengthSquared();
			var L2sqr = d2.LengthSquared();

			if (L1sqr * L2sqr == 0.0)
			{
				continue;
			}

			var a = b2Cross_v2_v2(d1, d2);
			var b = b2Dot_v2_v2(d1, d2);

			var angle = b2Atan2(a, b);

			var Jd1 = b2Vec2.Multiply((-1.0 / L1sqr), d1.Skew());
			var Jd2 = b2Vec2.Multiply((1.0 / L2sqr), d2.Skew());

			var J1 = b2Vec2.Negate(Jd1);
			var J2 = b2Vec2.Subtract(Jd1, Jd2);
			var J3 = Jd2;

			var mass = m1 * b2Dot_v2_v2(J1, J1) + m2 * b2Dot_v2_v2(J2, J2) + m3 * b2Dot_v2_v2(J3, J3);
			if (mass == 0.0)
			{
				continue;
			}

			mass = 1.0 / mass;

			var C = angle - this.m_as[i];

			while (C > b2_pi)
			{
				angle -= 2 * b2_pi;
				C = angle - this.m_as[i];
			}

			while (C < -b2_pi)
			{
				angle += 2.0 * b2_pi;
				C = angle - this.m_as[i];
			}

			var impulse = - this.m_k3 * mass * C;

			p1.Add(b2Vec2.Multiply((m1 * impulse), J1));
			p2.Add(b2Vec2.Multiply((m2 * impulse), J2));
			p3.Add(b2Vec2.Multiply((m3 * impulse), J3));
		}
	}
};
