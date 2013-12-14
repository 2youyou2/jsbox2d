function TestConvexHull()
{
	this.parent.call(this);
}

TestConvexHull.prototype =
{
	Initialize: function()
	{
		this.Generate();
		this.m_auto = false;
	},

	Generate: function()
	{
		var lowerBound = new b2Vec2(-8.0, -8.0);
		var upperBound = new b2Vec2(8.0, 8.0);

		this.m_points = [];

		for (var i = 0; i < b2_maxPolygonVertices; ++i)
		{
			var x = 10.0 * b2RandomFloat();
			var y = 10.0 * b2RandomFloat();

			// Clamp onto a square to help create collinearities.
			// This will stress the convex hull algorithm.
			var v = new b2Vec2(x, y);
			v.Assign(b2Clamp_v2(v, lowerBound, upperBound));
			this.m_points[i] = v;
		}

		this.m_count = b2_maxPolygonVertices;
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'A'.charCodeAt():
			this.m_auto = !this.m_auto;
			break;

		case 'G'.charCodeAt():
			this.Generate();
			break;
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var shape = new b2PolygonShape();
		shape.Set(this.m_points, this.m_count);

		this.m_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, new b2Color(0.9, 0.9, 0.9));

		for (var i = 0; i < this.m_count; ++i)
			this.m_debugDraw.DrawPoint(this.m_points[i], 2.0 / 14, new b2Color(0.9, 0.5, 0.5));

		if (this.m_auto)
		{
			this.Generate();
		}
	}
};

TestConvexHull._extend(Test);