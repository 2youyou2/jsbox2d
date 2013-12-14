function TestDistance()
{
	this.parent.call(this);
}

TestDistance.prototype =
{
	Initialize: function()
	{
		this.m_positionB = new b2Vec2();
		this.m_angleB;

		this.m_transformA = new b2Transform();
		this.m_transformB = new b2Transform();
		this.m_polygonA = new b2PolygonShape();
		this.m_polygonB = new b2PolygonShape();

		{
			this.m_transformA.SetIdentity();
			this.m_transformA.p.Set(0.0, -0.2);
			this.m_polygonA.SetAsBox(10.0, 0.2);
		}

		{
			this.m_positionB.Set(12.017401, 0.13678508);
			this.m_angleB = -0.0109265;
			this.m_transformB.Set(this.m_positionB, this.m_angleB);

			this.m_polygonB.SetAsBox(2.0, 0.1);
		}
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'A'.charCodeAt():
			this.m_positionB.x -= 0.1;
			break;

		case 'D'.charCodeAt():
			this.m_positionB.x += 0.1;
			break;

		case 'S'.charCodeAt():
			this.m_positionB.y -= 0.1;
			break;

		case 'W'.charCodeAt():
			this.m_positionB.y += 0.1;
			break;

		case 'Q'.charCodeAt():
			this.m_angleB += 0.1 * Math.PI;
			break;

		case 'E'.charCodeAt():
			this.m_angleB -= 0.1 * Math.PI;
			break;
		}

		this.m_transformB.Set(this.m_positionB, this.m_angleB);
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var input = new b2DistanceInput();
		input.proxyA.Set(this.m_polygonA, 0);
		input.proxyB.Set(this.m_polygonB, 0);
		input.transformA.Assign(this.m_transformA);
		input.transformB.Assign(this.m_transformB);
		input.useRadii = true;
		var cache = new b2SimplexCache();
		cache.count = 0;
		var output = new b2DistanceOutput();
		b2DistanceFunc(output, cache, input);

		this.m_drawStringFunc("distance = " + output.distance);
		this.m_drawStringFunc("iterations = " + output.iterations);

		{
			var color = new b2Color(0.9, 0.9, 0.9);
			var v = [];
			for (var i = 0; i < this.m_polygonA.m_count; ++i)
			{
				v[i] = (b2Mul_t_v2(this.m_transformA, this.m_polygonA.m_vertices[i]));
			}
			this.m_debugDraw.DrawPolygon(v, this.m_polygonA.m_count, color);

			for (var i = 0; i < this.m_polygonB.m_count; ++i)
			{
				v[i] = (b2Mul_t_v2(this.m_transformB, this.m_polygonB.m_vertices[i]));
			}
			this.m_debugDraw.DrawPolygon(v, this.m_polygonB.m_count, color);
		}

		var x1 = output.pointA;
		var x2 = output.pointB;

		var c1 = new b2Color(1.0, 0.0, 0.0);
		this.m_debugDraw.DrawPoint(x1, 4.0 / 14, c1);

		var c2 = new b2Color(1.0, 1.0, 0.0);
		this.m_debugDraw.DrawPoint(x2, 4.0 / 14, c2);
	}
};

TestDistance._extend(Test);