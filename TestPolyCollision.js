function TestPolyCollision()
{
	this.parent.call(this);
}

TestPolyCollision.prototype =
{
	Initialize: function()
	{
		this.m_polygonA = new b2PolygonShape();
		this.m_polygonB = new b2PolygonShape();
		this.m_transformA = new b2Transform();
		this.m_transformB = new b2Transform();
		this.m_positionA = new b2Vec2();
		this.m_positionB = new b2Vec2();

		{
			this.m_polygonA.SetAsBox(0.2, 0.4);
			this.m_transformA.Set(new b2Vec2(0.0, 0.0), 0.0);
		}

		{
			this.m_polygonB.SetAsBox(0.5, 0.5);
			this.m_positionB.Set(19.345284, 1.5632932);
			this.m_angleB = 1.9160721;
			this.m_transformB.Set(this.m_positionB, this.m_angleB);
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var manifold = new b2Manifold();
		b2CollidePolygons(manifold, this.m_polygonA, this.m_transformA, this.m_polygonB, this.m_transformB);

		var worldManifold = new b2WorldManifold();
		worldManifold.Initialize(manifold, this.m_transformA, this.m_polygonA.m_radius, this.m_transformB, this.m_polygonB.m_radius);

		this.m_drawStringFunc("point count = " + manifold.pointCount);

		{
			var color = new b2Color(0.9, 0.9, 0.9);
			var v = [];
			for (var i = 0; i < this.m_polygonA.m_count; ++i)
			{
				v[i] = b2Mul_t_v2(this.m_transformA, this.m_polygonA.m_vertices[i]);
			}
			this.m_debugDraw.DrawPolygon(v, this.m_polygonA.m_count, color);

			for (i = 0; i < this.m_polygonB.m_count; ++i)
			{
				v[i] = b2Mul_t_v2(this.m_transformB, this.m_polygonB.m_vertices[i]);
			}
			this.m_debugDraw.DrawPolygon(v, this.m_polygonB.m_count, color);
		}

		for (var i = 0; i < manifold.pointCount; ++i)
		{
			this.m_debugDraw.DrawPoint(worldManifold.points[i], 4.0 / 14, new b2Color(0.9, 0.3, 0.3));
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
	}
};

TestPolyCollision._extend(Test);