function TestCompoundShapes()
{
	this.parent.call(this);
}

TestCompoundShapes.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			bd.position.Set(0.0, 0.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(50.0, 0.0), new b2Vec2(-50.0, 0.0));

			body.CreateFixture(shape, 0.0);
		}

		{
			var circle1 = new b2CircleShape();
			circle1.m_radius = 0.5;
			circle1.m_p.Set(-0.5, 0.5);

			var circle2 = new b2CircleShape();
			circle2.m_radius = 0.5;
			circle2.m_p.Set(0.5, 0.5);

			for (var i = 0; i < 10; ++i)
			{
				var x = b2RandomFloat(-0.1, 0.1);
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(x + 5.0, 1.05 + 2.5 * i);
				bd.angle = b2RandomFloat(-Math.PI, Math.PI);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(circle1, 2.0);
				body.CreateFixture(circle2, 0.0);
			}
		}

		{
			var polygon1 = new b2PolygonShape();
			polygon1.SetAsBox(0.25, 0.5);

			var polygon2 = new b2PolygonShape();
			polygon2.SetAsBox(0.25, 0.5, new b2Vec2(0.0, -0.5), 0.5 * Math.PI);

			for (var i = 0; i < 10; ++i)
			{
				var x = b2RandomFloat(-0.1, 0.1);
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(x - 5.0, 1.05 + 2.5 * i);
				bd.angle = b2RandomFloat(-Math.PI, Math.PI);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(polygon1, 2.0);
				body.CreateFixture(polygon2, 2.0);
			}
		}

		{
			var xf1 = new b2Transform();
			xf1.q.Set(0.3524 * Math.PI);
			xf1.p = xf1.q.GetXAxis();

			var vertices = [];

			var triangle1 = new b2PolygonShape();
			vertices[0] = b2Mul_t_v2(xf1, new b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf1, new b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf1, new b2Vec2(0.0, 0.5));
			triangle1.Set(vertices, 3);

			var xf2 = new b2Transform();
			xf2.q.Set(-0.3524 * Math.PI);
			xf2.p = xf2.q.GetXAxis().Negate();

			var triangle2 = new b2PolygonShape();
			vertices[0] = b2Mul_t_v2(xf2, new b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf2, new b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf2, new b2Vec2(0.0, 0.5));
			triangle2.Set(vertices, 3);

			for (var i = 0; i < 10; ++i)
			{
				var x = b2RandomFloat(-0.1, 0.1);
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(x, 2.05 + 2.5 * i);
				bd.angle = 0.0;
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(triangle1, 2.0);
				body.CreateFixture(triangle2, 2.0);
			}
		}

		{
			var bottom = new b2PolygonShape();
			bottom.SetAsBox( 1.5, 0.15 );

			var left = new b2PolygonShape();
			left.SetAsBox(0.15, 2.7, new b2Vec2(-1.45, 2.35), 0.2);

			var right = new b2PolygonShape();
			right.SetAsBox(0.15, 2.7, new b2Vec2(1.45, 2.35), -0.2);

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set( 0.0, 2.0 );
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(bottom, 4.0);
			body.CreateFixture(left, 4.0);
			body.CreateFixture(right, 4.0);
		}
	}
};

TestCompoundShapes._extend(Test);