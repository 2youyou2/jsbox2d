function TestEdge()
{
	this.parent.call(this);
}

TestEdge.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var v1 = new b2Vec2(-10.0, 0.0), v2 = new b2Vec2(-7.0, -2.0), v3 = new b2Vec2(-4.0, 0.0);
			var v4 = new b2Vec2(0.0, 0.0), v5 = new b2Vec2(4.0, 0.0), v6 = new b2Vec2(7.0, 2.0), v7 = new b2Vec2(10.0, 0.0);

			var shape = new b2EdgeShape();

			shape.Set(v1, v2);
			shape.m_hasVertex3 = true;
			shape.m_vertex3 = v3;
			ground.CreateFixture(shape, 0.0);

			shape.Set(v2, v3);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v1;
			shape.m_vertex3 = v4;
			ground.CreateFixture(shape, 0.0);

			shape.Set(v3, v4);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v2;
			shape.m_vertex3 = v5;
			ground.CreateFixture(shape, 0.0);

			shape.Set(v4, v5);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v3;
			shape.m_vertex3 = v6;
			ground.CreateFixture(shape, 0.0);

			shape.Set(v5, v6);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v4;
			shape.m_vertex3 = v7;
			ground.CreateFixture(shape, 0.0);

			shape.Set(v6, v7);
			shape.m_hasVertex0 = true;
			shape.m_vertex0 = v5;
			ground.CreateFixture(shape, 0.0);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-0.5, 0.6);
			bd.allowSleep = false;
			var body = this.m_world.CreateBody(bd);

			var shape = new b2CircleShape();
			shape.m_radius = 0.5;

			body.CreateFixture(shape, 1.0);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(1.0, 0.6);
			bd.allowSleep = false;
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.5);

			body.CreateFixture(shape, 1.0);
		}
	}
};

TestEdge._extend(Test);