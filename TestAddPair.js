function TestAddPair()
{
	this.parent.call(this);
}

TestAddPair.prototype =
{
	Initialize: function()
	{
		this.m_world.SetGravity(new b2Vec2(0.0,0.0));
		{
			var shape = new b2CircleShape();
			shape.m_p.SetZero();
			shape.m_radius = 0.1;

			var minX = -6.0;
			var maxX = 0.0;
			var minY = 4.0;
			var maxY = 6.0;

			for (var i = 0; i < 400; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Assign(new b2Vec2(b2RandomFloat(minX,maxX),b2RandomFloat(minY,maxY)));
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(shape, 0.01);
			}
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(1.5, 1.5);
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-40.0,5.0);
			bd.bullet = true;
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(shape, 1.0);
			body.SetLinearVelocity(new b2Vec2(150.0, 0.0));
		}
	}
};

TestAddPair._extend(Test);