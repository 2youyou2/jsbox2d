function TestChain()
{
	this.parent.call(this);
}

TestChain.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.6, 0.125);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;
			fd.friction = 0.2;

			var jd = new b2RevoluteJointDef();
			jd.collideConnected = false;

			var y = 25.0;
			var prevBody = ground;
			for (var i = 0; i < 30; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.angularDamping = 2;
				bd.linearDamping = 2;
				bd.position.Set(0.5 + i, y);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(fd);

				var anchor = new b2Vec2(i, y);
				jd.Initialize(prevBody, body, anchor);
				this.m_world.CreateJoint(jd);

				prevBody = body;
			}
		}
	}
};

TestChain._extend(Test);