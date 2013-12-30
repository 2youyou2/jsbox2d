function TestChaosPendulum()
{
	this.parent.call(this);
}

TestChaosPendulum.prototype =
{
	Initialize: function()
	{
		this.m_world.SetGravity(new b2Vec2(0, -50));
		{
			var bd = new b2BodyDef();
			bd.position.Set(10, 25);
			var ground = this.m_world.CreateBody(bd);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-20, 25);
			bd.linearVelocity.Set(b2RandomFloat(-20, 20), b2RandomFloat(-20, 20));
			var fd = new b2FixtureDef();
			fd.density = 0.4;
			fd.shape = new b2CircleShape();
			fd.shape.m_radius = 1;
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(fd);

			var rjd = new b2DistanceJointDef();
			rjd.Initialize(ground, body, ground.GetWorldCenter(), body.GetWorldCenter());
			this.m_world.CreateJoint(rjd);

			bd.position.Set(-30, 25);
			bd.linearVelocity.Set(b2RandomFloat(-20, 20), b2RandomFloat(-20, 20));
			var body2 = this.m_world.CreateBody(bd);
			fd.density = 0.4;
			body2.CreateFixture(fd);

			var rjd = new b2DistanceJointDef();
			rjd.Initialize(body, body2, body.GetWorldCenter(), body2.GetWorldCenter());
			this.m_world.CreateJoint(rjd);
		}
	}
};

TestChaosPendulum._extend(Test);