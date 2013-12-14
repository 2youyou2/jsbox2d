function TestTumbler()
{
	this.parent.call(this);
}

TestTumbler.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.allowSleep = false;
			bd.position.Set(0.0, 10.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 10.0, new b2Vec2(10.0, 0.0), 0.0);
			body.CreateFixture(shape, 5.0);
			shape.SetAsBox(0.5, 10.0, new b2Vec2(-10.0, 0.0), 0.0);
			body.CreateFixture(shape, 5.0);
			shape.SetAsBox(10.0, 0.5, new b2Vec2(0.0, 10.0), 0.0);
			body.CreateFixture(shape, 5.0);
			shape.SetAsBox(10.0, 0.5, new b2Vec2(0.0, -10.0), 0.0);
			body.CreateFixture(shape, 5.0);

			var jd = new b2RevoluteJointDef();
			jd.bodyA = ground;
			jd.bodyB = body;
			jd.localAnchorA.Set(0.0, 10.0);
			jd.localAnchorB.Set(0.0, 0.0);
			jd.referenceAngle = 0.0;
			jd.motorSpeed = 0.05 * Math.PI;
			jd.maxMotorTorque = 1e8;
			jd.enableMotor = true;
			this.m_joint = this.m_world.CreateJoint(jd);
		}

		this.m_count = 0;
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var e_count = 400;

		if (this.m_count < e_count)
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 10.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.125, 0.125);
			body.CreateFixture(shape, 1.0);

			++this.m_count;
		}
	}
};

TestTumbler._extend(Test);