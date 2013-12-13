function TestPrismatic()
{
	this.parent.call(this);
}

TestPrismatic.prototype =
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
			shape.SetAsBox(2.0, 0.5);

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-10.0, 10.0);
			bd.angle = 0.5 * b2_pi;
			bd.allowSleep = false;
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(shape, 5.0);

			var pjd = new b2PrismaticJointDef();

			// Bouncy limit
			var axis = new b2Vec2(2.0, 1.0);
			axis.Normalize();
			pjd.Initialize(ground, body, new b2Vec2(0.0, 0.0), axis);

			// Non-bouncy limit
			//pjd.Initialize(ground, body, b2Vec2(-10.0, 10.0), b2Vec2(1.0, 0.0));

			pjd.motorSpeed = 10.0;
			pjd.maxMotorForce = 10000.0;
			pjd.enableMotor = true;
			pjd.lowerTranslation = 0.0;
			pjd.upperTranslation = 20.0;
			pjd.enableLimit = true;

			this.m_joint = this.m_world.CreateJoint(pjd);
		}
	},

	Keyboard: function(key)
	{
		switch (key)
		{
		case 'L'.charCodeAt():
			this.m_joint.EnableLimit(!this.m_joint.IsLimitEnabled());
			break;

		case 'M'.charCodeAt():
			this.m_joint.EnableMotor(!this.m_joint.IsMotorEnabled());
			break;

		case 'S'.charCodeAt():
			this.m_joint.SetMotorSpeed(-this.m_joint.GetMotorSpeed());
			break;
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);
		this.m_drawStringFunc("Keys: (l) limits, (m) motors, (s) speed");
		var force = this.m_joint.GetMotorForce(this.m_hz_raw);
		this.m_drawStringFunc("Motor Force = " + force);
	}
};

TestPrismatic._extend(Test);