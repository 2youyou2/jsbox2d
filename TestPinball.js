function TestPinball()
{
	this.parent.call(this);
}

TestPinball.prototype =
{
	Initialize: function()
	{
		// Ground body
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);

			var vs = [new b2Vec2(0.0, -2.0), new b2Vec2(8.0, 6.0), new b2Vec2(8.0, 20.0), new b2Vec2(-8.0, 20.0), new b2Vec2(-8.0, 6.0)];

			var loop = new b2ChainShape();
			loop.CreateLoop(vs, 5);
			var fd = new b2FixtureDef();
			fd.shape = loop;
			fd.density = 0.0;
			ground.CreateFixture(fd);
		}

		// Flippers
		{
			var p1 = new b2Vec2(-2.0, 0.0), p2 = new b2Vec2(2.0, 0.0);

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;

			bd.position = p1;
			var leftFlipper = this.m_world.CreateBody(bd);

			bd.position = p2;
			var rightFlipper = this.m_world.CreateBody(bd);

			var box = new b2PolygonShape();
			box.SetAsBox(1.75, 0.1);

			var fd = new b2FixtureDef();
			fd.shape = box;
			fd.density = 1.0;

			leftFlipper.CreateFixture(fd);
			rightFlipper.CreateFixture(fd);

			var jd = new b2RevoluteJointDef();
			jd.bodyA = ground;
			jd.localAnchorB.SetZero();
			jd.enableMotor = true;
			jd.maxMotorTorque = 1000.0;
			jd.enableLimit = true;

			jd.motorSpeed = 0.0;
			jd.localAnchorA = p1;
			jd.bodyB = leftFlipper;
			jd.lowerAngle = -30.0 * Math.PI / 180.0;
			jd.upperAngle = 5.0 * Math.PI / 180.0;
			this.m_leftJoint = this.m_world.CreateJoint(jd);

			jd.motorSpeed = 0.0;
			jd.localAnchorA = p2;
			jd.bodyB = rightFlipper;
			jd.lowerAngle = -5.0 * Math.PI / 180.0;
			jd.upperAngle = 30.0 * Math.PI / 180.0;
			this.m_rightJoint = this.m_world.CreateJoint(jd);
		}

		// Circle character
		{
			var bd = new b2BodyDef();
			bd.position.Set(1.0, 15.0);
			bd.type = b2Body.b2_dynamicBody;
			bd.bullet = true;

			this.m_ball = this.m_world.CreateBody(bd);

			var shape = new b2CircleShape();
			shape.m_radius = 0.2;

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 1.0;
			this.m_ball.CreateFixture(fd);
		}

		this.m_button = false;
	},

	Step: function()
	{
		if (this.m_button)
		{
			this.m_leftJoint.SetMotorSpeed(20.0);
			this.m_rightJoint.SetMotorSpeed(-20.0);
		}
		else
		{
			this.m_leftJoint.SetMotorSpeed(-10.0);
			this.m_rightJoint.SetMotorSpeed(10.0);
		}

		this.parent.prototype.Step.call(this);
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'A'.charCodeAt():
			this.m_button = true;
			break;
		}
	},

	KeyboardUp: function(key)
	{
		this.parent.prototype.KeyboardUp.call(this, key);

		switch (key)
		{
		case 'A'.charCodeAt():
			this.m_button = false;
			break;
		}
	}
};

TestPinball._extend(Test);