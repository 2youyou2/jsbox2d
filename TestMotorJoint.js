function TestMotorJoint()
{
	this.parent.call(this);
}

TestMotorJoint.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-20.0, 0.0), new b2Vec2(20.0, 0.0));

			var fd = new b2FixtureDef();
			fd.shape = shape;

			ground.CreateFixture(fd);
		}

		// Define motorized body
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 8.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(2.0, 0.5);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.friction = 0.6;
			fd.density = 2.0;
			body.CreateFixture(fd);

			var mjd = new b2MotorJointDef();
			mjd.Initialize(ground, body);
			mjd.maxForce = 1000.0;
			mjd.maxTorque = 1000.0;
			this.m_joint = this.m_world.CreateJoint(mjd);
		}

		this.m_go = false;
		this.m_time = 0.0;
	},

	Keyboard: function(key)
	{
		switch (key)
		{
		case 'S'.charCodeAt():
			this.m_go = !this.m_go;
			break;
		}
	},

	Step: function()
	{
		if (this.m_go && this.m_hz > 0.0)
		{
			this.m_time += this.m_hz;
		}

		var linearOffset = new b2Vec2();
		linearOffset.x = 6.0 * Math.sin(2.0 * this.m_time);
		linearOffset.y = 8.0 + 4.0 * Math.sin(1.0 * this.m_time);

		var angularOffset = 4.0 * this.m_time;

		this.m_joint.SetLinearOffset(linearOffset);
		this.m_joint.SetAngularOffset(angularOffset);

		this.m_debugDraw.DrawPoint(linearOffset, 4.0 / 14, new b2Color(0.9, 0.9, 0.9));

		this.parent.prototype.Step.call(this);
		this.m_drawStringFunc("Keys: (s) pause");
	}
};

TestMotorJoint._extend(Test);