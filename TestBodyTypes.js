function TestBodyTypes()
{
	this.parent.call(this);
}

TestBodyTypes.prototype =
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

		// Define attachment
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 3.0);
			this.m_attachment = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 2.0);
			this.m_attachment.CreateFixture(shape, 2.0);
		}

		// Define platform
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-4.0, 5.0);
			this.m_platform = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 4.0, new b2Vec2(4.0, 0.0), 0.5 * Math.PI);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.friction = 0.6;
			fd.density = 2.0;
			this.m_platform.CreateFixture(fd);

			var rjd = new b2RevoluteJointDef();
			rjd.Initialize(this.m_attachment, this.m_platform, new b2Vec2(0.0, 5.0));
			rjd.maxMotorTorque = 50.0;
			rjd.enableMotor = true;
			this.m_world.CreateJoint(rjd);

			var pjd = new b2PrismaticJointDef();
			pjd.Initialize(ground, this.m_platform, new b2Vec2(0.0, 5.0), new b2Vec2(1.0, 0.0));

			pjd.maxMotorForce = 1000.0;
			pjd.enableMotor = true;
			pjd.lowerTranslation = -10.0;
			pjd.upperTranslation = 10.0;
			pjd.enableLimit = true;

			this.m_world.CreateJoint(pjd);

			this.m_speed = 3.0;
		}

		// Create a payload
		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 8.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.75, 0.75);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.friction = 0.6;
			fd.density = 2.0;

			body.CreateFixture(fd);
		}
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'D'.charCodeAt():
			this.m_platform.SetType(b2Body.b2_dynamicBody);
			break;

		case 'S'.charCodeAt():
			this.m_platform.SetType(b2Body.b2_staticBody);
			break;

		case 'K'.charCodeAt():
			this.m_platform.SetType(b2Body.b2_kinematicBody);
			this.m_platform.SetLinearVelocity(new b2Vec2(-this.m_speed, 0.0));
			this.m_platform.SetAngularVelocity(0.0);
			break;
		}
	},

	Step: function()
	{		// Drive the kinematic body.
		if (this.m_platform.GetType() == b2Body.b2_kinematicBody)
		{
			var p = this.m_platform.GetTransform().p.Clone();
			var v = this.m_platform.GetLinearVelocity().Clone();

			if ((p.x < -10.0 && v.x < 0.0) ||
				(p.x > 10.0 && v.x > 0.0))
			{
				v.x = -v.x;
				this.m_platform.SetLinearVelocity(v);
			}
		}

		this.parent.prototype.Step.call(this);
	}
};

TestBodyTypes._extend(Test);