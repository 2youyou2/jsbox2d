function TestTheoJansenWalker()
{
	this.parent.call(this);
}

TestTheoJansenWalker.prototype =
{
	Initialize: function()
	{
		this.m_offset = new b2Vec2(0.0, 8.0);
		this.m_motorSpeed = 2.0;
		this.m_motorOn = true;
		var pivot = new b2Vec2(0.0, 0.8);

		// Ground
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-50.0, 0.0), new b2Vec2(50.0, 0.0));
			ground.CreateFixture(shape, 0.0);

			shape.Set(new b2Vec2(-50.0, 0.0), new b2Vec2(-50.0, 10.0));
			ground.CreateFixture(shape, 0.0);

			shape.Set(new b2Vec2(50.0, 0.0), new b2Vec2(50.0, 10.0));
			ground.CreateFixture(shape, 0.0);
		}

		// Balls
		for (var i = 0; i < 40; ++i)
		{
			var shape = new b2CircleShape();
			shape.m_radius = 0.25;

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-40.0 + 2.0 * i, 0.5);

			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(shape, 1.0);
		}

		// Chassis
		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(2.5, 1.0);

			var sd = new b2FixtureDef();
			sd.density = 1.0;
			sd.shape = shape;
			sd.filter.groupIndex = -1;
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position = b2Vec2.Add(pivot, this.m_offset);
			this.m_chassis = this.m_world.CreateBody(bd);
			this.m_chassis.CreateFixture(sd);
		}

		{
			var shape = new b2CircleShape();
			shape.m_radius = 1.6;

			var sd = new b2FixtureDef();
			sd.density = 1.0;
			sd.shape = shape;
			sd.filter.groupIndex = -1;
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position = b2Vec2.Add(pivot, this.m_offset);
			this.m_wheel = this.m_world.CreateBody(bd);
			this.m_wheel.CreateFixture(sd);
		}

		{
			var jd = new b2RevoluteJointDef();
			jd.Initialize(this.m_wheel, this.m_chassis, b2Vec2.Add(pivot, this.m_offset));
			jd.collideConnected = false;
			jd.motorSpeed = this.m_motorSpeed;
			jd.maxMotorTorque = 400.0;
			jd.enableMotor = this.m_motorOn;
			this.m_motorJoint = this.m_world.CreateJoint(jd);
		}

		var wheelAnchor = b2Vec2.Add(pivot, new b2Vec2(0.0, -0.8));

		this.CreateLeg(-1.0, wheelAnchor);
		this.CreateLeg(1.0, wheelAnchor);

		this.m_wheel.SetTransform(this.m_wheel.GetPosition(), 120.0 * Math.PI / 180.0);
		this.CreateLeg(-1.0, wheelAnchor);
		this.CreateLeg(1.0, wheelAnchor);

		this.m_wheel.SetTransform(this.m_wheel.GetPosition(), -120.0 * Math.PI / 180.0);
		this.CreateLeg(-1.0, wheelAnchor);
		this.CreateLeg(1.0, wheelAnchor);
	},

	CreateLeg: function(s, wheelAnchor)
	{
		var p1 = new b2Vec2(5.4 * s, -6.1);
		var p2 = new b2Vec2(7.2 * s, -1.2);
		var p3 = new b2Vec2(4.3 * s, -1.9);
		var p4 = new b2Vec2(3.1 * s, 0.8);
		var p5 = new b2Vec2(6.0 * s, 1.5);
		var p6 = new b2Vec2(2.5 * s, 3.7);

		var fd1 = new b2FixtureDef(), fd2 = new b2FixtureDef();
		fd1.filter.groupIndex = -1;
		fd2.filter.groupIndex = -1;
		fd1.density = 1.0;
		fd2.density = 1.0;

		var poly1 = new b2PolygonShape(), poly2 = new b2PolygonShape();

		if (s > 0.0)
		{
			var vertices = [p1, p2, p3];
			poly1.Set(vertices, 3);

			vertices[0] = new b2Vec2(0, 0);
			vertices[1] = b2Vec2.Subtract(p5, p4);
			vertices[2] = b2Vec2.Subtract(p6, p4);
			poly2.Set(vertices, 3);
		}
		else
		{
			var vertices = [p1, p3, p2];
			poly1.Set(vertices, 3);

			vertices[0] = new b2Vec2(0, 0);
			vertices[1] = b2Vec2.Subtract(p6, p4);
			vertices[2] = b2Vec2.Subtract(p5, p4);
			poly2.Set(vertices, 3);
		}

		fd1.shape = poly1;
		fd2.shape = poly2;

		var bd1 = new b2BodyDef(), bd2 = new b2BodyDef();
		bd1.type = b2Body.b2_dynamicBody;
		bd2.type = b2Body.b2_dynamicBody;
		bd1.position = this.m_offset;
		bd2.position = b2Vec2.Add(p4, this.m_offset);

		bd1.angularDamping = 10.0;
		bd2.angularDamping = 10.0;

		var body1 = this.m_world.CreateBody(bd1);
		var body2 = this.m_world.CreateBody(bd2);

		body1.CreateFixture(fd1);
		body2.CreateFixture(fd2);

		var djd = new b2DistanceJointDef();

		// Using a soft distance constraint can reduce some jitter.
		// It also makes the structure seem a bit more fluid by
		// acting like a suspension system.
		djd.dampingRatio = 0.5;
		djd.frequencyHz = 10.0;

		djd.Initialize(body1, body2, b2Vec2.Add(p2, this.m_offset), b2Vec2.Add(p5, this.m_offset));
		this.m_world.CreateJoint(djd);

		djd.Initialize(body1, body2, b2Vec2.Add(p3, this.m_offset), b2Vec2.Add(p4, this.m_offset));
		this.m_world.CreateJoint(djd);

		djd.Initialize(body1, this.m_wheel, b2Vec2.Add(p3, this.m_offset), b2Vec2.Add(wheelAnchor, this.m_offset));
		this.m_world.CreateJoint(djd);

		djd.Initialize(body2, this.m_wheel, b2Vec2.Add(p6, this.m_offset), b2Vec2.Add(wheelAnchor, this.m_offset));
		this.m_world.CreateJoint(djd);

		var rjd = new b2RevoluteJointDef();

		rjd.Initialize(body2, this.m_chassis, b2Vec2.Add(p4, this.m_offset));
		this.m_world.CreateJoint(rjd);
	},

	Keyboard: function(key)
	{
		switch (key)
		{
		case 'A'.charCodeAt():
			this.m_motorJoint.SetMotorSpeed(-this.m_motorSpeed);
			break;

		case 'S'.charCodeAt():
			this.m_motorJoint.SetMotorSpeed(0.0);
			break;

		case 'D'.charCodeAt():
			this.m_motorJoint.SetMotorSpeed(this.m_motorSpeed);
			break;

		case 'M'.charCodeAt():
			this.m_motorJoint.EnableMotor(!this.m_motorJoint.IsMotorEnabled());
			break;
		}
	}
};

TestTheoJansenWalker._extend(Test);