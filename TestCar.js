function TestCar()
{
	this.parent.call(this);
}

TestCar.prototype =
{
	Initialize: function()
	{
		this.m_chz = 4.0;
		this.m_zeta = 0.7;
		this.m_speed = 50.0;

		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 0.0;
			fd.friction = 0.6;

			shape.Set(new b2Vec2(-20.0, 0.0), new b2Vec2(20.0, 0.0));
			ground.CreateFixture(fd);

			var hs = [0.25, 1.0, 4.0, 0.0, 0.0, -1.0, -2.0, -2.0, -1.25, 0.0];

			var x = 20.0, y1 = 0.0, dx = 5.0;

			for (var i = 0; i < 10; ++i)
			{
				var y2 = hs[i];
				shape.Set(new b2Vec2(x, y1), new b2Vec2(x + dx, y2));
				ground.CreateFixture(fd);
				y1 = y2;
				x += dx;
			}

			for (var i = 0; i < 10; ++i)
			{
				var y2 = hs[i];
				shape.Set(new b2Vec2(x, y1), new b2Vec2(x + dx, y2));
				ground.CreateFixture(fd);
				y1 = y2;
				x += dx;
			}

			shape.Set(new b2Vec2(x, 0.0), new b2Vec2(x + 40.0, 0.0));
			ground.CreateFixture(fd);

			x += 80.0;
			shape.Set(new b2Vec2(x, 0.0), new b2Vec2(x + 40.0, 0.0));
			ground.CreateFixture(fd);

			x += 40.0;
			shape.Set(new b2Vec2(x, 0.0), new b2Vec2(x + 10.0, 5.0));
			ground.CreateFixture(fd);

			x += 20.0;
			shape.Set(new b2Vec2(x, 0.0), new b2Vec2(x + 40.0, 0.0));
			ground.CreateFixture(fd);

			x += 40.0;
			shape.Set(new b2Vec2(x, 0.0), new b2Vec2(x, 20.0));
			ground.CreateFixture(fd);
		}

		// Teeter
		{
			var bd = new b2BodyDef();
			bd.position.Set(140.0, 1.0);
			bd.type = b2Body.b2_dynamicBody;
			var body = this.m_world.CreateBody(bd);

			var box = new b2PolygonShape();
			box.SetAsBox(10.0, 0.25);
			body.CreateFixture(box, 1.0);

			var jd = new b2RevoluteJointDef();
			jd.Initialize(ground, body, body.GetPosition());
			jd.lowerAngle = -8.0 * Math.PI / 180.0;
			jd.upperAngle = 8.0 * Math.PI / 180.0;
			jd.enableLimit = true;
			this.m_world.CreateJoint(jd);

			body.ApplyAngularImpulse(100.0, true);
		}

		// Bridge
		{
			var N = 20;
			var shape = new b2PolygonShape();
			shape.SetAsBox(1.0, 0.125);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 1.0;
			fd.friction = 0.6;

			var jd = new b2RevoluteJointDef();

			var prevBody = ground;
			for (var i = 0; i < N; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(161.0 + 2.0 * i, -0.125);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(fd);

				var anchor = new b2Vec2(160.0 + 2.0 * i, -0.125);
				jd.Initialize(prevBody, body, anchor);
				this.m_world.CreateJoint(jd);

				prevBody = body;
			}

			var anchor = new b2Vec2(160.0 + 2.0 * N, -0.125);
			jd.Initialize(prevBody, ground, anchor);
			this.m_world.CreateJoint(jd);
		}

		// Boxes
		{
			var box = new b2PolygonShape();
			box.SetAsBox(0.5, 0.5);

			var body = null;
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;

			bd.position.Set(230.0, 0.5);
			body = this.m_world.CreateBody(bd);
			body.CreateFixture(box, 0.5);

			bd.position.Set(230.0, 1.5);
			body = this.m_world.CreateBody(bd);
			body.CreateFixture(box, 0.5);

			bd.position.Set(230.0, 2.5);
			body = this.m_world.CreateBody(bd);
			body.CreateFixture(box, 0.5);

			bd.position.Set(230.0, 3.5);
			body = this.m_world.CreateBody(bd);
			body.CreateFixture(box, 0.5);

			bd.position.Set(230.0, 4.5);
			body = this.m_world.CreateBody(bd);
			body.CreateFixture(box, 0.5);
		}

		// Car
		{
			var chassis = new b2PolygonShape();
			var vertices = [];
			vertices[0] = new b2Vec2(-1.5, -0.5);
			vertices[1] = new b2Vec2(1.5, -0.5);
			vertices[2] = new b2Vec2(1.5, 0.0);
			vertices[3] = new b2Vec2(0.0, 0.9);
			vertices[4] = new b2Vec2(-1.15, 0.9);
			vertices[5] = new b2Vec2(-1.5, 0.2);
			chassis.Set(vertices, 6);

			var circle = new b2CircleShape();
			circle.m_radius = 0.4;

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0.0, 1.0);
			this.m_car = this.m_world.CreateBody(bd);
			this.m_car.CreateFixture(chassis, 1.0);

			var fd = new b2FixtureDef();
			fd.shape = circle;
			fd.density = 1.0;
			fd.friction = 0.9;

			bd.position.Set(-1.0, 0.35);
			this.m_wheel1 = this.m_world.CreateBody(bd);
			this.m_wheel1.CreateFixture(fd);

			bd.position.Set(1.0, 0.4);
			this.m_wheel2 = this.m_world.CreateBody(bd);
			this.m_wheel2.CreateFixture(fd);

			var jd = new b2WheelJointDef();
			var axis = new b2Vec2(0.0, 1.0);

			jd.Initialize(this.m_car, this.m_wheel1, this.m_wheel1.GetPosition(), axis);
			jd.motorSpeed = 0.0;
			jd.maxMotorTorque = 20.0;
			jd.enableMotor = true;
			jd.frequencyHz = this.m_chz;
			jd.dampingRatio = this.m_zeta;
			this.m_spring1 = this.m_world.CreateJoint(jd);

			jd.Initialize(this.m_car, this.m_wheel2, this.m_wheel2.GetPosition(), axis);
			jd.motorSpeed = 0.0;
			jd.maxMotorTorque = 10.0;
			jd.enableMotor = false;
			jd.frequencyHz = this.m_chz;
			jd.dampingRatio = this.m_zeta;
			this.m_spring2 = this.m_world.CreateJoint(jd);
		}
	},

	Keyboard: function(key)
	{
		switch (key)
		{
		case 'A'.charCodeAt():
			this.m_spring1.SetMotorSpeed(this.m_speed);
			break;

		case 'S'.charCodeAt():
			this.m_spring1.SetMotorSpeed(0.0);
			break;

		case 'D'.charCodeAt():
			this.m_spring1.SetMotorSpeed(-this.m_speed);
			break;

		case 'Q'.charCodeAt():
			this.m_chz = Math.max(0.0, this.m_chz - 1.0);
			this.m_spring1.SetSpringFrequencyHz(this.m_chz);
			this.m_spring2.SetSpringFrequencyHz(this.m_chz);
			break;

		case 'E'.charCodeAt():
			this.m_chz += 1.0;
			this.m_spring1.SetSpringFrequencyHz(this.m_chz);
			this.m_spring2.SetSpringFrequencyHz(this.m_chz);
			break;
		}
	},

	Step: function()
	{
		this.m_drawStringFunc("Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		this.m_drawStringFunc("frequency = " + this.m_chz + " hz, damping ratio = " + this.m_zeta);

		this.m_center.x = this.m_car.GetPosition().x;
		this.parent.prototype.Step.call(this);
	}
};

TestCar._extend(Test);