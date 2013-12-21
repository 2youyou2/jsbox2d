function TestBulldozer()
{
	this.parent.call(this);
}

TestBulldozer.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var fd = new b2FixtureDef();
			fd.density = 0;
			fd.restitution = 0.2;

			fd.shape = new b2ChainShape();

			var verts = [ new b2Vec2(-20, -20), new b2Vec2(-20, 20), new b2Vec2(20, 20), new b2Vec2(20, -20) ];

			fd.shape.CreateLoop(verts, verts.length);
			ground.CreateFixture(fd);
			this.ground = ground;
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.gravityScale = 0;

			var fd = new b2FixtureDef();
			fd.density = 0.2;
			fd.restitution = 0.1;

			fd.shape = new b2PolygonShape();
			fd.shape.SetAsBox(0.8, 0.8);

			for (var i = 0; i < 20; ++i)
			{
				bd.position.Set(b2RandomFloat(-18, 18), b2RandomFloat(-18, 18));
				var b = this.m_world.CreateBody(bd);
				b.CreateFixture(fd);

				var gravity = 10.0;
				var I = b.GetInertia();
				var mass = b.GetMass();

				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				var radius = Math.sqrt(2.0 * I / mass);

				var jd = new b2FrictionJointDef();
				jd.localAnchorA.SetZero();
				jd.localAnchorB.SetZero();
				jd.bodyA = ground;
				jd.bodyB = b;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = mass * radius * gravity;

				this.m_world.CreateJoint(jd);
			}

			fd.density = 0.8;

			fd.shape.SetAsBox(1.2, 1.6);
			bd.position.SetZero();
			var b = this.m_world.CreateBody(bd);
			b.CreateFixture(fd);

			this.bulldozer = b;

			var gravity = 4.0;
			var I = b.GetInertia();
			var mass = b.GetMass();

			// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
			var radius = Math.sqrt(2.0 * I / mass);

			var jd = new b2FrictionJointDef();
			jd.localAnchorA.SetZero();
			jd.localAnchorB.SetZero();
			jd.bodyA = ground;
			jd.bodyB = b;
			jd.collideConnected = true;
			jd.maxForce = mass * gravity;
			jd.maxTorque = mass * radius * gravity;

			this.m_world.CreateJoint(jd);
		}

		this.m_center.y -= 18;
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
			case 'A'.charCodeAt():
				if (this.bulldozer.GetAngularVelocity() < 5)
					this.bulldozer.ApplyAngularImpulse(2, true);
				break;
			case 'D'.charCodeAt():
				if (this.bulldozer.GetAngularVelocity() > -5)
					this.bulldozer.ApplyAngularImpulse(-2, true);
				break;
			case 'W'.charCodeAt():
				var f = this.bulldozer.GetWorldVector(new b2Vec2(0.0, -100.0));
				var p = this.bulldozer.GetWorldPoint(new b2Vec2(0, 2.0));
				this.bulldozer.ApplyForce(f, p, true);
				break;
			case 'S'.charCodeAt():
				var f = this.bulldozer.GetWorldVector(new b2Vec2(0.0, 100.0));
				var p = this.bulldozer.GetWorldPoint(new b2Vec2(0, 2.0));
				this.bulldozer.ApplyForce(f, p, true);
				break;
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		this.m_drawStringFunc("Use W-S-A-D to drive");
	}
};

TestBulldozer._extend(Test);