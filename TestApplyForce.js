function TestApplyForce()
{
	this.parent.call(this);
}

TestApplyForce.prototype =
{
	Initialize: function()
	{
		this.m_world.SetGravity(new b2Vec2(0.0, 0.0));

		var k_restitution = 0.4;

		var ground;
		{
			var bd = new b2BodyDef();
			bd.position.Set(0.0, 20.0);
			ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();

			var sd = new b2FixtureDef();
			sd.shape = shape;
			sd.density = 0.0;
			sd.restitution = k_restitution;

			// Left vertical
			shape.Set(new b2Vec2(-20.0, -20.0), new b2Vec2(-20.0, 20.0));
			ground.CreateFixture(sd);

			// Right vertical
			shape.Set(new b2Vec2(20.0, -20.0), new b2Vec2(20.0, 20.0));
			ground.CreateFixture(sd);

			// Top horizontal
			shape.Set(new b2Vec2(-20.0, 20.0), new b2Vec2(20.0, 20.0));
			ground.CreateFixture(sd);

			// Bottom horizontal
			shape.Set(new b2Vec2(-20.0, -20.0), new b2Vec2(20.0, -20.0));
			ground.CreateFixture(sd);
		}

		{
			var xf1 = new b2Transform();
			xf1.q.Set(0.3524 * Math.PI);
			xf1.p = xf1.q.GetXAxis();

			var vertices = [];
			vertices[0] = b2Mul_t_v2(xf1, new b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf1, new b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf1, new b2Vec2(0.0, 0.5));

			var poly1 = new b2PolygonShape();
			poly1.Set(vertices, 3);

			var sd1 = new b2FixtureDef();
			sd1.shape = poly1;
			sd1.density = 4.0;

			var xf2 = new b2Transform();
			xf2.q.Set(-0.3524 * Math.PI);
			xf2.p = xf2.q.GetXAxis().Negate();

			vertices[0] = b2Mul_t_v2(xf2, new b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf2, new b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf2, new b2Vec2(0.0, 0.5));

			var poly2 = new b2PolygonShape();
			poly2.Set(vertices, 3);

			var sd2 = new b2FixtureDef();
			sd2.shape = poly2;
			sd2.density = 2.0;

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.angularDamping = 2.0;
			bd.linearDamping = 0.5;

			bd.position.Set(0.0, 2.0);
			bd.angle = Math.PI;
			bd.allowSleep = false;
			this.m_body = this.m_world.CreateBody(bd);
			this.m_body.CreateFixture(sd1);
			this.m_body.CreateFixture(sd2);
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.5);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 1.0;
			fd.friction = 0.3;

			for (var i = 0; i < 10; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;

				bd.position.Set(0.0, 5.0 + 1.54 * i);
				var body = this.m_world.CreateBody(bd);

				body.CreateFixture(fd);

				var gravity = 10.0;
				var I = body.GetInertia();
				var mass = body.GetMass();

				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				var radius = Math.sqrt(2.0 * I / mass);

				var jd = new b2FrictionJointDef();
				jd.localAnchorA.SetZero();
				jd.localAnchorB.SetZero();
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = mass * radius * gravity;

				this.m_world.CreateJoint(jd);
			}
		}
	},

	Keyboard: function(key)
	{
		switch (key)
		{
		case 'W'.charCodeAt():
			{
				var f = this.m_body.GetWorldVector(new b2Vec2(0.0, -200.0));
				var p = this.m_body.GetWorldPoint(new b2Vec2(0.10, 2.0));
				this.m_body.ApplyForce(f, p, true);
			}
			break;

		case 'A'.charCodeAt():
			{
				this.m_body.ApplyTorque(50.0, true);
			}
			break;

		case 'D'.charCodeAt():
			{
				this.m_body.ApplyTorque(-50.0, true);
			}
			break;
		}
	}
};

TestApplyForce._extend(Test);