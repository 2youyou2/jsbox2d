function TestRopeJoint()
{
	this.parent.call(this);
}

TestRopeJoint.prototype =
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
			shape.SetAsBox(0.5, 0.125);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;
			fd.friction = 0.2;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;

			var jd = new b2RevoluteJointDef();
			jd.collideConnected = false;

			var N = 10;
			var y = 15.0;
			this.m_ropeDef = new b2RopeJointDef();
			this.m_ropeDef.localAnchorA.Set(0.0, y);

			var prevBody = ground;
			for (var i = 0; i < N; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(0.5 + 1.0 * i, y);
				if (i == N - 1)
				{
					shape.SetAsBox(1.5, 1.5);
					fd.density = 100.0;
					fd.filter.categoryBits = 0x0002;
					bd.position.Set(1.0 * i, y);
					bd.angularDamping = 0.4;
				}

				var body = this.m_world.CreateBody(bd);

				body.CreateFixture(fd);

				var anchor = new b2Vec2(i, y);
				jd.Initialize(prevBody, body, anchor);
				this.m_world.CreateJoint(jd);

				prevBody = body;
			}

			this.m_ropeDef.localAnchorB.SetZero();

			var extraLength = 0.01;
			this.m_ropeDef.maxLength = N - 1.0 + extraLength;
			this.m_ropeDef.bodyB = prevBody;
		}

		{
			this.m_ropeDef.bodyA = ground;
			this.m_rope = this.m_world.CreateJoint(this.m_ropeDef);
		}
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'J'.charCodeAt():
			if (this.m_rope)
			{
				this.m_world.DestroyJoint(this.m_rope);
				this.m_rope = null;
			}
			else
			{
				this.m_rope = this.m_world.CreateJoint(this.m_ropeDef);
			}
			break;
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		this.m_drawStringFunc("Press (j) to toggle the rope joint.");
		if (this.m_rope)
		{
			this.m_drawStringFunc("Rope ON");
		}
		else
		{
			this.m_drawStringFunc("Rope OFF");
		}
	}
};

TestRopeJoint._extend(Test);