function TestVerticalStack()
{
	this.parent.call(this);
}

TestVerticalStack.prototype =
{
	Initialize: function()
	{
		var e_columnCount = 5;
		var e_rowCount = 16;

		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);

			shape.Set(new b2Vec2(20.0, 0.0), new b2Vec2(20.0, 20.0));
			ground.CreateFixture(shape, 0.0);
		}

		var xs = [0.0, -10.0, -5.0, 5.0, 10.0];

		this.m_indices = [];
		this.m_bodies = [];

		for (var j = 0; j < e_columnCount; ++j)
		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.5);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 1.0;
			fd.friction = 0.3;

			for (var i = 0; i < e_rowCount; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;

				var n = j * e_rowCount + i;
				this.m_indices[n] = n;
				bd.userData = this.m_indices + n;

				var x = 0.0;
				//float32 x = RandomFloat(-0.02f, 0.02f);
				//float32 x = i % 2 == 0 ? -0.025f : 0.025f;
				bd.position.Set(xs[j] + x, 0.752 + 1.54 * i);
				var body = this.m_world.CreateBody(bd);

				this.m_bodies[n] = body;

				body.CreateFixture(fd);
			}
		}

		this.m_bullet = null;
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 188:
			if (this.m_bullet != null)
			{
				this.m_world.DestroyBody(this.m_bullet);
				this.m_bullet = null;
			}

			{
				var shape = new b2CircleShape();
				shape.m_radius = 0.25;

				var fd = new b2FixtureDef();
				fd.shape = shape;
				fd.density = 20.0;
				fd.restitution = 0.05;

				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.bullet = true;
				bd.position.Set(-31.0, 5.0);

				this.m_bullet = this.m_world.CreateBody(bd);
				this.m_bullet.CreateFixture(fd);

				this.m_bullet.SetLinearVelocity(new b2Vec2(400.0, 0.0));
			}
			break;
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		this.m_drawStringFunc("Press: (,) to launch a bullet.");
	}
};

TestVerticalStack._extend(Test);