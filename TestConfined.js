function TestConfined()
{
	this.parent.call(this);
}

TestConfined.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();

			// Floor
			shape.Set(new b2Vec2(-10.0, 0.0), new b2Vec2(10.0, 0.0));
			ground.CreateFixture(shape, 0.0);

			// Left wall
			shape.Set(new b2Vec2(-10.0, 0.0), new b2Vec2(-10.0, 20.0));
			ground.CreateFixture(shape, 0.0);

			// Right wall
			shape.Set(new b2Vec2(10.0, 0.0), new b2Vec2(10.0, 20.0));
			ground.CreateFixture(shape, 0.0);

			// Roof
			shape.Set(new b2Vec2(-10.0, 20.0), new b2Vec2(10.0, 20.0));
			ground.CreateFixture(shape, 0.0);
		}

		var radius = 0.5;
		var shape = new b2CircleShape();
		shape.m_p.SetZero();
		shape.m_radius = radius;

		var fd = new b2FixtureDef();
		fd.shape = shape;
		fd.density = 1.0;
		fd.friction = 0.1;

		var e_rowCount = 0;
		var e_columnCount = 0;

		for (var j = 0; j < e_columnCount; ++j)
		{
			for (var i = 0; i < e_rowCount; ++i)
			{
				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(-10.0 + (2.1 * j + 1.0 + 0.01 * i) * radius, (2.0 * i + 1.0) * radius);
				var body = this.m_world.CreateBody(bd);

				body.CreateFixture(fd);
			}
		}

		this.m_world.SetGravity(new b2Vec2(0.0, 0.0));
	},

	CreateCircle: function()
	{
		var radius = 2.0;
		var shape = new b2CircleShape();
		shape.m_p.SetZero();
		shape.m_radius = radius;

		var fd = new b2FixtureDef();
		fd.shape = shape;
		fd.density = 1.0;
		fd.friction = 0.0;

		var p = new b2Vec2(b2RandomFloat(), 3.0 + b2RandomFloat());
		var bd = new b2BodyDef();
		bd.type = b2Body.b2_dynamicBody;
		bd.position = p;
		//bd.allowSleep = false;
		var body = this.m_world.CreateBody(bd);

		body.CreateFixture(fd);
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'C'.charCodeAt():
			this.CreateCircle();
			break;
		}
	}
};

TestConfined._extend(Test);