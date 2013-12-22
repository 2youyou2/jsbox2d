function TestSnake()
{
	this.parent.call(this);
}

TestSnake.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);
			this.m_world.SetGravity(new b2Vec2(0, 0));
		}

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.125, 0.6);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20;
			fd.friction = 0.2;

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.linearDamping = bd.angularDamping = 2;
			bd.position.Set(0, 0);
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(fd);

			this.m_snakeHead = body;
			this.m_snakeTail = body;
			this.m_snakeParts = 1;
		}
	},

	CreateChain: function()
	{
		var oldTail = this.m_snakeTail;

		var shape = new b2PolygonShape();
		shape.SetAsBox(0.125, 0.6);

		var fd = new b2FixtureDef();
		fd.shape = shape;
		fd.density = 0.01;
		fd.friction = 0.2;

		var bd = new b2BodyDef();
		bd.type = b2Body.b2_dynamicBody;
		bd.linearDamping = bd.angularDamping = 2;
		bd.angle = oldTail.GetAngle();
		bd.position.Assign(oldTail.GetWorldPoint(new b2Vec2(0, 1.2)));
		var body = this.m_world.CreateBody(bd);
		body.CreateFixture(fd);

		var jd = new b2RevoluteJointDef();
		jd.collideConnected = false;

		var anchor = oldTail.GetWorldPoint(new b2Vec2(0, 0.6));
		jd.Initialize(oldTail, body, anchor);
		this.m_world.CreateJoint(jd);

		this.m_snakeTail = body;
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
			case 'A'.charCodeAt():
				if (this.m_snakeHead.GetAngularVelocity() < 2 * this.m_snakeParts)
					this.m_snakeHead.SetAngularVelocity(this.m_snakeHead.GetAngularVelocity() + (1 * this.m_snakeParts), true);
				break;
			case 'D'.charCodeAt():
				if (this.m_snakeHead.GetAngularVelocity() > -2 * this.m_snakeParts)
					this.m_snakeHead.SetAngularVelocity(this.m_snakeHead.GetAngularVelocity() + (-1 * this.m_snakeParts), true);
				break;
			case 'W'.charCodeAt():
				var f = this.m_snakeHead.GetWorldVector(new b2Vec2(0.0, -200.0 * this.m_snakeParts));
				var p = this.m_snakeHead.GetWorldPoint(new b2Vec2(0, 20.0));
				this.m_snakeHead.ApplyForce(f, p, true);
				break;
			case 'S'.charCodeAt():
				var f = this.m_snakeHead.GetWorldVector(new b2Vec2(0.0, 200.0 * this.m_snakeParts));
				var p = this.m_snakeHead.GetWorldPoint(new b2Vec2(0, 20.0));
				this.m_snakeHead.ApplyForce(f, p, true);
				break;

			case 'C'.charCodeAt():
				this.CreateChain();
				break;
		}
	}
};

TestSnake._extend(Test);