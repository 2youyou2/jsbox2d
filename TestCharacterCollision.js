function TestCharacterCollision()
{
	this.parent.call(this);
}

TestCharacterCollision.prototype =
{
	Initialize: function()
	{
		// Ground body
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-20.0, 0.0), new b2Vec2(20.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		// Collinear edges with no adjacency information.
		// This shows the problematic case where a box shape can hit
		// an internal vertex.
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-8.0, 1.0), new b2Vec2(-6.0, 1.0));
			ground.CreateFixture(shape, 0.0);
			shape.Set(new b2Vec2(-6.0, 1.0), new b2Vec2(-4.0, 1.0));
			ground.CreateFixture(shape, 0.0);
			shape.Set(new b2Vec2(-4.0, 1.0), new b2Vec2(-2.0, 1.0));
			ground.CreateFixture(shape, 0.0);
		}

		// Chain shape
		{
			var bd = new b2BodyDef();
			bd.angle = 0.25 * b2_pi;
			var ground = this.m_world.CreateBody(bd);

			var vs = [new b2Vec2(5.0, 7.0), new b2Vec2(6.0, 8.0), new b2Vec2(7.0, 8.0), new b2Vec2(8.0, 7.0)];
			var shape = new b2ChainShape();
			shape.CreateChain(vs, 4);
			ground.CreateFixture(shape, 0.0);
		}

		// Square tiles. This shows that adjacency shapes may
		// have non-smooth collision. There is no solution
		// to this problem.
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(1.0, 1.0, new b2Vec2(4.0, 3.0), 0.0);
			ground.CreateFixture(shape, 0.0);
			shape.SetAsBox(1.0, 1.0, new b2Vec2(6.0, 3.0), 0.0);
			ground.CreateFixture(shape, 0.0);
			shape.SetAsBox(1.0, 1.0, new b2Vec2(8.0, 3.0), 0.0);
			ground.CreateFixture(shape, 0.0);
		}

		// Square made from an edge loop. Collision should be smooth.
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var vs = [new b2Vec2(-1.0, 3.0), new b2Vec2(1.0, 3.0), new b2Vec2(1.0, 5.0), new b2Vec2(-1.0, 5.0)];
			var shape = new b2ChainShape();
			shape.CreateLoop(vs, 4);
			ground.CreateFixture(shape, 0.0);
		}

		// Edge loop. Collision should be smooth.
		{
			var bd = new b2BodyDef();
			bd.position.Set(-10.0, 4.0);
			var ground = this.m_world.CreateBody(bd);

			vs = [new b2Vec2(0.0, 0.0), new b2Vec2(6.0, 0.0), new b2Vec2(6.0, 2.0), new b2Vec2(4.0, 1.0), new b2Vec2(2.0, 2.0), new b2Vec2(0.0, 2.0), new b2Vec2(-2.0, 2.0), new b2Vec2(-4.0, 3.0), new b2Vec2(-6.0, 2.0), new b2Vec2(-6.0, 0.0)];
			var shape = new b2ChainShape();
			shape.CreateLoop(vs, 10);
			ground.CreateFixture(shape, 0.0);
		}

		// Square character 1
		{
			var bd = new b2BodyDef();
			bd.position.Set(-3.0, 8.0);
			bd.type = b2Body.b2_dynamicBody;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.5);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;
			body.CreateFixture(fd);
		}

		// Square character 2
		{
			var bd = new b2BodyDef();
			bd.position.Set(-5.0, 5.0);
			bd.type = b2Body.b2_dynamicBody;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.25, 0.25);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;
			body.CreateFixture(fd);
		}

		// Hexagon character
		{
			var bd = new b2BodyDef();
			bd.position.Set(-5.0, 8.0);
			bd.type = b2Body.b2_dynamicBody;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			var body = this.m_world.CreateBody(bd);

			var angle = 0.0;
			var delta = b2_pi / 3.0;
			var vertices = [];
			for (var i = 0; i < 6; ++i)
			{
				vertices[i] = new b2Vec2(0.5 * cosf(angle), 0.5 * sinf(angle));
				angle += delta;
			}

			var shape = new b2PolygonShape();
			shape.Set(vertices, 6);

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;
			body.CreateFixture(fd);
		}

		// Circle character
		{
			var bd = new b2BodyDef();
			bd.position.Set(3.0, 5.0);
			bd.type = b2Body.b2_dynamicBody;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			var body = this.m_world.CreateBody(bd);

			var shape = new b2CircleShape();
			shape.m_radius = 0.5;

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;
			body.CreateFixture(fd);
		}

		// Circle character
		{
			var bd = new b2BodyDef();
			bd.position.Set(-7.0, 6.0);
			bd.type = b2Body.b2_dynamicBody;
			bd.allowSleep = false;

			this.m_character = this.m_world.CreateBody(bd);

			var shape = new b2CircleShape();
			shape.m_radius = 0.25;

			var fd = new b2FixtureDef();
			fd.shape = shape;
			fd.density = 20.0;
			fd.friction = 1.0;
			this.m_character.CreateFixture(fd);
		}
	}
};

TestCharacterCollision._extend(Test);