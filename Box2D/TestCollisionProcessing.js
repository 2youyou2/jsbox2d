function TestCollisionProcessing()
{
	this.parent.call(this);
}

TestCollisionProcessing.prototype =
{
	Initialize: function()
	{
		// Ground body
		{
			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-50.0, 0.0), new b2Vec2(50.0, 0.0));

			var sd = new b2FixtureDef();
			sd.shape = shape;

			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);
			ground.CreateFixture(sd);
		}

		var xLo = -5.0, xHi = 5.0;
		var yLo = 2.0, yHi = 35.0;

		// Small triangle
		var vertices = [new b2Vec2(-1.0, 0.0), new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 2.0)];

		var polygon = new b2PolygonShape();
		polygon.Set(vertices, 3);

		var triangleShapeDef = new b2FixtureDef();
		triangleShapeDef.shape = polygon;
		triangleShapeDef.density = 1.0;

		var triangleBodyDef = new b2BodyDef();
		triangleBodyDef.type = b2Body.b2_dynamicBody;
		triangleBodyDef.position.Set(b2RandomFloat(xLo, xHi), b2RandomFloat(yLo, yHi));

		var body1 = this.m_world.CreateBody(triangleBodyDef);
		body1.CreateFixture(triangleShapeDef);

		// Large triangle (recycle definitions)
		vertices[0].Multiply(2.0);
		vertices[1].Multiply(2.0);
		vertices[2].Multiply(2.0);
		polygon.Set(vertices, 3);

		triangleBodyDef.position.Set(b2RandomFloat(xLo, xHi), b2RandomFloat(yLo, yHi));

		var body2 = this.m_world.CreateBody(triangleBodyDef);
		body2.CreateFixture(triangleShapeDef);

		// Small box
		polygon.SetAsBox(1.0, 0.5);

		var boxShapeDef = new b2FixtureDef();
		boxShapeDef.shape = polygon;
		boxShapeDef.density = 1.0;

		var boxBodyDef = new b2BodyDef();
		boxBodyDef.type = b2Body.b2_dynamicBody;
		boxBodyDef.position.Set(b2RandomFloat(xLo, xHi), b2RandomFloat(yLo, yHi));

		var body3 = this.m_world.CreateBody(boxBodyDef);
		body3.CreateFixture(boxShapeDef);

		// Large box (recycle definitions)
		polygon.SetAsBox(2.0, 1.0);
		boxBodyDef.position.Set(b2RandomFloat(xLo, xHi), b2RandomFloat(yLo, yHi));

		var body4 = this.m_world.CreateBody(boxBodyDef);
		body4.CreateFixture(boxShapeDef);

		// Small circle
		var circle = new b2CircleShape();
		circle.m_radius = 1.0;

		var circleShapeDef = new b2FixtureDef();
		circleShapeDef.shape = circle;
		circleShapeDef.density = 1.0;

		var circleBodyDef = new b2BodyDef();
		circleBodyDef.type = b2Body.b2_dynamicBody;
		circleBodyDef.position.Set(b2RandomFloat(xLo, xHi), b2RandomFloat(yLo, yHi));

		var body5 = this.m_world.CreateBody(circleBodyDef);
		body5.CreateFixture(circleShapeDef);

		// Large circle
		circle.m_radius *= 2.0;
		circleBodyDef.position.Set(b2RandomFloat(xLo, xHi), b2RandomFloat(yLo, yHi));

		var body6 = this.m_world.CreateBody(circleBodyDef);
		body6.CreateFixture(circleShapeDef);
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		// We are going to destroy some bodies according to contact
		// points. We must buffer the bodies that should be destroyed
		// because they may belong to multiple contact points.
		var nuke = [];
		var nukeCount = 0;

		// Traverse the contact results. Destroy bodies that
		// are touching heavier bodies.
		for (var i = 0; i < this.m_pointCount; ++i)
		{
			var point = this.m_points[i];

			var body1 = point.fixtureA.GetBody();
			var body2 = point.fixtureB.GetBody();
			var mass1 = body1.GetMass();
			var mass2 = body2.GetMass();

			if (mass1 > 0.0 && mass2 > 0.0)
			{
				if (mass2 > mass1)
				{
					nuke[nukeCount++] = body1;
				}
				else
				{
					nuke[nukeCount++] = body2;
				}
			}
		}

		// Sort the nuke array to group duplicates.
		nuke.sort();

		// Destroy the bodies, skipping duplicates.
		var i = 0;
		while (i < nukeCount)
		{
			var b = nuke[i++];
			while (i < nukeCount && nuke[i] === b)
			{
				++i;
			}

			if (b != this.m_bomb)
			{
				if (!b.m_destroyed)
					this.m_world.DestroyBody(b);
			}
		}
	}

};

TestCollisionProcessing._extend(Test);