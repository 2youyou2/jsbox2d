function TestCollisionFiltering()
{
	this.parent.call(this);
}

TestCollisionFiltering.prototype =
{
	Initialize: function()
	{
		//// This is a test of collision filtering.
		// There is a triangle, a box, and a circle.
		// There are 6 shapes. 3 large and 3 small.
		// The 3 small ones always collide.
		// The 3 large ones never collide.
		// The boxes don't collide with triangles (except if both are small).
		var	k_smallGroup = 1;
		var k_largeGroup = -1;

		var k_defaultCategory = 0x0001;
		var k_triangleCategory = 0x0002;
		var k_boxCategory = 0x0004;
		var k_circleCategory = 0x0008;

		var k_triangleMask = 0xFFFF;
		var k_boxMask = 0xFFFF ^ k_triangleCategory;
		var k_circleMask = 0xFFFF;

		// Ground body
		{
			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));

			var sd = new b2FixtureDef();
			sd.shape = shape;
			sd.friction = 0.3;

			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);
			ground.CreateFixture(sd);
		}

		// Small triangle
		var vertices = [new b2Vec2(-1.0, 0.0), new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 2.0)];
		var polygon = new b2PolygonShape();
		polygon.Set(vertices, 3);

		var triangleShapeDef = new b2FixtureDef();
		triangleShapeDef.shape = polygon;
		triangleShapeDef.density = 1.0;

		triangleShapeDef.filter.groupIndex = k_smallGroup;
		triangleShapeDef.filter.categoryBits = k_triangleCategory;
		triangleShapeDef.filter.maskBits = k_triangleMask;

		var triangleBodyDef = new b2BodyDef();
		triangleBodyDef.type = b2Body.b2_dynamicBody;
		triangleBodyDef.position.Set(-5.0, 2.0);

		var body1 = this.m_world.CreateBody(triangleBodyDef);
		body1.CreateFixture(triangleShapeDef);

		// Large triangle (recycle definitions)
		vertices[0].Multiply(2.0);
		vertices[1].Multiply(2.0);
		vertices[2].Multiply(2.0);
		polygon.Set(vertices, 3);
		triangleShapeDef.filter.groupIndex = k_largeGroup;
		triangleBodyDef.position.Set(-5.0, 6.0);
		triangleBodyDef.fixedRotation = true; // look at me!

		var body2 = this.m_world.CreateBody(triangleBodyDef);
		body2.CreateFixture(triangleShapeDef);

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(-5.0, 10.0);
			var body = this.m_world.CreateBody(bd);

			var p = new b2PolygonShape();
			p.SetAsBox(0.5, 1.0);
			body.CreateFixture(p, 1.0);

			var jd = new b2PrismaticJointDef();
			jd.bodyA = body2;
			jd.bodyB = body;
			jd.enableLimit = true;
			jd.localAnchorA.Set(0.0, 4.0);
			jd.localAnchorB.SetZero();
			jd.localAxisA.Set(0.0, 1.0);
			jd.lowerTranslation = -1.0;
			jd.upperTranslation = 1.0;

			this.m_world.CreateJoint(jd);
		}

		// Small box
		polygon.SetAsBox(1.0, 0.5);
		var boxShapeDef = new b2FixtureDef();
		boxShapeDef.shape = polygon;
		boxShapeDef.density = 1.0;
		boxShapeDef.restitution = 0.1;

		boxShapeDef.filter.groupIndex = k_smallGroup;
		boxShapeDef.filter.categoryBits = k_boxCategory;
		boxShapeDef.filter.maskBits = k_boxMask;

		var boxBodyDef = new b2BodyDef();
		boxBodyDef.type = b2Body.b2_dynamicBody;
		boxBodyDef.position.Set(0.0, 2.0);

		var body3 = this.m_world.CreateBody(boxBodyDef);
		body3.CreateFixture(boxShapeDef);

		// Large box (recycle definitions)
		polygon.SetAsBox(2.0, 1.0);
		boxShapeDef.filter.groupIndex = k_largeGroup;
		boxBodyDef.position.Set(0.0, 6.0);

		var body4 = this.m_world.CreateBody(boxBodyDef);
		body4.CreateFixture(boxShapeDef);

		// Small circle
		var circle = new b2CircleShape();
		circle.m_radius = 1.0;

		var circleShapeDef = new b2FixtureDef();
		circleShapeDef.shape = circle;
		circleShapeDef.density = 1.0;

		circleShapeDef.filter.groupIndex = k_smallGroup;
		circleShapeDef.filter.categoryBits = k_circleCategory;
		circleShapeDef.filter.maskBits = k_circleMask;

		var circleBodyDef = new b2BodyDef();
		circleBodyDef.type = b2Body.b2_dynamicBody;
		circleBodyDef.position.Set(5.0, 2.0);

		var body5 = this.m_world.CreateBody(circleBodyDef);
		body5.CreateFixture(circleShapeDef);

		// Large circle
		circle.m_radius *= 2.0;
		circleShapeDef.filter.groupIndex = k_largeGroup;
		circleBodyDef.position.Set(5.0, 6.0);

		var body6 = this.m_world.CreateBody(circleBodyDef);
		body6.CreateFixture(circleShapeDef);
	}
};

TestCollisionFiltering._extend(Test);