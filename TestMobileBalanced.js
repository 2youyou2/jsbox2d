function TestMobileBalanced()
{
	this.parent.call(this);
}

TestMobileBalanced.prototype =
{
	e_depth: 4,

	Initialize: function()
	{
		var ground;

		// Create ground body.
		{
			var bodyDef = new b2BodyDef();
			bodyDef.position.Set(0.0, 20.0);
			ground = this.m_world.CreateBody(bodyDef);
		}

		var a = 0.5;
		var h = new b2Vec2(0.0, a);

		var root = this.AddNode(ground, new b2Vec2(0, 0), 0, 3.0, a);

		var jointDef = new b2RevoluteJointDef();
		jointDef.bodyA = ground;
		jointDef.bodyB = root;
		jointDef.localAnchorA.SetZero();
		jointDef.localAnchorB.Assign(h);
		this.m_world.CreateJoint(jointDef);
	},

	AddNode: function(parent, localAnchor, depth, offset, a)
	{
		var density = 20.0;
		var h = new b2Vec2(0.0, a);

		var p = b2Vec2.Add(parent.GetPosition(), b2Vec2.Subtract(localAnchor, h));

		var bodyDef = new b2BodyDef();
		bodyDef.type = b2Body.b2_dynamicBody;
		bodyDef.position.Assign(p);
		var body = this.m_world.CreateBody(bodyDef);

		var shape = new b2PolygonShape();
		shape.SetAsBox(0.25 * a, a);
		body.CreateFixture(shape, density);

		if (depth == this.e_depth)
		{
			return body;
		}

		shape.SetAsBox(offset, 0.25 * a, new b2Vec2(0, -a), 0.0);
		body.CreateFixture(shape, density);

		var a1 = new b2Vec2(offset, -a);
		var a2 = new b2Vec2(-offset, -a);
		var body1 = this.AddNode(body, a1, depth + 1, 0.5 * offset, a);
		var body2 = this.AddNode(body, a2, depth + 1, 0.5 * offset, a);

		var jointDef = new b2RevoluteJointDef();
		jointDef.bodyA = body;
		jointDef.localAnchorB.Assign(h);

		jointDef.localAnchorA.Assign(a1);
		jointDef.bodyB = body1;
		this.m_world.CreateJoint(jointDef);

		jointDef.localAnchorA.Assign(a2);
		jointDef.bodyB = body2;
		this.m_world.CreateJoint(jointDef);

		return body;
	}
};

TestMobileBalanced._extend(Test);