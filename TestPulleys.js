function TestPulleys()
{
	this.parent.call(this);
}

TestPulleys.prototype =
{
	y: 16.0,
	L: 12.0,
	a: 1.0,
	b: 2.0,

	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var edge = new b2EdgeShape();
			edge.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			//ground.CreateFixture(shape, 0.0);

			var circle = new b2CircleShape();
			circle.m_radius = 2.0;

			circle.m_p.Set(-10.0, this.y + this.b + this.L);
			ground.CreateFixture(circle, 0.0);

			circle.m_p.Set(10.0, this.y + this.b + this.L);
			ground.CreateFixture(circle, 0.0);
		}

		{

			var shape = new b2PolygonShape();
			shape.SetAsBox(this.a, this.b);

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;

			//bd.fixedRotation = true;
			bd.position.Set(-10.0, this.y);
			var body1 = this.m_world.CreateBody(bd);
			body1.CreateFixture(shape, 5.0);

			bd.position.Set(10.0, this.y);
			var body2 = this.m_world.CreateBody(bd);
			body2.CreateFixture(shape, 5.0);

			var pulleyDef = new b2PulleyJointDef();
			var anchor1 = new b2Vec2(-10.0, this.y + this.b);
			var anchor2 = new b2Vec2(10.0, this.y + this.b);
			var groundAnchor1 = new b2Vec2(-10.0, this.y + this.b + this.L);
			var groundAnchor2 = new b2Vec2(10.0, this.y + this.b + this.L);
			pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5);

			this.m_joint1 = this.m_world.CreateJoint(pulleyDef);
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var ratio = this.m_joint1.GetRatio();
		var L = this.m_joint1.GetCurrentLengthA() + ratio * this.m_joint1.GetCurrentLengthB();
		this.m_drawStringFunc("L1 + " + ratio + " * L2 = " + L);
	}
};

TestPulleys._extend(Test);