function TestGears()
{
	this.parent.call(this);
}

TestGears.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(50.0, 0.0), new b2Vec2(-50.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		{
			var circle1 = new b2CircleShape();
			circle1.m_radius = 1.0;

			var box = new b2PolygonShape();
			box.SetAsBox(0.5, 5.0);

			var circle2 = new b2CircleShape();
			circle2.m_radius = 2.0;

			var bd1 = new b2BodyDef();
			bd1.type = b2Body.b2_staticBody;
			bd1.position.Set(10.0, 9.0);
			var body1 = this.m_world.CreateBody(bd1);
			body1.CreateFixture(circle1, 5.0);

			var bd2 = new b2BodyDef();
			bd2.type = b2Body.b2_dynamicBody;
			bd2.position.Set(10.0, 8.0);
			var body2 = this.m_world.CreateBody(bd2);
			body2.CreateFixture(box, 5.0);

			var bd3 = new b2BodyDef();
			bd3.type = b2Body.b2_dynamicBody;
			bd3.position.Set(10.0, 6.0);
			var body3 = this.m_world.CreateBody(bd3);
			body3.CreateFixture(circle2, 5.0);

			var jd1 = new b2RevoluteJointDef();
			jd1.Initialize(body2, body1, bd1.position);
			var joint1 = this.m_world.CreateJoint(jd1);

			var jd2 = new b2RevoluteJointDef();
			jd2.Initialize(body2, body3, bd3.position);
			var joint2 = this.m_world.CreateJoint(jd2);

			var jd4 = new b2GearJointDef();
			jd4.bodyA = body1;
			jd4.bodyB = body3;
			jd4.joint1 = joint1;
			jd4.joint2 = joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			this.m_world.CreateJoint(jd4);
		}

		{
			var circle1 = new b2CircleShape();
			circle1.m_radius = 1.0;

			var circle2 = new b2CircleShape();
			circle2.m_radius = 2.0;

			var box = new b2PolygonShape();
			box.SetAsBox(0.5, 5.0);

			var bd1 = new b2BodyDef();
			bd1.type = b2Body.b2_dynamicBody;
			bd1.position.Set(-3.0, 12.0);
			var body1 = this.m_world.CreateBody(bd1);
			body1.CreateFixture(circle1, 5.0);

			var jd1 = new b2RevoluteJointDef();
			jd1.bodyA = ground;
			jd1.bodyB = body1;
			jd1.localAnchorA.Assign(ground.GetLocalPoint(bd1.position));
			jd1.localAnchorB.Assign(body1.GetLocalPoint(bd1.position));
			jd1.referenceAngle = body1.GetAngle() - ground.GetAngle();
			this.m_joint1 = this.m_world.CreateJoint(jd1);

			var bd2 = new b2BodyDef();
			bd2.type = b2Body.b2_dynamicBody;
			bd2.position.Set(0.0, 12.0);
			var body2 = this.m_world.CreateBody(bd2);
			body2.CreateFixture(circle2, 5.0);

			var jd2 = new b2RevoluteJointDef();
			jd2.Initialize(ground, body2, bd2.position);
			this.m_joint2 = this.m_world.CreateJoint(jd2);

			var bd3 = new b2BodyDef();
			bd3.type = b2Body.b2_dynamicBody;
			bd3.position.Set(2.5, 12.0);
			var body3 = this.m_world.CreateBody(bd3);
			body3.CreateFixture(box, 5.0);

			var jd3 = new b2PrismaticJointDef();
			jd3.Initialize(ground, body3, bd3.position, new b2Vec2(0.0, 1.0));
			jd3.lowerTranslation = -5.0;
			jd3.upperTranslation = 5.0;
			jd3.enableLimit = true;

			this.m_joint3 = this.m_world.CreateJoint(jd3);

			var jd4 = new b2GearJointDef();
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = this.m_joint1;
			jd4.joint2 = this.m_joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			this.m_joint4 = this.m_world.CreateJoint(jd4);

			var jd5 = new b2GearJointDef();
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = this.m_joint2;
			jd5.joint2 = this.m_joint3;
			jd5.ratio = -1.0 / circle2.m_radius;
			this.m_joint5 = this.m_world.CreateJoint(jd5);
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		var ratio = this.m_joint4.GetRatio();
		var value = this.m_joint1.GetJointAngle() + ratio * this.m_joint2.GetJointAngle();
		this.m_drawStringFunc("theta1 + " + ratio + " * theta2 = " + value);

		ratio = this.m_joint5.GetRatio();
		value = this.m_joint2.GetJointAngle() + ratio * this.m_joint3.GetJointTranslation();
		this.m_drawStringFunc("theta2 + " + ratio + " * delta = " + value);
	}
};

TestGears._extend(Test);