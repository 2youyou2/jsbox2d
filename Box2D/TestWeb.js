function TestWeb()
{
	this.parent.call(this);
}

TestWeb.prototype =
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

		this.m_bodies = [];
		this.m_joints = [];

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 0.5);

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;

			bd.position.Set(-5.0, 5.0);
			this.m_bodies[0] = this.m_world.CreateBody(bd);
			this.m_bodies[0].CreateFixture(shape, 5.0);

			bd.position.Set(5.0, 5.0);
			this.m_bodies[1] = this.m_world.CreateBody(bd);
			this.m_bodies[1].CreateFixture(shape, 5.0);

			bd.position.Set(5.0, 15.0);
			this.m_bodies[2] = this.m_world.CreateBody(bd);
			this.m_bodies[2].CreateFixture(shape, 5.0);

			bd.position.Set(-5.0, 15.0);
			this.m_bodies[3] = this.m_world.CreateBody(bd);
			this.m_bodies[3].CreateFixture(shape, 5.0);

			var jd = new b2DistanceJointDef();
			var p1 = new b2Vec2(), p2 = new b2Vec2(), d = new b2Vec2();

			jd.frequencyHz = 2.0;
			jd.dampingRatio = 0.0;

			jd.bodyA = ground;
			jd.bodyB = this.m_bodies[0];
			jd.localAnchorA.Set(-10.0, 0.0);
			jd.localAnchorB.Set(-0.5, -0.5);
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[0] = this.m_world.CreateJoint(jd);

			jd.bodyA = ground;
			jd.bodyB = this.m_bodies[1];
			jd.localAnchorA.Set(10.0, 0.0);
			jd.localAnchorB.Set(0.5, -0.5);
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[1] = this.m_world.CreateJoint(jd);

			jd.bodyA = ground;
			jd.bodyB = this.m_bodies[2];
			jd.localAnchorA.Set(10.0, 20.0);
			jd.localAnchorB.Set(0.5, 0.5);
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[2] = this.m_world.CreateJoint(jd);

			jd.bodyA = ground;
			jd.bodyB = this.m_bodies[3];
			jd.localAnchorA.Set(-10.0, 20.0);
			jd.localAnchorB.Set(-0.5, 0.5);
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[3] = this.m_world.CreateJoint(jd);

			jd.bodyA = this.m_bodies[0];
			jd.bodyB = this.m_bodies[1];
			jd.localAnchorA.Set(0.5, 0.0);
			jd.localAnchorB.Set(-0.5, 0.0);;
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[4] = this.m_world.CreateJoint(jd);

			jd.bodyA = this.m_bodies[1];
			jd.bodyB = this.m_bodies[2];
			jd.localAnchorA.Set(0.0, 0.5);
			jd.localAnchorB.Set(0.0, -0.5);
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[5] = this.m_world.CreateJoint(jd);

			jd.bodyA = this.m_bodies[2];
			jd.bodyB = this.m_bodies[3];
			jd.localAnchorA.Set(-0.5, 0.0);
			jd.localAnchorB.Set(0.5, 0.0);
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[6] = this.m_world.CreateJoint(jd);

			jd.bodyA = this.m_bodies[3];
			jd.bodyB = this.m_bodies[0];
			jd.localAnchorA.Set(0.0, -0.5);
			jd.localAnchorB.Set(0.0, 0.5);
			p1 = jd.bodyA.GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB.GetWorldPoint(jd.localAnchorB);
			d = b2Vec2.Subtract(p2, p1);
			jd.length = d.Length();
			this.m_joints[7] = this.m_world.CreateJoint(jd);
		}
	},

	Keyboard: function(key)
	{
		switch (key)
		{
		case 'B'.charCodeAt():
			for (var i = 0; i < 4; ++i)
			{
				if (this.m_bodies[i])
				{
					this.m_world.DestroyBody(this.m_bodies[i]);
					this.m_bodies[i] = null;
					break;
				}
			}
			break;

		case 'J'.charCodeAt():
			for (var i = 0; i < 8; ++i)
			{
				if (this.m_joints[i])
				{
					this.m_world.DestroyJoint(this.m_joints[i]);
					this.m_joints[i] = null;
					break;
				}
			}
			break;
		}
	},

	JointDestroyed: function(joint)
	{
		this.parent.prototype.JointDestroyed.call(this, joint);

		for (var i = 0; i < 8; ++i)
		{
			if (this.m_joints[i] == joint)
			{
				this.m_joints[i] = null;
				break;
			}
		}
	}
};

TestWeb._extend(Test);