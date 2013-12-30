function TestWaveMachine()
{
	this.parent.call(this);
}

TestWaveMachine.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.allowSleep = false;
			bd.position.Set(0.0, 10.0);
			var body = this.m_world.CreateBody(bd);

			var shape = new b2PolygonShape();
			shape.SetAsBox(0.5, 10.0, new b2Vec2(20.0, 0.0), 0.0);
			body.CreateFixture(shape, 5.0);
			shape.SetAsBox(0.5, 10.0, new b2Vec2(-20.0, 0.0), 0.0);
			body.CreateFixture(shape, 5.0);
			shape.SetAsBox(20.0, 0.5, new b2Vec2(0.0, 10.0), 0.0);
			body.CreateFixture(shape, 5.0);
			shape.SetAsBox(20.0, 0.5, new b2Vec2(0.0, -10.0), 0.0);
			body.CreateFixture(shape, 5.0);

			var jd = new b2RevoluteJointDef();
			jd.bodyA = ground;
			jd.bodyB = body;
			jd.localAnchorA.Set(0.0, 10.0);
			jd.localAnchorB.Set(0.0, 0.0);
			jd.referenceAngle = 0.0;
			jd.motorSpeed = 0.05 * Math.PI;
			jd.maxMotorTorque = 1e7;
			jd.enableMotor = true;
			this.m_joint = this.m_world.CreateJoint(jd);
		}

		this.m_world.SetParticleRadius(0.15);
		/*if (TestParticleType() == b2_waterParticle)
		{
			m_world->SetParticleDamping(0.2f);
		}*/

		{
			var pd = new b2ParticleGroupDef();
			//pd.flags = TestParticleType();

			var shape = new b2PolygonShape();
			shape.SetAsBox(9.0, 9.0, new b2Vec2(0.0, 10.0), 0.0);

			pd.shape = shape;
			this.m_world.CreateParticleGroup(pd);

		}

		this.m_time = 0;
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);
		if (this.m_hz > 0)
		{
			this.m_time += this.m_hz;
		}
		this.m_joint.SetMotorSpeed(0.20 * Math.cos(this.m_time) * Math.PI);
	}
};

TestWaveMachine._extend(Test);