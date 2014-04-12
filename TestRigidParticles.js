function TestRigidParticles()
{
	this.parent.call(this);
}

TestRigidParticles.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			{
				var shape = new b2PolygonShape();
				var vertices = [
					new b2Vec2(-40, -10),
					new b2Vec2(40, -10),
					new b2Vec2(40, 0),
					new b2Vec2(-40, 0)];
				shape.Set(vertices, 4);
				ground.CreateFixture(shape, 0.0);
			}

			{
				vertices = [
					new b2Vec2(-40, -1),
					new b2Vec2(-20, -1),
					new b2Vec2(-20, 20),
					new b2Vec2(-40, 20)];
				shape.Set(vertices, 4);
				ground.CreateFixture(shape, 0.0);
			}

			{
				vertices = [
					new b2Vec2(20, -1),
					new b2Vec2(40, -1),
					new b2Vec2(40, 20),
					new b2Vec2(20, 20)];
				shape.Set(vertices, 4);
				ground.CreateFixture(shape, 0.0);
			}
		}

		this.m_world.SetParticleRadius(0.3);

		{
			var shape = new b2CircleShape();
			shape.m_p.Set(0, 30);
			shape.m_radius = 5;
			var pd = new b2ParticleGroupDef();
			pd.groupFlags = b2ParticleGroup.b2_rigidParticleGroup | b2ParticleGroup.b2_solidParticleGroup;
			pd.shape = shape;
			pd.color.Set(255, 0, 0, 255);
			this.m_world.CreateParticleGroup(pd);
		}

		{
			shape.m_p.Set(-10, 30);
			shape.m_radius = 5;
			var pd = new b2ParticleGroupDef();
			pd.groupFlags = b2ParticleGroup.b2_rigidParticleGroup | b2ParticleGroup.b2_solidParticleGroup;
			pd.shape = shape;
			pd.color.Set(0, 255, 0, 255);
			this.m_world.CreateParticleGroup(pd);
		}

		{
			shape = new b2PolygonShape();
			shape.SetAsBox(10, 5);
			var pd = new b2ParticleGroupDef();
			pd.groupFlags = b2ParticleGroup.b2_rigidParticleGroup | b2ParticleGroup.b2_solidParticleGroup;
			pd.position.Set(10, 40);
			pd.angle = -0.5;
			pd.angularVelocity = 2.0;
			pd.shape = shape;
			pd.color.Set(0, 0, 255, 255);
			this.m_world.CreateParticleGroup(pd);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			var body = this.m_world.CreateBody(bd);
			shape = new b2CircleShape();
			shape.m_p.Set(0, 80);
			shape.m_radius = 5;
			body.CreateFixture(shape, 0.5);
		}
	}
};

TestRigidParticles._extend(Test);