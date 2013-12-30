function TestSurfaceTension()
{
	this.parent.call(this);
}

TestSurfaceTension.prototype =
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
					new b2Vec2(-20, 40),
					new b2Vec2(-40, 40)];
				shape.Set(vertices, 4);
				ground.CreateFixture(shape, 0.0);
			}

			{
				vertices = [
					new b2Vec2(20, -1),
					new b2Vec2(40, -1),
					new b2Vec2(40, 40),
					new b2Vec2(20, 40)];
				shape.Set(vertices, 4);
				ground.CreateFixture(shape, 0.0);
			}
		}

		this.m_world.SetParticleRadius(0.3);

		{
			var shape = new b2CircleShape();
			shape.m_p.Set(0, 20);
			shape.m_radius = 5;
			var pd = new b2ParticleGroupDef();
			pd.flags = b2ParticleDef.b2_tensileParticle | b2ParticleDef.b2_colorMixingParticle;
			pd.shape = shape;
			pd.color.Set(Math.floor(Math.random() * 255), Math.floor(Math.random() * 255), Math.floor(Math.random() * 255), Math.floor(Math.random() * 255));
			this.m_world.CreateParticleGroup(pd);
		}

		{
			var shape = new b2CircleShape();
			shape.m_p.Set(-10, 20);
			shape.m_radius = 5;
			var pd = new b2ParticleGroupDef();
			pd.flags = b2ParticleDef.b2_tensileParticle | b2ParticleDef.b2_colorMixingParticle;
			pd.shape = shape;
			pd.color.Set(Math.floor(Math.random() * 255), Math.floor(Math.random() * 255), Math.floor(Math.random() * 255), Math.floor(Math.random() * 255));
			this.m_world.CreateParticleGroup(pd);
		}

		{
			var shape = new b2PolygonShape();
			var vertices = [
				new b2Vec2(0, 30),
				new b2Vec2(20, 30),
				new b2Vec2(20, 35),
				new b2Vec2(0, 35)];
			shape.Set(vertices, 4);
			var pd = new b2ParticleGroupDef();
			pd.flags = b2ParticleDef.b2_colorMixingParticle | b2ParticleDef.b2_powderParticle;
			pd.shape = shape;
			pd.color.Set(Math.floor(Math.random() * 255), Math.floor(Math.random() * 255), Math.floor(Math.random() * 255), 255);
			this.m_world.CreateParticleGroup(pd);
		}

		{
			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(0, 25);
			var body = this.m_world.CreateBody(bd);
			var shape = new b2PolygonShape();
			shape.Set([new b2Vec2(-1, -1), new b2Vec2(0.5, 1), new b2Vec2(1, -1)], 3);
			body.CreateFixture(shape, 0.5);
		}
	}
};

TestSurfaceTension._extend(Test);