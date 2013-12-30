function TestDamBreak()
{
	this.parent.call(this);
}

TestDamBreak.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2ChainShape();
			var vertices = [
				new b2Vec2(-20, 0),
				new b2Vec2(20, 0),
				new b2Vec2(20, 40),
				new b2Vec2(-20, 40) ];
			shape.CreateLoop(vertices, 4);
			ground.CreateFixture(shape, 0.0);

		}

		this.m_world.SetParticleRadius(0.15);
		this.m_world.SetParticleDamping(0.2);

		{
			var shape = new b2PolygonShape();
			shape.SetAsBox(8, 10, new b2Vec2(-12, 10.1), 0);
			var pd = new b2ParticleGroupDef();
			pd.shape = shape;
			this.m_world.CreateParticleGroup(pd);
		}

	}
};

TestDamBreak._extend(Test);