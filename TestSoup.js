function TestSoup()
{
	this.parent.call(this);
}

TestSoup.prototype =
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
				var vertices = [
					new b2Vec2(-40, -1),
					new b2Vec2(-20, -1),
					new b2Vec2(-20, 20),
					new b2Vec2(-40, 30)];
				shape.Set(vertices, 4);
				ground.CreateFixture(shape, 0.0);
			}

			{
				var vertices = [
					new b2Vec2(20, -1),
					new b2Vec2(40, -1),
					new b2Vec2(40, 30),
					new b2Vec2(20, 20)];
				shape.Set(vertices, 4);
				ground.CreateFixture(shape, 0.0);
			}
		}

		this.m_world.SetParticleRadius(0.3);
		{
			shape.SetAsBox(20, 10, new b2Vec2(0, 10), 0);
			var pd = new b2ParticleGroupDef();
			pd.flags = b2ParticleDef.b2_waterParticle;
			pd.shape = shape;
			this.m_world.CreateParticleGroup(pd);
		}

		{
			bd.type = b2Body.b2_dynamicBody;
			var body = this.m_world.CreateBody(bd);
			shape = new b2CircleShape();
			shape.m_p.Set(0, 10);
			shape.m_radius = 1;
			body.CreateFixture(shape, 0.1);
		}

		bd.position.y = 16;

		for (var x = 0; x < 50; ++x)
		{
			bd.position.x = b2RandomFloat(-20, 20);
			body = this.m_world.CreateBody(bd);
			shape = new b2PolygonShape();

			var verts = [];

			for (var i = 0; i < 4; ++i)
				verts[i] = new b2Vec2(b2RandomFloat(-1, 1), b2RandomFloat(-1, 1));

			shape.Set(verts, 4);
			body.CreateFixture(shape, 0.1);
		}
	}
};

TestSoup._extend(Test);