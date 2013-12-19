function TestPlinko()
{
	this.parent.call(this);
}

TestPlinko.prototype =
{
	Initialize: function()
	{
		{
			var separationy = 2.8;
			var separationx = 1.0;
			var rad = 0.05;
			var w = 9;
			var h = 10;

			var def = new b2BodyDef();
			var body = this.m_world.CreateBody(def);

			var fd = new b2FixtureDef();
			fd.shape = new b2PolygonShape();
			fd.shape.SetAsBox(rad, rad, new b2Vec2(0, 0), Math.PI / 4);
			fd.restitution = 1.0;

			for (var x = 0; x < w; ++x)
			{
				for (var y = 0; y < h; ++y)
				{
					fd.shape.m_centroid.Set(x * separationx, y * separationy);

					if (x & 1)
						fd.shape.m_centroid.y -= separationy / 2;

					fd.shape.SetAsBox(rad, rad, fd.shape.m_centroid, Math.PI / 4);

					body.CreateFixture(fd);
				}
			}

			fd.restitution = 0.4;

			fd.shape = new b2ChainShape();
			var verts = [];

			for (var y = -1; y < (h * 2) - 1; ++y)
			{
				verts.push(new b2Vec2(-2 * separationx, y * (separationy / 2)));

				if (y & 1)
					verts[verts.length - 1].x += separationx;
			}
			fd.shape.CreateChain(verts, verts.length);

			body.CreateFixture(fd);

			verts.length = 0;

			for (var y = -1; y < (h * 2) - 1; ++y)
			{
				verts.push(new b2Vec2((w + 1) * separationx, y * (separationy / 2)));

				if (y & 1)
					verts[verts.length - 1].x -= separationx;
			}
			fd.shape.CreateChain(verts, verts.length);

			body.CreateFixture(fd);

			fd.shape = new b2EdgeShape();
			fd.shape.Set(new b2Vec2(-1 * separationx, -separationy / 2), new b2Vec2(w * separationx, -separationy / 2));
			body.CreateFixture(fd);

			def.type = b2Body.b2_dynamicBody;
			def.position.Set(0, 14);
			fd.shape = new b2CircleShape();
			fd.density = 0.1;
			fd.shape.m_radius = separationx / 2;
			fd.shape.m_p.SetZero();

			this.m_world.CreateBody(def).CreateFixture(fd);
		}
	}
};

TestPlinko._extend(Test);