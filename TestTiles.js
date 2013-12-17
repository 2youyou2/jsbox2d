function TestTiles()
{
	this.parent.call(this);
}

TestTiles.prototype =
{
	Initialize: function()
	{
		var e_count = 20;
		this.m_fixtureCount = 0;

		{
			var a = 0.5;
			var bd = new b2BodyDef();
			bd.position.y = -a;
			var ground = this.m_world.CreateBody(bd);

			if (true)
			{
				var N = 200;
				var M = 10;
				var position = new b2Vec2();
				position.y = 0.0;
				for (var j = 0; j < M; ++j)
				{
					position.x = -N * a;
					for (var i = 0; i < N; ++i)
					{
						var shape = new b2PolygonShape();
						shape.SetAsBox(a, a, position, 0.0);
						ground.CreateFixture(shape, 0.0);
						++this.m_fixtureCount;
						position.x += 2.0 * a;
					}
					position.y -= 2.0 * a;
				}
			}
			else
			{
				var N = 200;
				var M = 10;
				var position = new b2Vec2();
				position.x = -N * a;
				for (var i = 0; i < N; ++i)
				{
					position.y = 0.0;
					for (var j = 0; j < M; ++j)
					{
						var shape = new b2PolygonShape();
						shape.SetAsBox(a, a, position, 0.0);
						ground.CreateFixture(shape, 0.0);
						position.y -= 2.0 * a;
					}
					position.x += 2.0 * a;
				}
			}
		}

		{
			var a = 0.5;
			var shape = new b2PolygonShape();
			shape.SetAsBox(a, a);

			var x = new b2Vec2(-7.0, 0.75);
			var y = new b2Vec2();
			var deltaX = new b2Vec2(0.5625, 1.25);
			var deltaY = new b2Vec2(1.125, 0.0);

			for (var i = 0; i < e_count; ++i)
			{
				y.Assign(x);

				for (var j = i; j < e_count; ++j)
				{
					var bd = new b2BodyDef();
					bd.type = b2Body.b2_dynamicBody;
					bd.position.Assign(y);

					//if (i == 0 && j == 0)
					//{
					//	bd.allowSleep = false;
					//}
					//else
					//{
					//	bd.allowSleep = true;
					//}

					var body = this.m_world.CreateBody(bd);
					body.CreateFixture(shape, 5.0);
					++this.m_fixtureCount;
					y.Add(deltaY);
				}

				x.Add(deltaX);
			}
		}
	}
};

TestTiles._extend(Test);