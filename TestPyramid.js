function TestPyramid()
{
	this.parent.call(this);
}

TestPyramid.prototype =
{
	Initialize: function()
	{
		{
			var bd = new b2BodyDef();
			var ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));
			ground.CreateFixture(shape, 0.0);
		}

		{
			var a = 0.5;
			var shape = new b2PolygonShape();
			shape.SetAsBox(a, a);

			var x = new b2Vec2(-7.0, 0.75);
			var y = new b2Vec2();
			var deltaX = new b2Vec2(0.5625, 1.25);
			var deltaY = new b2Vec2(1.125, 0.0);
			var e_count = 20;

			for (var i = 0; i < e_count; ++i)
			{
				y.Assign(x);

				for (var j = i; j < e_count; ++j)
				{
					var bd = new b2BodyDef();
					bd.type = b2Body.b2_dynamicBody;
					bd.position.Assign(y);
					var body = this.m_world.CreateBody(bd);
					body.CreateFixture(shape, 5.0);

					y.Add(deltaY);
				}

				x.Add(deltaX);
			}
		}
	}
};

TestPyramid._extend(Test);