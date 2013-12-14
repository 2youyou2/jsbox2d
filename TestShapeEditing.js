function TestShapeEditing()
{
	this.parent.call(this);
}

TestShapeEditing.prototype =
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

		var bd = new b2BodyDef();
		bd.type = b2Body.b2_dynamicBody;
		bd.position.Set(0.0, 10.0);
		this.m_body = this.m_world.CreateBody(bd);

		var shape = new b2PolygonShape();
		shape.SetAsBox(4.0, 4.0, new b2Vec2(0.0, 0.0), 0.0);
		this.m_fixture1 = this.m_body.CreateFixture(shape, 10.0);

		this.m_fixture2 = null;

		this.m_sensor = false;
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'C'.charCodeAt():
			if (this.m_fixture2 == null)
			{
				var shape = new b2CircleShape();
				shape.m_radius = 3.0;
				shape.m_p.Set(0.5, -4.0);
				this.m_fixture2 = this.m_body.CreateFixture(shape, 10.0);
				this.m_body.SetAwake(true);
			}
			break;

		case 'D'.charCodeAt():
			if (this.m_fixture2 != null)
			{
				this.m_body.DestroyFixture(this.m_fixture2);
				this.m_fixture2 = null;
				this.m_body.SetAwake(true);
			}
			break;

		case 'S'.charCodeAt():
			if (this.m_fixture2 != null)
			{
				this.m_sensor = !this.m_sensor;
				this.m_fixture2.SetSensor(this.m_sensor);
			}
			break;
		}
	},

	Step: function()
	{
		this.parent.prototype.Step.call(this);

		this.m_drawStringFunc("Press: (c) create a shape, (d) destroy a shape, (s) to toggle sensor.");
	}
};

TestShapeEditing._extend(Test);