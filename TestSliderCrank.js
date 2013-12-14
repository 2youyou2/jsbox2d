function TestSliderCrank()
{
	this.parent.call(this);
}

TestSliderCrank.prototype =
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

		{
			var prevBody = ground;

			// Define crank.
			{
				var shape = new b2PolygonShape();
				shape.SetAsBox(0.5, 2.0);

				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(0.0, 7.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(shape, 2.0);

				var rjd = new b2RevoluteJointDef();
				rjd.Initialize(prevBody, body, new b2Vec2(0.0, 5.0));
				rjd.motorSpeed = 1.0 * Math.PI;
				rjd.maxMotorTorque = 10000.0;
				rjd.enableMotor = true;
				this.m_joint1 = this.m_world.CreateJoint(rjd);

				prevBody = body;
			}

			// Define follower.
			{
				var shape = new b2PolygonShape();
				shape.SetAsBox(0.5, 4.0);

				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(0.0, 13.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(shape, 2.0);

				var rjd = new b2RevoluteJointDef();
				rjd.Initialize(prevBody, body, new b2Vec2(0.0, 9.0));
				rjd.enableMotor = false;
				this.m_world.CreateJoint(rjd);

				prevBody = body;
			}

			// Define piston
			{
				var shape = new b2PolygonShape();
				shape.SetAsBox(1.5, 1.5);

				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.fixedRotation = true;
				bd.position.Set(0.0, 17.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(shape, 2.0);

				var rjd = new b2RevoluteJointDef();
				rjd.Initialize(prevBody, body, new b2Vec2(0.0, 17.0));
				this.m_world.CreateJoint(rjd);

				var pjd = new b2PrismaticJointDef();
				pjd.Initialize(ground, body, new b2Vec2(0.0, 17.0), new b2Vec2(0.0, 1.0));

				pjd.maxMotorForce = 1000.0;
				pjd.enableMotor = true;

				this.m_joint2 = this.m_world.CreateJoint(pjd);
			}

			// Create a payload
			{
				var shape = new b2PolygonShape();
				shape.SetAsBox(1.5, 1.5);

				var bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(0.0, 23.0);
				var body = this.m_world.CreateBody(bd);
				body.CreateFixture(shape, 2.0);
			}
		}
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'F'.charCodeAt():
			this.m_joint2.EnableMotor(!this.m_joint2.IsMotorEnabled());
			this.m_joint2.GetBodyB().SetAwake(true);
			break;

		case 'F'.charCodeAt():
			this.m_joint1.EnableMotor(!this.m_joint1.IsMotorEnabled());
			this.m_joint1.GetBodyB().SetAwake(true);
			break;
		}
	}
};

TestSliderCrank._extend(Test);