function TestRevolute()
{
	this.parent.call(this);
}

TestRevolute.prototype =
{
	Initialize: function()
	{
		var ground = null;
		{
			var bd = new b2BodyDef();
			ground = this.m_world.CreateBody(bd);

			var shape = new b2EdgeShape();
			shape.Set(new b2Vec2(-40.0, 0.0), new b2Vec2(40.0, 0.0));

			var fd = new b2FixtureDef();
			fd.shape = shape;
			//fd.filter.categoryBits = 2;

			ground.CreateFixture(fd);
		}

		{
			var shape = new b2CircleShape();
			shape.m_radius = 0.5;

			var bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;

			var rjd = new b2RevoluteJointDef();

			bd.position.Set(-10.0, 20.0);
			var body = this.m_world.CreateBody(bd);
			body.CreateFixture(shape, 5.0);

			var w = 100.0;
			body.SetAngularVelocity(w);
			body.SetLinearVelocity(new b2Vec2(-8.0 * w, 0.0));

			rjd.Initialize(ground, body, new b2Vec2(-10.0, 12.0));
			rjd.motorSpeed = 1.0 * Math.PI;
			rjd.maxMotorTorque = 10000.0;
			rjd.enableMotor = false;
			rjd.lowerAngle = -0.25 * Math.PI;
			rjd.upperAngle = 0.5 * Math.PI;
			rjd.enableLimit = true;
			rjd.collideConnected = true;

			this.m_joint = this.m_world.CreateJoint(rjd);
		}

		{
			var circle_shape = new b2CircleShape();
			circle_shape.m_radius = 3.0;

			var circle_bd = new b2BodyDef();
			circle_bd.type = b2Body.b2_dynamicBody;
			circle_bd.position.Set(5.0, 30.0);

			var fd = new b2FixtureDef();
			fd.density = 5.0;
			fd.filter.maskBits = 1;
			fd.shape = circle_shape;

			this.m_ball = this.m_world.CreateBody(circle_bd);
			this.m_ball.CreateFixture(fd);

			var polygon_shape = new b2PolygonShape();
			polygon_shape.SetAsBox(10.0, 0.2, new b2Vec2(-10.0, 0.0), 0.0);

			var polygon_bd = new b2BodyDef();
			polygon_bd.position.Set(20.0, 10.0);
			polygon_bd.type = b2Body.b2_dynamicBody;
			polygon_bd.bullet = true;
			var polygon_body = this.m_world.CreateBody(polygon_bd);
			polygon_body.CreateFixture(polygon_shape, 2.0);

			var rjd = new b2RevoluteJointDef();
			rjd.Initialize(ground, polygon_body, new b2Vec2(20.0, 10.0));
			rjd.lowerAngle = -0.25 * Math.PI;
			rjd.upperAngle = 0.0 * Math.PI;
			rjd.enableLimit = true;
			this.m_world.CreateJoint(rjd);
		}

		// Tests mass computation of a small object far from the origin
		{
			var bodyDef = new b2BodyDef();
			bodyDef.type = b2Body.b2_dynamicBody;
			var body = this.m_world.CreateBody(bodyDef);

			var polyShape = new b2PolygonShape();
			var verts = [new b2Vec2( 17.63, 36.31 ), new b2Vec2( 17.52, 36.69 ), new b2Vec2( 17.19, 36.36 )];
			polyShape.Set(verts, 3);

			var polyFixtureDef = new b2FixtureDef();
			polyFixtureDef.shape = polyShape;
			polyFixtureDef.density = 1;

			body.CreateFixture(polyFixtureDef);	//assertion hits inside here
		}
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'L'.charCodeAt():
			this.m_joint.EnableLimit(!this.m_joint.IsLimitEnabled());
			break;

		case 'M'.charCodeAt():
			this.m_joint.EnableMotor(!this.m_joint.IsMotorEnabled());
			break;
		}
	}
};

TestRevolute._extend(Test);