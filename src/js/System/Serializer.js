// The frontend to the JSON serializer.
var b2JsonSerializer =
{
	/** @param {b2World} world */
	serialize: function(world)
	{
		// compile list of shapes
		// TODO: check for duplicate shapes
		var shapes = [];

		/** @type Number */
		var i;

		/** @type String */
		var serialized;

		/** @type b2Body */
		var b;

		/** @type b2Fixture */
		var f;

		/** @type b2Shape */
		var shape;

		for (b = world.GetBodyList(); b; b = b.GetNext())
		{
			for (f = b.GetFixtureList(); f; f = f.GetNext())
			{
				shape = f.GetShape();
				f.__temp_shape_id = shapes.length;
				shapes.push(shape._serialize());
			}
		}

		// compile list of fixtures
		// TODO: check for duplicate fixtures
		var fixtures = [];

		for (b = world.GetBodyList(); b; b = b.GetNext())
		{
			b.__temp_fixture_ids = [];

			for (f = b.GetFixtureList(); f; f = f.GetNext())
			{
				serialized = f._serialize();
				serialized['shape'] = f.__temp_shape_id;
				delete f.__temp_shape_id;

				b.__temp_fixture_ids.push(fixtures.length);

				fixtures.push(serialized);
			}
		}

		// compile list of bodies
		var bodies = [];

		for (b = world.GetBodyList(); b; b = b.GetNext())
		{
			serialized = b._serialize();
			serialized.fixtures = [];

			for (i = 0; i < b.__temp_fixture_ids.length; ++i)
				serialized.fixtures.push(b.__temp_fixture_ids[i]);

			delete b.__temp_fixture_ids;

			b.__temp_body_id = bodies.length;
			bodies.push(serialized);
		};

		// compile list of joints
		var joints = [];

		/** @type b2Joint */
		var j;

		for (j = world.GetJointList(), i = 0; j; j = j.GetNext(), ++i)
			j.__temp_joint_id = i;

		for (j = world.GetJointList(); j; j = j.GetNext())
		{
			// special case: don't serialize mouse joints
			if (j.GetType() === b2Joint.e_mouseJoint)
				continue;

			serialized = j._serialize();

			serialized['bodyA'] = j.GetBodyA().__temp_body_id;
			serialized['bodyB'] = j.GetBodyB().__temp_body_id;

			joints.push(serialized);
		}

		for (j = world.GetJointList(); j; j = j.GetNext())
			delete j.__temp_joint_id;

		for (b = world.GetBodyList(); b; b = b.GetNext())
			delete b.__temp_body_id;

		return { shapes: shapes, fixtures: fixtures, bodies: bodies, joints: joints };
	},

	/** @param {b2World} world */
	deserialize: function(serialized, world, clear)
	{
		var deserialized = JSON.parse(serialized);

		if (clear)
		{
			for (var b = world.GetBodyList(); b; )
			{
				var next = b.GetNext();
				world.DestroyBody(b);
				b = next;
			}

			for (var j = world.GetJointList(); j; )
			{
				var next = j.GetNext();
				world.DestroyJoint(j);
				j = next;
			}
		}

		// decompile shapes
		var shapes = [];

		for (var i = 0; i < deserialized.shapes.length; ++i)
		{
			var shapeData = deserialized.shapes[i];
			var shape;

			switch (shapeData.m_type)
			{
				case b2Shape.e_circle:
					shape = new b2CircleShape();
					break;
				case b2Shape.e_edge:
					shape = new b2EdgeShape();
					break;
				case b2Shape.e_chain:
					shape = new b2ChainShape();
					break;
				case b2Shape.e_polygon:
					shape = new b2PolygonShape();
					break;
			}

			shape._deserialize(shapeData);
			shapes.push(shape);
		}

		// decompile fixtures
		var fixtures = [];

		for (i = 0; i < deserialized.fixtures.length; ++i)
		{
			var fixtureData = deserialized.fixtures[i];
			var fixture = new b2FixtureDef();

			fixture._deserialize(fixtureData);
			fixture.shape = shapes[fixtureData['shape']];

			fixtures.push(fixture);
		}

		// decompile bodies
		var bodies = [];

		for (i = 0; i < deserialized.bodies.length; ++i)
		{
			var bodyData = deserialized.bodies[i];
			var def = new b2BodyDef();

			def._deserialize(bodyData);

			var body = world.CreateBody(def);

			for (var x = 0; x < bodyData.fixtures.length; ++x)
				body.CreateFixture(fixtures[bodyData.fixtures[x]]);

			bodies.push(body);
		}

		// decompile joints
		var joints = [];
		var gears = [];

		for (i = 0; i < deserialized.joints.length; ++i)
		{
			var jointData = deserialized.joints[i];
			var jointDef;

			switch (jointData.type)
			{
				case b2Joint.e_revoluteJoint:
					jointDef = new b2RevoluteJointDef();
					break;
				case b2Joint.e_prismaticJoint:
					jointDef = new b2PrismaticJointDef();
					break;
				case b2Joint.e_distanceJoint:
					jointDef = new b2DistanceJointDef();
					break;
				case b2Joint.e_pulleyJoint:
					jointDef = new b2PulleyJointDef();
					break;
				case b2Joint.e_gearJoint:
					jointDef = new b2GearJointDef();
					break;
				case b2Joint.e_wheelJoint:
					jointDef = new b2WheelJointDef();
					break;
				case b2Joint.e_weldJoint:
					jointDef = new b2WeldJointDef();
					break;
				case b2Joint.e_frictionJoint:
					jointDef = new b2FrictionJointDef();
					break;
				case b2Joint.e_ropeJoint:
					jointDef = new b2RopeJointDef();
					break;
				case b2Joint.e_motorJoint:
					jointDef = new b2MotorJointDef();
					break;
				default:
					throw new Error('unknown joint');
			}

			jointDef._deserialize(jointData, bodies);

			if (jointData.type === b2Joint.e_gearJoint)
			{
				gears.push([jointDef, joints.length]);
				joints.push(null);
			}
			else
			{
				var joint = world.CreateJoint(jointDef);
				joints.push(joint);
			}
		}

		for (i = 0; i < gears.length; ++i)
		{
			gears[i][0].joint1 = joints[gears[i][0].joint1];
			gears[i][0].joint2 = joints[gears[i][0].joint2];

			joint = world.CreateJoint(gears[i][0]);
			joints[gears[i][1]] = joint;
		}
	}
};