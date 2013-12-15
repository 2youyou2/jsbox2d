// RUBE loader for JSBox2D.
// Unlike built-in serialization, the RUBE method is all contained in this file.

var b2RUBELoader = (function()
{
	function parseVector(obj)
	{
		return new b2Vec2(obj ? (obj.x || 0) : 0, obj ? (obj.y || 0) : 0);
	}

	function parseVectorArray(obj)
	{
		var vals = new Array(obj.x.length);

		for (var i = 0; i < vals.length; ++i)
			vals[i] = new b2Vec2(obj.x[i], obj.y[i]);

		return vals;
	}

	function parseProperty(obj, instance)
	{
		var name = obj.name;
		var val;

		if (typeof(obj['int']) !== 'undefined')
			val = obj['int'];
		else if (typeof(obj['float']) !== 'undefined')
			val = obj['float'];
		else if (typeof(obj['string']) !== 'undefined')
			val = obj['string'];
		else if (typeof(obj['bool']) !== 'undefined')
			val = obj['bool'];
		else if (typeof(obj.vec2) !== 'undefined')
			val = parseVector(obj.vec2);
		else
			throw new Error("unknown property type");

		if (instance.hasOwnProperty(name))
			throw new Error("custom property possibly overwriting an existing one");

		instance[name] = val;
	}

	function parseFixture(obj, body)
	{
		var def = new b2FixtureDef();

		def.density = obj.density || 0;
		def.filter.categoryBits = typeof(obj['filter-categoryBits']) === 'undefined' ? 1 : obj['filter-categoryBits'];
		def.filter.maskBits = typeof(obj['filter-maskBits']) === 'undefined' ? 65535 : obj['filter-maskBits'];
		def.filter.groupIndex = typeof(obj['filter-groupIndex']) === 'undefined' ? 0 : obj['filter-groupIndex'];
		def.friction = obj.friction || 0;
		def.restitution = obj.restitution || 0;
		def.isSensor = obj.sensor || 0;

		var shape;

		if (typeof(obj.circle) !== 'undefined')
		{
			shape = new b2CircleShape();
			shape.m_p = parseVector(obj.circle.center);
			shape.m_radius = obj.circle.radius || 0;
		}
		else if (typeof(obj.polygon) !== 'undefined')
		{
			var vertices = parseVectorArray(obj.polygon.vertices);

			shape = new b2PolygonShape();
			shape.Set(vertices, vertices.length);
		}
		else if (typeof(obj.chain) !== 'undefined')
		{
			var vertices = parseVectorArray(obj.chain.vertices);

			shape = new b2ChainShape();
			shape.m_count = vertices.length;
			shape.m_vertices = vertices;
			if (shape.m_hasNextVertex = obj.chain.hasNextVertex)
				shape.m_nextVertex = parseVector(obj.chain.nextVertex);
			if (shape.m_hasPrevVertex = obj.chain.hasPrevVertex)
				shape.m_prevVertex = parseVector(obj.chain.prevVertex);
		}
		else
			throw new Error("unknown shape type");

		def.shape = shape;

		var fixture = body.CreateFixture(def);

		fixture.name = obj.name; // TODO better place? good idea?

		if (obj.customProperties)
			for (var i = 0; i < obj.customProperties.length; ++i)
				parseProperty(obj, fixture);
	}

	function parseBody(obj, world)
	{
		var def = new b2BodyDef();

		def.type = obj.type || b2Body.b2_staticBody;
		def.angle = obj.angle || 0;
		def.angularDamping = obj.angularDamping || 0;
		def.angularVelocity = obj.angularVelocity || 0;
		def.awake = obj.awake || false;
		def.bullet = obj.bullet || false;
		def.fixedRotation = obj.fixedRotation || false;
		def.linearDamping = obj.linearDamping || false;
		def.linearVelocity = parseVector(obj.linearVelocity);

		var md = new b2MassData();
		md.mass = obj['massData-mass'] || 0;
		md.center = parseVector(obj['massData-center']);
		md.I = obj['massData-I'] || 0;

		def.position = parseVector(obj.position);

		var body = world.CreateBody(def);
		body.name = obj.name; // TODO better place? good idea?
		body.SetMassData(md);

		if (obj.fixture)
			for (var i = 0; i < obj.fixture.length; ++i)
				parseFixture(obj.fixture[i], body);

		if (obj.customProperties)
			for (i = 0; i < obj.customProperties.length; ++i)
				parseProperty(obj, body);

		return body;
	}

	var jointsList = {
		'revolute': b2RevoluteJointDef,
		'distance': b2DistanceJointDef,
		'prismatic': b2PrismaticJointDef,
		'wheel': b2WheelJointDef,
		'rope': b2RopeJointDef,
		'motor': b2MotorJointDef,
		'weld': b2WeldJointDef,
		'friction': b2FrictionJointDef
	};

	function parseJoint(obj, world, bodies)
	{
		if (!jointsList[obj.type])
			throw new Error("unknown joint type");

		var jd = new jointsList[obj.type]();
		jd.bodyA = bodies[obj.bodyA || 0];
		jd.bodyB = bodies[obj.bodyB || 0];
		jd.collideConnected = obj.collideConnected || false;

		switch (jd.type)
		{
			case b2Joint.e_revoluteJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.enableLimit = obj.enableLimit || false;
				jd.enableMotor = obj.enableMotor || false;
				//jd.jointSpeed = obj.jointSpeed || 0; // what prop is this..?
				jd.lowerAngle = obj.lowerLimit || 0;
				jd.maxMotorTorque = obj.maxMotorTorque || 0;
				jd.motorSpeed = obj.motorSpeed || 0;
				jd.referenceAngle = obj.refAngle || 0;
				jd.upperAngle = obj.upperLimit || 0;
				break;
			case b2Joint.e_distanceJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.dampingRatio = obj.dampingRatio || 0;
				jd.frequencyHz = obj.frequency || 0;
				jd.length = obj.length || 0;
				break;
			case b2Joint.e_prismaticJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.enableLimit = obj.enableLimit || false;
				jd.enableMotor = obj.enableMotor || false;
				jd.localAxisA = parseVector(obj.localAxisA);
				jd.lowerTranslation = obj.lowerLimit || 0;
				jd.maxMotorForce = obj.maxMotorForce || 0;
				jd.motorSpeed = obj.motorSpeed || 0;
				jd.referenceAngle = obj.refAngle || 0;
				jd.upperTranslation = obj.upperLimit || 0;
				break;
			case b2Joint.e_wheelJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.enableMotor = obj.enableMotor || false;
				jd.localAxisA = parseVector(obj.localAxisA);
				jd.maxMotorTorque = obj.maxMotorTorque || 0;
				jd.motorSpeed = obj.motorSpeed || 0;
				jd.dampingRatio = obj.springDampingRatio || 0;
				jd.frequencyHz = obj.springFrequency || 0;
				break;
			case b2Joint.e_ropeJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.maxLength = obj.maxLength || 0;
				break;
			case b2Joint.e_motorJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.maxForce = obj.maxForce || 0;
				jd.maxTorque = obj.maxTorque || 0;
				jd.correctionFactor = obj.correctionFactor || 0;
				break;
			case b2Joint.e_weldJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.referenceAngle = obj.refAngle || 0;
				jd.dampingRatio = obj.dampingRatio || 0;
				jd.frequencyHz = obj.frequencyHz || 0;
				break;
			case b2Joint.e_frictionJoint:
				jd.localAnchorA = parseVector(obj.anchorA);
				jd.localAnchorB = parseVector(obj.anchorB);
				jd.maxForce = obj.refAngle || 0;
				jd.maxTorque = obj.dampingRatio || 0;
				break;
			default:
				throw new Error("wat?");
		}

		var joint = world.CreateJoint(jd);

		joint.name = obj.name; // TODO better place? good idea?

		if (obj.customProperties)
			for (var i = 0; i < obj.customProperties.length; ++i)
				parseProperty(obj, joint);

		return joint;
	}

	function b2RubeParameters()
	{
		this.world = null;
		this.positionIterations = 0;
		this.velocityIterations = 0;
		this.stepsPerSecond = 0;
		this.fixtures = [];
		this.bodies = [];
		this.joints = [];

		Object.seal(this);
	}

	function parseWorld(obj, world)
	{
		var params = new b2RubeParameters();

		params.world = world = world || new b2World(new b2Vec2(0, 0));

		params.positionIterations = obj.positionIterations || 0;
		params.velocityIterations = obj.velocityIterations || 0;
		params.stepsPerSecond = obj.stepsPerSecond || 0;

		if (obj.gravity)
			world.SetGravity(parseVector(obj.gravity));

		world.SetAllowSleeping(obj.allowSleep || false);
		world.SetAutoClearForces(obj.autoClearForces || false);
		world.SetWarmStarting(obj.warmStarting || false);
		world.SetContinuousPhysics(obj.continuousPhysics || false);
		world.SetSubStepping(obj.subStepping || false);

		var bodies = [];
		var bl = obj.body;

		if (bl)
		{
			for (var i = 0; i < bl.length; ++i)
			{
				var body = parseBody(bl[i], world);
				bodies.push(body);

				for (var f = body.GetFixtureList(); f; f = f.GetNext())
				{
					if (!params.fixtures[f.name])
						params.fixtures[f.name] = [];

					params.fixtures[f.name].push(f);
				}

				if (!params.bodies[body.name])
					params.bodies[body.name] = [];

				params.bodies[body.name].push(body);
			}
		}

		var joints = [];
		var jl = obj.joint;

		if (jl)
		{
			for (i = 0; i < jl.length; ++i)
			{
				var joint = parseJoint(jl[i], world, bodies);
				joints.push(joint);

				if (!params.joints[joint.name])
					params.joints[joint.name] = [];

				params.joints[joint.name].push(joint);
			}
		}

		return params;
	}

	return {
		parseWorld: parseWorld
	};
})();