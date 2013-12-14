"use strict";

/// This holds contact filtering data.
function b2Filter()
{
	/// The collision category bits. Normally you would just set one bit.
	this.categoryBits = 0x0001;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	this.maskBits = 0xFFFF;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	this.groupIndex = 0;
}

b2Filter.prototype =
{
	Clone: function()
	{
		var filter = new b2Filter();
		filter.categoryBits = this.categoryBits;
		filter.maskBits = this.maskBits;
		filter.groupIndex = this.groupIndex;
		return filter;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		obj['categoryBits'] = this.categoryBits;
		obj['maskBits'] = this.maskBits;
		obj['groupIndex'] = this.groupIndex;

		return obj;
	},

	_deserialize: function(data)
	{
		this.categoryBits = data['categoryBits'];
		this.maskBits = data['maskBits'];
		this.groupIndex = data['groupIndex'];
	}
};

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
function b2FixtureDef()
{
	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	this.shape = null;

	/// Use this to store application specific fixture data.
	this.userData = null;

	/// The friction coefficient, usually in the range [0,1].
	this.friction = 0.2;

	/// The restitution (elasticity) usually in the range [0,1].
	this.restitution = 0.0;

	/// The density, usually in kg/m^2.
	this.density = 0.0;

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	this.isSensor = false;

	/// Contact filtering data.
	this.filter = new b2Filter();
}

b2FixtureDef.prototype =
{
	_deserialize: function(data)
	{
		this.friction = data['friction'];
		this.restitution = data['restitution'];
		this.density = data['density'];
		this.isSensor = data['isSensor'];
		this.filter._deserialize(data['filter']);
	}
};

/// This proxy is used internally to connect fixtures to the broad-phase.
function b2FixtureProxy()
{
	this.aabb = new b2AABB();
	this.fixture = null;
	this.childIndex = 0;
	this.proxyId = 0;
};

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
function b2Fixture()
{
	this.m_userData = null;
	this.m_body = null;
	this.m_next = null;
	this.m_proxies = null;
	this.m_proxyCount = 0;
	this.m_shape = null;
	this.m_density = 0.0;
	this.m_filter = new b2Filter();
	this.m_isSensor = false;
	this.m_friction = 0;
	this.m_restitution = 0;
}

b2Fixture.prototype =
{
	/// Get the type of the child shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	GetType: function()
	{
		return this.m_shape.GetType();
	},

	/// Get the child shape. You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	GetShape: function()
	{
		return this.m_shape;
	},

	/// Set if this fixture is a sensor.
	SetSensor: function(sensor)
	{
		if (sensor != this.m_isSensor)
		{
			this.m_body.SetAwake(true);
			this.m_isSensor = sensor;
		}
	},

	/// Is this fixture a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	IsSensor: function()
	{
		return this.m_isSensor;
	},

	/// Set the contact filtering data. This will not update contacts until the next time
	/// step when either parent body is active and awake.
	/// This automatically calls Refilter.
	SetFilterData: function(filter)
	{
		this.m_filter = filter;

		this.Refilter();
	},

	/// Get the contact filtering data.
	GetFilterData: function()
	{
		return this.m_filter;
	},

	/// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
	Refilter: function()
	{
		if (this.m_body == null)
		{
			return;
		}

		// Flag associated contacts for filtering.
		var edge = this.m_body.GetContactList();
		while (edge)
		{
			var contact = edge.contact;
			var fixtureA = contact.GetFixtureA();
			var fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
			{
				contact.FlagForFiltering();
			}

			edge = edge.next;
		}

		var world = this.m_body.GetWorld();

		if (world == null)
		{
			return;
		}

		// Touch each proxy so that new pairs may be created
		var broadPhase = world.m_contactManager.m_broadPhase;
		for (var i = 0; i < this.m_proxyCount; ++i)
		{
			broadPhase.TouchProxy(this.m_proxies[i].proxyId);
		}
	},

	/// Get the parent body of this fixture. This is null if the fixture is not attached.
	/// @return the parent body.
	GetBody: function()
	{
		return this.m_body;
	},

	/// Get the next fixture in the parent body's fixture list.
	/// @return the next shape.
	GetNext: function()
	{
		return this.m_next;
	},

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	GetUserData: function()
	{
		return this.m_userData;
	},

	/// Set the user data. Use this to store your application specific data.
	SetUserData: function(data)
	{
		this.m_userData = data;
	},

	/// Test a point for containment in this fixture.
	/// @param p a point in world coordinates.
	TestPoint: function(p)
	{
		return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
	},

	/// Cast a ray against this shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	RayCast: function(output, input, childIndex)
	{
		return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
	},

	/// Get the mass data for this fixture. The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	GetMassData: function(massData)
	{
		this.m_shape.ComputeMass(massData, this.m_density);
	},

	/// Set the density of this fixture. This will _not_ automatically adjust the mass
	/// of the body. You must call b2Body::ResetMassData to update the body's mass.
	SetDensity: function(density)
	{
		b2Assert(b2IsValid(density) && density >= 0.0);
		this.m_density = density;
	},

	/// Get the density of this fixture.
	GetDensity: function()
	{
		return this.m_density;
	},

	/// Get the coefficient of friction.
	GetFriction: function()
	{
		return this.m_friction;
	},

	/// Set the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	SetFriction: function(friction)
	{
		this.m_friction = friction;
	},

	/// Get the coefficient of restitution.
	GetRestitution: function()
	{
		return this.m_restitution;
	},

	/// Set the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	SetRestitution: function(restitution)
	{
		this.m_restitution = restitution;
	},

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	GetAABB: function(childIndex)
	{
		b2Assert(0 <= childIndex && childIndex < this.m_proxyCount);
		return this.m_proxies[childIndex].aabb;
	},

	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by C++).
	Create: function(body, def)
	{
		this.m_userData = def.userData;
		this.m_friction = def.friction;
		this.m_restitution = def.restitution;

		this.m_body = body;
		this.m_next = null;

		this.m_filter = def.filter.Clone();

		this.m_isSensor = def.isSensor;

		this.m_shape = def.shape.Clone();

		// Reserve proxy space
		var childCount = this.m_shape.GetChildCount();
		this.m_proxies = new Array(childCount);
		for (var i = 0; i < childCount; ++i)
		{
			this.m_proxies[i] = new b2FixtureProxy();
			this.m_proxies[i].fixture = null;
			this.m_proxies[i].proxyId = b2BroadPhase.e_nullProxy;
		}
		this.m_proxyCount = 0;

		this.m_density = def.density;
	},
	Destroy: function()
	{
		// The proxies must be destroyed before calling this.
		b2Assert(this.m_proxyCount == 0);

		// Free the proxy array.
		this.m_proxies = null;
		this.m_shape = null;
	},

	// These support body activation/deactivation.
	CreateProxies: function(broadPhase, xf)
	{
		b2Assert(this.m_proxyCount == 0);

		// Create proxies in the broad-phase.
		this.m_proxyCount = this.m_shape.GetChildCount();

		for (var i = 0; i < this.m_proxyCount; ++i)
		{
			var proxy = this.m_proxies[i];
			this.m_shape.ComputeAABB(proxy.aabb, xf, i);
			proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, proxy);
			proxy.fixture = this;
			proxy.childIndex = i;
		}
	},
	DestroyProxies: function(broadPhase)
	{
		// Destroy proxies in the broad-phase.
		for (var i = 0; i < this.m_proxyCount; ++i)
		{
			var proxy = this.m_proxies[i];
			broadPhase.DestroyProxy(proxy.proxyId);
			proxy.proxyId = b2BroadPhase.e_nullProxy;
		}

		this.m_proxyCount = 0;
	},

	Synchronize: function(broadPhase, transform1, transform2)
	{
		if (this.m_proxyCount == 0)
		{
			return;
		}

		for (var i = 0; i < this.m_proxyCount; ++i)
		{
			var proxy = this.m_proxies[i];

			// Compute an AABB that covers the swept shape (may miss some rotation effect).
			var aabb1 = new b2AABB(), aabb2 = new b2AABB();
			this.m_shape.ComputeAABB(aabb1, transform1, proxy.childIndex);
			this.m_shape.ComputeAABB(aabb2, transform2, proxy.childIndex);

			proxy.aabb.Combine(aabb1, aabb2);

			var displacement = b2Vec2.Subtract(transform2.p, transform1.p);

			broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
		}
	},

	_serialize: function(out)
	{
		var obj = out || {};

		// this will be filled in later by the serializer
		obj['shape'] = null;
		obj['friction'] = this.m_friction;
		obj['restitution'] = this.m_restitution;
		obj['density'] = this.m_density;
		obj['isSensor'] = this.m_isSensor;
		obj['filter'] = this.m_filter._serialize();

		return obj;
	}
};
