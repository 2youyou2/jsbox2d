/// Joints and fixtures are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
function b2DestructionListener()
{
}

b2DestructionListener.prototype =
{
	/// Called when any joint is about to be destroyed due
	/// to the destruction of one of its attached bodies.
	SayGoodbyeJoint: function(joint) { },

	/// Called when any fixture is about to be destroyed due
	/// to the destruction of its parent body.
	SayGoodbyeFixture: function(fixture) { }

//'#if @LIQUIDFUN';
	,
	/// Called when any particle group is about to be destroyed.
	SayGoodbyeParticleGroup: function(group) { },

	/// Called when a particle is about to be destroyed.
	/// The index can be used in conjunction with
	/// b2World::GetParticleUserDataBuffer() to determine which particle has
	/// been destroyed.
	SayGoodbyeParticle: function (index) { }
//'#endif';
};

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
function b2ContactFilter()
{
}

b2ContactFilter.prototype =
{
	/// Return true if contact calculations should be performed between these two shapes.
	/// @warning for performance reasons this is only called when the AABBs begin to overlap.
	ShouldCollide: function(fixtureA, fixtureB)
	{
		var filterA = fixtureA.GetFilterData();
		var filterB = fixtureB.GetFilterData();

		if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
		{
			return filterA.groupIndex > 0;
		}

		var collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
		return collide;
	}
};

/// Contact impulses for reporting. Impulses are used instead of forces because
/// sub-step forces may approach infinity for rigid body collisions. These
/// match up one-to-one with the contact points in b2Manifold.
function b2ContactImpulse()
{
	this.normalImpulses = new Array(b2_maxManifoldPoints);
	this.tangentImpulses = new Array(b2_maxManifoldPoints);
	this.count = 0;
}


/// Implement this class to get contact information. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
/// @warning You cannot create/destroy Box2D entities inside these callbacks.
function b2ContactListener()
{
}

b2ContactListener.prototype =
{
	/// Called when two fixtures begin to touch.
	BeginContact: function(contact) { },

	/// Called when two fixtures cease to touch.
	EndContact: function(contact) { },

	/// This is called after a contact is updated. This allows you to inspect a
	/// contact before it goes to the solver. If you are careful, you can modify the
	/// contact manifold (e.g. disable contact).
	/// A copy of the old manifold is provided so that you can detect changes.
	/// Note: this is called only for awake bodies.
	/// Note: this is called even when the number of contact points is zero.
	/// Note: this is not called for sensors.
	/// Note: if you set the number of contact points to zero, you will not
	/// get an EndContact callback. However, you may get a BeginContact callback
	/// the next step.
	PreSolve: function(contact, oldManifold)
	{
	},

	/// This lets you inspect a contact after the solver is finished. This is useful
	/// for inspecting impulses.
	/// Note: the contact manifold does not include time of impact impulses, which can be
	/// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
	/// in a separate data structure.
	/// Note: this is only called for contacts that are touching, solid, and awake.
	PostSolve: function(contact, impulse)
	{
	}
};

/// Callback class for AABB queries.
/// See b2World::Query
function b2QueryCallback()
{
}

b2QueryCallback.prototype =
{
	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	ReportFixture: function(fixture) { return false; }

//'#if @LIQUIDFUN';
	,
	/// Called for each particle found in the query AABB.
	/// @return false to terminate the query.
	ReportParticle: function(index) { return false; }
//'#endif';
};

/// Callback class for ray casts.
/// See b2World::RayCast
function b2RayCastCallback()
{
}

b2RayCastCallback.prototype =
{
	/// Called for each fixture found in the query. You control how the ray cast
	/// proceeds by returning a float:
	/// return -1: ignore this fixture and continue
	/// return 0: terminate the ray cast
	/// return fraction: clip the ray to this point
	/// return 1: don't clip the ray and continue
	/// @param fixture the fixture hit by the ray
	/// @param point the point of initial intersection
	/// @param normal the normal vector at the point of intersection
	/// @return -1 to filter, 0 to terminate, fraction to clip the ray for
	/// closest hit, 1 to continue
	ReportFixture: function(fixture, point, normal, fraction) { }

//'#if @LIQUIDFUN';
	,
	/// Called for each particle found in the query.
	ReportParticle: function(index, point, normal, fraction) { return 0; }
//'#endif';
};