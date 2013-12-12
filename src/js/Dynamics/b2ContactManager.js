"use strict";

var b2_defaultFilter = new b2ContactFilter();
var b2_defaultListener = new b2ContactListener();

// Delegate of b2World.
function b2ContactManager()
{
	this.m_broadPhase = new b2BroadPhase();
	this.m_contactList = null;
	this.m_contactCount = 0;
	this.m_contactFilter = b2_defaultFilter;
	this.m_contactListener = b2_defaultListener;
}

b2ContactManager.prototype =
{
	// Broad-phase callback.
	AddPair: function(proxyUserDataA, proxyUserDataB)
	{
		var proxyA = proxyUserDataA;
		var proxyB = proxyUserDataB;

		var fixtureA = proxyA.fixture;
		var fixtureB = proxyB.fixture;

		var indexA = proxyA.childIndex;
		var indexB = proxyB.childIndex;

		var bodyA = fixtureA.GetBody();
		var bodyB = fixtureB.GetBody();

		// Are the fixtures on the same body?
		if (bodyA == bodyB)
		{
			return;
		}

		// TODO_ERIN use a hash table to remove a potential bottleneck when both
		// bodies have a lot of contacts.
		// Does a contact already exist?
		var edge = bodyB.GetContactList();
		while (edge)
		{
			if (edge.other == bodyA)
			{
				var fA = edge.contact.GetFixtureA();
				var fB = edge.contact.GetFixtureB();
				var iA = edge.contact.GetChildIndexA();
				var iB = edge.contact.GetChildIndexB();

				if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
				{
					// A contact already exists.
					return;
				}

				if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
				{
					// A contact already exists.
					return;
				}
			}

			edge = edge.next;
		}

		// Does a joint override collision? Is at least one body dynamic?
		if (bodyB.ShouldCollide(bodyA) == false)
		{
			return;
		}

		// Check user filtering.
		if (this.m_contactFilter && this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
		{
			return;
		}

		// Call the factory.
		var c = b2Contact.Create(fixtureA, indexA, fixtureB, indexB);
		if (c == null)
		{
			return;
		}

		// Contact creation may swap fixtures.
		fixtureA = c.GetFixtureA();
		fixtureB = c.GetFixtureB();
		indexA = c.GetChildIndexA();
		indexB = c.GetChildIndexB();
		bodyA = fixtureA.GetBody();
		bodyB = fixtureB.GetBody();

		// Insert into the world.
		c.m_prev = null;
		c.m_next = this.m_contactList;
		if (this.m_contactList != null)
		{
			this.m_contactList.m_prev = c;
		}
		this.m_contactList = c;

		// Connect to island graph.

		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;

		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null)
		{
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;

		// Connect to body B
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;

		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null)
		{
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;

		// Wake up the bodies
		if (fixtureA.IsSensor() == false && fixtureB.IsSensor() == false)
		{
			bodyA.SetAwake(true);
			bodyB.SetAwake(true);
		}

		++this.m_contactCount;
	},

	FindNewContacts: function()
	{
		this.m_broadPhase.UpdatePairs(this);
	},

	Destroy: function(c)
	{
		var fixtureA = c.GetFixtureA();
		var fixtureB = c.GetFixtureB();
		var bodyA = fixtureA.GetBody();
		var bodyB = fixtureB.GetBody();

		if (this.m_contactListener && c.IsTouching())
		{
			this.m_contactListener.EndContact(c);
		}

		// Remove from the world.
		if (c.m_prev)
		{
			c.m_prev.m_next = c.m_next;
		}

		if (c.m_next)
		{
			c.m_next.m_prev = c.m_prev;
		}

		if (c == this.m_contactList)
		{
			this.m_contactList = c.m_next;
		}

		// Remove from body 1
		if (c.m_nodeA.prev)
		{
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}

		if (c.m_nodeA.next)
		{
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}

		if (c.m_nodeA == bodyA.m_contactList)
		{
			bodyA.m_contactList = c.m_nodeA.next;
		}

		// Remove from body 2
		if (c.m_nodeB.prev)
		{
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}

		if (c.m_nodeB.next)
		{
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}

		if (c.m_nodeB == bodyB.m_contactList)
		{
			bodyB.m_contactList = c.m_nodeB.next;
		}

		// Call the factory.
		b2Contact.Destroy(c);
		--this.m_contactCount;
	},

	Collide: function()
	{
		// Update awake contacts.
		var c = this.m_contactList;
		while (c)
		{
			var fixtureA = c.GetFixtureA();
			var fixtureB = c.GetFixtureB();
			var indexA = c.GetChildIndexA();
			var indexB = c.GetChildIndexB();
			var bodyA = fixtureA.GetBody();
			var bodyB = fixtureB.GetBody();

			// Is this contact flagged for filtering?
			if (c.m_flags & b2Contact.e_filterFlag)
			{
				// Should these bodies collide?
				if (bodyB.ShouldCollide(bodyA) == false)
				{
					var cNuke = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}

				// Check user filtering.
				if (this.m_contactFilter && this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
				{
					var cNuke = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}

				// Clear the filtering flag.
				c.m_flags &= ~b2Contact.e_filterFlag;
			}

			var activeA = bodyA.IsAwake() && bodyA.m_type != b2Body.b2_staticBody;
			var activeB = bodyB.IsAwake() && bodyB.m_type != b2Body.b2_staticBody;

			// At least one body must be awake and it must be dynamic or kinematic.
			if (activeA == false && activeB == false)
			{
				c = c.GetNext();
				continue;
			}

			var proxyIdA = fixtureA.m_proxies[indexA].proxyId;
			var proxyIdB = fixtureB.m_proxies[indexB].proxyId;
			var overlap = this.m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (overlap == false)
			{
				var cNuke = c;
				c = cNuke.GetNext();
				this.Destroy(cNuke);
				continue;
			}

			// The contact persists.
			c.Update(this.m_contactListener);
			c = c.GetNext();
		}
	}
};
