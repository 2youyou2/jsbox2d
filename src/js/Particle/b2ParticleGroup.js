/// A particle group definition holds all the data needed to construct a particle group.
/// You can safely re-use these definitions.
function b2ParticleGroupDef()
{
	/// The particle-behavior flags.
	this.flags = 0;

	/// The group-construction flags.
	this.groupFlags = 0;

	/// The world position of the group.
	/// Moves the group's shape a distance equal to the value of position.
	this.position = new b2Vec2();

	/// The world angle of the group in radians.
	/// Rotates the shape by an angle equal to the value of angle.
	this.angle = 0;

	/// The linear velocity of the group's origin in world co-ordinates.
	this.linearVelocity = new b2Vec2();

	/// The angular velocity of the group.
	this.angularVelocity = 0;

	/// The color of all particles in the group.
	this.color = new b2ParticleColor();

	/// The strength of cohesion among the particles in a group with flag b2_elasticParticle or b2_springParticle.
	this.strength = 1;

	/// Shape containing the particle group.
	this.shape = null;

	/// If true, destroy the group automatically after its last particle has been destroyed.
	this.destroyAutomatically = true;

	/// Use this to store application-specific group data.
	this.userData = null;
}

/// A group of particles. These are created via b2World::CreateParticleGroup.
function b2ParticleGroup()
{
	this.m_system = null;
	this.m_firstIndex = 0;
	this.m_lastIndex = 0;
	this.m_groupFlags = 0;
	this.m_strength = 1.0;
	this.m_prev = null;
	this.m_next = null;

	this.m_timestamp = -1;
	this.m_mass = 0;
	this.m_inertia = 0;
	this.m_center = new b2Vec2();
	this.m_linearVelocity = new b2Vec2();
	this.m_angularVelocity = 0;
	this.m_transform = new b2Transform();
	this.m_transform.SetIdentity();

	this.m_destroyAutomatically = true;
	this.m_toBeDestroyed = false;
	this.m_toBeSplit = false;

	this.m_userData = null;
}

b2ParticleGroup.prototype =
{
	/// Get the next particle group from the list in b2_World.
	GetNext: function() { return this.m_next; },

	/// Get the number of particles.
	GetParticleCount: function()
	{
		return this.m_lastIndex - this.m_firstIndex;
	},

	/// Get the offset of this group in the global particle buffer
	GetBufferIndex: function()
	{
		return this.m_firstIndex;
	},

	/// Get the construction flags for the group.
	GetGroupFlags: function()
	{
		return this.m_groupFlags;
	},

	/// Set the construction flags for the group.
	SetGroupFlags: function(flags)
	{
		this.m_groupFlags = flags;
	},

	/// Get the total mass of the group: the sum of all particles in it.
	GetMass: function()
	{
		this.UpdateStatistics();
		return this.m_mass;
	},

	/// Get the moment of inertia for the group.
	GetInertia: function()
	{
		this.UpdateStatistics();
		return this.m_inertia;
	},

	/// Get the center of gravity for the group.
	GetCenter: function()
	{
		this.UpdateStatistics();
		return this.m_center;
	},

	/// Get the linear velocity of the group.
	GetLinearVelocity: function()
	{
		this.UpdateStatistics();
		return this.m_linearVelocity;
	},

	/// Get the angular velocity of the group.
	GetAngularVelocity: function()
	{
		this.UpdateStatistics();
		return this.m_angularVelocity;
	},

	/// Get the position of the group's origin and rotation.
	/// Used only with groups of rigid particles.
	GetTransform: function()
	{
		return this.m_transform;
	},

	/// Get position of the particle group as a whole.
	/// Used only with groups of rigid particles.
	GetPosition: function()
	{
		return this.m_transform.p;
	},

	/// Get the rotational angle of the particle group as a whole.
	/// Used only with groups of rigid particles.
	GetAngle: function()
	{
		return this.m_transform.q.GetAngle();
	},

	/// Get the user data pointer that was provided in the group definition.
	GetUserData: function()
	{
		return this.m_userData;
	},

	/// Set the user data. Use this to store your application specific data.
	SetUserData: function(data)
	{
		this.m_userData = data;
	},

	UpdateStatistics: function()
	{
		if (this.m_timestamp != this.m_system.m_timestamp)
		{
			var m = this.m_system.GetParticleMass();
			this.m_mass = 0;
			this.m_center.SetZero();
			this.m_linearVelocity.SetZero();
			for (var i = this.m_firstIndex; i < this.m_lastIndex; i++)
			{
				this.m_mass += m;
				this.m_center.Add(b2Vec2.Multiply(m, this.m_system.m_positionBuffer.data[i]));
				this.m_linearVelocity.Add(b2Vec2.Multiply(m, this.m_system.m_velocityBuffer.data[i]));
			}
			if (this.m_mass > 0)
			{
				this.m_center.Multiply(1 / this.m_mass);
				this.m_linearVelocity.Multiply(1 / this.m_mass);
			}
			this.m_inertia = 0;
			this.m_angularVelocity = 0;
			for (var i = this.m_firstIndex; i < this.m_lastIndex; i++)
			{
				var p = b2Vec2.Subtract(this.m_system.m_positionBuffer.data[i], this.m_center);
				var v = b2Vec2.Subtract(this.m_system.m_velocityBuffer.data[i], this.m_linearVelocity);
				this.m_inertia += m * b2Dot_v2_v2(p, p);
				this.m_angularVelocity += m * b2Cross_v2_v2(p, v);
			}
			if (this.m_inertia > 0)
			{
				this.m_angularVelocity *= 1 / this.m_inertia;
			}
			this.m_timestamp = this.m_system.m_timestamp;
		}
	}
};

b2ParticleGroup.b2_solidParticleGroup =   1 << 0; // resists penetration
b2ParticleGroup.b2_rigidParticleGroup =   1 << 1; // keeps its shape
