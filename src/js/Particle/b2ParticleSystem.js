function b2ParticleContact()
{
	///Indices of the respective particles making contact.
	///
	this.indexA = this.indexB = 0;
	///The logical sum of the particle behaviors that have been set.
	///
	this.flags = 0;
	/// Weight of the contact. A value between 0.0f and 1.0f.
	///
	this.weight = 0.0;
	///The normalized direction from A to B.
	///
	this.normal = new b2Vec2();
};

function b2ParticleBodyContact()
{
	/// Index of the particle making contact.
	///
	this.index = 0;
	/// The body making contact.
	///
	this.body = null;
	///Weight of the contact. A value between 0.0f and 1.0f.
	///
	this.weight = 0.0;
	/// The normalized direction from the particle to the body.
	///
	this.normal = new b2Vec2();
	/// The effective mass used in calculating force.
	///
	this.mass = 0.0;
};

function b2ParticleSystem()
{
	this.m_timestamp = 0;
	this.m_allParticleFlags = 0;
	this.m_allGroupFlags = 0;
	this.m_density = 1;
	this.m_inverseDensity = 1;
	this.m_gravityScale = 1;
	this.m_particleDiameter = 1;
	this.m_inverseDiameter = 1;
	this.m_squaredDiameter = 1;

	this.m_count = 0;
	this.m_internalAllocatedCapacity = 0;
	this.m_maxCount = 0;
	this.m_flagsBuffer = new b2ParticleSystem.ParticleBuffer();
	this.m_positionBuffer = new b2ParticleSystem.ParticleBuffer();
	this.m_velocityBuffer = new b2ParticleSystem.ParticleBuffer();
	this.m_accumulationBuffer = null;
	this.m_accumulation2Buffer = null;
	this.m_depthBuffer = null;
	this.m_colorBuffer = new b2ParticleSystem.ParticleBuffer();
	this.m_groupBuffer = null;
	this.m_userDataBuffer = new b2ParticleSystem.ParticleBuffer();

	this.m_proxyCount = 0;
	this.m_proxyCapacity = 0;
	this.m_proxyBuffer = null;

	this.m_contactCount = 0;
	this.m_contactCapacity = 0;
	this.m_contactBuffer = null;

	this.m_bodyContactCount = 0;
	this.m_bodyContactCapacity = 0;
	this.m_bodyContactBuffer = null;

	this.m_pairCount = 0;
	this.m_pairCapacity = 0;
	this.m_pairBuffer = null;

	this.m_triadCount = 0;
	this.m_triadCapacity = 0;
	this.m_triadBuffer = null;

	this.m_groupCount = 0;
	this.m_groupList = null;

	this.m_pressureStrength = 0.05;
	this.m_dampingStrength = 1.0;
	this.m_elasticStrength = 0.25;
	this.m_springStrength = 0.25;
	this.m_viscousStrength = 0.25;
	this.m_surfaceTensionStrengthA = 0.1;
	this.m_surfaceTensionStrengthB = 0.2;
	this.m_powderStrength = 0.5;
	this.m_ejectionStrength = 0.5;
	this.m_colorMixingStrength = 0.5;

	this.m_world = null;
}

b2ParticleSystem.ParticleBuffer = function()
{
	this.data = null;
	this.userSuppliedCapacity = 0;
};

/// Used for detecting particle contacts
b2ParticleSystem.Proxy = function()
{
	this.index = 0;
	this.tag = 0;
};

b2ParticleSystem.Proxy.LessThan_p_p = function(a, b)
{
	return a.tag < b.tag;
};

b2ParticleSystem.Proxy.LessThan_i_p = function(a, b)
{
	return a < b.tag;
};

b2ParticleSystem.Proxy.LessThan_p_i = function(a, b)
{
	return a.tag < b;
};

/// Connection between two particles
b2ParticleSystem.Pair = function()
{
	this.indexA = this.indexB = 0;
	this.flags = 0;
	this.strength = 0.0;
	this.distance = 0.0;
};

/// Connection between three particles
b2ParticleSystem.Triad = function()
{
	this.indexA = this.indexB = this.indexC = 0;
	this.flags = 0;
	this.strength = 0.0;
	this.pa = new b2Vec2(), this.pb = new b2Vec2(), this.pc = new b2Vec2();
	this.ka = 0.0, this.kb = 0.0, this.kc = 0.0, this.s = 0.0;
};

/// All particle types that require creating pairs
b2ParticleSystem.k_pairFlags = b2ParticleDef.b2_springParticle;
/// All particle types that require creating triads
b2ParticleSystem.k_triadFlags = b2ParticleDef.b2_elasticParticle;
/// All particle types that require computing depth
b2ParticleSystem.k_noPressureFlags = b2ParticleDef.b2_powderParticle;

b2ParticleSystem.xTruncBits = 12;
b2ParticleSystem.yTruncBits = 12;
b2ParticleSystem.tagBits = 8 * 4/*sizeof(uint32)*/;
b2ParticleSystem.yOffset = 1 << (b2ParticleSystem.yTruncBits - 1);
b2ParticleSystem.yShift = b2ParticleSystem.tagBits - b2ParticleSystem.yTruncBits;
b2ParticleSystem.xShift = b2ParticleSystem.tagBits - b2ParticleSystem.yTruncBits - b2ParticleSystem.xTruncBits;
b2ParticleSystem.xScale = 1 << b2ParticleSystem.xShift;
b2ParticleSystem.xOffset = b2ParticleSystem.xScale * (1 << (b2ParticleSystem.xTruncBits - 1));
b2ParticleSystem.xMask = (1 << b2ParticleSystem.xTruncBits) - 1;
b2ParticleSystem.yMask = (1 << b2ParticleSystem.yTruncBits) - 1;

function computeTag(x, y)
{
	return ((y + b2ParticleSystem.yOffset) << b2ParticleSystem.yShift) + (b2ParticleSystem.xScale * x + b2ParticleSystem.xOffset) >>> 0;
}

function computeRelativeTag(tag, x, y)
{
	return tag + (y << b2ParticleSystem.yShift) + (x << b2ParticleSystem.xShift);
}

function LimitCapacity(capacity, maxCount)
{
	return maxCount && capacity > maxCount ? maxCount : capacity;
}

function b2ParticleContactIsZombie(contact)
{
	return (contact.flags & b2ParticleDef.b2_zombieParticle) == b2ParticleDef.b2_zombieParticle;
}

b2ParticleSystem.prototype =
{
	/*// Callback used with b2VoronoiDiagram.
	class CreateParticleGroupCallback
	{
	public:
		void operator()(int32 a, int32 b, int32 c) const;
		b2ParticleSystem* system;
		const b2ParticleGroupDef* def;
		int32 firstIndex;
	};

	// Callback used with b2VoronoiDiagram.
	class JoinParticleGroupsCallback
	{
	public:
		void operator()(int32 a, int32 b, int32 c) const;
		b2ParticleSystem* system;
		b2ParticleGroup* groupA;
		b2ParticleGroup* groupB;
	};*/


	// Reallocate a buffer
	ReallocateBuffer3: function(oldBuffer, oldCapacity, newCapacity)
	{
'#if @DEBUG';
		b2Assert(newCapacity > oldCapacity);
'#endif';
		var newBuffer = (oldBuffer) ? oldBuffer.slice() : [];
		newBuffer.length = newCapacity;
		return newBuffer;
	},

	// Reallocate a buffer
	ReallocateBuffer5: function(buffer, userSuppliedCapacity, oldCapacity, newCapacity, deferred)
	{
'#if @DEBUG';
		b2Assert(newCapacity > oldCapacity);
		// A 'deferred' buffer is reallocated only if it is not null.
		// If 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
		b2Assert(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity);
'#endif';
		if ((!deferred || buffer) && !userSuppliedCapacity)
		{
			buffer = this.ReallocateBuffer3(buffer, oldCapacity, newCapacity);
		}
		return buffer;
	},

	// Reallocate a buffer
	ReallocateBuffer4: function(buffer, oldCapacity, newCapacity, deferred)
	{
'#if @DEBUG';
		b2Assert(newCapacity > oldCapacity);
'#endif';
		return this.ReallocateBuffer5(buffer.data, buffer.userSuppliedCapacity, oldCapacity, newCapacity, deferred);
	},

	RequestParticleBuffer: function(buffer)
	{
		if (!buffer)
		{
			buffer = new Array(this.m_internalAllocatedCapacity);
		}
		return buffer;
	},

	CreateParticle: function(def)
	{
		if (this.m_count >= this.m_internalAllocatedCapacity)
		{
			var capacity = this.m_count ? 2 * this.m_count : b2_minParticleBufferCapacity;
			capacity = LimitCapacity(capacity, this.m_maxCount);
			capacity = LimitCapacity(capacity, this.m_flagsBuffer.userSuppliedCapacity);
			capacity = LimitCapacity(capacity, this.m_positionBuffer.userSuppliedCapacity);
			capacity = LimitCapacity(capacity, this.m_velocityBuffer.userSuppliedCapacity);
			capacity = LimitCapacity(capacity, this.m_colorBuffer.userSuppliedCapacity);
			capacity = LimitCapacity(capacity, this.m_userDataBuffer.userSuppliedCapacity);
			if (this.m_internalAllocatedCapacity < capacity)
			{
				this.m_flagsBuffer.data = this.ReallocateBuffer4(this.m_flagsBuffer, this.m_internalAllocatedCapacity, capacity, false);
				this.m_positionBuffer.data = this.ReallocateBuffer4(this.m_positionBuffer, this.m_internalAllocatedCapacity, capacity, false);
				this.m_velocityBuffer.data = this.ReallocateBuffer4(this.m_velocityBuffer, this.m_internalAllocatedCapacity, capacity, false);
				this.m_accumulationBuffer = this.ReallocateBuffer5(this.m_accumulationBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
				this.m_accumulation2Buffer = this.ReallocateBuffer5(this.m_accumulation2Buffer, 0, this.m_internalAllocatedCapacity, capacity, true);
				this.m_depthBuffer = this.ReallocateBuffer5(this.m_depthBuffer, 0, this.m_internalAllocatedCapacity, capacity, true);
				this.m_colorBuffer.data = this.ReallocateBuffer4(this.m_colorBuffer, this.m_internalAllocatedCapacity, capacity, true);
				this.m_groupBuffer = this.ReallocateBuffer5(this.m_groupBuffer, 0, this.m_internalAllocatedCapacity, capacity, false);
				this.m_userDataBuffer.data = this.ReallocateBuffer4(this.m_userDataBuffer, this.m_internalAllocatedCapacity, capacity, true);
				this.m_internalAllocatedCapacity = capacity;
			}
		}
		if (this.m_count >= this.m_internalAllocatedCapacity)
		{
			return this.b2_invalidParticleIndex;
		}
		var index = this.m_count++;
		this.m_flagsBuffer.data[index] = def.flags;
		this.m_positionBuffer.data[index] = def.position.Clone();
		this.m_velocityBuffer.data[index] = def.velocity.Clone();
		this.m_groupBuffer[index] = null;
		if (this.m_depthBuffer)
		{
			this.m_depthBuffer[index] = 0;
		}
		if (this.m_colorBuffer.data || !def.color.IsZero())
		{
			this.m_colorBuffer.data = this.RequestParticleBuffer(this.m_colorBuffer.data);
			this.m_colorBuffer.data[index] = def.color.Clone();
		}
		if (this.m_userDataBuffer.data || def.userData)
		{
			this.m_userDataBuffer.data= this.RequestParticleBuffer(this.m_userDataBuffer.data);
			this.m_userDataBuffer.data[index] = def.userData;
		}
		if (this.m_proxyCount >= this.m_proxyCapacity)
		{
			var oldCapacity = this.m_proxyCapacity;
			var newCapacity = this.m_proxyCount ? 2 * this.m_proxyCount : b2_minParticleBufferCapacity;
			this.m_proxyBuffer = this.ReallocateBuffer3(this.m_proxyBuffer, oldCapacity, newCapacity);
			this.m_proxyCapacity = newCapacity;
		}
		this.m_proxyBuffer[this.m_proxyCount] = new b2ParticleSystem.Proxy();
		this.m_proxyBuffer[this.m_proxyCount++].index = index;
		return index;
	},

	DestroyParticle: function(index, callDestructionListener)
	{
		var flags = b2ParticleDef.b2_zombieParticle;
		if (callDestructionListener)
		{
			flags |= b2ParticleDef.b2_destructionListener;
		}
		this.m_flagsBuffer.data[index] |= flags;
	},

	DestroyParticlesInShape: function(shape, xf, callDestructionListener)
	{
		function DestroyParticlesInShapeCallback(system, shape, xf, callDestructionListener)
		{
			this.m_system = system;
			this.m_shape = shape;
			this.m_xf = xf;
			this.m_callDestructionListener = callDestructionListener;
			this.m_destroyed = 0;
		}

		DestroyParticlesInShapeCallback.prototype =
		{
			ReportFixture: function(fixture)
			{
				return false;
			},

			ReportParticle: function(index)
			{
'#if @DEBUG';
				b2Assert(index >=0 && index < this.m_system.m_count);
'#endif';
				if (this.m_shape.TestPoint(this.m_xf, this.m_system.m_positionBuffer.data[index]))
				{
					this.m_system.DestroyParticle(index, this.m_callDestructionListener);
					this.m_destroyed++;
				}
				return true;
			},

			Destroyed: function() { return this.m_destroyed; }
		};

		var callback = new DestroyParticlesInShapeCallback(this, shape, xf, callDestructionListener);

		var aabb = new b2AABB();
		shape.ComputeAABB(aabb, xf, 0);
		this.m_world.QueryAABB(callback, aabb);
		return callback.Destroyed();
	},

	DestroyParticlesInGroup: function(group, callDestructionListener)
	{
		for (var i = group.m_firstIndex; i < group.m_lastIndex; i++) {
			this.DestroyParticle(i, callDestructionListener);
		}
	},

	CreateParticleGroup: function(groupDef)
	{
		var stride = this.GetParticleStride();
		var identity = new b2Transform();
		identity.SetIdentity();
		var transform = identity.Clone();
		var firstIndex = this.m_count;
		if (groupDef.shape)
		{
			var particleDef = new b2ParticleDef();
			particleDef.flags = groupDef.flags;
			particleDef.color = groupDef.color;
			particleDef.userData = groupDef.userData;
			var shape = groupDef.shape;
			transform.Set(groupDef.position, groupDef.angle);
			var aabb = new b2AABB();
			var childCount = shape.GetChildCount();
			for (var childIndex = 0; childIndex < childCount; childIndex++)
			{
				if (childIndex == 0)
				{
					shape.ComputeAABB(aabb, identity, childIndex);
				}
				else
				{
					var childAABB = new b2AABB();
					shape.ComputeAABB(childAABB, identity, childIndex);
					aabb.Combine(childAABB);
				}
			}
			for (var y = Math.floor(aabb.lowerBound.y / stride) * stride; y < aabb.upperBound.y; y += stride)
			{
				for (var x = Math.floor(aabb.lowerBound.x / stride) * stride; x < aabb.upperBound.x; x += stride)
				{
					var p = new b2Vec2(x, y);
					if (shape.TestPoint(identity, p))
					{
						p = b2Mul_t_v2(transform, p);
						particleDef.position.Assign(p);
						particleDef.velocity.Assign(b2Vec2.Add(groupDef.linearVelocity, b2Cross_f_v2(groupDef.angularVelocity, b2Vec2.Subtract(p, groupDef.position))));
						this.CreateParticle(particleDef);
					}
				}
			}
		}
		var lastIndex = this.m_count;

		var group = new b2ParticleGroup();
		group.m_system = this;
		group.m_firstIndex = firstIndex;
		group.m_lastIndex = lastIndex;
		group.m_groupFlags = groupDef.groupFlags;
		group.m_strength = groupDef.strength;
		group.m_userData = groupDef.userData;
		group.m_transform = transform;
		group.m_destroyAutomatically = groupDef.destroyAutomatically;
		group.m_prev = null;
		group.m_next = this.m_groupList;
		if (this.m_groupList)
		{
			this.m_groupList.m_prev = group;
		}
		this.m_groupList = group;
		++this.m_groupCount;
		for (var i = firstIndex; i < lastIndex; i++)
		{
			this.m_groupBuffer[i] = group;
		}

		this.UpdateContacts(true);
		if (groupDef.flags & b2ParticleSystem.k_pairFlags)
		{
			for (var k = 0; k < this.m_contactCount; k++)
			{
				var contact = this.m_contactBuffer[k];
				var a = contact.indexA;
				var b = contact.indexB;
				if (a > b)
				{
					var oa = a;
					a = b;
					b = oa;
				}
				if (firstIndex <= a && b < lastIndex)
				{
					if (this.m_pairCount >= this.m_pairCapacity)
					{
						var oldCapacity = this.m_pairCapacity;
						var newCapacity = this.m_pairCount ? 2 * this.m_pairCount : b2_minParticleBufferCapacity;
						this.m_pairBuffer = this.ReallocateBuffer3(this.m_pairBuffer, oldCapacity, newCapacity);
						this.m_pairCapacity = newCapacity;
					}
					var pair = this.m_pairBuffer[this.m_pairCount] = new b2ParticleSystem.Pair();
					pair.indexA = a;
					pair.indexB = b;
					pair.flags = contact.flags;
					pair.strength = groupDef.strength;
					pair.distance = b2Distance(
						this.m_positionBuffer.data[a],
						this.m_positionBuffer.data[b]);
					this.m_pairCount++;
				}
			}
		}
		if (groupDef.flags & b2ParticleSystem.k_triadFlags)
		{
			var diagram = new b2VoronoiDiagram(lastIndex - firstIndex);
			for (var i = firstIndex; i < lastIndex; i++)
			{
				diagram.AddGenerator(this.m_positionBuffer.data[i], i);
			}
			diagram.Generate(stride / 2);
			var callback = function CreateParticleGroupCallback(a, b, c)
			{
				var pa = this.m_positionBuffer.data[a];
				var pb = this.m_positionBuffer.data[b];
				var pc = this.m_positionBuffer.data[c];
				var dab = b2Vec2.Subtract(pa, pb);
				var dbc = b2Vec2.Subtract(pb, pc);
				var dca = b2Vec2.Subtract(pc, pa);
				var maxDistanceSquared = b2_maxTriadDistanceSquared * this.m_squaredDiameter;
				if (b2Dot_b2_b2(dab, dab) < maxDistanceSquared &&
					b2Dot_b2_b2(dbc, dbc) < maxDistanceSquared &&
					b2Dot_b2_b2(dca, dca) < maxDistanceSquared)
				{
					if (this.m_triadCount >= this.m_triadCapacity)
					{
						var oldCapacity = this.m_triadCapacity;
						var newCapacity = this.m_triadCount ? 2 * this.m_triadCount : b2_minParticleBufferCapacity;
						this.m_triadBuffer = this.ReallocateBuffer3(this.m_triadBuffer, oldCapacity, newCapacity);
						this.m_triadCapacity = newCapacity;
					}
					var triad = this.m_triadBuffer[this.m_triadCount];
					triad.indexA = a;
					triad.indexB = b;
					triad.indexC = c;
					triad.flags =
						this.m_flagsBuffer.data[a] |
						this.m_flagsBuffer.data[b] |
						this.m_flagsBuffer.data[c];
					triad.strength = groupDef.strength;
					var midPoint = b2Vec2.Multiply(1.0 / 3.0, b2Vec2.Add(pa, b2Vec2.Add(pb, pc)));
					triad.pa = b2Vec2.Subtract(pa, midPoint);
					triad.pb = b2Vec2.Subtract(pb, midPoint);
					triad.pc = b2Vec2.Subtract(pc, midPoint);
					triad.ka = -b2Dot_v2_v2(dca, dab);
					triad.kb = -b2Dot_v2_v2(dab, dbc);
					triad.kc = -b2Dot_v2_v2(dbc, dca);
					triad.s = b2Cross_v2_v2(pa, pb) + b2Cross_v2_v2(pb, pc) + b2Cross_v2_v2(pc, pa);
					this.m_triadCount++;
				}
			};
			//callback.system = this;
			//callback.def = groupDef;
			//callback.firstIndex = firstIndex;
			diagram.GetNodes(callback);
		}
		if (groupDef.groupFlags & b2ParticleDef.b2_solidParticleGroup)
		{
			ComputeDepthForGroup(group);
		}

		return group;
	},

	JoinParticleGroups: function(groupA, groupB)
	{
'#if @DEBUG';
		b2Assert(groupA != groupB);
'#endif';
		this.RotateBuffer(groupB.m_firstIndex, groupB.m_lastIndex, this.m_count);
'#if @DEBUG';
		b2Assert(groupB.m_lastIndex == this.m_count);
'#endif';
		this.RotateBuffer(groupA.m_firstIndex, groupA.m_lastIndex, groupB.m_firstIndex);
'#if @DEBUG';
		this.b2Assert(groupA.m_lastIndex == groupB.m_firstIndex);
'#endif';

		var particleFlags = 0;
		for (var i = groupA.m_firstIndex; i < groupB.m_lastIndex; i++)
		{
			particleFlags |= this.m_flagsBuffer.data[i];
		}

		this.UpdateContacts(true);
		if (particleFlags & b2ParticleSystem.k_pairFlags)
		{
			for (var k = 0; k < this.m_contactCount; k++)
			{
				var contact = this.m_contactBuffer[k];
				var a = contact.indexA;
				var b = contact.indexB;
				if (a > b)
				{
					var oa = a;
					a = b;
					b = oa;
				}
				if (groupA.m_firstIndex <= a && a < groupA.m_lastIndex &&
					groupB.m_firstIndex <= b && b < groupB.m_lastIndex)
				{
					if (this.m_pairCount >= this.m_pairCapacity)
					{
						var oldCapacity = this.m_pairCapacity;
						var newCapacity = this.m_pairCount ? 2 * this.m_pairCount : b2_minParticleBufferCapacity;
						this.m_pairBuffer = this.ReallocateBuffer3(this.m_pairBuffer, oldCapacity, newCapacity);
						this.m_pairCapacity = newCapacity;
					}
					var pair = this.m_pairBuffer[this.m_pairCount] = new b2ParticleSystem.Pair();
					pair.indexA = a;
					pair.indexB = b;
					pair.flags = contact.flags;
					pair.strength = b2Min(groupA.m_strength, groupB.m_strength);
					pair.distance = b2Distance(this.m_positionBuffer.data[a], this.m_positionBuffer.data[b]);
					this.m_pairCount++;
				}
			}
		}
		if (particleFlags & b2ParticleSystem.k_triadFlags)
		{
			var diagram = new b2VoronoiDiagram(groupB.m_lastIndex - groupA.m_firstIndex);
			for (var i = groupA.m_firstIndex; i < groupB.m_lastIndex; i++)
			{
				if (!(this.m_flagsBuffer.data[i] & b2ParticleDef.b2_zombieParticle))
				{
					diagram.AddGenerator(this.m_positionBuffer.data[i], i);
				}
			}
			diagram.Generate(this.GetParticleStride() / 2);
			var callback = new JoinParticleGroupsCallback();
			callback.system = this;
			callback.groupA = groupA;
			callback.groupB = groupB;
			diagram.GetNodes(callback);
		}

		for (var i = groupB.m_firstIndex; i < groupB.m_lastIndex; i++)
		{
			this.m_groupBuffer[i] = groupA;
		}
		var groupFlags = groupA.m_groupFlags | groupB.m_groupFlags;
		groupA.m_groupFlags = groupFlags;
		groupA.m_lastIndex = groupB.m_lastIndex;
		groupB.m_firstIndex = groupB.m_lastIndex;
		this.DestroyParticleGroup(groupB);

		if (groupFlags & b2ParticleDef.b2_solidParticleGroup)
		{
			this.ComputeDepthForGroup(groupA);
		}
	},

	DestroyParticleGroup: function(group)
	{
'#if @DEBUG';
		b2Assert(this.m_groupCount > 0);
		b2Assert(group);
'#endif';

		if (this.m_world.m_destructionListener)
		{
			this.m_world.m_destructionListener.SayGoodbye(group);
		}

		for (var i = group.m_firstIndex; i < group.m_lastIndex; i++)
		{
			this.m_groupBuffer[i] = null;
		}

		if (group.m_prev)
		{
			group.m_prev.m_next = group.m_next;
		}
		if (group.m_next)
		{
			group.m_next.m_prev = group.m_prev;
		}
		if (group == this.m_groupList)
		{
			this.m_groupList = group.m_next;
		}

		--this.m_groupCount;
	},

	ComputeDepthForGroup: function(group)
	{
		for (var i = group.m_firstIndex; i < group.m_lastIndex; i++)
		{
			this.m_accumulationBuffer[i] = 0;
		}
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			var a = contact.indexA;
			var b = contact.indexB;
			if (a >= group.m_firstIndex && a < group.m_lastIndex &&
				b >= group.m_firstIndex && b < group.m_lastIndex)
			{
				var w = contact.weight;
				this.m_accumulationBuffer[a] += w;
				this.m_accumulationBuffer[b] += w;
			}
		}
		this.m_depthBuffer = this.RequestParticleBuffer(this.m_depthBuffer);
		for (var i = group.m_firstIndex; i < group.m_lastIndex; i++)
		{
			var w = this.m_accumulationBuffer[i];
			this.m_depthBuffer[i] = w < 0.8 ? 0 : b2_maxFloat;
		}
		var interationCount = group.GetParticleCount();
		for (var t = 0; t < interationCount; t++)
		{
			var updated = false;
			for (var k = 0; k < this.m_contactCount; k++)
			{
				var contact = this.m_contactBuffer[k];
				var a = contact.indexA;
				var b = contact.indexB;
				if (a >= group.m_firstIndex && a < group.m_lastIndex &&
					b >= group.m_firstIndex && b < group.m_lastIndex)
				{
					var r = 1 - contact.weight;
					var ap0 = this.m_depthBuffer[a];
					var bp0 = this.m_depthBuffer[b];
					var ap1 = bp0 + r;
					var bp1 = ap0 + r;
					if (ap0 > ap1)
					{
						ap0 = ap1;
						updated = true;
					}
					if (bp0 > bp1)
					{
						bp0 = bp1;
						updated = true;
					}
					this.m_depthBuffer[a] = ap0;
					this.m_depthBuffer[b] = bp0;
				}
			}
			if (!updated)
			{
				break;
			}
		}
		for (var i = group.m_firstIndex; i < group.m_lastIndex; i++)
		{
			var p = this.m_depthBuffer[i];
			if (p < b2_maxFloat)
			{
				p *= this.m_particleDiameter;
			}
			else
			{
				p = 0;
			}
			this.m_depthBuffer[i] = p;
		}
	},

	AddContact: function(a, b)
	{
		var d = b2Vec2.Subtract(this.m_positionBuffer.data[b], this.m_positionBuffer.data[a]);
		var d2 = b2Dot_v2_v2(d, d);
		if (d2 < this.m_squaredDiameter)
		{
			if (this.m_contactCount >= this.m_contactCapacity)
			{
				var oldCapacity = this.m_contactCapacity;
				var newCapacity = this.m_contactCount ? 2 * this.m_contactCount : b2_minParticleBufferCapacity;
				this.m_contactBuffer = this.ReallocateBuffer3(this.m_contactBuffer, oldCapacity, newCapacity);
				this.m_contactCapacity = newCapacity;
			}
			var invD = b2InvSqrt(d2);
			var contact = this.m_contactBuffer[this.m_contactCount] = new b2ParticleContact();
			contact.indexA = a;
			contact.indexB = b;
			contact.flags = this.m_flagsBuffer.data[a] | this.m_flagsBuffer.data[b];
			contact.weight = 1 - d2 * invD * this.m_inverseDiameter;
			contact.normal.Assign(b2Vec2.Multiply(invD, d));
			this.m_contactCount++;
		}
	},

	UpdateContacts: function(exceptZombie)
	{
		var beginProxy = 0;
		var endProxy = this.m_proxyCount;
		for (var proxyID = beginProxy; proxyID < endProxy; ++proxyID)
		{
			var proxy = this.m_proxyBuffer[proxyID];
			var i = proxy.index;
			var p = this.m_positionBuffer.data[i];
			proxy.tag = computeTag(this.m_inverseDiameter * p.x, this.m_inverseDiameter * p.y);
		}
		this.m_proxyBuffer.qsort(beginProxy, endProxy, function(a, b) { return b2ParticleSystem.Proxy.LessThan_p_p(a, b); });
		this.m_contactCount = 0;
		for (var a = beginProxy, c = beginProxy; a < endProxy; a++)
		{
			var rightTag = computeRelativeTag(this.m_proxyBuffer[a].tag, 1, 0);
			for (var b = a + 1; b < endProxy; b++)
			{
				if (rightTag < this.m_proxyBuffer[b].tag) break;
				this.AddContact(this.m_proxyBuffer[a].index, this.m_proxyBuffer[b].index);
			}
			var bottomLeftTag = computeRelativeTag(this.m_proxyBuffer[a].tag, -1, 1);
			for (; c < endProxy; c++)
			{
				if (bottomLeftTag <= this.m_proxyBuffer[c].tag) break;
			}
			var bottomRightTag = computeRelativeTag(this.m_proxyBuffer[a].tag, 1, 1);
			for (var b = c; b < endProxy; b++)
			{
				if (bottomRightTag < this.m_proxyBuffer[b].tag) break;
				this.AddContact(this.m_proxyBuffer[a].index, this.m_proxyBuffer[b].index);
			}
		}
		if (exceptZombie)
		{
			/*b2ParticleContact* lastContact = std::remove_if(
				m_contactBuffer, m_contactBuffer + m_contactCount,
				b2ParticleContactIsZombie);
			m_contactCount = (int32) (lastContact - m_contactBuffer);*/
			this.m_contactCount = this.m_contactBuffer.collapse(b2ParticleContactIsZombie, this.m_contactCount);
		}
	},

	UpdateBodyContacts: function()
	{
		var aabb = new b2AABB();
		aabb.lowerBound.x = +b2_maxFloat;
		aabb.lowerBound.y = +b2_maxFloat;
		aabb.upperBound.x = -b2_maxFloat;
		aabb.upperBound.y = -b2_maxFloat;
		for (var i = 0; i < this.m_count; i++)
		{
			var p = this.m_positionBuffer.data[i];
			aabb.lowerBound.Assign(b2Min_v2(aabb.lowerBound, p));
			aabb.upperBound.Assign(b2Max_v2(aabb.upperBound, p));
		}
		aabb.lowerBound.x -= this.m_particleDiameter;
		aabb.lowerBound.y -= this.m_particleDiameter;
		aabb.upperBound.x += this.m_particleDiameter;
		aabb.upperBound.y += this.m_particleDiameter;
		this.m_bodyContactCount = 0;
		function UpdateBodyContactsCallback(system)
		{
			this.m_system = system;
		}

		UpdateBodyContactsCallback.prototype =
		{
			ReportFixture: function(fixture)
			{
				if (fixture.IsSensor())
				{
					return true;
				}
				var shape = fixture.GetShape();
				var b = fixture.GetBody();
				var bp = b.GetWorldCenter();
				var bm = b.GetMass();
				var bI = b.GetInertia() - bm * b.GetLocalCenter().LengthSquared();
				var invBm = bm > 0 ? 1 / bm : 0;
				var invBI = bI > 0 ? 1 / bI : 0;
				var childCount = shape.GetChildCount();
				for (var childIndex = 0; childIndex < childCount; childIndex++)
				{
					var aabb = fixture.GetAABB(childIndex).Clone();
					aabb.lowerBound.x -= this.m_system.m_particleDiameter;
					aabb.lowerBound.y -= this.m_system.m_particleDiameter;
					aabb.upperBound.x += this.m_system.m_particleDiameter;
					aabb.upperBound.y += this.m_system.m_particleDiameter;
					var beginProxy = 0;
					var endProxy = this.m_system.m_proxyCount;

					var firstProxy = this.m_system.m_proxyBuffer.lower_bound(
						beginProxy, endProxy,
						computeTag(
							this.m_system.m_inverseDiameter * aabb.lowerBound.x,
							this.m_system.m_inverseDiameter * aabb.lowerBound.y),
						function(a, b) { return b2ParticleSystem.Proxy.LessThan_p_i(a, b); });

					var lastProxy = this.m_system.m_proxyBuffer.upper_bound(
						firstProxy, endProxy,
						computeTag(
							this.m_system.m_inverseDiameter * aabb.upperBound.x,
							this.m_system.m_inverseDiameter * aabb.upperBound.y),
						function(a, b) { return b2ParticleSystem.Proxy.LessThan_i_p(a, b); });

					for (var proxy = firstProxy; proxy != lastProxy; ++proxy)
					{
						var actualProxy = this.m_system.m_proxyBuffer[proxy];
						var a = actualProxy.index;
						var ap = this.m_system.m_positionBuffer.data[a];
						if (aabb.lowerBound.x <= ap.x && ap.x <= aabb.upperBound.x &&
							aabb.lowerBound.y <= ap.y && ap.y <= aabb.upperBound.y)
						{
							var d = [0];
							var n = new b2Vec2();
							fixture.ComputeDistance(ap, d, n, childIndex);
							if (d[0] < this.m_system.m_particleDiameter)
							{
								var invAm =
									this.m_system.m_flagsBuffer.data[a] & b2ParticleDef.b2_wallParticle ?
									0 : this.m_system.GetParticleInvMass();
								var rp = b2Vec2.Subtract(ap, bp);
								var rpn = b2Cross_v2_v2(rp, n);
								if (this.m_system.m_bodyContactCount >= this.m_system.m_bodyContactCapacity)
								{
									var oldCapacity = this.m_system.m_bodyContactCapacity;
									var newCapacity = this.m_system.m_bodyContactCount ? 2 * this.m_system.m_bodyContactCount : b2_minParticleBufferCapacity;
									this.m_system.m_bodyContactBuffer = this.m_system.ReallocateBuffer3(this.m_system.m_bodyContactBuffer, oldCapacity, newCapacity);
									this.m_system.m_bodyContactCapacity = newCapacity;
								}
								var contact = this.m_system.m_bodyContactBuffer[this.m_system.m_bodyContactCount] = new b2ParticleBodyContact();
								contact.index = a;
								contact.body = b;
								contact.weight = 1 - d[0] * this.m_system.m_inverseDiameter;
								contact.normal.Assign(n.Negate());
								contact.mass = 1 / (invAm + invBm + invBI * rpn * rpn);
								this.m_system.m_bodyContactCount++;
							}
						}
					}
				}
				return true;
			},

			ReportParticle: function(i) { return false; }
		};

		var callback = new UpdateBodyContactsCallback(this);
		this.m_world.QueryAABB(callback, aabb);
	},

	Solve: function(step)
	{
		++this.m_timestamp;
		if (this.m_count == 0)
		{
			return;
		}
		this.m_allParticleFlags = 0;
		for (var i = 0; i < this.m_count; i++)
		{
			this.m_allParticleFlags |= this.m_flagsBuffer.data[i];
		}
		if (this.m_allParticleFlags & b2ParticleDef.b2_zombieParticle)
		{
			this.SolveZombie();
		}
		this.m_allGroupFlags = 0;
		for (var group = this.m_groupList; group; group = group.GetNext())
		{
			this.m_allGroupFlags |= group.m_groupFlags;
		}
		var gravity = b2Vec2.Multiply(step.dt * this.m_gravityScale, this.m_world.GetGravity());
		var criticalVelocytySquared = this.GetCriticalVelocitySquared(step);
		for (var i = 0; i < this.m_count; i++)
		{
			var v = this.m_velocityBuffer.data[i];
			v.Add(gravity);
			var v2 = b2Dot_v2_v2(v, v);
			if (v2 > criticalVelocytySquared)
			{
				v.Multiply(b2Sqrt(criticalVelocytySquared / v2));
			}
		}
		this.SolveCollision(step);
		if (this.m_allGroupFlags & b2ParticleGroup.b2_rigidParticleGroup)
		{
			this.SolveRigid(step);
		}
		if (this.m_allParticleFlags & b2ParticleDef.b2_wallParticle)
		{
			this.SolveWall(step);
		}
		for (var i = 0; i < this.m_count; i++)
		{
			this.m_positionBuffer.data[i].Add(b2Vec2.Multiply(step.dt, this.m_velocityBuffer.data[i]));
		}
		this.UpdateBodyContacts();
		this.UpdateContacts(false);
		if (this.m_allParticleFlags & b2ParticleDef.b2_viscousParticle)
		{
			this.SolveViscous(step);
		}
		if (this.m_allParticleFlags & b2ParticleDef.b2_powderParticle)
		{
			this.SolvePowder(step);
		}
		if (this.m_allParticleFlags & b2ParticleDef.b2_tensileParticle)
		{
			this.SolveTensile(step);
		}
		if (this.m_allParticleFlags & b2ParticleDef.b2_elasticParticle)
		{
			this.SolveElastic(step);
		}
		if (this.m_allParticleFlags & b2ParticleDef.b2_springParticle)
		{
			this.SolveSpring(step);
		}
		if (this.m_allGroupFlags & b2ParticleGroup.b2_solidParticleGroup)
		{
			this.SolveSolid(step);
		}
		if (this.m_allParticleFlags & b2ParticleDef.b2_colorMixingParticle)
		{
			this.SolveColorMixing(step);
		}
		this.SolvePressure(step);
		this.SolveDamping(step);
	},

	SolveCollision: function(step)
	{
		var aabb = new b2AABB();
		aabb.lowerBound.x = +b2_maxFloat;
		aabb.lowerBound.y = +b2_maxFloat;
		aabb.upperBound.x = -b2_maxFloat;
		aabb.upperBound.y = -b2_maxFloat;
		for (var i = 0; i < this.m_count; i++)
		{
			var v = this.m_velocityBuffer.data[i];
			var p1 = this.m_positionBuffer.data[i];
			var p2 = b2Vec2.Add(p1, b2Vec2.Multiply(step.dt, v));
			aabb.lowerBound = b2Min_v2(aabb.lowerBound, b2Min_v2(p1, p2));
			aabb.upperBound = b2Max_v2(aabb.upperBound, b2Max_v2(p1, p2));
		}

		function SolveCollisionCallback(system, step)
		{
			this.m_system = system;
			this.m_step = step;
		}

		SolveCollisionCallback.prototype =
		{
			ReportFixture: function(fixture)
			{
				if (fixture.IsSensor())
				{
					return true;
				}
				var shape = fixture.GetShape();
				var body = fixture.GetBody();
				var beginProxy = 0;
				var endProxy = this.m_system.m_proxyCount;
				var childCount = shape.GetChildCount();
				for (var childIndex = 0; childIndex < childCount; childIndex++)
				{
					var aabb = fixture.GetAABB(childIndex).Clone();
					aabb.lowerBound.x -= this.m_system.m_particleDiameter;
					aabb.lowerBound.y -= this.m_system.m_particleDiameter;
					aabb.upperBound.x += this.m_system.m_particleDiameter;
					aabb.upperBound.y += this.m_system.m_particleDiameter;
					var firstProxy = this.m_system.m_proxyBuffer.lower_bound(
						beginProxy, endProxy,
						computeTag(
							this.m_system.m_inverseDiameter * aabb.lowerBound.x,
							this.m_system.m_inverseDiameter * aabb.lowerBound.y),
						function(a, b) { return b2ParticleSystem.Proxy.LessThan_p_i(a, b); });
					var lastProxy = this.m_system.m_proxyBuffer.upper_bound(
						firstProxy, endProxy,
						computeTag(
							this.m_system.m_inverseDiameter * aabb.upperBound.x,
							this.m_system.m_inverseDiameter * aabb.upperBound.y),
						function(a, b) { return b2ParticleSystem.Proxy.LessThan_i_p(a, b); });

					for (var proxy = firstProxy; proxy != lastProxy; ++proxy)
					{
						var actualProxy = this.m_system.m_proxyBuffer[proxy];
						var a = actualProxy.index;
						var ap = this.m_system.m_positionBuffer.data[a];
						if (aabb.lowerBound.x <= ap.x && ap.x <= aabb.upperBound.x &&
							aabb.lowerBound.y <= ap.y && ap.y <= aabb.upperBound.y)
						{
							var av = this.m_system.m_velocityBuffer.data[a];
							var output = new b2RayCastOutput();
							var input = new b2RayCastInput();
							input.p1 = b2Mul_t_v2(body.m_xf, b2MulT_t_v2(body.m_xf0, ap));
							input.p2 = b2Vec2.Add(ap, b2Vec2.Multiply(this.m_step.dt, av));
							input.maxFraction = 1;
							if (fixture.RayCast(output, input, childIndex))
							{
								var p =
									b2Vec2.Add(b2Vec2.Add(b2Vec2.Multiply((1 - output.fraction), input.p1),
									b2Vec2.Multiply(output.fraction, input.p2)),
									b2Vec2.Multiply(b2_linearSlop, output.normal));

								var v = b2Vec2.Multiply(this.m_step.inv_dt, b2Vec2.Subtract(p, ap));
								this.m_system.m_velocityBuffer.data[a].Assign(v);
								var f = b2Vec2.Multiply(this.m_system.GetParticleMass(), b2Vec2.Subtract(av, v));
								f = b2Vec2.Multiply(b2Dot_v2_v2(f, output.normal), output.normal);
								body.ApplyLinearImpulse(f, p, true);
							}
						}
					}
				}
				return true;
			},

			ReportParticle: function(i) { return false; }
		};

		var callback = new SolveCollisionCallback(this, step);
		this.m_world.QueryAABB(callback, aabb);
	},

	SolvePressure: function(step)
	{
		// calculates the sum of contact-weights for each particle
		// that means dimensionless density
		for (var i = 0; i < this.m_count; i++)
		{
			this.m_accumulationBuffer[i] = 0;
		}
		for (var k = 0; k < this.m_bodyContactCount; k++)
		{
			var contact = this.m_bodyContactBuffer[k];
			var a = contact.index;
			var w = contact.weight;
			this.m_accumulationBuffer[a] += w;
		}
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			var a = contact.indexA;
			var b = contact.indexB;
			var w = contact.weight;
			this.m_accumulationBuffer[a] += w;
			this.m_accumulationBuffer[b] += w;
		}
		// ignores powder particles
		if (this.m_allParticleFlags & b2ParticleSystem.k_noPressureFlags)
		{
			for (var i = 0; i < this.m_count; i++)
			{
				if (this.m_flagsBuffer.data[i] & b2ParticleSystem.k_noPressureFlags)
				{
					this.m_accumulationBuffer[i] = 0;
				}
			}
		}
		// calculates pressure as a linear function of density
		var pressurePerWeight = this.m_pressureStrength * this.GetCriticalPressure(step);
		for (var i = 0; i < this.m_count; i++)
		{
			var w = this.m_accumulationBuffer[i];
			var h = pressurePerWeight * b2Max(0.0, b2Min(w, b2_maxParticleWeight) - b2_minParticleWeight);
			this.m_accumulationBuffer[i] = h;
		}
		// applies pressure between each particles in contact
		var velocityPerPressure = step.dt / (this.m_density * this.m_particleDiameter);
		for (var k = 0; k < this.m_bodyContactCount; k++)
		{
			var contact = this.m_bodyContactBuffer[k];
			var a = contact.index;
			var b = contact.body;
			var w = contact.weight;
			var m = contact.mass;
			var n = contact.normal;
			var p = this.m_positionBuffer.data[a];
			var h = this.m_accumulationBuffer[a] + pressurePerWeight * w;
			var f = b2Vec2.Multiply(velocityPerPressure * w * m * h, n);
			this.m_velocityBuffer.data[a].Subtract(b2Vec2.Multiply(this.GetParticleInvMass(), f));
			b.ApplyLinearImpulse(f, p, true);
		}
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			var a = contact.indexA;
			var b = contact.indexB;
			var w = contact.weight;
			var n = contact.normal;
			var h = this.m_accumulationBuffer[a] + this.m_accumulationBuffer[b];
			var f = b2Vec2.Multiply(velocityPerPressure * w * h, n);
			this.m_velocityBuffer.data[a].Subtract(f);
			this.m_velocityBuffer.data[b].Add(f);
		}
	},

	SolveDamping: function(step)
	{
		// reduces normal velocity of each contact
		var damping = this.m_dampingStrength;
		for (var k = 0; k < this.m_bodyContactCount; k++)
		{
			var contact = this.m_bodyContactBuffer[k];
			var a = contact.index;
			var b = contact.body;
			var w = contact.weight;
			var m = contact.mass;
			var n = contact.normal;
			var p = this.m_positionBuffer.data[a];
			var v = b2Vec2.Subtract(b.GetLinearVelocityFromWorldPoint(p), this.m_velocityBuffer.data[a]);
			var vn = b2Dot_v2_v2(v, n);
			if (vn < 0)
			{
				var f = b2Vec2.Multiply(damping * w * m * vn, n);
				this.m_velocityBuffer.data[a].Add(b2Vec2.Multiply(this.GetParticleInvMass(), f));
				b.ApplyLinearImpulse(f.Negate(), p, true);
			}
		}
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			var a = contact.indexA;
			var b = contact.indexB;
			var w = contact.weight;
			var n = contact.normal;
			var v = b2Vec2.Subtract(this.m_velocityBuffer.data[b], this.m_velocityBuffer.data[a]);
			var vn = b2Dot_v2_v2(v, n);
			if (vn < 0)
			{
				var f = b2Vec2.Multiply(damping * w * vn, n);
				this.m_velocityBuffer.data[a].Add(f);
				this.m_velocityBuffer.data[b].Subtract(f);
			}
		}
	},

	SolveWall: function(step)
	{
		for (var i = 0; i < this.m_count; i++)
		{
			if (this.m_flagsBuffer.data[i] & b2ParticleDef.b2_wallParticle)
			{
				this.m_velocityBuffer.data[i].SetZero();
			}
		}
	},

	SolveRigid: function(step)
	{
		for (var group = this.m_groupList; group; group = group.GetNext())
		{
			if (group.m_groupFlags & b2ParticleGroup.b2_rigidParticleGroup)
			{
				group.UpdateStatistics();
				var rotation = new b2Rot(step.dt * group.m_angularVelocity);
				var transform = new b2Transform(
					b2Vec2.Add(group.m_center, b2Vec2.Subtract(b2Vec2.Multiply(step.dt, group.m_linearVelocity), b2Mul_r_v2(rotation, group.m_center))),
					rotation);
				group.m_transform = b2Mul_t_t(transform, group.m_transform);
				var velocityTransform = new b2Transform();
				velocityTransform.p.x = step.inv_dt * transform.p.x;
				velocityTransform.p.y = step.inv_dt * transform.p.y;
				velocityTransform.q.s = step.inv_dt * transform.q.s;
				velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
				for (var i = group.m_firstIndex; i < group.m_lastIndex; i++)
				{
					this.m_velocityBuffer.data[i].Assign(b2Mul_t_v2(velocityTransform, this.m_positionBuffer.data[i]));
				}
			}
		}
	},

	SolveElastic: function(step)
	{
		var elasticStrength = step.inv_dt * this.m_elasticStrength;
		for (var k = 0; k < this.m_triadCount; k++)
		{
			var triad = this.m_triadBuffer[k];
			if (triad.flags & b2ParticleDef.b2_elasticParticle)
			{
				var a = triad.indexA;
				var b = triad.indexB;
				var c = triad.indexC;
				var oa = triad.pa;
				var ob = triad.pb;
				var oc = triad.pc;
				var pa = this.m_positionBuffer.data[a];
				var pb = this.m_positionBuffer.data[b];
				var pc = this.m_positionBuffer.data[c];
				var p = b2Vec2.Multiply(1 / 3, b2Vec2.Add(pa, b2Vec2.Add(pb, pc)));
				var r = new b2Rot();
				r.s = b2Cross_v2_v2(oa, pa) + b2Cross_v2_v2(ob, pb) + b2Cross_v2_v2(oc, pc);
				r.c = b2Dot_v2_v2(oa, pa) + b2Dot_v2_v2(ob, pb) + b2Dot_v2_v2(oc, pc);
				var r2 = r.s * r.s + r.c * r.c;
				var invR = b2InvSqrt(r2);
				r.s *= invR;
				r.c *= invR;
				var strength = elasticStrength * triad.strength;
				this.m_velocityBuffer.data[a].Add(b2Vec2.Multiply(strength, (b2Vec2.Subtract(b2Mul(r, oa), (b2Vec2.Subtract(pa, p))))));
				this.m_velocityBuffer.data[b].Add(b2Vec2.Multiply(strength, (b2Vec2.Subtract(b2Mul(r, ob), (b2Vec2.Subtract(pb, p))))));
				this.m_velocityBuffer.data[c].Add(b2Vec2.Multiply(strength, (b2Vec2.Subtract(b2Mul(r, oc), (b2Vec2.Subtract(pc, p))))));
			}
		}
	},

	SolveSpring: function(step)
	{
		var springStrength = step.inv_dt * this.m_springStrength;
		for (var k = 0; k < this.m_pairCount; k++)
		{
			var pair = this.m_pairBuffer[k];
			if (pair.flags & b2ParticleDef.b2_springParticle)
			{
				var a = pair.indexA;
				var b = pair.indexB;
				var d = b2Vec2.Subtract(this.m_positionBuffer.data[b], this.m_positionBuffer.data[a]);
				var r0 = pair.distance;
				var r1 = d.Length();
				var strength = springStrength * pair.strength;
				var f = b2Vec2.Multiply(strength * (r0 - r1) / r1, d);
				this.m_velocityBuffer.data[a].Subtract(f);
				this.m_velocityBuffer.data[b].Add(f);
			}
		}
	},

	SolveTensile: function(step)
	{
		this.m_accumulation2Buffer = this.RequestParticleBuffer(this.m_accumulation2Buffer);
		for (var i = 0; i < this.m_count; i++)
		{
			this.m_accumulationBuffer[i] = 0;
			this.m_accumulation2Buffer[i] = new b2Vec2();
		}
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			if (contact.flags & b2ParticleDef.b2_tensileParticle)
			{
				var a = contact.indexA;
				var b = contact.indexB;
				var w = contact.weight;
				var n = contact.normal;
				this.m_accumulationBuffer[a] += w;
				this.m_accumulationBuffer[b] += w;
				this.m_accumulation2Buffer[a].Subtract(b2Vec2.Multiply((1 - w) * w, n));
				this.m_accumulation2Buffer[b].Add(b2Vec2.Multiply((1 - w) * w, n));
			}
		}
		var strengthA = this.m_surfaceTensionStrengthA * this.GetCriticalVelocity(step);
		var strengthB = this.m_surfaceTensionStrengthB * this.GetCriticalVelocity(step);
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			if (contact.flags & b2ParticleDef.b2_tensileParticle)
			{
				var a = contact.indexA;
				var b = contact.indexB;
				var w = contact.weight;
				var n = contact.normal;
				var h = this.m_accumulationBuffer[a] + this.m_accumulationBuffer[b];
				var s = b2Vec2.Subtract(this.m_accumulation2Buffer[b], this.m_accumulation2Buffer[a]);
				var fn = (strengthA * (h - 2) + strengthB * b2Dot_v2_v2(s, n)) * w;
				var f = b2Vec2.Multiply(fn, n);
				this.m_velocityBuffer.data[a].Subtract(f);
				this.m_velocityBuffer.data[b].Add(f);
			}
		}
	},

	SolveViscous: function(step)
	{
		var viscousStrength = this.m_viscousStrength;
		for (var k = 0; k < this.m_bodyContactCount; k++)
		{
			var contact = this.m_bodyContactBuffer[k];
			var a = contact.index;
			if (this.m_flagsBuffer.data[a] & b2ParticleDef.b2_viscousParticle)
			{
				var b = contact.body;
				var w = contact.weight;
				var m = contact.mass;
				var p = this.m_positionBuffer.data[a];
				var v = b2Vec2.Subtract(b.GetLinearVelocityFromWorldPoint(p), this.m_velocityBuffer.data[a]);
				var f = b2Vec2.Multiply(viscousStrength * m * w, v);
				this.m_velocityBuffer.data[a].Add(b2Vec2.Multiply(this.GetParticleInvMass(), f));
				b.ApplyLinearImpulse(f.Negate(), p, true);
			}
		}
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			if (contact.flags & b2ParticleDef.b2_viscousParticle)
			{
				var a = contact.indexA;
				var b = contact.indexB;
				var w = contact.weight;
				var v = b2Vec2.Subtract(this.m_velocityBuffer.data[b], this.m_velocityBuffer.data[a]);
				var f = b2Vec2.Multiply(viscousStrength * w, v);
				this.m_velocityBuffer.data[a].Add(f);
				this.m_velocityBuffer.data[b].Subtract(f);
			}
		}
	},

	SolvePowder: function(step)
	{
		var powderStrength = this.m_powderStrength * this.GetCriticalVelocity(step);
		var minWeight = 1.0 - b2_particleStride;
		for (var k = 0; k < this.m_bodyContactCount; k++)
		{
			var contact = this.m_bodyContactBuffer[k];
			var a = contact.index;
			if (this.m_flagsBuffer.data[a] & b2ParticleDef.b2_powderParticle)
			{
				var w = contact.weight;
				if (w > minWeight)
				{
					var b = contact.body;
					var m = contact.mass;
					var p = this.m_positionBuffer.data[a];
					var n = contact.normal;
					var f = b2Vec2.Multiply(powderStrength * m * (w - minWeight), n);
					this.m_velocityBuffer.data[a].Subtract(b2Vec2.Multiply(this.GetParticleInvMass(), f));
					b.ApplyLinearImpulse(f, p, true);
				}
			}
		}
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			if (contact.flags & b2ParticleDef.b2_powderParticle)
			{
				var w = contact.weight;
				if (w > minWeight)
				{
					var a = contact.indexA;
					var b = contact.indexB;
					var n = contact.normal;
					var f = b2Vec2.Multiply(powderStrength * (w - minWeight), n);
					this.m_velocityBuffer.data[a].Subtract(f);
					this.m_velocityBuffer.data[b].Add(f);
				}
			}
		}
	},

	SolveSolid: function(step)
	{
		// applies extra repulsive force from solid particle groups
		this.m_depthBuffer = this.RequestParticleBuffer(this.m_depthBuffer);
		var ejectionStrength = step.inv_dt * this.m_ejectionStrength;
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			var a = contact.indexA;
			var b = contact.indexB;
			if (this.m_groupBuffer[a] != this.m_groupBuffer[b])
			{
				var w = contact.weight;
				var n = contact.normal;
				var h = this.m_depthBuffer[a] + this.m_depthBuffer[b];
				var f = b2Vec2.Multiply(ejectionStrength * h * w, n);
				this.m_velocityBuffer.data[a].Subtract(f);
				this.m_velocityBuffer.data[b].Add(f);
			}
		}
	},

	SolveColorMixing: function(step)
	{
		// mixes color between contacting particles
		this.m_colorBuffer.data = this.RequestParticleBuffer(this.m_colorBuffer.data);
		var colorMixing256 = Math.floor(256 * this.m_colorMixingStrength);
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			var a = contact.indexA;
			var b = contact.indexB;
			if (this.m_flagsBuffer.data[a] & this.m_flagsBuffer.data[b] & b2ParticleDef.b2_colorMixingParticle)
			{
				var colorA = this.m_colorBuffer.data[a];
				var colorB = this.m_colorBuffer.data[b];
				var dr = (colorMixing256 * (colorB.r - colorA.r)) >> 8;
				var dg = (colorMixing256 * (colorB.g - colorA.g)) >> 8;
				var db = (colorMixing256 * (colorB.b - colorA.b)) >> 8;
				var da = (colorMixing256 * (colorB.a - colorA.a)) >> 8;
				colorA.r += dr;
				colorA.g += dg;
				colorA.b += db;
				colorA.a += da;
				colorB.r -= dr;
				colorB.g -= dg;
				colorB.b -= db;
				colorB.a -= da;
			}
		}
	},

	SolveZombie: function()
	{
		// removes particles with zombie flag
		var newCount = 0;
		var newIndices = new Array(this.m_count);
		for (var i = 0; i < this.m_count; i++)
		{
			var flags = this.m_flagsBuffer.data[i];
			if (flags & b2ParticleDef.b2_zombieParticle)
			{
				var destructionListener = this.m_world.m_destructionListener;
				if ((flags & b2ParticleDef.b2_destructionListener) && destructionListener)
				{
					destructionListener.SayGoodbyeParticle(i);
				}
				newIndices[i] = b2ParticleSystem.b2_invalidParticleIndex;
			}
			else
			{
				newIndices[i] = newCount;
				if (i != newCount)
				{
					this.m_flagsBuffer.data[newCount] = this.m_flagsBuffer.data[i];
					this.m_positionBuffer.data[newCount] = this.m_positionBuffer.data[i];
					this.m_velocityBuffer.data[newCount] = this.m_velocityBuffer.data[i];
					this.m_groupBuffer[newCount] = this.m_groupBuffer[i];
					if (this.m_depthBuffer)
					{
						this.m_depthBuffer[newCount] = this.m_depthBuffer[i];
					}
					if (this.m_colorBuffer.data)
					{
						this.m_colorBuffer.data[newCount] = this.m_colorBuffer.data[i];
					}
					if (this.m_userDataBuffer.data)
					{
						this.m_userDataBuffer.data[newCount] = this.m_userDataBuffer.data[i];
					}
				}
				newCount++;
			}
		}

		// predicate functions
		var Test =
		{
			IsProxyInvalid: function(proxy)
			{
				return proxy.index < 0;
			},
			IsContactInvalid: function(contact)
			{
				return contact.indexA < 0 || contact.indexB < 0;
			},
			IsBodyContactInvalid: function(contact)
			{
				return contact.index < 0;
			},
			IsPairInvalid: function(pair)
			{
				return pair.indexA < 0 || pair.indexB < 0;
			},
			IsTriadInvalid: function(triad)
			{
				return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
			}
		};

		// update proxies
		for (var k = 0; k < this.m_proxyCount; k++)
		{
			var proxy = this.m_proxyBuffer[k];
			proxy.index = newIndices[proxy.index];
		}
		/*Proxy* lastProxy = std::remove_if(
			m_proxyBuffer, m_proxyBuffer + m_proxyCount,
			Test::IsProxyInvalid);
		m_proxyCount = (int32) (lastProxy - m_proxyBuffer);*/
		this.m_proxyCount = this.m_proxyBuffer.collapse(Test.IsProxyInvalid, this.m_proxyCount);

		// update contacts
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			contact.indexA = newIndices[contact.indexA];
			contact.indexB = newIndices[contact.indexB];
		}
		/*b2ParticleContact* lastContact = std::remove_if(
			m_contactBuffer, m_contactBuffer + m_contactCount,
			Test::IsContactInvalid);
		m_contactCount = (int32) (lastContact - m_contactBuffer);*/
		this.m_contactCount = this.m_contactBuffer.collapse(Test.IsContactInvalid, this.m_contactCount);

		// update particle-body contacts
		for (var k = 0; k < this.m_bodyContactCount; k++)
		{
			var contact = this.m_bodyContactBuffer[k];
			contact.index = newIndices[contact.index];
		}
		/*b2ParticleBodyContact* lastBodyContact = std::remove_if(
			m_bodyContactBuffer, m_bodyContactBuffer + m_bodyContactCount,
			Test::IsBodyContactInvalid);
		m_bodyContactCount = (int32) (lastBodyContact - m_bodyContactBuffer);*/
		this.m_bodyContactCount = this.m_bodyContactBuffer.collapse(Test.IsBodyContactInvalid, this.m_bodyContactCount);

		// update pairs
		for (var k = 0; k < this.m_pairCount; k++)
		{
			var pair = this.m_pairBuffer[k];
			pair.indexA = newIndices[pair.indexA];
			pair.indexB = newIndices[pair.indexB];
		}
		/*Pair* lastPair = std::remove_if(
			m_pairBuffer, m_pairBuffer + m_pairCount, Test::IsPairInvalid);
		m_pairCount = (int32) (lastPair - m_pairBuffer);*/
		this.m_pairCount = this.m_pairBuffer.collapse(Test.IsPairInvalid, this.m_pairCount);

		// update triads
		for (var k = 0; k < this.m_triadCount; k++)
		{
			var triad = this.m_triadBuffer[k];
			triad.indexA = newIndices[triad.indexA];
			triad.indexB = newIndices[triad.indexB];
			triad.indexC = newIndices[triad.indexC];
		}
		/*Triad* lastTriad = std::remove_if(
			m_triadBuffer, m_triadBuffer + m_triadCount,
			Test::IsTriadInvalid);
		m_triadCount = (int32) (lastTriad - m_triadBuffer);*/
		this.m_triadCount = this.m_triadBuffer.collapse(Test.IsTriadInvalid, this.m_triadCount);

		// update groups
		for (var group = this.m_groupList; group; group = group.GetNext())
		{
			var firstIndex = newCount;
			var lastIndex = 0;
			var modified = false;
			for (var i = group.m_firstIndex; i < group.m_lastIndex; i++)
			{
				var j = newIndices[i];
				if (j >= 0) {
					firstIndex = b2Min(firstIndex, j);
					lastIndex = b2Max(lastIndex, j + 1);
				} else {
					modified = true;
				}
			}
			if (firstIndex < lastIndex)
			{
				group.m_firstIndex = firstIndex;
				group.m_lastIndex = lastIndex;
				if (modified)
				{
					if (group.m_groupFlags & b2ParticleGroup.b2_rigidParticleGroup)
					{
						group.m_toBeSplit = true;
					}
				}
			}
			else
			{
				group.m_firstIndex = 0;
				group.m_lastIndex = 0;
				if (group.m_destroyAutomatically)
				{
					group.m_toBeDestroyed = true;
				}
			}
		}

		// update particle count
		this.m_count = newCount;

		// destroy bodies with no particles
		for (var group = this.m_groupList; group;)
		{
			var next = group.GetNext();
			if (group.m_toBeDestroyed)
			{
				this.DestroyParticleGroup(group);
			}
			else if (group.m_toBeSplit)
			{
				// TODO: split the group
			}
			group = next;
		}
	},

	RotateBuffer: function(start, mid, end)
	{
		// move the particles assigned to the given group toward the end of array
		if (start == mid || mid == end)
		{
			return;
		}

		function newIndices(i)
		{
			if (i < start)
			{
				return i;
			}
			else if (i < mid)
			{
				return i + end - mid;
			}
			else if (i < end)
			{
				return i + start - mid;
			}
			else
			{
				return i;
			}
		}

		//std.rotate(m_flagsBuffer.data + start, m_flagsBuffer.data + mid, m_flagsBuffer.data + end);
		this.m_flagsBuffer.data.rotate(start, mid, end);
		//std.rotate(m_positionBuffer.data + start, m_positionBuffer.data + mid, m_positionBuffer.data + end);
		this.m_positionBuffer.data.rotate(start, mid, end);
		//std.rotate(m_velocityBuffer.data + start, m_velocityBuffer.data + mid, m_velocityBuffer.data + end);
		this.m_velocityBuffer.data.rotate(start, mid, end);
		//std.rotate(m_groupBuffer + start, m_groupBuffer + mid, m_groupBuffer + end);
		this.m_groupBuffer.rotate(start, mid, end);

		if (this.m_depthBuffer)
		{
			//std.rotate(m_depthBuffer + start, m_depthBuffer + mid, m_depthBuffer + end);
			this.m_depthBuffer.rotate(start, mid, end);
		}
		if (this.m_colorBuffer.data)
		{
			//std.rotate(m_colorBuffer.data + start, m_colorBuffer.data + mid, m_colorBuffer.data + end);
			this.m_colorBuffer.data.rotate(start, mid, end);
		}
		if (this.m_userDataBuffer.data)
		{
			//std.rotate(m_userDataBuffer.data + start, m_userDataBuffer.data + mid, m_userDataBuffer.data + end);
			this.m_userDataBuffer.data.rotate(start, mid, end);
		}

		// update proxies
		for (var k = 0; k < this.m_proxyCount; k++)
		{
			var proxy = this.m_proxyBuffer[k];
			proxy.index = newIndices(proxy.index);
		}

		// update contacts
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			contact.indexA = newIndices(contact.indexA);
			contact.indexB = newIndices(contact.indexB);
		}

		// update particle-body contacts
		for (var k = 0; k < this.m_bodyContactCount; k++)
		{
			var contact = this.m_bodyContactBuffer[k];
			contact.index = newIndices(contact.index);
		}

		// update pairs
		for (var k = 0; k < this.m_pairCount; k++)
		{
			var pair = this.m_pairBuffer[k];
			pair.indexA = newIndices(pair.indexA);
			pair.indexB = newIndices(pair.indexB);
		}

		// update triads
		for (var k = 0; k < this.m_triadCount; k++)
		{
			var triad = this.m_triadBuffer[k];
			triad.indexA = newIndices(triad.indexA);
			triad.indexB = newIndices(triad.indexB);
			triad.indexC = newIndices(triad.indexC);
		}

		// update groups
		for (var group = this.m_groupList; group; group = group.GetNext())
		{
			group.m_firstIndex = newIndices(group.m_firstIndex);
			group.m_lastIndex = newIndices(group.m_lastIndex - 1) + 1;
		}
	},

	SetParticleRadius: function(radius)
	{
		this.m_particleDiameter = 2 * radius;
		this.m_squaredDiameter = this.m_particleDiameter * this.m_particleDiameter;
		this.m_inverseDiameter = 1 / this.m_particleDiameter;
	},

	SetParticleDensity: function(density)
	{
		this.m_density = density;
		this.m_inverseDensity =  1 / this.m_density;
	},

	GetParticleDensity: function()
	{
		return this.m_density;
	},

	SetParticleGravityScale: function(gravityScale)
	{
		this.m_gravityScale = gravityScale;
	},

	GetParticleGravityScale: function()
	{
		return this.m_gravityScale;
	},

	SetParticleDamping: function(damping)
	{
		this.m_dampingStrength = damping;
	},

	GetParticleDamping: function()
	{
		return this.m_dampingStrength;
	},

	GetParticleRadius: function()
	{
		return this.m_particleDiameter / 2;
	},

	GetCriticalVelocity: function(step)
	{
		return this.m_particleDiameter * step.inv_dt;
	},

	GetCriticalVelocitySquared: function(step)
	{
		var velocity = this.GetCriticalVelocity(step);
		return velocity * velocity;
	},

	GetCriticalPressure: function(step)
	{
		return this.m_density * this.GetCriticalVelocitySquared(step);
	},

	GetParticleStride: function()
	{
		return b2_particleStride * this.m_particleDiameter;
	},

	GetParticleMass: function()
	{
		var stride = this.GetParticleStride();
		return this.m_density * stride * stride;
	},

	GetParticleInvMass: function()
	{
		return 1.777777 * this.m_inverseDensity * this.m_inverseDiameter * this.m_inverseDiameter;
	},

	GetParticleFlagsBuffer: function()
	{
		return this.m_flagsBuffer.data;
	},

	GetParticlePositionBuffer: function()
	{
		return this.m_positionBuffer.data;
	},

	GetParticleVelocityBuffer: function()
	{
		return this.m_velocityBuffer.data;
	},

	GetParticleColorBuffer: function()
	{
		this.m_colorBuffer.data = this.RequestParticleBuffer(this.m_colorBuffer.data);
		return this.m_colorBuffer.data;
	},

	GetParticleUserDataBuffer: function()
	{
		this.m_userDataBuffer.data = this.RequestParticleBuffer(this.m_userDataBuffer.data);
		return this.m_userDataBuffer.data;
	},

	GetParticleMaxCount: function()
	{
		return this.m_maxCount;
	},

	SetParticleMaxCount: function(count)
	{
'#if @DEBUG';
		b2Assert(this.m_count <= count);
'#endif';
		this.m_maxCount = count;
	},

	GetParticleGroupBuffer: function()
	{
		return this.m_groupBuffer;
	},

	SetParticleBuffer: function(buffer, newData, newCapacity)
	{
'#if @DEBUG';
		b2Assert((newData && newCapacity) || (!newData && !newCapacity));
'#endif';
		if (!buffer.userSuppliedCapacity)
		{
			//this.m_world.m_blockAllocator.Free(buffer->data, sizeof(T) * m_internalAllocatedCapacity);
		}
		buffer.data = newData;
		buffer.userSuppliedCapacity = newCapacity;
	},

	SetParticleFlagsBuffer: function(buffer, capacity)
	{
		this.SetParticleBuffer(this.m_flagsBuffer, buffer, capacity);
	},

	SetParticlePositionBuffer: function(buffer, capacity)
	{
		this.SetParticleBuffer(this.m_positionBuffer, buffer, capacity);
	},

	SetParticleVelocityBuffer: function(buffer, capacity)
	{
		this.SetParticleBuffer(this.m_velocityBuffer, buffer, capacity);
	},

	SetParticleColorBuffer: function(buffer, capacity)
	{
		this.SetParticleBuffer(this.m_colorBuffer, buffer, capacity);
	},

	SetParticleUserDataBuffer: function(buffer, capacity)
	{
		this.SetParticleBuffer(this.m_userDataBuffer, buffer, capacity);
	},

	QueryAABB: function(callback, aabb)
	{
		if (this.m_proxyCount == 0)
		{
			return;
		}
		var beginProxy = 0;
		var endProxy = this.m_proxyCount;
		/*Proxy* firstProxy = std::lower_bound(
			beginProxy, endProxy,
			computeTag(
				m_inverseDiameter * aabb.lowerBound.x,
				m_inverseDiameter * aabb.lowerBound.y));*/
		var firstProxy = this.m_proxyBuffer.lower_bound(
			beginProxy, endProxy,
			computeTag(
				this.m_inverseDiameter * aabb.lowerBound.x,
				this.m_inverseDiameter * aabb.lowerBound.y),
			function(a, b) { return b2ParticleSystem.Proxy.LessThan_p_i(a, b); });
		/*Proxy* lastProxy = std::upper_bound(
			firstProxy, endProxy,
			computeTag(
				m_inverseDiameter * aabb.upperBound.x,
				m_inverseDiameter * aabb.upperBound.y));*/
		var lastProxy = this.m_proxyBuffer.upper_bound(
			firstProxy, endProxy,
			computeTag(
				this.m_inverseDiameter * aabb.upperBound.x,
				this.m_inverseDiameter * aabb.upperBound.y),
			function(a, b) { return b2ParticleSystem.Proxy.LessThan_i_p(a, b); });

		for (var proxy = firstProxy; proxy < lastProxy; ++proxy)
		{
			var actualProxy = this.m_proxyBuffer[proxy];
			var i = actualProxy.index;
			var p = this.m_positionBuffer.data[i];
			if (aabb.lowerBound.x < p.x && p.x < aabb.upperBound.x &&
				aabb.lowerBound.y < p.y && p.y < aabb.upperBound.y)
			{
				if (!callback.ReportParticle(i))
				{
					break;
				}
			}
		}
	},

	RayCast: function(callback, point1, point2)
	{
		if (this.m_proxyCount == 0)
		{
			return;
		}
		var beginProxy = 0;
		var endProxy = this.m_proxyCount;
		/*Proxy* firstProxy = std::lower_bound(
			beginProxy, endProxy,
			computeTag(
				m_inverseDiameter * b2Min(point1.x, point2.x) - 1,
				m_inverseDiameter * b2Min(point1.y, point2.y) - 1));*/
		var firstProxy = this.m_proxyBuffer.lower_bound(
			beginProxy, endProxy,
			computeTag(
				this.m_inverseDiameter * b2Min(point1.x, point2.x) - 1,
				this.m_inverseDiameter * b2Min(point1.y, point2.y) - 1),
			function(a, b) { return b2ParticleSystem.Proxy.LessThan_p_i(a, b); });
		/*Proxy* lastProxy = std::upper_bound(
			firstProxy, endProxy,
			computeTag(
				m_inverseDiameter * b2Max(point1.x, point2.x) + 1,
				m_inverseDiameter * b2Max(point1.y, point2.y) + 1));*/
		var lastProxy = this.m_proxyBuffer.upper_bound(
			beginProxy, endProxy,
			computeTag(
				this.m_inverseDiameter * b2Max(point1.x, point2.x) + 1,
				this.m_inverseDiameter * b2Max(point1.y, point2.y) + 1),
			function(a, b) { return b2ParticleSystem.Proxy.LessThan_i_p(a, b); });
		var fraction = 1;
		// solving the following equation:
		// ((1-t)*point1+t*point2-position)^2=diameter^2
		// where t is a potential fraction
		var v = b2Vec2.Subtract(point2, point1);
		var v2 = b2Dot_v2_v2(v, v);
		for (var proxy = firstProxy; proxy < lastProxy; ++proxy)
		{
			var actualProxy = this.m_proxyBuffer[proxy];
			var i = actualProxy.index;
			var p = b2Vec2.Subtract(point1, this.m_positionBuffer.data[i]);
			var pv = b2Dot_v2_v2(p, v);
			var p2 = b2Dot_v2_v2(p, p);
			var determinant = pv * pv - v2 * (p2 - this.m_squaredDiameter);
			if (determinant >= 0)
			{
				var sqrtDeterminant = b2Sqrt(determinant);
				// find a solution between 0 and fraction
				var t = (-pv - sqrtDeterminant) / v2;
				if (t > fraction)
				{
					continue;
				}
				if (t < 0)
				{
					t = (-pv + sqrtDeterminant) / v2;
					if (t < 0 || t > fraction)
					{
						continue;
					}
				}
				var n = b2Vec2.Add(p, b2Vec2.Subtract(t, v));
				n.Normalize();
				var f = callback.ReportParticle(i, b2Vec2.Add(point1, b2Vec2.Multiply(t, v)), n, t);
				fraction = b2Min(fraction, f);
				if (fraction <= 0)
				{
					break;
				}
			}
		}
	},

	ComputeParticleCollisionEnergy: function()
	{
		var sum_v2 = 0;
		for (var k = 0; k < this.m_contactCount; k++)
		{
			var contact = this.m_contactBuffer[k];
			var a = contact.indexA;
			var b = contact.indexB;
			var n = contact.normal;
			var v = b2Vec2.Subtract(this.m_velocityBuffer.data[b], this.m_velocityBuffer.data[a]);
			var vn = b2Dot_v2_v2(v, n);
			if (vn < 0)
			{
				sum_v2 += vn * vn;
			}
		}
		return 0.5 * this.GetParticleMass() * sum_v2;
	},

	GetParticleGroupList: function()
	{
		return this.m_groupList;
	},

	GetParticleGroupCount: function()
	{
		return this.m_groupCount;
	},

	GetParticleCount: function()
	{
		return this.m_count;
	}
};
