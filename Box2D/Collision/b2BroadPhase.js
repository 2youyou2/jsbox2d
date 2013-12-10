"use strict";

function b2Pair()
{
	this.proxyIdA = 0;
	this.proxyIdB = 0;
}

/// This is used to sort pairs.
function b2PairLessThan(pair1, pair2)
{
	if (pair1.proxyIdA == pair2.proxyIdA)
	{
		return pair1.proxyIdB - pair2.proxyIdB;
	}

	return pair1.proxyIdA - pair2.proxyIdA;
}

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
function b2BroadPhase()
{
	this.m_tree = new b2DynamicTree();
	this.m_queryProxyId = 0;

	this.m_proxyCount = 0;

	this.m_pairCount = 0;
	this.m_pairBuffer = [];

	this.m_moveCount = 0;
	this.m_moveBuffer = [];
}

b2BroadPhase.prototype =
{
	/// Create a proxy with an initial AABB. Pairs are not reported until
	/// UpdatePairs is called.
	CreateProxy: function(aabb, userData)
	{
		var proxyId = this.m_tree.CreateProxy(aabb, userData);
		++this.m_proxyCount;
		this.BufferMove(proxyId);
		return proxyId;
	},

	/// Destroy a proxy. It is up to the client to remove any pairs.
	DestroyProxy: function(proxyId)
	{
		this.UnBufferMove(proxyId);
		--this.m_proxyCount;
		this.m_tree.DestroyProxy(proxyId);
	},

	/// Call MoveProxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
	MoveProxy: function(proxyId, aabb, displacement)
	{
		var buffer = this.m_tree.MoveProxy(proxyId, aabb, displacement);
		if (buffer)
		{
			this.BufferMove(proxyId);
		}
	},

	/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
	TouchProxy: function(proxyId)
	{
		this.BufferMove(proxyId);
	},

	/// Get the fat AABB for a proxy.
	GetFatAABB: function(proxyId)
	{
		return this.m_tree.GetFatAABB(proxyId);
	},

	/// Get user data from a proxy. Returns null if the id is invalid.
	GetUserData: function(proxyId)
	{
		return this.m_tree.GetUserData(proxyId);
	},

	/// Test overlap of fat AABBs.
	TestOverlap: function(proxyIdA, proxyIdB)
	{
		var aabbA = this.m_tree.GetFatAABB(proxyIdA);
		var aabbB = this.m_tree.GetFatAABB(proxyIdB);
		return b2TestOverlap(aabbA, aabbB);
	},

	/// Get the number of proxies.
	GetProxyCount: function()
	{
		return this.m_proxyCount;
	},

	/// Update the pairs. This results in pair callbacks. This can only add pairs.
	UpdatePairs: function(callback)
	{
		// Reset pair buffer
		this.m_pairCount = 0;
		this.m_pairBuffer = [];

		// Perform tree queries for all moving proxies.
		for (var i = 0; i < this.m_moveCount; ++i)
		{
			this.m_queryProxyId = this.m_moveBuffer[i];
			if (this.m_queryProxyId == b2BroadPhase.e_nullProxy)
			{
				continue;
			}

			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			var fatAABB = this.m_tree.GetFatAABB(this.m_queryProxyId);

			// Query tree, create pairs and add them pair buffer.
			this.m_tree.Query(this, fatAABB);
		}

		// Reset move buffer
		this.m_moveCount = 0;

		// Sort the pair buffer to expose duplicates.
		//std.sort(this.m_pairBuffer, this.m_pairBuffer + this.m_pairCount, b2PairLessThan);
		this.m_pairBuffer.sort(b2PairLessThan);

		// Send the pairs back to the client.
		var i = 0;
		while (i < this.m_pairCount)
		{
			var primaryPair = this.m_pairBuffer[i];
			var userDataA = this.m_tree.GetUserData(primaryPair.proxyIdA);
			var userDataB = this.m_tree.GetUserData(primaryPair.proxyIdB);

			callback.AddPair(userDataA, userDataB);
			++i;

			// Skip any duplicate pairs.
			while (i < this.m_pairCount)
			{
				var pair = this.m_pairBuffer[i];
				if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB)
				{
					break;
				}
				++i;
			}
		}

		// Try to keep the tree balanced.
		//this->m_tree.Rebalance(4);
	},

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	Query: function(callback, aabb)
	{
		this.m_tree.Query(callback, aabb);
	},

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	RayCast: function(callback, input)
	{
		this.m_tree.RayCast(callback, input);
	},

	/// Get the height of the embedded tree.
	GetTreeHeight: function()
	{
		return this.m_tree.GetHeight();
	},

	/// Get the balance of the embedded tree.
	GetTreeBalance: function()
	{
		return this.m_tree.GetMaxBalance();
	},

	/// Get the quality metric of the embedded tree.
	GetTreeQuality: function()
	{
		return this.m_tree.GetAreaRatio();
	},

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	ShiftOrigin: function(newOrigin)
	{
		this.m_tree.ShiftOrigin(newOrigin);
	},

	BufferMove: function(proxyId)
	{
		this.m_moveBuffer[this.m_moveCount] = proxyId;
		++this.m_moveCount;
	},

	UnBufferMove: function(proxyId)
	{
		for (var i = 0; i < this.m_moveCount; ++i)
		{
			if (this.m_moveBuffer[i] == proxyId)
			{
				this.m_moveBuffer[i] = b2BroadPhase.e_nullProxy;
			}
		}
	},

	QueryCallback: function(proxyId)
	{
		// A proxy cannot form a pair with itself.
		if (proxyId == this.m_queryProxyId)
		{
			return true;
		}

		// Grow the pair buffer as needed.
		this.m_pairBuffer[this.m_pairCount] = new b2Pair();
		this.m_pairBuffer[this.m_pairCount].proxyIdA = b2Min(proxyId, this.m_queryProxyId);
		this.m_pairBuffer[this.m_pairCount].proxyIdB = b2Max(proxyId, this.m_queryProxyId);
		++this.m_pairCount;

		return true;
	}
};

b2BroadPhase.e_nullProxy = -1;