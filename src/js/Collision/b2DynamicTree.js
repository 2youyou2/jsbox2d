/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

"use strict";

var b2_nullNode = -1;

/// A node in the dynamic tree. The client does not interact with this directly.
function b2TreeNode()
{
	this.aabb = new b2AABB();
	this.userData = null;
	this.parent = 0; // next = parent!!
	this.child1 = this.child2 = this.height = 0;
}

b2TreeNode.prototype =
{
	IsLeaf: function()
	{
		return this.child1 == b2_nullNode;
	}
};

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
function b2DynamicTree()
{
	this.m_root = b2_nullNode;

	this.m_nodeCapacity = 16;
	this.m_nodeCount = 0;
	this.m_nodes = new Array(this.m_nodeCapacity);

	// Build a linked list for the free list.
	for (var i = 0; i < this.m_nodeCapacity - 1; ++i)
	{
		this.m_nodes[i] = new b2TreeNode();
		this.m_nodes[i].parent = i + 1;
		this.m_nodes[i].height = -1;
	}

	this.m_nodes[this.m_nodeCapacity-1] = new b2TreeNode();
	this.m_nodes[this.m_nodeCapacity-1].parent = b2_nullNode;
	this.m_nodes[this.m_nodeCapacity-1].height = -1;
	this.m_freeList = 0;

	this.m_path = 0;

	this.m_insertionCount = 0;
}

b2DynamicTree.prototype =
{
	/// Create a proxy. Provide a tight fitting AABB and a userData pointer.
	CreateProxy: function(aabb, userData)
	{
		var proxyId = this.AllocateNode();

		// Fatten the aabb.
		var r = new b2Vec2(b2_aabbExtension, b2_aabbExtension);
		this.m_nodes[proxyId].aabb.lowerBound.Assign(b2Vec2.Subtract(aabb.lowerBound, r));
		this.m_nodes[proxyId].aabb.upperBound.Assign(b2Vec2.Add(aabb.upperBound, r));
		this.m_nodes[proxyId].userData = userData;
		this.m_nodes[proxyId].height = 0;

		this.InsertLeaf(proxyId);

		return proxyId;
	},

	/// Destroy a proxy. This asserts if the id is invalid.
	DestroyProxy: function(proxyId)
	{
		b2Assert(0 <= proxyId && proxyId < this.m_nodeCapacity);
		b2Assert(this.m_nodes[proxyId].IsLeaf());

		this.RemoveLeaf(proxyId);
		this.FreeNode(proxyId);
	},

	/// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
	/// then the proxy is removed from the tree and re-inserted. Otherwise
	/// the function returns immediately.
	/// @return true if the proxy was re-inserted.
	MoveProxy: function(proxyId, aabb, displacement)
	{
		b2Assert(0 <= proxyId && proxyId < this.m_nodeCapacity);

		b2Assert(this.m_nodes[proxyId].IsLeaf());

		if (this.m_nodes[proxyId].aabb.Contains(aabb))
		{
			return false;
		}

		this.RemoveLeaf(proxyId);

		// Extend AABB.
		var b = aabb.Clone();
		var r = new b2Vec2(b2_aabbExtension, b2_aabbExtension);
		b.lowerBound.Assign(b2Vec2.Subtract(b.lowerBound, r));
		b.upperBound.Assign(b2Vec2.Add(b.upperBound, r));

		// Predict AABB displacement.
		var d = b2Vec2.Multiply(b2_aabbMultiplier, displacement);

		if (d.x < 0.0)
		{
			b.lowerBound.x += d.x;
		}
		else
		{
			b.upperBound.x += d.x;
		}

		if (d.y < 0.0)
		{
			b.lowerBound.y += d.y;
		}
		else
		{
			b.upperBound.y += d.y;
		}

		this.m_nodes[proxyId].aabb.Assign(b);

		this.InsertLeaf(proxyId);
		return true;
	},

	/// Get proxy user data.
	/// @return the proxy user data or 0 if the id is invalid.
	GetUserData: function(proxyId)
	{
		b2Assert(0 <= proxyId && proxyId < this.m_nodeCapacity);
		return this.m_nodes[proxyId].userData;
	},

	/// Get the fat AABB for a proxy.
	GetFatAABB: function(proxyId)
	{
		b2Assert(0 <= proxyId && proxyId < this.m_nodeCapacity);
		return this.m_nodes[proxyId].aabb;
	},

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	Query: function(callback, aabb)
	{
		var stack = [];
		stack.push(this.m_root);

		while (stack.length > 0)
		{
			var nodeId = stack.pop();
			if (nodeId == b2_nullNode)
			{
				continue;
			}

			var node = this.m_nodes[nodeId];

			if (b2TestOverlap(node.aabb, aabb))
			{
				if (node.IsLeaf())
				{
					var proceed = callback.QueryCallback(nodeId);
					if (proceed == false)
					{
						return;
					}
				}
				else
				{
					stack.push(node.child1);
					stack.push(node.child2);
				}
			}
		}
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
		var p1 = input.p1;
		var p2 = input.p2;
		var r = b2Vec2.Subtract(p2, p1);
		b2Assert(r.LengthSquared() > 0.0);
		r.Normalize();

		// v is perpendicular to the segment.
		var v = b2Cross_f_v2(1.0, r);
		var abs_v = b2Abs_v2(v);

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)

		var maxFraction = input.maxFraction;

		// Build a bounding box for the segment.
		var segmentAABB = new b2AABB();
		{
			var t = b2Vec2.Add(p1, b2Vec2.Multiply(maxFraction, b2Vec2.Subtract(p2, p1)));
			segmentAABB.lowerBound.Assign(b2Min_v2(p1, t));
			segmentAABB.upperBound.Assign(b2Max_v2(p1, t));
		}

		var stack = [];
		stack.push(this.m_root);

		while (stack.length > 0)
		{
			var nodeId = stack.pop();
			if (nodeId == b2_nullNode)
			{
				continue;
			}

			var node = this.m_nodes[nodeId];

			if (b2TestOverlap(node.aabb, segmentAABB) == false)
			{
				continue;
			}

			// Separating axis for segment (Gino, p80).
			// |dot(v, p1 - c)| > dot(|v|, h)
			var c = node.aabb.GetCenter();
			var h = node.aabb.GetExtents();
			var separation = b2Abs(b2Dot_v2_v2(v, b2Vec2.Subtract(p1, c))) - b2Dot_v2_v2(abs_v, h);
			if (separation > 0.0)
			{
				continue;
			}

			if (node.IsLeaf())
			{
				var subInput = new b2RayCastInput();
				subInput.p1.Assign(input.p1);
				subInput.p2.Assign(input.p2);
				subInput.maxFraction = maxFraction;

				var value = callback.RayCastCallback(subInput, nodeId);

				if (value == 0.0)
				{
					// The client has terminated the ray cast.
					return;
				}

				if (value > 0.0)
				{
					// Update segment bounding box.
					maxFraction = value;
					var t = b2Vec2.Add(p1, b2Vec2.Multiply(maxFraction, b2Vec2.Subtract(p2, p1)));
					segmentAABB.lowerBound.Assign(b2Min_v2(p1, t));
					segmentAABB.upperBound.Assign(b2Max_v2(p1, t));
				}
			}
			else
			{
				stack.push(node.child1);
				stack.push(node.child2);
			}
		}
	},

	/// Validate this tree. For testing.
	Validate: function()
	{
		this.ValidateStructure(this.m_root);
		this.ValidateMetrics(this.m_root);

		var freeCount = 0;
		var freeIndex = this.m_freeList;
		while (freeIndex != b2_nullNode)
		{
			b2Assert(0 <= freeIndex && freeIndex < this.m_nodeCapacity);
			freeIndex = this.m_nodes[freeIndex].parent;
			++freeCount;
		}

		b2Assert(this.GetHeight() == this.ComputeHeight());

		b2Assert(this.m_nodeCount + freeCount == this.m_nodeCapacity);
	},

	/// Compute the height of the binary tree in O(N) time. Should not be
	/// called often.
	GetHeight: function()
	{
		if (this.m_root == b2_nullNode)
		{
			return 0;
		}

		return this.m_nodes[this.m_root].height;
	},

	/// Get the maximum balance of an node in the tree. The balance is the difference
	/// in height of the two children of a node.
	GetMaxBalance: function()
	{
		var maxBalance = 0;
		for (var i = 0; i < this.m_nodeCapacity; ++i)
		{
			var node = this.m_nodes[i];
			if (node.height <= 1)
			{
				continue;
			}

			b2Assert(node.IsLeaf() == false);

			var child1 = node.child1;
			var child2 = node.child2;
			var balance = b2Abs(this.m_nodes[child2].height - this.m_nodes[child1].height);
			maxBalance = b2Max(maxBalance, balance);
		}

		return maxBalance;
	},

	/// Get the ratio of the sum of the node areas to the root area.
	GetAreaRatio: function()
	{
		if (this.m_root == b2_nullNode)
		{
			return 0.0;
		}

		var root = this.m_nodes[this.m_root];
		var rootArea = root.aabb.GetPerimeter();

		var totalArea = 0.0;
		for (var i = 0; i < this.m_nodeCapacity; ++i)
		{
			var node = this.m_nodes[i];
			if (node.height < 0)
			{
				// Free node in pool
				continue;
			}

			totalArea += node.aabb.GetPerimeter();
		}

		return totalArea / rootArea;
	},

	/// Build an optimal tree. Very expensive. For testing.
	RebuildBottomUp: function()
	{
		var nodes = new Array(this.m_nodeCount);
		var count = 0;

		// Build array of leaves. Free the rest.
		for (var i = 0; i < this.m_nodeCapacity; ++i)
		{
			if (this.m_nodes[i].height < 0)
			{
				// free node in pool
				continue;
			}

			if (this.m_nodes[i].IsLeaf())
			{
				this.m_nodes[i].parent = b2_nullNode;
				nodes[count] = i;
				++count;
			}
			else
			{
				this.FreeNode(i);
			}
		}

		while (count > 1)
		{
			var minCost = b2_maxFloat;
			var iMin = -1, jMin = -1;
			for (i = 0; i < count; ++i)
			{
				var aabbi = this.m_nodes[nodes[i]].aabb;

				for (var j = i + 1; j < count; ++j)
				{
					var aabbj = this.m_nodes[nodes[j]].aabb;
					var b = new b2AABB();
					b.Combine(aabbi, aabbj);
					var cost = b.GetPerimeter();
					if (cost < minCost)
					{
						iMin = i;
						jMin = j;
						minCost = cost;
					}
				}
			}

			var index1 = nodes[iMin];
			var index2 = nodes[jMin];
			var child1 = this.m_nodes[index1];
			var child2 = this.m_nodes[index2];

			var parentIndex = this.AllocateNode();
			var parent = this.m_nodes[parentIndex];
			parent.child1 = index1;
			parent.child2 = index2;
			parent.height = 1 + b2Max(child1.height, child2.height);
			parent.aabb.Combine(child1.aabb, child2.aabb);
			parent.parent = b2_nullNode;

			child1.parent = parentIndex;
			child2.parent = parentIndex;

			nodes[jMin] = nodes[count-1];
			nodes[iMin] = parentIndex;
			--count;
		}

		this.m_root = nodes[0];

		this.Validate();
	},

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	ShiftOrigin: function(newOrigin)
	{
		// Build array of leaves. Free the rest.
		for (var i = 0; i < this.m_nodeCapacity; ++i)
		{
			this.m_nodes[i].aabb.lowerBound.Subtract(newOrigin);
			this.m_nodes[i].aabb.upperBound.Subtract(newOrigin);
		}
	},

	AllocateNode: function()
	{
		// Expand the node pool as needed.
		if (this.m_freeList == b2_nullNode)
		{
			b2Assert(this.m_nodeCount == this.m_nodeCapacity);

			// The free list is empty. Rebuild a bigger pool.
			var oldNodes = this.m_nodes;
			this.m_nodeCapacity *= 2;
			this.m_nodes = oldNodes.concat(new Array(this.m_nodeCapacity - this.m_nodeCount));

			// Build a linked list for the free list. The parent
			// pointer becomes the "next" pointer.
			for (var i = this.m_nodeCount; i < this.m_nodeCapacity - 1; ++i)
			{
				this.m_nodes[i] = new b2TreeNode();
				this.m_nodes[i].parent = i + 1;
				this.m_nodes[i].height = -1;
			}
			this.m_nodes[this.m_nodeCapacity - 1] = new b2TreeNode();
			this.m_nodes[this.m_nodeCapacity - 1].parent = b2_nullNode;
			this.m_nodes[this.m_nodeCapacity-1].height = -1;
			this.m_freeList = this.m_nodeCount;
		}

		// Peel a node off the free list.
		var nodeId = this.m_freeList;
		this.m_freeList = this.m_nodes[nodeId].parent;
		this.m_nodes[nodeId].parent = b2_nullNode;
		this.m_nodes[nodeId].child1 = b2_nullNode;
		this.m_nodes[nodeId].child2 = b2_nullNode;
		this.m_nodes[nodeId].height = 0;
		this.m_nodes[nodeId].userData = null;
		++this.m_nodeCount;
		return nodeId;
	},

	FreeNode: function(nodeId)
	{
		b2Assert(0 <= nodeId && nodeId < this.m_nodeCapacity);
		b2Assert(0 < this.m_nodeCount);
		this.m_nodes[nodeId].parent = this.m_freeList;
		this.m_nodes[nodeId].height = -1;
		this.m_freeList = nodeId;
		--this.m_nodeCount;
	},

	InsertLeaf: function(leaf)
	{
		++this.m_insertionCount;

		if (this.m_root == b2_nullNode)
		{
			this.m_root = leaf;
			this.m_nodes[this.m_root].parent = b2_nullNode;
			return;
		}

		// Find the best sibling for this node
		var leafAABB = this.m_nodes[leaf].aabb;
		var index = this.m_root;

		while (this.m_nodes[index].IsLeaf() == false)
		{
			var child1 = this.m_nodes[index].child1;
			var child2 = this.m_nodes[index].child2;

			var area = this.m_nodes[index].aabb.GetPerimeter();

			var combinedAABB = new b2AABB();
			combinedAABB.Combine(this.m_nodes[index].aabb, leafAABB);
			var combinedArea = combinedAABB.GetPerimeter();

			// Cost of creating a new parent for this node and the new leaf
			var cost = 2.0 * combinedArea;

			// Minimum cost of pushing the leaf further down the tree
			var inheritanceCost = 2.0 * (combinedArea - area);

			// Cost of descending into child1
			var cost1;
			var aabb;

			if (this.m_nodes[child1].IsLeaf())
			{
				aabb = new b2AABB();
				aabb.Combine(leafAABB, this.m_nodes[child1].aabb);
				cost1 = aabb.GetPerimeter() + inheritanceCost;
			}
			else
			{
				aabb = new b2AABB();
				aabb.Combine(leafAABB, this.m_nodes[child1].aabb);
				var oldArea = this.m_nodes[child1].aabb.GetPerimeter();
				var newArea = aabb.GetPerimeter();
				cost1 = (newArea - oldArea) + inheritanceCost;
			}

			// Cost of descending into child2
			var cost2;
			if (this.m_nodes[child2].IsLeaf())
			{
				aabb = new b2AABB();
				aabb.Combine(leafAABB, this.m_nodes[child2].aabb);
				cost2 = aabb.GetPerimeter() + inheritanceCost;
			}
			else
			{
				aabb = new b2AABB();
				aabb.Combine(leafAABB, this.m_nodes[child2].aabb);
				var oldArea = this.m_nodes[child2].aabb.GetPerimeter();
				var newArea = aabb.GetPerimeter();
				cost2 = newArea - oldArea + inheritanceCost;
			}

			// Descend according to the minimum cost.
			if (cost < cost1 && cost < cost2)
			{
				break;
			}

			// Descend
			if (cost1 < cost2)
			{
				index = child1;
			}
			else
			{
				index = child2;
			}
		}

		var sibling = index;

		// Create a new parent.
		var oldParent = this.m_nodes[sibling].parent;
		var newParent = this.AllocateNode();
		this.m_nodes[newParent].parent = oldParent;
		this.m_nodes[newParent].userData = null;
		this.m_nodes[newParent].aabb.Combine(leafAABB, this.m_nodes[sibling].aabb);
		this.m_nodes[newParent].height = this.m_nodes[sibling].height + 1;

		if (oldParent != b2_nullNode)
		{
			// The sibling was not the root.
			if (this.m_nodes[oldParent].child1 == sibling)
			{
				this.m_nodes[oldParent].child1 = newParent;
			}
			else
			{
				this.m_nodes[oldParent].child2 = newParent;
			}

			this.m_nodes[newParent].child1 = sibling;
			this.m_nodes[newParent].child2 = leaf;
			this.m_nodes[sibling].parent = newParent;
			this.m_nodes[leaf].parent = newParent;
		}
		else
		{
			// The sibling was the root.
			this.m_nodes[newParent].child1 = sibling;
			this.m_nodes[newParent].child2 = leaf;
			this.m_nodes[sibling].parent = newParent;
			this.m_nodes[leaf].parent = newParent;
			this.m_root = newParent;
		}

		// Walk back up the tree fixing heights and AABBs
		index = this.m_nodes[leaf].parent;
		while (index != b2_nullNode)
		{
			index = this.Balance(index);

			var child1 = this.m_nodes[index].child1;
			var child2 = this.m_nodes[index].child2;

			b2Assert(child1 != b2_nullNode);
			b2Assert(child2 != b2_nullNode);

			this.m_nodes[index].height = 1 + b2Max(this.m_nodes[child1].height, this.m_nodes[child2].height);
			this.m_nodes[index].aabb.Combine(this.m_nodes[child1].aabb, this.m_nodes[child2].aabb);

			index = this.m_nodes[index].parent;
		}

		//this.Validate();
	},
	RemoveLeaf: function(leaf)
	{
		if (leaf == this.m_root)
		{
			this.m_root = b2_nullNode;
			return;
		}

		var parent = this.m_nodes[leaf].parent;
		var grandParent = this.m_nodes[parent].parent;
		var sibling;
		if (this.m_nodes[parent].child1 == leaf)
		{
			sibling = this.m_nodes[parent].child2;
		}
		else
		{
			sibling = this.m_nodes[parent].child1;
		}

		if (grandParent != b2_nullNode)
		{
			// Destroy parent and connect sibling to grandParent.
			if (this.m_nodes[grandParent].child1 == parent)
			{
				this.m_nodes[grandParent].child1 = sibling;
			}
			else
			{
				this.m_nodes[grandParent].child2 = sibling;
			}
			this.m_nodes[sibling].parent = grandParent;
			this.FreeNode(parent);

			// Adjust ancestor bounds.
			var index = grandParent;
			while (index != b2_nullNode)
			{
				index = this.Balance(index);

				var child1 = this.m_nodes[index].child1;
				var child2 = this.m_nodes[index].child2;

				this.m_nodes[index].aabb.Combine(this.m_nodes[child1].aabb, this.m_nodes[child2].aabb);
				this.m_nodes[index].height = 1 + b2Max(this.m_nodes[child1].height, this.m_nodes[child2].height);

				index = this.m_nodes[index].parent;
			}
		}
		else
		{
			this.m_root = sibling;
			this.m_nodes[sibling].parent = b2_nullNode;
			this.FreeNode(parent);
		}

		//this.Validate();
	},

	Balance: function(iA)
	{
		b2Assert(iA != b2_nullNode);

		var A = this.m_nodes[iA];
		if (A.IsLeaf() || A.height < 2)
		{
			return iA;
		}

		var iB = A.child1;
		var iC = A.child2;
		b2Assert(0 <= iB && iB < this.m_nodeCapacity);
		b2Assert(0 <= iC && iC < this.m_nodeCapacity);

		var B = this.m_nodes[iB];
		var C = this.m_nodes[iC];

		var balance = C.height - B.height;

		// Rotate C up
		if (balance > 1)
		{
			var iF = C.child1;
			var iG = C.child2;
			var F = this.m_nodes[iF];
			var G = this.m_nodes[iG];
			b2Assert(0 <= iF && iF < this.m_nodeCapacity);
			b2Assert(0 <= iG && iG < this.m_nodeCapacity);

			// Swap A and C
			C.child1 = iA;
			C.parent = A.parent;
			A.parent = iC;

			// A's old parent should point to C
			if (C.parent != b2_nullNode)
			{
				if (this.m_nodes[C.parent].child1 == iA)
				{
					this.m_nodes[C.parent].child1 = iC;
				}
				else
				{
					b2Assert(this.m_nodes[C.parent].child2 == iA);
					this.m_nodes[C.parent].child2 = iC;
				}
			}
			else
			{
				this.m_root = iC;
			}

			// Rotate
			if (F.height > G.height)
			{
				C.child2 = iF;
				A.child2 = iG;
				G.parent = iA;
				A.aabb.Combine(B.aabb, G.aabb);
				C.aabb.Combine(A.aabb, F.aabb);

				A.height = 1 + b2Max(B.height, G.height);
				C.height = 1 + b2Max(A.height, F.height);
			}
			else
			{
				C.child2 = iG;
				A.child2 = iF;
				F.parent = iA;
				A.aabb.Combine(B.aabb, F.aabb);
				C.aabb.Combine(A.aabb, G.aabb);

				A.height = 1 + b2Max(B.height, F.height);
				C.height = 1 + b2Max(A.height, G.height);
			}

			return iC;
		}

		// Rotate B up
		if (balance < -1)
		{
			var iD = B.child1;
			var iE = B.child2;
			var D = this.m_nodes[iD];
			var E = this.m_nodes[iE];
			b2Assert(0 <= iD && iD < this.m_nodeCapacity);
			b2Assert(0 <= iE && iE < this.m_nodeCapacity);

			// Swap A and B
			B.child1 = iA;
			B.parent = A.parent;
			A.parent = iB;

			// A's old parent should point to B
			if (B.parent != b2_nullNode)
			{
				if (this.m_nodes[B.parent].child1 == iA)
				{
					this.m_nodes[B.parent].child1 = iB;
				}
				else
				{
					b2Assert(this.m_nodes[B.parent].child2 == iA);
					this.m_nodes[B.parent].child2 = iB;
				}
			}
			else
			{
				this.m_root = iB;
			}

			// Rotate
			if (D.height > E.height)
			{
				B.child2 = iD;
				A.child1 = iE;
				E.parent = iA;
				A.aabb.Combine(C.aabb, E.aabb);
				B.aabb.Combine(A.aabb, D.aabb);

				A.height = 1 + b2Max(C.height, E.height);
				B.height = 1 + b2Max(A.height, D.height);
			}
			else
			{
				B.child2 = iE;
				A.child1 = iD;
				D.parent = iA;
				A.aabb.Combine(C.aabb, D.aabb);
				B.aabb.Combine(A.aabb, E.aabb);

				A.height = 1 + b2Max(C.height, D.height);
				B.height = 1 + b2Max(A.height, E.height);
			}

			return iB;
		}

		return iA;
	},

	ComputeHeight: function(nodeId)
	{
		if (typeof(nodeId) === 'undefined')
			nodeId = this.m_root;

		b2Assert(0 <= nodeId && nodeId < this.m_nodeCapacity);
		var node = this.m_nodes[nodeId];

		if (node.IsLeaf())
		{
			return 0;
		}

		var height1 = this.ComputeHeight(node.child1);
		var height2 = this.ComputeHeight(node.child2);
		return 1 + b2Max(height1, height2);
	},

	ValidateStructure: function(index)
	{
		if (index == b2_nullNode)
		{
			return;
		}

		if (index == this.m_root)
		{
			b2Assert(this.m_nodes[index].parent == b2_nullNode);
		}

		var node = this.m_nodes[index];

		var child1 = node.child1;
		var child2 = node.child2;

		if (node.IsLeaf())
		{
			b2Assert(child1 == b2_nullNode);
			b2Assert(child2 == b2_nullNode);
			b2Assert(node.height == 0);
			return;
		}

		b2Assert(0 <= child1 && child1 < this.m_nodeCapacity);
		b2Assert(0 <= child2 && child2 < this.m_nodeCapacity);

		b2Assert(this.m_nodes[child1].parent == index);
		b2Assert(this.m_nodes[child2].parent == index);

		this.ValidateStructure(child1);
		this.ValidateStructure(child2);
	},
	ValidateMetrics: function(index)
	{
		if (index == b2_nullNode)
		{
			return;
		}

		var node = this.m_nodes[index];

		var child1 = node.child1;
		var child2 = node.child2;

		if (node.IsLeaf())
		{
			b2Assert(child1 == b2_nullNode);
			b2Assert(child2 == b2_nullNode);
			b2Assert(node.height == 0);
			return;
		}

		b2Assert(0 <= child1 && child1 < this.m_nodeCapacity);
		b2Assert(0 <= child2 && child2 < this.m_nodeCapacity);

		var height1 = this.m_nodes[child1].height;
		var height2 = this.m_nodes[child2].height;
		var height;
		height = 1 + b2Max(height1, height2);
		b2Assert(node.height == height);

		var aabb = new b2AABB();
		aabb.Combine(this.m_nodes[child1].aabb, this.m_nodes[child2].aabb);

		b2Assert(b2Vec2.Equals(aabb.lowerBound, node.aabb.lowerBound));
		b2Assert(b2Vec2.Equals(aabb.upperBound, node.aabb.upperBound));

		this.ValidateMetrics(child1);
		this.ValidateMetrics(child2);
	}
};