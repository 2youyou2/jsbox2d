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

#ifndef DYNAMIC_TREE_TEST_H
#define DYNAMIC_TREE_TEST_H

class DynamicTreeTest : public Test
{
public:

	enum
	{
		e_actorCount = 128
	};

	DynamicTreeTest()
	{
		this->m_worldExtent = 15.0;
		this->m_proxyExtent = 0.5;

		srand(888);

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			Actor* actor = this->m_actors + i;
			GetRandomAABB(&actor->aabb);
			actor->proxyId = this->m_tree.CreateProxy(actor->aabb, actor);
		}

		this->m_stepCount = 0;

		float32 h = this->m_worldExtent;
		this->m_queryAABB.lowerBound.Set(-3.0, -4.0 + h);
		this->m_queryAABB.upperBound.Set(5.0, 6.0 + h);

		this->m_rayCastInput.p1.Set(-5.0, 5.0 + h);
		this->m_rayCastInput.p2.Set(7.0, -4.0 + h);
		//this->m_rayCastInput.p1.Set(0.0, 2.0 + h);
		//this->m_rayCastInput.p2.Set(0.0, -2.0 + h);
		this->m_rayCastInput.maxFraction = 1.0;

		this->m_automated = false;
	}

	static Test* Create()
	{
		return new DynamicTreeTest;
	}

	void Step(Settings* settings)
	{
		B2_NOT_USED(settings);

		this->m_rayActor = null;
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			this->m_actors[i].fraction = 1.0;
			this->m_actors[i].overlap = false;
		}

		if (this->m_automated == true)
		{
			int32 actionCount = b2Max(1, e_actorCount >> 2);

			for (int32 i = 0; i < actionCount; ++i)
			{
				Action();
			}
		}

		Query();
		RayCast();

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			Actor* actor = this->m_actors + i;
			if (actor->proxyId == b2_nullNode)
				continue;

			b2Color c(0.9, 0.9, 0.9);
			if (actor == this->m_rayActor && actor->overlap)
			{
				c.Set(0.9, 0.6, 0.6);
			}
			else if (actor == this->m_rayActor)
			{
				c.Set(0.6, 0.9, 0.6);
			}
			else if (actor->overlap)
			{
				c.Set(0.6, 0.6, 0.9);
			}

			g_debugDraw.DrawAABB(&actor->aabb, c);
		}

		b2Color c(0.7, 0.7, 0.7);
		g_debugDraw.DrawAABB(&this->m_queryAABB, c);

		g_debugDraw.DrawSegment(this->m_rayCastInput.p1, this->m_rayCastInput.p2, c);

		b2Color c1(0.2, 0.9, 0.2);
		b2Color c2(0.9, 0.2, 0.2);
		g_debugDraw.DrawPoint(this->m_rayCastInput.p1, 6.0, c1);
		g_debugDraw.DrawPoint(this->m_rayCastInput.p2, 6.0, c2);

		if (this->m_rayActor)
		{
			b2Color cr(0.2, 0.2, 0.9);
			b2Vec2 p = b2Vec2::Add(this->m_rayCastInput.p1, b2Vec2::Multiply(this->m_rayActor->fraction, b2Vec2::Subtract(this->m_rayCastInput.p2, this->m_rayCastInput.p1)));
			g_debugDraw.DrawPoint(p, 6.0, cr);
		}

		{
			int32 height = this->m_tree.GetHeight();
			g_debugDraw.DrawString(5, this->m_textLine, "dynamic tree height = %d", height);
			this->m_textLine += DRAW_STRING_NEW_LINE;
		}

		++this->m_stepCount;
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			this->m_automated = !this->m_automated;
			break;

		case GLFW_KEY_C:
			CreateProxy();
			break;

		case GLFW_KEY_D:
			DestroyProxy();
			break;

		case GLFW_KEY_M:
			MoveProxy();
			break;
		}
	}

	bool QueryCallback(int32 proxyId)
	{
		Actor* actor = (Actor*)this->m_tree.GetUserData(proxyId);
		actor->overlap = b2TestOverlap(this->m_queryAABB, actor->aabb);
		return true;
	}

	float32 RayCastCallback(const b2RayCastInput& input, int32 proxyId)
	{
		Actor* actor = (Actor*)this->m_tree.GetUserData(proxyId);

		b2RayCastOutput output;
		bool hit = actor->aabb.RayCast(&output, input);

		if (hit)
		{
			this->m_rayCastOutput = output;
			this->m_rayActor = actor;
			this->m_rayActor->fraction = output.fraction;
			return output.fraction;
		}

		return input.maxFraction;
	}

private:

	struct Actor
	{
		b2AABB aabb;
		float32 fraction;
		bool overlap;
		int32 proxyId;
	};

	void GetRandomAABB(b2AABB* aabb)
	{
		b2Vec2 w; w.Set(2.0 * this->m_proxyExtent, 2.0 * this->m_proxyExtent);
		//aabb->lowerBound.x = -this->m_proxyExtent;
		//aabb->lowerBound.y = -this->m_proxyExtent + this->m_worldExtent;
		aabb->lowerBound.x = RandomFloat(-this->m_worldExtent, this->m_worldExtent);
		aabb->lowerBound.y = RandomFloat(0.0, 2.0 * this->m_worldExtent);
		aabb->upperBound = b2Vec2::Add(aabb->lowerBound, w);
	}

	void MoveAABB(b2AABB* aabb)
	{
		b2Vec2 d;
		d.x = RandomFloat(-0.5, 0.5);
		d.y = RandomFloat(-0.5, 0.5);
		//d.x = 2.0;
		//d.y = 0.0;
		aabb->lowerBound.Add(d);
		aabb->upperBound.Add(d);

		b2Vec2 c0 = b2Vec2::Multiply(0.5, b2Vec2::Add(aabb->lowerBound, aabb->upperBound));
		b2Vec2 min; min.Set(-this->m_worldExtent, 0.0);
		b2Vec2 max; max.Set(this->m_worldExtent, 2.0 * this->m_worldExtent);
		b2Vec2 c = b2Clamp_v2(c0, min, max);

		aabb->lowerBound.Add(b2Vec2::Subtract(c, c0));
		aabb->upperBound.Add(b2Vec2::Subtract(c, c0));
	}

	void CreateProxy()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = this->m_actors + j;
			if (actor->proxyId == b2_nullNode)
			{
				GetRandomAABB(&actor->aabb);
				actor->proxyId = this->m_tree.CreateProxy(actor->aabb, actor);
				return;
			}
		}
	}

	void DestroyProxy()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = this->m_actors + j;
			if (actor->proxyId != b2_nullNode)
			{
				this->m_tree.DestroyProxy(actor->proxyId);
				actor->proxyId = b2_nullNode;
				return;
			}
		}
	}

	void MoveProxy()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = this->m_actors + j;
			if (actor->proxyId == b2_nullNode)
			{
				continue;
			}

			b2AABB aabb0 = actor->aabb;
			MoveAABB(&actor->aabb);
			b2Vec2 displacement = b2Vec2::Subtract(actor->aabb.GetCenter(), aabb0.GetCenter());
			this->m_tree.MoveProxy(actor->proxyId, actor->aabb, displacement);
			return;
		}
	}

	void Action()
	{
		int32 choice = rand() % 20;

		switch (choice)
		{
		case 0:
			CreateProxy();
			break;

		case 1:
			DestroyProxy();
			break;

		default:
			MoveProxy();
		}
	}

	void Query()
	{
		this->m_tree.Query(this, this->m_queryAABB);

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			if (this->m_actors[i].proxyId == b2_nullNode)
			{
				continue;
			}

			bool overlap = b2TestOverlap(this->m_queryAABB, this->m_actors[i].aabb);
			B2_NOT_USED(overlap);
			b2Assert(overlap == this->m_actors[i].overlap);
		}
	}

	void RayCast()
	{
		this->m_rayActor = null;

		b2RayCastInput input = this->m_rayCastInput;

		// Ray cast against the dynamic tree.
		this->m_tree.RayCast(this, input);

		// Brute force ray cast.
		Actor* bruteActor = null;
		b2RayCastOutput bruteOutput;
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			if (this->m_actors[i].proxyId == b2_nullNode)
			{
				continue;
			}

			b2RayCastOutput output;
			bool hit = this->m_actors[i].aabb.RayCast(&output, input);
			if (hit)
			{
				bruteActor = this->m_actors + i;
				bruteOutput = output;
				input.maxFraction = output.fraction;
			}
		}

		if (bruteActor != null)
		{
			b2Assert(bruteOutput.fraction == this->m_rayCastOutput.fraction);
		}
	}

	float32 m_worldExtent;
	float32 m_proxyExtent;

	b2DynamicTree m_tree;
	b2AABB m_queryAABB;
	b2RayCastInput m_rayCastInput;
	b2RayCastOutput m_rayCastOutput;
	Actor* m_rayActor;
	Actor m_actors[e_actorCount];
	int32 m_stepCount;
	bool m_automated;
};

#endif
