/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef WEB_H
#define WEB_H

// This tests distance joints, body destruction, and joint destruction.
class Web : public Test
{
public:
	Web()
	{
		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.5, 0.5);

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;

			bd.position.Set(-5.0, 5.0);
			this->m_bodies[0] = this->m_world->CreateBody(&bd);
			this->m_bodies[0]->CreateFixture(&shape, 5.0);

			bd.position.Set(5.0, 5.0);
			this->m_bodies[1] = this->m_world->CreateBody(&bd);
			this->m_bodies[1]->CreateFixture(&shape, 5.0);

			bd.position.Set(5.0, 15.0);
			this->m_bodies[2] = this->m_world->CreateBody(&bd);
			this->m_bodies[2]->CreateFixture(&shape, 5.0);

			bd.position.Set(-5.0, 15.0);
			this->m_bodies[3] = this->m_world->CreateBody(&bd);
			this->m_bodies[3]->CreateFixture(&shape, 5.0);

			b2DistanceJointDef jd;
			b2Vec2 p1, p2, d;

			jd.frequencyHz = 2.0;
			jd.dampingRatio = 0.0;

			jd.bodyA = ground;
			jd.bodyB = this->m_bodies[0];
			jd.localAnchorA.Set(-10.0, 0.0);
			jd.localAnchorB.Set(-0.5, -0.5);
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[0] = this->m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = this->m_bodies[1];
			jd.localAnchorA.Set(10.0, 0.0);
			jd.localAnchorB.Set(0.5, -0.5);
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[1] = this->m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = this->m_bodies[2];
			jd.localAnchorA.Set(10.0, 20.0);
			jd.localAnchorB.Set(0.5, 0.5);
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[2] = this->m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = this->m_bodies[3];
			jd.localAnchorA.Set(-10.0, 20.0);
			jd.localAnchorB.Set(-0.5, 0.5);
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[3] = this->m_world->CreateJoint(&jd);

			jd.bodyA = this->m_bodies[0];
			jd.bodyB = this->m_bodies[1];
			jd.localAnchorA.Set(0.5, 0.0);
			jd.localAnchorB.Set(-0.5, 0.0);;
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[4] = this->m_world->CreateJoint(&jd);

			jd.bodyA = this->m_bodies[1];
			jd.bodyB = this->m_bodies[2];
			jd.localAnchorA.Set(0.0, 0.5);
			jd.localAnchorB.Set(0.0, -0.5);
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[5] = this->m_world->CreateJoint(&jd);

			jd.bodyA = this->m_bodies[2];
			jd.bodyB = this->m_bodies[3];
			jd.localAnchorA.Set(-0.5, 0.0);
			jd.localAnchorB.Set(0.5, 0.0);
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[6] = this->m_world->CreateJoint(&jd);

			jd.bodyA = this->m_bodies[3];
			jd.bodyB = this->m_bodies[0];
			jd.localAnchorA.Set(0.0, -0.5);
			jd.localAnchorB.Set(0.0, 0.5);
			p1.Assign(jd.bodyA->GetWorldPoint(jd.localAnchorA));
			p2.Assign(jd.bodyB->GetWorldPoint(jd.localAnchorB));
			d.Assign(b2Vec2::Subtract(p2, p1));
			jd.length = d.Length();
			this->m_joints[7] = this->m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_B:
			for (int32 i = 0; i < 4; ++i)
			{
				if (this->m_bodies[i])
				{
					this->m_world->DestroyBody(this->m_bodies[i]);
					this->m_bodies[i] = null;
					break;
				}
			}
			break;

		case GLFW_KEY_J:
			for (int32 i = 0; i < 8; ++i)
			{
				if (this->m_joints[i])
				{
					this->m_world->DestroyJoint(this->m_joints[i]);
					this->m_joints[i] = null;
					break;
				}
			}
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "This demonstrates a soft distance joint.");
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "Press: (b) to delete a body, (j) to delete a joint");
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	void JointDestroyed(b2Joint* joint)
	{
		for (int32 i = 0; i < 8; ++i)
		{
			if (this->m_joints[i] == joint)
			{
				this->m_joints[i] = null;
				break;
			}
		}
	}

	static Test* Create()
	{
		return new Web;
	}

	b2Body* m_bodies[4];
	b2Joint* m_joints[8];
};

#endif
