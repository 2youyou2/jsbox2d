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

#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

class ApplyForce : public Test
{
public:
	ApplyForce()
	{
		this->m_world->SetGravity(b2Vec2(0.0, 0.0));

		const float32 k_restitution = 0.4;

		b2Body* ground;
		{
			b2BodyDef bd;
			bd.position.Set(0.0, 20.0);
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef sd;
			sd.shape = &shape;
			sd.density = 0.0;
			sd.restitution = k_restitution;

			// Left vertical
			shape.Set(b2Vec2(-20.0, -20.0), b2Vec2(-20.0, 20.0));
			ground->CreateFixture(&sd);

			// Right vertical
			shape.Set(b2Vec2(20.0, -20.0), b2Vec2(20.0, 20.0));
			ground->CreateFixture(&sd);

			// Top horizontal
			shape.Set(b2Vec2(-20.0, 20.0), b2Vec2(20.0, 20.0));
			ground->CreateFixture(&sd);

			// Bottom horizontal
			shape.Set(b2Vec2(-20.0, -20.0), b2Vec2(20.0, -20.0));
			ground->CreateFixture(&sd);
		}

		{
			b2Transform xf1;
			xf1.q.Set(0.3524f * b2_pi);
			xf1.p = xf1.q.GetXAxis();

			b2Vec2 vertices[3];
			vertices[0] = b2Mul_t_v2(xf1, b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf1, b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf1, b2Vec2(0.0, 0.5));
			
			b2PolygonShape poly1;
			poly1.Set(vertices, 3);

			b2FixtureDef sd1;
			sd1.shape = &poly1;
			sd1.density = 4.0;

			b2Transform xf2;
			xf2.q.Set(-0.3524f * b2_pi);
			xf2.p = xf2.q.GetXAxis().Negate();

			vertices[0] = b2Mul_t_v2(xf2, b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf2, b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf2, b2Vec2(0.0, 0.5));
			
			b2PolygonShape poly2;
			poly2.Set(vertices, 3);

			b2FixtureDef sd2;
			sd2.shape = &poly2;
			sd2.density = 2.0;

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.angularDamping = 2.0;
			bd.linearDamping = 0.5;

			bd.position.Set(0.0, 2.0);
			bd.angle = b2_pi;
			bd.allowSleep = false;
			this->m_body = this->m_world->CreateBody(&bd);
			this->m_body->CreateFixture(&sd1);
			this->m_body->CreateFixture(&sd2);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.5, 0.5);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0;
			fd.friction = 0.3;

			for (int i = 0; i < 10; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;

				bd.position.Set(0.0, 5.0 + 1.54f * i);
				b2Body* body = this->m_world->CreateBody(&bd);

				body->CreateFixture(&fd);

				float32 gravity = 10.0;
				float32 I = body->GetInertia();
				float32 mass = body->GetMass();

				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				float32 radius = b2Sqrt(2.0 * I / mass);

				b2FrictionJointDef jd;
				jd.localAnchorA.SetZero();
				jd.localAnchorB.SetZero();
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = mass * radius * gravity;

				this->m_world->CreateJoint(&jd);
			}
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_W:
			{
				b2Vec2 f = this->m_body->GetWorldVector(b2Vec2(0.0, -200.0));
				b2Vec2 p = this->m_body->GetWorldPoint(b2Vec2(0.0, 2.0));
				this->m_body->ApplyForce(f, p, true);
			}
			break;

		case GLFW_KEY_A:
			{
				this->m_body->ApplyTorque(50.0, true);
			}
			break;

		case GLFW_KEY_D:
			{
				this->m_body->ApplyTorque(-50.0, true);
			}
			break;
		}
	}

	static Test* Create()
	{
		return new ApplyForce;
	}

	b2Body* m_body;
};

#endif
