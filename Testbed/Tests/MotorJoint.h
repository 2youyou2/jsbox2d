/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef MOTOR_JOINT_H
#define MOTOR_JOINT_H

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
class MotorJoint : public Test
{
public:
	MotorJoint()
	{
		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-20.0, 0.0), b2Vec2(20.0, 0.0));

			b2FixtureDef fd;
			fd.shape = &shape;

			ground->CreateFixture(&fd);
		}

		// Define motorized body
		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(0.0, 8.0);
			b2Body* body = this->m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(2.0, 0.5);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6;
			fd.density = 2.0;
			body->CreateFixture(&fd);

			b2MotorJointDef mjd;
			mjd.Initialize(ground, body);
			mjd.maxForce = 1000.0;
			mjd.maxTorque = 1000.0;
			this->m_joint = (b2MotorJoint*)this->m_world->CreateJoint(&mjd);
		}

		this->m_go = false;
		this->m_time = 0.0;
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_S:
			this->m_go = !this->m_go;
			break;
		}
	}

	void Step(Settings* settings)
	{
		if (this->m_go && settings->hz > 0.0)
		{
			this->m_time += 1.0 / settings->hz;
		}

		b2Vec2 linearOffset;
		linearOffset.x = 6.0 * sinf(2.0 * this->m_time);
		linearOffset.y = 8.0 + 4.0 * sinf(1.0 * this->m_time);
		
		float32 angularOffset = 4.0 * this->m_time;

		this->m_joint->SetLinearOffset(linearOffset);
		this->m_joint->SetAngularOffset(angularOffset);

		g_debugDraw.DrawPoint(linearOffset, 4.0, b2Color(0.9, 0.9, 0.9));

		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Keys: (s) pause");
		this->m_textLine += 15;
	}

	static Test* Create()
	{
		return new MotorJoint;
	}

	b2MotorJoint* m_joint;
	float32 m_time;
	bool m_go;
};

#endif
