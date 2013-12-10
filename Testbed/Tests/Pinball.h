/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef PINBALL_H
#define PINBALL_H

/// This tests bullet collision and provides an example of a gameplay scenario.
/// This also uses a loop shape.
class Pinball : public Test
{
public:
	Pinball()
	{
		// Ground body
		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2Vec2 vs[5];
			vs[0].Set(0.0, -2.0);
			vs[1].Set(8.0, 6.0);
			vs[2].Set(8.0, 20.0);
			vs[3].Set(-8.0, 20.0);
			vs[4].Set(-8.0, 6.0);

			b2ChainShape loop;
			loop.CreateLoop(vs, 5);
			b2FixtureDef fd;
			fd.shape = &loop;
			fd.density = 0.0;
			ground->CreateFixture(&fd);
		}

		// Flippers
		{
			b2Vec2 p1(-2.0, 0.0), p2(2.0, 0.0);

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;

			bd.position = p1;
			b2Body* leftFlipper = this->m_world->CreateBody(&bd);

			bd.position = p2;
			b2Body* rightFlipper = this->m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(1.75f, 0.1);

			b2FixtureDef fd;
			fd.shape = &box;
			fd.density = 1.0;

			leftFlipper->CreateFixture(&fd);
			rightFlipper->CreateFixture(&fd);

			b2RevoluteJointDef jd;
			jd.bodyA = ground;
			jd.localAnchorB.SetZero();
			jd.enableMotor = true;
			jd.maxMotorTorque = 1000.0;
			jd.enableLimit = true;

			jd.motorSpeed = 0.0;
			jd.localAnchorA = p1;
			jd.bodyB = leftFlipper;
			jd.lowerAngle = -30.0 * b2_pi / 180.0;
			jd.upperAngle = 5.0 * b2_pi / 180.0;
			this->m_leftJoint = (b2RevoluteJoint*)this->m_world->CreateJoint(&jd);

			jd.motorSpeed = 0.0;
			jd.localAnchorA = p2;
			jd.bodyB = rightFlipper;
			jd.lowerAngle = -5.0 * b2_pi / 180.0;
			jd.upperAngle = 30.0 * b2_pi / 180.0;
			this->m_rightJoint = (b2RevoluteJoint*)this->m_world->CreateJoint(&jd);
		}

		// Circle character
		{
			b2BodyDef bd;
			bd.position.Set(1.0, 15.0);
			bd.type = b2Body::b2_dynamicBody;
			bd.bullet = true;

			this->m_ball = this->m_world->CreateBody(&bd);

			b2CircleShape shape;
			shape.m_radius = 0.2;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0;
			this->m_ball->CreateFixture(&fd);
		}

		this->m_button = false;
	}

	void Step(Settings* settings)
	{
		if (this->m_button)
		{
			this->m_leftJoint->SetMotorSpeed(20.0);
			this->m_rightJoint->SetMotorSpeed(-20.0);
		}
		else
		{
			this->m_leftJoint->SetMotorSpeed(-10.0);
			this->m_rightJoint->SetMotorSpeed(10.0);
		}

		Test::Step(settings);

		g_debugDraw.DrawString(5, this->m_textLine, "Press 'a' to control the flippers");
		this->m_textLine += DRAW_STRING_NEW_LINE;

	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			this->m_button = true;
			break;
		}
	}

	void KeyboardUp(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			this->m_button = false;
			break;
		}
	}

	static Test* Create()
	{
		return new Pinball;
	}

	b2RevoluteJoint* m_leftJoint;
	b2RevoluteJoint* m_rightJoint;
	b2Body* m_ball;
	bool m_button;
};

#endif
