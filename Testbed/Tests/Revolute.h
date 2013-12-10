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

#ifndef REVOLUTE_H
#define REVOLUTE_H

class Revolute : public Test
{
public:
	Revolute()
	{
		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));

			b2FixtureDef fd;
			fd.shape = &shape;
			//fd.filter.categoryBits = 2;

			ground->CreateFixture(&fd);
		}

		{
			b2CircleShape shape;
			shape.m_radius = 0.5;

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;

			b2RevoluteJointDef rjd;

			bd.position.Set(-10.0, 20.0);
			b2Body* body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 5.0);

			float32 w = 100.0;
			body->SetAngularVelocity(w);
			body->SetLinearVelocity(b2Vec2(-8.0 * w, 0.0));

			rjd.Initialize(ground, body, b2Vec2(-10.0, 12.0));
			rjd.motorSpeed = 1.0 * b2_pi;
			rjd.maxMotorTorque = 10000.0;
			rjd.enableMotor = false;
			rjd.lowerAngle = -0.25f * b2_pi;
			rjd.upperAngle = 0.5 * b2_pi;
			rjd.enableLimit = true;
			rjd.collideConnected = true;

			this->m_joint = (b2RevoluteJoint*)this->m_world->CreateJoint(&rjd);
		}

		{
			b2CircleShape circle_shape;
			circle_shape.m_radius = 3.0;

			b2BodyDef circle_bd;
			circle_bd.type = b2Body::b2_dynamicBody;
			circle_bd.position.Set(5.0, 30.0);

			b2FixtureDef fd;
			fd.density = 5.0;
			fd.filter.maskBits = 1;
			fd.shape = &circle_shape;

			this->m_ball = this->m_world->CreateBody(&circle_bd);
			this->m_ball->CreateFixture(&fd);

			b2PolygonShape polygon_shape;
			polygon_shape.SetAsBox(10.0, 0.2, b2Vec2 (-10.0, 0.0), 0.0);

			b2BodyDef polygon_bd;
			polygon_bd.position.Set(20.0, 10.0);
			polygon_bd.type = b2Body::b2_dynamicBody;
			polygon_bd.bullet = true;
			b2Body* polygon_body = this->m_world->CreateBody(&polygon_bd);
			polygon_body->CreateFixture(&polygon_shape, 2.0);

			b2RevoluteJointDef rjd;
			rjd.Initialize(ground, polygon_body, b2Vec2(20.0, 10.0));
			rjd.lowerAngle = -0.25f * b2_pi;
			rjd.upperAngle = 0.0 * b2_pi;
			rjd.enableLimit = true;
			this->m_world->CreateJoint(&rjd);
		}

		// Tests mass computation of a small object far from the origin
		{
			b2BodyDef bodyDef;
			bodyDef.type = b2Body::b2_dynamicBody;
			b2Body* body = this->m_world->CreateBody(&bodyDef);
		
			b2PolygonShape polyShape;		
			b2Vec2 verts[3];
			verts[0].Set( 17.63f, 36.31f );
			verts[1].Set( 17.52f, 36.69f );
			verts[2].Set( 17.19f, 36.36f );
			polyShape.Set(verts, 3);
		
			b2FixtureDef polyFixtureDef;
			polyFixtureDef.shape = &polyShape;
			polyFixtureDef.density = 1;

			body->CreateFixture(&polyFixtureDef);	//assertion hits inside here
		}

	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_L:
			this->m_joint->EnableLimit(!this->m_joint->IsLimitEnabled());
			break;

		case GLFW_KEY_M:
			this->m_joint->EnableMotor(!this->m_joint->IsMotorEnabled());
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Keys: (l) limits, (m) motor");
		this->m_textLine += DRAW_STRING_NEW_LINE;

		//if (this->m_stepCount == 360)
		//{
		//	this->m_ball->SetTransform(b2Vec2(0.0, 0.5), 0.0);
		//}

		//float32 torque1 = this->m_joint1->GetMotorTorque();
		//g_debugDraw.DrawString(5, this->m_textLine, "Motor Torque = %4.0, %4.0 : Motor Force = %4.0", (float) torque1, (float) torque2, (float) force3);
		//this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Revolute;
	}

	b2Body* m_ball;
	b2RevoluteJoint* m_joint;
};

#endif
