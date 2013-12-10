/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef CAR_H
#define CAR_H

// This is a fun demo that shows off the wheel joint
class Car : public Test
{
public:
	Car()
	{		
		this->m_hz = 4.0;
		this->m_zeta = 0.7;
		this->m_speed = 50.0;

		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0;
			fd.friction = 0.6;

			shape.Set(b2Vec2(-20.0, 0.0), b2Vec2(20.0, 0.0));
			ground->CreateFixture(&fd);

			float32 hs[10] = {0.25f, 1.0, 4.0, 0.0, 0.0, -1.0, -2.0, -2.0, -1.25f, 0.0};

			float32 x = 20.0, y1 = 0.0, dx = 5.0;

			for (int32 i = 0; i < 10; ++i)
			{
				float32 y2 = hs[i];
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

			for (int32 i = 0; i < 10; ++i)
			{
				float32 y2 = hs[i];
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

			shape.Set(b2Vec2(x, 0.0), b2Vec2(x + 40.0, 0.0));
			ground->CreateFixture(&fd);

			x += 80.0;
			shape.Set(b2Vec2(x, 0.0), b2Vec2(x + 40.0, 0.0));
			ground->CreateFixture(&fd);

			x += 40.0;
			shape.Set(b2Vec2(x, 0.0), b2Vec2(x + 10.0, 5.0));
			ground->CreateFixture(&fd);

			x += 20.0;
			shape.Set(b2Vec2(x, 0.0), b2Vec2(x + 40.0, 0.0));
			ground->CreateFixture(&fd);

			x += 40.0;
			shape.Set(b2Vec2(x, 0.0), b2Vec2(x, 20.0));
			ground->CreateFixture(&fd);
		}

		// Teeter
		{
			b2BodyDef bd;
			bd.position.Set(140.0, 1.0);
			bd.type = b2Body::b2_dynamicBody;
			b2Body* body = this->m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(10.0, 0.25f);
			body->CreateFixture(&box, 1.0);

			b2RevoluteJointDef jd;
			jd.Initialize(ground, body, body->GetPosition());
			jd.lowerAngle = -8.0 * b2_pi / 180.0;
			jd.upperAngle = 8.0 * b2_pi / 180.0;
			jd.enableLimit = true;
			this->m_world->CreateJoint(&jd);

			body->ApplyAngularImpulse(100.0, true);
		}

		// Bridge
		{
			int32 N = 20;
			b2PolygonShape shape;
			shape.SetAsBox(1.0, 0.125f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0;
			fd.friction = 0.6;

			b2RevoluteJointDef jd;

			b2Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(161.0 + 2.0 * i, -0.125f);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(160.0 + 2.0 * i, -0.125f);
				jd.Initialize(prevBody, body, anchor);
				this->m_world->CreateJoint(&jd);

				prevBody = body;
			}

			b2Vec2 anchor(160.0 + 2.0 * N, -0.125f);
			jd.Initialize(prevBody, ground, anchor);
			this->m_world->CreateJoint(&jd);
		}

		// Boxes
		{
			b2PolygonShape box;
			box.SetAsBox(0.5, 0.5);

			b2Body* body = null;
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;

			bd.position.Set(230.0, 0.5);
			body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5);

			bd.position.Set(230.0, 1.5);
			body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5);

			bd.position.Set(230.0, 2.5);
			body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5);

			bd.position.Set(230.0, 3.5);
			body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5);

			bd.position.Set(230.0, 4.5);
			body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5);
		}

		// Car
		{
			b2PolygonShape chassis;
			b2Vec2 vertices[8];
			vertices[0].Set(-1.5, -0.5);
			vertices[1].Set(1.5, -0.5);
			vertices[2].Set(1.5, 0.0);
			vertices[3].Set(0.0, 0.9);
			vertices[4].Set(-1.15f, 0.9);
			vertices[5].Set(-1.5, 0.2);
			chassis.Set(vertices, 6);

			b2CircleShape circle;
			circle.m_radius = 0.4;

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(0.0, 1.0);
			this->m_car = this->m_world->CreateBody(&bd);
			this->m_car->CreateFixture(&chassis, 1.0);

			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0;
			fd.friction = 0.9;

			bd.position.Set(-1.0, 0.35f);
			this->m_wheel1 = this->m_world->CreateBody(&bd);
			this->m_wheel1->CreateFixture(&fd);

			bd.position.Set(1.0, 0.4);
			this->m_wheel2 = this->m_world->CreateBody(&bd);
			this->m_wheel2->CreateFixture(&fd);

			b2WheelJointDef jd;
			b2Vec2 axis(0.0, 1.0);

			jd.Initialize(this->m_car, this->m_wheel1, this->m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0;
			jd.maxMotorTorque = 20.0;
			jd.enableMotor = true;
			jd.frequencyHz = this->m_hz;
			jd.dampingRatio = this->m_zeta;
			this->m_spring1 = (b2WheelJoint*)this->m_world->CreateJoint(&jd);

			jd.Initialize(this->m_car, this->m_wheel2, this->m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0;
			jd.maxMotorTorque = 10.0;
			jd.enableMotor = false;
			jd.frequencyHz = this->m_hz;
			jd.dampingRatio = this->m_zeta;
			this->m_spring2 = (b2WheelJoint*)this->m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			this->m_spring1->SetMotorSpeed(this->m_speed);
			break;

		case GLFW_KEY_S:
			this->m_spring1->SetMotorSpeed(0.0);
			break;

		case GLFW_KEY_D:
			this->m_spring1->SetMotorSpeed(-this->m_speed);
			break;

		case GLFW_KEY_Q:
			this->m_hz = b2Max(0.0, this->m_hz - 1.0);
			this->m_spring1->SetSpringFrequencyHz(this->m_hz);
			this->m_spring2->SetSpringFrequencyHz(this->m_hz);
			break;

		case GLFW_KEY_E:
			this->m_hz += 1.0;
			this->m_spring1->SetSpringFrequencyHz(this->m_hz);
			this->m_spring2->SetSpringFrequencyHz(this->m_hz);
			break;
		}
	}

	void Step(Settings* settings)
	{
		g_debugDraw.DrawString(5, this->m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "frequency = %g hz, damping ratio = %g", this->m_hz, this->m_zeta);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		g_camera.m_center.x = this->m_car->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Car;
	}

	b2Body* m_car;
	b2Body* m_wheel1;
	b2Body* m_wheel2;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;
	b2WheelJoint* m_spring1;
	b2WheelJoint* m_spring2;
};

#endif
