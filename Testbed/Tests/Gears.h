/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef GEARS_H
#define GEARS_H

class Gears : public Test
{
public:
	Gears()
	{
		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(50.0, 0.0), b2Vec2(-50.0, 0.0));
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2CircleShape circle1;
			circle1.m_radius = 1.0;

			b2PolygonShape box;
			box.SetAsBox(0.5, 5.0);

			b2CircleShape circle2;
			circle2.m_radius = 2.0;
			
			b2BodyDef bd1;
			bd1.type = b2Body::b2_staticBody;
			bd1.position.Set(10.0, 9.0);
			b2Body* body1 = this->m_world->CreateBody(&bd1);
			body1->CreateFixture(&circle1, 5.0);

			b2BodyDef bd2;
			bd2.type = b2Body::b2_dynamicBody;
			bd2.position.Set(10.0, 8.0);
			b2Body* body2 = this->m_world->CreateBody(&bd2);
			body2->CreateFixture(&box, 5.0);

			b2BodyDef bd3;
			bd3.type = b2Body::b2_dynamicBody;
			bd3.position.Set(10.0, 6.0);
			b2Body* body3 = this->m_world->CreateBody(&bd3);
			body3->CreateFixture(&circle2, 5.0);

			b2RevoluteJointDef jd1;
			jd1.Initialize(body2, body1, bd1.position);
			b2Joint* joint1 = this->m_world->CreateJoint(&jd1);

			b2RevoluteJointDef jd2;
			jd2.Initialize(body2, body3, bd3.position);
			b2Joint* joint2 = this->m_world->CreateJoint(&jd2);

			b2GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body3;
			jd4.joint1 = joint1;
			jd4.joint2 = joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			this->m_world->CreateJoint(&jd4);
		}

		{
			b2CircleShape circle1;
			circle1.m_radius = 1.0;

			b2CircleShape circle2;
			circle2.m_radius = 2.0;
			
			b2PolygonShape box;
			box.SetAsBox(0.5, 5.0);

			b2BodyDef bd1;
			bd1.type = b2Body::b2_dynamicBody;
			bd1.position.Set(-3.0, 12.0);
			b2Body* body1 = this->m_world->CreateBody(&bd1);
			body1->CreateFixture(&circle1, 5.0);

			b2RevoluteJointDef jd1;
			jd1.bodyA = ground;
			jd1.bodyB = body1;
			jd1.localAnchorA.Assign(ground->GetLocalPoint(bd1.position));
			jd1.localAnchorB.Assign(body1->GetLocalPoint(bd1.position));
			jd1.referenceAngle = body1->GetAngle() - ground->GetAngle();
			this->m_joint1 = (b2RevoluteJoint*)this->m_world->CreateJoint(&jd1);

			b2BodyDef bd2;
			bd2.type = b2Body::b2_dynamicBody;
			bd2.position.Set(0.0, 12.0);
			b2Body* body2 = this->m_world->CreateBody(&bd2);
			body2->CreateFixture(&circle2, 5.0);

			b2RevoluteJointDef jd2;
			jd2.Initialize(ground, body2, bd2.position);
			this->m_joint2 = (b2RevoluteJoint*)this->m_world->CreateJoint(&jd2);

			b2BodyDef bd3;
			bd3.type = b2Body::b2_dynamicBody;
			bd3.position.Set(2.5, 12.0);
			b2Body* body3 = this->m_world->CreateBody(&bd3);
			body3->CreateFixture(&box, 5.0);

			b2PrismaticJointDef jd3;
			jd3.Initialize(ground, body3, bd3.position, b2Vec2(0.0, 1.0));
			jd3.lowerTranslation = -5.0;
			jd3.upperTranslation = 5.0;
			jd3.enableLimit = true;

			this->m_joint3 = (b2PrismaticJoint*)this->m_world->CreateJoint(&jd3);

			b2GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = this->m_joint1;
			jd4.joint2 = this->m_joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			this->m_joint4 = (b2GearJoint*)this->m_world->CreateJoint(&jd4);

			b2GearJointDef jd5;
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = this->m_joint2;
			jd5.joint2 = this->m_joint3;
			jd5.ratio = -1.0 / circle2.m_radius;
			this->m_joint5 = (b2GearJoint*)this->m_world->CreateJoint(&jd5);
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		float32 ratio, value;
		
		ratio = this->m_joint4->GetRatio();
		value = this->m_joint1->GetJointAngle() + ratio * this->m_joint2->GetJointAngle();
		g_debugDraw.DrawString(5, this->m_textLine, "theta1 + %4.2 * theta2 = %4.2", (float) ratio, (float) value);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		ratio = this->m_joint5->GetRatio();
		value = this->m_joint2->GetJointAngle() + ratio * this->m_joint3->GetJointTranslation();
		g_debugDraw.DrawString(5, this->m_textLine, "theta2 + %4.2 * delta = %4.2", (float) ratio, (float) value);
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Gears;
	}

	b2RevoluteJoint* m_joint1;
	b2RevoluteJoint* m_joint2;
	b2PrismaticJoint* m_joint3;
	b2GearJoint* m_joint4;
	b2GearJoint* m_joint5;
};

#endif
