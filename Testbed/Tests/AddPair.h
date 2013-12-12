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

#ifndef AddPair_H
#define AddPair_H

class AddPair : public Test
{
public:

	AddPair()
	{
		this->m_world->SetGravity(b2Vec2(0.0,0.0));
		{
			b2CircleShape shape;
			shape.m_p.SetZero();
			shape.m_radius = 0.1;

			float minX = -6.0;
			float maxX = 0.0;
			float minY = 4.0;
			float maxY = 6.0;
			
			for (int32 i = 0; i < 400; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Assign(b2Vec2(RandomFloat(minX,maxX),RandomFloat(minY,maxY)));
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 0.01f);
			}
		}
		
		{
			b2PolygonShape shape;
			shape.SetAsBox(1.5, 1.5);
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(-40.0,5.0);
			bd.bullet = true;
			b2Body* body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 1.0);
			body->SetLinearVelocity(b2Vec2(150.0, 0.0));
		}
	}

	static Test* Create()
	{
		return new AddPair;
	}
};

#endif
