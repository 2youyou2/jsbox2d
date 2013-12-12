/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2Joint.h>
#include <Box2D/Dynamics/Joints/b2DistanceJoint.h>
#include <Box2D/Dynamics/Joints/b2WheelJoint.h>
#include <Box2D/Dynamics/Joints/b2MouseJoint.h>
#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/Joints/b2PrismaticJoint.h>
#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/Joints/b2GearJoint.h>
#include <Box2D/Dynamics/Joints/b2WeldJoint.h>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>
#include <Box2D/Dynamics/Joints/b2RopeJoint.h>
#include <Box2D/Dynamics/Joints/b2MotorJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/b2BlockAllocator.h>

#include <new>

b2JointDef::b2JointDef()
{
	this->type = b2Joint::e_unknownJoint;
	this->userData = null;
	this->bodyA = null;
	this->bodyB = null;
	this->collideConnected = false;
}

b2Joint* b2Joint::Create(const b2JointDef* def, b2BlockAllocator* allocator)
{
	b2Joint* joint = null;

	switch (def->type)
	{
	case b2Joint::e_distanceJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
			joint = new (mem) b2DistanceJoint(static_cast<const b2DistanceJointDef*>(def));
		}
		break;

	case b2Joint::e_mouseJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2MouseJoint));
			joint = new (mem) b2MouseJoint(static_cast<const b2MouseJointDef*>(def));
		}
		break;

	case b2Joint::e_prismaticJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
			joint = new (mem) b2PrismaticJoint(static_cast<const b2PrismaticJointDef*>(def));
		}
		break;

	case b2Joint::e_revoluteJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
			joint = new (mem) b2RevoluteJoint(static_cast<const b2RevoluteJointDef*>(def));
		}
		break;

	case b2Joint::e_pulleyJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
			joint = new (mem) b2PulleyJoint(static_cast<const b2PulleyJointDef*>(def));
		}
		break;

	case b2Joint::e_gearJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2GearJoint));
			joint = new (mem) b2GearJoint(static_cast<const b2GearJointDef*>(def));
		}
		break;

	case b2Joint::e_wheelJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2WheelJoint));
			joint = new (mem) b2WheelJoint(static_cast<const b2WheelJointDef*>(def));
		}
		break;

	case b2Joint::e_weldJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2WeldJoint));
			joint = new (mem) b2WeldJoint(static_cast<const b2WeldJointDef*>(def));
		}
		break;
        
	case b2Joint::e_frictionJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2FrictionJoint));
			joint = new (mem) b2FrictionJoint(static_cast<const b2FrictionJointDef*>(def));
		}
		break;

	case b2Joint::e_ropeJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2RopeJoint));
			joint = new (mem) b2RopeJoint(static_cast<const b2RopeJointDef*>(def));
		}
		break;

	case b2Joint::e_motorJoint:
		{
			void* mem = allocator->Allocate(sizeof(b2MotorJoint));
			joint = new (mem) b2MotorJoint(static_cast<const b2MotorJointDef*>(def));
		}
		break;

	default:
		b2Assert(false);
		break;
	}

	return joint;
}

void b2Joint::Destroy(b2Joint* joint, b2BlockAllocator* allocator)
{
	joint->~b2Joint();
	switch (joint->m_type)
	{
	case b2Joint::e_distanceJoint:
		allocator->Free(joint, sizeof(b2DistanceJoint));
		break;

	case b2Joint::e_mouseJoint:
		allocator->Free(joint, sizeof(b2MouseJoint));
		break;

	case b2Joint::e_prismaticJoint:
		allocator->Free(joint, sizeof(b2PrismaticJoint));
		break;

	case b2Joint::e_revoluteJoint:
		allocator->Free(joint, sizeof(b2RevoluteJoint));
		break;

	case b2Joint::e_pulleyJoint:
		allocator->Free(joint, sizeof(b2PulleyJoint));
		break;

	case b2Joint::e_gearJoint:
		allocator->Free(joint, sizeof(b2GearJoint));
		break;

	case b2Joint::e_wheelJoint:
		allocator->Free(joint, sizeof(b2WheelJoint));
		break;
    
	case b2Joint::e_weldJoint:
		allocator->Free(joint, sizeof(b2WeldJoint));
		break;

	case b2Joint::e_frictionJoint:
		allocator->Free(joint, sizeof(b2FrictionJoint));
		break;

	case b2Joint::e_ropeJoint:
		allocator->Free(joint, sizeof(b2RopeJoint));
		break;

	case b2Joint::e_motorJoint:
		allocator->Free(joint, sizeof(b2MotorJoint));
		break;

	default:
		b2Assert(false);
		break;
	}
}

b2Joint::b2Joint(const b2JointDef* def)
{
	b2Assert(def->bodyA != def->bodyB);

	this->m_type = def->type;
	this->m_prev = null;
	this->m_next = null;
	this->m_bodyA = def->bodyA;
	this->m_bodyB = def->bodyB;
	this->m_index = 0;
	this->m_collideConnected = def->collideConnected;
	this->m_islandFlag = false;
	this->m_userData = def->userData;

	this->m_edgeA.joint = null;
	this->m_edgeA.other = null;
	this->m_edgeA.prev = null;
	this->m_edgeA.next = null;

	this->m_edgeB.joint = null;
	this->m_edgeB.other = null;
	this->m_edgeB.prev = null;
	this->m_edgeB.next = null;
}

bool b2Joint::IsActive() const
{
	return this->m_bodyA->IsActive() && this->m_bodyB->IsActive();
}
