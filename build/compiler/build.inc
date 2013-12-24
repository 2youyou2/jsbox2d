(function()
{
'use strict';
'#include ../../src/js/System/Class.js';
'#include ../../src/js/System/Profiling.js';

'#include ../../src/js/Common/b2Settings.js';
'#include ../../src/js/Common/b2Math.js';
'#include ../../src/js/Common/b2Draw.js';
'#include ../../src/js/Common/b2Timer.js';

'#include ../../src/js/Collision/Shapes/b2Shape.js';
'#include ../../src/js/Collision/Shapes/b2CircleShape.js';
'#include ../../src/js/Collision/Shapes/b2EdgeShape.js';
'#include ../../src/js/Collision/Shapes/b2ChainShape.js';
'#include ../../src/js/Collision/Shapes/b2PolygonShape.js';

'#include ../../src/js/Collision/b2BroadPhase.js';
'#include ../../src/js/Collision/b2Distance.js';
'#include ../../src/js/Collision/b2Collision.js';
'#include ../../src/js/Collision/b2DynamicTree.js';
'#include ../../src/js/Collision/b2TimeOfImpact.js';

'#include ../../src/js/Dynamics/b2Body.js';
'#include ../../src/js/Dynamics/b2Fixture.js';
'#include ../../src/js/Dynamics/b2WorldCallbacks.js';
'#include ../../src/js/Dynamics/b2TimeStep.js';
'#include ../../src/js/Dynamics/b2World.js';

'#include ../../src/js/Dynamics/Contacts/b2Contact.js';
'#include ../../src/js/Dynamics/b2ContactManager.js';
'#include ../../src/js/Dynamics/Contacts/b2ContactSolver.js';
'#include ../../src/js/Dynamics/b2Island.js';

'#include ../../src/js/Dynamics/Joints/b2Joint.js';
'#include ../../src/js/Dynamics/Joints/b2RevoluteJoint.js';
'#include ../../src/js/Dynamics/Joints/b2MouseJoint.js';
'#include ../../src/js/Dynamics/Joints/b2DistanceJoint.js';
'#include ../../src/js/Dynamics/Joints/b2PrismaticJoint.js';
'#include ../../src/js/Dynamics/Joints/b2FrictionJoint.js';
'#include ../../src/js/Dynamics/Joints/b2WeldJoint.js';
'#include ../../src/js/Dynamics/Joints/b2WheelJoint.js';
'#include ../../src/js/Dynamics/Joints/b2GearJoint.js';
'#include ../../src/js/Dynamics/Joints/b2MotorJoint.js';
'#include ../../src/js/Dynamics/Joints/b2PulleyJoint.js';
'#include ../../src/js/Dynamics/Joints/b2RopeJoint.js';

'#include ../../src/js/Rope/b2Rope.js';

'#include ../../src/js/System/Serializer.js';
'#include ../../src/js/System/RUBE.js';

	'#export
	b2_version b2Vec2 b2Vec3 b2Mat22 b2Mat33 b2Rot b2Transform b2Sweep b2Dot_v2_v2 b2Cross_v2_v2 b2Cross_v2_f b2Cross_f_v2 b2Mul_m22_v2 b2MulT_m22_v2 b2Distance b2DistanceSquared b2Dot_v3_v3 b2Cross_v3_v3 b2Mul_m22_m22 b2MulT_m22_m22 b2Mul_m33_v3 b2Mul22_m33_v2 b2Mul_r_r b2MulT_r_r b2Mul_r_v2 b2MulT_r_v2 b2Mul_t_v2 b2Min_v2 b2Max_v2 b2Clamp b2MulT_t_v2 b2Mul_t_t b2MulT_t_t b2Clamp_v2 b2NextPowerOfTwo b2Abs_v2 b2Abs_m22 b2IsPowerOfTwo b2RandomFloat

	b2Timer

	b2Color b2Draw

	b2ContactID b2ManifoldPoint b2Manifold b2WorldManifold b2GetPointStates b2ClipVertex b2RayCastInput b2RayCastOutput b2AABB b2CollideCircles b2CollidePolygonAndCircle b2FindMaxSeparation b2FindIncidentEdge b2CollidePolygons b2CollideEdgeAndCircle b2EPAxis b2TempPolygon b2ReferenceFace b2EPCollider b2CollideEdgeAndPolygon b2ClipSegmentToLine b2TestShapeOverlap b2TestOverlap

	b2Shape b2CircleShape b2EdgeShape b2ChainShape b2PolygonShape

	b2Pair b2PairLessThan b2BroadPhase

	b2DistanceProxy b2SimplexCache b2DistanceInput b2DistanceOutput b2SimplexVertex b2Simplex b2DistanceFunc

	b2TreeNode b2DynamicTree

	b2TOIInput b2TOIOutput b2SeparationFunction b2TimeOfImpact

	b2BodyDef b2Body

	b2Filter b2FixtureDef b2Fixture

	b2DestructionListener b2ContactFilter b2ContactImpulse b2ContactListener b2QueryCallback b2RayCastCallback

	b2TimeStep b2Position b2Velocity b2SolverData

	b2World

	b2MixFriction b2MixRestitution b2ContactRegister b2ContactEdge b2Contact b2CircleContact b2PolygonContact b2ChainAndCircleContact b2ChainAndPolygonContact b2EdgeAndCircleContact b2EdgeAndPolygonContact b2PolygonAndCircleContact

	b2_defaultFilter b2_defaultListener b2ContactManager

	b2VelocityConstraintPoint b2ContactPositionConstraint b2ContactVelocityConstraint b2PositionSolverManifold b2ContactSolverDef b2ContactSolver

	b2Island

	b2Jacobian b2JointEdge b2JointDef b2Joint

	b2RevoluteJointDef b2RevoluteJoint

	b2MouseJointDef b2MouseJoint

	b2DistanceJointDef b2DistanceJoint

	b2PrismaticJointDef b2PrismaticJoint

	b2FrictionJointDef b2FrictionJoint

	b2WeldJointDef b2WeldJoint

	b2WheelJointDef b2WheelJoint

	b2GearJointDef b2GearJoint

	b2MotorJointDef b2MotorJoint

	b2PulleyJointDef b2PulleyJoint

	b2RopeJointDef b2RopeJoint

	b2RopeDef b2Rope

	b2_maxManifoldPoints b2_maxPolygonVertices b2_aabbExtension b2_aabbMultiplier b2_linearSlop b2_angularSlop b2_polygonRadius b2_maxSubSteps b2_maxTOIContacts b2_velocityThreshold b2_maxLinearCorrection b2_maxAngularCorrection b2_maxTranslation b2_maxTranslationSquared b2_maxRotation b2_maxRotationSquared b2_baumgarte b2_toiBaugarte b2_timeToSleep b2_linearSleepTolerance b2_angularSleepTolerance b2_epsilon

	b2JsonSerializer

	b2RUBELoader

	b2Profiler
';
})();