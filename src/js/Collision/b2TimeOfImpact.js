"use strict";

/// Input parameters for b2TimeOfImpact
function b2TOIInput()
{
	this.proxyA = new b2DistanceProxy();
	this.proxyB = new b2DistanceProxy();
	this.sweepA = new b2Sweep();
	this.sweepB = new b2Sweep();
	this.tMax = 0;		// defines sweep interval [0, tMax]
};

// Output parameters for b2TimeOfImpact.
function b2TOIOutput()
{
	this.state = 0;
	this.t = 0;
};

b2TOIOutput.e_unknown = 0;
b2TOIOutput.e_failed = 1;
b2TOIOutput.e_overlapped = 2;
b2TOIOutput.e_touching = 3;
b2TOIOutput.e_separated = 4;

//
function b2SeparationFunction()
{
	this.m_proxyA = null;
	this.m_proxyB = null;
	this.m_sweepA = null;
	this.m_sweepB = null;
	this.m_type = 0;
	this.m_localPoint = new b2Vec2();
	this.m_axis = new b2Vec2();
}

var _local_xfA = new b2Transform();
var _local_xfB = new b2Transform();

b2SeparationFunction.prototype =
{
	// TODO_ERIN might not need to return the separation
	Initialize: function(cache,
		proxyA, sweepA,
		proxyB, sweepB,
		t1)
	{
		this.m_proxyA = proxyA;
		this.m_proxyB = proxyB;
		var count = cache.count;
'#if @DEBUG';
		b2Assert(0 < count && count < 3);
'#endif';

		this.m_sweepA = sweepA;
		this.m_sweepB = sweepB;

		this.m_sweepA.GetTransform(_local_xfA, t1);
		this.m_sweepB.GetTransform(_local_xfB, t1);

		if (count == 1)
		{
			this.m_type = b2SeparationFunction.e_points;
			var localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
			var localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
			var pointA = b2Mul_t_v2(_local_xfA, localPointA);
			var pointB = b2Mul_t_v2(_local_xfB, localPointB);
			this.m_axis = b2Vec2.Subtract(pointB, pointA);
			var s = this.m_axis.Normalize();
			return s;
		}
		else if (cache.indexA[0] == cache.indexA[1])
		{
			// Two points on B and one on A.
			this.m_type = b2SeparationFunction.e_faceB;
			var localPointB1 = proxyB.GetVertex(cache.indexB[0]);
			var localPointB2 = proxyB.GetVertex(cache.indexB[1]);

			this.m_axis = b2Cross_v2_f(b2Vec2.Subtract(localPointB2, localPointB1), 1.0);
			this.m_axis.Normalize();
			var normal = b2Mul_r_v2(_local_xfB.q, this.m_axis);

			this.m_localPoint = b2Vec2.Multiply(0.5, b2Vec2.Add(localPointB1, localPointB2));
			var pointB = b2Mul_t_v2(_local_xfB, this.m_localPoint);

			var localPointA = proxyA.GetVertex(cache.indexA[0]);
			var pointA = b2Mul_t_v2(_local_xfA, localPointA);

			var s = b2Dot_v2_v2(b2Vec2.Subtract(pointA, pointB), normal);
			if (s < 0.0)
			{
				this.m_axis = this.m_axis.Negate();
				s = -s;
			}
			return s;
		}
		else
		{
			// Two points on A and one or two points on B.
			this.m_type = b2SeparationFunction.e_faceA;
			var localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
			var localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);

			this.m_axis = b2Cross_v2_f(b2Vec2.Subtract(localPointA2, localPointA1), 1.0);
			this.m_axis.Normalize();
			var normal = b2Mul_r_v2(_local_xfA.q, this.m_axis);

			this.m_localPoint = b2Vec2.Multiply(0.5, b2Vec2.Add(localPointA1, localPointA2));
			var pointA = b2Mul_t_v2(_local_xfA, this.m_localPoint);

			var localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
			var pointB = b2Mul_t_v2(_local_xfB, localPointB);

			var s = b2Dot_v2_v2(b2Vec2.Subtract(pointB, pointA), normal);
			if (s < 0.0)
			{
				this.m_axis = this.m_axis.Negate();
				s = -s;
			}
			return s;
		}
	},

	//
	FindMinSeparation: function(indices, t)
	{
		this.m_sweepA.GetTransform(_local_xfA, t);
		this.m_sweepB.GetTransform(_local_xfB, t);

		switch (this.m_type)
		{
		case b2SeparationFunction.e_points:
			{
				var axisA = b2MulT_r_v2(_local_xfA.q,  this.m_axis);
				var axisB = b2MulT_r_v2(_local_xfB.q, this.m_axis.Negate());

				indices[0] = this.m_proxyA.GetSupport(axisA);
				indices[1] = this.m_proxyB.GetSupport(axisB);

				var localPointA = this.m_proxyA.GetVertex(indices[0]);
				var localPointB = this.m_proxyB.GetVertex(indices[1]);

				var pointA = b2Mul_t_v2(_local_xfA, localPointA);
				var pointB = b2Mul_t_v2(_local_xfB, localPointB);

				var separation = b2Dot_v2_v2(b2Vec2.Subtract(pointB, pointA), this.m_axis);
				return separation;
			}

		case b2SeparationFunction.e_faceA:
			{
				var normal = b2Mul_r_v2(_local_xfA.q, this.m_axis);
				var pointA = b2Mul_t_v2(_local_xfA, this.m_localPoint);

				var axisB = b2MulT_r_v2(_local_xfB.q, normal.Negate());

				indices[0] = -1;
				indices[1] = this.m_proxyB.GetSupport(axisB);

				var localPointB = this.m_proxyB.GetVertex(indices[1]);
				var pointB = b2Mul_t_v2(_local_xfB, localPointB);

				var separation = b2Dot_v2_v2(b2Vec2.Subtract(pointB, pointA), normal);
				return separation;
			}

		case b2SeparationFunction.e_faceB:
			{
				var normal = b2Mul_r_v2(_local_xfB.q, this.m_axis);
				var pointB = b2Mul_t_v2(_local_xfB, this.m_localPoint);

				var axisA = b2MulT_r_v2(_local_xfA.q, normal.Negate());

				indices[1] = -1;
				indices[0] = this.m_proxyA.GetSupport(axisA);

				var localPointA = this.m_proxyA.GetVertex(indices[0]);
				var pointA = b2Mul_t_v2(_local_xfA, localPointA);

				var separation = b2Dot_v2_v2(b2Vec2.Subtract(pointA, pointB), normal);
				return separation;
			}

'#if @DEBUG';
		default:
			b2Assert(false);
			indices[0] = -1;
			indices[1] = -1;
			return 0.0;
'#endif';
		}
	},

	//
	Evaluate: function(indexA, indexB, t)
	{
		this.m_sweepA.GetTransform(_local_xfA, t);
		this.m_sweepB.GetTransform(_local_xfB, t);

		switch (this.m_type)
		{
		case b2SeparationFunction.e_points:
			{
				var localPointA = this.m_proxyA.GetVertex(indexA);
				var localPointB = this.m_proxyB.GetVertex(indexB);

				var pointA = b2Mul_t_v2(_local_xfA, localPointA);
				var pointB = b2Mul_t_v2(_local_xfB, localPointB);
				var separation = b2Dot_v2_v2(b2Vec2.Subtract(pointB, pointA), this.m_axis);

				return separation;
			}

		case b2SeparationFunction.e_faceA:
			{
				var normal = b2Mul_r_v2(_local_xfA.q, this.m_axis);
				var pointA = b2Mul_t_v2(_local_xfA, this.m_localPoint);

				var localPointB = this.m_proxyB.GetVertex(indexB);
				var pointB = b2Mul_t_v2(_local_xfB, localPointB);

				var separation = b2Dot_v2_v2(b2Vec2.Subtract(pointB, pointA), normal);
				return separation;
			}

		case b2SeparationFunction.e_faceB:
			{
				var normal = b2Mul_r_v2(_local_xfB.q, this.m_axis);
				var pointB = b2Mul_t_v2(_local_xfB, this.m_localPoint);

				var localPointA = this.m_proxyA.GetVertex(indexA);
				var pointA = b2Mul_t_v2(_local_xfA, localPointA);

				var separation = b2Dot_v2_v2(b2Vec2.Subtract(pointA, pointB), normal);
				return separation;
			}

'#if @DEBUG';
		default:
			b2Assert(false);
			return 0.0;
'#endif';
		}
	}
};

b2SeparationFunction.e_points = 0;
b2SeparationFunction.e_faceA = 1;
b2SeparationFunction.e_faceB = 2;

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collision. If you change the time interval, you should call this function
/// again.
/// Note: use b2Distance to compute the contact point and normal at the time of impact.
var profile_toi = b2Profiler.create("toi", "solveTOI");
function b2TimeOfImpact(output, input)
{
	profile_toi.start();

	++b2TimeOfImpact.b2_toiCalls;

	output.state = b2TOIOutput.e_unknown;
	output.t = input.tMax;

	var proxyA = input.proxyA;
	var proxyB = input.proxyB;

	b2TimeOfImpact._temp_sweepA.Assign(input.sweepA);
	b2TimeOfImpact._temp_sweepB.Assign(input.sweepB);

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	b2TimeOfImpact._temp_sweepA.Normalize();
	b2TimeOfImpact._temp_sweepB.Normalize();

	var tMax = input.tMax;

	var totalRadius = proxyA.m_radius + proxyB.m_radius;
	var target = b2Max(b2_linearSlop, totalRadius - 3.0 * b2_linearSlop);
	var tolerance = 0.25 * b2_linearSlop;
'#if @DEBUG';
	b2Assert(target > tolerance);
'#endif';

	var t1 = 0.0;
	var k_maxIterations = 20;	// TODO_ERIN b2Settings
	var iter = 0;

	// Prepare input for distance query.
	var cache = new b2SimplexCache();
	cache.count = 0;
	var distanceInput = new b2DistanceInput();
	distanceInput.proxyA.Assign(input.proxyA);
	distanceInput.proxyB.Assign(input.proxyB);
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for(;;)
	{
		b2TimeOfImpact._temp_sweepA.GetTransform(distanceInput.transformA, t1);
		b2TimeOfImpact._temp_sweepB.GetTransform(distanceInput.transformB, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		var distanceOutput = new b2DistanceOutput();
		b2DistanceFunc(distanceOutput, cache, distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= 0.0)
		{
			// Failure!
			output.state = b2TOIOutput.e_overlapped;
			output.t = 0.0;
			break;
		}

		if (distanceOutput.distance < target + tolerance)
		{
			// Victory!
			output.state = b2TOIOutput.e_touching;
			output.t = t1;
			break;
		}

		// Initialize the separating axis.
		var fcn = new b2SeparationFunction();
		fcn.Initialize(cache, proxyA, b2TimeOfImpact._temp_sweepA, proxyB, b2TimeOfImpact._temp_sweepB, t1);

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		var done = false;
		var t2 = tMax;
		var pushBackIter = 0;
		for (;;)
		{
			// Find the deepest point at t2. Store the witness point indices.
			var indices = [];
			var s2 = fcn.FindMinSeparation(indices, t2);

			// Is the final configuration separated?
			if (s2 > target + tolerance)
			{
				// Victory!
				output.state = b2TOIOutput.e_separated;
				output.t = tMax;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			var s1 = fcn.Evaluate(indices[0], indices[1], t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < target - tolerance)
			{
				output.state = b2TOIOutput.e_failed;
				output.t = t1;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= target + tolerance)
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output.state = b2TOIOutput.e_touching;
				output.t = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			var rootIterCount = 0;
			var a1 = t1, a2 = t2;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				var t;
				if (rootIterCount & 1)
				{
					// Secant rule to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
				}
				else
				{
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2);
				}

				++rootIterCount;
				++b2TimeOfImpact.b2_toiRootIters;

				var s = fcn.Evaluate(indices[0], indices[1], t);

				if (b2Abs(s - target) < tolerance)
				{
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target)
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}

				if (rootIterCount == 50)
				{
					break;
				}
			}

			b2TimeOfImpact.b2_toiMaxRootIters = b2Max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount);

			++pushBackIter;

			if (pushBackIter == b2_maxPolygonVertices)
			{
				break;
			}
		}

		++iter;
		++b2TimeOfImpact.b2_toiIters;

		if (done)
		{
			break;
		}

		if (iter == k_maxIterations)
		{
			// Root finder got stuck. Semi-victory.
			output.state = b2TOIOutput.e_failed;
			output.t = t1;
			break;
		}
	}

	b2TimeOfImpact.b2_toiMaxIters = b2Max(b2TimeOfImpact.b2_toiMaxIters, iter);

	profile_toi.stop();
	b2TimeOfImpact.b2_toiMaxTime = b2Max(b2TimeOfImpact.b2_toiMaxTime, profile_toi.elapsedTime);
	b2TimeOfImpact.b2_toiTime += profile_toi.elapsedTime;
}

b2TimeOfImpact._temp_sweepA = new b2Sweep();
b2TimeOfImpact._temp_sweepB = new b2Sweep();

b2TimeOfImpact.b2_toiTime = 0;
b2TimeOfImpact.b2_toiMaxTime = 0;
b2TimeOfImpact.b2_toiCalls = 0;
b2TimeOfImpact.b2_toiIters = 0;
b2TimeOfImpact.b2_toiMaxIters = 0;
b2TimeOfImpact.b2_toiRootIters = 0;
b2TimeOfImpact.b2_toiMaxRootIters = 0;