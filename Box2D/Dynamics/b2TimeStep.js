"use strict";

/// Profiling data. Times are in milliseconds.
function b2Profile()
{
	this.step = 0;
	this.collide = 0;
	this.solve = 0;
	this.solveInit = 0;
	this.solveVelocity = 0;
	this.solvePosition = 0;
	this.broadphase = 0;
	this.solveTOI = 0;
}

/// This is an internal structure.
function b2TimeStep()
{
	this.dt = 0;			// time step
	this.inv_dt = 0;		// inverse time step (0 if dt == 0).
	this.dtRatio = 0;	// dt * inv_dt0
	this.velocityIterations = 0;
	this.positionIterations = 0;
	this.warmStarting = false;
}

/// This is an internal structure.
function b2Position()
{
	this.c = new b2Vec2();
	this.a = 0;
}

/// This is an internal structure.
function b2Velocity()
{
	this.v = new b2Vec2();
	this.w = 0;
}

/// Solver Data
function b2SolverData()
{
	this.step = new b2TimeStep();
	this.positions = null;
	this.velocities = null;
}