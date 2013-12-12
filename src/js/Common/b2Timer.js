"use strict";

if (typeof(performance) === 'undefined')
{
	window.performance = { now: function() { return +new Date(); } };
}

/// Timer for profiling.
function b2Timer()
{
	this.Reset();
}

b2Timer.prototype =
{
	/// Reset the timer.
	Reset: function ()
	{
		this.m_start = performance.now();
	},

	/// Get the time since construction or the last reset.
	GetMilliseconds: function()
	{
		return performance.now() - this.m_start;
	}
};