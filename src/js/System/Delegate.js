"use strict";

Function.prototype.delegate = function(me)
{
	return (function(_t) { return function() { return _t.apply(me, arguments); }; })(this);
};