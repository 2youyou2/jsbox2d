"use strict";

Function.prototype._extend = function(parent)
{
	this.prototype.parent = parent;

	for (var x in parent.prototype)
	{
		if (!this.prototype[x])
			this.prototype[x] = parent.prototype[x];
	}
};

Function.prototype._implement = function(parent)
{
	return this._extend(parent);
};