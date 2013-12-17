var b2Profiler = (function()
{
	if (typeof(performance) === 'undefined')
		window['performance'] = { now: function() { return +new Date(); } };

	function profileStruct(name, parent)
	{
		this.name = name;
		this.parent = parent;
		this.children = {};

		this.startTime = 0;
		this.elapsedTime = 0;
		this.totalTime = 0;
		this.running = false;
		this.childrenCount = 0;
	}

	profileStruct.prototype =
	{
		// starts timer.
		start: function()
		{
			this.startTime = performance.now();
			this.running = true;
		},

		// stops timer, accumulates elapsed time.
		// may also reset start time.
		stop: function(reset)
		{
			if (!this.running)
				return;

			this.running = false;
			this.elapsedTime += performance.now() - this.startTime;

			if (reset)
				this.start();

			for (var x in this.children)
				this.children[x].stop();
		},

		// resets timer and pushes elapsed time into total
		reset: function(dontRun)
		{
			if (!dontRun)
			{
				this.running = true;
				this.totalTime += this.elapsedTime;
				this.start();
			}

			this.elapsedTime = 0;

			for (var x in this.children)
				this.children[x].reset(true);
		}
	};

	var profiles = [];
	var root = new profileStruct("root");

	function create(name, parent)
	{
		if (!profiles)
			throw new Error("late profile creation not allowed");

		var s = new profileStruct(name, parent || 'root');
		profiles.push(s);
		return s;
	}

	function destroy(profile)
	{
		profile.childrenCount--;
		delete profile.children[profile.name];
	}

	function recursiveParentCheck(node, profile)
	{
		if (node.name === profile.parent)
			return node;

		for (var x in node.children)
		{
			var n;

			if (n = recursiveParentCheck(node.children[x], profile))
				return n;
		}

		return null;
	}

	// profiles are created statically, so this has to be done
	// once in a place after all profiles are known to have been
	// made.
	function init()
	{
		// do it!
		while (profiles.length)
		{
			var p = profiles.pop();

			if (!(p.parentNode = recursiveParentCheck(root, p)))
				profiles.unshift(p);
			else
			{
				p.parentNode.children[p.name] = p;
				p.parentNode.childrenCount++;
			}
		}

		// prevent additions to it
		profiles = null;
	}

	function resetAll()
	{
		root.reset(true);
	}

	return {
		create: create,
		destroy: destroy,
		init: init,
		reset: resetAll,

		profileRoot: root
	};
}());