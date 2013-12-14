function TestRope()
{
	this.parent.call(this);
}

TestRope.prototype =
{
	Initialize: function()
	{
		var N = 40;
		var vertices = [];
		var masses = [];

		for (var i = 0; i < N; ++i)
		{
			vertices[i] = new b2Vec2(0.0, 20.0 - 0.25 * i);
			masses[i] = 1.0;
		}
		masses[0] = 0.0;
		masses[1] = 0.0;

		var def = new b2RopeDef();
		def.vertices = vertices;
		def.count = N;
		def.gravity.Set(0.0, -10.0);
		def.masses = masses;
		def.damping = 0.1;
		def.k2 = 1.0;
		def.k3 = 0.5;

		this.m_rope = new b2Rope();
		this.m_rope.Initialize(def);

		this.m_angle = 0.0;
		this.m_rope.SetAngle(this.m_angle);
	},

	Keyboard: function(key)
	{
		this.parent.prototype.Keyboard.call(this, key);

		switch (key)
		{
		case 'Q'.charCodeAt():
			this.m_angle = Math.max(-Math.PI, this.m_angle - 0.05 * Math.PI);
			this.m_rope.SetAngle(this.m_angle);
			break;

		case 'E'.charCodeAt():
			this.m_angle = Math.min(Math.PI, this.m_angle + 0.05 * Math.PI);
			this.m_rope.SetAngle(this.m_angle);
			break;
		}
	},

	Step: function()
	{
		var dt = this.m_hz > 0.0 ? this.m_hz : 0.0;

		if (this.m_pause == 1 && this.m_singleStep == 0)
		{
			dt = 0.0;
		}

		this.m_rope.Step(dt, 1);

		this.parent.prototype.Step.call(this);

		this.m_rope.Draw(this.m_debugDraw);

		this.m_drawStringFunc("Press (q,e) to adjust target angle");
		this.m_drawStringFunc("Target angle = " + (this.m_angle * 180.0 / Math.PI) + " degrees");
	}
};

TestRope._extend(Test);