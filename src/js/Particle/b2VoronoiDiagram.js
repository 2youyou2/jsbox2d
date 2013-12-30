/// A field representing the nearest generator from each point.

function b2VoronoiDiagram(generatorCapacity)
{
	this.m_generatorBuffer = new Array(generatorCapacity);
	this.m_generatorCount = 0;
	this.m_countX = 0;
	this.m_countY = 0;
	this.m_diagram = null;
}

b2VoronoiDiagram.Generator = function()
{
	this.center = new b2Vec2();
	this.tag = 0;
};

b2VoronoiDiagram.b2VoronoiDiagramTask = function(x, y, i, g)
{
	this.m_x = x;
	this.m_y = y;
	this.m_i = i;
	this.m_generator = g;
};

b2VoronoiDiagram.prototype =
{
	AddGenerator: function(center, tag)
	{
		var g = (this.m_generatorBuffer[this.m_generatorCount++] = new b2VoronoiDiagram.Generator());
		g.center.Assign(center);
		g.tag = tag;
	},

	Generate: function(radius)
	{
'#if @DEBUG';
		b2Assert(this.m_diagram == null);
'#endif';
		var inverseRadius = 1 / radius;
		var lower = new b2Vec2(+b2_maxFloat, +b2_maxFloat);
		var upper = new b2Vec2(-b2_maxFloat, -b2_maxFloat);

		for (var k = 0; k < this.m_generatorCount; k++)
		{
			var g = this.m_generatorBuffer[k];
			lower.Assign(b2Min_v2(lower, g.center));
			upper.Assign(b2Max_v2(upper, g.center));
		}

		this.m_countX = 1 + ((inverseRadius * (upper.x - lower.x)) >>> 0);
		this.m_countY = 1 + ((inverseRadius * (upper.y - lower.y)) >>> 0);

		this.m_diagram = new Array(this.m_countX * this.m_countY);

		for (var i = 0; i < this.m_countX * this.m_countY; i++)
			this.m_diagram[i] = null;

		var queue = new b2StackQueue(this.m_countX * this.m_countX);
		for (var k = 0; k < this.m_generatorCount; k++)
		{
			var g = this.m_generatorBuffer[k];
			g.center.Assign(b2Vec2.Multiply(inverseRadius, b2Vec2.Subtract(g.center, lower)));
			var x = b2Max(0, b2Min(Math.floor(g.center.x), this.m_countX - 1));
			var y = b2Max(0, b2Min(Math.floor(g.center.y), this.m_countY - 1));
			queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y, x + y * this.m_countX, g));
		}
		while (!queue.Empty())
		{
			var x = queue.Front().m_x;
			var y = queue.Front().m_y;
			var i = queue.Front().m_i;
			var g = queue.Front().m_generator;
			queue.Pop();
			if (!this.m_diagram[i])
			{
				this.m_diagram[i] = g;
				if (x > 0)
				{
					queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x - 1, y, i - 1, g));
				}
				if (y > 0)
				{
					queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y - 1, i - this.m_countX, g));
				}
				if (x < this.m_countX - 1)
				{
					queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x + 1, y, i + 1, g));
				}
				if (y < this.m_countY - 1)
				{
					queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y + 1, i + this.m_countX, g));
				}
			}
		}
		var maxIteration = this.m_countX + this.m_countY;
		for (var iteration = 0; iteration < maxIteration; iteration++)
		{
			for (var y = 0; y < this.m_countY; y++)
			{
				for (var x = 0; x < this.m_countX - 1; x++)
				{
					var i = x + y * this.m_countX;
					var a = this.m_diagram[i];
					var b = this.m_diagram[i + 1];
					if (a != b)
					{
						queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y, i, b));
						queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x + 1, y, i + 1, a));
					}
				}
			}
			for (var y = 0; y < this.m_countY - 1; y++)
			{
				for (var x = 0; x < this.m_countX; x++)
				{
					var i = x + y * this.m_countX;
					var a = this.m_diagram[i];
					var b = this.m_diagram[i + this.m_countX];
					if (a != b)
					{
						queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y, i, b));
						queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y + 1, i + this.m_countX, a));
					}
				}
			}
			var updated = false;
			while (!queue.Empty())
			{
				var x = queue.Front().m_x;
				var y = queue.Front().m_y;
				var i = queue.Front().m_i;
				var k = queue.Front().m_generator;
				queue.Pop();
				var a = this.m_diagram[i];
				var b = k;
				if (a != b)
				{
					var ax = a.center.x - x;
					var ay = a.center.y - y;
					var bx = b.center.x - x;
					var by = b.center.y - y;
					var a2 = ax * ax + ay * ay;
					var b2 = bx * bx + by * by;
					if (a2 > b2)
					{
						this.m_diagram[i] = b;
						if (x > 0)
						{
							queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x - 1, y, i - 1, b));
						}
						if (y > 0)
						{
							queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y - 1, i - this.m_countX, b));
						}
						if (x < this.m_countX - 1)
						{
							queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x + 1, y, i + 1, b));
						}
						if (y < this.m_countY - 1)
						{
							queue.Push(new b2VoronoiDiagram.b2VoronoiDiagramTask(x, y + 1, i + this.m_countX, b));
						}
						updated = true;
					}
				}
			}
			if (!updated)
			{
				break;
			}
		}
	},

	GetNodes: function(callback)
	{
		for (var y = 0; y < this.m_countY - 1; y++)
		{
			for (var x = 0; x < this.m_countX - 1; x++)
			{
				var i = x + y * this.m_countX;
				var a = this.m_diagram[i];
				var b = this.m_diagram[i + 1];
				var c = this.m_diagram[i + this.m_countX];
				var d = this.m_diagram[i + 1 + this.m_countX];
				if (b != c)
				{
					if (a != b && a != c)
					{
						callback(a.tag, b.tag, c.tag);
					}
					if (d != b && d != c)
					{
						callback(b.tag, d.tag, c.tag);
					}
				}
			}
		}
	}
};
