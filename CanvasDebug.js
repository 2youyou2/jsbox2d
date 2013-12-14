function CanvasDebugDraw()
{
	this.parent.call(this);
	this.context = null;
}

CanvasDebugDraw.prototype =
{
	//
	DrawPolygon: function(vertices, vertexCount, color)
	{
		this.context.strokeStyle = this.ColorFor(color, 1.0);
		this.context.beginPath();
		this.context.moveTo(vertices[0].x, vertices[0].y);

		for (var i = 1; i < vertexCount; ++i)
			this.context.lineTo(vertices[i].x, vertices[i].y);

		this.context.closePath();
		this.context.stroke();
	},

	DrawSolidPolygon: function(vertices, vertexCount, color)
	{
		this.context.fillStyle = this.ColorFor(new b2Color(color.r * 0.5, color.g * 0.5, color.b * 0.5)	, 0.5);
		this.context.strokeStyle = this.ColorFor(color, 1.0);
		this.context.beginPath();
		this.context.moveTo(vertices[0].x, vertices[0].y);

		for (var i = 1; i < vertexCount; ++i)
			this.context.lineTo(vertices[i].x, vertices[i].y);

		this.context.closePath();
		this.context.stroke();
		this.context.fill();
	},

	DrawCircle: function(center, radius, color)
	{
		this.context.strokeStyle = this.ColorFor(color, 1.0);
		this.context.beginPath();
		this.context.arc(center.x, center.y, radius, 0, Math.PI * 2);
		this.context.closePath();
		this.context.stroke();
	},

	DrawSolidCircle: function(center, radius, axis, color)
	{
		this.context.fillStyle = this.ColorFor(new b2Color(color.r * 0.5, color.g * 0.5, color.b * 0.5), 0.5);
		this.context.strokeStyle = this.ColorFor(color, 1.0);
		this.context.beginPath();
		this.context.arc(center.x, center.y, radius, 0, Math.PI * 2);
		this.context.moveTo(center.x, center.y);
		var p = b2Vec2.Add(center, b2Vec2.Multiply(radius, axis));
		this.context.lineTo(p.x, p.y);
		this.context.closePath();
		this.context.stroke();
		this.context.fill();
	},

	DrawSegment: function(p1, p2, color)
	{
		this.context.strokeStyle = this.ColorFor(color, 1.0);
		this.context.beginPath();
		this.context.moveTo(p1.x, p1.y);
		this.context.lineTo(p2.x, p2.y);
		this.context.closePath();
		this.context.stroke();
	},

	DrawTransform: function(xf)
	{
		var k_axisScale = 0.4;
		var p1 = xf.p, p2;

		this.context.strokeStyle = this.ColorFor(new b2Color(1, 0, 0), 1.0);
		this.context.beginPath();
		this.context.moveTo(p1.x, p1.y);
		p2 = b2Vec2.Add(p1, b2Vec2.Multiply(k_axisScale, xf.q.GetXAxis()));
		this.context.lineTo(p2.x, p2.y);
		this.context.closePath();
		this.context.stroke();

		this.context.strokeStyle = this.ColorFor(new b2Color(0, 1, 0), 1.0);
		this.context.beginPath();
		this.context.moveTo(p1.x, p1.y);
		p2 = b2Vec2.Add(p1, b2Vec2.Multiply(k_axisScale, xf.q.GetYAxis()));
		this.context.lineTo(p2.x, p2.y);
		this.context.closePath();
		this.context.stroke();
	},

	DrawPoint: function(p, size, color)
	{
		var hs = size / 2;
		this.context.fillStyle = this.ColorFor(color, 1.0);
		this.context.fillRect(p.x - hs, p.y - hs, size, size);
	},

	DrawAABB: function(aabb, c)
	{
		this.context.fillStyle = this.ColorFor(c, 1.0);
		this.context.rect(aabb.lowerBound.x, aabb.lowerBound.y, aabb.upperBound.x - aabb.lowerBound.x, aabb.upperBound.y - aabb.lowerBound.y);
		this.context.stroke();
	},

	ColorFor: function(c, a)
	{
		var r = Math.floor(c.r * 255);
		var g = Math.floor(c.g * 255);
		var b = Math.floor(c.b * 255);

		return 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';
	}
};

CanvasDebugDraw._extend(b2Draw);