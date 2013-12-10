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
		/*glColor3f(color.r, color.g, color.b);
		glBegin(GL_LINE_LOOP);
		for (int32 i = 0; i < vertexCount; ++i)
		{
			glVertex2f(vertices[i].x, vertices[i].y);
		}
		glEnd();*/

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
		/*glEnable(GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
		glBegin(GL_TRIANGLE_FAN);
		for (int32 i = 0; i < vertexCount; ++i)
		{
			glVertex2f(vertices[i].x, vertices[i].y);
		}
		glEnd();
		glDisable(GL_BLEND);

		glColor4f(color.r, color.g, color.b, 1.0f);
		glBegin(GL_LINE_LOOP);
		for (int32 i = 0; i < vertexCount; ++i)
		{
			glVertex2f(vertices[i].x, vertices[i].y);
		}
		glEnd();*/

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
		/*const float32 k_segments = 16.0f;
		const float32 k_increment = 2.0f * b2_pi / k_segments;
		float32 theta = 0.0f;
		glColor3f(color.r, color.g, color.b);
		glBegin(GL_LINE_LOOP);
		for (int32 i = 0; i < k_segments; ++i)
		{
			b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
			glVertex2f(v.x, v.y);
			theta += k_increment;
		}
		glEnd();*/

		this.context.strokeStyle = this.ColorFor(color, 1.0);
		this.context.beginPath();
		this.context.arc(center.x, center.y, radius, 0, Math.PI * 2);
		this.context.closePath();
		this.context.stroke();
	},

	DrawSolidCircle: function(center, radius, axis, color)
	{
		/*const float32 k_segments = 16.0f;
		const float32 k_increment = 2.0f * b2_pi / k_segments;
		float32 theta = 0.0f;
		glEnable(GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
		glBegin(GL_TRIANGLE_FAN);
		for (int32 i = 0; i < k_segments; ++i)
		{
			b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
			glVertex2f(v.x, v.y);
			theta += k_increment;
		}
		glEnd();
		glDisable(GL_BLEND);

		theta = 0.0f;
		glColor4f(color.r, color.g, color.b, 1.0f);
		glBegin(GL_LINE_LOOP);
		for (int32 i = 0; i < k_segments; ++i)
		{
			b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
			glVertex2f(v.x, v.y);
			theta += k_increment;
		}
		glEnd();

		b2Vec2 p = center + radius * axis;
		glBegin(GL_LINES);
		glVertex2f(center.x, center.y);
		glVertex2f(p.x, p.y);
		glEnd();*/

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
		/*glColor3f(color.r, color.g, color.b);
		glBegin(GL_LINES);
		glVertex2f(p1.x, p1.y);
		glVertex2f(p2.x, p2.y);
		glEnd();*/

		this.context.strokeStyle = this.ColorFor(color, 1.0);
		this.context.beginPath();
		this.context.moveTo(p1.x, p1.y);
		this.context.lineTo(p2.x, p2.y);
		this.context.closePath();
		this.context.stroke();
	},

	DrawTransform: function(xf)
	{
		/*b2Vec2 p1 = xf.p, p2;
		const float32 k_axisScale = 0.4f;
		glBegin(GL_LINES);

		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex2f(p1.x, p1.y);
		p2 = p1 + k_axisScale * xf.q.GetXAxis();
		glVertex2f(p2.x, p2.y);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex2f(p1.x, p1.y);
		p2 = p1 + k_axisScale * xf.q.GetYAxis();
		glVertex2f(p2.x, p2.y);

		glEnd();*/

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
		/*glPointSize(size);
		glBegin(GL_POINTS);
		glColor3f(color.r, color.g, color.b);
		glVertex2f(p.x, p.y);
		glEnd();
		glPointSize(1.0f);*/

		var hs = size / 2;
		this.context.fillStyle = this.ColorFor(color, 1.0);
		this.context.fillRect(p.x - hs, p.y - hs, size, size);
	},

	DrawAABB: function(aabb, c)
	{
		/*glColor3f(c.r, c.g, c.b);
		glBegin(GL_LINE_LOOP);
		glVertex2f(aabb->lowerBound.x, aabb->lowerBound.y);
		glVertex2f(aabb->upperBound.x, aabb->lowerBound.y);
		glVertex2f(aabb->upperBound.x, aabb->upperBound.y);
		glVertex2f(aabb->lowerBound.x, aabb->upperBound.y);
		glEnd();*/

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