/*
* Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "DebugDraw.h"

#include <glew/glew.h>
#include <glfw/glfw3.h>
#include <stdio.h>
#include <stdarg.h>

#include "RenderGL3.h"

DebugDraw g_debugDraw;
Camera g_camera;

//
b2Vec2 Camera::ConvertScreenToWorld(const b2Vec2& ps)
{
	float32 u = ps.x / this->m_width;
	float32 v = (this->m_height - ps.y) / this->m_height;

	float32 ratio = this->m_width / this->m_height;
	b2Vec2 extents(ratio * 25.0, 25.0);
	extents.Multiply(this->m_zoom);

	b2Vec2 lower = b2Vec2::Subtract(this->m_center, extents);
	b2Vec2 upper = b2Vec2::Add(this->m_center, extents);

	b2Vec2 pw;
	pw.x = (1.0 - u) * lower.x + u * upper.x;
	pw.y = (1.0 - v) * lower.y + v * upper.y;
	return pw;
}

//
b2Vec2 Camera::ConvertWorldToScreen(const b2Vec2& pw)
{
	float32 ratio = this->m_width / this->m_height;
	b2Vec2 extents(ratio * 25.0, 25.0);
	extents.Multiply(this->m_zoom);

	b2Vec2 lower = b2Vec2::Subtract(this->m_center, extents);
	b2Vec2 upper = b2Vec2::Add(this->m_center, extents);

	float32 u = (pw.x - lower.x) / (upper.x - lower.x);
	float32 v = (pw.y - lower.y) / (upper.y - lower.y);

	b2Vec2 ps;
	ps.x = u * this->m_width;
	ps.y = (1.0 - v) * this->m_height;
	return ps;
}

//
void DebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x, vertices[i].y);
	}
	glEnd();
}

void DebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5);
	glBegin(GL_TRIANGLE_FAN);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x, vertices[i].y);
	}
	glEnd();
	glDisable(GL_BLEND);

	glColor4f(color.r, color.g, color.b, 1.0);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x, vertices[i].y);
	}
	glEnd();
}

void DebugDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color)
{
	const float32 k_segments = 16.0;
	const float32 k_increment = 2.0 * b2_pi / k_segments;
	float32 theta = 0.0;
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = b2Vec2::Add(center, b2Vec2::Multiply(radius, b2Vec2(cosf(theta), sinf(theta))));
		glVertex2f(v.x, v.y);
		theta += k_increment;
	}
	glEnd();
}

void DebugDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color)
{
	const float32 k_segments = 16.0;
	const float32 k_increment = 2.0 * b2_pi / k_segments;
	float32 theta = 0.0;
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5);
	glBegin(GL_TRIANGLE_FAN);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = b2Vec2::Add(center, b2Vec2::Multiply(radius, b2Vec2(cosf(theta), sinf(theta))));
		glVertex2f(v.x, v.y);
		theta += k_increment;
	}
	glEnd();
	glDisable(GL_BLEND);

	theta = 0.0;
	glColor4f(color.r, color.g, color.b, 1.0);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = b2Vec2::Add(center, b2Vec2::Multiply(radius, b2Vec2(cosf(theta), sinf(theta))));
		glVertex2f(v.x, v.y);
		theta += k_increment;
	}
	glEnd();

	b2Vec2 p = b2Vec2::Add(center, b2Vec2::Multiply(radius, axis));
	glBegin(GL_LINES);
	glVertex2f(center.x, center.y);
	glVertex2f(p.x, p.y);
	glEnd();
}

void DebugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINES);
	glVertex2f(p1.x, p1.y);
	glVertex2f(p2.x, p2.y);
	glEnd();
}

void DebugDraw::DrawTransform(const b2Transform& xf)
{
	b2Vec2 p1 = xf.p, p2;
	const float32 k_axisScale = 0.4;
	glBegin(GL_LINES);
	
	glColor3f(1.0, 0.0, 0.0);
	glVertex2f(p1.x, p1.y);
	p2 = b2Vec2::Add(p1, b2Vec2::Multiply(k_axisScale, xf.q.GetXAxis()));
	glVertex2f(p2.x, p2.y);

	glColor3f(0.0, 1.0, 0.0);
	glVertex2f(p1.x, p1.y);
	p2 = b2Vec2::Add(p1, b2Vec2::Multiply(k_axisScale, xf.q.GetYAxis()));
	glVertex2f(p2.x, p2.y);

	glEnd();
}

void DebugDraw::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color)
{
	glPointSize(size);
	glBegin(GL_POINTS);
	glColor3f(color.r, color.g, color.b);
	glVertex2f(p.x, p.y);
	glEnd();
	glPointSize(1.0);
}

void DebugDraw::DrawString(int x, int y, const char *string, ...)
{
	float32 h = g_camera.m_height;

	char buffer[128];

	va_list arg;
	va_start(arg, string);
	vsprintf(buffer, string, arg);
	va_end(arg);

	AddGfxCmdText(float(x), h - float(y), TEXT_ALIGN_LEFT, buffer, SetRGBA(230, 153, 153, 255));
}

void DebugDraw::DrawString(const b2Vec2& pw, const char *string, ...)
{
	b2Vec2 ps = g_camera.ConvertWorldToScreen(pw);
	float32 h = g_camera.m_height;

	char buffer[128];

	va_list arg;
	va_start(arg, string);
	vsprintf(buffer, string, arg);
	va_end(arg);

	AddGfxCmdText((float)ps.x, (float)(h - ps.y), TEXT_ALIGN_LEFT, buffer, SetRGBA(230, 153, 153, 255));
}

void DebugDraw::DrawAABB(b2AABB* aabb, const b2Color& c)
{
	glColor3f(c.r, c.g, c.b);
	glBegin(GL_LINE_LOOP);
	glVertex2f(aabb->lowerBound.x, aabb->lowerBound.y);
	glVertex2f(aabb->upperBound.x, aabb->lowerBound.y);
	glVertex2f(aabb->upperBound.x, aabb->upperBound.y);
	glVertex2f(aabb->lowerBound.x, aabb->upperBound.y);
	glEnd();
}
