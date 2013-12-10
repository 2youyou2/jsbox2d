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

#include "imgui.h"
#include "RenderGL3.h"
#include "DebugDraw.h"
#include "Test.h"

#include <glew/glew.h>
#include <glfw/glfw3.h>
#include <stdio.h>

//
struct UIState
{
	bool showMenu;
	int scroll;
	int scrollarea1;
	bool mouseOverMenu;
	bool chooseTest;
};

//
namespace
{
	GLFWwindow* mainWindow = null;
	int windowWidth = 1280, windowHeight = 800;
	UIState ui;

	int32 testIndex = 0;
	int32 testSelection = 0;
	int32 testCount = 0;
	TestEntry* entry;
	Test* test;
	Settings settings;
	int32 framePeriod = 16;
	bool rightMouseDown;
	b2Vec2 lastp;
}

//
static void sCreateUI()
{
	ui.showMenu = true;
	ui.scroll = 0;
	ui.scrollarea1 = 0;
	ui.chooseTest = false;
	ui.mouseOverMenu = false;

	// Init UI
	if (!RenderGLInit("DroidSans.ttf"))
	{
		fprintf(stderr, "Could not init GUI renderer.\n");
		assert(false);
		return;
	}
}

//
static void sResizeWindow(GLFWwindow*, int width, int height)
{
	windowWidth = width;
	windowHeight = height;
	g_camera.m_width = float(width);
	g_camera.m_height = float(height);
}

//
static inline int32 FloatToInt(float32 x)
{
	return x >= 0.0 ? (int32)(x + 0.5) : (int32)(x + 0.5);
}

//
static void sKeyCallback(GLFWwindow*, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS)
	{
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
			// Quit
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);
			break;

		case GLFW_KEY_LEFT:
			// Pan left
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(2.0, 0.0);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.x -= 0.5;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_RIGHT:
			// Pan right
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(-2.0, 0.0);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.x += 0.5;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_DOWN:
			// Pan down
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(0.0, 2.0);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.y -= 0.5;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_UP:
			// Pan up
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(0.0, -2.0);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.y += 0.5;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_HOME:
			// Reset view
			g_camera.m_zoom = 1.0;
			g_camera.m_center.Set(0.0, 20.0);
			sResizeWindow(mainWindow, windowWidth, windowHeight);
			break;

		case GLFW_KEY_Z:
			// Zoom out
			g_camera.m_zoom = b2Min(1.1 * g_camera.m_zoom, 20.0);
			sResizeWindow(mainWindow, windowWidth, windowHeight);
			break;

		case GLFW_KEY_X:
			// Zoom in
			g_camera.m_zoom = b2Max(0.9 * g_camera.m_zoom, 0.02);
			sResizeWindow(mainWindow, windowWidth, windowHeight);
			break;

		case GLFW_KEY_R:
			// Reset test
			delete test;
			test = entry->createFcn();
			break;

		case GLFW_KEY_SPACE:
			// Launch a bomb.
			if (test)
			{
				test->LaunchBomb();
			}
			break;

		case GLFW_KEY_P:
			// Pause
			settings.pause = !settings.pause;
			break;

		case GLFW_KEY_LEFT_BRACKET:
			// Switch to previous test
			--testSelection;
			if (testSelection < 0)
			{
				testSelection = testCount - 1;
			}
			break;

		case GLFW_KEY_RIGHT_BRACKET:
			// Switch to next test
			++testSelection;
			if (testSelection == testCount)
			{
				testSelection = 0;
			}
			break;

		case GLFW_KEY_TAB:
			ui.showMenu = !ui.showMenu;

		default:
			if (test)
			{
				test->Keyboard(key);
			}
		}
	}
	else if (action == GLFW_RELEASE)
	{
		test->KeyboardUp(key);
	}
	// else GLFW_REPEAT
}

//
static void sMouseButton(GLFWwindow*, int32 button, int32 action, int32 mods)
{
	double xd, yd;
	glfwGetCursorPos(mainWindow, &xd, &yd);
	b2Vec2 ps((float32)xd, (float32)yd);

	// Use the mouse to move things around.
	if (button == GLFW_MOUSE_BUTTON_1)
	{
		b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
		if (action == GLFW_PRESS)
		{
			if (mods == GLFW_MOD_SHIFT)
			{
				test->ShiftMouseDown(pw);
			}
			else
			{
				test->MouseDown(pw);
			}
		}
		
		if (action == GLFW_RELEASE)
		{
			test->MouseUp(pw);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_2)
	{
		if (action == GLFW_PRESS)
		{	
			lastp = g_camera.ConvertScreenToWorld(ps);
			rightMouseDown = true;
		}

		if (action == GLFW_RELEASE)
		{
			rightMouseDown = false;
		}
	}
}

//
static void sMouseMotion(GLFWwindow*, double xd, double yd)
{
	b2Vec2 ps((float)xd, (float)yd);

	b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
	test->MouseMove(pw);
	
	if (rightMouseDown)
	{
		b2Vec2 diff = b2Vec2::Subtract(pw, lastp);
		g_camera.m_center.x -= diff.x;
		g_camera.m_center.y -= diff.y;
		lastp = g_camera.ConvertScreenToWorld(ps);
	}
}

//
static void sScrollCallback(GLFWwindow*, double, double dy)
{
	if (ui.mouseOverMenu)
	{
		ui.scroll = -int(dy);
	}
	else
	{
		if (dy > 0)
		{
			g_camera.m_zoom /= 1.1;
		}
		else
		{
			g_camera.m_zoom *= 1.1;
		}
	}
}

//
static void sRestart()
{
	delete test;
	entry = g_testEntries + testIndex;
	test = entry->createFcn();
	sResizeWindow(mainWindow, windowWidth, windowHeight);
}

//
static void sExit()
{
	// Quit
	glfwSetWindowShouldClose(mainWindow, GL_TRUE);
}

//
static void sSingleStep()
{
	settings.pause = 1;
	settings.singleStep = 1;
}

//
static void sSimulate()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float32 ratio = float32(windowWidth) / float32(windowHeight);

	b2Vec2 extents(ratio * 25.0, 25.0);
	extents.Multiply(g_camera.m_zoom);

	b2Vec2 lower = b2Vec2::Subtract(g_camera.m_center, extents);
	b2Vec2 upper = b2Vec2::Add(g_camera.m_center, extents);

	// L/R/B/T
	gluOrtho2D(lower.x, upper.x, lower.y, upper.y);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	test->Step(&settings);

	test->DrawTitle(entry->name);

	if (testSelection != testIndex)
	{
		testIndex = testSelection;
		delete test;
		entry = g_testEntries + testIndex;
		test = entry->createFcn();
		g_camera.m_zoom = 1.0;
		g_camera.m_center.Set(0.0, 20.0);
	}
}

//
static void sInterface()
{
	int menuWidth = 200;
	ui.mouseOverMenu = false;
	if (ui.showMenu)
	{
		bool over = imguiBeginScrollArea("Testbed Controls", windowWidth - menuWidth - 10, 10, menuWidth, windowHeight - 20, &ui.scrollarea1);
		if (over) ui.mouseOverMenu = true;

		imguiSeparatorLine();

		imguiLabel("Test");
		if (imguiButton(entry->name, true))
		{
			ui.chooseTest = !ui.chooseTest;
		}

		imguiSeparatorLine();

		imguiSlider("Vel Iters", &settings.velocityIterations, 0, 50, 1, true);
		imguiSlider("Pos Iters", &settings.positionIterations, 0, 50, 1, true);
		float tempHertz = (float)settings.hz;
		imguiSlider("Hertz", &tempHertz, 5.0f, 120.0f, 5.0f, true);

		if (imguiCheck("Sleep", settings.enableSleep, true))
			settings.enableSleep = !settings.enableSleep;
		if (imguiCheck("Warm Starting", settings.enableWarmStarting, true))
			settings.enableWarmStarting = !settings.enableWarmStarting;
		if (imguiCheck("Time of Impact", settings.enableContinuous, true))
			settings.enableContinuous = !settings.enableContinuous;
		if (imguiCheck("Sub-Stepping", settings.enableSubStepping, true))
			settings.enableSubStepping = !settings.enableSubStepping;

		imguiSeparatorLine();

		if (imguiCheck("Shapes", settings.drawShapes, true))
			settings.drawShapes = !settings.drawShapes;
		if (imguiCheck("Joints", settings.drawJoints, true))
			settings.drawJoints = !settings.drawJoints;
		if (imguiCheck("AABBs", settings.drawAABBs, true))
			settings.drawAABBs = !settings.drawAABBs;
		if (imguiCheck("Contact Points", settings.drawContactPoints, true))
			settings.drawContactPoints = !settings.drawContactPoints;
		if (imguiCheck("Contact Normals", settings.drawContactNormals, true))
			settings.drawContactNormals = !settings.drawContactNormals;
		if (imguiCheck("Contact Impulses", settings.drawContactImpulse, true))
			settings.drawContactImpulse = !settings.drawContactImpulse;
		if (imguiCheck("Friction Impulses", settings.drawFrictionImpulse, true))
			settings.drawFrictionImpulse = !settings.drawFrictionImpulse;
		if (imguiCheck("Center of Masses", settings.drawCOMs, true))
			settings.drawCOMs = !settings.drawCOMs;
		if (imguiCheck("Statistics", settings.drawStats, true))
			settings.drawStats = !settings.drawStats;
		if (imguiCheck("Profile", settings.drawProfile, true))
			settings.drawProfile = !settings.drawProfile;

		if (imguiButton("Pause", true))
			settings.pause = !settings.pause;

		if (imguiButton("Single Step", true))
			settings.singleStep = !settings.singleStep;

		if (imguiButton("Restart", true))
			sRestart();

		if (imguiButton("Quit", true))
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);

		imguiEndScrollArea();
	}

	int testMenuWidth = 200;
	if (ui.chooseTest)
	{
		static int testScroll = 0;
		bool over = imguiBeginScrollArea("Choose Sample", windowWidth - menuWidth - testMenuWidth - 20, 10, testMenuWidth, windowHeight - 20, &testScroll);
		if (over) ui.mouseOverMenu = true;

		for (int i = 0; i < testCount; ++i)
		{
			if (imguiItem(g_testEntries[i].name, true))
			{
				delete test;
				entry = g_testEntries + i;
				test = entry->createFcn();
				ui.chooseTest = false;
			}
		}

		imguiEndScrollArea();
	}

	imguiEndFrame();

}

//
int main(int argc, char** argv)
{
	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	char title[64];
	sprintf(title, "Box2D Testbed Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
	mainWindow = glfwCreateWindow(windowWidth, windowHeight, title, null, null);
	if (mainWindow == null)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);
	glfwSetScrollCallback(mainWindow, sScrollCallback);
	glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
	glfwSetKeyCallback(mainWindow, sKeyCallback);
	glfwSetMouseButtonCallback(mainWindow, sMouseButton);
	glfwSetCursorPosCallback(mainWindow, sMouseMotion);
	glfwSetScrollCallback(mainWindow, sScrollCallback);

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(EXIT_FAILURE);
	}

	sCreateUI();

	testCount = 0;
	while (g_testEntries[testCount].createFcn != null)
	{
		++testCount;
	}

	testIndex = b2Clamp(testIndex, 0, testCount - 1);
	testSelection = testIndex;

	entry = g_testEntries + testIndex;
	test = entry->createFcn();

	// Control the frame rate. One draw per monitor refresh.
	glfwSwapInterval(1);

	glClearColor(0.3, 0.3, 0.3, 1.f);
	// glfw scrolling
	int glfwscroll = 0;
	while (!glfwWindowShouldClose(mainWindow))
	{
		glfwGetWindowSize(mainWindow, &windowWidth, &windowHeight);
		glViewport(0, 0, windowWidth, windowHeight);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		unsigned char mousebutton = 0;
		int mscroll = ui.scroll;
		ui.scroll = 0;

		double xd, yd;
		glfwGetCursorPos(mainWindow, &xd, &yd);
		int mousex = int(xd);
		int mousey = int(yd);

		mousey = windowHeight - mousey;
		int leftButton = glfwGetMouseButton(mainWindow, GLFW_MOUSE_BUTTON_LEFT);
		if (leftButton == GLFW_PRESS)
			mousebutton |= IMGUI_MBUT_LEFT;

		imguiBeginFrame(mousex, mousey, mousebutton, mscroll);

		sSimulate();
		sInterface();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);
		RenderGLFlush(windowWidth, windowHeight);

		glfwSwapBuffers(mainWindow);

		glfwPollEvents();
	}

	RenderGLDestroy();
	glfwTerminate();

	return 0;
}
