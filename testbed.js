"use strict";

// Fix Function#name on browsers that do not support it (IE):
if (typeof((function f() {}).name) === 'undefined') {
	Object.defineProperty(Function.prototype, 'name', {
		get: function() {
			var name = this.toString().match(/^\s*function\s*(\w*)\s*\(/)[1];
			// For better performance only parse once, and then cache the
			// result through a new accessor for repeated access.
			Object.defineProperty(this, 'name', { value: name });
			return name;
		}
	});
}

function loadTestBed()
{
	var canvas = document.getElementById('canvas');
	var cQ = $(canvas);

	canvas.width = canvas.clientWidth;
	canvas.height = canvas.clientHeight;

	var context = canvas.getContext('2d');

	var tests =
		[
			TestPyramid,
			TestEdge,
			TestSphereStack,
			TestConveyorBelt,
			TestVaryingRestitution,
			TestVaryingFriction,
			TestContinuous,
			TestAddPair,
			TestTiles,
			TestTumbler,
			TestBridge,
			TestChain,
			TestRevolute,
			TestTheoJansenWalker,
			TestSliderCrank,
			TestConfined,
			TestPinball,
			TestBullet,
			TestCharacterCollision,
			TestCollisionFiltering,
			TestCollisionProcessing,
			TestCompoundShapes,
			TestSensor,
			TestDominos,
			TestApplyForce,
			TestWeb,
			TestBodyTypes,
			TestBreakable,
			TestConvexHull,
			TestCantilever,
			TestCar,
			TestDistance,
			TestEdgeShapes,
			TestGears,
			TestMobile,
			TestMobileBalanced,
			TestMotorJoint,
			TestOneSidedPlatform,
			TestPolyCollision,
			TestPolyShapes,
			TestPrismatic,
			TestPulleys,
			TestRayCast,
			TestRope,
			TestRopeJoint,
			TestShapeEditing,
			TestVerticalStack,
			TestRUBE,
			TestRUBEJointTypes,
			TestPlinko,
			TestBulldozer,
			TestSnake,
			TestDamBreak,
			TestSurfaceTension,
			TestChaosPendulum,
			TestWaveMachine,
			TestSoup
		];

	tests.sort(function(a, b)
	{
		return a.name.localeCompare(b.name);
	});

	var testSelect = $('#test');

	for (var i = 0; i < tests.length; ++i)
		testSelect.append('<option>' + tests[i].name.substr(4) + '</option>');

	testSelect[0].selectedIndex = 4;
	var currentTest = null;

	var debugDraw = new CanvasDebugDraw();
	debugDraw.context = context;
	debugDraw.scale = 14;
	debugDraw.SetFlags(b2Draw.e_shapeBit | b2Draw.e_jointBit);
	var dbg = $('#debugText');

	var drawLine = function(string)
	{
		dbg.text(dbg.text() + string + '\n');
	};

	testSelect.change(function()
	{
		var index = testSelect.prop("selectedIndex");
		var test = tests[index];

		var wasPaused = currentTest ? currentTest.m_pause : false;

		currentTest = new test();
		currentTest.m_debugDraw = debugDraw;
		currentTest.m_world.SetDebugDraw(debugDraw);
		currentTest.m_drawStringFunc = drawLine;
		currentTest.Initialize();

		currentTest.m_hz = 1 / $('#hertz').val();
		currentTest.m_hz_raw = $('#hertz').val();
		currentTest.m_velIters = $('#velIters').val();
		currentTest.m_posIters = $('#posIters').val();
		currentTest.m_world.SetAllowSleeping($('#allowSleep').is(':checked'));
		currentTest.m_world.SetWarmStarting($('#warmStarting').is(':checked'));
		currentTest.m_world.SetContinuousPhysics($('#toi').is(':checked'));
		currentTest.m_world.SetSubStepping($('#substep').is(':checked'));

		currentTest.m_pause = wasPaused;
		currentTest.m_scale = debugDraw.scale;
	});

	$('#hertz').change(function()
	{
		currentTest.m_hz = 1 / $('#hertz').val();
		currentTest.m_hz_raw = $('#hertz').val();
		$('#label_hertz').find('.val').text($('#hertz').val());
	});

	$('#velIters').change(function()
	{
		currentTest.m_velIters = $('#velIters').val();
		$('#label_velIters').find('.val').text($('#velIters').val());
	});

	$('#posIters').change(function()
	{
		currentTest.m_posIters = $('#posIters').val();
		$('#label_posIters').find('.val').text($('#posIters').val());
	});


	$('#allowSleep').click(function()
	{
		currentTest.m_world.SetAllowSleeping($('#allowSleep').is(':checked'));
	});

	$('#warmStarting').click(function()
	{
		currentTest.m_world.SetWarmStarting($('#warmStarting').is(':checked'));
	});

	$('#toi').click(function()
	{
		currentTest.m_world.SetContinuousPhysics($('#toi').is(':checked'));
	});

	$('#substep').click(function()
	{
		currentTest.m_world.SetSubStepping($('#substep').is(':checked'));
	});


	$('#drawShapes').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_shapeBit);
	});

	$('#drawJoints').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_jointBit);
	});

	$('#drawAABBs').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_aabbBit);
	});

	$('#drawCenters').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_centerOfMassBit);
	});

	$('#drawPairs').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_pairBit);
	});

	$('#drawContactPoints').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_contactPoints);
	});

	$('#drawContactNormals').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_contactNormals);
	});

	$('#drawContactImpulses').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_contactImpulses);
	});

	$('#drawFrictionImpulses').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_frictionImpulses);
	});

	$('#drawStatistics').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_statistics);
	});

	$('#drawProfile').click(function()
	{
		debugDraw.ToggleFlags(b2Draw.e_profile);
	});


	$('#pause').click(function()
	{
		currentTest.m_pause = !currentTest.m_pause;
	});

	$('#step').click(function()
	{
		currentTest.m_pause = true;
		currentTest.m_singleStep = true;
	});

	$('#restart').click(function()
	{
		testSelect.change();
	});

	var m_clickPos;

	cQ.attr("tabindex", 0);

	cQ.mousedown(function(e)
	{
		var x = e.clientX;
		var y = canvas.height - e.clientY;

		x -= canvas.width / 2;
		y -= 128;

		x /= debugDraw.scale;
		y /= debugDraw.scale;

		if (e.which === 1)
			currentTest.MouseDown(new b2Vec2(x, y));
		else if (e.which === 3)
			m_clickPos = [x, y];

		cQ.focus();

		e.preventDefault();
		return false;
	});

	cQ.mouseup(function(e)
	{
		var x = e.clientX;
		var y = canvas.height - e.clientY;

		x -= canvas.width / 2;
		y -= 128;

		x /= debugDraw.scale;
		y /= debugDraw.scale;

		if (e.which === 1)
			currentTest.MouseUp(new b2Vec2(x, y));

		e.preventDefault();
		return false;
	});

	cQ.mousemove(function(e)
	{
		var x = e.clientX;
		var y = canvas.height - e.clientY;

		x -= canvas.width / 2;
		y -= 128;

		x /= debugDraw.scale;
		y /= debugDraw.scale;

		if (e.which === 1)
			currentTest.MouseMove(new b2Vec2(x, y));
		else if (e.which === 3)
		{
			var diff = [x - m_clickPos[0], y - m_clickPos[1]];

			currentTest.m_center.x -= diff[0];
			currentTest.m_center.y -= diff[1];

			m_clickPos = [x, y];
		}

		e.preventDefault();
		return false;
	});


	function mouseWheel(e)
	{
	var evt=window.event || e; //equalize event object
	var delta=evt.detail? evt.detail*(-120) : evt.wheelDelta; //delta returns +120 when wheel is scrolled up, -120 when scrolled down

	if (delta < 0)
	{
		debugDraw.scale--;
		currentTest.m_scale--;
	}
	else
	{
		debugDraw.scale++;
		currentTest.m_scale++;
	}

	if (evt.preventDefault) //disable default wheel action of scrolling page
		evt.preventDefault();

	return false;
	}

	var mousewheelevt=(/Firefox/i.test(navigator.userAgent))? "DOMMouseScroll" : "mousewheel"; //FF doesn't recognize mousewheel as of FF3.x

	if (cQ[0].attachEvent) //if IE (and Opera depending on user setting)
	cQ[0].attachEvent("on"+mousewheelevt, mouseWheel);
	else if (cQ[0].addEventListener) //WC3 browsers
	cQ[0].addEventListener(mousewheelevt, mouseWheel, false);


	cQ[0].oncontextmenu = function() { return false; };

	$(document).keydown(function(e)
	{
		if (e.keyCode === 'R'.charCodeAt())
			testSelect.change();
		else
			currentTest.Keyboard(e.keyCode);
	});

	$(document).keyup(function(e)
	{
		currentTest.KeyboardUp(e.keyCode);
	});

	testSelect.change();

	window.requestAnimationFrame = window.requestAnimationFrame || window.mozRequestAnimationFrame ||
					  window.webkitRequestAnimationFrame || window.msRequestAnimationFrame;

	b2Profiler.init();

	function step()
	{
		dbg.text("");
		context.save();
		context.clearRect(0, 0, canvas.width, canvas.height);

		currentTest.Step();
		context.restore();

		window.requestAnimationFrame(step);
	}

	window.requestAnimationFrame(step);
}