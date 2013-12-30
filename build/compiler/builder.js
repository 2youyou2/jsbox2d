function StringConsumer(str)
{
	this.str = str;
	this.pos = 0;
	this.len = str.length;
}

StringConsumer.prototype =
{
	reset: function(str)
	{
		if (str)
		{
			this.str = str;
			this.len = str.length;
		}

		this.pos = 0;
	},

	end: function(pos)
	{
		if (typeof(pos) === 'undefined')
			return this.pos === this.len;

		return pos === this.len;
	},

	get: function()
	{
		if (this.end())
			return false;

		return this.str[this.pos++];
	},

	peekChar: function()
	{
		if (this.end())
			return false;

		return this.str[this.pos];
	},

	peek: function(token, max)
	{
		if (this.end())
			return false;

		if (token.length === 1)
			return (this.str[this.pos] === token) ? token : false;

		var result = '';
		var matchesLeft = max;


		var p = this.pos;
		while (!this.end(p) && this.str[p].match(token) !== null)
		{
			result += this.str[p++];

			if (max && !--matchesLeft)
				break;
		}

		if (result.length)
			return result;

		return false;
	},

	consume: function(token, removeWhitespace, max)
	{
		if (removeWhitespace)
			this.skipWhitespace();

		var result = result = this.peek(token, max);

		if (result === false)
			return false;

		var v = this.str.substr(this.pos, result.length);
		this.pos += result.length;
		return v;
	},

	skipWhitespace: function()
	{
		this.consume(/\s/);
	}
};

function processArguments(args)
{
	var list = [];
	var len = args.length, i = 0;
	var c = '\0';
	var tok = '';

	while (true)
	{
		var done = i >= len;

		if (!done)
		{
			c = args[i];
			++i;
		}

		if (c === '\"' && tok.Length == 0)
		{
			while (true)
			{
				c = args[i++];

				if (c === '\"' || i >= len)
					break;

				tok += c;
			}

			done = i >= len;
		}
		else if (!done)
			tok += c;

		if (tok.Length != 0 && (' \t\n\r\v'.indexOf(c) > -1 || done))
		{
			list.push(tok);
			tok = '';
		}

		if (done)
			break;
	}

	return list;
}

var definitions = {};

function evaluate(str)
{
	var replaced = str.replace(/@(\w+)/ig,
	function (match, name)
	{
		return 'definitions["' + name + '"]';
	});

	return eval('(' + replaced + ')');
}

function processString(tabPos, folder, file, s)
{
	var consumer = new StringConsumer(s);
	var processed = '';
	var conditionals = [];

	while (true)
	{
		if (consumer.end())
			break;

		if (consumer.consume("'"))
		{
			if (consumer.consume("#"))
			{
				// escape this thing with ##
				if (consumer.consume("#"))
				{
					processed += "'#";
					continue;
				}

				// get all of it
				var line = consumer.consume(/[^']/gi, true);
				consumer.consume("'", true);
				consumer.consume(";", true);

				var nameIndex = line.search(/\s/gi);

				if (nameIndex === -1)
					nameIndex = line.length;

				var name = line.substr(0, nameIndex).toLowerCase();
				var arg = line.substr(nameIndex + 1);
				var args = processArguments(arg.trim());

				switch (name)
				{
					case 'include':
						{
							var path;

							try
							{
								path = evaluate(args[0]);
							}
							catch (e)
							{
								path = args[0];
							}
							
							processed += processString(tabPos + 1, folder, path, fileio.read(path));
						}
						break;
					case 'def':
						processed += evaluate(args[0]);
						break;
					case 'export':
						{
							var names = {};
							var split = evaluate(arg.trim()).split(' ');

							for (var i = 0; i < split.length; ++i)
							{
								var name = split[i].trim();
								var trimmed = name.replace(/b2(?:_?)/ig, '');

								if (trimmed.length)
								{
									if (names[trimmed])
										alert(trimmed);

									names[trimmed] = name;

									trimmed = trimmed.substr(0, 1).toUpperCase() + trimmed.substr(1);
								}
							}

							processed += 'var mappings = [';
							var first = true;

							for (var x in names)
							{
								if (!first)
									processed += ',';
								else
									first = false;

								processed += '{"trimmed":"' + x + '","name":"' + names[x] + '","def":' + names[x] + '}';
							}

							processed += '];\n';

							// offer C++ compatibility mode for web users
							// TODO: replace sections below with #include
							processed +=
'if (typeof(b2_compatibility) !== "undefined" && typeof(window) !== "undefined")\n\
{\n\
	for (var i = 0; i < mappings.length; ++i)\n\
		window[mappings[i].name] = mappings[i].def;\n\
}\n\
else\n\
{\n\
var b2 = {};\n\
\n\
for (var i = 0; i < mappings.length; ++i)\n\
	b2[mappings[i].trimmed] = mappings[i].def;\n\
\n\
if (typeof(module) !== "undefined")\n\
	module.exports = b2;\n\
else\n\
	window["b2"] = b2;\n\
}';
						}
						break;
					case 'define':
						{
							var trimmed = arg.trim();
							definitions[args[0].trim()] = evaluate(trimmed.substr(trimmed.indexOf(' ') + 1));
						}
						break;
					case 'if':
						{
							var nest = 1;
							var val = evaluate(arg);
							conditionals.push(val);

							var inside = '';

							while (true)
							{
								var c = consumer.consume(/[^']/gi);

								if (c !== false)
									inside += c;

								var begin = consumer.pos;

								if (!consumer.consume("'"))
									throw new Error("expecting elif, else or endif");
								else if (!consumer.consume("#"))
								{
									inside += "'";
									continue;
								}

								// found a '#
								// if it's an elif or else, then we have to double-back and let execution continue below
								var name = consumer.consume(/[^'\s]/gi);
								var lower = name ? name.toLowerCase() : null;
								var breakOut = false;

								if (name)
								{
									switch (lower)
									{
										case 'if':
											++nest;
											break;
										case 'else':
										case 'elif':
										case 'endif':
										{
											if (!--nest)
											{
												// good - push back the consumer to the start of this expression
												consumer.pos = begin;

												// if our last expression succeeded, parse and include the text we got from here
												if (val)
													processed += processString(tabPos, folder, file, inside);

												// we're done here
												breakOut = true;
												break;
											}
										}
									}

									if (breakOut)
										break;
								}

								// not an elif/else, keep trying
								inside += "'#" + name;
							}
						}
						break;
					case 'elif':
						{
							debugger;

							// if the last conditional was true, we don't have to bother evaluating this one.
							var quickExit = conditionals[conditionals.length - 1];
							val = quickExit ? false : evaluate(arg);

							conditionals.pop();
							conditionals.push(quickExit ? true : val); // if we quick-exited, we push true so that further elifs/elses don't get hit

							var inside = '';

							while (true)
							{
								inside += consumer.consume(/[^']/gi);
								var begin = consumer.pos;

								if (!consumer.consume("'"))
									throw new Error("expecting elif, else or endif");
								else if (!consumer.consume("#"))
								{
									inside += "'";
									continue;
								}

								// found a '#
								// if it's an elif or else, then we have to double-back and let execution continue below
								var name = consumer.consume(/[^'\s]/gi);
								var lower = name ? name.toLowerCase() : null;

								if (name && (lower === 'else' || lower === 'elif' || lower === 'endif'))
								{
									// good - push back the consumer to the start of this expression
									consumer.pos = begin;

									// if our last expression succeeded, parse and include the text we got from here
									if (!quickExit && val)
										processed += processString(tabPos, folder, file, inside);

									// we're done here
									break;
								}
								else
									inside += "'#" + name;

								// not an elif/else, keep trying
							}
						}
						break;
					case 'else':
						{
							// only evaluate if the last condition is false
							val = conditionals[conditionals.length - 1];

							conditionals.pop();
							conditionals.push(true); // this is for endif
							var inside = '';

							while (true)
							{
								inside += consumer.consume(/[^']/gi);
								var begin = consumer.pos;

								if (!consumer.consume("'"))
									throw new Error("expecting endif");
								else if (!consumer.consume("#"))
								{
									inside += "'";
									continue;
								}

								// found a '#
								// if it's an elif or else, then we have to double-back and let execution continue below
								var name = consumer.consume(/[^'\s]/gi);
								var lower = name ? name.toLowerCase() : null;

								if (name && (lower === 'endif'))
								{
									// good - push back the consumer to the start of this expression
									consumer.pos = begin;

									// if our last expressions failed, parse and include the text we got from here
									if (!val)
										processed += processString(tabPos, folder, file, inside);

									// we're done here
									break;
								}
								else
									inside += "'#" + name;

								// not an elif/else, keep trying
							}
						}
						break;
					case 'endif':
						{
							conditionals.pop();
						}
						break;
				}
			}
			else
				processed += "'";
		}
		else
			processed += consumer.consume(/[^']/gi);
	}

	return processed;
}
