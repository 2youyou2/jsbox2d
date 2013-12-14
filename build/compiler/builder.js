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

function processString(tabPos, folder, file, s)
{
	//for (var i = 0; i < tabPos; ++i)
	//	Console.Write("  ");
	//Console.WriteLine("Parsing " + file + "...");
	var r = /'#(\S*)(?:\s??)([\s\S]*?)';?/ig;

	return s.replace(r,
		function (match, func, arg)
		{
			var args = null;

			if (arg)
				args = processArguments(arg.trim());

			if (func === "include")
			{
				var path = args[0];
				var combined = folder + '/' + path;

				//for (var i = 0; i < tabPos; ++i)
				//	Console.Write("  ");
				//Console.WriteLine("Including " + combined + "...");

				return processString(tabPos + 1, folder, path, fileio.read(combined));
			}
			else if (func === "export")
			{
				var compiled = '';
				var names = {};

				for (var i = 0; i < args.length; ++i)
				{
					var name = args[i].trim();
					var trimmed = name.replace(/b2(?:_?)/ig, '');

					if (trimmed.length)
					{
						if (names[trimmed])
							alert(trimmed);
						names[trimmed] = true;

						trimmed = trimmed.substr(0, 1).toUpperCase() + trimmed.substr(1);
						compiled += 'window["' + name + '"] = ' + name + ';';
					}
				}

				return compiled;
			}

			return match;
		});
}
