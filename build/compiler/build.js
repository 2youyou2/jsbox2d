function build()
{
	var cwd = (typeof(currentDirectory) === 'undefined') ? './' : currentDirectory;

	var compiled = processString(0, cwd, 'compile.html', "'#include build.inc';");
	fileio.write(cwd + '/../jsbox2d.js', compiled);
}