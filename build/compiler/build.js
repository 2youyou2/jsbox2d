function build()
{
	var cwd = (typeof(currentDirectory) === 'undefined') ? './' : currentDirectory;

	var compiled = processString(0, cwd, 'compile.html', "'#include @__INPUT__'");
	fileio.write(cwd + '/../' + definitions['__OUTPUT__'], compiled);
}