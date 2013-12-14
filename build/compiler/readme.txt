This is a built system that uses JS as a backend.
On Windows, this will run using the Noesis JS.NET library, which uses V8 as a backend.
To properly run the build script, you must run every JS file, then call the function "build".
The following type must be exposed to the global namespace:

interface FileIO
{
	string read(string fileName);
	void write(string fileName, string contents);
}

As well, there must be an instance of File called "fileio" exposed.

Included with the build system is a pure JS backend named "compile" that you can run from any platform. It will use XMLHttpRequests to load the files (you must have a local web server for this to work properly) and will simply echo the result to the browser.