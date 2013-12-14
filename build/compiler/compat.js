if (typeof(FileIO) === 'undefined')
{
	var FileIO = (function()
	{
		function FileIO()
		{
		}
		FileIO.prototype =
		{
			read: function(fileName)
			{
				var contents;
				var req = new XMLHttpRequest();
				req.onload = function() { contents = this.responseText; };
				req.onerror = function() { alert(this.statusText); };
				req.open('GET', fileName, false);
				req.send();
				return contents;
			},

			write: function(fileName, contents)
			{
				document.write(fileName + '<br/><pre>' + contents + '</pre><br/>');
			}
		};
		return FileIO;
	}());

	window['fileio'] = new FileIO();
}