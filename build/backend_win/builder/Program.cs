using Noesis.Javascript;
using System;
using System.CodeDom.Compiler;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace builder
{
	//Add this code to the Class File
	public class FileIO
	{
		public string read(string filename)
		{
			return File.ReadAllText(filename).Replace("\r\n", "\n");
		}

		public void write(string filename, string contents)
		{
			File.WriteAllText(filename, contents);
		}
	}

	static class Program
	{
		static JavascriptContext ctx = new JavascriptContext();

		private static void AddClass<T1>(string instanceName = null)
			where T1 : new()
		{
			string className = typeof(T1).Name;
			Func<T1> _constructDelegate = () => { return new T1(); };

			ctx.SetParameter(className, _constructDelegate);

			if (instanceName != null)
				ctx.Run(String.Format("{0} = new {1}();", instanceName, className));
		}

		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		[STAThread]
		static void Main(string[] args)
		{
			AddClass<FileIO>("fileio");

			foreach (var f in Directory.EnumerateFiles(".", "*.js"))
				ctx.Run(File.ReadAllText(f));

			Dictionary<string, string> arguments = new Dictionary<string, string>();

			for (var i = 0; i < args.Length; ++i)
			{
				var vals = args[i].Split('=');

				arguments.Add(vals[0], vals[1]);
			}

			if (arguments.ContainsKey("pArgs"))
			{
				var a = arguments["pArgs"].Split(';');

				for (var i = 0; i < a.Length; ++i)
				{
					var ns = a[i].Split(':');

					if (ns.Length == 1)
						ctx.Run("definitions[\"" + ns[0] + "\"] = true;");
					else
						ctx.Run("definitions[\"" + ns[0] + "\"] = " + ns[1] + ";");
				}
			}

			ctx.Run("build()");
		}
	}
}
