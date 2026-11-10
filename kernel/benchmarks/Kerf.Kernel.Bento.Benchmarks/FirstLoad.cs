// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Reflection;
using System.Runtime;
using System.Runtime.CompilerServices;
using System.Runtime.Loader;
using Kerf.Kernel.Benchmarks;
using Kerf.Modeling;
using Assembly = System.Reflection.Assembly;

// A first fillet, cut and STEP export, first thing in a new process, with Kerf loaded as a Bento extension loads it
// (decisions 1891 and 1892). This assembly stands in for the extension and always loads as Bento loads one: collectible,
// from bytes. Only where Kerf loads differs. By hand, from the published folder:
//   Kerf.Kernel.Bento.Benchmarks first-load bytes|files|normal
// (normal, not in the rig: everything in the default context, what the ReadyToRun code does when nothing is in its way)
[SuppressMessage("Usage", "CA2255:The 'ModuleInitializer' attribute should not be used in libraries", Justification = "An application, and it has to run before Main, which refers to Kerf")]
internal static class FirstLoad
{
    private const string Group = "first use";

    /// <summary>Takes the run before Main, as compiling Main would load Kerf into the default context, where Bento never has it.</summary>
    [ModuleInitializer]
    internal static void Start()
    {
        string[] args = Environment.GetCommandLineArgs();
        if (args.Length == 3 && args[1] == "first-load" && AssemblyLoadContext.GetLoadContext(typeof(FirstLoad).Assembly) == AssemblyLoadContext.Default)
        {
            Run(args[2]);
            Environment.Exit(0);
        }
    }

    private static void Run(string how)
    {
        string name = how switch
        {
            "bytes" => "loaded as Bento does: collectible, from bytes",
            "files" => "Kerf from its files, not collectible",
            "normal" => "loaded normally",
            _ => throw new ArgumentException(how),
        };
        string folder = AppContext.BaseDirectory;
        var clock = Stopwatch.StartNew();
        string volume;
        if (how == "normal")
        {
            volume = FirstLoadProbe.Work();
        }
        else
        {
            var extension = new ExtensionContext(folder, how == "files" ? new SharedContext(folder) : null);
            Assembly probe = extension.FromBytes(typeof(FirstLoad).Assembly.Location);
            volume = (string)probe.GetType(typeof(FirstLoadProbe).FullName!, throwOnError: true)!.GetMethod(nameof(FirstLoadProbe.Work))!.Invoke(null, null)!;
            // The context has to be held until the work is done (unit 15G, decision 28201): the runtime keeps a collectible
            // context alive only through references to the context itself, not through its assemblies' types or the frames
            // running their code, and once it's collected its finalizer starts its unload, after which it refuses to load
            // anything more. With the local dead after the line above, a full collection falling between two of Kerf's
            // assemblies being resolved (the fillet and the cut, before the STEP export brings in Kerf.Kernel.DataExchange)
            // took the process down with "AssemblyLoadContext is unloading or was already unloaded": every time with a 1 MB
            // gen0 budget, now and then on a loaded machine. Bento's loader holds its context in a field, so Bento is fine.
            GC.KeepAlive(extension);
        }

        clock.Stop();
        BenchRows.Add(Group, name, "fillet, cut and STEP ms", clock.Elapsed.TotalMilliseconds);
        BenchRows.Add(Group, name, "methods compiled by the JIT count", JitInfo.GetCompiledMethodCount());
        BenchRows.Add(Group, name, "result", volume);
        Console.WriteLine($"{name}: {clock.Elapsed.TotalMilliseconds:F0} ms, {JitInfo.GetCompiledMethodCount()} methods compiled by the JIT, volume {volume}");
    }

    private static bool IsKerf(AssemblyName name) => name.Name!.StartsWith("Kerf.", StringComparison.Ordinal);

    /// <summary>A Bento extension's context: what isn't Kerf comes from the host.</summary>
    private sealed class ExtensionContext(string folder, SharedContext? shared) : AssemblyLoadContext("extension", isCollectible: true)
    {
        public Assembly FromBytes(string path)
        {
            using var image = new MemoryStream(File.ReadAllBytes(path));
            return LoadFromStream(image);
        }

        protected override Assembly? Load(AssemblyName assemblyName)
            => !IsKerf(assemblyName) ? null : shared?.LoadFromAssemblyName(assemblyName) ?? FromBytes(Path.Combine(folder, assemblyName.Name + ".dll"));
    }

    /// <summary>The context extensions share Kerf in, which never unloads, so its ReadyToRun code is used.</summary>
    private sealed class SharedContext(string folder) : AssemblyLoadContext("shared")
    {
        protected override Assembly? Load(AssemblyName assemblyName)
            => IsKerf(assemblyName) ? LoadFromAssemblyPath(Path.Combine(folder, assemblyName.Name + ".dll")) : null;
    }
}

/// <summary>The work, kept in a class of its own as it runs in the extension's copy of this assembly.</summary>
public static class FirstLoadProbe
{
    public static string Work()
    {
        Part rounded = Operations.Fillet(new Box(10, 10, 10).Edges(), 1).Value;
        Part part = rounded - new Cylinder(2, 20);
        using var step = new MemoryStream();
        if (!Exporters3D.ExportStep(part, step).IsSuccess)
        {
            throw new InvalidOperationException("the STEP export failed");
        }

        return part.Volume.ToString("F6", CultureInfo.InvariantCulture);
    }
}
