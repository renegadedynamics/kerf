using System.Diagnostics;
using System.IO.Compression;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Loader;
using System.Text.RegularExpressions;

namespace Build123d.Packaging.Tests;

/// <summary>
/// The packages as Bento's 3D-models extension uses them: unpacked into a folder of their own, loaded through a
/// collectible <see cref="AssemblyLoadContext"/> that resolves managed and native libraries as Bento's does, with
/// Bento.Collections shared from the host.
/// </summary>
public class LoadContextTests
{
    private const string Rid = "win-x64";

    /// <summary>The extension's load context: Bento's resolution rules, nothing from the test's own folder.</summary>
    private sealed class ExtensionContext(string root) : AssemblyLoadContext("Build123d extension", isCollectible: true)
    {
        private readonly string _lib = Path.Combine(root, "lib", "net10.0");

        public List<string> NativeLoads { get; } = [];

        protected override Assembly? Load(AssemblyName name)
        {
            if (name.Name == "Bento.Collections")
                return null;   // the host's copy
            var path = Path.Combine(_lib, name.Name + ".dll");
            return File.Exists(path) ? LoadFromAssemblyPath(path) : null;
        }

        protected override IntPtr LoadUnmanagedDll(string name)
        {
            var file = name.EndsWith(".dll", StringComparison.OrdinalIgnoreCase) ? name : name + ".dll";
            foreach (var dir in new[] { _lib, Path.Combine(_lib, "runtimes", Rid, "native"), Path.Combine(root, "runtimes", Rid, "native") })
            {
                var path = Path.Combine(dir, file);
                if (File.Exists(path))
                {
                    NativeLoads.Add(path);
                    return LoadUnmanagedDllFromPath(path);
                }
            }
            throw new DllNotFoundException($"{file} isn't in the extension folder");
        }
    }

    [Fact]
    public void ABoxTessellatesThroughACollectibleLoadContext()
    {
        var root = Unpack();
        var (context, triangles, nativeLoads) = Run(root);

        Assert.Equal(12, triangles);
        Assert.Contains(nativeLoads, p => Path.GetFileName(p).Equals("b3docct.dll", StringComparison.OrdinalIgnoreCase));

        // OCCT came from the extension's folder, found beside b3docct
        var modules = Process.GetCurrentProcess().Modules.Cast<ProcessModule>()
            .Where(m => Regex.IsMatch(m.ModuleName, @"^(b3docct|TKernel|TKBRep|freetype)\.dll$", RegexOptions.IgnoreCase)).ToList();
        Assert.Equal(4, modules.Count);
        Assert.All(modules, m => Assert.StartsWith(root, m.FileName, StringComparison.OrdinalIgnoreCase));

        // And the extension unloads
        for (int i = 0; context.IsAlive && i < 20; i++)
        {
            GC.Collect();
            GC.WaitForPendingFinalizers();
        }
        Assert.False(context.IsAlive, "the extension's load context didn't unload");
    }

    /// <summary>Makes a box in the extension and tessellates it on a background thread, as Bento calculates cells.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (WeakReference Context, int Triangles, List<string> NativeLoads) Run(string root)
    {
        var context = new ExtensionContext(root);
        var assembly = context.LoadFromAssemblyName(new AssemblyName("Build123d"));
        Assert.Same(context, AssemblyLoadContext.GetLoadContext(assembly));
        var triangles = Task.Run(() =>
        {
            const BindingFlags invoke = BindingFlags.Public | BindingFlags.Instance | BindingFlags.OptionalParamBinding;
            var boxType = assembly.GetType("Build123d.Box", throwOnError: true)!;
            var box = boxType.InvokeMember("", invoke | BindingFlags.CreateInstance, null, null, [80.0, 60.0, 10.0])!;
            var mesh = boxType.InvokeMember("Tessellate", invoke | BindingFlags.InvokeMethod, null, box, [0.1])!;
            var list = (System.Collections.ICollection)mesh.GetType().GetField("Item2")!.GetValue(mesh)!;
            return list.Count;
        }).GetAwaiter().GetResult();
        var loads = context.NativeLoads.ToList();
        context.Unload();
        return (new WeakReference(context), triangles, loads);
    }

    /// <summary>The newest Build123d and Build123d.Occt packages unpacked into one extension folder.</summary>
    private static string Unpack()
    {
        Assert.True(RuntimeInformation.RuntimeIdentifier == Rid, $"the packages carry natives for {Rid} only");
        var feed = Environment.GetEnvironmentVariable("B3D_PACKAGES")
                   ?? Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "artifacts", "packages"));
        string Newest(string id)
        {
            var pattern = new Regex($@"^{Regex.Escape(id)}\.(\d+\.\d+\.\d+(-[0-9A-Za-z.-]+)?)\.nupkg$", RegexOptions.IgnoreCase);
            return Directory.Exists(feed)
                ? Directory.GetFiles(feed, "*.nupkg").Where(f => pattern.IsMatch(Path.GetFileName(f)))
                      .OrderBy(File.GetLastWriteTimeUtc).LastOrDefault() ?? throw new InvalidOperationException($"No {id} package in {feed}: run tools/pack.ps1")
                : throw new InvalidOperationException($"No package feed at {feed}: run tools/pack.ps1");
        }

        var root = Path.Combine(Path.GetTempPath(), "b3d-extension-" + Guid.NewGuid().ToString("N"));
        foreach (var package in new[] { Newest("Build123d"), Newest("Build123d.Occt") })
            using (var zip = ZipFile.OpenRead(package))
                foreach (var entry in zip.Entries.Where(e => e.FullName.StartsWith("lib/") || e.FullName.StartsWith("runtimes/")))
                {
                    var target = Path.Combine(root, entry.FullName.Replace('/', Path.DirectorySeparatorChar));
                    Directory.CreateDirectory(Path.GetDirectoryName(target)!);
                    entry.ExtractToFile(target, overwrite: true);
                }
        return root;
    }
}
