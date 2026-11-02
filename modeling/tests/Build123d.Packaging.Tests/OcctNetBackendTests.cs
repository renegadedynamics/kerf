using System.Diagnostics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.Loader;

namespace Build123d.Packaging.Tests;

/// <summary>
/// A test that needs the OCCT.NET backend's build output: the folder of a project built with
/// <c>-p:Build123dBackend=OcctNet</c>, given as B3D_OCCTNET_LIB. Skipped without it.
/// </summary>
public sealed class OcctNetFactAttribute : FactAttribute
{
    public OcctNetFactAttribute()
    {
        if (Environment.GetEnvironmentVariable("B3D_OCCTNET_LIB") is not { Length: > 0 })
            Skip = "set B3D_OCCTNET_LIB to a build output of Build123dBackend=OcctNet";
    }
}

/// <summary>
/// The OCCT.NET backend as Bento's 3D-models extension would load it: Build123d, Build123d.OcctNet and OCCT.NET's
/// assemblies alone in a folder, loaded through a collectible <see cref="AssemblyLoadContext"/> with Schematiq.Collections
/// shared from the host. No native library may load.
/// </summary>
public class OcctNetBackendTests
{
    private sealed class ExtensionContext(string lib) : AssemblyLoadContext("Build123d extension (OCCT.NET)", isCollectible: true)
    {
        public List<string> NativeLoads { get; } = [];

        protected override Assembly? Load(AssemblyName name)
        {
            if (name.Name == "Schematiq.Collections")
                return null;   // the host's copy
            var path = Path.Combine(lib, name.Name + ".dll");
            return File.Exists(path) ? LoadFromAssemblyPath(path) : null;
        }

        protected override IntPtr LoadUnmanagedDll(string name)
        {
            NativeLoads.Add(name);
            throw new DllNotFoundException($"{name}: the OCCT.NET backend loads no native library");
        }
    }

    [OcctNetFact]
    public void ABoxTriangulatesWithoutNativeLibraries()
    {
        var lib = Path.Combine(Path.GetTempPath(), "b3d-occtnet-extension-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(lib);
        var source = Environment.GetEnvironmentVariable("B3D_OCCTNET_LIB")!;
        foreach (var name in new[] { "Build123d", "Build123d.OcctNet", "OcctNet.Foundation", "OcctNet.ModelingData", "OcctNet.ModelingAlgorithms",
                                  "OcctNet.DataExchange" })
            File.Copy(Path.Combine(source, name + ".dll"), Path.Combine(lib, name + ".dll"));
        Assert.DoesNotContain(Directory.GetFiles(lib, "*.dll"), f => Path.GetFileName(f) is "b3docct.dll" or "TKernel.dll");

        var (context, triangles, nativeLoads) = Run(lib);

        Assert.Equal(12, triangles);
        Assert.Empty(nativeLoads);
        Assert.DoesNotContain(Process.GetCurrentProcess().Modules.Cast<ProcessModule>(),
                              m => m.ModuleName.Equals("b3docct.dll", StringComparison.OrdinalIgnoreCase));
        for (int i = 0; context.IsAlive && i < 20; i++)
        {
            GC.Collect();
            GC.WaitForPendingFinalizers();
        }
        Assert.False(context.IsAlive, "the extension's load context didn't unload");
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static (WeakReference Context, int Triangles, List<string> NativeLoads) Run(string lib)
    {
        var context = new ExtensionContext(lib);
        var assembly = context.LoadFromAssemblyName(new AssemblyName("Build123d"));
        Assert.Same(context, AssemblyLoadContext.GetLoadContext(assembly));
        var triangles = Task.Run(() =>
        {
            const BindingFlags invoke = BindingFlags.Public | BindingFlags.Instance | BindingFlags.OptionalParamBinding;
            var boxType = assembly.GetType("Build123d.Box", throwOnError: true)!;
            var box = boxType.InvokeMember("", invoke | BindingFlags.CreateInstance, null, null, [80.0, 60.0, 10.0])!;
            var mesh = boxType.InvokeMember("Triangulate", invoke | BindingFlags.InvokeMethod, null, box, [0.1])!;
            return (int)mesh.GetType().GetProperty("TriangleCount")!.GetValue(mesh)!;
        }).GetAwaiter().GetResult();
        var loads = context.NativeLoads.ToList();
        context.Unload();
        return (new WeakReference(context), triangles, loads);
    }
}
