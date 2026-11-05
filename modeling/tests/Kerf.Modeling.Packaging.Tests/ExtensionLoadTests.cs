using System.Diagnostics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.Loader;

namespace Kerf.Modeling.Packaging.Tests;

/// <summary>
/// A test that needs Kerf.Modeling's build output: the folder of a project that references it (Kerf.Modeling's own, or
/// its tests'), given as KERF_MODELING_LIB. Skipped without it.
/// </summary>
public sealed class ModelingBuildFactAttribute : FactAttribute
{
    public ModelingBuildFactAttribute()
    {
        if (Environment.GetEnvironmentVariable("KERF_MODELING_LIB") is not { Length: > 0 })
            Skip = "set KERF_MODELING_LIB to a build output of Kerf.Modeling";
    }
}

/// <summary>
/// Kerf.Modeling as Bento's 3D-models extension would load it: Kerf.Modeling and the kernel's assemblies alone in a folder,
/// loaded through a collectible <see cref="AssemblyLoadContext"/>. No
/// native library may load.
/// </summary>
public class ExtensionLoadTests
{
    private sealed class ExtensionContext(string lib) : AssemblyLoadContext("Kerf.Modeling extension", isCollectible: true)
    {
        public List<string> NativeLoads { get; } = [];

        protected override Assembly? Load(AssemblyName name)
        {
            var path = Path.Combine(lib, name.Name + ".dll");
            return File.Exists(path) ? LoadFromAssemblyPath(path) : null;
        }

        protected override IntPtr LoadUnmanagedDll(string name)
        {
            NativeLoads.Add(name);
            throw new DllNotFoundException($"{name}: Kerf.Modeling loads no native library");
        }
    }

    [ModelingBuildFact]
    public void ABoxTriangulatesWithoutNativeLibraries()
    {
        var lib = Path.Combine(Path.GetTempPath(), "kerf-modeling-extension-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(lib);
        var source = Environment.GetEnvironmentVariable("KERF_MODELING_LIB")!;
        foreach (var name in new[] { "Kerf.Modeling", "Kerf.Kernel.Foundation", "Kerf.Kernel.ModelingData", "Kerf.Kernel.ModelingAlgorithms",
                                  "Kerf.Kernel.DataExchange" })
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
        var assembly = context.LoadFromAssemblyName(new AssemblyName("Kerf.Modeling"));
        Assert.Same(context, AssemblyLoadContext.GetLoadContext(assembly));
        var triangles = Task.Run(() =>
        {
            const BindingFlags invoke = BindingFlags.Public | BindingFlags.Instance | BindingFlags.OptionalParamBinding;
            var boxType = assembly.GetType("Kerf.Modeling.Box", throwOnError: true)!;
            var box = boxType.InvokeMember("", invoke | BindingFlags.CreateInstance, null, null, [80.0, 60.0, 10.0])!;
            // An extension member: a static method of the extensions' class, generic in the receiver
            var triangulate = assembly.GetType("Kerf.Modeling.ShapeExtensions", throwOnError: true)!.GetMethods()
                .First(m => m.Name == "Triangulate" && m.GetParameters().Length == 4).MakeGenericMethod(boxType);
            var mesh = triangulate.Invoke(null, [box, 0.1, 0.2, false])!;
            return (int)mesh.GetType().GetProperty("TriangleCount")!.GetValue(mesh)!;
        }).GetAwaiter().GetResult();
        var loads = context.NativeLoads.ToList();
        context.Unload();
        return (new WeakReference(context), triangles, loads);
    }
}
