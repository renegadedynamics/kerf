// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2008-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The global factory of triangulation algorithms: <see cref="Get"/> returns it, and <see cref="Discret"/> makes an
/// algorithm through the <see cref="DiscretAlgoFactory"/> registry.
/// </summary>
/// <remarks>
/// The default name is a setting of the <see cref="OperationContext"/> (process-wide in OCCT; unit 10I) with an
/// internal setter, as decision 415's: only Draw-lite's <c>mpsetdefaultname</c> changes it, for the rest of its flow
/// of execution (decision 492). The factory itself holds nothing.
/// </remarks>
internal sealed class DiscretFactory
{
    private static readonly DiscretFactory THE_GLOBAL_FACTORY = new();

    // The name of the meshing algorithm (process-wide in OCCT): a setting of the operation context (unit 10I)
    internal static readonly ContextKey<string> DefaultNameSetting = new("BRepMesh_DiscretFactory.DefaultName", "FastDiscret");

    private DiscretFactory()
    {
    }

    /// <summary>The global factory.</summary>
    public static DiscretFactory Get() => THE_GLOBAL_FACTORY;

    /// <summary>
    /// Sets the meshing algorithm by name; false (keeping the previous one) if no factory has the name. Internal
    /// (decision 492).
    /// </summary>
    internal bool SetDefaultName(string name)
    {
        DiscretAlgoFactory? factory = DiscretAlgoFactory.FindFactory(name);
        if (factory is null)
        {
            return false;
        }

        OperationContext.Set(DefaultNameSetting, name);
        return true;
    }

    /// <summary>The name of the current meshing algorithm.</summary>
    public string DefaultName() => DefaultNameSetting.Value;

    /// <summary>
    /// A new triangulation algorithm for the shape with the deflections, not yet performed; null if no algorithm is
    /// available.
    /// </summary>
    public DiscretRoot? Discret(in Shape shape, double linDeflection, double angDeflection)
    {
        string defaultName = DefaultNameSetting.Value;
        DiscretAlgoFactory? factory =
            defaultName.Length == 0 ? DiscretAlgoFactory.DefaultFactory()
                                     : DiscretAlgoFactory.FindFactory(defaultName);
        if (factory is null)
        {
            return null;
        }

        return factory.CreateAlgorithm(shape, linDeflection, angDeflection);
    }
}
