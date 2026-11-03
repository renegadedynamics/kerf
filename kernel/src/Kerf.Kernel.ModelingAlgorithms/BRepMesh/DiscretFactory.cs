// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2008-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The global factory of triangulation algorithms: <see cref="Get"/> returns it, and <see cref="Discret"/> makes an
/// algorithm through the <see cref="DiscretAlgoFactory"/> registry.
/// </summary>
/// <remarks>
/// The default name is process-wide state with an internal setter, as decision 415's: only Draw-lite's
/// <c>mpsetdefaultname</c> changes it (decision 492).
/// </remarks>
public sealed class DiscretFactory
{
    private static readonly DiscretFactory THE_GLOBAL_FACTORY = new();

    private volatile string _defaultName;

    private DiscretFactory()
    {
        _defaultName = "FastDiscret";
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

        _defaultName = name;
        return true;
    }

    /// <summary>The name of the current meshing algorithm.</summary>
    public string DefaultName() => _defaultName;

    /// <summary>
    /// A new triangulation algorithm for the shape with the deflections, not yet performed; null if no algorithm is
    /// available.
    /// </summary>
    public DiscretRoot? Discret(in Shape shape, double linDeflection, double angDeflection)
    {
        string defaultName = _defaultName;
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
