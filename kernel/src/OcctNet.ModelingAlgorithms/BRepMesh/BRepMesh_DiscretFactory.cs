// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2008-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The global factory of triangulation algorithms: <see cref="Get"/> returns it, and <see cref="Discret"/> makes an
/// algorithm through the <see cref="BRepMesh_DiscretAlgoFactory"/> registry.
/// </summary>
/// <remarks>
/// The default name is process-wide state with an internal setter, as decision 415's: only Draw-lite's
/// <c>mpsetdefaultname</c> changes it (decision 492).
/// </remarks>
public sealed class BRepMesh_DiscretFactory
{
    private static readonly BRepMesh_DiscretFactory THE_GLOBAL_FACTORY = new();

    private volatile string myDefaultName;

    private BRepMesh_DiscretFactory()
    {
        myDefaultName = "FastDiscret";
    }

    /// <summary>The global factory.</summary>
    public static BRepMesh_DiscretFactory Get() => THE_GLOBAL_FACTORY;

    /// <summary>
    /// Sets the meshing algorithm by name; false (keeping the previous one) if no factory has the name. Internal
    /// (decision 492).
    /// </summary>
    internal bool SetDefaultName(string theName)
    {
        BRepMesh_DiscretAlgoFactory? aFactory = BRepMesh_DiscretAlgoFactory.FindFactory(theName);
        if (aFactory is null)
        {
            return false;
        }

        myDefaultName = theName;
        return true;
    }

    /// <summary>The name of the current meshing algorithm.</summary>
    public string DefaultName() => myDefaultName;

    /// <summary>
    /// A new triangulation algorithm for the shape with the deflections, not yet performed; null if no algorithm is
    /// available.
    /// </summary>
    public BRepMesh_DiscretRoot? Discret(in TopoDS_Shape theShape, double theLinDeflection, double theAngDeflection)
    {
        string aDefaultName = myDefaultName;
        BRepMesh_DiscretAlgoFactory? aFactory =
            aDefaultName.Length == 0 ? BRepMesh_DiscretAlgoFactory.DefaultFactory()
                                     : BRepMesh_DiscretAlgoFactory.FindFactory(aDefaultName);
        if (aFactory is null)
        {
            return null;
        }

        return aFactory.CreateAlgorithm(theShape, theLinDeflection, theAngDeflection);
    }
}
