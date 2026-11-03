// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMeshFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMeshFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>
/// The factory of <see cref="BRepMesh_IncrementalMesh"/>, registered under the name "FastDiscret" as the default
/// (preferred) meshing algorithm.
/// </summary>
/// <remarks>
/// OCCT registers its instance when the library loads; the port's registers when
/// <see cref="BRepMesh_DiscretAlgoFactory"/> initialises (decision 492).
/// </remarks>
public sealed class BRepMesh_IncrementalMeshFactory : BRepMesh_DiscretAlgoFactory
{
    // Self-registering factory instance
    private static BRepMesh_IncrementalMeshFactory? THE_FACTORY_INSTANCE;

    // Registers this factory under the name "FastDiscret".
    private BRepMesh_IncrementalMeshFactory()
        : base("FastDiscret")
    {
        RegisterFactory(this, true);
    }

    // The FactoryInitializer: creates (and so registers) the instance once.
    internal static void Register()
    {
        THE_FACTORY_INSTANCE ??= new BRepMesh_IncrementalMeshFactory();
    }

    /// <summary>
    /// A new <see cref="BRepMesh_IncrementalMesh"/> for the shape with the deflections, in parallel if
    /// <see cref="BRepMesh_IncrementalMesh.IsParallelDefault"/>, not yet performed.
    /// </summary>
    public override BRepMesh_DiscretRoot CreateAlgorithm(in Shape theShape,
                                                         double theLinDeflection,
                                                         double theAngDeflection)
    {
        BRepMesh_IncrementalMesh anAlgo = new();
        anAlgo.ChangeParameters().Deflection = theLinDeflection;
        anAlgo.ChangeParameters().Angle = theAngDeflection;
        anAlgo.ChangeParameters().InParallel = BRepMesh_IncrementalMesh.IsParallelDefault();
        anAlgo.SetShape(theShape);
        return anAlgo;
    }
}
