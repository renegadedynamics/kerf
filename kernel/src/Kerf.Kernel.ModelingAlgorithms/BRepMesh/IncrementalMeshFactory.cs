// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMeshFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMeshFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The factory of <see cref="IncrementalMesh"/>, registered under the name "FastDiscret" as the default
/// (preferred) meshing algorithm.
/// </summary>
/// <remarks>
/// OCCT registers its instance when the library loads; the port's registers when
/// <see cref="DiscretAlgoFactory"/> initialises (decision 492).
/// </remarks>
internal sealed class IncrementalMeshFactory : DiscretAlgoFactory
{
    // The instance DiscretAlgoFactory's registry holds, made when it initialises (OCCT's self-registering instance)
    internal IncrementalMeshFactory()
        : base("FastDiscret")
    {
    }

    /// <summary>
    /// A new <see cref="IncrementalMesh"/> for the shape with the deflections, in parallel if
    /// <see cref="IncrementalMesh.IsParallelDefault"/>, not yet performed.
    /// </summary>
    public override DiscretRoot CreateAlgorithm(in Shape shape,
                                                         double linDeflection,
                                                         double angDeflection)
    {
        IncrementalMesh algo = new();
        algo.ChangeParameters().Deflection = linDeflection;
        algo.ChangeParameters().Angle = angDeflection;
        algo.ChangeParameters().InParallel = IncrementalMesh.IsParallelDefault;
        algo.SetShape(shape);
        return algo;
    }
}
