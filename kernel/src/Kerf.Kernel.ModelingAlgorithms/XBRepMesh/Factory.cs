// Port of: src/ModelingAlgorithms/TKXMesh/XBRepMesh/XBRepMesh_Factory.hxx
// Port of: src/ModelingAlgorithms/TKXMesh/XBRepMesh/XBRepMesh_Factory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The factory of the "XBRepMesh" meshing algorithm (TKXMesh): a <see cref="IncrementalMesh"/> by another
/// name, registered after "FastDiscret".
/// </summary>
/// <remarks>
/// OCCT registers its instance when TKXMesh loads; the port's registers when
/// <see cref="DiscretAlgoFactory"/> initialises (decision 492). Unlike FastDiscret's, its algorithms don't
/// take <see cref="IncrementalMesh.IsParallelDefault"/>, as in OCCT.
/// </remarks>
public sealed class Factory : DiscretAlgoFactory
{
    // The instance DiscretAlgoFactory's registry holds, made when it initialises (OCCT's self-registering instance)
    internal Factory()
        : base("XBRepMesh")
    {
    }

    /// <summary>A new <see cref="IncrementalMesh"/> for the shape with the deflections, not yet performed.</summary>
    public override DiscretRoot CreateAlgorithm(in Shape shape,
                                                         double linDeflection,
                                                         double angDeflection)
    {
        IncrementalMesh algo = new();
        algo.ChangeParameters().Deflection = linDeflection;
        algo.ChangeParameters().Angle = angDeflection;
        algo.SetShape(shape);
        return algo;
    }
}
