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
    // Self-registering factory instance
    private static Factory? THE_FACTORY_INSTANCE;

    // Registers this factory under the name "XBRepMesh".
    private Factory()
        : base("XBRepMesh")
    {
        RegisterFactory(this);
    }

    // The FactoryInitializer: creates (and so registers) the instance once.
    internal static void Register()
    {
        THE_FACTORY_INSTANCE ??= new Factory();
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
