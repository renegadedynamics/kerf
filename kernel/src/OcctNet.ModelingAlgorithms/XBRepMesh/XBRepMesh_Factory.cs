// Port of: src/ModelingAlgorithms/TKXMesh/XBRepMesh/XBRepMesh_Factory.hxx
// Port of: src/ModelingAlgorithms/TKXMesh/XBRepMesh/XBRepMesh_Factory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The factory of the "XBRepMesh" meshing algorithm (TKXMesh): a <see cref="BRepMesh_IncrementalMesh"/> by another
/// name, registered after "FastDiscret".
/// </summary>
/// <remarks>
/// OCCT registers its instance when TKXMesh loads; the port's registers when
/// <see cref="BRepMesh_DiscretAlgoFactory"/> initialises (decision 492). Unlike FastDiscret's, its algorithms don't
/// take <see cref="BRepMesh_IncrementalMesh.IsParallelDefault"/>, as in OCCT.
/// </remarks>
public sealed class XBRepMesh_Factory : BRepMesh_DiscretAlgoFactory
{
    // Self-registering factory instance
    private static XBRepMesh_Factory? THE_FACTORY_INSTANCE;

    // Registers this factory under the name "XBRepMesh".
    private XBRepMesh_Factory()
        : base("XBRepMesh")
    {
        RegisterFactory(this);
    }

    // The FactoryInitializer: creates (and so registers) the instance once.
    internal static void Register()
    {
        THE_FACTORY_INSTANCE ??= new XBRepMesh_Factory();
    }

    /// <summary>A new <see cref="BRepMesh_IncrementalMesh"/> for the shape with the deflections, not yet performed.</summary>
    public override BRepMesh_DiscretRoot CreateAlgorithm(in TopoDS_Shape theShape,
                                                         double theLinDeflection,
                                                         double theAngDeflection)
    {
        BRepMesh_IncrementalMesh anAlgo = new();
        anAlgo.ChangeParameters().Deflection = theLinDeflection;
        anAlgo.ChangeParameters().Angle = theAngDeflection;
        anAlgo.SetShape(theShape);
        return anAlgo;
    }
}
