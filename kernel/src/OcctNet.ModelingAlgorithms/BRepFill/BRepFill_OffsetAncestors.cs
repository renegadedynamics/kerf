// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetAncestors.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetAncestors.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The ancestors (spine edges and vertices) of the edges of a parallel built by BRepFill_OffsetWire.</summary>
/// <remarks>A working object (decision 21); it reads the offset's internal (unfrozen) forms.</remarks>
public sealed class BRepFill_OffsetAncestors
{
    private bool myIsPerform;
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>An empty tool.</summary>
    public BRepFill_OffsetAncestors()
    {
        myIsPerform = false;
    }

    /// <summary>The ancestors of the parallel of <paramref name="Paral"/>.</summary>
    public BRepFill_OffsetAncestors(BRepFill_OffsetWire Paral)
    {
        Perform(Paral);
    }

    /// <summary>Computes the ancestors of the parallel of <paramref name="Paral"/>.</summary>
    public void Perform(BRepFill_OffsetWire Paral)
    {
        TopoDS_Face Spine = Paral.InternalSpine();

        TopExp_Explorer Exp = new();
        NCollection_List<TopoDS_Shape>.Iterator it = new();

        // on itere sur les edges.
        for (Exp.Init(Spine, TopAbs_EDGE); Exp.More(); Exp.Next())
        {
            for (it.Initialize(Paral.InternalGeneratedShapes(Exp.Current())); it.More(); it.Next())
            {
                myMap.Bind(it.Value(), Exp.Current());
            }
        }

        // on itere sur les vertex.
        for (Exp.Init(Spine, TopAbs_VERTEX); Exp.More(); Exp.Next())
        {
            for (it.Initialize(Paral.InternalGeneratedShapes(Exp.Current())); it.More(); it.Next())
            {
                myMap.Bind(it.Value(), Exp.Current());
            }
        }

        myIsPerform = true;
    }

    /// <summary>True once <see cref="Perform"/> has run.</summary>
    public bool IsDone() => myIsPerform;

    /// <summary>True if the edge <paramref name="S1"/> of the parallel has an ancestor.</summary>
    public bool HasAncestor(in TopoDS_Edge S1) => myMap.IsBound(S1);

    /// <summary>The ancestor of the edge <paramref name="S1"/> of the parallel.</summary>
    /// <exception cref="StdFail_NotDone"><see cref="Perform"/> hasn't run.</exception>
    public TopoDS_Shape Ancestor(in TopoDS_Edge S1)
    {
        StdFail_NotDone_Raise_if(!myIsPerform,
            "BRepFill_OffsetAncestors::Ancestor() - Perform() should be called before accessing results");
        return myMap[S1];
    }
}
