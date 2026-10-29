// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Tool.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Indexes the sub-shapes of the generating shape of a sweep (TopExp::MapShapes).</summary>
internal sealed class BRepSweep_Tool
{
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);

    public BRepSweep_Tool(in TopoDS_Shape aShape)
    {
        TopExp.MapShapes(aShape, myMap);
    }

    public int NbShapes() => myMap.Extent();

    /// <summary>The index of the shape (0 if it isn't a sub-shape).</summary>
    public int Index(in TopoDS_Shape aShape)
    {
        if (!myMap.Contains(aShape))
        {
            return 0;
        }
        return myMap.FindIndex(aShape);
    }

    public TopoDS_Shape Shape(int anIndex) => myMap.FindKey(anIndex);

    public TopAbs_ShapeEnum Type(in TopoDS_Shape aShape) => aShape.ShapeType();

    public TopAbs_Orientation Orientation(in TopoDS_Shape aShape) => aShape.Orientation();

    public void SetOrientation(ref TopoDS_Shape aShape, TopAbs_Orientation Or) => aShape.Orientation(Or);
}
