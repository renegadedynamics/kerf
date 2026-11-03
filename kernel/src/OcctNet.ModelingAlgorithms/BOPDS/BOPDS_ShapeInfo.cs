// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_ShapeInfo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_ShapeInfo.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_ShapeInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// What the data structure knows about one shape: the shape, its type, its bounding box, the indices of its
/// sub-shapes, the index of its pave blocks or face information (the reference) and a flag.
/// </summary>
/// <remarks>
/// A class, a working object of the data structure (decision 631): OCCT's by-value class is held in the data
/// structure's vector and changed in place through <c>ChangeShapeInfo</c>. OCCT's by-value copies are the copy
/// constructor, which copies the list of sub-shapes.
/// </remarks>
public class BOPDS_ShapeInfo
{
    protected TopoDS_Shape myShape = new();
    protected TopAbs_ShapeEnum myType;
    protected Box myBox = new();
    protected NCollection_List<int> mySubShapes;
    protected int myReference;
    protected int myFlag;

    /// <summary>Empty information: type <see cref="TopAbs_SHAPE"/>, no reference, no flag.</summary>
    public BOPDS_ShapeInfo()
    {
        myType = TopAbs_SHAPE;
        mySubShapes = new NCollection_List<int>();
        myReference = -1;
        myFlag = -1;
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's implicit copy constructor).</summary>
    public BOPDS_ShapeInfo(BOPDS_ShapeInfo theOther)
    {
        myShape = theOther.myShape;
        myType = theOther.myType;
        myBox = theOther.myBox;
        mySubShapes = new NCollection_List<int>(theOther.mySubShapes);
        myReference = theOther.myReference;
        myFlag = theOther.myFlag;
    }

    internal void SetShape(in TopoDS_Shape theS) => myShape = theS;

    public TopoDS_Shape Shape() => myShape;

    internal void SetShapeType(TopAbs_ShapeEnum theType) => myType = theType;

    public TopAbs_ShapeEnum ShapeType() => myType;

    internal void SetBox(in Box theBox) => myBox = theBox;

    public Box Box() => myBox;

    internal ref Box ChangeBox() => ref myBox;

    /// <summary>The indices of the sub-shapes (the data structure's own list; read only for callers).</summary>
    public NCollection_List<int> SubShapes() => mySubShapes;

    internal NCollection_List<int> ChangeSubShapes() => mySubShapes;

    /// <summary>True if the shape has the sub-shape with the index.</summary>
    public bool HasSubShape(int theI) => mySubShapes.Contains(theI);

    public bool HasReference() => myReference >= 0;

    internal void SetReference(int theI) => myReference = theI;

    public int Reference() => myReference;

    /// <summary>True if the shape has a boundary representation.</summary>
    public bool HasBRep() => BOPDS_Tools.HasBRep(myType);

    /// <summary>True if the shape can take part in an interference.</summary>
    public bool IsInterfering() => HasBRep() || myType == TopAbs_SOLID;

    public bool HasFlag() => myFlag >= 0;

    public bool HasFlag(out int theFlag)
    {
        theFlag = myFlag;
        return myFlag >= 0;
    }

    internal void SetFlag(int theFlag) => myFlag = theFlag;

    public int Flag() => myFlag;

    /// <summary>Writes the type and the sub-shape indices as OCCT's <c>Dump</c> prints them.</summary>
    public void Dump(TextWriter theOS)
    {
        TopAbs_ShapeEnum aTS = ShapeType();
        theOS.Write(" " + TopAbs.ShapeTypeToString(aTS));
        theOS.Write(" {");
        for (NCollection_List<int>.Iterator aIt = new(mySubShapes); aIt.More(); aIt.Next())
        {
            int n = aIt.Value();
            theOS.Write(" " + n);
        }
        theOS.Write(" }");
    }
}
