// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.BRepLib_ShapeModification;

namespace Occt;

/// <summary>The root of the BRepLib makers that build a shape.</summary>
/// <remarks>
/// <see cref="Shape"/> and the makers' other result accessors are publication points (docs/phase3-design.md,
/// section 4.2): they freeze the result before returning it. Ported code that goes on building on the result reads
/// it through the internal accessors (<c>InternalShape</c>, <c>InternalEdge</c>, ...), which don't freeze.
/// </remarks>
public abstract class BRepLib_MakeShape : BRepLib_Command
{
    protected TopoDS_Shape myShape = new();
    protected NCollection_List<TopoDS_Shape> myGenFaces = new();
    protected NCollection_List<TopoDS_Shape> myNewFaces = new();
    protected NCollection_List<TopoDS_Shape> myEdgFaces = new();

    protected BRepLib_MakeShape()
    {
    }

    /// <summary>This is called by <see cref="Shape"/>. It does nothing but may be redefined.</summary>
    public void Build()
    {
    }

    /// <summary>The result (frozen). Builds it if the command isn't done, then checks that it is.</summary>
    /// <exception cref="StdFail_NotDone">The command is not done.</exception>
    public TopoDS_Shape Shape()
    {
        TopoDS_Shape aShape = InternalShape();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code that goes on building on the result.</summary>
    internal TopoDS_Shape InternalShape()
    {
        if (!IsDone())
        {
            Build();
            Check();
        }
        return myShape;
    }

    /// <summary>The result (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Shape(BRepLib_MakeShape theMaker) => theMaker.Shape();

    // Freezes the result and a sub-shape the maker hands out with it, and returns the sub-shape.
    private protected TopoDS_Shape Published(in TopoDS_Shape theShape)
    {
        myShape.TShape()?.FreezeGraph();
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    /// <summary>
    /// OCCT.NET: before a maker changes its result again (MakeWire, MakePolygon, MakeSolid and MakeFace's Add, Close),
    /// replaces a result that a public accessor has frozen by a new TShape with the same sub-shapes, location,
    /// orientation and flags (Closed, Orientable, Infinite, Convex), so that the published shape doesn't change. OCCT
    /// changes the one shape in place.
    /// </summary>
    private protected void UnfreezeResult()
    {
        if (myShape.IsNull() || !myShape.TShape()!.IsFrozen())
        {
            return;
        }
        TopoDS_Shape aNew = myShape.EmptyCopied();
        var B = new BRep_Builder();
        for (var anIt = new TopoDS_Iterator(myShape, false, false); anIt.More(); anIt.Next())
        {
            B.Add(aNew, anIt.Value());
        }
        aNew.Closed(myShape.Closed());
        aNew.Orientable(myShape.Orientable());
        aNew.Infinite(myShape.Infinite());
        aNew.Convex(myShape.Convex());
        myShape = aNew;
    }

    /// <summary>Returns the status of the face after the shape creation.</summary>
    public virtual BRepLib_ShapeModification FaceStatus(in TopoDS_Face F)
    {
        BRepLib_ShapeModification myStatus = BRepLib_Trimmed;
        return myStatus;
    }

    /// <summary>Returns true if the face generates new topology.</summary>
    public virtual bool HasDescendants(in TopoDS_Face F)
    {
        return true;
    }

    /// <summary>Returns the list of generated faces (a frozen copy of the maker's list).</summary>
    public ISchematiqList<TopoDS_Shape> DescendantFaces(in TopoDS_Face F) => publishedList(InternalDescendantFaces(F));

    /// <summary>The maker's own list behind <see cref="DescendantFaces"/>, unfrozen, for ported code.</summary>
    internal virtual NCollection_List<TopoDS_Shape> InternalDescendantFaces(in TopoDS_Face F)
    {
        return myGenFaces;
    }

    /// <summary>Returns the number of surfaces after the shape creation.</summary>
    public virtual int NbSurfaces()
    {
        return 0;
    }

    /// <summary>Returns the faces of the surface <paramref name="I"/> (a frozen copy of the maker's list).</summary>
    public ISchematiqList<TopoDS_Shape> NewFaces(int I) => publishedList(InternalNewFaces(I));

    /// <summary>The maker's own list behind <see cref="NewFaces"/>, unfrozen, for ported code.</summary>
    internal virtual NCollection_List<TopoDS_Shape> InternalNewFaces(int I)
    {
        return myNewFaces;
    }

    /// <summary>Returns the faces built from the edge <paramref name="E"/> (a frozen copy of the maker's list).</summary>
    public ISchematiqList<TopoDS_Shape> FacesFromEdges(in TopoDS_Edge E) => publishedList(InternalFacesFromEdges(E));

    /// <summary>The maker's own list behind <see cref="FacesFromEdges"/>, unfrozen, for ported code.</summary>
    internal virtual NCollection_List<TopoDS_Shape> InternalFacesFromEdges(in TopoDS_Edge E)
    {
        return myEdgFaces;
    }

    private static ISchematiqList<TopoDS_Shape> publishedList(NCollection_List<TopoDS_Shape> theList)
    {
        if (theList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        var aShapes = new TopoDS_Shape[theList.Extent()];
        int i = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aShapes[i] = anIt.Value();
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }
}
