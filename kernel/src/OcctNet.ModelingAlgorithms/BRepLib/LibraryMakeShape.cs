// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Construction.Library.LibraryShapeModification;

namespace Kerf.Engine.Construction.Library;

/// <summary>The root of the BRepLib makers that build a shape.</summary>
/// <remarks>
/// <see cref="Shape"/> and the makers' other result accessors are publication points (docs/phase3-design.md,
/// section 4.2): they freeze the result before returning it. Ported code that goes on building on the result reads
/// it through the internal accessors (<c>InternalShape</c>, <c>InternalEdge</c>, ...), which don't freeze.
/// </remarks>
public abstract class LibraryMakeShape : LibraryCommand
{
    protected Shape _shape = new();
    protected NCollection_List<Shape> _genFaces = new();
    protected NCollection_List<Shape> _newFaces = new();
    protected NCollection_List<Shape> _edgFaces = new();

    protected LibraryMakeShape()
    {
    }

    /// <summary>This is called by <see cref="Shape"/>. It does nothing but may be redefined.</summary>
    public void Build()
    {
    }

    /// <summary>The result (frozen). Builds it if the command isn't done, then checks that it is.</summary>
    /// <exception cref="NotDoneException">The command is not done.</exception>
    public Shape Shape()
    {
        Shape shape = InternalShape();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code that goes on building on the result.</summary>
    internal Shape InternalShape()
    {
        if (!IsDone)
        {
            Build();
            Check();
        }
        return _shape;
    }

    /// <summary>The result (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shape(LibraryMakeShape maker) => maker.Shape();

    // Freezes the result and a sub-shape the maker hands out with it, and returns the sub-shape.
    private protected Shape Published(in Shape shape)
    {
        _shape.TShape()?.FreezeGraph();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>
    /// OCCT.NET: before a maker changes its result again (MakeWire, MakePolygon, MakeSolid and MakeFace's Add, Close),
    /// replaces a result that a public accessor has frozen by a new TShape with the same sub-shapes, location,
    /// orientation and flags (Closed, Orientable, Infinite, Convex), so that the published shape doesn't change. OCCT
    /// changes the one shape in place.
    /// </summary>
    private protected void UnfreezeResult()
    {
        if (_shape.IsNull || !_shape.TShape()!.IsFrozen)
        {
            return;
        }
        Shape aNew = _shape.EmptyCopied();
        var b = new BrepBuilder();
        for (var it = new Iterator(_shape, false, false); it.More(); it.Next())
        {
            b.Add(aNew, it.Value());
        }
        aNew.Closed(_shape.Closed());
        aNew.Orientable(_shape.Orientable());
        aNew.Infinite(_shape.Infinite());
        aNew.Convex(_shape.Convex());
        _shape = aNew;
    }

    /// <summary>Returns the status of the face after the shape creation.</summary>
    public virtual LibraryShapeModification FaceStatus(in Face f)
    {
        LibraryShapeModification myStatus = LibraryShapeModification.Trimmed;
        return myStatus;
    }

    /// <summary>Returns true if the face generates new topology.</summary>
    public virtual bool HasDescendants(in Face f)
    {
        return true;
    }

    /// <summary>Returns the list of generated faces (a frozen copy of the maker's list).</summary>
    public ISchematiqList<Shape> DescendantFaces(in Face f) => PublishedList(InternalDescendantFaces(f));

    /// <summary>The maker's own list behind <see cref="DescendantFaces"/>, unfrozen, for ported code.</summary>
    internal virtual NCollection_List<Shape> InternalDescendantFaces(in Face f)
    {
        return _genFaces;
    }

    /// <summary>Returns the number of surfaces after the shape creation.</summary>
    public virtual int SurfaceCount
    {
        get
        {
            return 0;
        }
    }

    /// <summary>Returns the faces of the surface <paramref name="i"/> (a frozen copy of the maker's list).</summary>
    public ISchematiqList<Shape> NewFaces(int i) => PublishedList(InternalNewFaces(i));

    /// <summary>The maker's own list behind <see cref="NewFaces"/>, unfrozen, for ported code.</summary>
    internal virtual NCollection_List<Shape> InternalNewFaces(int i)
    {
        return _newFaces;
    }

    /// <summary>Returns the faces built from the edge <paramref name="e"/> (a frozen copy of the maker's list).</summary>
    public ISchematiqList<Shape> FacesFromEdges(in Edge e) => PublishedList(InternalFacesFromEdges(e));

    /// <summary>The maker's own list behind <see cref="FacesFromEdges"/>, unfrozen, for ported code.</summary>
    internal virtual NCollection_List<Shape> InternalFacesFromEdges(in Edge e)
    {
        return _edgFaces;
    }

    private static ISchematiqList<Shape> PublishedList(NCollection_List<Shape> list)
    {
        if (list.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        var shapes = new Shape[list.Extent()];
        int i = 0;
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            shapes[i] = it.Value();
            shapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(shapes);
    }
}
