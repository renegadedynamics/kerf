// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Edge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The description of an edge sent to the face classifier: the edge and its face (so that its pcurve can be found),
/// the next edge of its wire, and the intersector's settings.
/// </summary>
/// <remarks>
/// A value, as in OCCT (the classifiers copy it). <c>new()</c> is OCCT's default (an infinite maximum tolerance);
/// <c>default</c> has a zero one. The setters are internal, with <c>With</c> twins.
/// </remarks>
public partial struct BRepClass_Edge
{
    private TopoDS_Edge myEdge;
    private TopoDS_Face myFace;
    private TopoDS_Edge myNextEdge;
    private double myMaxTolerance;
    private bool myUseBndBox;

    /// <summary>An empty edge.</summary>
    public BRepClass_Edge()
    {
        myEdge = new TopoDS_Edge();
        myFace = new TopoDS_Face();
        myNextEdge = new TopoDS_Edge();
        myMaxTolerance = Precision.Infinite();
        myUseBndBox = false;
    }

    /// <summary>The edge <paramref name="E"/> of the face <paramref name="F"/>.</summary>
    public BRepClass_Edge(in TopoDS_Edge E, in TopoDS_Face F)
    {
        myEdge = E;
        myFace = F;
        myNextEdge = new TopoDS_Edge();
        myMaxTolerance = Precision.Infinite();
        myUseBndBox = false;
    }

    /// <summary>The current edge.</summary>
    public readonly TopoDS_Edge Edge() => myEdge;

    /// <summary>OCCT's non-const <c>Edge()</c>.</summary>
    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref TopoDS_Edge ChangeEdge() => ref myEdge;

    /// <summary>The face of the current edge.</summary>
    public readonly TopoDS_Face Face() => myFace;

    /// <summary>OCCT's non-const <c>Face()</c>.</summary>
    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref TopoDS_Face ChangeFace() => ref myFace;

    /// <summary>The next edge (null if none was found).</summary>
    public readonly TopoDS_Edge NextEdge() => myNextEdge;

    /// <summary>
    /// Finds and sets the next edge of the current one: the other edge at its last vertex, when that vertex has exactly
    /// two edges in <paramref name="theMapVE"/>. Leaves the next edge unchanged otherwise.
    /// </summary>
    internal void SetNextEdge(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapVE)
    {
        if (theMapVE.IsEmpty() || myEdge.IsNull())
        {
            return;
        }

        TopExp.Vertices(myEdge, out TopoDS_Vertex aVF, out TopoDS_Vertex aVL, true);

        if (aVL.IsNull() || aVL.IsSame(aVF))
        {
            return;
        }

        // OCCT dereferences the result of Seek without a check: the vertex is in the map of its own wire
        theMapVE.FindFromKey(aVL, out NCollection_List<TopoDS_Shape>? aListE);
        if (aListE!.Extent() == 2)
        {
            for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(aListE); anIt.More(); anIt.Next())
            {
                if ((!anIt.Value().IsNull()) && (!anIt.Value().IsSame(myEdge)))
                {
                    myNextEdge = TopoDS.Edge(anIt.Value());
                }
            }
        }
    }

    /// <summary>The maximum tolerance from which the intersector checks for a vertex with a high tolerance.</summary>
    public readonly double MaxTolerance() => myMaxTolerance;

    /// <summary>Sets the maximum tolerance.</summary>
    internal void SetMaxTolerance(double theValue) => myMaxTolerance = theValue;

    /// <summary>True if the intersector uses bounding boxes.</summary>
    public readonly bool UseBndBox() => myUseBndBox;

    /// <summary>Sets whether the intersector uses bounding boxes.</summary>
    internal void SetUseBndBox(bool theValue) => myUseBndBox = theValue;

    // Functional twins

    /// <summary>The edge with the next edge found in <paramref name="theMapVE"/> (see <see cref="SetNextEdge"/>).</summary>
    public readonly BRepClass_Edge WithNextEdge(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapVE)
    {
        BRepClass_Edge aCopy = this;
        aCopy.SetNextEdge(theMapVE);
        return aCopy;
    }

    /// <summary>The edge with another maximum tolerance.</summary>
    public readonly BRepClass_Edge WithMaxTolerance(double theValue)
    {
        BRepClass_Edge aCopy = this;
        aCopy.SetMaxTolerance(theValue);
        return aCopy;
    }

    /// <summary>The edge with bounding boxes used or not.</summary>
    public readonly BRepClass_Edge WithUseBndBox(bool theValue)
    {
        BRepClass_Edge aCopy = this;
        aCopy.SetUseBndBox(theValue);
        return aCopy;
    }

    /// <inheritdoc/>
    public override readonly string ToString() => $"BRepClass_Edge({myEdge}, {myFace})";
}
