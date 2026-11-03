// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Edge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Topology;

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
    private Edge myEdge;
    private Face myFace;
    private Edge myNextEdge;
    private double myMaxTolerance;
    private bool myUseBndBox;

    /// <summary>An empty edge.</summary>
    public BRepClass_Edge()
    {
        myEdge = new Edge();
        myFace = new Face();
        myNextEdge = new Edge();
        myMaxTolerance = global::Kerf.Engine.Foundation.Precision.Infinite();
        myUseBndBox = false;
    }

    /// <summary>The edge <paramref name="E"/> of the face <paramref name="F"/>.</summary>
    public BRepClass_Edge(in Edge E, in Face F)
    {
        myEdge = E;
        myFace = F;
        myNextEdge = new Edge();
        myMaxTolerance = global::Kerf.Engine.Foundation.Precision.Infinite();
        myUseBndBox = false;
    }

    /// <summary>The current edge.</summary>
    public readonly Edge Edge() => myEdge;

    /// <summary>OCCT's non-const <c>Edge()</c>.</summary>
    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Edge ChangeEdge() => ref myEdge;

    /// <summary>The face of the current edge.</summary>
    public readonly Face Face() => myFace;

    /// <summary>OCCT's non-const <c>Face()</c>.</summary>
    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Face ChangeFace() => ref myFace;

    /// <summary>The next edge (null if none was found).</summary>
    public readonly Edge NextEdge() => myNextEdge;

    /// <summary>
    /// Finds and sets the next edge of the current one: the other edge at its last vertex, when that vertex has exactly
    /// two edges in <paramref name="theMapVE"/>. Leaves the next edge unchanged otherwise.
    /// </summary>
    internal void SetNextEdge(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapVE)
    {
        if (theMapVE.IsEmpty() || myEdge.IsNull)
        {
            return;
        }

        Exploration.Vertices(myEdge, out Vertex aVF, out Vertex aVL, true);

        if (aVL.IsNull || aVL.IsSame(aVF))
        {
            return;
        }

        // The vertex is in the map of its own wire (C++ dereferences the result of Seek unchecked)
        theMapVE.FindFromKey(aVL, out NCollection_List<Shape>? aListE);
        if (aListE!.Extent() == 2)
        {
            for (var anIt = new NCollection_List<Shape>.Iterator(aListE); anIt.More(); anIt.Next())
            {
                if ((!anIt.Value().IsNull) && (!anIt.Value().IsSame(myEdge)))
                {
                    myNextEdge = ShapeCasts.Edge(anIt.Value());
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
    public readonly BRepClass_Edge WithNextEdge(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapVE)
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
