// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Edge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Classification;

/// <summary>
/// The description of an edge sent to the face classifier: the edge and its face (so that its pcurve can be found),
/// the next edge of its wire, and the intersector's settings.
/// </summary>
/// <remarks>
/// A value, as in OCCT (the classifiers copy it). <c>new()</c> is OCCT's default (an infinite maximum tolerance);
/// <c>default</c> has a zero one. The setters are internal, with <c>With</c> twins.
/// </remarks>
public partial struct ClassifierEdge
{
    private Edge _edge;
    private Face _face;
    private Edge _nextEdge;
    private double _maxTolerance;
    private bool _useBndBox;

    /// <summary>An empty edge.</summary>
    public ClassifierEdge()
    {
        _edge = new Edge();
        _face = new Face();
        _nextEdge = new Edge();
        _maxTolerance = global::Kerf.Engine.Foundation.Precision.Infinite();
        _useBndBox = false;
    }

    /// <summary>The edge <paramref name="e"/> of the face <paramref name="f"/>.</summary>
    public ClassifierEdge(in Edge e, in Face f)
    {
        _edge = e;
        _face = f;
        _nextEdge = new Edge();
        _maxTolerance = global::Kerf.Engine.Foundation.Precision.Infinite();
        _useBndBox = false;
    }

    /// <summary>The current edge.</summary>
    public readonly Edge Edge() => _edge;

    /// <summary>OCCT's non-const <c>Edge()</c>.</summary>
    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Edge ChangeEdge() => ref _edge;

    /// <summary>The face of the current edge.</summary>
    public readonly Face Face() => _face;

    /// <summary>OCCT's non-const <c>Face()</c>.</summary>
    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Face ChangeFace() => ref _face;

    /// <summary>The next edge (null if none was found).</summary>
    public readonly Edge NextEdge() => _nextEdge;

    /// <summary>
    /// Finds and sets the next edge of the current one: the other edge at its last vertex, when that vertex has exactly
    /// two edges in <paramref name="mapVE"/>. Leaves the next edge unchanged otherwise.
    /// </summary>
    internal void SetNextEdge(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapVE)
    {
        if (mapVE.IsEmpty() || _edge.IsNull)
        {
            return;
        }

        Exploration.Vertices(_edge, out Vertex vf, out Vertex vl, true);

        if (vl.IsNull || vl.IsSame(vf))
        {
            return;
        }

        // The vertex is in the map of its own wire (C++ dereferences the result of Seek unchecked)
        mapVE.FindFromKey(vl, out NCollection_List<Shape>? listE);
        if (listE!.Extent() == 2)
        {
            for (var it = new NCollection_List<Shape>.Iterator(listE); it.More(); it.Next())
            {
                if ((!it.Value().IsNull) && (!it.Value().IsSame(_edge)))
                {
                    _nextEdge = ShapeCasts.Edge(it.Value());
                }
            }
        }
    }

    /// <summary>The maximum tolerance from which the intersector checks for a vertex with a high tolerance.</summary>
    public readonly double MaxTolerance() => _maxTolerance;

    /// <summary>Sets the maximum tolerance.</summary>
    internal void SetMaxTolerance(double value) => _maxTolerance = value;

    /// <summary>True if the intersector uses bounding boxes.</summary>
    public readonly bool UseBndBox() => _useBndBox;

    /// <summary>Sets whether the intersector uses bounding boxes.</summary>
    internal void SetUseBndBox(bool value) => _useBndBox = value;

    // Functional twins

    /// <summary>The edge with the next edge found in <paramref name="mapVE"/> (see <see cref="SetNextEdge"/>).</summary>
    public readonly ClassifierEdge WithNextEdge(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapVE)
    {
        ClassifierEdge copy = this;
        copy.SetNextEdge(mapVE);
        return copy;
    }

    /// <summary>The edge with another maximum tolerance.</summary>
    public readonly ClassifierEdge WithMaxTolerance(double value)
    {
        ClassifierEdge copy = this;
        copy.SetMaxTolerance(value);
        return copy;
    }

    /// <summary>The edge with bounding boxes used or not.</summary>
    public readonly ClassifierEdge WithUseBndBox(bool value)
    {
        ClassifierEdge copy = this;
        copy.SetUseBndBox(value);
        return copy;
    }

    /// <inheritdoc/>
    public override readonly string ToString() => $"BRepClass_Edge({_edge}, {_face})";
}
