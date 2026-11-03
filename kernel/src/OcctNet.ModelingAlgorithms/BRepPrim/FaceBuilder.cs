// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_FaceBuilder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_FaceBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>
/// Builds a face from a surface: the whole surface, or the area between UMin, UMax, VMin, VMax.
/// </summary>
/// <remarks>Internal (decision 465): the face is unfrozen, for ported code that goes on building on it.</remarks>
internal sealed class FaceBuilder
{
    private readonly Vertex[] _vertex = GWedge.NewArray<Vertex>(4);
    private readonly Edge[] _edges = GWedge.NewArray<Edge>(4);
    private Face _face = new();

    public FaceBuilder()
    {
    }

    public FaceBuilder(BrepBuilder b, Surface s)
    {
        Init(b, s);
    }

    public FaceBuilder(BrepBuilder b, Surface s, double uMin, double uMax, double vMin, double vMax)
    {
        Init(b, s, uMin, uMax, vMin, vMax);
    }

    public void Init(BrepBuilder b, Surface s)
    {
        s.Bounds(out double uMin, out double uMax, out double vMin, out double vMax);
        Init(b, s, uMin, uMax, vMin, vMax);
    }

    /// <exception cref="ConstructionErrorException">The bounds are empty or out of the surface's.</exception>
    public void Init(BrepBuilder b, Surface s, double uMin, double uMax, double vMin, double vMax)
    {
        // Check the values
        s.Bounds(out double usMin, out double usMax, out double vsMin, out double vsMax);

        if (uMin >= uMax)
        {
            throw new ConstructionErrorException("BRepPrim_FaceBuilder");
        }
        if (vMin >= vMax)
        {
            throw new ConstructionErrorException("BRepPrim_FaceBuilder");
        }
        if (uMin < usMin)
        {
            throw new ConstructionErrorException("BRepPrim_FaceBuilder");
        }
        if (uMax > usMax)
        {
            throw new ConstructionErrorException("BRepPrim_FaceBuilder");
        }
        if (vMin < vsMin)
        {
            throw new ConstructionErrorException("BRepPrim_FaceBuilder");
        }
        if (vMax > vsMax)
        {
            throw new ConstructionErrorException("BRepPrim_FaceBuilder");
        }

        // Make the vertices
        b.MakeVertex(ref _vertex[0], s.Value(uMin, vMin), global::Kerf.Engine.Foundation.Precision.Confusion());
        b.MakeVertex(ref _vertex[1], s.Value(uMax, vMin), global::Kerf.Engine.Foundation.Precision.Confusion());
        b.MakeVertex(ref _vertex[2], s.Value(uMax, vMax), global::Kerf.Engine.Foundation.Precision.Confusion());
        b.MakeVertex(ref _vertex[3], s.Value(uMin, vMax), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Make the edges
        b.MakeEdge(ref _edges[0]);
        b.MakeEdge(ref _edges[1]);
        b.MakeEdge(ref _edges[2]);
        b.MakeEdge(ref _edges[3]);

        // Make the face
        b.MakeFace(ref _face, s, global::Kerf.Engine.Foundation.Precision.Confusion());

        // set the pcurves
        LineCurve2D l;
        l = new LineCurve2D(new Point2D(uMin, vMin), new Direction2D(Direction2D.D.X));
        b.UpdateEdge(_edges[0], l, _face, global::Kerf.Engine.Foundation.Precision.Confusion());
        l = new LineCurve2D(new Point2D(uMax, vMin), new Direction2D(Direction2D.D.Y));
        b.UpdateEdge(_edges[1], l, _face, global::Kerf.Engine.Foundation.Precision.Confusion());
        l = new LineCurve2D(new Point2D(uMax, vMax), new Direction2D(Direction2D.D.NX));
        b.UpdateEdge(_edges[2], l, _face, global::Kerf.Engine.Foundation.Precision.Confusion());
        l = new LineCurve2D(new Point2D(uMin, vMax), new Direction2D(Direction2D.D.NY));
        b.UpdateEdge(_edges[3], l, _face, global::Kerf.Engine.Foundation.Precision.Confusion());

        // set the parameters
        b.UpdateVertex(_vertex[0], 0, _edges[0], 0);
        b.UpdateVertex(_vertex[1], uMax - uMin, _edges[0], 0);
        b.UpdateVertex(_vertex[1], 0, _edges[1], 0);
        b.UpdateVertex(_vertex[2], vMax - vMin, _edges[1], 0);
        b.UpdateVertex(_vertex[2], 0, _edges[2], 0);
        b.UpdateVertex(_vertex[3], uMax - uMin, _edges[2], 0);
        b.UpdateVertex(_vertex[3], 0, _edges[3], 0);
        b.UpdateVertex(_vertex[0], vMax - vMin, _edges[3], 0);

        // insert vertices in edges
        _vertex[0].Orientation(Reversed);
        b.Add(_edges[3], _vertex[0]);
        _vertex[0].Orientation(Forward);
        b.Add(_edges[0], _vertex[0]);
        _vertex[1].Orientation(Reversed);
        b.Add(_edges[0], _vertex[1]);
        _vertex[1].Orientation(Forward);
        b.Add(_edges[1], _vertex[1]);
        _vertex[2].Orientation(Reversed);
        b.Add(_edges[1], _vertex[2]);
        _vertex[2].Orientation(Forward);
        b.Add(_edges[2], _vertex[2]);
        _vertex[3].Orientation(Reversed);
        b.Add(_edges[2], _vertex[3]);
        _vertex[3].Orientation(Forward);
        b.Add(_edges[3], _vertex[3]);

        // insert edges in a wire and the wire in the face
        Wire w = new();
        b.MakeWire(ref w);
        b.Add(w, _edges[0]);
        b.Add(w, _edges[1]);
        b.Add(w, _edges[2]);
        b.Add(w, _edges[3]);
        b.Add(_face, w);

        // set the natural restriction flag
        if (uMin == usMin && uMax == usMax && vMin == vsMin && vMax == vsMax)
        {
            b.NaturalRestriction(_face, true);
        }
    }

    public Face Face()
    {
        return _face;
    }

    /// <summary>The edge of index I: 1 VMin, 2 UMax, 3 VMax, 4 UMin.</summary>
    public Edge Edge(int i)
    {
        Standard_OutOfRange_Raise_if(i < 1 || i > 4, "BRepPrim_FaceBuilder::Edge");
        return _edges[i - 1];
    }

    /// <summary>The vertex of index I: 1 (UMin,VMin), 2 (UMax,VMin), 3 (UMax,VMax), 4 (UMin,VMax).</summary>
    public Vertex Vertex(int i)
    {
        Standard_OutOfRange_Raise_if(i < 1 || i > 4, "BRepPrim_FaceBuilder::Vertex");
        return _vertex[i - 1];
    }

    public static explicit operator Face(FaceBuilder builder) => builder.Face();
}
