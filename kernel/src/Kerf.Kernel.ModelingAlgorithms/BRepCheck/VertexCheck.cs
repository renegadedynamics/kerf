// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Vertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Vertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The checks of a vertex: in the context of an edge or a face, that the points its representations give are within
/// its tolerance of its point.
/// </summary>
internal class VertexCheck : ResultCheck
{
    /// <summary>Checks the vertex on its own (<see cref="Minimum"/>).</summary>
    public VertexCheck(in Vertex v)
    {
        Init(v);
    }

    /// <summary>Records <see cref="NoError"/>: a vertex on its own has nothing to check.</summary>
    public override void Minimum()
    {
        if (!_min)
        {
            // checks the existence of a point 3D
            var newList = new NCollection_List<StatusCheck>();
            NCollection_List<StatusCheck> lst = _map.SetValueRef(_shape, newList);
            lst.Append(StatusCheck.NoError);
            _min = true;
        }
    }

    /// <summary>
    /// Checks the vertex in an edge (its point on each curve representation of the edge) or a face (its point on the
    /// surface), within the larger of the two tolerances.
    /// </summary>
    public override void InContext(in Shape s)
    {
        NCollection_List<StatusCheck> hList;
        {
            bool locked = _isParallel;
            if (locked)
            {
                _mutex.Enter();
            }

            try
            {
                if (_map.ContainsKey(s))
                {
                    return;
                }

                var newList = new NCollection_List<StatusCheck>();
                hList = _map.SetValueRef(s, newList);
            }
            finally
            {
                if (locked)
                {
                    _mutex.Exit();
                }
            }
        }

        NCollection_List<StatusCheck> lst = hList;

        var exp = new Explorer(s, ShapeType.Vertex);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(_shape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheckCheck.Add(lst, StatusCheck.SubshapeNotInShape);
            return; // leaves
        }

        var tv = (BrepTVertex)_shape.TShape()!;
        Point3D prep = tv.Pnt();
        Point3D controlp;

        ShapeType styp = s.Type;
        switch (styp)
        {
            case ShapeType.Edge:
            {
                // Try to find the vertex on the edge
                Edge e = ShapeCasts.Edge(s);
                var itv = new Iterator(e.Oriented(Forward));
                Vertex vFind = new();
                bool multiple = false;
                while (itv.More())
                {
                    Vertex vf = ShapeCasts.Vertex(itv.Value());
                    if (itv.Value().IsSame(_shape))
                    {
                        if (vFind.IsNull)
                        {
                            vFind = vf;
                        }
                        else
                        {
                            if ((vFind.Orientation == Forward && vf.Orientation == Reversed)
                                || (vFind.Orientation == Reversed && vf.Orientation == Forward))
                            {
                                // the vertex on the edge is at once F and R
                                multiple = true;
                            }

                            if (vFind.Orientation != Forward && vFind.Orientation != Reversed)
                            {
                                if (vf.Orientation == Forward || vf.Orientation == Reversed)
                                {
                                    vFind = vf;
                                }
                            }
                        }
                    }

                    itv.Next();
                }

                // VFind is not null for sure
                Orientation orv = vFind.Orientation;

                double tol = Tool.Tolerance(ShapeCasts.Vertex(_shape));
                tol = Max(tol, Tool.Tolerance(e)); // to check
                tol *= tol;

                var te = (BrepTEdge)e.TShape()!;
                var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
                Placement eloc = e.Placement;

                var itpr = new NCollection_List<PointRepresentation>.Iterator();
                while (itcr.More())
                {
                    // For each CurveRepresentation, the provided parameter is checked
                    CurveRepresentation cr = itcr.Value();
                    Placement loc = cr.Location();
                    Placement l = (eloc * loc).Predivided(_shape.Placement);

                    if (cr.IsCurve3D)
                    {
                        Curve3D? c = cr.Curve3D();
                        if (c is not null) // edge non degenerated
                        {
                            itpr.Initialize(tv.Points());
                            while (itpr.More())
                            {
                                PointRepresentation pr = itpr.Value();
                                if (pr.IsPointOnCurve(c, l))
                                {
                                    controlp = c.Value(pr.Parameter());
                                    controlp = controlp.Transformed(l.Transformation);
                                    if (prep.SquareDistanceTo(controlp) > tol)
                                    {
                                        BRepCheckCheck.Add(lst, StatusCheck.InvalidPointOnCurve);
                                    }
                                }

                                itpr.Next();
                            }

                            if (orv == Forward || orv == Reversed)
                            {
                                var gc = (GCurve)cr;
                                if (orv == Forward || multiple)
                                {
                                    controlp = c.Value(gc.First());
                                    controlp = controlp.Transformed(l.Transformation);
                                    if (prep.SquareDistanceTo(controlp) > tol)
                                    {
                                        BRepCheckCheck.Add(lst, StatusCheck.InvalidPointOnCurve);
                                    }
                                }

                                if (orv == Reversed || multiple)
                                {
                                    controlp = c.Value(gc.Last());
                                    controlp = controlp.Transformed(l.Transformation);
                                    if (prep.SquareDistanceTo(controlp) > tol)
                                    {
                                        BRepCheckCheck.Add(lst, StatusCheck.InvalidPointOnCurve);
                                    }
                                }
                            }
                        }
                    }
                    else if (cr.IsCurveOnSurface())
                    {
                        Surface su = cr.Surface()!;
                        Curve2D pc = cr.PCurve()!;
                        Curve2D? pc2 = null;
                        if (cr.IsCurveOnClosedSurface)
                        {
                            pc2 = cr.PCurve2();
                        }

                        itpr.Initialize(tv.Points());
                        while (itpr.More())
                        {
                            PointRepresentation pr = itpr.Value();
                            if (pr.IsPointOnCurveOnSurface(pc, su, l))
                            {
                                Point2D p2d = pc.Value(pr.Parameter());
                                controlp = su.Value(p2d.X, p2d.Y);
                                controlp = controlp.Transformed(l.Transformation);
                                if (prep.SquareDistanceTo(controlp) > tol)
                                {
                                    BRepCheckCheck.Add(lst, StatusCheck.InvalidPointOnCurveOnSurface);
                                }
                            }

                            if (pc2 is not null && pr.IsPointOnCurveOnSurface(pc2, su, l))
                            {
                                Point2D p2d = pc2.Value(pr.Parameter());
                                controlp = su.Value(p2d.X, p2d.Y);
                                controlp = controlp.Transformed(l.Transformation);
                                if (prep.SquareDistanceTo(controlp) > tol)
                                {
                                    BRepCheckCheck.Add(lst, StatusCheck.InvalidPointOnCurveOnSurface);
                                }
                            }

                            itpr.Next();
                        }
                    }

                    itcr.Next();
                }

                if (lst.IsEmpty())
                {
                    lst.Append(StatusCheck.NoError);
                }

                break;
            }
            case ShapeType.Face:
            {
                var tf = (BrepTFace)s.TShape()!;
                Placement floc = s.Placement;
                Placement tFloc = tf.Location();
                Surface? su = tf.Surface();
                Placement l = (floc * tFloc).Predivided(_shape.Placement);

                double tol = Tool.Tolerance(ShapeCasts.Vertex(_shape));
                tol = Max(tol, Tool.Tolerance(ShapeCasts.Face(s))); // to check
                tol *= tol;

                var itpr = new NCollection_List<PointRepresentation>.Iterator(tv.Points());
                while (itpr.More())
                {
                    PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnSurface(su, l))
                    {
                        controlp = su!.Value(pr.Parameter(), pr.Parameter2());
                        controlp = controlp.Transformed(l.Transformation);
                        if (prep.SquareDistanceTo(controlp) > tol)
                        {
                            BRepCheckCheck.Add(lst, StatusCheck.InvalidPointOnSurface);
                        }
                    }

                    itpr.Next();
                }

                if (lst.IsEmpty())
                {
                    lst.Append(StatusCheck.NoError);
                }

                break;
            }
            default:
            {
                break;
            }
        }
    }

    /// <summary>Nothing more than the minimum (OCCT removed the body).</summary>
    public override void Blind()
    {
        if (_blind)
        {
            return;
        }

        _blind = true;
    }

    /// <summary>
    /// The tolerance the vertex needs: the largest distance from its point to the points its representations give
    /// (at least its tolerance), with a 5% margin.
    /// </summary>
    public double Tolerance()
    {
        // Check all the representations  of the vertex. (i-e checks the TVertex
        var tv = (BrepTVertex)_shape.TShape()!;
        Point3D prep = tv.Pnt();
        double tol = Tool.Tolerance(ShapeCasts.Vertex(_shape));
        tol *= tol;

        Point3D controlp;
        controlp = prep;
        var itpr = new NCollection_List<PointRepresentation>.Iterator(tv.Points());
        while (itpr.More())
        {
            PointRepresentation pr = itpr.Value();
            Placement loc = pr.Location();
            if (pr.IsPointOnCurve())
            {
                // OCCT: with a null curve, the previous point is kept and transformed again below
                if (pr.Curve() is not null)
                {
                    controlp = pr.Curve()!.Value(pr.Parameter());
                }
            }
            else if (pr.IsPointOnCurveOnSurface())
            {
                Point2D puv = pr.PCurve()!.Value(pr.Parameter());
                controlp = pr.Surface()!.Value(puv.X, puv.Y);
            }
            else if (pr.IsPointOnSurface())
            {
                controlp = pr.Surface()!.Value(pr.Parameter(), pr.Parameter2());
            }

            controlp = controlp.Transformed(loc.Transformation);
            if (prep.SquareDistanceTo(controlp) > tol)
            {
                tol = prep.SquareDistanceTo(controlp);
            }

            itpr.Next();
        }

        return Math.Sqrt(tol * 1.05);
    }
}
