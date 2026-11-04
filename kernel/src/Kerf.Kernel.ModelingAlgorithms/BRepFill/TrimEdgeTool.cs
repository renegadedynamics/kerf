// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimEdgeTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimEdgeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Geometric tool of the 2D offset (BRepFill_OffsetWire): the intersections of a bisector of the medial axis with the
/// parallels of its two generating elements.
/// </summary>
/// <remarks>
/// A working object (decision 21). It reads the bisector it is given and never changes it: OCCT keeps a copy of the
/// Bisector_Bisec sharing the same curve, the port the Bisector_Bisec itself (decision 822).
/// </remarks>
internal sealed class TrimEdgeTool
{
    private readonly bool _isPoint1;
    private readonly bool _isPoint2;
    private readonly Point2D _p1;
    private readonly Point2D _p2;
    private readonly Curve2D? _c1;
    private readonly Curve2D? _c2;
    private readonly double _offset;
    private readonly Bisec? _bisec;
    private readonly CurveAdaptor2D _bis = new();

    // the simple expression of the bisector: a BisecAna is replaced by its conic, trimmed as the bisector
    private static void SimpleExpression(Bisec b, out Curve2D bis)
    {
        bis = b.Value();

        Type bt = bis.GetType();
        if (bt == typeof(TrimmedCurve2D))
        {
            TrimmedCurve2D trBis = (TrimmedCurve2D)bis;
            // OCCT.NET: ChangeBasisCurve, so the live bisector of the locus isn't frozen (decision 816)
            Curve2D basBis = trBis.ChangeBasisCurve();
            bt = basBis.GetType();
            if (bt == typeof(BisecAna))
            {
                bis = ((BisecAna)basBis).Geom2dCurve();
                bis = new TrimmedCurve2D(bis, trBis.FirstParameter, trBis.LastParameter);
            }
        }
    }

    /// <summary>An empty tool.</summary>
    public TrimEdgeTool()
    {
    }

    /// <summary>
    /// The tool of the bisector <paramref name="bisec"/> of the elements <paramref name="s1"/> and <paramref name="s2"/>
    /// (2D points or curves) for the offset <paramref name="offset"/>.
    /// </summary>
    public TrimEdgeTool(Bisec bisec, Geometry2D s1, Geometry2D s2, double offset)
    {
        _offset = offset;
        _bisec = bisec;
        _isPoint1 = s1.GetType() == typeof(CartesianPoint2);
        _isPoint2 = s2.GetType() == typeof(CartesianPoint2);

        // return geometries of shapes.
        if (_isPoint1)
        {
            _p1 = ((Point2)s1).Pnt2d();
        }
        else
        {
            _c1 = s1 as Curve2D;
        }
        if (_isPoint2)
        {
            _p2 = ((Point2)s2).Pnt2d();
        }
        else
        {
            _c2 = s2 as Curve2D;
        }
        // return the simple expression of the bissectrice
        SimpleExpression(_bisec, out Curve2D bis);
        _bis = new CurveAdaptor2D(bis);
    }

    // Order the sequence of points by increasing x.
    private static void Bubble(NCollection_Sequence<Point3D> seq)
    {
        bool invert = true;
        int nbPoints = seq.Length();
        while (invert)
        {
            invert = false;
            for (int i = 1; i < nbPoints; i++)
            {
                Point3D p1 = seq.Value(i);
                Point3D p2 = seq.Value(i + 1);
                if (p2.X < p1.X)
                {
                    seq.Exchange(i, i + 1);
                    invert = true;
                }
            }
        }
    }

    private static void EvalParameters(CurveAdaptor2D bis, CurveAdaptor2D ac, NCollection_Sequence<Point3D> Params)
    {
        GInter intersector;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        CurveAdaptor2D cBis = bis;
        CurveAdaptor2D cac = ac;

        intersector = new GInter(cac, cBis, tol, tol);

        int nbPoints, nbSegments;
        double u1, u2;
        Point3D p;

        if (!intersector.IsDone)
        {
            throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
        }

        nbPoints = intersector.PointCount;

        if (nbPoints > 0)
        {
            for (int i = 1; i <= nbPoints; i++)
            {
                u1 = intersector.Point(i).ParamOnSecond();
                u2 = intersector.Point(i).ParamOnFirst();
                p = new Point3D(u1, u2, 0.0);
                Params.Append(p);
            }
        }

        nbSegments = intersector.SegmentCount;

        if (nbSegments > 0)
        {
            IntersectionSegment2D seg;
            for (int i = 1; i <= nbSegments; i++)
            {
                seg = intersector.Segment(i);
                u1 = seg.FirstPoint().ParamOnSecond();
                double ulast = seg.LastPoint().ParamOnSecond();
                if (Math.Abs(u1 - cBis.FirstParameter()) <= tol
                    && Math.Abs(ulast - cBis.LastParameter()) <= tol)
                {
                    p = new Point3D(u1, seg.FirstPoint().ParamOnFirst(), 0.0);
                    Params.Append(p);
                    p = new Point3D(ulast, seg.LastPoint().ParamOnFirst(), 0.0);
                    Params.Append(p);
                }
                else
                {
                    u1 += seg.LastPoint().ParamOnSecond();
                    u1 /= 2.0;
                    u2 = seg.FirstPoint().ParamOnFirst();
                    u2 += seg.LastPoint().ParamOnFirst();
                    u2 /= 2.0;
                    p = new Point3D(u1, u2, 0.0);
                    Params.Append(p);
                }
            }
        }

        // Order the sequence by growing parameter on the bissectrice.
        Bubble(Params);
    }

    private static void EvalParametersBis(CurveAdaptor2D bis, CurveAdaptor2D ac, NCollection_Sequence<Point3D> Params,
                                          double tol)
    {
        GInter intersector;
        double tolC = tol;

        CurveAdaptor2D cBis = bis;
        CurveAdaptor2D cac = ac;

        intersector = new GInter(cac, cBis, tolC, tol);

        int nbPoints, nbSegments;
        double u1, u2;
        Point3D p;

        if (!intersector.IsDone)
        {
            throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
        }

        nbPoints = intersector.PointCount;

        if (nbPoints > 0)
        {
            for (int i = 1; i <= nbPoints; i++)
            {
                u1 = intersector.Point(i).ParamOnSecond();
                u2 = intersector.Point(i).ParamOnFirst();
                p = new Point3D(u1, u2, 0.0);
                Params.Append(p);
            }
        }

        nbSegments = intersector.SegmentCount;

        if (nbSegments > 0)
        {
            IntersectionSegment2D seg;
            for (int i = 1; i <= nbSegments; i++)
            {
                seg = intersector.Segment(i);
                u1 = seg.FirstPoint().ParamOnSecond();
                double ulast = seg.LastPoint().ParamOnSecond();
                if (Math.Abs(u1 - cBis.FirstParameter()) <= tol
                    && Math.Abs(ulast - cBis.LastParameter()) <= tol)
                {
                    p = new Point3D(u1, seg.FirstPoint().ParamOnFirst(), 0.0);
                    Params.Append(p);
                    p = new Point3D(ulast, seg.LastPoint().ParamOnFirst(), 0.0);
                    Params.Append(p);
                }
                else
                {
                    u1 += seg.LastPoint().ParamOnSecond();
                    u1 /= 2.0;
                    u2 = seg.FirstPoint().ParamOnFirst();
                    u2 += seg.LastPoint().ParamOnFirst();
                    u2 /= 2.0;
                    p = new Point3D(u1, u2, 0.0);
                    Params.Append(p);
                }
            }
        }

        // Order the sequence by parameter growing on the bissectrice.
        Bubble(Params);
    }

    /// <summary>
    /// The intersections of the bisector with the parallels <paramref name="edge1"/> and <paramref name="edge2"/>
    /// (generated by <paramref name="initShape1"/> and <paramref name="initShape2"/>): for each point, X is the
    /// parameter on the bisector, Y on the first parallel and Z on the second.
    /// </summary>
    public void IntersectWith(in Edge edge1,
                              in Edge edge2,
                              in Shape initShape1,
                              in Shape initShape2,
                              in Vertex end1,
                              in Vertex end2,
                              JoinKind joinType,
                              bool isOpenResult,
                              NCollection_Sequence<Point3D> Params)
    {
        Params.Clear();

        // return curves associated to edges.
        double f, l;

        Tool.CurveOnSurface(edge1, out Curve2D? c1, out Surface? _, out Placement _, out f, out l);
        CurveAdaptor2D ac1 = new(c1!, f, l);

        Tool.CurveOnSurface(edge2, out Curve2D? c2, out Surface? _, out Placement _, out f, out l);
        CurveAdaptor2D ac2 = new(c2!, f, l);

        // Calculate intersection
        NCollection_Sequence<Point3D> points2 = new();
        Point3D pSeq;

        EvalParameters(_bis, ac1, Params);
        EvalParameters(_bis, ac2, points2);

        int seanceDeRattrapage = 0;
        double tolInit = 1.0e-9;
        int nn = 7;

        if ((ac1.GetType() != Circle && ac1.GetType() != Line)
            || (ac2.GetType() != Circle && ac2.GetType() != Line))
        {
            tolInit = 1.0e-8;
            nn = 6;
        }

        if (Params.IsEmpty() && points2.IsEmpty())
        {
            // Check, may be there are no intersections at all
            //  for case myBis == Line
            if (_bis.GetType() == Line)
            {
                double dmax = tolInit;
                int n = 0;
                while (n < nn)
                {
                    dmax *= 10.0;
                    ++n;
                }
                dmax *= dmax;
                //
                Line2D anL = _bis.Line();
                bool isFar1;
                bool isFar2;
                Point2D aP;
                //
                double d = RealLast();
                ac1.D0(ac1.FirstParameter(), out aP);
                double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= _bis.FirstParameter() && par <= _bis.LastParameter())
                {
                    d = anL.SquareDistanceTo(aP);
                }
                ac1.D0(ac1.LastParameter(), out aP);
                par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= _bis.FirstParameter() && par <= _bis.LastParameter())
                {
                    d = Min(anL.SquareDistanceTo(aP), d);
                }
                isFar1 = d > dmax;
                //
                d = RealLast();
                ac2.D0(ac2.FirstParameter(), out aP);
                par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= _bis.FirstParameter() && par <= _bis.LastParameter())
                {
                    d = anL.SquareDistanceTo(aP);
                }
                ac2.D0(ac2.LastParameter(), out aP);
                par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= _bis.FirstParameter() && par <= _bis.LastParameter())
                {
                    d = Min(anL.SquareDistanceTo(aP), d);
                }
                isFar2 = d > dmax;
                //
                if (isFar1 && isFar2)
                {
                    return;
                }
            }
        }

        while (seanceDeRattrapage < nn // TolInit <= 0.01
               && (points2.Length() != Params.Length() || (points2.Length() == 0 && Params.Length() == 0)))
        {
            Params.Clear();
            points2.Clear();

            tolInit *= 10.0;

            EvalParametersBis(_bis, ac1, Params, tolInit);
            EvalParametersBis(_bis, ac2, points2, tolInit);
            seanceDeRattrapage++;
        }

        if (Params.Length() == 0 && points2.Length() == 1)
        {
            double dmin, dmax = 0.25 * _offset * _offset;
            double tBis = points2[1].X;
            Point2D pBis = _bis.Value(tBis);

            double t = ac1.FirstParameter();
            Point2D pc = ac1.Value(t);
            dmin = pc.SquareDistanceTo(pBis);
            Point3D p = new(tBis, t, 0.0);
            if (dmin < dmax)
            {
                Params.Append(p);
            }

            t = ac1.LastParameter();
            pc = ac1.Value(t);
            double dmin1 = pc.SquareDistanceTo(pBis);
            if (dmin > dmin1 && dmin1 < dmax)
            {
                p = p.WithY(t);
                if (Params.IsEmpty())
                {
                    Params.Append(p);
                }
                else
                {
                    Params.SetValue(1, p);
                }
            }
        }
        else if (Params.Length() == 1 && points2.Length() == 0)
        {
            double dmin, dmax = 0.25 * _offset * _offset;
            double tBis = Params[1].X;
            Point2D pBis = _bis.Value(tBis);

            double t = ac2.FirstParameter();
            Point2D pc = ac2.Value(t);
            dmin = pc.SquareDistanceTo(pBis);
            Point3D p = new(tBis, t, 0.0);
            if (dmin < dmax)
            {
                points2.Append(p);
            }

            t = ac2.LastParameter();
            pc = ac2.Value(t);
            double dmin1 = pc.SquareDistanceTo(pBis);
            if (dmin > dmin1 && dmin1 < dmax)
            {
                p = p.WithY(t);
                if (points2.IsEmpty())
                {
                    points2.Append(p);
                }
                else
                {
                    points2.SetValue(1, p);
                }
            }
        }

        // small manipulation to remove incorrect intersections:
        // return only common intersections (same parameter
        // on the bissectrice.).
        // The tolerance can be eventually changed.

        Point3D p1, p2;
        double tol = 4 * 100 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        int i;
        int nbPoints = Params.Length();

        if (nbPoints == 1 && points2.Length() == 1)
        {
            for (i = 1; i <= nbPoints; i++)
            {
                pSeq = Params[i];
                pSeq = pSeq.WithZ(points2.Value(i).Y);
                Params.SetValue(i, pSeq);
            }
            return;
        }

        i = 1;
        while (i <= Min(Params.Length(), points2.Length()))
        {
            p1 = Params[i];
            p2 = points2[i];
            double p1xP2x = Math.Abs(p1.X - p2.X);

            if (p1xP2x > tol)
            {
                if (p1xP2x > tolInit)
                {
                    i++;
                }
                else
                {
                    if (p1.X < p2.X)
                    {
                        Params.Remove(i);
                    }
                    else
                    {
                        points2.Remove(i);
                    }
                }
            }
            else
            {
                i++;
            }
        }

        if (Params.Length() > points2.Length())
        {
            Params.Remove(points2.Length() + 1, Params.Length());
        }
        else if (Params.Length() < points2.Length())
        {
            points2.Remove(Params.Length() + 1, points2.Length());
        }

        nbPoints = Params.Length();

        // Now we define: if there are more than one point of intersection
        // is it Ok ?
        double init_fpar = RealFirst(), init_lpar = RealLast();
        if (nbPoints > 1 && joinType == global::Kerf.Geometry.JoinKind.Intersection && initShape1.Type != ShapeType.Vertex
            && initShape2.Type != ShapeType.Vertex)
        {
            // definition of initial first and last parameters:
            // this is inverse procedure to extension of parameters
            //(see BRepFill_OffsetWire, function MakeOffset, case of Circle)
            Edge initEdge1 = ShapeCasts.Edge(initShape1);
            bool toExtendFirstPar = true;
            bool toExtendLastPar = true;
            if (isOpenResult)
            {
                Exploration.Vertices(initEdge1, out Vertex v1, out Vertex v2);
                if (v1.IsSame(end1) || v1.IsSame(end2))
                {
                    toExtendFirstPar = false;
                }
                if (v2.IsSame(end1) || v2.IsSame(end2))
                {
                    toExtendLastPar = false;
                }
            }
            EdgeCurveAdaptor ic1 = new(initEdge1);
            if (ic1.GetType() == Circle)
            {
                double delta = 2 * Math.PI - ic1.LastParameter() + ic1.FirstParameter();
                if (toExtendFirstPar && toExtendLastPar)
                {
                    init_fpar = ac1.FirstParameter() + delta / 2;
                }
                else if (toExtendFirstPar)
                {
                    init_fpar = ac1.FirstParameter() + delta;
                }
                else if (toExtendLastPar)
                {
                    init_fpar = ac1.FirstParameter();
                }
                init_lpar = init_fpar + ic1.LastParameter() - ic1.FirstParameter();
            }
        }

        if (nbPoints > 1 && joinType == global::Kerf.Geometry.JoinKind.Intersection)
        {
            // Remove all vertices with non-minimal parameter
            // if they are out of initial range
            int imin = 1;
            for (i = 2; i <= nbPoints; i++)
            {
                if (Params[i].X < Params[imin].X)
                {
                    imin = i;
                }
            }

            NCollection_Sequence<Point3D> paramsCopy = new(Params);
            NCollection_Sequence<Point3D> points2Copy = new(points2);
            Params.Clear();
            points2.Clear();
            for (i = 1; i <= paramsCopy.Length(); i++)
            {
                if (imin == i || (paramsCopy[i].Y >= init_fpar && paramsCopy[i].Y <= init_lpar))
                {
                    Params.Append(paramsCopy[i]);
                    points2.Append(points2Copy[i]);
                }
            }
        }

        nbPoints = Params.Length();
        for (i = 1; i <= nbPoints; i++)
        {
            pSeq = Params[i];
            pSeq = pSeq.WithZ(points2.Value(i).Y);
            Params.SetValue(i, pSeq);
        }
    }

    /// <summary>
    /// The first (<paramref name="start"/>) or last point of the bisector is on the parallels: if it wasn't found among
    /// the intersections, it is projected on both parallels and added to <paramref name="Params"/>.
    /// </summary>
    public void AddOrConfuse(bool start, in Edge edge1, in Edge edge2, NCollection_Sequence<Point3D> Params)
    {
        bool toProj = true;
        Point2D pBis;
        double tol = 10 * global::Kerf.Engine.Foundation.Precision.Confusion();

        // return curves associated to edges.
        Tool.CurveOnSurface(edge1, out Curve2D? c1, out Surface? _, out Placement _, out double f, out double l);
        CurveAdaptor2D ac1 = new(c1!, f, l);

        if (start)
        {
            pBis = _bis.Value(_bis.FirstParameter());
        }
        else
        {
            pBis = _bis.Value(_bis.LastParameter());
        }

        // Test if the end of the bissectrice is in the set of intersection points.
        if (!Params.IsEmpty())
        {
            Point2D p;
            if (start)
            {
                p = ac1.Value(Params.First().Y);
            }
            else
            {
                p = ac1.Value(Params.Last().Y);
            }
            toProj = !pBis.IsEqual(p, tol);
        }

        if (toProj)
        {
            // Project point on parallels and add in Params
            Tool.CurveOnSurface(edge2, out Curve2D? c2, out Surface? _, out Placement _, out double f2, out double l2);

            ProjectPointOnCurve2D projector1 = new(pBis, c1!, f, l);
            ProjectPointOnCurve2D projector2 = new(pBis, c2!, f2, l2);

            if (projector1.NbPoints() == 0)
            {
                return;
            }
            if (!projector1.NearestPoint().IsEqual(pBis, tol))
            {
                return;
            }
            if (projector2.NbPoints() == 0)
            {
                return;
            }
            if (!projector2.NearestPoint().IsEqual(pBis, tol))
            {
                return;
            }
            Point3D pInt = new(0, projector1.LowerDistanceParameter(), projector2.LowerDistanceParameter());
            if (start)
            {
                pInt = pInt.WithX(_bis.FirstParameter());
                Params.Prepend(pInt);
            }
            else
            {
                pInt = pInt.WithX(_bis.LastParameter());
                Params.Append(pInt);
            }
        }
    }

    /// <summary>True if <paramref name="p"/> is closer than the offset to the first generating element.</summary>
    public bool IsInside(in Point2D p)
    {
        double dist = RealLast();
        if (_isPoint1)
        {
            dist = p.DistanceTo(_p1);
        }
        else if (_isPoint2)
        {
            dist = p.DistanceTo(_p2);
        }
        else
        {
            ProjectPointOnCurve2D projector = new(p, _c1!);
            if (projector.NbPoints() > 0)
            {
                dist = projector.LowerDistance();
            }

            // Check of distances between P and first and last point of the first curve
            // should be performed in any case, despite of the results of projection.
            Point2D pf = _c1!.Value(_c1.FirstParameter);
            Point2D pl = _c1.Value(_c1.LastParameter);
            double distMin = Min(p.DistanceTo(pf), p.DistanceTo(pl));

            if (dist > distMin)
            {
                dist = distMin;
            }
        }

        return dist < Math.Abs(_offset) - global::Kerf.Engine.Foundation.Precision.Confusion();
    }
}
