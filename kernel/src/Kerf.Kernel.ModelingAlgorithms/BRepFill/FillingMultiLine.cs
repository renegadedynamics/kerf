// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_MultiLine.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_MultiLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// The continuous MultiLine BRepFill_ComputeCLine approximates: a bisector curve seen as a 3D curve on the first face
/// and a 2D curve on each of two faces (BRepFill_TrimSurfaceTool's intersection of two faces of an evolved shape).
/// </summary>
/// <remarks>A working object of BRepFill_TrimSurfaceTool (decision 21).</remarks>
public sealed class FillingMultiLine : Function
{
    private readonly Face _face1 = new();
    private readonly Face _face2 = new();
    private readonly CurveAdaptor2D _u1 = new();
    private readonly CurveAdaptor2D _v1 = new();
    private readonly CurveAdaptor2D _u2 = new();
    private readonly CurveAdaptor2D _v2 = new();
    private readonly bool _isoU1;
    private readonly bool _isoU2;
    private readonly CurveAdaptor2D _bis = new();
    private readonly int _kPart;
    private readonly Continuity _cont;

    private static bool IsIsoU(in Face face, in Edge edge)
    {
        Curve2D? c = Tool.CurveOnSurface(edge, face, out double f, out double _);
        if (c is null)
        {
            throw new ConstructionErrorException("BRepFill_MultiLine : Edge without PCurve");
        }

        Direction2D d = new(c.DN(f, 1));

        return Math.Abs(d.Dot(Constants.DX2d())) < Math.Abs(d.Dot(Constants.DY2d()));
    }

    /// <summary>An empty MultiLine (one 3D point and two 2D points).</summary>
    public FillingMultiLine()
    {
        _nbPnt2d = 2;
        _nbPnt = 1;
    }

    /// <summary>The bisector <paramref name="bissec"/> between Edge1 of Face1 and Edge2 of Face2.</summary>
    public FillingMultiLine(in Face face1,
                              in Face face2,
                              in Edge edge1,
                              in Edge edge2,
                              bool inv1,
                              bool inv2,
                              Curve2D bissec)
    {
        _face1 = face1;
        _face2 = face2;
        _bis = new CurveAdaptor2D(bissec);
        _kPart = 0;
        //
        const double mult = 5.0;
        double eps = mult * global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        _nbPnt2d = 2;
        _nbPnt = 1;

        // eval if myedges are IsoU or not
        _isoU1 = IsIsoU(face1, edge1);
        _isoU2 = IsIsoU(face2, edge2);

        // eval myU1, myV1, myU2, myV2;
        PlaneSurface refPlane;
        PlaneSurface basisPlane = new(0.0, 0.0, 1.0, 0.0);
        Placement l;

        Explorer exp = new();
        double umin = 0.0, vmin = 0.0, umax = 0.0, vmax = 0.0, u, v;
        Point2D p1, p2;
        Vector3D dz;
        Point3D p;

        // Result on Face1
        bool first = true;
        for (exp.Init(_face1, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge curEdge = ShapeCasts.Edge(exp.Current());
            Tool.UVPoints(curEdge, _face1, out p1, out p2);
            if (first)
            {
                first = false;
                umin = Min(p1.X(), p2.X());
                umax = Max(p1.X(), p2.X());

                vmin = Min(p1.Y(), p2.Y());
                vmax = Max(p1.Y(), p2.Y());
            }
            else
            {
                u = Min(p1.X(), p2.X());
                umin = Min(umin, u);
                u = Max(p1.X(), p2.X());
                umax = Max(umax, u);

                v = Min(p1.Y(), p2.Y());
                vmin = Min(vmin, v);
                v = Max(p1.Y(), p2.Y());
                vmax = Max(vmax, v);
            }
        }

        // return isos in their domain of restriction.
        Curve3D uu1, uu2, vv1, vv2;
        Surface s = Tool.Surface(_face1, out l)!;
        if (!l.IsIdentity)
        {
            s = (Surface)s.Transformed(l.Transformation());
        }

        if (_isoU1)
        {
            if (!Tool.Degenerated(edge1) && !inv1)
            {
                uu1 = s.UIso(umin);
                CurveAdaptor dummy = new(uu1);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu1 = s.UIso(umax);
                }
            }
            else
            {
                uu1 = s.UIso(umax);
                CurveAdaptor dummy = new(uu1);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu1 = s.UIso(umin);
                }
            }
            vv1 = s.VIso(vmin);
        }
        else
        {
            if (!Tool.Degenerated(edge1) && !inv1)
            {
                uu1 = s.VIso(vmin);
                CurveAdaptor dummy = new(uu1);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu1 = s.VIso(vmax);
                }
            }
            else
            {
                uu1 = s.VIso(vmax);
                CurveAdaptor dummy = new(uu1);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu1 = s.VIso(vmin);
                }
            }
            vv1 = s.UIso(umin);
        }

        if (_isoU1)
        {
            double dummyUmin = umin, dummyUmax = umax;
            umin = vmin;
            umax = vmax;
            vmin = dummyUmin;
            vmax = dummyUmax;
        }

        // try duplication
        SurfaceAdaptor gas1 = new(s);
        SurfaceKind type1 = gas1.GetType();

        if (uu1.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(uu1.FirstParameter(), uu1.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref umin, ref umax);
        }
        if (vv1.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(vv1.FirstParameter(), vv1.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref vmin, ref vmax);
        }
        // end try duplication

        _u1 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(uu1, basisPlane)!, umin, umax);

        uu1.D1(umin, out p, out dz);
        refPlane = new PlaneSurface(new CoordinateSystem(p, new Direction3D(dz), Constants.DZ()));

        _v1 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(vv1, refPlane)!, vmin, vmax);

        first = true;
        for (exp.Init(_face2, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge curEdge = ShapeCasts.Edge(exp.Current());
            Tool.UVPoints(curEdge, _face2, out p1, out p2);
            if (first)
            {
                first = false;
                umin = Min(p1.X(), p2.X());
                umax = Max(p1.X(), p2.X());

                vmin = Min(p1.Y(), p2.Y());
                vmax = Max(p1.Y(), p2.Y());
            }
            else
            {
                u = Min(p1.X(), p2.X());
                umin = Min(umin, u);
                u = Max(p1.X(), p2.X());
                umax = Max(umax, u);

                v = Min(p1.Y(), p2.Y());
                vmin = Min(vmin, v);
                v = Max(p1.Y(), p2.Y());
                vmax = Max(vmax, v);
            }
        }

        // return isos in their domain of restriction.
        s = Tool.Surface(_face2, out l)!;

        if (!l.IsIdentity)
        {
            s = (Surface)s.Transformed(l.Transformation());
        }

        if (_isoU2)
        {
            if (!Tool.Degenerated(edge2) && !inv2)
            {
                uu2 = s.UIso(umin);
                CurveAdaptor dummy = new(uu2);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu2 = s.UIso(umax);
                }
            }
            else
            {
                uu2 = s.UIso(umax);
                CurveAdaptor dummy = new(uu2);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu2 = s.UIso(umin);
                }
            }
            vv2 = s.VIso(vmin);
        }
        else
        {
            if (!Tool.Degenerated(edge2) && !inv2)
            {
                uu2 = s.VIso(vmin);
                CurveAdaptor dummy = new(uu2);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu2 = s.VIso(vmax);
                }
            }
            else
            {
                uu2 = s.VIso(vmax);
                CurveAdaptor dummy = new(uu2);
                if (dummy.GetType() == Circle && dummy.Circle().Radius() < eps)
                {
                    uu2 = s.VIso(vmin);
                }
            }
            vv2 = s.UIso(umin);
        }

        if (_isoU2)
        {
            double dummyUmin = umin, dummyUmax = umax;
            umin = vmin;
            umax = vmax;
            vmin = dummyUmin;
            vmax = dummyUmax;
        }

        // try duplication
        SurfaceAdaptor gas2 = new(s);
        SurfaceKind type2 = gas2.GetType();

        if (uu2.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(uu2.FirstParameter(), uu2.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref umin, ref umax);
        }
        if (vv2.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(vv2.FirstParameter(), vv2.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref vmin, ref vmax);
        }
        // end try duplication

        _u2 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(uu2, basisPlane)!, umin, umax);

        uu2.D1(umin, out p, out dz);
        refPlane = new PlaneSurface(new CoordinateSystem(p, new Direction3D(dz), Constants.DZ()));
        _v2 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(vv2, refPlane)!, vmin, vmax);

        // eval if in a particular case.
        // Particular case if :
        //     1) - Straight Bissectrice
        //        - Bissectrice orthogonal to the base element.
        //        ==> Iso on 2 faces.
        //     2) - Straight Bissectrice
        //        - 2 surfaces are planes.
        _cont = C0;

        if (_bis.GetType() == Line)
        {
            double deltaU = _bis.LastParameter() - _bis.FirstParameter();
            Point2D pnt1 = ValueOnF1(_bis.FirstParameter() + 0.1 * deltaU);
            Point2D pnt2 = ValueOnF1(_bis.FirstParameter() + 0.9 * deltaU);
            if (_isoU1)
            {
                if (Math.Abs(pnt1.Y() - pnt2.Y()) < eps)
                {
                    _kPart = 1;
                }
            }
            else
            {
                if (Math.Abs(pnt1.X() - pnt2.X()) < eps)
                {
                    _kPart = 1;
                }
            }

            if (_kPart == 1)
            {
                _cont = G1;
            }

            if ((type1 == Plane) && (type2 == Plane))
            {
                _kPart = 2;
            }
        }
    }

    /// <summary>True if the bisector is a line that gives isos on both faces, or lies between two planes.</summary>
    public bool IsParticularCase => _kPart != 0;

    /// <summary>The continuity of the line.</summary>
    public Continuity Continuity() => _cont;

    /// <summary>In a particular case, the 3D curve and the two pcurves, built exactly.</summary>
    public void Curves(out Curve3D? curve, out Curve2D? pCurve1, out Curve2D? pCurve2)
    {
        curve = null;
        pCurve1 = null;
        pCurve2 = null;
        if (_kPart == 1)
        {
            Point2D p1, p2, pMil;
            double f, l;

            p1 = ValueOnF1(_bis.FirstParameter());
            p2 = ValueOnF1(_bis.LastParameter());

            // find value of the with medium point
            // the ends can be degenerated points.

            pMil = ValueOnF1(0.5 * (_bis.FirstParameter() + _bis.LastParameter()));

            Surface s = Tool.Surface(_face1, out Placement L)!;
            if (!L.IsIdentity)
            {
                s = (Surface)s.Transformed(L.Transformation());
            }

            bool sens;
            if (!_isoU1)
            {
                curve = s.UIso(pMil.X());
                sens = p1.Y() < p2.Y();
                if (sens)
                {
                    curve = new TrimmedCurve3D(curve, p1.Y(), p2.Y(), sens);
                }
                else
                {
                    curve = new TrimmedCurve3D(curve, p2.Y(), p1.Y(), sens);
                }

                f = curve.FirstParameter();
                l = curve.LastParameter();
                if (sens)
                {
                    pCurve1 = new LineCurve2D(new Point2D(pMil.X(), p1.Y() - f), Constants.DY2d());
                }
                else
                {
                    pCurve1 = new LineCurve2D(new Point2D(pMil.X(), p1.Y() + f), Constants.DY2d().Reversed());
                }
                pCurve1 = new TrimmedCurve2D(pCurve1, f, l);
            }
            else
            {
                curve = s.VIso(pMil.Y());
                sens = p1.X() < p2.X();
                if (sens)
                {
                    curve = new TrimmedCurve3D(curve, p1.X(), p2.X(), sens);
                }
                else
                {
                    curve = new TrimmedCurve3D(curve, p2.X(), p1.X(), sens);
                }

                f = curve.FirstParameter();
                l = curve.LastParameter();
                if (sens)
                {
                    pCurve1 = new LineCurve2D(new Point2D(p1.X() - f, pMil.Y()), Constants.DX2d());
                }
                else
                {
                    pCurve1 = new LineCurve2D(new Point2D(p1.X() + f, pMil.Y()), Constants.DX2d().Reversed());
                }
                pCurve1 = new TrimmedCurve2D(pCurve1, f, l);
            }

            p1 = ValueOnF2(_bis.FirstParameter());
            p2 = ValueOnF2(_bis.LastParameter());
            pMil = ValueOnF2(0.5 * (_bis.FirstParameter() + _bis.LastParameter()));

            if (!_isoU2)
            {
                sens = p1.Y() < p2.Y();
                if (sens)
                {
                    pCurve2 = new LineCurve2D(new Point2D(pMil.X(), (p1.Y() - f)), Constants.DY2d());
                }
                else
                {
                    pCurve2 = new LineCurve2D(new Point2D(pMil.X(), (p1.Y() + f)), Constants.DY2d().Reversed());
                }
            }
            else
            {
                sens = p1.X() < p2.X();
                if (sens)
                {
                    pCurve2 = new LineCurve2D(new Point2D(p1.X() - f, pMil.Y()), Constants.DX2d());
                }
                else
                {
                    pCurve2 = new LineCurve2D(new Point2D(p1.X() + f, pMil.Y()), Constants.DX2d().Reversed());
                }
            }
            pCurve2 = new TrimmedCurve2D(pCurve2, f, l);
        }
        else if (_kPart == 2)
        {
            Surface s = Tool.Surface(_face1, out Placement L)!;
            if (!L.IsIdentity)
            {
                s = (Surface)s.Transformed(L.Transformation());
            }

            if (s is TrimmedSurface rts1)
            {
                s = rts1.BasisSurface();
            }

            PlaneSurface plane = (PlaneSurface)s;
            // eval the 3d curve corresponding to the bissectrice.
            Point2D p = _bis.Line().Location();
            Direction2D d = _bis.Line().Direction();
            LineCurve3D line = new(new Point3D(p.X(), p.Y(), 0.0), new Direction3D(d.X(), d.Y(), 0.0));
            TrimmedCurve3D tLine = new(line, _bis.FirstParameter(), _bis.LastParameter());
            curve = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(tLine, plane, Constants.DZ(), false);

            // eval PCurve1
            pCurve1 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(curve!, plane);

            // eval PCurve2
            s = Tool.Surface(_face2, out L)!;
            if (!L.IsIdentity)
            {
                s = (Surface)s.Transformed(L.Transformation());
            }
            if (s is TrimmedSurface rts2)
            {
                s = rts2.BasisSurface();
            }
            plane = (PlaneSurface)s;
            pCurve2 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(curve!, plane);
        }
    }

    /// <inheritdoc/>
    public override double FirstParameter() => _bis.FirstParameter();

    /// <inheritdoc/>
    public override double LastParameter() => _bis.LastParameter();

    /// <summary>The 3D point at <paramref name="u"/> (on the first face).</summary>
    public Point3D Value(double u)
    {
        Surface s = Tool.Surface(_face1, out Placement l)!;

        Point2D p2d = ValueOnF1(u);

        Point3D p3d = s.Value(p2d.X(), p2d.Y());
        p3d = p3d.Transformed(l.Transformation());

        return p3d;
    }

    private static Point2D ValueOnFace(double u,
                                        CurveAdaptor2D theBis,
                                        CurveAdaptor2D theU,
                                        CurveAdaptor2D theV,
                                        bool isIsoU)
    {
        Point2D p = theBis.Value(u);

        ProjectPointOnCurve2D ext = new(p, theU.Curve()!, theU.FirstParameter(), theU.LastParameter());
        //
        const double mult = 5.0;
        double eps = mult * global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        double uu = 0.0, dist = global::Kerf.Engine.Foundation.Precision.Infinite(), d1, d2;

        if (ext.NbPoints() != 0)
        {
            uu = ext.LowerDistanceParameter();
            dist = ext.LowerDistance();
        }
        // Control with `ends`
        d1 = p.Distance(theU.Value(theU.FirstParameter()));
        d2 = p.Distance(theU.Value(theU.LastParameter()));

        if (d1 < dist || d2 < dist || Math.Abs(d1 - dist) < eps || Math.Abs(d2 - dist) < eps)
        {
            if (Math.Abs(d1 - d2) < eps)
            {
                if (theU.GetType() == Circle)
                {
                    Vector2D axis = new(theU.Circle().XAxis().Direction());
                    Vector2D d12d = theBis.DN(theBis.FirstParameter(), 1);
                    double ang = axis.Angle(d12d);
                    if (!theU.Circle().IsDirect)
                    {
                        ang = -ang;
                    }
                    uu = global::Kerf.Engine.Geometry.ElCLib.InPeriod(ang, theU.FirstParameter(), theU.FirstParameter() + 2 * Math.PI);
                    dist = theU.Circle().Radius();
                }
                else
                {
                    throw new ConstructionErrorException("BRepFill_MultiLine: ValueOnFace");
                }
            }
            else if (d1 < d2)
            {
                dist = d1;
                uu = theU.FirstParameter();
            }
            else
            {
                dist = d2;
                uu = theU.LastParameter();
            }
        }

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double vv;

        Point2D pf = theV.Value(theV.FirstParameter());
        Point2D pl = theV.Value(theV.LastParameter());

        if (Math.Abs(dist - Math.Abs(pf.Y())) < tol)
        {
            vv = theV.FirstParameter();
        }
        else if (Math.Abs(dist - Math.Abs(pl.Y())) < tol)
        {
            vv = theV.LastParameter();
        }
        else
        {
            // test if the curve is at the side `negative Y`.
            if (Min(pf.Y(), pl.Y()) < -tol)
            {
                dist = -dist;
            }

            LineCurve2D line = new(new Point2D(0.0, dist), Constants.DX2d());

            CurveAdaptor2D cu1 = theV;
            CurveAdaptor2D cu2 = new(line);

            double tolConf = 0.0;

            GInter intersector = new(cu1, cu2, tolConf, tol);

            if (!intersector.IsDone)
            {
                return new Point2D(0.0, 0.0);
            }
            else
            {
                if (intersector.PointCount > 0)
                {
                    vv = intersector.Point(1).ParamOnFirst();
                }
                else if (intersector.SegmentCount > 0)
                {
                    IntersectionSegment2D seg = intersector.Segment(1);
                    double vs1 = seg.FirstPoint().ParamOnFirst();
                    double vs2 = seg.LastPoint().ParamOnFirst();
                    Point2D ps1 = theV.Value(vs1);
                    Point2D ps2 = theV.Value(vs2);
                    double alp = (dist - ps1.Y()) / (ps2.Y() - ps1.Y());
                    vv = alp * (vs2 - vs1) + vs1;
                }
                else
                {
                    if (Math.Abs(dist - pl.Y()) < Math.Abs(dist - pf.Y()))
                    {
                        vv = theV.LastParameter();
                    }
                    else
                    {
                        vv = theV.FirstParameter();
                    }
                }
            }
        }

        if (isIsoU)
        {
            return new Point2D(vv, uu);
        }
        else
        {
            return new Point2D(uu, vv);
        }
    }

    /// <summary>The point at <paramref name="u"/> on the first face's parameter plane.</summary>
    public Point2D ValueOnF1(double u) => ValueOnFace(u, _bis, _u1, _v1, _isoU1);

    /// <summary>The point at <paramref name="u"/> on the second face's parameter plane.</summary>
    public Point2D ValueOnF2(double u) => ValueOnFace(u, _bis, _u2, _v2, _isoU2);

    /// <summary>The 3D point and both 2D points at <paramref name="u"/>.</summary>
    public void Value3dOnF1OnF2(double u, out Point3D p3d, out Point2D pf1, out Point2D pf2)
    {
        pf1 = ValueOnFace(u, _bis, _u1, _v1, _isoU1);
        pf2 = ValueOnFace(u, _bis, _u2, _v2, _isoU2);

        Surface s = Tool.Surface(_face1, out Placement l)!;
        p3d = s.Value(pf1.X(), pf1.Y());
        p3d = p3d.Transformed(l.Transformation());
    }

    /// <inheritdoc/>
    public override bool Value(double t, NCollection_Array1<Point2D> pnt2d, NCollection_Array1<Point3D> pnt)
    {
        pnt[1] = Value(t);
        pnt2d[1] = ValueOnF1(t);
        pnt2d[2] = ValueOnF2(t);
        return true;
    }

    /// <inheritdoc/>
    public override bool D1(double t, NCollection_Array1<Vector2D> vec2d, NCollection_Array1<Vector3D> vec) => false;
}
