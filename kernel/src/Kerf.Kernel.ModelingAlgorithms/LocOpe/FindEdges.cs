// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// Finds the edges of a shape lying on edges of another (same line, circle, ellipse, B-spline or Bezier curve, the
/// first edge's range inside the second's), as pairs (<see cref="EdgeFrom"/>, <see cref="EdgeTo"/>).
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shapes; the edges are theirs.</remarks>
internal sealed class FindEdges
{
    private Shape _fFrom = new();
    private Shape _fTo = new();
    private readonly NCollection_List<Shape> _lFrom = new();
    private readonly NCollection_List<Shape> _lTo = new();
    private readonly NCollection_List<Shape>.Iterator _itFrom = new();
    private readonly NCollection_List<Shape>.Iterator _itTo = new();

    public FindEdges()
    {
    }

    public FindEdges(in Shape fFrom, in Shape fTo)
    {
        Set(fFrom, fTo);
    }

    public void InitIterator()
    {
        _itFrom.Initialize(_lFrom);
        _itTo.Initialize(_lTo);
    }

    public bool More() => _itFrom.More();

    public Edge EdgeFrom() => ShapeCasts.Edge(_itFrom.Value());

    public Edge EdgeTo() => ShapeCasts.Edge(_itTo.Value());

    public void Next()
    {
        _itFrom.Next();
        _itTo.Next();
    }

    public void Set(in Shape fFrom, in Shape fTo)
    {
        _fFrom = fFrom;
        _fTo = fTo;
        _lFrom.Clear();
        _lTo.Clear();

        Explorer expf = new(), expt = new();
        Curve3D cf, ct;
        double ff, lf, ft, lt;
        Type tf, tt;

        for (expf.Init(_fFrom, ShapeType.Edge); expf.More(); expf.Next())
        {
            Edge edgf = ShapeCasts.Edge(expf.Current());
            cf = Tool.Curve(edgf, out Placement loc, out ff, out lf)!;
            if (!loc.IsIdentity)
            {
                Geometry3D gGf = cf.Transformed(loc.Transformation);
                cf = (Curve3D)gGf;
            }
            tf = cf.GetType();
            if (tf == typeof(TrimmedCurve3D))
            {
                cf = ((TrimmedCurve3D)cf).BasisCurve;
                tf = cf.GetType();
            }
            if (tf != typeof(LineCurve3D) && tf != typeof(CircleCurve3D) && tf != typeof(EllipseCurve3D)
                && tf != typeof(BSplineCurve3D) && tf != typeof(BezierCurve3D))
            {
                continue;
            }
            for (expt.Init(_fTo, ShapeType.Edge); expt.More(); expt.Next())
            {
                Edge edgt = ShapeCasts.Edge(expt.Current());
                ct = Tool.Curve(edgt, out loc, out ft, out lt)!;
                if (!loc.IsIdentity)
                {
                    Geometry3D gGt = ct.Transformed(loc.Transformation);
                    ct = (Curve3D)gGt;
                }
                tt = ct.GetType();
                if (tt == typeof(TrimmedCurve3D))
                {
                    ct = ((TrimmedCurve3D)ct).BasisCurve;
                    tt = ct.GetType();
                }
                if (tt != tf)
                {
                    continue;
                }
                // On a presomption de confusion
                double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                if (tt == typeof(LineCurve3D))
                {
                    Line3D lif = ((LineCurve3D)cf).Line;
                    Line3D lit = ((LineCurve3D)ct).Line;
                    Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(ff, lif);
                    Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lf, lif);
                    double prm1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lit, p1);
                    double prm2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lit, p2);
                    if (prm1 >= ft - tol && prm1 <= lt + tol && prm2 >= ft - tol && prm2 <= lt + tol)
                    {
                        tol *= tol;
                        Point3D pt = global::Kerf.Engine.Geometry.ElCLib.Value(prm1, lit);
                        if (pt.SquareDistanceTo(p1) <= tol)
                        {
                            pt = global::Kerf.Engine.Geometry.ElCLib.Value(prm2, lit);
                            if (pt.SquareDistanceTo(p2) <= tol)
                            {
                                _lFrom.Append(edgf);
                                _lTo.Append(edgt);
                                break;
                            }
                        }
                    }
                }
                else if (tt == typeof(CircleCurve3D))
                {
                    Circle3D cif = ((CircleCurve3D)cf).Circle;
                    Circle3D cit = ((CircleCurve3D)ct).Circle;
                    if (Math.Abs(cif.Radius - cit.Radius) <= tol
                        && cif.Location.SquareDistanceTo(cit.Location) <= tol * tol)
                    {
                        // Point debut, calage dans periode, et detection meme sens

                        global::Kerf.Engine.Geometry.ElCLib.D1(ff, cif, out Point3D p1, out Vector3D tgf);
                        Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lf, cif);

                        double prm1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cit, p1);
                        double tol2d = global::Kerf.Engine.Foundation.Precision.PConfusion();
                        if (Math.Abs(prm1 - ft) <= tol2d)
                        {
                            prm1 = ft;
                        }
                        prm1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prm1, ft, ft + 2.0 * Math.PI);
                        global::Kerf.Engine.Geometry.ElCLib.D1(prm1, cit, out p1, out Vector3D tgt);

                        double prm2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cit, p2);
                        if (tgt.Dot(tgf) > 0.0)
                        { // meme sens
                            while (prm2 <= prm1)
                            {
                                prm2 += 2.0 * Math.PI;
                            }
                        }
                        else
                        {
                            if (Math.Abs(prm1 - ft) <= global::Kerf.Engine.Foundation.Precision.Angular())
                            {
                                prm1 += 2.0 * Math.PI;
                            }
                            while (prm2 >= prm1)
                            {
                                prm2 -= 2.0 * Math.PI;
                            }
                        }

                        if (prm1 >= ft - tol && prm1 <= lt + tol && prm2 >= ft - tol && prm2 <= lt + tol)
                        {
                            _lFrom.Append(edgf);
                            _lTo.Append(edgt);
                            break;
                        }
                        else
                        {
                            // Cas non traite : on est a cheval (OCCT_DEBUG trace left out)
                        }
                    }
                }
                else if (tt == typeof(EllipseCurve3D))
                {
                    Ellipse3D cif = ((EllipseCurve3D)cf).Ellipse;
                    Ellipse3D cit = ((EllipseCurve3D)ct).Ellipse;

                    if (Math.Abs(cif.MajorRadius - cit.MajorRadius) <= tol
                        && Math.Abs(cif.MinorRadius - cit.MinorRadius) <= tol
                        && cif.Location.SquareDistanceTo(cit.Location) <= tol * tol)
                    {
                        // Point debut, calage dans periode, et detection meme sens

                        global::Kerf.Engine.Geometry.ElCLib.D1(ff, cif, out Point3D p1, out Vector3D tgf);
                        Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lf, cif);

                        double prm1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cit, p1);
                        prm1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prm1, ft, ft + 2.0 * Math.PI);
                        global::Kerf.Engine.Geometry.ElCLib.D1(prm1, cit, out p1, out Vector3D tgt);

                        double prm2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cit, p2);
                        if (tgt.Dot(tgf) > 0.0)
                        { // meme sens
                            while (prm2 <= prm1)
                            {
                                prm2 += 2.0 * Math.PI;
                            }
                        }
                        else
                        {
                            if (Math.Abs(prm1 - ft) <= global::Kerf.Engine.Foundation.Precision.Angular())
                            {
                                prm1 += 2.0 * Math.PI;
                            }
                            while (prm2 >= prm1)
                            {
                                prm2 -= 2.0 * Math.PI;
                            }
                        }

                        if (prm1 >= ft - tol && prm1 <= lt + tol && prm2 >= ft - tol && prm2 <= lt + tol)
                        {
                            _lFrom.Append(edgf);
                            _lTo.Append(edgt);
                            break;
                        }
                        else
                        {
                            // Cas non traite : on est a cheval (OCCT_DEBUG trace left out)
                        }
                    }
                }
                else if (tt == typeof(BSplineCurve3D))
                {
                    BSplineCurve3D bf = (BSplineCurve3D)cf;
                    BSplineCurve3D bt = (BSplineCurve3D)ct;

                    bool isSame = true;

                    int nbpoles = bf.PoleCount;
                    if (nbpoles != bt.PoleCount)
                    {
                        isSame = false;
                    }

                    if (isSame)
                    {
                        int nbknots = bf.KnotCount;
                        if (nbknots != bt.KnotCount)
                        {
                            isSame = false;
                        }

                        if (isSame)
                        {
                            NCollection_Array1<Point3D> pf = bf.InternalPoles();
                            NCollection_Array1<Point3D> Pt = bt.InternalPoles();

                            double tol3d = Tool.Tolerance(edgt);
                            for (int p = 1; p <= nbpoles; p++)
                            {
                                if ((pf[p]).DistanceTo(Pt[p]) > tol3d)
                                {
                                    isSame = false;
                                    break;
                                }
                            }

                            if (isSame)
                            {
                                NCollection_Array1<double> kf = bf.InternalKnots();
                                NCollection_Array1<double> kt = bt.InternalKnots();

                                NCollection_Array1<int> mf = bf.InternalMultiplicities();
                                NCollection_Array1<int> mt = bt.InternalMultiplicities();

                                for (int k = 1; k <= nbknots; k++)
                                {
                                    if ((kf[k] - kt[k]) > tol) // OCCT: the difference isn't taken in absolute value
                                    {
                                        isSame = false;
                                        break;
                                    }
                                    if (Math.Abs(mf[k] - mt[k]) > tol)
                                    {
                                        isSame = false;
                                        break;
                                    }
                                }

                                if (!bf.IsRational)
                                {
                                    if (bt.IsRational)
                                    {
                                        isSame = false;
                                    }
                                }
                                else
                                {
                                    if (!bt.IsRational)
                                    {
                                        isSame = false;
                                    }
                                }

                                if (isSame && bf.IsRational)
                                {
                                    NCollection_Array1<double> wf = bf.InternalWeightsArray();
                                    NCollection_Array1<double> wt = bt.InternalWeightsArray();

                                    for (int w = 1; w <= nbpoles; w++)
                                    {
                                        if (Math.Abs(wf[w] - wt[w]) > tol)
                                        {
                                            isSame = false;
                                            break;
                                        }
                                    }
                                }

                                if (isSame)
                                {
                                    // OCCT_DEBUG trace left out ("memes bsplines.")
                                    _lFrom.Append(edgf);
                                    _lTo.Append(edgt);
                                    break;
                                }
                            }
                        }
                    }
                }
                else if (tt == typeof(BezierCurve3D))
                {
                    BezierCurve3D bf = (BezierCurve3D)cf;
                    BezierCurve3D bt = (BezierCurve3D)ct;

                    bool isSame = true;

                    int nbpoles = bf.PoleCount;
                    if (nbpoles != bt.PoleCount)
                    {
                        isSame = false;
                    }

                    if (isSame)
                    {
                        NCollection_Array1<Point3D> pf = bf.InternalPoles();
                        NCollection_Array1<Point3D> Pt = bt.InternalPoles();

                        for (int p = 1; p <= nbpoles; p++)
                        {
                            if ((pf[p]).DistanceTo(Pt[p]) > tol)
                            {
                                isSame = false;
                                break;
                            }
                        }

                        if (isSame)
                        {
                            if (!bf.IsRational)
                            {
                                if (bt.IsRational)
                                {
                                    isSame = false;
                                }
                            }
                            else
                            {
                                if (!bt.IsRational)
                                {
                                    isSame = false;
                                }
                            }

                            if (isSame && bf.IsRational)
                            {
                                NCollection_Array1<double> wf = bf.InternalWeightsArray();
                                NCollection_Array1<double> wt = bt.InternalWeightsArray();

                                for (int w = 1; w <= nbpoles; w++)
                                {
                                    if (Math.Abs(wf[w] - wt[w]) > tol)
                                    {
                                        isSame = false;
                                        break;
                                    }
                                }
                            }

                            if (isSame)
                            {
                                // OCCT_DEBUG trace left out ("memes beziers.")
                                _lFrom.Append(edgf);
                                _lTo.Append(edgt);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}
