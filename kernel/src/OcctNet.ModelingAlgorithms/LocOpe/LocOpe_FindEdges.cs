// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Finds the edges of a shape lying on edges of another (same line, circle, ellipse, B-spline or Bezier curve, the
/// first edge's range inside the second's), as pairs (<see cref="EdgeFrom"/>, <see cref="EdgeTo"/>).
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shapes; the edges are theirs.</remarks>
public sealed class LocOpe_FindEdges
{
    private Shape myFFrom = new();
    private Shape myFTo = new();
    private readonly NCollection_List<Shape> myLFrom = new();
    private readonly NCollection_List<Shape> myLTo = new();
    private readonly NCollection_List<Shape>.Iterator myItFrom = new();
    private readonly NCollection_List<Shape>.Iterator myItTo = new();

    public LocOpe_FindEdges()
    {
    }

    public LocOpe_FindEdges(in Shape FFrom, in Shape FTo)
    {
        Set(FFrom, FTo);
    }

    public void InitIterator()
    {
        myItFrom.Initialize(myLFrom);
        myItTo.Initialize(myLTo);
    }

    public bool More() => myItFrom.More();

    public Edge EdgeFrom() => ShapeCasts.Edge(myItFrom.Value());

    public Edge EdgeTo() => ShapeCasts.Edge(myItTo.Value());

    public void Next()
    {
        myItFrom.Next();
        myItTo.Next();
    }

    public void Set(in Shape FFrom, in Shape FTo)
    {
        myFFrom = FFrom;
        myFTo = FTo;
        myLFrom.Clear();
        myLTo.Clear();

        Explorer expf = new(), expt = new();
        Curve3D Cf, Ct;
        double ff, lf, ft, lt;
        Type Tf, Tt;

        for (expf.Init(myFFrom, ShapeType.Edge); expf.More(); expf.Next())
        {
            Edge edgf = ShapeCasts.Edge(expf.Current());
            Cf = Tool.Curve(edgf, out Placement Loc, out ff, out lf)!;
            if (!Loc.IsIdentity)
            {
                Geometry3D GGf = Cf.Transformed(Loc.Transformation());
                Cf = (Curve3D)GGf;
            }
            Tf = Cf.GetType();
            if (Tf == typeof(TrimmedCurve3D))
            {
                Cf = ((TrimmedCurve3D)Cf).BasisCurve();
                Tf = Cf.GetType();
            }
            if (Tf != typeof(LineCurve3D) && Tf != typeof(CircleCurve3D) && Tf != typeof(EllipseCurve3D)
                && Tf != typeof(BSplineCurve3D) && Tf != typeof(BezierCurve3D))
            {
                continue;
            }
            for (expt.Init(myFTo, ShapeType.Edge); expt.More(); expt.Next())
            {
                Edge edgt = ShapeCasts.Edge(expt.Current());
                Ct = Tool.Curve(edgt, out Loc, out ft, out lt)!;
                if (!Loc.IsIdentity)
                {
                    Geometry3D GGt = Ct.Transformed(Loc.Transformation());
                    Ct = (Curve3D)GGt;
                }
                Tt = Ct.GetType();
                if (Tt == typeof(TrimmedCurve3D))
                {
                    Ct = ((TrimmedCurve3D)Ct).BasisCurve();
                    Tt = Ct.GetType();
                }
                if (Tt != Tf)
                {
                    continue;
                }
                // On a presomption de confusion
                double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                if (Tt == typeof(LineCurve3D))
                {
                    Line3D lif = ((LineCurve3D)Cf).Lin();
                    Line3D lit = ((LineCurve3D)Ct).Lin();
                    Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(ff, lif);
                    Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lf, lif);
                    double prm1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lit, p1);
                    double prm2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lit, p2);
                    if (prm1 >= ft - Tol && prm1 <= lt + Tol && prm2 >= ft - Tol && prm2 <= lt + Tol)
                    {
                        Tol *= Tol;
                        Point3D pt = global::Kerf.Engine.Geometry.ElCLib.Value(prm1, lit);
                        if (pt.SquareDistance(p1) <= Tol)
                        {
                            pt = global::Kerf.Engine.Geometry.ElCLib.Value(prm2, lit);
                            if (pt.SquareDistance(p2) <= Tol)
                            {
                                myLFrom.Append(edgf);
                                myLTo.Append(edgt);
                                break;
                            }
                        }
                    }
                }
                else if (Tt == typeof(CircleCurve3D))
                {
                    Circle3D cif = ((CircleCurve3D)Cf).Circ();
                    Circle3D cit = ((CircleCurve3D)Ct).Circ();
                    if (Math.Abs(cif.Radius() - cit.Radius()) <= Tol
                        && cif.Location().SquareDistance(cit.Location()) <= Tol * Tol)
                    {
                        // Point debut, calage dans periode, et detection meme sens

                        global::Kerf.Engine.Geometry.ElCLib.D1(ff, cif, out Point3D p1, out Vector3D tgf);
                        Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(lf, cif);

                        double prm1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cit, p1);
                        double Tol2d = global::Kerf.Engine.Foundation.Precision.PConfusion();
                        if (Math.Abs(prm1 - ft) <= Tol2d)
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

                        if (prm1 >= ft - Tol && prm1 <= lt + Tol && prm2 >= ft - Tol && prm2 <= lt + Tol)
                        {
                            myLFrom.Append(edgf);
                            myLTo.Append(edgt);
                            break;
                        }
                        else
                        {
                            // Cas non traite : on est a cheval (OCCT_DEBUG trace left out)
                        }
                    }
                }
                else if (Tt == typeof(EllipseCurve3D))
                {
                    Ellipse3D cif = ((EllipseCurve3D)Cf).Elips();
                    Ellipse3D cit = ((EllipseCurve3D)Ct).Elips();

                    if (Math.Abs(cif.MajorRadius() - cit.MajorRadius()) <= Tol
                        && Math.Abs(cif.MinorRadius() - cit.MinorRadius()) <= Tol
                        && cif.Location().SquareDistance(cit.Location()) <= Tol * Tol)
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

                        if (prm1 >= ft - Tol && prm1 <= lt + Tol && prm2 >= ft - Tol && prm2 <= lt + Tol)
                        {
                            myLFrom.Append(edgf);
                            myLTo.Append(edgt);
                            break;
                        }
                        else
                        {
                            // Cas non traite : on est a cheval (OCCT_DEBUG trace left out)
                        }
                    }
                }
                else if (Tt == typeof(BSplineCurve3D))
                {
                    BSplineCurve3D Bf = (BSplineCurve3D)Cf;
                    BSplineCurve3D Bt = (BSplineCurve3D)Ct;

                    bool IsSame = true;

                    int nbpoles = Bf.PoleCount;
                    if (nbpoles != Bt.PoleCount)
                    {
                        IsSame = false;
                    }

                    if (IsSame)
                    {
                        int nbknots = Bf.KnotCount;
                        if (nbknots != Bt.KnotCount)
                        {
                            IsSame = false;
                        }

                        if (IsSame)
                        {
                            NCollection_Array1<Point3D> Pf = Bf.InternalPoles();
                            NCollection_Array1<Point3D> Pt = Bt.InternalPoles();

                            double tol3d = Tool.Tolerance(edgt);
                            for (int p = 1; p <= nbpoles; p++)
                            {
                                if ((Pf[p]).Distance(Pt[p]) > tol3d)
                                {
                                    IsSame = false;
                                    break;
                                }
                            }

                            if (IsSame)
                            {
                                NCollection_Array1<double> Kf = Bf.InternalKnots();
                                NCollection_Array1<double> Kt = Bt.InternalKnots();

                                NCollection_Array1<int> Mf = Bf.InternalMultiplicities();
                                NCollection_Array1<int> Mt = Bt.InternalMultiplicities();

                                for (int k = 1; k <= nbknots; k++)
                                {
                                    if ((Kf[k] - Kt[k]) > Tol) // OCCT: the difference isn't taken in absolute value
                                    {
                                        IsSame = false;
                                        break;
                                    }
                                    if (Math.Abs(Mf[k] - Mt[k]) > Tol)
                                    {
                                        IsSame = false;
                                        break;
                                    }
                                }

                                if (!Bf.IsRational)
                                {
                                    if (Bt.IsRational)
                                    {
                                        IsSame = false;
                                    }
                                }
                                else
                                {
                                    if (!Bt.IsRational)
                                    {
                                        IsSame = false;
                                    }
                                }

                                if (IsSame && Bf.IsRational)
                                {
                                    NCollection_Array1<double> Wf = Bf.InternalWeightsArray();
                                    NCollection_Array1<double> Wt = Bt.InternalWeightsArray();

                                    for (int w = 1; w <= nbpoles; w++)
                                    {
                                        if (Math.Abs(Wf[w] - Wt[w]) > Tol)
                                        {
                                            IsSame = false;
                                            break;
                                        }
                                    }
                                }

                                if (IsSame)
                                {
                                    // OCCT_DEBUG trace left out ("memes bsplines.")
                                    myLFrom.Append(edgf);
                                    myLTo.Append(edgt);
                                    break;
                                }
                            }
                        }
                    }
                }
                else if (Tt == typeof(BezierCurve3D))
                {
                    BezierCurve3D Bf = (BezierCurve3D)Cf;
                    BezierCurve3D Bt = (BezierCurve3D)Ct;

                    bool IsSame = true;

                    int nbpoles = Bf.PoleCount;
                    if (nbpoles != Bt.PoleCount)
                    {
                        IsSame = false;
                    }

                    if (IsSame)
                    {
                        NCollection_Array1<Point3D> Pf = Bf.InternalPoles();
                        NCollection_Array1<Point3D> Pt = Bt.InternalPoles();

                        for (int p = 1; p <= nbpoles; p++)
                        {
                            if ((Pf[p]).Distance(Pt[p]) > Tol)
                            {
                                IsSame = false;
                                break;
                            }
                        }

                        if (IsSame)
                        {
                            if (!Bf.IsRational)
                            {
                                if (Bt.IsRational)
                                {
                                    IsSame = false;
                                }
                            }
                            else
                            {
                                if (!Bt.IsRational)
                                {
                                    IsSame = false;
                                }
                            }

                            if (IsSame && Bf.IsRational)
                            {
                                NCollection_Array1<double> Wf = Bf.InternalWeightsArray();
                                NCollection_Array1<double> Wt = Bt.InternalWeightsArray();

                                for (int w = 1; w <= nbpoles; w++)
                                {
                                    if (Math.Abs(Wf[w] - Wt[w]) > Tol)
                                    {
                                        IsSame = false;
                                        break;
                                    }
                                }
                            }

                            if (IsSame)
                            {
                                // OCCT_DEBUG trace left out ("memes beziers.")
                                myLFrom.Append(edgf);
                                myLTo.Append(edgt);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}
