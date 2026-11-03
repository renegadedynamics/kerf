// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdges.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Finds the edges of a shape lying on edges of another (same line, circle, ellipse, B-spline or Bezier curve, the
/// first edge's range inside the second's), as pairs (<see cref="EdgeFrom"/>, <see cref="EdgeTo"/>).
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shapes; the edges are theirs.</remarks>
public sealed class LocOpe_FindEdges
{
    private TopoDS_Shape myFFrom = new();
    private TopoDS_Shape myFTo = new();
    private readonly NCollection_List<TopoDS_Shape> myLFrom = new();
    private readonly NCollection_List<TopoDS_Shape> myLTo = new();
    private readonly NCollection_List<TopoDS_Shape>.Iterator myItFrom = new();
    private readonly NCollection_List<TopoDS_Shape>.Iterator myItTo = new();

    public LocOpe_FindEdges()
    {
    }

    public LocOpe_FindEdges(in TopoDS_Shape FFrom, in TopoDS_Shape FTo)
    {
        Set(FFrom, FTo);
    }

    public void InitIterator()
    {
        myItFrom.Initialize(myLFrom);
        myItTo.Initialize(myLTo);
    }

    public bool More() => myItFrom.More();

    public TopoDS_Edge EdgeFrom() => TopoDS.Edge(myItFrom.Value());

    public TopoDS_Edge EdgeTo() => TopoDS.Edge(myItTo.Value());

    public void Next()
    {
        myItFrom.Next();
        myItTo.Next();
    }

    public void Set(in TopoDS_Shape FFrom, in TopoDS_Shape FTo)
    {
        myFFrom = FFrom;
        myFTo = FTo;
        myLFrom.Clear();
        myLTo.Clear();

        TopExp_Explorer expf = new(), expt = new();
        Geom_Curve Cf, Ct;
        double ff, lf, ft, lt;
        Type Tf, Tt;

        for (expf.Init(myFFrom, TopAbs_EDGE); expf.More(); expf.Next())
        {
            TopoDS_Edge edgf = TopoDS.Edge(expf.Current());
            Cf = BRep_Tool.Curve(edgf, out Placement Loc, out ff, out lf)!;
            if (!Loc.IsIdentity)
            {
                Geom_Geometry GGf = Cf.Transformed(Loc.Transformation());
                Cf = (Geom_Curve)GGf;
            }
            Tf = Cf.GetType();
            if (Tf == typeof(Geom_TrimmedCurve))
            {
                Cf = ((Geom_TrimmedCurve)Cf).BasisCurve();
                Tf = Cf.GetType();
            }
            if (Tf != typeof(Geom_Line) && Tf != typeof(Geom_Circle) && Tf != typeof(Geom_Ellipse)
                && Tf != typeof(Geom_BSplineCurve) && Tf != typeof(Geom_BezierCurve))
            {
                continue;
            }
            for (expt.Init(myFTo, TopAbs_EDGE); expt.More(); expt.Next())
            {
                TopoDS_Edge edgt = TopoDS.Edge(expt.Current());
                Ct = BRep_Tool.Curve(edgt, out Loc, out ft, out lt)!;
                if (!Loc.IsIdentity)
                {
                    Geom_Geometry GGt = Ct.Transformed(Loc.Transformation());
                    Ct = (Geom_Curve)GGt;
                }
                Tt = Ct.GetType();
                if (Tt == typeof(Geom_TrimmedCurve))
                {
                    Ct = ((Geom_TrimmedCurve)Ct).BasisCurve();
                    Tt = Ct.GetType();
                }
                if (Tt != Tf)
                {
                    continue;
                }
                // On a presomption de confusion
                double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                if (Tt == typeof(Geom_Line))
                {
                    Line3D lif = ((Geom_Line)Cf).Lin();
                    Line3D lit = ((Geom_Line)Ct).Lin();
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
                else if (Tt == typeof(Geom_Circle))
                {
                    Circle3D cif = ((Geom_Circle)Cf).Circ();
                    Circle3D cit = ((Geom_Circle)Ct).Circ();
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
                else if (Tt == typeof(Geom_Ellipse))
                {
                    Ellipse3D cif = ((Geom_Ellipse)Cf).Elips();
                    Ellipse3D cit = ((Geom_Ellipse)Ct).Elips();

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
                else if (Tt == typeof(Geom_BSplineCurve))
                {
                    Geom_BSplineCurve Bf = (Geom_BSplineCurve)Cf;
                    Geom_BSplineCurve Bt = (Geom_BSplineCurve)Ct;

                    bool IsSame = true;

                    int nbpoles = Bf.NbPoles();
                    if (nbpoles != Bt.NbPoles())
                    {
                        IsSame = false;
                    }

                    if (IsSame)
                    {
                        int nbknots = Bf.NbKnots();
                        if (nbknots != Bt.NbKnots())
                        {
                            IsSame = false;
                        }

                        if (IsSame)
                        {
                            NCollection_Array1<Point3D> Pf = Bf.InternalPoles();
                            NCollection_Array1<Point3D> Pt = Bt.InternalPoles();

                            double tol3d = BRep_Tool.Tolerance(edgt);
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

                                if (!Bf.IsRational())
                                {
                                    if (Bt.IsRational())
                                    {
                                        IsSame = false;
                                    }
                                }
                                else
                                {
                                    if (!Bt.IsRational())
                                    {
                                        IsSame = false;
                                    }
                                }

                                if (IsSame && Bf.IsRational())
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
                else if (Tt == typeof(Geom_BezierCurve))
                {
                    Geom_BezierCurve Bf = (Geom_BezierCurve)Cf;
                    Geom_BezierCurve Bt = (Geom_BezierCurve)Ct;

                    bool IsSame = true;

                    int nbpoles = Bf.NbPoles();
                    if (nbpoles != Bt.NbPoles())
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
                            if (!Bf.IsRational())
                            {
                                if (Bt.IsRational())
                                {
                                    IsSame = false;
                                }
                            }
                            else
                            {
                                if (!Bt.IsRational())
                                {
                                    IsSame = false;
                                }
                            }

                            if (IsSame && Bf.IsRational())
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
