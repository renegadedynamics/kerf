// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeProjAux.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeProjAux.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//: r5 abv 06.04.99: ec_turbine-A.stp, #4313: protect against null curve
//    abv 09.04.99  S4136: add parameter preci (to eliminate BRepAPI::Precision)

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Projects the ends of an edge (of its 3D curve, or its vertices) onto its pcurve on a face, giving the parameters
/// of the pcurve at the edge's ends.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the edge and the face: where OCCT reverses the edge's own pcurve in
/// place (an OCCT bug, occt-issues.md), the port reverses a copy.
/// </remarks>
public class ShapeFix_EdgeProjAux : Standard_Transient
{
    protected TopoDS_Face myFace;
    protected TopoDS_Edge myEdge;
    protected double      myFirstParam;
    protected double      myLastParam;
    protected bool        myFirstDone;
    protected bool        myLastDone;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_EdgeProjAux()
    {
        myFace       = new TopoDS_Face();
        myEdge       = new TopoDS_Edge();
        myFirstParam = 0.0;
        myLastParam  = 0.0;
        myFirstDone  = false;
        myLastDone   = false;
    }

    /// <summary>Creates the tool for an edge on a face.</summary>
    public ShapeFix_EdgeProjAux(in TopoDS_Face F, in TopoDS_Edge E)
    {
        Init(F, E);
    }

    /// <summary>Sets the edge and the face.</summary>
    public void Init(in TopoDS_Face F, in TopoDS_Edge E)
    {
        myFace       = F;
        myEdge       = E;
        myFirstParam = myLastParam = 0.0;
        myFirstDone  = myLastDone = false;
    }

    /// <summary>Computes the parameters of the ends on the pcurve, sorted in increasing order.</summary>
    public void Compute(double preci)
    {
        myFirstDone = myLastDone = false;

        // Project Point3d on Surface
        // TEMPORARY Call ShapeFix_EdgeProjAux
        myFirstParam = 0.0;
        myLastParam  = 0.0;
        Init2d(preci);
        if (IsFirstDone() && IsLastDone())
        {
            double U1 = FirstParam();
            double U2 = LastParam();
            if (U1 >= U2)
            {
                double tmp = U1;
                U1 = U2;
                U2 = tmp;
            }
            myFirstParam = U1;
            myFirstDone  = true;
            myLastParam  = U2;
            myLastDone   = true;
        }
    }

    public bool IsFirstDone() => myFirstDone;

    public bool IsLastDone() => myLastDone;

    public double FirstParam() => myFirstParam;

    public double LastParam() => myLastParam;

    /// <summary>Whether the pcurve is an iso (always false: not recognised until Adaptor3d_Curve does).</summary>
    public bool IsIso(Geom2d_Curve? theCurve2d)
    {
        // Until an ISO is recognized by Adaptor3d_Curve
        return false;
    }

    // ----------------------------------------------------------------------------
    // static  : FindParameterWithExt
    // Purpose : Computes the trimming parameter of Pt1 on COnS
    // ----------------------------------------------------------------------------

    private static bool FindParameterWithExt(in gp_Pnt Pt1, Adaptor3d_CurveOnSurface COnS, double Uinf, double Usup,
                                             double preci, ref double w1)
    {
        try
        { // et allez donc !
            var myExtPC = new Extrema_ExtPC(Pt1, COnS, Uinf, Usup, preci);

            if (myExtPC.IsDone())
            {
                int NbExt1 = myExtPC.NbExt();
                for (int i = 1; i <= NbExt1; i++)
                {
                    if (myExtPC.IsMin(i))
                    {
                        w1 = myExtPC.Point(i).Parameter();
                    }
                }
                return true;
            }
            else
            {
                return false;
            }
        } // end try
        catch (Standard_Failure)
        {
            return false;
        }
    }

    protected void Init2d(double preci)
    {
        double cl = 0.0, cf = 0.0;
        // Extract Geometries
        myFirstDone = myLastDone = false;
        Geom_Surface  theSurface = BRep_Tool.Surface(myFace)!;
        Geom2d_Curve? theCurve2d = BRep_Tool.CurveOnSurface(myEdge, myFace, out cf, out cl);
        if (theCurve2d is null)
        {
            return; //: r5 abv 6 Apr 99:  ec_turbine-A.stp, #4313
        }
        myFirstParam = 0.0;
        myLastParam  = 0.0;
        TopExp.Vertices(myEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        gp_Pnt Pt1, Pt2;
        // pdn 28.12.98: r_39-db.stp #605: use ends of 3d curve instead of vertices
        var sae = new ShapeAnalysis_Edge();
        if (sae.Curve3d(myEdge, out Geom_Curve? C3d, out double a, out double b, false))
        {
            Pt1 = C3d!.Value(a);
            Pt2 = C3d.Value(b);
        }
        else
        {
            Pt1 = BRep_Tool.Pnt(V1);
            Pt2 = BRep_Tool.Pnt(V2);
        }
        //: S4136  double preci = BRepAPI::Precision();
        // pdn to manage degenerated case
        if (V1.IsSame(V2))
        {
            var      stsu = new ShapeAnalysis_Surface(theSurface);
            var      aPt1 = new gp_Pnt2d();
            var      aPt2 = new gp_Pnt2d();
            double   firstpar = 0.0, lastpar = 0.0;
            if (stsu.DegeneratedValues(Pt1, preci, ref aPt1, ref aPt2, ref firstpar, ref lastpar))
            {
                if (theCurve2d is Geom2d_Line)
                {
                    if (aPt1.IsEqual(theCurve2d.Value(firstpar), preci) && aPt2.IsEqual(theCurve2d.Value(lastpar), preci))
                    {
                        myFirstParam = firstpar;
                        myLastParam  = lastpar;
                        myFirstDone  = myLastDone = true;
                        return;
                    }
                }
            }
        }

        bool parU = false, parV = false;
        var  SA     = new GeomAdaptor_Surface(theSurface);
        var  myHSur = new GeomAdaptor_Surface(theSurface);

        cf = theCurve2d.FirstParameter();
        cl = theCurve2d.LastParameter();
        // pdn cutting pcurve by surface bounds
        if (Occt.Precision.IsInfinite(cf) || Occt.Precision.IsInfinite(cl))
        {
            if (theCurve2d is Geom2d_Line)
            {
                theSurface.Bounds(out double uf, out double ul, out double vf, out double vl);
                // Correct surface limits for extrusion/revolution surfaces based on hyperbola
                // 23 is ln(1.0e+10)
                if (SA.GetType() == GeomAbs_SurfaceOfExtrusion)
                {
                    if (SA.BasisCurve().GetType() == GeomAbs_Hyperbola)
                    {
                        uf = Max(uf, -23.0);
                        ul = Min(ul, 23.0);
                    }
                }
                if (SA.GetType() == GeomAbs_SurfaceOfRevolution)
                {
                    if (SA.BasisCurve().GetType() == GeomAbs_Hyperbola)
                    {
                        vf = Max(vf, -23.0);
                        vl = Min(vl, 23.0);
                    }
                }
                if (!Occt.Precision.IsInfinite(uf) && !Occt.Precision.IsInfinite(ul) && !Occt.Precision.IsInfinite(vf)
                    && !Occt.Precision.IsInfinite(vl))
                {
                    double   cfi, cli;
                    var      lin = (Geom2d_Line)theCurve2d;
                    gp_Pnt2d pnt = lin.Location();
                    gp_Dir2d dir = lin.Direction();
                    if (dir.Y() == 0)
                    {
                        parU = true;
                        cfi  = (uf - pnt.X()) / dir.X();
                        cli  = (ul - pnt.X()) / dir.X();
                    }
                    else if (dir.X() == 0)
                    {
                        parV = true;
                        cfi  = (vf - pnt.Y()) / dir.Y();
                        cli  = (vl - pnt.Y()) / dir.Y();
                    }
                    else
                    { // common case
                        double xfi, xli, yfi, yli;
                        xfi = (uf - pnt.X()) / dir.X();
                        xli = (ul - pnt.X()) / dir.X();
                        yfi = (vf - pnt.Y()) / dir.Y();
                        yli = (vl - pnt.Y()) / dir.Y();
                        if (dir.X() * dir.Y() > 0)
                        {
                            cfi = (Math.Abs(xli - xfi) < Math.Abs(xli - yfi) ? xfi : yfi);
                            cli = (Math.Abs(xfi - xli) < Math.Abs(xfi - yli) ? xli : yli);
                        }
                        else
                        {
                            cfi = (Math.Abs(xli - xfi) < Math.Abs(xli - yli) ? xfi : yli);
                            cli = (Math.Abs(yli - xli) < Math.Abs(yli - yfi) ? xli : yfi);
                        }
                    }
                    if (cfi < cli)
                    {
                        cf = cfi;
                        cl = cli;
                    }
                    else
                    {
                        cf = cli;
                        cl = cfi;
                    }
                }
                else if (!Occt.Precision.IsInfinite(uf) && !Occt.Precision.IsInfinite(ul))
                {
                    var      lin = (Geom2d_Line)theCurve2d;
                    gp_Dir2d dir = lin.Direction();
                    if (dir.X() != 0)
                    {
                        if (dir.Y() == 0)
                        {
                            parU = true;
                        }
                        gp_Pnt2d pnt = lin.Location(); // szv#4:S4163:12Mar99 moved
                        double   cfi = (uf - pnt.X()) / dir.X();
                        double   cli = (ul - pnt.X()) / dir.X();
                        if (cfi < cli)
                        {
                            cf = cfi;
                            cl = cli;
                        }
                        else
                        {
                            cf = cli;
                            cl = cfi;
                        }
                    }
                    else
                    {
                        cf = -10000;
                        cl = 10000;
                    }
                }
                else
                {
                    cf = -10000;
                    cl = 10000;
                    // pdn not cut by bounds
                }
            }
            else
            {
                // pdn not linear case not managed
                cf = -10000;
                cl = 10000;
                if (theCurve2d is Geom2d_BSplineCurve)
                {
                    // Try to reparametrize (CASE dxf read bug25899)
                    var aBspl     = (Geom2d_BSplineCurve)theCurve2d.Copy();
                    var aNewKnots = new NCollection_Array1<double>(aBspl.InternalKnots());
                    BSplCLib.Reparametrize(cf, cl, aNewKnots);
                    aBspl.SetKnots(aNewKnots);
                    theCurve2d = aBspl;
                }
            }
        }

        var myHCur = new Geom2dAdaptor_Curve(theCurve2d, cf, cl);

        var COnS = new Adaptor3d_CurveOnSurface(myHCur, myHSur);

        // ----------------------------------------------
        // --- topological limit == geometric limit ? ---
        // ----------------------------------------------
        double Uinf = COnS.FirstParameter();
        double Usup = COnS.LastParameter();

        double w1 = 0.0, w2 = 0.0;
        var    sac  = new ShapeAnalysis_Curve();
        double dist = sac.Project(COnS, Pt1, preci, out gp_Pnt pnt0, out w1, false);
        // if distance is infinite then projection is not performed
        if (Occt.Precision.IsInfinite(dist))
        {
            return;
        }

        myFirstDone  = true;
        myFirstParam = w1;

        dist = sac.Project(COnS, Pt2, preci, out pnt0, out w2, false);

        if (Occt.Precision.IsInfinite(dist))
        {
            return;
        }

        myLastDone  = true;
        myLastParam = w2;

        if (Math.Abs(w1 - w2) < Occt.Precision.PConfusion())
        {
            if (!theSurface.IsUPeriodic() && !theSurface.IsVPeriodic())
            {
                return;
            }
        }

        if (myFirstParam == Uinf && myLastParam == Usup)
        {
            return;
        }
        if (myFirstParam == Usup && myLastParam == Uinf)
        {
            myFirstParam = theCurve2d.ReversedParameter(Usup);
            myLastParam  = theCurve2d.ReversedParameter(Uinf);
            // OCCT: reverses the edge's own pcurve in place (when it wasn't copied above), changing the edge and every
            // shape sharing the pcurve. OCCT.NET: frozen: the port reverses a copy, which nothing reads afterwards.
            theCurve2d = (Geom2d_Curve)theCurve2d.Copy();
            theCurve2d.Reverse();
            return;
        }
        //: abv 29.08.01: SAT: fix for closed case
        if (COnS.Value(Uinf).Distance(COnS.Value(Usup)) < Occt.Precision.Confusion())
        {
            // 18.11.2002 SKL OCC630 compare values with tolerance Precision::PConfusion() instead of "=="
            if (Math.Abs(myFirstParam - Uinf) < Occt.Precision.PConfusion()
                && Math.Abs(myLastParam - Uinf) < Occt.Precision.PConfusion())
            {
                myLastParam = w2 = Usup;
                // 18.11.2002 SKL OCC630 compare values with tolerance Precision::PConfusion() instead of "=="
            }
            else if (Math.Abs(myFirstParam - Usup) < Occt.Precision.PConfusion()
                     && Math.Abs(myLastParam - Usup) < Occt.Precision.PConfusion())
            {
                myFirstParam = w1 = Uinf;
            }
        }

        // pdn adjust parameters in periodic case
        if (parU || parV)
        {
            theSurface.Bounds(out double uf, out double ul, out double vf, out double vl);
            double period = (parU ? ul - uf : vl - vf);
            w1 += ShapeAnalysis.AdjustToPeriod(w1, 0, period);
            myFirstParam = w1;
            w2 += ShapeAnalysis.AdjustToPeriod(w2, 0, period);
            myLastParam = w2;
            if (!sae.Curve3d(myEdge, out Geom_Curve? C3d1, out cf, out cl, false))
            {
                UpdateParam2d(theCurve2d);
                return;
            }
            gp_Pnt mid = C3d1!.Value((cf + cl) / 2);
            sac.Project(COnS, mid, preci, out pnt0, out double wmid, false);
            wmid += ShapeAnalysis.AdjustToPeriod(wmid, 0, period);
            if (w1 >= w2)
            {
                if (w2 > wmid)
                {
                    myFirstParam -= period;
                }
                else if (w1 > wmid)
                {
                    UpdateParam2d(theCurve2d);
                }
                else
                {
                    myLastParam += period;
                }
            }
            else
            {
                if (w1 > wmid)
                {
                    myLastParam -= period;
                    UpdateParam2d(theCurve2d);
                }
                else if (w2 < wmid)
                {
                    myFirstParam += period;
                    UpdateParam2d(theCurve2d);
                }
            }
        }
        UpdateParam2d(theCurve2d);
    }

    protected void Init3d(double preci)
    {
        // Extract Geometries
        Geom_Surface  theSurface = BRep_Tool.Surface(myFace)!;
        Geom2d_Curve? theCurve2d = BRep_Tool.CurveOnSurface(myEdge, myFace, out _, out _);
        if (theCurve2d is null)
        {
            return; //: r5 abv 6 Apr 99:  ec_turbine-A.stp, #4313
        }

        TopoDS_Vertex V1  = TopExp.FirstVertex(myEdge);
        TopoDS_Vertex V2  = TopExp.LastVertex(myEdge);
        gp_Pnt        Pt1 = BRep_Tool.Pnt(V1);
        gp_Pnt        Pt2 = BRep_Tool.Pnt(V2);

        var myHSur = new GeomAdaptor_Surface(theSurface);

        var myHCur = new Geom2dAdaptor_Curve(theCurve2d);

        var COnS = new Adaptor3d_CurveOnSurface(myHCur, myHSur);

        //: S4136  double preci = BRepAPI::Precision();
        double Uinf = theCurve2d.FirstParameter();
        double Usup = theCurve2d.LastParameter();

        // ----------------------------------------------
        // --- topological limit == geometric limit ? ---
        // ----------------------------------------------

        if (theCurve2d is Geom2d_BoundedCurve)
        {
            gp_Pnt Pdeb = COnS.Value(Uinf);
            gp_Pnt Pfin = COnS.Value(Usup);

            // szv#4:S4163:12Mar99 optimized
            if (Pdeb.IsEqual(Pt1, preci) && Pfin.IsEqual(Pt2, preci))
            {
                myFirstParam = Uinf;
                myLastParam  = Usup;
                myFirstDone  = myLastDone = true;
                return;
            }
        }

        // ------------------------------------------
        // --- The CurveOnSurface is not infinite ---
        // ---          Try with Extrema          ---
        // ------------------------------------------

        double w1 = COnS.FirstParameter();
        double w2 = COnS.LastParameter();

        if ((!Occt.Precision.IsInfinite(w1) && !Occt.Precision.IsInfinite(w2) && theCurve2d.Continuity() != GeomAbs_C0)
            || IsIso(theCurve2d))
        {
            // szv#4:S4163:12Mar99 optimized
            if (FindParameterWithExt(Pt1, COnS, Uinf, Usup, preci, ref w1)
                && FindParameterWithExt(Pt2, COnS, Uinf, Usup, preci, ref w2))
            {
                myFirstParam = w1;
                myLastParam  = w2;
                UpdateParam2d(theCurve2d);
                myFirstDone = myLastDone = true;
                return;
            }
        }
        myFirstDone = myLastDone = true;
    }

    protected void UpdateParam2d(Geom2d_Curve theCurve2d)
    {
        if (myFirstParam < myLastParam)
        {
            return;
        }

        double cf = theCurve2d.FirstParameter();
        double cl = theCurve2d.LastParameter();
        //: S4136  double preci = BRepAPI::Precision();
        double preci2d = Occt.Precision.PConfusion(); //: S4136: Parametric(preci, 0.01);

        // 15.11.2002 PTV OCC966
        if (ShapeAnalysis_Curve.IsPeriodic(theCurve2d))
        {
            ElCLib.AdjustPeriodic(cf, cl, preci2d, ref myFirstParam, ref myLastParam);
        }
        else if (theCurve2d.IsClosed())
        {
            // szv#4:S4163:12Mar99 optimized
            if (Math.Abs(myFirstParam - cl) <= preci2d)
            {
                myFirstParam = cf;
            }
            else if (Math.Abs(myLastParam - cf) <= preci2d)
            {
                myLastParam = cl;
            }
            else
            {
                // add fail result;
                return;
            }
        }
        // the curve is closed within the 'file' 2D tolerance
        else if (theCurve2d is Geom2d_BSplineCurve aBSpline2d)
        {
            if (aBSpline2d.StartPoint().Distance(aBSpline2d.EndPoint()) <= preci2d)
            {
                if (Math.Abs(myFirstParam - cl) <= preci2d)
                {
                    myFirstParam = cf;
                }
                else if (Math.Abs(myLastParam - cf) <= preci2d)
                {
                    myLastParam = cl;
                }
            }
        }
        else
        {
            double tmp1 = myFirstParam, tmp2 = myLastParam;
            myFirstParam = theCurve2d.ReversedParameter(tmp1);
            myLastParam  = theCurve2d.ReversedParameter(tmp2);
            // OCCT: reverses the caller's pcurve in place (the edge's own, from Init3d or Init2d). OCCT.NET: frozen: the
            // port reverses a copy, which nothing reads afterwards.
            Geom2d_Curve aCopy = (Geom2d_Curve)theCurve2d.Copy();
            aCopy.Reverse();
        }
    }
}
