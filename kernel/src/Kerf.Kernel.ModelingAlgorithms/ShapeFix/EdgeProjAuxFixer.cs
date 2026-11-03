// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeProjAux.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeProjAux.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//: r5 abv 06.04.99: ec_turbine-A.stp, #4313: protect against null curve
//    abv 09.04.99  S4136: add parameter preci (to eliminate BRepAPI::Precision)

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Projects the ends of an edge (of its 3D curve, or its vertices) onto its pcurve on a face, giving the parameters
/// of the pcurve at the edge's ends.
/// </summary>
/// <remarks>
/// A working object (decision 21). Where OCCT reverses the edge's own pcurve in place (occt-issues.md), the port
/// puts a reversed copy in its place in the edge's representations when the edge isn't frozen (decision 1263), so
/// the parameters it computes are read on the curve they were computed for.
/// </remarks>
public class EdgeProjAuxFixer : Transient
{
    protected Face _face;
    protected Edge _edge;
    protected double      _firstParam;
    protected double      _lastParam;
    protected bool        _firstDone;
    protected bool        _lastDone;

    /// <summary>Empty constructor.</summary>
    public EdgeProjAuxFixer()
    {
        _face       = new Face();
        _edge       = new Edge();
        _firstParam = 0.0;
        _lastParam  = 0.0;
        _firstDone  = false;
        _lastDone   = false;
    }

    /// <summary>Creates the tool for an edge on a face.</summary>
    public EdgeProjAuxFixer(in Face f, in Edge e)
    {
        Init(f, e);
    }

    /// <summary>Sets the edge and the face.</summary>
    public void Init(in Face f, in Edge e)
    {
        _face       = f;
        _edge       = e;
        _firstParam = _lastParam = 0.0;
        _firstDone  = _lastDone = false;
    }

    /// <summary>Computes the parameters of the ends on the pcurve, sorted in increasing order.</summary>
    public void Compute(double preci)
    {
        _firstDone = _lastDone = false;

        // Project Point3d on Surface
        // TEMPORARY Call ShapeFix_EdgeProjAux
        _firstParam = 0.0;
        _lastParam  = 0.0;
        Init2d(preci);
        if (IsFirstDone && IsLastDone)
        {
            double u1 = FirstParam();
            double u2 = LastParam();
            if (u1 >= u2)
            {
                double tmp = u1;
                u1 = u2;
                u2 = tmp;
            }
            _firstParam = u1;
            _firstDone  = true;
            _lastParam  = u2;
            _lastDone   = true;
        }
    }

    public bool IsFirstDone => _firstDone;

    public bool IsLastDone => _lastDone;

    public double FirstParam() => _firstParam;

    public double LastParam() => _lastParam;

    /// <summary>Whether the pcurve is an iso (always false: not recognised until Adaptor3d_Curve does).</summary>
    public bool IsIso(Curve2D? curve2d)
    {
        // Until an ISO is recognized by Adaptor3d_Curve
        return false;
    }

    // ----------------------------------------------------------------------------
    // static  : FindParameterWithExt
    // Purpose : Computes the trimming parameter of Pt1 on COnS
    // ----------------------------------------------------------------------------

    private static bool FindParameterWithExt(in Point3D pt1, CurveOnSurface cOnS, double uinf, double usup,
                                             double preci, ref double w1)
    {
        try
        { // et allez donc !
            var myExtPC = new CurvePointExtrema(pt1, cOnS, uinf, usup, preci);

            if (myExtPC.IsDone)
            {
                int nbExt1 = myExtPC.ExtCount;
                for (int i = 1; i <= nbExt1; i++)
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
        catch (FailureException)
        {
            return false;
        }
    }

    protected void Init2d(double preci)
    {
        double cl = 0.0, cf = 0.0;
        // Extract Geometries
        _firstDone = _lastDone = false;
        Surface  theSurface = Tool.Surface(_face)!;
        Curve2D? theCurve2d = Tool.CurveOnSurface(_edge, _face, out cf, out cl);
        if (theCurve2d is null)
        {
            return; //: r5 abv 6 Apr 99:  ec_turbine-A.stp, #4313
        }
        _firstParam = 0.0;
        _lastParam  = 0.0;
        Exploration.Vertices(_edge, out Vertex v1, out Vertex v2);
        Point3D Pt1, Pt2;
        // pdn 28.12.98: r_39-db.stp #605: use ends of 3d curve instead of vertices
        var sae = new EdgeAnalysis();
        if (sae.Curve3d(_edge, out Curve3D? c3d, out double a, out double b, false))
        {
            Pt1 = c3d!.Value(a);
            Pt2 = c3d.Value(b);
        }
        else
        {
            Pt1 = Tool.Pnt(v1);
            Pt2 = Tool.Pnt(v2);
        }
        //: S4136  double preci = BRepAPI::Precision();
        // pdn to manage degenerated case
        if (v1.IsSame(v2))
        {
            var      stsu = new SurfaceAnalysis(theSurface);
            var      pt1 = new Point2D();
            var      pt2 = new Point2D();
            double   firstpar = 0.0, lastpar = 0.0;
            if (stsu.DegeneratedValues(Pt1, preci, ref pt1, ref pt2, ref firstpar, ref lastpar))
            {
                if (theCurve2d is LineCurve2D)
                {
                    if (pt1.IsEqual(theCurve2d.Value(firstpar), preci) && pt2.IsEqual(theCurve2d.Value(lastpar), preci))
                    {
                        _firstParam = firstpar;
                        _lastParam  = lastpar;
                        _firstDone  = _lastDone = true;
                        return;
                    }
                }
            }
        }

        bool parU = false, parV = false;
        var  sa     = new SurfaceAdaptor(theSurface);
        var  myHSur = new SurfaceAdaptor(theSurface);

        cf = theCurve2d.FirstParameter();
        cl = theCurve2d.LastParameter();
        // pdn cutting pcurve by surface bounds
        if (Kerf.Engine.Foundation.Precision.IsInfinite(cf) || Kerf.Engine.Foundation.Precision.IsInfinite(cl))
        {
            if (theCurve2d is LineCurve2D)
            {
                theSurface.Bounds(out double uf, out double ul, out double vf, out double vl);
                // Correct surface limits for extrusion/revolution surfaces based on hyperbola
                // 23 is ln(1.0e+10)
                if (sa.GetType() == SurfaceOfExtrusion)
                {
                    if (sa.BasisCurve().GetType() == Hyperbola)
                    {
                        uf = Max(uf, -23.0);
                        ul = Min(ul, 23.0);
                    }
                }
                if (sa.GetType() == SurfaceOfRevolution)
                {
                    if (sa.BasisCurve().GetType() == Hyperbola)
                    {
                        vf = Max(vf, -23.0);
                        vl = Min(vl, 23.0);
                    }
                }
                if (!Kerf.Engine.Foundation.Precision.IsInfinite(uf) && !Kerf.Engine.Foundation.Precision.IsInfinite(ul) && !Kerf.Engine.Foundation.Precision.IsInfinite(vf)
                    && !Kerf.Engine.Foundation.Precision.IsInfinite(vl))
                {
                    double   cfi, cli;
                    var      lin = (LineCurve2D)theCurve2d;
                    Point2D pnt = lin.Location();
                    Direction2D dir = lin.Direction();
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
                else if (!Kerf.Engine.Foundation.Precision.IsInfinite(uf) && !Kerf.Engine.Foundation.Precision.IsInfinite(ul))
                {
                    var      lin = (LineCurve2D)theCurve2d;
                    Direction2D dir = lin.Direction();
                    if (dir.X() != 0)
                    {
                        if (dir.Y() == 0)
                        {
                            parU = true;
                        }
                        Point2D pnt = lin.Location(); // szv#4:S4163:12Mar99 moved
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
                if (theCurve2d is BSplineCurve2D)
                {
                    // Try to reparametrize (CASE dxf read bug25899)
                    var bspl     = (BSplineCurve2D)theCurve2d.Copy();
                    var newKnots = new NCollection_Array1<double>(bspl.InternalKnots());
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(cf, cl, newKnots);
                    bspl.SetKnots(newKnots);
                    theCurve2d = bspl;
                }
            }
        }

        var myHCur = new CurveAdaptor2D(theCurve2d, cf, cl);

        var cOnS = new CurveOnSurface(myHCur, myHSur);

        // ----------------------------------------------
        // --- topological limit == geometric limit ? ---
        // ----------------------------------------------
        double uinf = cOnS.FirstParameter();
        double usup = cOnS.LastParameter();

        double w1 = 0.0, w2 = 0.0;
        var    sac  = new CurveAnalysis();
        double dist = sac.Project(cOnS, Pt1, preci, out Point3D pnt0, out w1, false);
        // if distance is infinite then projection is not performed
        if (Kerf.Engine.Foundation.Precision.IsInfinite(dist))
        {
            return;
        }

        _firstDone  = true;
        _firstParam = w1;

        dist = sac.Project(cOnS, Pt2, preci, out pnt0, out w2, false);

        if (Kerf.Engine.Foundation.Precision.IsInfinite(dist))
        {
            return;
        }

        _lastDone  = true;
        _lastParam = w2;

        if (Math.Abs(w1 - w2) < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (!theSurface.IsUPeriodic && !theSurface.IsVPeriodic)
            {
                return;
            }
        }

        if (_firstParam == uinf && _lastParam == usup)
        {
            return;
        }
        if (_firstParam == usup && _lastParam == uinf)
        {
            _firstParam = theCurve2d.ReversedParameter(usup);
            _lastParam  = theCurve2d.ReversedParameter(uinf);
            Reverse(ref theCurve2d);
            return;
        }
        //: abv 29.08.01: SAT: fix for closed case
        if (cOnS.Value(uinf).Distance(cOnS.Value(usup)) < Kerf.Engine.Foundation.Precision.Confusion())
        {
            // 18.11.2002 SKL OCC630 compare values with tolerance Precision::PConfusion() instead of "=="
            if (Math.Abs(_firstParam - uinf) < Kerf.Engine.Foundation.Precision.PConfusion()
                && Math.Abs(_lastParam - uinf) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                _lastParam = w2 = usup;
                // 18.11.2002 SKL OCC630 compare values with tolerance Precision::PConfusion() instead of "=="
            }
            else if (Math.Abs(_firstParam - usup) < Kerf.Engine.Foundation.Precision.PConfusion()
                     && Math.Abs(_lastParam - usup) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                _firstParam = w1 = uinf;
            }
        }

        // pdn adjust parameters in periodic case
        if (parU || parV)
        {
            theSurface.Bounds(out double uf, out double ul, out double vf, out double vl);
            double period = (parU ? ul - uf : vl - vf);
            w1 += ShapeAnalysisAnalysis.AdjustToPeriod(w1, 0, period);
            _firstParam = w1;
            w2 += ShapeAnalysisAnalysis.AdjustToPeriod(w2, 0, period);
            _lastParam = w2;
            if (!sae.Curve3d(_edge, out Curve3D? c3d1, out cf, out cl, false))
            {
                UpdateParam2d(ref theCurve2d);
                return;
            }
            Point3D mid = c3d1!.Value((cf + cl) / 2);
            sac.Project(cOnS, mid, preci, out pnt0, out double wmid, false);
            wmid += ShapeAnalysisAnalysis.AdjustToPeriod(wmid, 0, period);
            if (w1 >= w2)
            {
                if (w2 > wmid)
                {
                    _firstParam -= period;
                }
                else if (w1 > wmid)
                {
                    UpdateParam2d(ref theCurve2d);
                }
                else
                {
                    _lastParam += period;
                }
            }
            else
            {
                if (w1 > wmid)
                {
                    _lastParam -= period;
                    UpdateParam2d(ref theCurve2d);
                }
                else if (w2 < wmid)
                {
                    _firstParam += period;
                    UpdateParam2d(ref theCurve2d);
                }
            }
        }
        UpdateParam2d(ref theCurve2d);
    }

    protected void Init3d(double preci)
    {
        // Extract Geometries
        Surface  theSurface = Tool.Surface(_face)!;
        Curve2D? theCurve2d = Tool.CurveOnSurface(_edge, _face, out _, out _);
        if (theCurve2d is null)
        {
            return; //: r5 abv 6 Apr 99:  ec_turbine-A.stp, #4313
        }

        Vertex v1  = Exploration.FirstVertex(_edge);
        Vertex v2  = Exploration.LastVertex(_edge);
        Point3D        pt1 = Tool.Pnt(v1);
        Point3D        pt2 = Tool.Pnt(v2);

        var myHSur = new SurfaceAdaptor(theSurface);

        var myHCur = new CurveAdaptor2D(theCurve2d);

        var cOnS = new CurveOnSurface(myHCur, myHSur);

        //: S4136  double preci = BRepAPI::Precision();
        double uinf = theCurve2d.FirstParameter();
        double usup = theCurve2d.LastParameter();

        // ----------------------------------------------
        // --- topological limit == geometric limit ? ---
        // ----------------------------------------------

        if (theCurve2d is BoundedCurve2D)
        {
            Point3D pdeb = cOnS.Value(uinf);
            Point3D pfin = cOnS.Value(usup);

            // szv#4:S4163:12Mar99 optimized
            if (pdeb.IsEqual(pt1, preci) && pfin.IsEqual(pt2, preci))
            {
                _firstParam = uinf;
                _lastParam  = usup;
                _firstDone  = _lastDone = true;
                return;
            }
        }

        // ------------------------------------------
        // --- The CurveOnSurface is not infinite ---
        // ---          Try with Extrema          ---
        // ------------------------------------------

        double w1 = cOnS.FirstParameter();
        double w2 = cOnS.LastParameter();

        if ((!Kerf.Engine.Foundation.Precision.IsInfinite(w1) && !Kerf.Engine.Foundation.Precision.IsInfinite(w2) && theCurve2d.Continuity() != C0)
            || IsIso(theCurve2d))
        {
            // szv#4:S4163:12Mar99 optimized
            if (FindParameterWithExt(pt1, cOnS, uinf, usup, preci, ref w1)
                && FindParameterWithExt(pt2, cOnS, uinf, usup, preci, ref w2))
            {
                _firstParam = w1;
                _lastParam  = w2;
                UpdateParam2d(ref theCurve2d);
                _firstDone = _lastDone = true;
                return;
            }
        }
        _firstDone = _lastDone = true;
    }

    protected void UpdateParam2d(ref Curve2D curve2d)
    {
        if (_firstParam < _lastParam)
        {
            return;
        }

        double cf = curve2d.FirstParameter();
        double cl = curve2d.LastParameter();
        //: S4136  double preci = BRepAPI::Precision();
        double preci2d = Kerf.Engine.Foundation.Precision.PConfusion(); //: S4136: Parametric(preci, 0.01);

        // 15.11.2002 PTV OCC966
        if (CurveAnalysis.IsPeriodic(curve2d))
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, preci2d, ref _firstParam, ref _lastParam);
        }
        else if (curve2d.IsClosed)
        {
            // szv#4:S4163:12Mar99 optimized
            if (Math.Abs(_firstParam - cl) <= preci2d)
            {
                _firstParam = cf;
            }
            else if (Math.Abs(_lastParam - cf) <= preci2d)
            {
                _lastParam = cl;
            }
            else
            {
                // add fail result;
                return;
            }
        }
        // the curve is closed within the 'file' 2D tolerance
        else if (curve2d is BSplineCurve2D bSpline2d)
        {
            if (bSpline2d.StartPoint().Distance(bSpline2d.EndPoint()) <= preci2d)
            {
                if (Math.Abs(_firstParam - cl) <= preci2d)
                {
                    _firstParam = cf;
                }
                else if (Math.Abs(_lastParam - cf) <= preci2d)
                {
                    _lastParam = cl;
                }
            }
        }
        else
        {
            double tmp1 = _firstParam, tmp2 = _lastParam;
            _firstParam = curve2d.ReversedParameter(tmp1);
            _lastParam  = curve2d.ReversedParameter(tmp2);
            Reverse(ref curve2d);
        }
    }

    // OCCT.NET: frozen (decision 1263). OCCT reverses theCurve2d in place: the edge's own pcurve (unless Init2d copied
    // it to reparametrize it), seen reversed by the edge, by the caller (who sets the edge's range on it to the
    // reversed parameters) and by every other shape sharing the curve. The port reverses a copy and puts it in place
    // of the curve in the edge's representations (both pcurves of a seam), unless the edge is frozen; other edges
    // sharing the curve keep it, as in ShapeFix_Edge.FixReversed2d.
    private void Reverse(ref Curve2D curve2d)
    {
        var reversed = (Curve2D)curve2d.Copy();
        reversed.Reverse();
        if (!WorkingOverlay.IsReadOnly(_edge.TShape()!))
        {
            EdgeFixer.InternalReplacePCurveObject(_edge, curve2d, reversed);
        }
        curve2d = reversed;
    }
}
