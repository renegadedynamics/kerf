// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NSections.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A section law through N sections: a B-spline surface interpolating the sections (approximated with
/// GeomFill_AppSurf, or cut from a given reference surface), whose v-isos are the sections.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps its own sequence of the caller's curves, as OCCT copies the sequence of
/// handles. OCCT reparametrises a B-spline section in place (<c>SetKnots</c> on the caller's curve); the port
/// reparametrises a copy and keeps the copy in its own sequence, so the law behaves as OCCT's and the caller's curve
/// is untouched (decisions.md). <see cref="BSplineSurface"/> hands out the law's surface frozen.
/// </remarks>
public class SurfaceNSections : SurfaceSectionLaw
{
    private double _uFirst;
    private double _uLast;
    private double _vFirst;
    private double _vLast;
    private readonly NCollection_Sequence<Curve3D> _sections = new();
    private readonly NCollection_Sequence<Transform3D> _trsfs = new();
    private readonly NCollection_Sequence<double> _params = new();
    private BSplineSurface? _surface;
    private BSplineSurface? _refSurf;

    // fonction d'evaluation des poles et des poids de mySurface pour D1 et D2
    private static void ResultEval(BSplineSurface surf, double v, int deriv, NCollection_Array1<double> result)
    {
        bool rational = surf.IsVRational;
        int gap = 3;
        if (rational)
        {
            gap++;
        }
        int cdeg = surf.VDegree(), cdim = surf.UPoleCount * gap, nbP = surf.VPoleCount;

        //  les noeuds plats
        NCollection_Array1<double> fKnots = surf.InternalVKnotSequence();

        //  les poles
        int psize = cdim * nbP;
        NCollection_Array1<double> sPoles = new(1, psize);
        int ii, jj, ipole = 1;
        for (jj = 1; jj <= nbP; jj++)
        {
            for (ii = 1; ii <= surf.UPoleCount; ii++)
            {
                sPoles[ipole] = surf.Pole(ii, jj).X();
                sPoles[ipole + 1] = surf.Pole(ii, jj).Y();
                sPoles[ipole + 2] = surf.Pole(ii, jj).Z();
                if (rational)
                {
                    sPoles[ipole + 3] = surf.Weight(ii, jj);
                    sPoles[ipole] *= sPoles[ipole + 3];
                    sPoles[ipole + 1] *= sPoles[ipole + 3];
                    sPoles[ipole + 2] *= sPoles[ipole + 3];
                }
                ipole += gap;
            }
        }

        bool periodic_flag = false;
        Span<int> extrap_mode = stackalloc int[2];
        extrap_mode[0] = extrap_mode[1] = cdeg;
        NCollection_Array1<double> evalBS = new(1, cdim * (deriv + 1));
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(v, periodic_flag, deriv, extrap_mode, cdeg, fKnots, cdim, sPoles.AsSpan(1), evalBS.AsSpan(1));

        for (ii = 1; ii <= cdim; ii++)
        {
            result[ii] = evalBS[ii + deriv * cdim];
        }
    }

    /// <summary>The law through the sections NC, with u in [0, 1] and v in [0, 1].</summary>
    public SurfaceNSections(NCollection_Sequence<Curve3D> nc)
    {
        _sections.Assign(nc);
        _uFirst = 0.0;
        _uLast = 1.0;
        _vFirst = 0.0;
        _vLast = 1.0;
        _refSurf = null;
        ComputeSurface();
    }

    /// <summary>The law through the sections NC at the parameters NP.</summary>
    public SurfaceNSections(NCollection_Sequence<Curve3D> nc, NCollection_Sequence<double> np)
    {
        _sections.Assign(nc);
        _params.Assign(np);
        _uFirst = 0.0;
        _uLast = 1.0;
        _vFirst = 0.0;
        _vLast = 1.0;
        _refSurf = null;
        ComputeSurface();
    }

    /// <summary>The law through the sections NC at the parameters NP, with u in [theUF, theUL].</summary>
    public SurfaceNSections(NCollection_Sequence<Curve3D> nc, NCollection_Sequence<double> np,
                              double uf, double ul)
    {
        _sections.Assign(nc);
        _params.Assign(np);
        _uFirst = uf;
        _uLast = ul;
        _vFirst = 0.0;
        _vLast = 1.0;
        _refSurf = null;
        ComputeSurface();
    }

    /// <summary>The law through the sections NC at the parameters NP, with u in [UF, UL] and v in [VF, VL].</summary>
    public SurfaceNSections(NCollection_Sequence<Curve3D> nc, NCollection_Sequence<double> np, double uf,
                              double ul, double vf, double vl)
    {
        _sections.Assign(nc);
        _params.Assign(np);
        _uFirst = uf;
        _uLast = ul;
        _vFirst = vf;
        _vLast = vl;
        _refSurf = null;
        ComputeSurface();
    }

    /// <summary>
    /// The law through the sections NC placed by Trsfs at the parameters NP, cut from the reference surface Surf
    /// (u in [UF, UL], v in [VF, VL]).
    /// </summary>
    public SurfaceNSections(NCollection_Sequence<Curve3D> nc, NCollection_Sequence<Transform3D> trsfs,
                              NCollection_Sequence<double> np, double uf, double ul, double vf, double vl,
                              BSplineSurface? surf)
    {
        _sections.Assign(nc);
        _trsfs.Assign(trsfs);
        _params.Assign(np);
        _uFirst = uf;
        _uLast = ul;
        _vFirst = vf;
        _vLast = vl;
        _refSurf = surf;
        ComputeSurface();
    }

    public override bool D0(double v, NCollection_Array1<Point3D> Poles, NCollection_Array1<double> Weights)
    {
        if (_surface is null)
        {
            return false;
        }
        else
        {
            BSplineCurve3D curve = (BSplineCurve3D)_surface.VIso(v, false);
            NCollection_Array1<Point3D> poles = curve.InternalPoles();
            NCollection_Array1<double> weights = curve.InternalWeightsArray();
            int ii, l = Poles.Length();
            for (ii = 1; ii <= l; ii++)
            {
                Poles[ii] = Poles[ii].WithXYZ(poles[ii].XYZ());
                Weights[ii] = weights[ii];
            }
        }
        return true;
    }

    public override bool D1(double v, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        if (_surface is null)
        {
            return false;
        }

        bool ok = D0(v, poles, weights);
        if (!ok)
        {
            return false;
        }

        int l = poles.Length(), derivative_request = 1;
        bool rational = _surface.IsVRational;
        int gap = 3;
        if (rational)
        {
            gap++;
        }

        int dimResult = _surface.UPoleCount * gap;
        BSplineSurface? surf_deper = null;
        if (_surface.IsVPeriodic)
        {
            surf_deper = (BSplineSurface)_surface.Copy();
            surf_deper.SetVNotPeriodic();
            dimResult = surf_deper.UPoleCount * gap;
        }
        NCollection_Array1<double> result = new(1, dimResult);
        if (_surface.IsVPeriodic)
        {
            ResultEval(surf_deper!, v, derivative_request, result);
        }
        else
        {
            ResultEval(_surface, v, derivative_request, result);
        }

        double ww;
        double epsW = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool nullWeight = false;
        if (!rational)
        {
            dWeights.Init(0.0);
        }
        int indice = 1, ii;

        //  recopie des poles du resultat sous forme de points 3D et de poids
        for (ii = 1; ii <= l && (!nullWeight); ii++)
        {
            dPoles[ii] = dPoles[ii].WithX(result[indice]);
            dPoles[ii] = dPoles[ii].WithY(result[indice + 1]);
            dPoles[ii] = dPoles[ii].WithZ(result[indice + 2]);
            if (rational)
            {
                ww = weights[ii];
                if (ww < epsW)
                {
                    nullWeight = true;
                }
                else
                {
                    dWeights[ii] = result[indice + 3];
                    dPoles[ii] = dPoles[ii].WithXYZ((dPoles[ii].XYZ() - dWeights[ii] * poles[ii].Coord()) / ww);
                }
            }
            indice += gap;
        }
        return !nullWeight;
    }

    public override bool D2(double v, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<double> weights,
                            NCollection_Array1<double> dWeights, NCollection_Array1<double> d2Weights)
    {
        if (_surface is null)
        {
            return false;
        }

        // pb dans BSplCLib::Eval() pour les surfaces rationnelles de degre 1
        // si l'ordre de derivation est egal a 2.
        if (_surface.VDegree() < 2)
        {
            return false;
        }

        bool ok = D1(v, poles, dPoles, weights, dWeights);
        if (!ok)
        {
            return false;
        }

        int l = poles.Length(), derivative_request = 2;
        bool rational = _surface.IsVRational;
        int gap = 3;
        if (rational)
        {
            gap++;
        }

        int dimResult = _surface.UPoleCount * gap;
        BSplineSurface? surf_deper = null;
        if (_surface.IsVPeriodic)
        {
            surf_deper = (BSplineSurface)_surface.Copy();
            surf_deper.SetVNotPeriodic();
            dimResult = surf_deper.UPoleCount * gap;
        }
        NCollection_Array1<double> result = new(1, dimResult);
        if (_surface.IsVPeriodic)
        {
            ResultEval(surf_deper!, v, derivative_request, result);
        }
        else
        {
            ResultEval(_surface, v, derivative_request, result);
        }

        double ww;
        double epsW = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool nullWeight = false;
        if (!rational)
        {
            d2Weights.Init(0.0);
        }
        int indice = 1, ii;

        //  recopie des poles du resultat sous forme de points 3D et de poids
        for (ii = 1; ii <= l && (!nullWeight); ii++)
        {
            d2Poles[ii] = d2Poles[ii].WithX(result[indice]);
            d2Poles[ii] = d2Poles[ii].WithY(result[indice + 1]);
            d2Poles[ii] = d2Poles[ii].WithZ(result[indice + 2]);
            if (rational)
            {
                ww = weights[ii];
                if (ww < epsW)
                {
                    nullWeight = true;
                }
                else
                {
                    d2Weights[ii] = result[indice + 3];
                    d2Poles[ii] = d2Poles[ii].WithXYZ((d2Poles[ii].XYZ() - d2Weights[ii] * poles[ii].Coord()
                                        - 2 * dWeights[ii] * dPoles[ii].XYZ())
                                       / ww);
                }
            }
            indice += gap;
        }
        return !nullWeight;
    }

    /// <summary>The law's surface (frozen), null if it couldn't be built (fewer than two sections).</summary>
    public override BSplineSurface? BSplineSurface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary>The law's surface, not frozen, for ported callers.</summary>
    internal BSplineSurface? InternalBSplineSurface() => _surface;

    /// <summary>Sets the reference surface used by the next <see cref="ComputeSurface"/>.</summary>
    public void SetSurface(BSplineSurface? refSurf)
    {
        _refSurf = refSurf;
    }

    /// <summary>Builds the law's surface: approximated through the sections, or cut from the reference surface.</summary>
    public void ComputeSurface()
    {
        BSplineSurface bs;
        if (_refSurf is null)
        {
            double myPres3d = 1.0e-06;
            int i, j, jdeb = 1, jfin = _sections.Length();

            if (jfin <= jdeb)
            {
                // We will not be able to create surface from single curve.
                return;
            }

            SectionGenerator section = new SectionGenerator();

            for (j = jdeb; j <= jfin; j++)
            {
                // read the j-th curve
                Curve3D curv = _sections[j];

                // transformation to BSpline reparametrized to [UFirst,ULast]
                BSplineCurve3D? curvBS = curv as BSplineCurve3D;
                if (curvBS is null)
                {
                    curvBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(curv, QuasiAngular);
                }
                else
                {
                    // OCCT reparametrises the caller's curve in place; the port reparametrises a copy and keeps it in
                    // its own sequence, as OCCT's sequence then holds the reparametrised curve (decisions.md).
                    curvBS = (BSplineCurve3D)curvBS.Copy();
                    _sections[j] = curvBS;
                }

                NCollection_Array1<double> bsk = new(curvBS.InternalKnots());
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(_uFirst, _uLast, bsk);
                curvBS.SetKnots(bsk);

                section.AddCurve(curvBS);
            }

            int nbcurves = _sections.Length();
            int nbpar = _params.Length();
            if (nbpar > 0)
            {
                NCollection_HArray1<double> hPar = new NCollection_HArray1<double>(1, nbpar);
                for (i = 1; i <= nbpar; i++)
                {
                    hPar.SetValue(i, _params[i]);
                }
                section.SetParam(hPar);
            }
            section.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());

            SurfaceLine line = new SurfaceLine(nbcurves);
            int nbIt = 0, degmin = 2, degmax = 6;
            bool knownP = nbpar > 0;
            SurfaceAppSurf approx = new SurfaceAppSurf(degmin, degmax, myPres3d, myPres3d, nbIt, knownP);
            approx.SetContinuity(C1);
            bool spApprox = true;
            approx.Perform(line, section, spApprox);

            bs = new BSplineSurface(approx.SurfPoles(),
                                         approx.SurfWeights(),
                                         approx.SurfUKnots(),
                                         approx.SurfVKnots(),
                                         approx.SurfUMults(),
                                         approx.SurfVMults(),
                                         approx.UDegree(),
                                         approx.VDegree(),
                                         section.IsPeriodic);
        }

        else
        {
            // segmentation de myRefSurf
            double ui1, ui2, v0, v1;
            bs = (BSplineSurface)_refSurf.Copy();
            ui1 = _uFirst;
            ui2 = _uLast;
            int i1, i2;
            _refSurf.LocateU(ui1, global::Kerf.Engine.Foundation.Precision.PConfusion(), out i1, out i2);
            if (Math.Abs(ui1 - _refSurf.UKnot(i1)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ui1 = _refSurf.UKnot(i1);
            }
            if (Math.Abs(ui1 - _refSurf.UKnot(i2)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ui1 = _refSurf.UKnot(i2);
            }
            _refSurf.LocateU(ui2, global::Kerf.Engine.Foundation.Precision.PConfusion(), out i1, out i2);
            if (Math.Abs(ui2 - _refSurf.UKnot(i1)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ui2 = _refSurf.UKnot(i1);
            }
            if (Math.Abs(ui2 - _refSurf.UKnot(i2)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ui2 = _refSurf.UKnot(i2);
            }
            v0 = _refSurf.VKnot(_refSurf.FirstVKnotIndex());
            v1 = _refSurf.VKnot(_refSurf.LastVKnotIndex());
            bs.CheckAndSegment(ui1, ui2, v0, v1);
        }
        _surface = bs;
        // On augmente le degre pour que le positionnement D2 soit correct
        if (_surface.VDegree() < 2)
        {
            _surface.IncreaseDegree(_surface.UDegree(), 2);
        }
    }

    public override void SectionShape(out int nbPoles, out int nbKnots, out int degree)
    {
        // OCCT leaves the outputs unwritten when there is no surface; they are 0 here.
        nbPoles = nbKnots = degree = 0;
        if (_surface is null)
        {
            return;
        }

        nbPoles = _surface.UPoleCount;
        nbKnots = _surface.UKnotCount;
        degree = _surface.UDegree();
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        if (_surface is not null)
        {
            tKnots.Assign(_surface.InternalUKnots());
        }
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        if (_surface is not null)
        {
            tMults.Assign(_surface.InternalUMultiplicities());
        }
    }

    public override bool IsRational
    {
        get
        {
            if (_surface is not null)
            {
                return _surface.IsURational;
            }

            return false;
        }
    }

    public override bool IsUPeriodic
    {
        get
        {
            if (_surface is not null)
            {
                return _surface.IsUPeriodic;
            }

            return false;
        }
    }

    public override bool IsVPeriodic
    {
        get
        {
            if (_surface is not null)
            {
                return _surface.IsVPeriodic;
            }

            return false;
        }
    }

    public override int NbIntervals(Continuity s)
    {
        if (_surface is null)
        {
            return 0;
        }

        SurfaceAdaptor adS = new SurfaceAdaptor(_surface);
        return adS.NbVIntervals(s);
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        if (_surface is null)
        {
            return;
        }

        SurfaceAdaptor adS = new SurfaceAdaptor(_surface);
        adS.VIntervals(t, s);
    }

    public override void SetInterval(double first, double last)
    {
        // rien a faire : mySurface est supposee Cn en V
    }

    public override void GetInterval(out double f, out double l)
    {
        f = _vFirst;
        l = _vLast;
    }

    public override void GetDomain(out double f, out double l)
    {
        f = _vFirst;
        l = _vLast;
    }

    public override void GetTolerance(double boundTol, double surfTol, double angleTol, NCollection_Array1<double> tol3d)
    {
        tol3d.Init(surfTol);
        if (boundTol < surfTol)
        {
            tol3d[tol3d.Lower()] = boundTol;
            tol3d[tol3d.Upper()] = boundTol;
        }
    }

    public override Point3D BarycentreOfSurf()
    {
        Point3D p;
        Point3D bary = new Point3D().WithCoord(0.0, 0.0, 0.0);

        if (_surface is null)
        {
            return bary;
        }

        int ii, jj;
        double u0, u1, v0, v1;
        _surface.Bounds(out u0, out u1, out v0, out v1);
        double v = v0, deltaV = (v1 - v0) / 20;
        // OCCT: U is not reset for each V, so only the first row samples the surface's u range.
        double u = u0, deltaU = (u1 - u0) / 20;
        for (jj = 0; jj <= 20; jj++, v += deltaV)
        {
            for (ii = 0; ii <= 20; ii++, u += deltaU)
            {
                p = _surface.Value(u, v);
                bary = new Point3D(bary.XYZ() + p.XYZ());
            }
        }

        bary = new Point3D(bary.XYZ() / ((21 * 21)));
        return bary;
    }

    public override double MaximalSection()
    {
        double l, lmax = 0.0;
        int ii;
        for (ii = 1; ii <= _sections.Length(); ii++)
        {
            CurveAdaptor ac = new CurveAdaptor(_sections[ii]);
            l = AbscissaPoint.Length(ac);
            if (l > lmax)
            {
                lmax = l;
            }
        }
        return lmax;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weights)
    {
        if (_surface is null)
        {
            return;
        }

        if (_surface.IsURational)
        {
            int nbU = _surface.UPoleCount, nbV = _surface.VPoleCount;
            NCollection_Array2<double>? wSurfPtr = _surface.InternalWeights();
            if (wSurfPtr is null)
            {
                weights.Init(1);
                return;
            }
            NCollection_Array2<double> wSurf = wSurfPtr;
            int i, j;
            for (i = 1; i <= nbU; i++)
            {
                double min = wSurf[i, 1];
                for (j = 2; j <= nbV; j++)
                {
                    if (min > wSurf[i, j])
                    {
                        min = wSurf[i, j];
                    }
                }
                weights.SetValue(i, min);
            }
        }
        else
        {
            weights.Init(1);
        }
    }

    public override bool IsConstant(ref double error)
    {
        // on se limite a 2 sections
        bool isconst = (_sections.Length() == 2);
        double err = 0.0;

        if (isconst)
        {
            CurveAdaptor ac1 = new CurveAdaptor(_sections[1]);
            CurveKind cType = ac1.GetType();
            CurveAdaptor ac2 = new CurveAdaptor(_sections[2]);
            // les sections doivent avoir le meme type
            isconst = (ac2.GetType() == cType);

            if (isconst)
            {
                if (cType == Circle)
                {
                    Circle3D c1 = ac1.Circle();
                    Circle3D c2 = ac2.Circle();
                    double tol = 1.0e-7;
                    bool samedir, samerad, samepos;
                    samedir = (c1.Axis().IsParallel(c2.Axis(), 1.0e-4));
                    samerad = (Math.Abs(c1.Radius() - c2.Radius()) < tol);
                    samepos = (c1.Location().Distance(c2.Location()) < tol);
                    if (!samepos)
                    {
                        Axis3D d = new Axis3D(c1.Location(), new Direction3D(new Vector3D(c1.Location(), c2.Location())));
                        samepos = (c1.Axis().IsParallel(d, 1.0e-4));
                    }
                    isconst = samedir && samerad && samepos;
                }
                else if (cType == Line)
                {
                    Line3D l1 = ac1.Line();
                    Line3D l2 = ac2.Line();
                    double tol = 1.0e-7;
                    bool samedir, samelength, samepos;
                    samedir = (l1.Direction().IsParallel(l2.Direction(), 1.0e-4));
                    Point3D p11 = ac1.Value(ac1.FirstParameter()), p12 = ac1.Value(ac1.LastParameter()),
                           p21 = ac2.Value(ac2.FirstParameter()), p22 = ac2.Value(ac2.LastParameter());
                    samelength = (Math.Abs(p11.Distance(p12) - p21.Distance(p22)) < tol);
                    // l'ecart entre les 2 sections ne compte pas
                    samepos = ((p11.Distance(p21) < tol && p12.Distance(p22) < tol)
                               || (p12.Distance(p21) < tol && p11.Distance(p22) < tol));
                    // samepos = true;
                    isconst = samedir && samelength && samepos;
                }
                else
                {
                    isconst = false;
                }
            }
        }

        error = err;
        return isconst;
    }

    /// <summary>A new copy of the first section.</summary>
    public override Curve3D ConstantSection()
    {
        //  double Err;
        //  if (!IsConstant(Err)) throw StdFail_NotDone("The Law is not Constant!");
        Curve3D c;
        c = (Curve3D)_sections[1].Copy();
        return c;
    }

    public override bool IsConicalLaw(ref double error)
    {
        bool isconic = (_sections.Length() == 2);
        double err = 0.0;
        if (isconic)
        {
            CurveAdaptor ac1 = new CurveAdaptor(_sections[1]);
            CurveAdaptor ac2 = new CurveAdaptor(_sections[2]);
            isconic = (ac1.GetType() == Circle) && (ac2.GetType() == Circle);
            if (isconic)
            {
                Circle3D c1 = ac1.Circle();
                if (!_trsfs.IsEmpty())
                {
                    c1 = c1.Transformed(_trsfs[1].Inverted());
                }
                Circle3D c2 = ac2.Circle();
                if (!_trsfs.IsEmpty())
                {
                    c2 = c2.Transformed(_trsfs[2].Inverted());
                }
                double tol = 1.0e-7;
                // bool samedir, linearrad, sameaxis;
                isconic = (c1.Axis().IsParallel(c2.Axis(), 1.0e-4));
                //  pour 2 sections, la variation du rayon est forcement lineaire
                if (isconic)
                {
                    Line3D line1 = new Line3D(c1.Axis());
                    isconic = (line1.Distance(c2.Location()) < tol);
                    if (isconic)
                    {
                        //// Modified by jgv, 18.02.2009 for OCC20866 ////
                        double first1 = ac1.FirstParameter(), last1 = ac1.LastParameter();
                        double first2 = ac2.FirstParameter(), last2 = ac2.LastParameter();
                        isconic = (Math.Abs(first1 - first2) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                                   && Math.Abs(last1 - last2) <= global::Kerf.Engine.Foundation.Precision.PConfusion());
                        //////////////////////////////////////////////////
                    }
                }
            }
        }

        error = err;
        return isconic;
    }

    /// <summary>The circle section at V (new geometry); throws StdFail_NotDone if the law is not conical.</summary>
    public override Curve3D CirclSection(double v)
    {
        double err = 0;
        if (!IsConicalLaw(ref err))
        {
            throw new NotDoneException("The Law is not Conical!");
        }

        CurveAdaptor ac1 = new CurveAdaptor(_sections[1]);
        CurveAdaptor ac2 = new CurveAdaptor(_sections[_sections.Length()]);
        Circle3D c1 = ac1.Circle();
        Circle3D c2 = ac2.Circle();

        double p1 = _params[1], p2 = _params[_params.Length()];
        double radius = (c2.Radius() - c1.Radius()) * (v - p1) / (p2 - p1) + c1.Radius();

        c1 = c1.WithRadius(radius);
        Curve3D c = new CircleCurve3D(c1);

        double parF = ac1.FirstParameter();
        double parL = ac1.LastParameter();
        double period = ac1.IsPeriodic ? ac1.Period() : 0.0;

        if ((period == 0.0) || (Math.Abs(parL - parF - period) > global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            Curve3D cbis = new TrimmedCurve3D(c, parF, parL);
            c = cbis;
        }
        return c;
    }
}
