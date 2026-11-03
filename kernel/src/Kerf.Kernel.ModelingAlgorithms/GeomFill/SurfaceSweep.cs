// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Sweep.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Sweep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.ApproxStyle;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// Builds the surface swept by a section law along a location law: an elementary surface where it can (plane,
/// cylinder, cone, sphere, torus, extrusion, revolution; <see cref="ExchangeUV"/> then tells whether the sections are
/// u-isos), else a B-spline surface approximated by <see cref="SweepApproximation"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's laws, as OCCT does, and changes their interval while it
/// approximates (<see cref="SweepFunction.SetInterval"/>). <see cref="Surface"/>, <see cref="Restriction"/> and
/// <see cref="Trace"/> hand out frozen geometry; ported code reads the unfrozen results through the
/// <c>Internal*</c> accessors.
/// </remarks>
public sealed class SurfaceSweep
{
    //=======================================================================
    // class : GeomFill_Sweep_Eval
    // purpose: The evaluator for curve approximation
    //=======================================================================
    private sealed class GeomFill_Sweep_Eval : EvaluatorFunction
    {
        private readonly LocFunction _ancore;

        public GeomFill_Sweep_Eval(LocFunction tool)
        {
            _ancore = tool;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double parameter,
                                      int derivativeRequest, Span<double> result, out int errorCode)
        {
            errorCode = 0;
            _ancore.DN(parameter, startEnd[0], startEnd[1], derivativeRequest, result, ref errorCode);
        }
    }

    private double _first;
    private double _last;
    private double _sFirst;
    private double _sLast;
    private double _tol3d;
    private double _boundTol;
    private double _tol2d;
    private double _tolAngular;
    private double _sError;
    private bool _forceApproxC1;
    private readonly SurfaceLocationLaw _loc;
    private SurfaceSectionLaw? _sec;
    private Surface? _surface;
    private NCollection_HArray1<Curve2D?>? _curve2d;
    private NCollection_HArray2<double>? _cError;
    private bool _done;
    private bool _exchUV;
    private bool _isUReversed;
    private bool _isVReversed;
    private readonly bool _kPart;

    /// <summary>A sweep along Location; WithKpart lets <see cref="Build"/> look for elementary surfaces.</summary>
    public SurfaceSweep(SurfaceLocationLaw location, bool withKpart = true)
    {
        _done = false;

        _loc = location;
        _kPart = withKpart;
        SetTolerance(1.0e-4);
        _forceApproxC1 = false;

        _loc.GetDomain(out _first, out _last);
        _sFirst = _sLast = 30.081996;
        _sError = RealLast();
    }

    /// <summary>
    /// Sets the parametric bounds [First, Last] of the surface and the matching bounds SectionFirst and SectionLast of
    /// the section law: the v-iso at v is Location(v) applied to Section(w), with
    /// w = SectionFirst + (v - First) / (Last - First) * (SectionLast - SectionFirst).
    /// </summary>
    public void SetDomain(double locFirst, double locLast, double sectionFirst, double sectionLast)
    {
        _first = locFirst;
        _last = locLast;
        _sFirst = sectionFirst;
        _sLast = sectionLast;
    }

    /// <summary>
    /// Sets the tolerances: Tolerance3d of the surface's approximation, BoundTolerance on its boundaries,
    /// Tolerance2d of the 2D curves, ToleranceAngular (radians) between the section law's tangents and the iso-v's.
    /// </summary>
    public void SetTolerance(double tolerance3d, double boundTolerance = 1.0, double tolerance2d = 1.0e-5,
                             double toleranceAngular = 1.0)
    {
        _tol3d = tolerance3d;
        _boundTol = boundTolerance;
        _tol2d = tolerance2d;
        _tolAngular = toleranceAngular;
    }

    /// <summary>Asks for a C1 surface to be tried when the swept surface proves to be C0.</summary>
    public void SetForceApproxC1(bool forceApproxC1)
    {
        _forceApproxC1 = forceApproxC1;
    }

    /// <summary>Whether the sections are u-isos (some elementary surfaces).</summary>
    public bool ExchangeUV() => _exchUV;

    /// <summary>Whether the u parametrisation runs against the section's (the path's if <see cref="ExchangeUV"/>).</summary>
    public bool UReversed() => _isUReversed;

    /// <summary>Whether the v parametrisation runs against the path's (the section's if <see cref="ExchangeUV"/>).</summary>
    public bool VReversed() => _isVReversed;

    /// <summary>
    /// Builds the swept surface of Section with the approximation strategy Methode, the continuity Continuity in v,
    /// at most degree Degmax and Segmax spans in v.
    /// </summary>
    public void Build(SurfaceSectionLaw section,
                      ApproxStyle methode = ApproxStyle.Location,
                      Continuity continuity = C2,
                      int degmax = 10,
                      int segmax = 30)
    {
        // Inits
        _done = false;
        _exchUV = false;
        _isUReversed = _isVReversed = false;
        _sec = section;

        if ((_sFirst == _sLast) && (_sLast == 30.081996))
        {
            _sec.GetDomain(out _sFirst, out _sLast);
        }

        bool isKPart = false, isProduct = false;

        // Traitement des KPart
        if (_kPart)
        {
            isKPart = BuildKPart();
        }

        if (!isKPart)
        {
            _exchUV = false;
            _isUReversed = _isVReversed = false;
        }

        // Traitement des produits Formelles
        if ((!isKPart) && (methode == ApproxStyle.Location))
        {
            BSplineSurface? bs;
            bs = _sec.BSplineSurface();
            if (bs is not null)
            {
                // Approx de la loi
                // OCCT: the call is commented out, so GeomFill_Location approximates globally (occt-issues.md).
                //    isProduct = BuildProduct(Continuity, Degmax, Segmax);
            }
        }

        if (isKPart || isProduct)
        {
            // Approx du 2d
            _done = Build2d(continuity, degmax, segmax);
        }
        else
        {
            // Approx globale
            _done = BuildAll(continuity, degmax, segmax);
        }
    }

    private bool Build2d(Continuity continuity, int degmax, int segmax)
    {
        bool ok = false;
        if (_loc.Nb2dCurves() == 0)
        {
            ok = true;
        }
        return ok;
    }

    private bool BuildAll(Continuity continuity, int degmax, int segmax)
    {
        bool ok = false;

        SurfaceSweepFunction func = new SurfaceSweepFunction(_sec!, _loc, _first, _sFirst,
                                                                 (_sLast - _sFirst) / (_last - _first));
        SweepApproximation approx = new SweepApproximation(func);

        approx.Perform(_first, _last, _tol3d, _boundTol, _tol2d, _tolAngular, continuity, degmax, segmax);

        if (approx.IsDone)
        {
            ok = true;

            // La surface
            approx.SurfShape(out _, out _, out int nbUPoles, out int nbVPoles, out int nbUKnots, out int nbVKnots);

            NCollection_Array2<Point3D> poles = new(1, nbUPoles, 1, nbVPoles);
            NCollection_Array2<double> weights = new(1, nbUPoles, 1, nbVPoles);
            NCollection_Array1<double> uKnots = new(1, nbUKnots), vKnots = new(1, nbVKnots);
            NCollection_Array1<int> uMults = new(1, nbUKnots), vMults = new(1, nbVKnots);

            approx.Surface(poles, weights, uKnots, vKnots, uMults, vMults);

            _surface = new BSplineSurface(poles,
                                                weights,
                                                uKnots,
                                                vKnots,
                                                uMults,
                                                vMults,
                                                approx.UDegree(),
                                                approx.VDegree(),
                                                _sec!.IsUPeriodic);
            _sError = approx.MaxErrorOnSurf();

            if (_forceApproxC1 && !_surface.IsCNv(1))
            {
                double theTol = 1.0e-4;
                Continuity theUCont = C1, theVCont = C1;
                int degU = 14, degV = 14;
                int nmax = 16;
                int thePrec = 1;

                ApproxSurface convertApprox = new ApproxSurface(_surface, theTol, theUCont,
                                                                                        theVCont, degU, degV, nmax,
                                                                                        thePrec);
                if (convertApprox.HasResult)
                {
                    _surface = convertApprox.Surface();
                    _curve2d = new NCollection_HArray1<Curve2D?>(1, 2);
                    _cError = new NCollection_HArray2<double>(1, 2, 1, 2);

                    BSplineSurface bSplSurf = (BSplineSurface)_surface!;

                    Direction2D d = new Direction2D(Direction2D.D.Y);
                    Point2D p = new Point2D(bSplSurf.UKnot(1), 0);
                    LineCurve2D lc1 = new LineCurve2D(p, d);
                    TrimmedCurve2D tc1 = new TrimmedCurve2D(lc1, 0, bSplSurf.VKnot(bSplSurf.VKnotCount));

                    _curve2d.SetValue(1, tc1);
                    _cError.SetValue(1, 1, 0.0);
                    _cError.SetValue(2, 1, 0.0);

                    p.SetCoord(bSplSurf.UKnot(bSplSurf.UKnotCount), 0);
                    LineCurve2D lc2 = new LineCurve2D(p, d);
                    TrimmedCurve2D tc2 = new TrimmedCurve2D(lc2, 0, bSplSurf.VKnot(bSplSurf.VKnotCount));

                    _curve2d.SetValue(_curve2d.Length(), tc2);
                    _cError.SetValue(1, _curve2d.Length(), 0.0);
                    _cError.SetValue(2, _curve2d.Length(), 0.0);

                    _sError = theTol;
                }
            } // if (!mySurface->IsCNv(1))

            // Les Courbes 2d
            if (_curve2d is null)
            {
                _curve2d = new NCollection_HArray1<Curve2D?>(1, 2 + _loc.TraceNumber());
                _cError = new NCollection_HArray2<double>(1, 2, 1, 2 + _loc.TraceNumber());
                int kk, ii, ifin = 1, ideb;

                if (_loc.HasFirstRestriction)
                {
                    ideb = 1;
                }
                else
                {
                    ideb = 2;
                }
                ifin += _loc.TraceNumber();
                if (_loc.HasLastRestriction)
                {
                    ifin++;
                }

                for (ii = ideb, kk = 1; ii <= ifin; ii++, kk++)
                {
                    BSplineCurve2D c = new BSplineCurve2D(approx.Curve2dPoles(kk),
                                                                    approx.Curves2dKnots(),
                                                                    approx.Curves2dMults(),
                                                                    approx.Curves2dDegree());
                    _curve2d.SetValue(ii, c);
                    _cError.SetValue(1, ii, approx.Max2dError(kk));
                    _cError.SetValue(2, ii, approx.Max2dError(kk));
                }

                // Si les courbes de restriction, ne sont pas calcules, on prend
                // les iso Bords.
                if (!_loc.HasFirstRestriction)
                {
                    Direction2D d = new Direction2D(Direction2D.D.Y);
                    Point2D p = new Point2D(uKnots[uKnots.Lower()], 0);
                    LineCurve2D lc = new LineCurve2D(p, d);
                    TrimmedCurve2D tc = new TrimmedCurve2D(lc, _first, _last);

                    _curve2d.SetValue(1, tc);
                    _cError.SetValue(1, 1, 0.0);
                    _cError.SetValue(2, 1, 0.0);
                }

                if (!_loc.HasLastRestriction)
                {
                    Direction2D d = new Direction2D(Direction2D.D.Y);
                    Point2D p = new Point2D(uKnots[uKnots.Upper()], 0);
                    LineCurve2D lc = new LineCurve2D(p, d);
                    TrimmedCurve2D tc = new TrimmedCurve2D(lc, _first, _last);
                    _curve2d.SetValue(_curve2d.Length(), tc);
                    _cError.SetValue(1, _curve2d.Length(), 0.0);
                    _cError.SetValue(2, _curve2d.Length(), 0.0);
                }
            } // if (myCurve2d.IsNull())
        }
        return ok;
    }

    // OCCT: BuildProduct is never called (Build's call is commented out); kept as OCCT has it.
    private bool BuildProduct(Continuity continuity, int degmax, int segmax)
    {
        bool ok = false;

        BSplineSurface? bSurf = _sec!.BSplineSurface()?.Copy() as BSplineSurface;
        if (bSurf is null)
        {
            return ok; // Ce mode de construction est impossible
        }

        int nbIntervalC2, nbIntervalC3;
        LocFunction func = new LocFunction(_loc);

        nbIntervalC2 = _loc.NbIntervals(C2);
        nbIntervalC3 = _loc.NbIntervals(C3);
        NCollection_Array1<double> Param_de_decoupeC2 = new(1, nbIntervalC2 + 1);
        _loc.Intervals(Param_de_decoupeC2, C2);
        NCollection_Array1<double> Param_de_decoupeC3 = new(1, nbIntervalC3 + 1);
        _loc.Intervals(Param_de_decoupeC3, C3);

        PrefAndRec preferentiel = new PrefAndRec(Param_de_decoupeC2, Param_de_decoupeC3);

        NCollection_HArray1<double> threeDTol = new NCollection_HArray1<double>(1, 4);
        threeDTol.Init(_tol3d); // A Affiner...

        GeomFill_Sweep_Eval eval = new GeomFill_Sweep_Eval(func);
        ApproxAFunction approx = new ApproxAFunction(0,
                                                                         0,
                                                                         4,
                                                                         threeDTol,
                                                                         threeDTol,
                                                                         threeDTol,
                                                                         _first,
                                                                         _last,
                                                                         continuity,
                                                                         degmax,
                                                                         segmax,
                                                                         eval,
                                                                         preferentiel);

        ok = approx.HasResult;
        if (ok)
        {
            // OCCT: the tensorial product is commented out; the section's surface is kept as it is.
            _surface = bSurf;
        }
        return ok;
    }

    //  Modified by skv - Thu Feb  5 18:05:03 2004 OCC5073 Begin
    //  Conditions:
    //     * theSec should be constant
    //     * the type of section should be a line
    //     * theLoc should represent a translation.
    private static bool IsSweepParallelSpine(SurfaceLocationLaw loc, SurfaceSectionLaw sec, double tol)
    {
        // Get the first and last transformations of the location
        double first;
        double last;
        Vector3D vBegin = new Vector3D();
        Vector3D vEnd = new Vector3D();
        Matrix3x3 m = new Matrix3x3();
        GeneralTransform3D gTfBegin = new GeneralTransform3D();
        Transform3D tfBegin = new Transform3D();
        GeneralTransform3D gTfEnd = new GeneralTransform3D();
        Transform3D tfEnd = new Transform3D();

        loc.GetDomain(out first, out last);

        // Get the first transformation
        loc.D0(first, ref m, ref vBegin);

        gTfBegin.SetVectorialPart(m);
        gTfBegin.SetTranslationPart(vBegin.XYZ());

        tfBegin.SetValues(gTfBegin[1, 1],
                          gTfBegin[1, 2],
                          gTfBegin[1, 3],
                          gTfBegin[1, 4],
                          gTfBegin[2, 1],
                          gTfBegin[2, 2],
                          gTfBegin[2, 3],
                          gTfBegin[2, 4],
                          gTfBegin[3, 1],
                          gTfBegin[3, 2],
                          gTfBegin[3, 3],
                          gTfBegin[3, 4]);

        // Get the last transformation
        loc.D0(last, ref m, ref vEnd);

        gTfEnd.SetVectorialPart(m);
        gTfEnd.SetTranslationPart(vEnd.XYZ());

        tfEnd.SetValues(gTfEnd[1, 1],
                        gTfEnd[1, 2],
                        gTfEnd[1, 3],
                        gTfEnd[1, 4],
                        gTfEnd[2, 1],
                        gTfEnd[2, 2],
                        gTfEnd[2, 3],
                        gTfEnd[2, 4],
                        gTfEnd[3, 1],
                        gTfEnd[3, 2],
                        gTfEnd[3, 3],
                        gTfEnd[3, 4]);

        Surface surf = sec.BSplineSurface()!;
        double umin;
        double umax;
        double vmin;
        double vmax;

        surf.Bounds(out umin, out umax, out vmin, out vmax);

        // Get and transform the first section
        Curve3D firstSection = sec.ConstantSection();
        CurveAdaptor acFirst = new CurveAdaptor(firstSection);

        double uFirst = acFirst.FirstParameter();
        Line3D l = acFirst.Line();

        l.Transform(tfBegin);

        // Get and transform the last section
        Curve3D lastSection = surf.VIso(vmax);
        double firstParameter = lastSection.FirstParameter();
        Point3D pntLastSec = lastSection.Value(firstParameter);

        pntLastSec.Transform(tfEnd);

        Point3D pntFirstSec = global::Kerf.Engine.Geometry.ElCLib.Value(uFirst, l);
        Vector3D vecSec = new Vector3D(pntFirstSec, pntLastSec);
        Vector3D vecSpine = vEnd - vBegin;

        bool isParallel = vecSec.IsParallel(vecSpine, tol);

        return isParallel;
    }

    //  Modified by skv - Thu Feb  5 18:05:01 2004 OCC5073 End

    private bool BuildKPart()
    {
        bool ok = false;
        bool isUPeriodic = false;
        bool isVPeriodic = false;
        bool isTrsf = true;

        isUPeriodic = _sec!.IsUPeriodic;
        Surface? s = null;
        CurveKind sectionType;
        Vector3D v = new Vector3D();
        Matrix3x3 m = new Matrix3x3();
        double levier, error = 0;
        double uFirst = 0, vFirst = _first, uLast = 0, vLast = _last;
        double tol = Min(_tol3d, _boundTol);

        // (1) Trajectoire Rectilignes -------------------------
        if (_loc.IsTranslation(ref error))
        {
            // Donne de la translation
            Vector3D dp, ds = new Vector3D();
            _loc.D0(1, ref m, ref ds);
            _loc.D0(0, ref m, ref v);
            dp = ds - v;
            dp.Normalize();
            GeneralTransform3D tf = new GeneralTransform3D();
            Transform3D tf2 = new Transform3D();
            tf.SetVectorialPart(m);
            tf.SetTranslationPart(v.XYZ());
            try
            { // Pas joli mais il n'y as pas d'autre moyens de tester SetValues
                tf2.SetValues(tf[1, 1],
                              tf[1, 2],
                              tf[1, 3],
                              tf[1, 4],
                              tf[2, 1],
                              tf[2, 2],
                              tf[2, 3],
                              tf[2, 4],
                              tf[3, 1],
                              tf[3, 2],
                              tf[3, 3],
                              tf[3, 4]);
            }
            catch (ConstructionErrorException)
            {
                isTrsf = false;
            }
            if (!isTrsf)
            {
                return false;
            }

            // (1.1) Cas Extrusion
            if (_sec.IsConstant(ref error))
            {
                Curve3D section;
                section = _sec.ConstantSection();
                CurveAdaptor ac = new CurveAdaptor(section);
                sectionType = ac.GetType();
                uFirst = ac.FirstParameter();
                uLast = ac.LastParameter();
                // (1.1.a) Cas Plan
                if ((sectionType == Line) && isTrsf)
                {
                    //  Modified by skv - Thu Feb  5 11:39:06 2004 OCC5073 Begin
                    if (!IsSweepParallelSpine(_loc, _sec, tol))
                    {
                        return false;
                    }
                    //  Modified by skv - Thu Feb  5 11:39:08 2004 OCC5073 End
                    Line3D L = ac.Line();
                    L.Transform(tf2);
                    ds.SetXYZ(L.Position().Direction().XYZ());
                    ds.Normalize();
                    levier = Math.Abs(ds.Dot(dp));
                    _sError = error + levier * Math.Abs(_last - _first);
                    if (_sError <= tol)
                    {
                        ok = true;
                        Frame3D axisOfPlane = new Frame3D(L.Location(), new Direction3D(ds ^ dp), new Direction3D(ds));
                        s = new PlaneSurface(new CoordinateSystem(axisOfPlane));
                    }
                    else
                    {
                        _sError = 0.0;
                    }
                }

                // (1.1.b) Cas Cylindrique
                if ((sectionType == Circle) && isTrsf)
                {
                    const double tolProd = 1.0e-6;

                    Circle3D c = ac.Circle();
                    c.Transform(tf2);

                    ds.SetXYZ(c.Position().Direction().XYZ());
                    ds.Normalize();
                    levier = Math.Abs(ds.CrossMagnitude(dp)) * c.Radius();
                    _sError = levier * Math.Abs(_last - _first);
                    if (_sError <= tolProd)
                    {
                        ok = true;
                        CoordinateSystem axe = new CoordinateSystem(c.Location(), new Direction3D(dp), c.Position().XDirection());
                        s = new CylindricalSurface(axe, c.Radius());
                        if (c.Position().Direction().IsOpposite(axe.Direction(), 0.1))
                        {
                            double f, l;
                            // L'orientation parametrique est inversee
                            l = 2 * M_PI - uFirst;
                            f = 2 * M_PI - uLast;
                            uFirst = f;
                            uLast = l;
                            _isUReversed = true;
                        }
                    }
                    else
                    {
                        _sError = 0.0;
                    }
                }

                // (1.1.c) C'est bien une extrusion
                if (!ok)
                {
                    if (isTrsf)
                    {
                        section.Transform(tf2);
                        s = new ExtrusionSurface(section, new Direction3D(dp));
                        _sError = 0.0;
                        ok = true;
                    }
                    else
                    { // extrusion sur BSpline
                    }
                }
            }

            // (1.2) Cas conique
            else if (_sec.IsConicalLaw(ref error))
            {
                Point3D p1, p2, centre0, centre1, centre2;
                Vector3D dsection;
                Curve3D section;
                CurveAdaptor ac = new CurveAdaptor();
                Circle3D c;
                double r1, r2;

                section = _sec.CirclSection(_sLast);
                section.Transform(tf2);
                section.Translate(_last * dp);
                ac.Load(section);
                c = ac.Circle();
                centre2 = c.Location();
                ac.D1(0, out p2, out dsection);
                r2 = c.Radius();

                section = _sec.CirclSection(_sFirst);
                section.Transform(tf2);
                section.Translate(_first * dp);
                ac.Load(section);
                c = ac.Circle();
                centre1 = c.Location();
                p1 = ac.Value(0);
                r1 = c.Radius();

                section = _sec.CirclSection(_sFirst - _first * (_sLast - _sFirst) / (_last - _first));
                section.Transform(tf2);
                ac.Load(section);
                c = ac.Circle();
                centre0 = c.Location();

                double angle;
                Vector3D n = new Vector3D(centre1, p1);
                if (n.Magnitude() < 1.0e-9)
                {
                    Vector3D bis = new Vector3D(centre2, p2);
                    n = bis;
                }
                Vector3D L = new Vector3D(p1, p2), dir = new Vector3D(centre1, centre2);

                angle = L.Angle(dir);
                if ((angle > 0.01) && (angle < M_PI / 2 - 0.01))
                {
                    if (r2 < r1)
                    {
                        angle = -angle;
                    }
                    _sError = error;
                    CoordinateSystem axis = new CoordinateSystem(centre0, new Direction3D(dir), new Direction3D(n));
                    s = new ConicalSurface(axis, angle, c.Radius());
                    // Calcul du glissement parametrique
                    vFirst = _first / Math.Cos(angle);
                    vLast = _last / Math.Cos(angle);

                    // Bornes en U
                    uFirst = ac.FirstParameter();
                    uLast = ac.LastParameter();
                    s.VIso(vLast).D1(0, out _, out Vector3D diso);
                    if (diso.Magnitude() > 1.0e-9 && dsection.Magnitude() > 1.0e-9)
                    {
                        _isUReversed = diso.IsOpposite(dsection, 0.1);
                    }
                    if (_isUReversed)
                    {
                        double f, l;
                        // L'orientation parametrique est inversee
                        l = 2 * M_PI - uFirst;
                        f = 2 * M_PI - uLast;
                        uFirst = f;
                        uLast = l;
                    }

                    // C'est un cone
                    ok = true;
                }
            }
        }

        // (2) Trajectoire Circulaire
        if (_loc.IsRotation(ref error))
        {
            if (_sec.IsConstant(ref error))
            {
                // La trajectoire
                Point3D centre = new Point3D();
                isVPeriodic = (Math.Abs(_last - _first - 2 * M_PI) < 1.0e-15);
                double rotRadius;
                Vector3D dp, ds = new Vector3D(), dn;
                _loc.D0(0.1, ref m, ref ds);
                _loc.D0(0, ref m, ref v);
                _loc.Rotation(ref centre);

                dp = ds - v;
                ds.SetXYZ(v.XYZ() - centre.XYZ());
                rotRadius = ds.Magnitude();
                if (rotRadius > 1.0e-15)
                {
                    ds.Normalize();
                }
                else
                {
                    return false; // Pas de KPart, rotation degeneree
                }
                dn = ds ^ dp;
                dn.Normalize();
                dp = dn ^ ds;
                dp.Normalize();

                GeneralTransform3D tf = new GeneralTransform3D();
                Transform3D tf2 = new Transform3D();
                tf.SetVectorialPart(m);
                tf.SetTranslationPart(v.XYZ());
                // TODO: finding #9 - exception handling is commented out unlike the translation
                // branch (~600-619). Restoring try/catch caused blend regressions. Needs
                // investigation to understand why the catch was intentionally disabled.
                tf2.SetValues(tf[1, 1],
                              tf[1, 2],
                              tf[1, 3],
                              tf[1, 4],
                              tf[2, 1],
                              tf[2, 2],
                              tf[2, 3],
                              tf[2, 4],
                              tf[3, 1],
                              tf[3, 2],
                              tf[3, 3],
                              tf[3, 4]);
                // La section
                Curve3D section;
                section = _sec.ConstantSection();
                CurveAdaptor ac = new CurveAdaptor(section);
                sectionType = ac.GetType();
                uFirst = ac.FirstParameter();
                uLast = ac.LastParameter();

                // (2.1) Tore/Sphere ?
                if ((sectionType == Circle) && isTrsf)
                {
                    Circle3D c = ac.Circle();
                    double Radius;
                    bool isGoodSide = true;
                    c.Transform(tf2);
                    Vector3D dc = new Vector3D();
                    // On calcul le centre eventuel
                    dc.SetXYZ(c.Location().XYZ() - centre.XYZ());
                    centre.ChangeCoord() += (dc.Dot(dn)) * dn.XYZ();
                    dc.SetXYZ(c.Location().XYZ() - centre.XYZ());
                    Radius = dc.Magnitude(); // grand Rayon du tore
                    if ((Radius > tol) && (dc.Dot(ds) < 0))
                    {
                        isGoodSide = false;
                    }
                    if (Radius < tol / 100)
                    {
                        dc = ds; // Pour definir le tore
                    }

                    // On verifie d'abord que le plan de la section est // a
                    // l'axe de rotation
                    Vector3D nc = new Vector3D();
                    nc.SetXYZ(c.Position().Direction().XYZ());
                    nc.Normalize();
                    error = Math.Abs(nc.Dot(dn));
                    // Puis on evalue l'erreur commise sur la section,
                    // en pivotant son plan ( pour contenir l'axe de rotation)
                    error += Math.Abs(nc.Dot(ds));
                    error *= c.Radius();
                    if (error <= tol)
                    {
                        _sError = error;
                        error += Radius;
                        if (Radius <= tol)
                        {
                            // (2.1.a) Sphere
                            double f = uFirst, l = uLast, aRadius = 0.0;
                            _sError = error;
                            centre.BaryCenter(1.0, c.Location(), 1.0);
                            CoordinateSystem axisOfSphere = new CoordinateSystem(centre, new Direction3D(dn), new Direction3D(ds));
                            aRadius = c.Radius();
                            Sphere theSphere = new Sphere(axisOfSphere, aRadius);
                            s = new SphericalSurface(theSphere);
                            // Pour les spheres on ne peut pas controler le parametre
                            // V (donc U car  myExchUV = true)
                            // Il faut donc modifier UFirst, ULast...
                            double fpar = ac.FirstParameter();
                            double lpar = ac.LastParameter();
                            Curve3D theSection = new TrimmedCurve3D(section, fpar, lpar);
                            theSection.Transform(tf2);
                            Point3D firstPoint = theSection.Value(theSection.FirstParameter());
                            Point3D lastPoint = theSection.Value(theSection.LastParameter());
                            double ufirstOnSec, vfirstOnSec, ulastOnSec, vlastOnSec;
                            global::Kerf.Engine.Geometry.ElSLib.Parameters(theSphere, firstPoint, out ufirstOnSec, out vfirstOnSec);
                            global::Kerf.Engine.Geometry.ElSLib.Parameters(theSphere, lastPoint, out ulastOnSec, out vlastOnSec);
                            if (vfirstOnSec < vlastOnSec)
                            {
                                f = vfirstOnSec;
                                l = vlastOnSec;
                            }
                            else
                            {
                                // L'orientation parametrique est inversee
                                f = vlastOnSec;
                                l = vfirstOnSec;
                                _isUReversed = true;
                            }

                            if (Math.Abs(l - f) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || Math.Abs(ulastOnSec - ufirstOnSec) > M_PI_2)
                            {
                                // l == f - "degenerated" surface
                                // UlastOnSec - UfirstOnSec > M_PI_2 - "twisted" surface,
                                // it is impossible to represent with help of trimmed sphere
                                _isUReversed = false;
                                return ok;
                            }

                            if ((f >= -M_PI / 2) && (l <= M_PI / 2))
                            {
                                ok = true;
                                _exchUV = true;
                                uFirst = f;
                                uLast = l;
                            }
                            else
                            { // On restaure ce qu'il faut
                                _isUReversed = false;
                            }
                        }
                        else if (isGoodSide)
                        {
                            // (2.1.b) Tore
                            CoordinateSystem axisOfTore = new CoordinateSystem(centre, new Direction3D(dn), new Direction3D(dc));
                            s = new ToroidalSurface(axisOfTore, Radius, c.Radius());

                            // Pour les tores on ne peut pas controler le parametre
                            // V (donc U car  myExchUV = true)
                            // Il faut donc modifier UFirst, ULast...
                            CircleCurve3D? iso;
                            iso = s.UIso(0.0) as CircleCurve3D;
                            Frame3D axeiso;
                            axeiso = iso!.Circ().Position();

                            if (c.Position().Direction().IsOpposite(axeiso.Direction(), 0.1))
                            {
                                double f, l;
                                // L'orientation parametrique est inversee
                                l = 2 * M_PI - uFirst;
                                f = 2 * M_PI - uLast;
                                uFirst = f;
                                uLast = l;
                                _isUReversed = true;
                            }
                            // On calcul le "glissement" parametrique.
                            double rot;
                            rot = c.Position().XDirection().AngleWithRef(axeiso.XDirection(), axeiso.Direction());
                            uFirst -= rot;
                            uLast -= rot;

                            _exchUV = true;
                            // Attention l'arete de couture dans le cas periodique
                            // n'est peut etre pas a la bonne place...
                            if (isUPeriodic && Math.Abs(uFirst) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                isUPeriodic = false; // Pour trimmer la surface...
                            }
                            ok = true;
                        }
                    }
                    else
                    {
                        _sError = 0.0;
                    }
                }
                // (2.2) Cone / Cylindre
                if ((sectionType == Line) && isTrsf)
                {
                    Line3D L = ac.Line();
                    L.Transform(tf2);
                    Vector3D dl = new Vector3D();
                    dl.SetXYZ(L.Direction().XYZ());
                    levier = Max(Math.Abs(ac.FirstParameter()), ac.LastParameter());
                    // si la line est ortogonale au cercle de rotation
                    _sError = error + levier * Math.Abs(dl.Dot(dp));
                    if (_sError <= tol)
                    {
                        bool reverse;
                        Line3D dir = new Line3D(centre, new Direction3D(dn));
                        double aux;
                        aux = dl.Dot(dn);
                        reverse = (aux < 0); // On choisit ici le sens de parametrisation

                        // Calcul du centre du vecteur supportant la "XDirection"
                        Point3D centreOfSurf;
                        Vector3D o1O2 = new Vector3D(centre, L.Location()), trans;
                        trans = dn;
                        trans *= dn.Dot(o1O2);
                        centreOfSurf = centre.Translated(trans);
                        ds.SetXYZ(L.Location().XYZ() - centreOfSurf.XYZ());

                        error = _sError;
                        error += (dl.XYZ()).CrossMagnitude(dn.XYZ()) * levier;
                        if (error <= tol)
                        {
                            // (2.2.a) Cylindre
                            // si la line est orthogonale au plan de rotation
                            _sError = error;
                            //
                            CoordinateSystem axis = new CoordinateSystem(centreOfSurf, dir.Direction());
                            if (ds.SquareMagnitude() > Constants.Resolution())
                            {
                                axis.SetXDirection(new Direction3D(ds));
                            }
                            s = new CylindricalSurface(axis, L.Distance(centreOfSurf));
                            ok = true;
                            _exchUV = true;
                        }
                        else
                        {
                            // On evalue l'angle du cone
                            double angle = Math.Abs(dir.Angle(L));
                            if (angle > M_PI / 2)
                            {
                                angle = M_PI - angle;
                            }
                            if (reverse)
                            {
                                angle = -angle;
                            }
                            aux = ds.Dot(dl);
                            if (aux < 0)
                            {
                                angle = -angle;
                            }
                            if (Math.Abs(Math.Abs(angle) - M_PI / 2) > 0.01)
                            {
                                // (2.2.b) Cone
                                // si les 2 droites ne sont pas orthogonales
                                double radius = centreOfSurf.Distance(L.Location());
                                CoordinateSystem axis = new CoordinateSystem(centreOfSurf, dir.Direction(), new Direction3D(ds));
                                s = new ConicalSurface(axis, angle, radius);
                                _exchUV = true;
                                ok = true;
                            }
                            else
                            {
                                // On n'as pas conclue, on remet l'erreur a 0.
                                _sError = 0.0;
                            }
                        }
                        if (ok && reverse)
                        {
                            // On reverse le parametre
                            double uf, ul;
                            LineCurve3D cl = new LineCurve3D(L);
                            uf = cl.ReversedParameter(uLast);
                            ul = cl.ReversedParameter(uFirst);

                            // Following the example of the code for the sphere:
                            // "we cannot control U because myExchUV = true,
                            // so it is necessary to change UFirst and ULast"
                            uFirst = ul;
                            uLast = uf;
                        }
                    }
                    else
                    {
                        _sError = 0.0;
                    }
                }

                // (2.3) Revolution
                if (!ok)
                {
                    if (isTrsf)
                    {
                        section.Transform(tf2);
                        Axis3D axis = new Axis3D(centre, new Direction3D(dn));
                        s = new RevolutionSurface(section, axis);
                        _exchUV = true;
                        _sError = 0.0;
                        ok = true;
                    }
                }
            }
        }

        if (ok)
        { // On trimme la surface
            if (_exchUV)
            {
                bool b;
                b = isUPeriodic;
                isUPeriodic = isVPeriodic;
                isVPeriodic = b;
                double r;
                r = uFirst;
                uFirst = vFirst;
                vFirst = r;
                r = uLast;
                uLast = vLast;
                vLast = r;
            }

            if (!isUPeriodic && !isVPeriodic)
            {
                _surface = new TrimmedSurface(s!, uFirst, uLast, vFirst, vLast);
            }
            else if (isUPeriodic)
            {
                if (isVPeriodic)
                {
                    _surface = s;
                }
                else
                {
                    _surface = new TrimmedSurface(s!, vFirst, vLast, false);
                }
            }
            else
            {
                _surface = new TrimmedSurface(s!, uFirst, uLast, true);
            }
        }

        return ok;
    }

    /// <summary>Whether the surface is built.</summary>
    public bool IsDone => _done;

    /// <summary>The approximation error on the surface.</summary>
    public double ErrorOnSurface() => _sError;

    /// <summary>The approximation errors of the first or last restriction.</summary>
    public void ErrorOnRestriction(bool isFirst, out double uError, out double vError)
    {
        int ind;
        if (isFirst)
        {
            ind = 1;
        }
        else
        {
            ind = _curve2d!.Length();
        }

        uError = _cError!.Value(1, ind);
        vError = _cError.Value(2, ind);
    }

    /// <summary>The approximation errors of the trace IndexOfTrace.</summary>
    public void ErrorOnTrace(int indexOfTrace, out double uError, out double vError)
    {
        int ind = indexOfTrace + 1;
        if (indexOfTrace > _loc.TraceNumber())
        {
            throw new OutOfRangeException(" GeomFill_Sweep::ErrorOnTrace");
        }

        uError = _cError!.Value(1, ind);
        vError = _cError.Value(2, ind);
    }

    /// <summary>The swept surface (frozen), or null before a successful <see cref="Build"/>.</summary>
    public Surface? Surface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary>The swept surface, not frozen, for ported callers.</summary>
    internal Surface? InternalSurface() => _surface;

    /// <summary>The first or last restriction, a 2D curve on the surface (frozen).</summary>
    public Curve2D? Restriction(bool isFirst)
    {
        Curve2D? c = InternalRestriction(isFirst);
        c?.Freeze();
        return c;
    }

    /// <summary>The first or last restriction, not frozen, for ported callers.</summary>
    internal Curve2D? InternalRestriction(bool isFirst)
    {
        if (isFirst)
        {
            return _curve2d!.Value(1);
        }
        return _curve2d!.Value(_curve2d.Length());
    }

    /// <summary>The number of traces of the location law.</summary>
    public int NumberOfTrace() => _loc.TraceNumber();

    /// <summary>The trace IndexOfTrace, a 2D curve on the surface (frozen).</summary>
    public Curve2D? Trace(int indexOfTrace)
    {
        Curve2D? c = InternalTrace(indexOfTrace);
        c?.Freeze();
        return c;
    }

    /// <summary>The trace IndexOfTrace, not frozen, for ported callers.</summary>
    internal Curve2D? InternalTrace(int indexOfTrace)
    {
        int ind = indexOfTrace + 1;
        if (indexOfTrace > _loc.TraceNumber())
        {
            throw new OutOfRangeException(" GeomFill_Sweep::Trace");
        }
        return _curve2d!.Value(ind);
    }
}
