// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_Approx.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.ParametrizationType;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// The static tool over a parametric surface that ApproxInt's generics use (OCCT's ThePSurfaceTool macro:
/// Adaptor3d_HSurfaceTool in GeomInt, BRepApprox_SurfaceTool in BRepApprox): IntImp's members, the surface's type
/// and its quadric, and its periods.
/// </summary>
/// <remarks>
/// Not an OCCT type (decision 607). Over Adaptor3d_Surface it is implemented by
/// <see cref="TheHSurfaceTool"/>, over BRepAdaptor_Surface by <see cref="ApproximationSurfaceTool"/>.
/// </remarks>
/// <typeparam name="ThePSurface">The surface.</typeparam>
internal interface IApproxSurfaceTool<ThePSurface> : IZerCSParPSurfaceTool<ThePSurface>
{
    static abstract SurfaceKind GetType(in ThePSurface s);

    static abstract Plane3D Plane(in ThePSurface s);

    static abstract Cylinder3D Cylinder(in ThePSurface s);

    static abstract Sphere3D Sphere(in ThePSurface s);

    static abstract Cone3D Cone(in ThePSurface s);

    static abstract bool IsUPeriodic(in ThePSurface s);

    static abstract double UPeriod(in ThePSurface s);

    static abstract bool IsVPeriodic(in ThePSurface s);

    static abstract double VPeriod(in ThePSurface s);
}

/// <summary>
/// Approximates (part of) a walking line, the intersection of two surfaces, by a B-spline curve in 3D and B-spline
/// curves in the parameter spaces of the surfaces: Bezier pieces between knots found on the discrete curvature
/// (<see cref="KnotTools"/>), joined into one B-spline, or one B-spline at once for short lines. The
/// points the approximation needs are added on the surfaces (quadric/parametric or parametric/parametric).
/// </summary>
/// <remarks>
/// <para>
/// OCCT's generic file ApproxInt_Approx.gxx, ported once as a generic class over the parametric surface, its tool and
/// the walking line (decision 607); OCCT's instantiations are GeomInt_WLApprox and BRepApprox_Approx. The macros
/// that are the same in every instantiation are named in the body: TheISurface and TheISurfaceTool (IntSurf_Quadric
/// and IntSurf_QuadricTool); the SvSurfaces, the MultiLine and its tool (ApproxInt's generics over the same
/// surface, tool and line); TheComputeLine and TheComputeLineBezier (Approx_BSplComputeLine and Approx_ComputeLine
/// over that MultiLine and tool, decision 202).
/// </para>
/// <para>
/// A working object (decision 21). <see cref="Value"/> returns the approximation's own MultiBSpCurve (decision 204;
/// shared: don't modify). OCCT declares a <c>Perform()</c> without arguments that it never defines: left out.
/// </para>
/// </remarks>
internal class Approx<ThePSurface, ThePSurfaceTool, TheWLine>
    where ThePSurfaceTool : IApproxSurfaceTool<ThePSurface>
    where TheWLine : class, IWLine<TheWLine>
{
    // If quantity of points is less than aMinNbPointsForApprox
    // then interpolation is used.
    private const int AMinNbPointsForApprox = 5;

    // This constant should be removed in the future.
    private const double RatioTol = 1.5;

    private struct Approx_Data
    {
        public Approx_Data()
        {
            BezierApprox = true;
            Xo = 0.0;
            Yo = 0.0;
            Zo = 0.0;
            U1o = 0.0;
            V1o = 0.0;
            U2o = 0.0;
            V2o = 0.0;
            ApproxXYZ = true;
            ApproxU1V1 = true;
            ApproxU2V2 = true;
            indicemin = 0;
            indicemax = 0;
            NbPntMax = 30;
            parametrization = ChordLength;
        }

        public bool BezierApprox;
        public double Xo, Yo, Zo, U1o, V1o, U2o, V2o;
        public bool ApproxXYZ, ApproxU1V1, ApproxU2V2;
        public int indicemin, indicemax, NbPntMax;
        public global::Kerf.Geometry.ParametrizationType parametrization;
    }

    private readonly BSplComputeLine<global::Kerf.Engine.Intersection.Approximation.MultiLine<TheWLine>, MultiLineTool<TheWLine>>
        _computeLine;
    private readonly ComputeLine<global::Kerf.Engine.Intersection.Approximation.MultiLine<TheWLine>, MultiLineTool<TheWLine>>
        _computeLineBezier;
    private readonly MCurvesToBSpCurve _bezToBSpl;
    private bool _withTangency;
    private double _tol3d;
    private double _tol2d;
    private int _degMin;
    private int _degMax;
    private int _nbIterMax;
    private double _tolReached3d;
    private double _tolReached2d;
    private Approx_Data _data;
    private readonly List<int> _knots;

    //=================================================================================================

    private static void ComputeTrsf3d(TheWLine theline, out double xo, out double yo, out double zo)
    {
        int nbPnts = theline.NbPnts();
        double xmin = RealLast(), ymin = RealLast(), zmin = RealLast();
        for (int i = 1; i <= nbPnts; i++)
        {
            Point3D p = theline.Point(i).Value();
            xmin = Min(p.X, xmin);
            ymin = Min(p.Y, ymin);
            zmin = Min(p.Z, zmin);
        }

        xo = -xmin;
        yo = -ymin;
        zo = -zmin;
    }

    //=================================================================================================

    private static void ComputeTrsf2d(TheWLine theline, bool onFirst, out double uo, out double vo)
    {
        int nbPnts = theline.NbPnts();
        double umin = RealLast(), vmin = RealLast();

        for (int i = 1; i <= nbPnts; i++)
        {
            PntOn2S pOn2S = theline.Point(i);
            double u, v;
            if (onFirst)
            {
                pOn2S.ParametersOnS1(out u, out v);
            }
            else
            {
                pOn2S.ParametersOnS2(out u, out v);
            }
            umin = Min(u, umin);
            vmin = Min(v, vmin);
        }

        uo = -umin;
        vo = -vmin;
    }

    /// <summary>
    /// The parameters of the points <paramref name="firstP"/> to <paramref name="lastP"/> of <paramref name="line"/>
    /// for the parametrisation <paramref name="par"/>, from 0 to 1, in <paramref name="theParameters"/>.
    /// </summary>
    public static void Parameters(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheWLine> line, int firstP, int lastP,
                                  global::Kerf.Geometry.ParametrizationType par, math_Vector theParameters)
    {
        int i, j, nbP2d, nbP3d;
        double dist;

        if (par == ChordLength || par == Centripetal)
        {
            nbP3d = MultiLineTool<TheWLine>.NbP3d(line);
            nbP2d = MultiLineTool<TheWLine>.NbP2d(line);
            int mynbP3d = nbP3d, mynbP2d = nbP2d;
            if (nbP3d == 0)
            {
                mynbP3d = 1;
            }
            if (nbP2d == 0)
            {
                mynbP2d = 1;
            }

            theParameters[firstP] = 0.0;
            dist = 0.0;
            NCollection_Array1<Point3D> tabP = new(1, mynbP3d);
            NCollection_Array1<Point3D> tabPP = new(1, mynbP3d);
            NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d);
            NCollection_Array1<Point2D> tabPP2d = new(1, mynbP2d);

            for (i = firstP + 1; i <= lastP; i++)
            {
                if (nbP3d != 0 && nbP2d != 0)
                {
                    MultiLineTool<TheWLine>.Value(line, i - 1, tabP, tabP2d);
                }
                else if (nbP2d != 0)
                {
                    MultiLineTool<TheWLine>.Value(line, i - 1, tabP2d);
                }
                else if (nbP3d != 0)
                {
                    MultiLineTool<TheWLine>.Value(line, i - 1, tabP);
                }

                if (nbP3d != 0 && nbP2d != 0)
                {
                    MultiLineTool<TheWLine>.Value(line, i, tabPP, tabPP2d);
                }
                else if (nbP2d != 0)
                {
                    MultiLineTool<TheWLine>.Value(line, i, tabPP2d);
                }
                else if (nbP3d != 0)
                {
                    MultiLineTool<TheWLine>.Value(line, i, tabPP);
                }
                dist = 0;
                for (j = 1; j <= nbP3d; j++)
                {
                    Point3D p1 = tabP[j], p2 = tabPP[j];
                    dist += p2.SquareDistanceTo(p1);
                }
                for (j = 1; j <= nbP2d; j++)
                {
                    Point2D p12d = tabP2d[j], p22d = tabPP2d[j];

                    dist += p22d.SquareDistanceTo(p12d);
                }

                dist = Math.Sqrt(dist);
                if (par == ChordLength)
                {
                    theParameters[i] = theParameters[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    theParameters[i] = theParameters[i - 1] + Math.Sqrt(dist);
                }
            }
            for (i = firstP; i <= lastP; i++)
            {
                theParameters[i] /= theParameters[lastP];
            }
        }
        else
        {
            for (i = firstP; i <= lastP; i++)
            {
                theParameters[i] = ((double)i - firstP) / ((double)lastP - (double)firstP);
            }
        }
    }

    //=================================================================================================

    public Approx()
    {
        _computeLine = new(4, 8, 0.001, 0.001, 5);
        _computeLineBezier = new(4, 8, 0.001, 0.001, 5);
        _bezToBSpl = new();
        _withTangency = true;
        _tol3d = 0.001;
        _tol2d = 0.001;
        _degMin = 4;
        _degMax = 8;
        _nbIterMax = 5;
        _tolReached3d = 0.0;
        _tolReached2d = 0.0;
        _data = new Approx_Data();
        _knots = new();
        _computeLine.SetContinuity(2);
        // myComputeLineBezier.SetContinuity(2);
    }

    /// <summary>
    /// Approximates the points <paramref name="indicemin"/> to <paramref name="indicemax"/> of the line alone,
    /// without the surfaces (no points are added): in 3D if <paramref name="approxXYZ"/>, and on the surfaces'
    /// parameter spaces if <paramref name="approxU1V1"/> and <paramref name="approxU2V2"/>.
    /// </summary>
    public void Perform(TheWLine theline, bool approxXYZ = true, bool approxU1V1 = true, bool approxU2V2 = true,
                        int indicemin = 0, int indicemax = 0)
    {
        // Prepare DS.
        PrepareDS(approxXYZ, approxU1V1, approxU2V2, indicemin, indicemax);

        int nbpntbez = _data.indicemax - _data.indicemin;
        if (nbpntbez < AMinNbPointsForApprox)
        {
            _data.BezierApprox = false;
        }
        else
        {
            _data.BezierApprox = true;
        }

        // Fill data structure.
        FillData(theline);

        // Build knots.
        BuildKnots(theline, null);
        if (_knots.Count == 2 && indicemax - indicemin > 2 * _data.NbPntMax)
        {
            // At least 3 knots for BrepApprox.
            _knots[^1] = (indicemax - indicemin) / 2; // OCCT: an index count, not indicemin + half of it
            _knots.Add(indicemax);
        }

        _computeLine.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, true, _data.parametrization);
        _computeLineBezier.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, true, _data.parametrization);

        BuildCurve(theline, null);
    }

    //=================================================================================================
    // function : Perform
    // purpose  : Definition of next steps according to surface types
    //            (i.e. coordination algorithm).
    //=================================================================================================

    /// <summary>
    /// Approximates the points <paramref name="indicemin"/> to <paramref name="indicemax"/> of the intersection
    /// line of <paramref name="surf1"/> and <paramref name="surf2"/>, adding points on the surfaces where needed:
    /// in 3D if <paramref name="approxXYZ"/>, and on the surfaces' parameter spaces if <paramref name="approxU1V1"/>
    /// and <paramref name="approxU2V2"/>.
    /// </summary>
    public void Perform(ThePSurface surf1, ThePSurface surf2, TheWLine theline, bool approxXYZ = true,
                        bool approxU1V1 = true, bool approxU2V2 = true, int indicemin = 0, int indicemax = 0)
    {
        _tolReached3d = _tolReached2d = 0.0;

        SurfaceKind typeS1 = ThePSurfaceTool.GetType(surf1);
        SurfaceKind typeS2 = ThePSurfaceTool.GetType(surf2);

        bool isQuadric = ((typeS1 == Plane) || (typeS1 == SurfaceKind.Cylinder) || (typeS1 == SurfaceKind.Sphere)
                          || (typeS1 == SurfaceKind.Cone) || (typeS2 == Plane) || (typeS2 == SurfaceKind.Cylinder)
                          || (typeS2 == SurfaceKind.Sphere) || (typeS2 == SurfaceKind.Cone));

        if (isQuadric)
        {
            SurfaceQuadric quad = new();
            bool secondIsImplicit = false;
            switch (typeS1)
            {
                case Plane:
                    quad.SetValue(ThePSurfaceTool.Plane(surf1));
                    break;

                case SurfaceKind.Cylinder:
                    quad.SetValue(ThePSurfaceTool.Cylinder(surf1));
                    break;

                case SurfaceKind.Sphere:
                    quad.SetValue(ThePSurfaceTool.Sphere(surf1));
                    break;

                case SurfaceKind.Cone:
                    quad.SetValue(ThePSurfaceTool.Cone(surf1));
                    break;

                default:
                {
                    secondIsImplicit = true;
                    switch (typeS2)
                    {
                        case Plane:
                            quad.SetValue(ThePSurfaceTool.Plane(surf2));
                            break;

                        case SurfaceKind.Cylinder:
                            quad.SetValue(ThePSurfaceTool.Cylinder(surf2));
                            break;

                        case SurfaceKind.Sphere:
                            quad.SetValue(ThePSurfaceTool.Sphere(surf2));
                            break;

                        case SurfaceKind.Cone:
                            quad.SetValue(ThePSurfaceTool.Cone(surf2));
                            break;

                        default:
                            break;
                    } // switch (typeS2)
                }

                    break;
            } // switch (typeS1)

            Perform(quad, (secondIsImplicit ? surf1 : surf2), theline, approxXYZ, approxU1V1, approxU2V2, indicemin,
                    indicemax, !secondIsImplicit);

            return;
        }

        // Here, isQuadric == FALSE.

        // Prepare DS.
        PrepareDS(approxXYZ, approxU1V1, approxU2V2, indicemin, indicemax);

        // Non-analytical case: Param-Param perform.
        PrmPrmSvSurfaces<ThePSurface, ThePSurfaceTool> myPrmPrmSvSurfaces = new(surf1, surf2);

        int nbpntbez = indicemax - indicemin;

        if (nbpntbez < AMinNbPointsForApprox)
        {
            _data.BezierApprox = false;
        }
        else
        {
            _data.BezierApprox = true;
        }

        // Fill data structure.
        FillData(theline);

        bool cut = _data.BezierApprox;
        SvSurfaces ptrsvsurf = myPrmPrmSvSurfaces;

        // Build knots.
        BuildKnots(theline, ptrsvsurf);

        _computeLine.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, cut, _data.parametrization);
        _computeLineBezier.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, cut, _data.parametrization);

        BuildCurve(theline, ptrsvsurf);
    }

    //=================================================================================================
    // function : Perform
    // purpose  : Analytic-Param perform.
    //=================================================================================================

    private void Perform(in SurfaceQuadric iSurf, ThePSurface pSurf, TheWLine theline, bool approxXYZ,
                         bool approxU1V1, bool approxU2V2, int indicemin, int indicemax, bool isTheQuadFirst)
    {
        // Prepare DS.
        PrepareDS(approxXYZ, approxU1V1, approxU2V2, indicemin, indicemax);

        // Non-analytical case: Analytic-Param perform.
        ImpPrmSvSurfaces<ThePSurface, ThePSurfaceTool> myImpPrmSvSurfaces =
            isTheQuadFirst ? new(iSurf, pSurf) : new(pSurf, iSurf);

        myImpPrmSvSurfaces.SetUseSolver(false);

        int nbpntbez = indicemax - indicemin;
        if (nbpntbez < AMinNbPointsForApprox)
        {
            _data.BezierApprox = false;
        }
        else
        {
            _data.BezierApprox = true;
        }

        bool cut = _data.BezierApprox;
        SvSurfaces ptrsvsurf = myImpPrmSvSurfaces;

        // Fill data structure.
        FillData(theline);

        // Build knots.
        BuildKnots(theline, ptrsvsurf);

        _computeLine.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, cut, _data.parametrization);
        _computeLineBezier.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, cut, _data.parametrization);

        BuildCurve(theline, ptrsvsurf);
    }

    /// <summary>
    /// Sets the tolerances (divided by 1.5 internally), the degrees, the number of iterations, the minimal number of
    /// points per knot interval, the tangency constraints at the ends and the parametrisation.
    /// </summary>
    public void SetParameters(double tol3d, double tol2d, int degMin, int degMax, int nbIterMax, int nbPntMax = 30,
                              bool approxWithTangency = true,
                              global::Kerf.Geometry.ParametrizationType parametrization = ChordLength)
    {
        _data.NbPntMax = nbPntMax;
        _withTangency = approxWithTangency;
        _tol3d = tol3d / RatioTol;
        _tol2d = tol2d / RatioTol;
        _degMin = degMin;
        _degMax = degMax;
        _nbIterMax = nbIterMax;

        _computeLine.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, true, parametrization);
        _computeLineBezier.Init(_degMin, _degMax, _tol3d, _tol2d, _nbIterMax, true, parametrization);

        if (!approxWithTangency)
        {
            _computeLine.SetConstraints(PassPoint, PassPoint);
            _computeLineBezier.SetConstraints(PassPoint, PassPoint);
        }

        _data.BezierApprox = true;
    }

    /// <summary>The number of MultiCurves: 1.</summary>
    public int MultiCurveCount => 1;

    //=================================================================================================

    private void UpdateTolReached()
    {
        if (_data.BezierApprox)
        {
            int nbCurves = _computeLineBezier.MultiCurveCount;
            for (int iCur = 1; iCur <= nbCurves; iCur++)
            {
                _computeLineBezier.Error(iCur, out double tol3D, out double tol2D);
                _tolReached3d = Max(_tolReached3d, tol3D);
                _tolReached2d = Max(_tolReached2d, tol2D);
            }
        }
        else
        {
            _computeLine.Error(out _tolReached3d, out _tolReached2d);
        }
    }

    /// <summary>The largest 3D error of the approximation.</summary>
    public double TolReached3d() => _tolReached3d * RatioTol;

    /// <summary>The largest 2D error of the approximation.</summary>
    public double TolReached2d() => _tolReached2d * RatioTol;

    public bool IsDone()
    {
        if (_data.BezierApprox)
        {
            return (_computeLineBezier.MultiCurveCount > 0);
        }
        else
        {
            return (_computeLine.IsToleranceReached);
        }
    }

    /// <summary>The approximation (whatever the index): the 3D curve first, then the 2D curves.</summary>
    public MultiBSpCurve Value(int index)
    {
        if (_data.BezierApprox)
        {
            return (_bezToBSpl.Value());
        }
        else
        {
            return (_computeLine.Value());
        }
    }

    //=================================================================================================

    private void FillData(TheWLine theline)
    {
        if (_data.ApproxXYZ)
        {
            ComputeTrsf3d(theline, out _data.Xo, out _data.Yo, out _data.Zo);
        }
        else
        {
            _data.Xo = _data.Yo = _data.Zo = 0.0;
        }

        if (_data.ApproxU1V1)
        {
            ComputeTrsf2d(theline, true, out _data.U1o, out _data.V1o);
        }
        else
        {
            _data.U1o = _data.V1o = 0.0;
        }

        if (_data.ApproxU2V2)
        {
            ComputeTrsf2d(theline, false, out _data.U2o, out _data.V2o);
        }
        else
        {
            _data.U2o = _data.V2o = 0.0;
        }
    }

    //=================================================================================================

    private void PrepareDS(bool approxXYZ, bool approxU1V1, bool approxU2V2, int indicemin,
                           int indicemax)
    {
        _tolReached3d = _tolReached2d = 0.0;
        _data.ApproxU1V1 = approxU1V1;
        _data.ApproxU2V2 = approxU2V2;
        _data.ApproxXYZ = approxXYZ;
        _data.indicemin = indicemin;
        _data.indicemax = indicemax;
        _data.parametrization = _computeLineBezier.Parametrization();
    }

    //=================================================================================================

    private void BuildKnots(TheWLine theline, SvSurfaces? ptrSVSurf)
    {
        _knots.Clear();
        if (!_data.BezierApprox)
        {
            _knots.Add(_data.indicemin);
            _knots.Add(_data.indicemax);
            return;
        }

        global::Kerf.Engine.Intersection.Approximation.MultiLine<TheWLine> testLine = new(theline, ptrSVSurf, ((_data.ApproxXYZ) ? 1 : 0),
                                                      ((_data.ApproxU1V1) ? 1 : 0) + ((_data.ApproxU2V2) ? 1 : 0),
                                                      _data.ApproxU1V1, _data.ApproxU2V2, _data.Xo, _data.Yo,
                                                      _data.Zo, _data.U1o, _data.V1o, _data.U2o, _data.V2o,
                                                      _data.ApproxU1V1, _data.indicemin, _data.indicemax);

        int nbp3d = testLine.P3dCount, nbp2d = testLine.P2dCount;
        NCollection_Array1<Point3D> tabPnt3d = new(1, Max(1, nbp3d));
        NCollection_Array1<Point2D> tabPnt2d = new(1, Max(1, nbp2d));
        NCollection_Array1<Point3D> pntXYZ = new(_data.indicemin, _data.indicemax);
        NCollection_Array1<Point2D> pntU1V1 = new(_data.indicemin, _data.indicemax);
        NCollection_Array1<Point2D> pntU2V2 = new(_data.indicemin, _data.indicemax);

        for (int i = _data.indicemin; i <= _data.indicemax; ++i)
        {
            if (nbp3d != 0 && nbp2d != 0)
            {
                testLine.Value(i, tabPnt3d, tabPnt2d);
            }
            else if (nbp2d != 0)
            {
                testLine.Value(i, tabPnt2d);
            }
            else if (nbp3d != 0)
            {
                testLine.Value(i, tabPnt3d);
            }
            //
            if (nbp3d > 0)
            {
                pntXYZ[i] = tabPnt3d[1];
            }
            if (nbp2d > 1)
            {
                pntU1V1[i] = tabPnt2d[1];
                pntU2V2[i] = tabPnt2d[2];
            }
            else if (nbp2d > 0)
            {
                if (_data.ApproxU1V1)
                {
                    pntU1V1[i] = tabPnt2d[1];
                }
                else
                {
                    pntU2V2[i] = tabPnt2d[1];
                }
            }
        }

        int minNbPnts = _data.NbPntMax;

        // Expected parametrization.
        math_Vector pars = new(_data.indicemin, _data.indicemax);
        Parameters(testLine, _data.indicemin, _data.indicemax, _data.parametrization, pars);

        KnotTools.BuildKnots(pntXYZ, pntU1V1, pntU2V2, pars, _data.ApproxXYZ, _data.ApproxU1V1,
                                       _data.ApproxU2V2, minNbPnts, _knots);
    }

    //=================================================================================================

    private void BuildCurve(TheWLine theline, SvSurfaces? ptrSVSurf)
    {
        if (_data.BezierApprox)
        {
            _bezToBSpl.Reset();
        }

        int kind = 0;
        int imin = 0, imax = 0;
        bool otherInter = false;
        do
        {
            // Base cycle: iterate over knots.
            imin = _knots[kind];
            imax = _knots[kind + 1];
            global::Kerf.Engine.Intersection.Approximation.MultiLine<TheWLine> myMultiLine = new(theline, ptrSVSurf, ((_data.ApproxXYZ) ? 1 : 0),
                                                            ((_data.ApproxU1V1) ? 1 : 0)
                                                                + ((_data.ApproxU2V2) ? 1 : 0),
                                                            _data.ApproxU1V1, _data.ApproxU2V2, _data.Xo,
                                                            _data.Yo, _data.Zo, _data.U1o, _data.V1o, _data.U2o,
                                                            _data.V2o, _data.ApproxU1V1, imin, imax);

            if (_data.BezierApprox)
            {
                _computeLineBezier.Perform(myMultiLine);
                if (_computeLineBezier.MultiCurveCount == 0)
                {
                    return;
                }
            }
            else
            {
                _computeLine.Perform(myMultiLine);
            }

            UpdateTolReached();

            int indice3d = 1, indice2d1 = 2, indice2d2 = 3;
            if (!_data.ApproxXYZ)
            {
                indice2d1--;
                indice2d2--;
            }
            if (!_data.ApproxU1V1)
            {
                indice2d2--;
            }
            if (_data.ApproxXYZ)
            {
                if (_data.BezierApprox)
                {
                    for (int nbmc = _computeLineBezier.MultiCurveCount; nbmc >= 1; nbmc--)
                    {
                        _computeLineBezier.ChangeValue(nbmc)
                            .Transform(indice3d, -_data.Xo, 1.0, -_data.Yo, 1.0, -_data.Zo, 1.0);
                    }
                }
                else
                {
                    _computeLine.ChangeValue().Transform(indice3d, -_data.Xo, 1.0, -_data.Yo, 1.0, -_data.Zo, 1.0);
                }
            }
            if (_data.ApproxU1V1)
            {
                if (_data.BezierApprox)
                {
                    for (int nbmc = _computeLineBezier.MultiCurveCount; nbmc >= 1; nbmc--)
                    {
                        _computeLineBezier.ChangeValue(nbmc).Transform2d(indice2d1, -_data.U1o, 1.0, -_data.V1o,
                                                                          1.0);
                    }
                }
                else
                {
                    _computeLine.ChangeValue().Transform2d(indice2d1, -_data.U1o, 1.0, -_data.V1o, 1.0);
                }
            }
            if (_data.ApproxU2V2)
            {
                if (_data.BezierApprox)
                {
                    for (int nbmc = _computeLineBezier.MultiCurveCount; nbmc >= 1; nbmc--)
                    {
                        _computeLineBezier.ChangeValue(nbmc).Transform2d(indice2d2, -_data.U2o, 1.0, -_data.V2o,
                                                                          1.0);
                    }
                }
                else
                {
                    _computeLine.ChangeValue().Transform2d(indice2d2, -_data.U2o, 1.0, -_data.V2o, 1.0);
                }
            }

            otherInter = false;
            if (_data.BezierApprox)
            {
                for (int nbmc = 1; nbmc <= _computeLineBezier.MultiCurveCount; nbmc++)
                {
                    _bezToBSpl.Append(_computeLineBezier.Value(nbmc));
                }
                kind++;
                if (kind < (_knots.Count - 1))
                {
                    otherInter = true;
                }
            }
        } while (otherInter);

        if (_data.BezierApprox)
        {
            _bezToBSpl.Perform();
        }
    }
}
