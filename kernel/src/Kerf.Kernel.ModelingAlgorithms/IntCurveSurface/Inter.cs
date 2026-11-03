// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_Inter.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The static tool over a curve that the generic curve/surface intersection uses (OCCT's CurveTool template
/// argument: IntCurveSurface_TheHCurveTool, HLRBRep_LineTool).
/// </summary>
public interface IIntersectionCurveTool<TheCurve> : IZerCSParCurveTool<TheCurve>
{
    static abstract CurveKind GetType(in TheCurve c);

    static abstract Line3D Line(in TheCurve c);

    static abstract Circle3D Circle(in TheCurve c);

    static abstract Ellipse3D Ellipse(in TheCurve c);

    static abstract Parabola3D Parabola(in TheCurve c);

    static abstract Hyperbola3D Hyperbola(in TheCurve c);

    static abstract int NbIntervals(in TheCurve c, Continuity s);

    static abstract void Intervals(in TheCurve c, NCollection_Array1<double> t, Continuity s);

    static abstract bool IsPeriodic(in TheCurve c);

    static abstract double Period(in TheCurve c);

    static abstract void D0(in TheCurve c, double u, out Point3D p);

    static abstract int NbSamples(in TheCurve c, double u0, double u1);

    static abstract NCollection_HArray1<double> SamplePars(in TheCurve c, double u0, double u1, double defl,
                                                           int nbMin);
}

/// <summary>
/// The static tool over a surface that the generic curve/surface intersection uses (OCCT's SurfaceTool template
/// argument: Adaptor3d_HSurfaceTool, through <see cref="TheHSurfaceTool"/>; HLRBRep_SurfaceTool).
/// </summary>
public interface IIntersectionSurfaceTool<TheSurface> : IZerCSParPSurfaceTool<TheSurface>
{
    static abstract SurfaceKind GetType(in TheSurface s);

    static abstract Plane3D Plane(in TheSurface s);

    static abstract Cylinder Cylinder(in TheSurface s);

    static abstract Cone Cone(in TheSurface s);

    static abstract Sphere Sphere(in TheSurface s);

    static abstract Torus Torus(in TheSurface s);

    static abstract int NbSamplesU(in TheSurface s, double u1, double u2);

    static abstract int NbSamplesV(in TheSurface s, double v1, double v2);

    static abstract int NbUIntervals(in TheSurface s, Continuity sh);

    static abstract int NbVIntervals(in TheSurface s, Continuity sh);

    static abstract void UIntervals(in TheSurface s, NCollection_Array1<double> t, Continuity sh);

    static abstract void VIntervals(in TheSurface s, NCollection_Array1<double> t, Continuity sh);

    static abstract SurfaceAdaptorBase UTrim(in TheSurface s, double first, double last, double tol);

    static abstract bool IsUClosed(in TheSurface s);

    static abstract bool IsVClosed(in TheSurface s);

    static abstract bool IsUPeriodic(in TheSurface s);

    static abstract bool IsVPeriodic(in TheSurface s);

    static abstract double UPeriod(in TheSurface s);

    static abstract double VPeriod(in TheSurface s);

    static abstract void D0(in TheSurface s, double u, double v, out Point3D p);

    static abstract SurfaceAdaptorBase BasisSurface(in TheSurface s);

    static abstract CurveAdaptorBase BasisCurve(in TheSurface s);

    static abstract Direction3D Direction(in TheSurface s);

    static abstract Axis3D AxeOfRevolution(in TheSurface s);

    static abstract double OffsetValue(in TheSurface s);
}

/// <summary>What the intersection reads from the polygon of a curve: its parameter range and parameters on the curve.</summary>
public interface IPolygon
{
    double InfParameter();

    double SupParameter();

    /// <summary>The parameter on the curve of the point at <paramref name="paramOnLine"/> (0 to 1) along the segment <paramref name="index"/>.</summary>
    double ApproxParamOnCurve(int index, double paramOnLine);
}

/// <summary>The polygon type of the generic intersection, with OCCT's constructors (IntCurveSurface_ThePolygonOfHInter).</summary>
public interface IThePolygon<TSelf, TheCurve> : IPolygon
    where TSelf : IThePolygon<TSelf, TheCurve>
{
    static abstract TSelf Create(in TheCurve curve, int nbPnt);

    static abstract TSelf Create(in TheCurve curve, double u1, double u2, int nbPnt);

    static abstract TSelf Create(in TheCurve curve, NCollection_Array1<double> upars);
}

/// <summary>What the intersection reads from the polyhedron of a surface: its box, triangles, points and parameters.</summary>
public interface IPolyhedron
{
    Box Bounding();

    void Triangle(int index, out int p1, out int p2, out int p3);

    Point3D Point(int index);

    Point3D Point(int index, out double u, out double v);

    void Parameters(int index, out double u, out double v);
}

/// <summary>The polyhedron type of the generic intersection, with OCCT's constructors (IntCurveSurface_ThePolyhedronOfHInter).</summary>
public interface IThePolyhedron<TSelf, TheSurface> : IPolyhedron
    where TSelf : IThePolyhedron<TSelf, TheSurface>
{
    static abstract TSelf Create(in TheSurface surface, int nbdU, int nbdV, double u1, double v1, double u2,
                                 double v2);

    static abstract TSelf Create(in TheSurface surface, NCollection_Array1<double> upars,
                                 NCollection_Array1<double> vpars);
}

/// <summary>The interference type of the generic intersection, with OCCT's constructors (IntCurveSurface_TheInterferenceOfHInter).</summary>
public interface ITheInterference<TSelf, ThePolygon, ThePolyhedron>
    where TSelf : ITheInterference<TSelf, ThePolygon, ThePolyhedron>
{
    static abstract TSelf Create(ThePolygon polyg, ThePolyhedron polyh);

    static abstract TSelf Create(ThePolygon polyg, ThePolyhedron polyh, BoundSortBox boundSB);
}

/// <summary>The curve/surface function of the generic intersection, with OCCT's constructor (IntCurveSurface_TheCSFunctionOfHInter).</summary>
public interface ITheCSFunction<TSelf, TheSurface, TheCurve>
    where TSelf : ITheCSFunction<TSelf, TheSurface, TheCurve>
{
    static abstract TSelf Create(in TheSurface s, in TheCurve c);
}

/// <summary>The exact intersection of the generic intersection, with OCCT's constructor (IntCurveSurface_TheExactHInter).</summary>
public interface ITheExactInter<TSelf, TheCSFunction>
    where TSelf : ITheExactInter<TSelf, TheCSFunction>
{
    static abstract TSelf Create(TheCSFunction f, double tolTangency);
}

/// <summary>The exact curve/quadric intersection of the generic intersection (IntCurveSurface_TheQuadCurvExactHInter).</summary>
public interface ITheQuadCurvExactInter<TSelf, TheSurface, TheCurve>
    where TSelf : ITheQuadCurvExactInter<TSelf, TheSurface, TheCurve>
{
    static abstract TSelf Create(in TheSurface s, in TheCurve c);

    bool IsDone();

    int NbRoots();

    double Root(int index);
}

/// <summary>
/// The function of a quadric along a curve, with OCCT's constructor (IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter).
/// </summary>
public interface ITheQuadCurvFunc<TSelf, TheCurve>
    where TSelf : ITheQuadCurvFunc<TSelf, TheCurve>
{
    static abstract TSelf Create(in SurfaceQuadric q, in TheCurve c);
}

/// <summary>
/// The intersection of a curve and a surface: analytic for a conic and an elementary surface (IntAna), by the roots
/// of the quadric's equation for other curves on quadrics, and otherwise from the interference of a polygon of the
/// curve and a polyhedron of the surface, refined by Newton's method.
/// </summary>
/// <remarks>
/// OCCT's template functions of IntCurveSurface_Inter.pxx (namespace IntCurveSurface_InterImpl), ported as a generic
/// class whose methods are OCCT's IntCurveSurface_HInter's (decision 337). OCCT passes each function lambdas that
/// call back the instantiating class (<c>[this]() { this-&gt;ResetFields(); }</c>, <c>this-&gt;PerformConicSurf</c>,
/// <c>this-&gt;Append</c>, ...); here the bodies call those methods directly. OCCT's instantiations
/// (IntCurveSurface_HInter, and HLRBRep_InterCSurf later) are sealed subclasses. OCCT's private DoSurface and
/// DoNewBounds of IntCurveSurface_HInter, never called, are left out.
/// </remarks>
public class Inter<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool, ThePolygon, ThePolyhedron,
                                   TheInterference, TheCSFunction, TheExactInter, TheQuadCurvExactInter>
    : IntersectionCurveSurfaceIntersection
    where TheCurveTool : IIntersectionCurveTool<TheCurve>
    where TheSurfaceTool : IIntersectionSurfaceTool<TheSurface>
    where ThePolygon : IThePolygon<ThePolygon, TheCurve>
    where ThePolyhedron : IThePolyhedron<ThePolyhedron, TheSurface>
    where TheInterference : PolygonInterference, ITheInterference<TheInterference, ThePolygon, ThePolyhedron>
    where TheCSFunction : FunctionSetWithDerivatives, ICSFunction<TheSurface, TheCurve>,
    ITheCSFunction<TheCSFunction, TheSurface, TheCurve>
    where TheExactInter : IntCS<TheCSFunction, TheSurface, TheSurfaceTool, TheCurve, TheCurveTool>,
    ITheExactInter<TheExactInter, TheCSFunction>
    where TheQuadCurvExactInter : ITheQuadCurvExactInter<TheQuadCurvExactInter, TheSurface, TheCurve>
{
    private const double THE_TOLTANGENCY = 0.00000001;
    private const double THE_TOLERANCE_ANGULAIRE = 1.0e-12;
    private const double THE_TOLERANCE = 0.00000001;
    private const int THE_NBSAMPLESONCIRCLE = 32;
    private const int THE_NBSAMPLESONELLIPSE = 32;
    private const int THE_NBSAMPLESONPARAB = 16;
    private const int THE_NBSAMPLESONHYPR = 32;

    /// <summary>
    /// Computes the intersection between the curve and the surface, on each C2 interval of the surface
    /// (IntCurveSurface_InterImpl::Perform).
    /// </summary>
    public void Perform(in TheCurve curve, in TheSurface surface)
    {
        ResetFields();
        _done = true;

        NCollection_DynamicArray<InterUtils.UVBounds> intervals = new();
        InterUtils.DecomposeSurfaceIntervals<TheSurface, TheSurfaceTool>(surface, intervals);

        foreach (InterUtils.UVBounds bounds in intervals)
        {
            Perform(curve, surface, bounds.U0, bounds.V0, bounds.U1, bounds.V1);
        }
    }

    /// <summary>
    /// Computes the intersection between the curve (whose polygon is given) and the surface; the polyhedron of the
    /// surface is built here (IntCurveSurface_InterImpl::PerformPolygon).
    /// </summary>
    public void Perform(in TheCurve curve, ThePolygon polygon, in TheSurface surface)
    {
        ResetFields();
        _done = true;
        double u1 = TheSurfaceTool.FirstUParameter(surface);
        double v1 = TheSurfaceTool.FirstVParameter(surface);
        double u2 = TheSurfaceTool.LastUParameter(surface);
        double v2 = TheSurfaceTool.LastVParameter(surface);
        int nbsu = TheSurfaceTool.NbSamplesU(surface, u1, u2);
        int nbsv = TheSurfaceTool.NbSamplesV(surface, v1, v2);
        if (nbsu > 40)
        {
            nbsu = 40;
        }
        if (nbsv > 40)
        {
            nbsv = 40;
        }
        ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, u1, v1, u2, v2);
        Perform(curve, polygon, surface, polyhedron);
    }

    /// <summary>
    /// Computes the intersection between the curve and the surface whose polyhedron is given; the polygon of the
    /// curve is built here (IntCurveSurface_InterImpl::PerformPolyhedron).
    /// </summary>
    public void Perform(in TheCurve curve, in TheSurface surface, ThePolyhedron polyhedron)
    {
        ResetFields();
        _done = true;
        double u1 = TheCurveTool.FirstParameter(curve);
        double u2 = TheCurveTool.LastParameter(curve);
        ThePolygon polygon = ThePolygon.Create(curve, TheCurveTool.NbSamples(curve, u1, u2));
        Perform(curve, polygon, surface, polyhedron);
    }

    /// <summary>
    /// Computes the intersection between the curve and the surface, whose polygon and polyhedron are given
    /// (IntCurveSurface_InterImpl::PerformPolygonPolyhedron).
    /// </summary>
    public void Perform(in TheCurve curve, ThePolygon polygon, in TheSurface surface, ThePolyhedron polyhedron)
    {
        ResetFields();
        _done = true;
        double u1 = TheSurfaceTool.FirstUParameter(surface);
        double v1 = TheSurfaceTool.FirstVParameter(surface);
        double u2 = TheSurfaceTool.LastUParameter(surface);
        double v2 = TheSurfaceTool.LastVParameter(surface);
        InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2);
    }

    /// <summary>
    /// Computes the intersection between the curve and the surface, whose polygon and polyhedron are given, with the
    /// triangles' boxes sorted in <paramref name="bndBSB"/> (IntCurveSurface_InterImpl::PerformPolygonPolyhedronBSB).
    /// </summary>
    public void Perform(in TheCurve curve, ThePolygon polygon, in TheSurface surface, ThePolyhedron polyhedron,
                        BoundSortBox bndBSB)
    {
        ResetFields();
        _done = true;
        double u1 = TheSurfaceTool.FirstUParameter(surface);
        double v1 = TheSurfaceTool.FirstVParameter(surface);
        double u2 = TheSurfaceTool.LastUParameter(surface);
        double v2 = TheSurfaceTool.LastVParameter(surface);
        InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2, bndBSB);
    }

    /// <summary>
    /// Computes the intersection on the parameter range of the surface, by the curve's type
    /// (IntCurveSurface_InterImpl::PerformBounds).
    /// </summary>
    protected void Perform(in TheCurve curve, in TheSurface surface, double U1, double v1, double U2, double v2)
    {
        // Protection from double type overflow (bug26525).
        double uu1 = U1, uu2 = U2, vv1 = v1, vv2 = v2;
        InterUtils.ClampUVParameters(ref uu1, ref uu2, ref vv1, ref vv2);

        CurveKind curveType = TheCurveTool.GetType(curve);

        switch (curveType)
        {
            case Line:
                PerformConicSurf(TheCurveTool.Line(curve), curve, surface, uu1, vv1, uu2, vv2);
                break;
            case Circle:
                PerformConicSurf(TheCurveTool.Circle(curve), curve, surface, uu1, vv1, uu2, vv2);
                break;
            case Ellipse:
                PerformConicSurf(TheCurveTool.Ellipse(curve), curve, surface, uu1, vv1, uu2, vv2);
                break;
            case Parabola:
                PerformConicSurf(TheCurveTool.Parabola(curve), curve, surface, uu1, vv1, uu2, vv2);
                break;
            case Hyperbola:
                PerformConicSurf(TheCurveTool.Hyperbola(curve), curve, surface, uu1, vv1, uu2, vv2);
                break;
            default:
            {
                int nbIntervalsOnCurve = TheCurveTool.NbIntervals(curve, C2);
                SurfaceKind surfaceType = TheSurfaceTool.GetType(surface);
                if ((surfaceType != Plane) && (surfaceType != SurfaceKind.Cylinder)
                    && (surfaceType != SurfaceKind.Cone) && (surfaceType != SurfaceKind.Sphere))
                {
                    if (nbIntervalsOnCurve > 1)
                    {
                        NCollection_Array1<double> tabW = new(1, nbIntervalsOnCurve + 1);
                        TheCurveTool.Intervals(curve, tabW, C2);
                        for (int i = 1; i <= nbIntervalsOnCurve; i++)
                        {
                            double u1 = tabW.Value(i);
                            double u2 = tabW.Value(i + 1);

                            double defl = 0.1;
                            int nbMin = 10;
                            NCollection_HArray1<double> pars = TheCurveTool.SamplePars(curve, u1, u2, defl, nbMin);

                            ThePolygon polygon = ThePolygon.Create(curve, pars.Array1());
                            InternalPerform(curve, polygon, surface, uu1, vv1, uu2, vv2);
                        }
                    }
                    else
                    {
                        double u1 = TheCurveTool.FirstParameter(curve);
                        double u2 = TheCurveTool.LastParameter(curve);

                        double defl = 0.1;
                        int nbMin = 10;
                        NCollection_HArray1<double> pars = TheCurveTool.SamplePars(curve, u1, u2, defl, nbMin);

                        ThePolygon polygon = ThePolygon.Create(curve, pars.Array1());
                        InternalPerform(curve, polygon, surface, uu1, vv1, uu2, vv2);
                    }
                }
                else
                {
                    InternalPerformCurveQuadric(curve, surface);
                }
                break;
            }
        }
    }

    /// <summary>
    /// The interference of the polygon and the polyhedron, with the triangles' boxes sorted in <paramref name="bsb"/>,
    /// refined (IntCurveSurface_InterImpl::InternalPerformBSB).
    /// </summary>
    protected void InternalPerform(in TheCurve curve, ThePolygon polygon, in TheSurface surface,
                                   ThePolyhedron polyhedron, double u0, double v0, double u1, double v1,
                                   BoundSortBox bsb)
    {
        TheInterference interference = TheInterference.Create(polygon, polyhedron, bsb);
        InternalPerformInterference(interference, curve, polygon, surface, polyhedron, u0, v0, u1, v1);
    }

    /// <summary>
    /// The interference of the polygon and the polyhedron, refined (IntCurveSurface_InterImpl::InternalPerform).
    /// </summary>
    protected void InternalPerform(in TheCurve curve, ThePolygon polygon, in TheSurface surface,
                                   ThePolyhedron polyhedron, double u0, double v0, double u1, double v1)
    {
        TheInterference interference = TheInterference.Create(polygon, polyhedron);
        InternalPerformInterference(interference, curve, polygon, surface, polyhedron, u0, v0, u1, v1);
    }

    /// <summary>
    /// The body that OCCT writes out twice, identically, in InternalPerformBSB and InternalPerform after building
    /// the interference.
    /// </summary>
    private void InternalPerformInterference(TheInterference interference, in TheCurve curve, ThePolygon polygon,
                                             in TheSurface surface, ThePolyhedron polyhedron, double u0, double v0,
                                             double u1, double v1)
    {
        TheCSFunction theicsfunction = TheCSFunction.Create(surface, curve);
        TheExactInter intersectionExacte = TheExactInter.Create(theicsfunction, THE_TOLTANGENCY);
        FunctionSetRoot rsnld = new(intersectionExacte.Function());

        double winf = polygon.InfParameter();
        double wsup = polygon.SupParameter();

        InterUtils.SortedStartPoints startPoints = new();
        InterUtils.CollectInterferencePoints(interference, polyhedron, polygon, startPoints);
        InterUtils.SortStartPoints(startPoints);

        NCollection_DynamicArray<CurveSurfaceIntersectionPoint> resultPoints = new();
        InterUtils.ProcessSortedPoints<TheExactInter, TheCSFunction, TheCurve, TheCurveTool,
                                                       TheSurface, TheSurfaceTool>(
            intersectionExacte, rsnld, startPoints, u0, u1, v0, v1, winf, wsup, curve, surface, resultPoints);

        foreach (CurveSurfaceIntersectionPoint point in resultPoints)
        {
            Append(point);
        }
    }

    /// <summary>The intersection of the curve with a quadric (IntCurveSurface_InterImpl::InternalPerformCurveQuadric).</summary>
    protected void InternalPerformCurveQuadric(in TheCurve curve, in TheSurface surface)
    {
        NCollection_DynamicArray<CurveSurfaceIntersectionPoint> points = new();

        InterUtils.PerformCurveQuadric<TheQuadCurvExactInter, TheCurve, TheCurveTool, TheSurface,
                                                       TheSurfaceTool>(curve, surface, points);

        foreach (CurveSurfaceIntersectionPoint point in points)
        {
            Append(point);
        }
    }

    /// <summary>
    /// The intersection of the curve, whose polygon is given, with the surface on the parameter range; the
    /// polyhedron is built here (IntCurveSurface_InterImpl::InternalPerformPolygonBounds).
    /// </summary>
    protected void InternalPerform(in TheCurve curve, ThePolygon polygon, in TheSurface surface, double u1, double v1,
                                   double u2, double v2)
    {
        SurfaceKind surfaceType = TheSurfaceTool.GetType(surface);
        if ((surfaceType != Plane) && (surfaceType != SurfaceKind.Cylinder) && (surfaceType != SurfaceKind.Cone)
            && (surfaceType != SurfaceKind.Sphere))
        {
            if (surfaceType != SurfaceKind.BSplineSurface)
            {
                int nbsu = TheSurfaceTool.NbSamplesU(surface, u1, u2);
                int nbsv = TheSurfaceTool.NbSamplesV(surface, v1, v2);
                if (nbsu > 40)
                {
                    nbsu = 40;
                }
                if (nbsv > 40)
                {
                    nbsv = 40;
                }
                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, u1, v1, u2, v2);
                InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2);
            }
            else
            {
                SurfaceAdaptorBase s = TheSurfaceTool.UTrim(surface, u1, u2, 1.0e-9);
                s = s.VTrim(v1, v2, 1.0e-9);
                TopolTool topTool = new(s);
                double defl = 0.1;
                topTool.SamplePnts(defl, 10, 10);

                int nbpu = topTool.NbSamplesU();
                int nbpv = topTool.NbSamplesV();
                NCollection_Array1<double> upars = new(1, nbpu), vpars = new(1, nbpv);
                topTool.UParameters(upars);
                topTool.VParameters(vpars);

                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, upars, vpars);
                InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2);
            }
        }
        else
        {
            InternalPerformCurveQuadric(curve, surface);
        }
    }

    /// <summary>The intersection of a line with the surface (IntCurveSurface_InterImpl::PerformConicSurfLine).</summary>
    protected void PerformConicSurf(in Line3D line, in TheCurve curve, in TheSurface surface, double u1, double v1,
                                    double u2, double v2)
    {
        SurfaceKind surfaceType = TheSurfaceTool.GetType(surface);
        bool isAnaProcessed = true;

        switch (surfaceType)
        {
            case Plane:
            {
                IntConicQuad linPlane = new(line, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE);
                AppendIntAna(curve, surface, linPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntConicQuad linCylinder = new(line, new Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, linCylinder);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntConicQuad linSphere = new(line, new Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, linSphere);
                break;
            }
            case SurfaceKind.Torus:
            {
                NCollection_DynamicArray<CurveSurfaceIntersectionPoint> points = new();
                if (InterUtils.ProcessLinTorus<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool>(
                        line, curve, surface, points))
                {
                    foreach (CurveSurfaceIntersectionPoint point in points)
                    {
                        Append(point);
                    }
                }
                else
                {
                    isAnaProcessed = false;
                }
                break;
            }
            case SurfaceKind.Cone:
            {
                double correction = 1.0E+5 * global::Kerf.Engine.Foundation.Precision.Angular();
                Cone cn = TheSurfaceTool.Cone(surface);
                if (Math.Abs(cn.SemiAngle()) < M_PI / 2.0 - correction)
                {
                    IntConicQuad linCone = new(line, new Quadric(cn));
                    AppendIntAna(curve, surface, linCone);
                }
                else
                {
                    isAnaProcessed = false;
                }
                break;
            }
            default:
                isAnaProcessed = false;
                break;
        }

        if (!isAnaProcessed)
        {
            int nbsu = TheSurfaceTool.NbSamplesU(surface, u1, u2);
            int nbsv = TheSurfaceTool.NbSamplesV(surface, v1, v2);
            if (nbsu < 1)
            {
                nbsu = 1;
            }
            if (nbsv < 1)
            {
                nbsv = 1;
            }

            bool u1inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(u1);
            bool u2inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(u2);
            bool v1inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(v1);
            bool v2inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(v2);

            double u1new = u1, u2new = u2, v1new = v1, v2new = v2;
            bool noIntersection = false;

            if (u1inf || u2inf || v1inf || v2inf)
            {
                if (surfaceType == SurfaceOfExtrusion)
                {
                    InterUtils.EstLimForInfExtr<TheSurface, TheSurfaceTool>(
                        line, surface, false, nbsu, u1inf, u2inf, v1inf, v2inf, ref u1new, ref u2new, ref v1new,
                        ref v2new, out noIntersection);
                }
                else if (surfaceType == SurfaceOfRevolution)
                {
                    InterUtils.EstLimForInfRevl<TheSurface, TheSurfaceTool>(
                        line, surface, u1inf, u2inf, v1inf, v2inf, ref u1new, ref u2new, ref v1new, ref v2new,
                        out noIntersection);
                }
                else if (surfaceType == SurfaceKind.OffsetSurface)
                {
                    InterUtils.EstLimForInfOffs<TheSurface, TheSurfaceTool>(
                        line, surface, nbsu, u1inf, u2inf, v1inf, v2inf, ref u1new, ref u2new, ref v1new, ref v2new,
                        out noIntersection);
                }
                else
                {
                    InterUtils.EstLimForInfSurf(ref u1new, ref u2new, ref v1new, ref v2new);
                }
            }

            if (noIntersection)
            {
                return;
            }

            if (nbsu < 20)
            {
                nbsu = 20;
            }
            if (nbsv < 20)
            {
                nbsv = 20;
            }

            ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, u1new, v1new, u2new, v2new);
            PolygonTool bndTool = new();
            Box boxLine = new();
            bndTool.LinBox(line, polyhedron.Bounding(), ref boxLine);

            for (int nbseg = 1; nbseg <= bndTool.SegmentCount; nbseg++)
            {
                double pinf = bndTool.BeginParam(nbseg);
                double psup = bndTool.EndParam(nbseg);
                if ((psup - pinf) < 1e-10)
                {
                    pinf -= 1e-10;
                    psup += 1e-10;
                }
                ThePolygon polygon = ThePolygon.Create(curve, pinf, psup, 2);
                InternalPerform(curve, polygon, surface, polyhedron, u1new, v1new, u2new, v2new);
            }
        }
    }

    /// <summary>The intersection of a circle with the surface (IntCurveSurface_InterImpl::PerformConicSurfCircle).</summary>
    protected void PerformConicSurf(in Circle3D circle, in TheCurve curve, in TheSurface surface, double u1, double v1,
                                    double u2, double v2)
    {
        SurfaceKind surfaceType = TheSurfaceTool.GetType(surface);
        switch (surfaceType)
        {
            case Plane:
            {
                IntConicQuad circPlane = new(circle, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE,
                                                    THE_TOLERANCE);
                AppendIntAna(curve, surface, circPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntConicQuad circCylinder = new(circle, new Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, circCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntConicQuad circCone = new(circle, new Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, circCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntConicQuad circSphere = new(circle, new Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, circSphere);
                break;
            }
            default:
            {
                ThePolygon polygon = ThePolygon.Create(curve, THE_NBSAMPLESONCIRCLE);
                InternalPerform(curve, polygon, surface, u1, v1, u2, v2);
                break;
            }
        }
    }

    /// <summary>The intersection of an ellipse with the surface (IntCurveSurface_InterImpl::PerformConicSurfEllipse).</summary>
    protected void PerformConicSurf(in Ellipse3D ellipse, in TheCurve curve, in TheSurface surface, double u1,
                                    double v1, double u2, double v2)
    {
        SurfaceKind surfaceType = TheSurfaceTool.GetType(surface);
        switch (surfaceType)
        {
            case Plane:
            {
                IntConicQuad ellipsePlane = new(ellipse, TheSurfaceTool.Plane(surface),
                                                       THE_TOLERANCE_ANGULAIRE, THE_TOLERANCE);
                AppendIntAna(curve, surface, ellipsePlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntConicQuad ellipseCylinder =
                    new(ellipse, new Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, ellipseCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntConicQuad ellipseCone = new(ellipse, new Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, ellipseCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntConicQuad ellipseSphere = new(ellipse, new Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, ellipseSphere);
                break;
            }
            default:
            {
                ThePolygon polygon = ThePolygon.Create(curve, THE_NBSAMPLESONELLIPSE);
                InternalPerform(curve, polygon, surface, u1, v1, u2, v2);
                break;
            }
        }
    }

    /// <summary>The intersection of a parabola with the surface (IntCurveSurface_InterImpl::PerformConicSurfParabola).</summary>
    protected void PerformConicSurf(in Parabola3D parab, in TheCurve curve, in TheSurface surface, double u1, double v1,
                                    double u2, double v2)
    {
        SurfaceKind surfaceType = TheSurfaceTool.GetType(surface);
        switch (surfaceType)
        {
            case Plane:
            {
                IntConicQuad parabPlane = new(parab, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE);
                AppendIntAna(curve, surface, parabPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntConicQuad parabCylinder = new(parab, new Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, parabCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntConicQuad parabCone = new(parab, new Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, parabCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntConicQuad parabSphere = new(parab, new Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, parabSphere);
                break;
            }
            default:
            {
                int nbsu = TheSurfaceTool.NbSamplesU(surface, u1, u2);
                int nbsv = TheSurfaceTool.NbSamplesV(surface, v1, v2);
                if (nbsu > 40)
                {
                    nbsu = 40;
                }
                if (nbsv > 40)
                {
                    nbsv = 40;
                }
                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, u1, v1, u2, v2);
                PolygonTool bndTool = new();
                Box boxParab = new();
                bndTool.ParabBox(parab, polyhedron.Bounding(), ref boxParab);
                for (int nbseg = 1; nbseg <= bndTool.SegmentCount; nbseg++)
                {
                    ThePolygon polygon = ThePolygon.Create(curve, bndTool.BeginParam(nbseg), bndTool.EndParam(nbseg),
                                                           THE_NBSAMPLESONPARAB);
                    InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2);
                }
                break;
            }
        }
    }

    /// <summary>The intersection of a hyperbola with the surface (IntCurveSurface_InterImpl::PerformConicSurfHyperbola).</summary>
    protected void PerformConicSurf(in Hyperbola3D hypr, in TheCurve curve, in TheSurface surface, double u1, double v1,
                                    double u2, double v2)
    {
        SurfaceKind surfaceType = TheSurfaceTool.GetType(surface);
        switch (surfaceType)
        {
            case Plane:
            {
                IntConicQuad hyprPlane = new(hypr, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE);
                AppendIntAna(curve, surface, hyprPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntConicQuad hyprCylinder = new(hypr, new Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, hyprCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntConicQuad hyprCone = new(hypr, new Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, hyprCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntConicQuad hyprSphere = new(hypr, new Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, hyprSphere);
                break;
            }
            default:
            {
                int nbsu = TheSurfaceTool.NbSamplesU(surface, u1, u2);
                int nbsv = TheSurfaceTool.NbSamplesV(surface, v1, v2);
                if (nbsu > 40)
                {
                    nbsu = 40;
                }
                if (nbsv > 40)
                {
                    nbsv = 40;
                }
                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, u1, v1, u2, v2);
                PolygonTool bndTool = new();
                Box boxHypr = new();
                bndTool.HyprBox(hypr, polyhedron.Bounding(), ref boxHypr);
                for (int nbseg = 1; nbseg <= bndTool.SegmentCount; nbseg++)
                {
                    ThePolygon polygon = ThePolygon.Create(curve, bndTool.BeginParam(nbseg), bndTool.EndParam(nbseg),
                                                           THE_NBSAMPLESONHYPR);
                    InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2);
                }
                break;
            }
        }
    }

    /// <summary>
    /// Appends the points of an analytic intersection inside the bounds, or records that the curve is parallel
    /// (IntCurveSurface_InterImpl::AppendIntAna).
    /// </summary>
    protected void AppendIntAna(in TheCurve curve, in TheSurface surface, IntConicQuad intana_ConicQuad)
    {
        NCollection_DynamicArray<CurveSurfaceIntersectionPoint> points = new();

        if (InterUtils.ProcessIntAna<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool>(
                curve, surface, intana_ConicQuad, out bool isParallel, points))
        {
            if (isParallel)
            {
                _isParallel = true;
            }
            else
            {
                foreach (CurveSurfaceIntersectionPoint point in points)
                {
                    Append(point);
                }
            }
        }
    }

    /// <summary>Appends the point of parameters (u, v) and w, if it is inside the bounds.</summary>
    protected void AppendPoint(in TheCurve curve, double lw, in TheSurface surface, double su, double sv)
    {
        if (InterUtils.ComputeAppendPoint<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool>(
                curve, lw, surface, su, sv, out CurveSurfaceIntersectionPoint point))
        {
            Append(point);
        }
    }

    /// <summary>Not implemented (as in OCCT): no segment is ever appended.</summary>
    protected void AppendSegment(in TheCurve curve, double u0, double u1, in TheSurface surface)
    {
        _ = curve;
        _ = u0;
        _ = u1;
        _ = surface;
    }
}
