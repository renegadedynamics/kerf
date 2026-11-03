// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_Inter.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The static tool over a curve that the generic curve/surface intersection uses (OCCT's CurveTool template
/// argument: IntCurveSurface_TheHCurveTool, HLRBRep_LineTool).
/// </summary>
public interface IIntCurveSurface_CurveTool<TheCurve> : IIntImp_ZerCSParCurveTool<TheCurve>
{
    static abstract CurveKind GetType(in TheCurve C);

    static abstract Line3D Line(in TheCurve C);

    static abstract Circle3D Circle(in TheCurve C);

    static abstract Ellipse3D Ellipse(in TheCurve C);

    static abstract Parabola3D Parabola(in TheCurve C);

    static abstract Hyperbola3D Hyperbola(in TheCurve C);

    static abstract int NbIntervals(in TheCurve C, Continuity S);

    static abstract void Intervals(in TheCurve C, NCollection_Array1<double> T, Continuity S);

    static abstract bool IsPeriodic(in TheCurve C);

    static abstract double Period(in TheCurve C);

    static abstract void D0(in TheCurve C, double U, out Point3D P);

    static abstract int NbSamples(in TheCurve C, double U0, double U1);

    static abstract NCollection_HArray1<double> SamplePars(in TheCurve C, double U0, double U1, double Defl,
                                                           int NbMin);
}

/// <summary>
/// The static tool over a surface that the generic curve/surface intersection uses (OCCT's SurfaceTool template
/// argument: Adaptor3d_HSurfaceTool, through <see cref="IntCurveSurface_TheHSurfaceTool"/>; HLRBRep_SurfaceTool).
/// </summary>
public interface IIntCurveSurface_SurfaceTool<TheSurface> : IIntImp_ZerCSParPSurfaceTool<TheSurface>
{
    static abstract SurfaceKind GetType(in TheSurface S);

    static abstract Plane3D Plane(in TheSurface S);

    static abstract Cylinder Cylinder(in TheSurface S);

    static abstract Cone Cone(in TheSurface S);

    static abstract Sphere Sphere(in TheSurface S);

    static abstract Torus Torus(in TheSurface S);

    static abstract int NbSamplesU(in TheSurface S, double u1, double u2);

    static abstract int NbSamplesV(in TheSurface S, double v1, double v2);

    static abstract int NbUIntervals(in TheSurface S, Continuity Sh);

    static abstract int NbVIntervals(in TheSurface S, Continuity Sh);

    static abstract void UIntervals(in TheSurface S, NCollection_Array1<double> T, Continuity Sh);

    static abstract void VIntervals(in TheSurface S, NCollection_Array1<double> T, Continuity Sh);

    static abstract Adaptor3d_Surface UTrim(in TheSurface S, double First, double Last, double Tol);

    static abstract bool IsUClosed(in TheSurface S);

    static abstract bool IsVClosed(in TheSurface S);

    static abstract bool IsUPeriodic(in TheSurface S);

    static abstract bool IsVPeriodic(in TheSurface S);

    static abstract double UPeriod(in TheSurface S);

    static abstract double VPeriod(in TheSurface S);

    static abstract void D0(in TheSurface S, double U, double V, out Point3D P);

    static abstract Adaptor3d_Surface BasisSurface(in TheSurface S);

    static abstract Adaptor3d_Curve BasisCurve(in TheSurface S);

    static abstract Direction3D Direction(in TheSurface S);

    static abstract Axis3D AxeOfRevolution(in TheSurface S);

    static abstract double OffsetValue(in TheSurface S);
}

/// <summary>What the intersection reads from the polygon of a curve: its parameter range and parameters on the curve.</summary>
public interface IIntCurveSurface_Polygon
{
    double InfParameter();

    double SupParameter();

    /// <summary>The parameter on the curve of the point at <paramref name="ParamOnLine"/> (0 to 1) along the segment <paramref name="Index"/>.</summary>
    double ApproxParamOnCurve(int Index, double ParamOnLine);
}

/// <summary>The polygon type of the generic intersection, with OCCT's constructors (IntCurveSurface_ThePolygonOfHInter).</summary>
public interface IIntCurveSurface_ThePolygon<TSelf, TheCurve> : IIntCurveSurface_Polygon
    where TSelf : IIntCurveSurface_ThePolygon<TSelf, TheCurve>
{
    static abstract TSelf Create(in TheCurve Curve, int NbPnt);

    static abstract TSelf Create(in TheCurve Curve, double U1, double U2, int NbPnt);

    static abstract TSelf Create(in TheCurve Curve, NCollection_Array1<double> Upars);
}

/// <summary>What the intersection reads from the polyhedron of a surface: its box, triangles, points and parameters.</summary>
public interface IIntCurveSurface_Polyhedron
{
    Box Bounding();

    void Triangle(int Index, out int P1, out int P2, out int P3);

    Point3D Point(int Index);

    Point3D Point(int Index, out double U, out double V);

    void Parameters(int Index, out double U, out double V);
}

/// <summary>The polyhedron type of the generic intersection, with OCCT's constructors (IntCurveSurface_ThePolyhedronOfHInter).</summary>
public interface IIntCurveSurface_ThePolyhedron<TSelf, TheSurface> : IIntCurveSurface_Polyhedron
    where TSelf : IIntCurveSurface_ThePolyhedron<TSelf, TheSurface>
{
    static abstract TSelf Create(in TheSurface Surface, int nbdU, int nbdV, double U1, double V1, double U2,
                                 double V2);

    static abstract TSelf Create(in TheSurface Surface, NCollection_Array1<double> Upars,
                                 NCollection_Array1<double> Vpars);
}

/// <summary>The interference type of the generic intersection, with OCCT's constructors (IntCurveSurface_TheInterferenceOfHInter).</summary>
public interface IIntCurveSurface_TheInterference<TSelf, ThePolygon, ThePolyhedron>
    where TSelf : IIntCurveSurface_TheInterference<TSelf, ThePolygon, ThePolyhedron>
{
    static abstract TSelf Create(ThePolygon thePolyg, ThePolyhedron thePolyh);

    static abstract TSelf Create(ThePolygon thePolyg, ThePolyhedron thePolyh, BoundSortBox theBoundSB);
}

/// <summary>The curve/surface function of the generic intersection, with OCCT's constructor (IntCurveSurface_TheCSFunctionOfHInter).</summary>
public interface IIntCurveSurface_TheCSFunction<TSelf, TheSurface, TheCurve>
    where TSelf : IIntCurveSurface_TheCSFunction<TSelf, TheSurface, TheCurve>
{
    static abstract TSelf Create(in TheSurface S, in TheCurve C);
}

/// <summary>The exact intersection of the generic intersection, with OCCT's constructor (IntCurveSurface_TheExactHInter).</summary>
public interface IIntCurveSurface_TheExactInter<TSelf, TheCSFunction>
    where TSelf : IIntCurveSurface_TheExactInter<TSelf, TheCSFunction>
{
    static abstract TSelf Create(TheCSFunction F, double TolTangency);
}

/// <summary>The exact curve/quadric intersection of the generic intersection (IntCurveSurface_TheQuadCurvExactHInter).</summary>
public interface IIntCurveSurface_TheQuadCurvExactInter<TSelf, TheSurface, TheCurve>
    where TSelf : IIntCurveSurface_TheQuadCurvExactInter<TSelf, TheSurface, TheCurve>
{
    static abstract TSelf Create(in TheSurface S, in TheCurve C);

    bool IsDone();

    int NbRoots();

    double Root(int Index);
}

/// <summary>
/// The function of a quadric along a curve, with OCCT's constructor (IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter).
/// </summary>
public interface IIntCurveSurface_TheQuadCurvFunc<TSelf, TheCurve>
    where TSelf : IIntCurveSurface_TheQuadCurvFunc<TSelf, TheCurve>
{
    static abstract TSelf Create(in IntSurf_Quadric Q, in TheCurve C);
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
public class IntCurveSurface_Inter<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool, ThePolygon, ThePolyhedron,
                                   TheInterference, TheCSFunction, TheExactInter, TheQuadCurvExactInter>
    : IntCurveSurface_Intersection
    where TheCurveTool : IIntCurveSurface_CurveTool<TheCurve>
    where TheSurfaceTool : IIntCurveSurface_SurfaceTool<TheSurface>
    where ThePolygon : IIntCurveSurface_ThePolygon<ThePolygon, TheCurve>
    where ThePolyhedron : IIntCurveSurface_ThePolyhedron<ThePolyhedron, TheSurface>
    where TheInterference : Intf_Interference, IIntCurveSurface_TheInterference<TheInterference, ThePolygon, ThePolyhedron>
    where TheCSFunction : FunctionSetWithDerivatives, IIntImp_CSFunction<TheSurface, TheCurve>,
    IIntCurveSurface_TheCSFunction<TheCSFunction, TheSurface, TheCurve>
    where TheExactInter : IntImp_IntCS<TheCSFunction, TheSurface, TheSurfaceTool, TheCurve, TheCurveTool>,
    IIntCurveSurface_TheExactInter<TheExactInter, TheCSFunction>
    where TheQuadCurvExactInter : IIntCurveSurface_TheQuadCurvExactInter<TheQuadCurvExactInter, TheSurface, TheCurve>
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
        done = true;

        NCollection_DynamicArray<IntCurveSurface_InterUtils.UVBounds> aIntervals = new();
        IntCurveSurface_InterUtils.DecomposeSurfaceIntervals<TheSurface, TheSurfaceTool>(surface, aIntervals);

        foreach (IntCurveSurface_InterUtils.UVBounds aBounds in aIntervals)
        {
            Perform(curve, surface, aBounds.U0, aBounds.V0, aBounds.U1, aBounds.V1);
        }
    }

    /// <summary>
    /// Computes the intersection between the curve (whose polygon is given) and the surface; the polyhedron of the
    /// surface is built here (IntCurveSurface_InterImpl::PerformPolygon).
    /// </summary>
    public void Perform(in TheCurve curve, ThePolygon polygon, in TheSurface surface)
    {
        ResetFields();
        done = true;
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
        done = true;
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
        done = true;
        double u1 = TheSurfaceTool.FirstUParameter(surface);
        double v1 = TheSurfaceTool.FirstVParameter(surface);
        double u2 = TheSurfaceTool.LastUParameter(surface);
        double v2 = TheSurfaceTool.LastVParameter(surface);
        InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2);
    }

    /// <summary>
    /// Computes the intersection between the curve and the surface, whose polygon and polyhedron are given, with the
    /// triangles' boxes sorted in <paramref name="BndBSB"/> (IntCurveSurface_InterImpl::PerformPolygonPolyhedronBSB).
    /// </summary>
    public void Perform(in TheCurve curve, ThePolygon polygon, in TheSurface surface, ThePolyhedron polyhedron,
                        BoundSortBox BndBSB)
    {
        ResetFields();
        done = true;
        double u1 = TheSurfaceTool.FirstUParameter(surface);
        double v1 = TheSurfaceTool.FirstVParameter(surface);
        double u2 = TheSurfaceTool.LastUParameter(surface);
        double v2 = TheSurfaceTool.LastVParameter(surface);
        InternalPerform(curve, polygon, surface, polyhedron, u1, v1, u2, v2, BndBSB);
    }

    /// <summary>
    /// Computes the intersection on the parameter range of the surface, by the curve's type
    /// (IntCurveSurface_InterImpl::PerformBounds).
    /// </summary>
    protected void Perform(in TheCurve curve, in TheSurface surface, double U1, double V1, double U2, double V2)
    {
        // Protection from double type overflow (bug26525).
        double UU1 = U1, UU2 = U2, VV1 = V1, VV2 = V2;
        IntCurveSurface_InterUtils.ClampUVParameters(ref UU1, ref UU2, ref VV1, ref VV2);

        CurveKind aCurveType = TheCurveTool.GetType(curve);

        switch (aCurveType)
        {
            case Line:
                PerformConicSurf(TheCurveTool.Line(curve), curve, surface, UU1, VV1, UU2, VV2);
                break;
            case Circle:
                PerformConicSurf(TheCurveTool.Circle(curve), curve, surface, UU1, VV1, UU2, VV2);
                break;
            case Ellipse:
                PerformConicSurf(TheCurveTool.Ellipse(curve), curve, surface, UU1, VV1, UU2, VV2);
                break;
            case Parabola:
                PerformConicSurf(TheCurveTool.Parabola(curve), curve, surface, UU1, VV1, UU2, VV2);
                break;
            case Hyperbola:
                PerformConicSurf(TheCurveTool.Hyperbola(curve), curve, surface, UU1, VV1, UU2, VV2);
                break;
            default:
            {
                int nbIntervalsOnCurve = TheCurveTool.NbIntervals(curve, C2);
                SurfaceKind aSurfaceType = TheSurfaceTool.GetType(surface);
                if ((aSurfaceType != Plane) && (aSurfaceType != SurfaceKind.Cylinder)
                    && (aSurfaceType != SurfaceKind.Cone) && (aSurfaceType != SurfaceKind.Sphere))
                {
                    if (nbIntervalsOnCurve > 1)
                    {
                        NCollection_Array1<double> TabW = new(1, nbIntervalsOnCurve + 1);
                        TheCurveTool.Intervals(curve, TabW, C2);
                        for (int i = 1; i <= nbIntervalsOnCurve; i++)
                        {
                            double u1 = TabW.Value(i);
                            double u2 = TabW.Value(i + 1);

                            double defl = 0.1;
                            int NbMin = 10;
                            NCollection_HArray1<double> aPars = TheCurveTool.SamplePars(curve, u1, u2, defl, NbMin);

                            ThePolygon polygon = ThePolygon.Create(curve, aPars.Array1());
                            InternalPerform(curve, polygon, surface, UU1, VV1, UU2, VV2);
                        }
                    }
                    else
                    {
                        double u1 = TheCurveTool.FirstParameter(curve);
                        double u2 = TheCurveTool.LastParameter(curve);

                        double defl = 0.1;
                        int NbMin = 10;
                        NCollection_HArray1<double> aPars = TheCurveTool.SamplePars(curve, u1, u2, defl, NbMin);

                        ThePolygon polygon = ThePolygon.Create(curve, aPars.Array1());
                        InternalPerform(curve, polygon, surface, UU1, VV1, UU2, VV2);
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
    /// The interference of the polygon and the polyhedron, with the triangles' boxes sorted in <paramref name="BSB"/>,
    /// refined (IntCurveSurface_InterImpl::InternalPerformBSB).
    /// </summary>
    protected void InternalPerform(in TheCurve curve, ThePolygon polygon, in TheSurface surface,
                                   ThePolyhedron polyhedron, double u0, double v0, double u1, double v1,
                                   BoundSortBox BSB)
    {
        TheInterference interference = TheInterference.Create(polygon, polyhedron, BSB);
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

        IntCurveSurface_InterUtils.SortedStartPoints aStartPoints = new();
        IntCurveSurface_InterUtils.CollectInterferencePoints(interference, polyhedron, polygon, aStartPoints);
        IntCurveSurface_InterUtils.SortStartPoints(aStartPoints);

        NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> aResultPoints = new();
        IntCurveSurface_InterUtils.ProcessSortedPoints<TheExactInter, TheCSFunction, TheCurve, TheCurveTool,
                                                       TheSurface, TheSurfaceTool>(
            intersectionExacte, rsnld, aStartPoints, u0, u1, v0, v1, winf, wsup, curve, surface, aResultPoints);

        foreach (IntCurveSurface_IntersectionPoint aPoint in aResultPoints)
        {
            Append(aPoint);
        }
    }

    /// <summary>The intersection of the curve with a quadric (IntCurveSurface_InterImpl::InternalPerformCurveQuadric).</summary>
    protected void InternalPerformCurveQuadric(in TheCurve curve, in TheSurface surface)
    {
        NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> aPoints = new();

        IntCurveSurface_InterUtils.PerformCurveQuadric<TheQuadCurvExactInter, TheCurve, TheCurveTool, TheSurface,
                                                       TheSurfaceTool>(curve, surface, aPoints);

        foreach (IntCurveSurface_IntersectionPoint aPoint in aPoints)
        {
            Append(aPoint);
        }
    }

    /// <summary>
    /// The intersection of the curve, whose polygon is given, with the surface on the parameter range; the
    /// polyhedron is built here (IntCurveSurface_InterImpl::InternalPerformPolygonBounds).
    /// </summary>
    protected void InternalPerform(in TheCurve curve, ThePolygon polygon, in TheSurface surface, double U1, double V1,
                                   double U2, double V2)
    {
        SurfaceKind aSurfaceType = TheSurfaceTool.GetType(surface);
        if ((aSurfaceType != Plane) && (aSurfaceType != SurfaceKind.Cylinder) && (aSurfaceType != SurfaceKind.Cone)
            && (aSurfaceType != SurfaceKind.Sphere))
        {
            if (aSurfaceType != BSplineSurface)
            {
                int nbsu = TheSurfaceTool.NbSamplesU(surface, U1, U2);
                int nbsv = TheSurfaceTool.NbSamplesV(surface, V1, V2);
                if (nbsu > 40)
                {
                    nbsu = 40;
                }
                if (nbsv > 40)
                {
                    nbsv = 40;
                }
                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, U1, V1, U2, V2);
                InternalPerform(curve, polygon, surface, polyhedron, U1, V1, U2, V2);
            }
            else
            {
                Adaptor3d_Surface aS = TheSurfaceTool.UTrim(surface, U1, U2, 1.0e-9);
                aS = aS.VTrim(V1, V2, 1.0e-9);
                Adaptor3d_TopolTool aTopTool = new(aS);
                double defl = 0.1;
                aTopTool.SamplePnts(defl, 10, 10);

                int nbpu = aTopTool.NbSamplesU();
                int nbpv = aTopTool.NbSamplesV();
                NCollection_Array1<double> Upars = new(1, nbpu), Vpars = new(1, nbpv);
                aTopTool.UParameters(Upars);
                aTopTool.VParameters(Vpars);

                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, Upars, Vpars);
                InternalPerform(curve, polygon, surface, polyhedron, U1, V1, U2, V2);
            }
        }
        else
        {
            InternalPerformCurveQuadric(curve, surface);
        }
    }

    /// <summary>The intersection of a line with the surface (IntCurveSurface_InterImpl::PerformConicSurfLine).</summary>
    protected void PerformConicSurf(in Line3D Line, in TheCurve curve, in TheSurface surface, double U1, double V1,
                                    double U2, double V2)
    {
        SurfaceKind aSurfaceType = TheSurfaceTool.GetType(surface);
        bool isAnaProcessed = true;

        switch (aSurfaceType)
        {
            case Plane:
            {
                IntAna_IntConicQuad LinPlane = new(Line, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE);
                AppendIntAna(curve, surface, LinPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntAna_IntConicQuad LinCylinder = new(Line, new IntAna_Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, LinCylinder);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntAna_IntConicQuad LinSphere = new(Line, new IntAna_Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, LinSphere);
                break;
            }
            case SurfaceKind.Torus:
            {
                NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> aPoints = new();
                if (IntCurveSurface_InterUtils.ProcessLinTorus<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool>(
                        Line, curve, surface, aPoints))
                {
                    foreach (IntCurveSurface_IntersectionPoint aPoint in aPoints)
                    {
                        Append(aPoint);
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
                    IntAna_IntConicQuad LinCone = new(Line, new IntAna_Quadric(cn));
                    AppendIntAna(curve, surface, LinCone);
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
            int nbsu = TheSurfaceTool.NbSamplesU(surface, U1, U2);
            int nbsv = TheSurfaceTool.NbSamplesV(surface, V1, V2);
            if (nbsu < 1)
            {
                nbsu = 1;
            }
            if (nbsv < 1)
            {
                nbsv = 1;
            }

            bool U1inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(U1);
            bool U2inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(U2);
            bool V1inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(V1);
            bool V2inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(V2);

            double U1new = U1, U2new = U2, V1new = V1, V2new = V2;
            bool NoIntersection = false;

            if (U1inf || U2inf || V1inf || V2inf)
            {
                if (aSurfaceType == SurfaceOfExtrusion)
                {
                    IntCurveSurface_InterUtils.EstLimForInfExtr<TheSurface, TheSurfaceTool>(
                        Line, surface, false, nbsu, U1inf, U2inf, V1inf, V2inf, ref U1new, ref U2new, ref V1new,
                        ref V2new, out NoIntersection);
                }
                else if (aSurfaceType == SurfaceOfRevolution)
                {
                    IntCurveSurface_InterUtils.EstLimForInfRevl<TheSurface, TheSurfaceTool>(
                        Line, surface, U1inf, U2inf, V1inf, V2inf, ref U1new, ref U2new, ref V1new, ref V2new,
                        out NoIntersection);
                }
                else if (aSurfaceType == OffsetSurface)
                {
                    IntCurveSurface_InterUtils.EstLimForInfOffs<TheSurface, TheSurfaceTool>(
                        Line, surface, nbsu, U1inf, U2inf, V1inf, V2inf, ref U1new, ref U2new, ref V1new, ref V2new,
                        out NoIntersection);
                }
                else
                {
                    IntCurveSurface_InterUtils.EstLimForInfSurf(ref U1new, ref U2new, ref V1new, ref V2new);
                }
            }

            if (NoIntersection)
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

            ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, U1new, V1new, U2new, V2new);
            Intf_Tool bndTool = new();
            Box boxLine = new();
            bndTool.LinBox(Line, polyhedron.Bounding(), ref boxLine);

            for (int nbseg = 1; nbseg <= bndTool.NbSegments(); nbseg++)
            {
                double pinf = bndTool.BeginParam(nbseg);
                double psup = bndTool.EndParam(nbseg);
                if ((psup - pinf) < 1e-10)
                {
                    pinf -= 1e-10;
                    psup += 1e-10;
                }
                ThePolygon polygon = ThePolygon.Create(curve, pinf, psup, 2);
                InternalPerform(curve, polygon, surface, polyhedron, U1new, V1new, U2new, V2new);
            }
        }
    }

    /// <summary>The intersection of a circle with the surface (IntCurveSurface_InterImpl::PerformConicSurfCircle).</summary>
    protected void PerformConicSurf(in Circle3D Circle, in TheCurve curve, in TheSurface surface, double U1, double V1,
                                    double U2, double V2)
    {
        SurfaceKind aSurfaceType = TheSurfaceTool.GetType(surface);
        switch (aSurfaceType)
        {
            case Plane:
            {
                IntAna_IntConicQuad CircPlane = new(Circle, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE,
                                                    THE_TOLERANCE);
                AppendIntAna(curve, surface, CircPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntAna_IntConicQuad CircCylinder = new(Circle, new IntAna_Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, CircCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntAna_IntConicQuad CircCone = new(Circle, new IntAna_Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, CircCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntAna_IntConicQuad CircSphere = new(Circle, new IntAna_Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, CircSphere);
                break;
            }
            default:
            {
                ThePolygon polygon = ThePolygon.Create(curve, THE_NBSAMPLESONCIRCLE);
                InternalPerform(curve, polygon, surface, U1, V1, U2, V2);
                break;
            }
        }
    }

    /// <summary>The intersection of an ellipse with the surface (IntCurveSurface_InterImpl::PerformConicSurfEllipse).</summary>
    protected void PerformConicSurf(in Ellipse3D Ellipse, in TheCurve curve, in TheSurface surface, double U1,
                                    double V1, double U2, double V2)
    {
        SurfaceKind aSurfaceType = TheSurfaceTool.GetType(surface);
        switch (aSurfaceType)
        {
            case Plane:
            {
                IntAna_IntConicQuad EllipsePlane = new(Ellipse, TheSurfaceTool.Plane(surface),
                                                       THE_TOLERANCE_ANGULAIRE, THE_TOLERANCE);
                AppendIntAna(curve, surface, EllipsePlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntAna_IntConicQuad EllipseCylinder =
                    new(Ellipse, new IntAna_Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, EllipseCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntAna_IntConicQuad EllipseCone = new(Ellipse, new IntAna_Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, EllipseCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntAna_IntConicQuad EllipseSphere = new(Ellipse, new IntAna_Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, EllipseSphere);
                break;
            }
            default:
            {
                ThePolygon polygon = ThePolygon.Create(curve, THE_NBSAMPLESONELLIPSE);
                InternalPerform(curve, polygon, surface, U1, V1, U2, V2);
                break;
            }
        }
    }

    /// <summary>The intersection of a parabola with the surface (IntCurveSurface_InterImpl::PerformConicSurfParabola).</summary>
    protected void PerformConicSurf(in Parabola3D Parab, in TheCurve curve, in TheSurface surface, double U1, double V1,
                                    double U2, double V2)
    {
        SurfaceKind aSurfaceType = TheSurfaceTool.GetType(surface);
        switch (aSurfaceType)
        {
            case Plane:
            {
                IntAna_IntConicQuad ParabPlane = new(Parab, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE);
                AppendIntAna(curve, surface, ParabPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntAna_IntConicQuad ParabCylinder = new(Parab, new IntAna_Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, ParabCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntAna_IntConicQuad ParabCone = new(Parab, new IntAna_Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, ParabCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntAna_IntConicQuad ParabSphere = new(Parab, new IntAna_Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, ParabSphere);
                break;
            }
            default:
            {
                int nbsu = TheSurfaceTool.NbSamplesU(surface, U1, U2);
                int nbsv = TheSurfaceTool.NbSamplesV(surface, V1, V2);
                if (nbsu > 40)
                {
                    nbsu = 40;
                }
                if (nbsv > 40)
                {
                    nbsv = 40;
                }
                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, U1, V1, U2, V2);
                Intf_Tool bndTool = new();
                Box boxParab = new();
                bndTool.ParabBox(Parab, polyhedron.Bounding(), ref boxParab);
                for (int nbseg = 1; nbseg <= bndTool.NbSegments(); nbseg++)
                {
                    ThePolygon polygon = ThePolygon.Create(curve, bndTool.BeginParam(nbseg), bndTool.EndParam(nbseg),
                                                           THE_NBSAMPLESONPARAB);
                    InternalPerform(curve, polygon, surface, polyhedron, U1, V1, U2, V2);
                }
                break;
            }
        }
    }

    /// <summary>The intersection of a hyperbola with the surface (IntCurveSurface_InterImpl::PerformConicSurfHyperbola).</summary>
    protected void PerformConicSurf(in Hyperbola3D Hypr, in TheCurve curve, in TheSurface surface, double U1, double V1,
                                    double U2, double V2)
    {
        SurfaceKind aSurfaceType = TheSurfaceTool.GetType(surface);
        switch (aSurfaceType)
        {
            case Plane:
            {
                IntAna_IntConicQuad HyprPlane = new(Hypr, TheSurfaceTool.Plane(surface), THE_TOLERANCE_ANGULAIRE);
                AppendIntAna(curve, surface, HyprPlane);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                IntAna_IntConicQuad HyprCylinder = new(Hypr, new IntAna_Quadric(TheSurfaceTool.Cylinder(surface)));
                AppendIntAna(curve, surface, HyprCylinder);
                break;
            }
            case SurfaceKind.Cone:
            {
                IntAna_IntConicQuad HyprCone = new(Hypr, new IntAna_Quadric(TheSurfaceTool.Cone(surface)));
                AppendIntAna(curve, surface, HyprCone);
                break;
            }
            case SurfaceKind.Sphere:
            {
                IntAna_IntConicQuad HyprSphere = new(Hypr, new IntAna_Quadric(TheSurfaceTool.Sphere(surface)));
                AppendIntAna(curve, surface, HyprSphere);
                break;
            }
            default:
            {
                int nbsu = TheSurfaceTool.NbSamplesU(surface, U1, U2);
                int nbsv = TheSurfaceTool.NbSamplesV(surface, V1, V2);
                if (nbsu > 40)
                {
                    nbsu = 40;
                }
                if (nbsv > 40)
                {
                    nbsv = 40;
                }
                ThePolyhedron polyhedron = ThePolyhedron.Create(surface, nbsu, nbsv, U1, V1, U2, V2);
                Intf_Tool bndTool = new();
                Box boxHypr = new();
                bndTool.HyprBox(Hypr, polyhedron.Bounding(), ref boxHypr);
                for (int nbseg = 1; nbseg <= bndTool.NbSegments(); nbseg++)
                {
                    ThePolygon polygon = ThePolygon.Create(curve, bndTool.BeginParam(nbseg), bndTool.EndParam(nbseg),
                                                           THE_NBSAMPLESONHYPR);
                    InternalPerform(curve, polygon, surface, polyhedron, U1, V1, U2, V2);
                }
                break;
            }
        }
    }

    /// <summary>
    /// Appends the points of an analytic intersection inside the bounds, or records that the curve is parallel
    /// (IntCurveSurface_InterImpl::AppendIntAna).
    /// </summary>
    protected void AppendIntAna(in TheCurve curve, in TheSurface surface, IntAna_IntConicQuad intana_ConicQuad)
    {
        NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> aPoints = new();

        if (IntCurveSurface_InterUtils.ProcessIntAna<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool>(
                curve, surface, intana_ConicQuad, out bool aIsParallel, aPoints))
        {
            if (aIsParallel)
            {
                myIsParallel = true;
            }
            else
            {
                foreach (IntCurveSurface_IntersectionPoint aPoint in aPoints)
                {
                    Append(aPoint);
                }
            }
        }
    }

    /// <summary>Appends the point of parameters (u, v) and w, if it is inside the bounds.</summary>
    protected void AppendPoint(in TheCurve curve, double lw, in TheSurface surface, double su, double sv)
    {
        if (IntCurveSurface_InterUtils.ComputeAppendPoint<TheCurve, TheCurveTool, TheSurface, TheSurfaceTool>(
                curve, lw, surface, su, sv, out IntCurveSurface_IntersectionPoint aPoint))
        {
            Append(aPoint);
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
