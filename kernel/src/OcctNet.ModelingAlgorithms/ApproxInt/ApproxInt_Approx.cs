// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_Approx.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.AppParCurves_Constraint;
using static Occt.Approx_ParametrizationType;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The static tool over a parametric surface that ApproxInt's generics use (OCCT's ThePSurfaceTool macro:
/// Adaptor3d_HSurfaceTool in GeomInt, BRepApprox_SurfaceTool in BRepApprox): IntImp's members, the surface's type
/// and its quadric, and its periods.
/// </summary>
/// <remarks>
/// Not an OCCT type (decision 607). Over Adaptor3d_Surface it is implemented by
/// <see cref="IntCurveSurface_TheHSurfaceTool"/>, over BRepAdaptor_Surface by <see cref="BRepApprox_SurfaceTool"/>.
/// </remarks>
/// <typeparam name="ThePSurface">The surface.</typeparam>
public interface IApproxInt_SurfaceTool<ThePSurface> : IIntImp_ZerCSParPSurfaceTool<ThePSurface>
{
    static abstract SurfaceKind GetType(in ThePSurface S);

    static abstract Plane3D Plane(in ThePSurface S);

    static abstract Cylinder Cylinder(in ThePSurface S);

    static abstract Sphere Sphere(in ThePSurface S);

    static abstract Cone Cone(in ThePSurface S);

    static abstract bool IsUPeriodic(in ThePSurface S);

    static abstract double UPeriod(in ThePSurface S);

    static abstract bool IsVPeriodic(in ThePSurface S);

    static abstract double VPeriod(in ThePSurface S);
}

/// <summary>
/// Approximates (part of) a walking line, the intersection of two surfaces, by a B-spline curve in 3D and B-spline
/// curves in the parameter spaces of the surfaces: Bezier pieces between knots found on the discrete curvature
/// (<see cref="ApproxInt_KnotTools"/>), joined into one B-spline, or one B-spline at once for short lines. The
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
public class ApproxInt_Approx<ThePSurface, ThePSurfaceTool, TheWLine>
    where ThePSurfaceTool : IApproxInt_SurfaceTool<ThePSurface>
    where TheWLine : class, IApproxInt_WLine<TheWLine>
{
    // If quantity of points is less than aMinNbPointsForApprox
    // then interpolation is used.
    private const int aMinNbPointsForApprox = 5;

    // This constant should be removed in the future.
    private const double RatioTol = 1.5;

    private struct Approx_Data
    {
        public Approx_Data()
        {
            myBezierApprox = true;
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
            myNbPntMax = 30;
            parametrization = Approx_ChordLength;
        }

        public bool myBezierApprox;
        public double Xo, Yo, Zo, U1o, V1o, U2o, V2o;
        public bool ApproxXYZ, ApproxU1V1, ApproxU2V2;
        public int indicemin, indicemax, myNbPntMax;
        public Approx_ParametrizationType parametrization;
    }

    private readonly Approx_BSplComputeLine<ApproxInt_MultiLine<TheWLine>, ApproxInt_MultiLineTool<TheWLine>>
        myComputeLine;
    private readonly Approx_ComputeLine<ApproxInt_MultiLine<TheWLine>, ApproxInt_MultiLineTool<TheWLine>>
        myComputeLineBezier;
    private readonly Approx_MCurvesToBSpCurve myBezToBSpl;
    private bool myWithTangency;
    private double myTol3d;
    private double myTol2d;
    private int myDegMin;
    private int myDegMax;
    private int myNbIterMax;
    private double myTolReached3d;
    private double myTolReached2d;
    private Approx_Data myData;
    private readonly NCollection_DynamicArray<int> myKnots;

    //=================================================================================================

    private static void ComputeTrsf3d(TheWLine theline, out double theXo, out double theYo, out double theZo)
    {
        int aNbPnts = theline.NbPnts();
        double aXmin = RealLast(), aYmin = RealLast(), aZmin = RealLast();
        for (int i = 1; i <= aNbPnts; i++)
        {
            Point3D P = theline.Point(i).Value();
            aXmin = Min(P.X(), aXmin);
            aYmin = Min(P.Y(), aYmin);
            aZmin = Min(P.Z(), aZmin);
        }

        theXo = -aXmin;
        theYo = -aYmin;
        theZo = -aZmin;
    }

    //=================================================================================================

    private static void ComputeTrsf2d(TheWLine theline, bool onFirst, out double theUo, out double theVo)
    {
        int aNbPnts = theline.NbPnts();
        double aUmin = RealLast(), aVmin = RealLast();

        for (int i = 1; i <= aNbPnts; i++)
        {
            IntSurf_PntOn2S POn2S = theline.Point(i);
            double U, V;
            if (onFirst)
            {
                POn2S.ParametersOnS1(out U, out V);
            }
            else
            {
                POn2S.ParametersOnS2(out U, out V);
            }
            aUmin = Min(U, aUmin);
            aVmin = Min(V, aVmin);
        }

        theUo = -aUmin;
        theVo = -aVmin;
    }

    /// <summary>
    /// The parameters of the points <paramref name="firstP"/> to <paramref name="lastP"/> of <paramref name="Line"/>
    /// for the parametrisation <paramref name="Par"/>, from 0 to 1, in <paramref name="TheParameters"/>.
    /// </summary>
    public static void Parameters(ApproxInt_MultiLine<TheWLine> Line, int firstP, int lastP,
                                  Approx_ParametrizationType Par, math_Vector TheParameters)
    {
        int i, j, nbP2d, nbP3d;
        double dist;

        if (Par == Approx_ChordLength || Par == Approx_Centripetal)
        {
            nbP3d = ApproxInt_MultiLineTool<TheWLine>.NbP3d(Line);
            nbP2d = ApproxInt_MultiLineTool<TheWLine>.NbP2d(Line);
            int mynbP3d = nbP3d, mynbP2d = nbP2d;
            if (nbP3d == 0)
            {
                mynbP3d = 1;
            }
            if (nbP2d == 0)
            {
                mynbP2d = 1;
            }

            TheParameters[firstP] = 0.0;
            dist = 0.0;
            NCollection_Array1<Point3D> tabP = new(1, mynbP3d);
            NCollection_Array1<Point3D> tabPP = new(1, mynbP3d);
            NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d);
            NCollection_Array1<Point2D> tabPP2d = new(1, mynbP2d);

            for (i = firstP + 1; i <= lastP; i++)
            {
                if (nbP3d != 0 && nbP2d != 0)
                {
                    ApproxInt_MultiLineTool<TheWLine>.Value(Line, i - 1, tabP, tabP2d);
                }
                else if (nbP2d != 0)
                {
                    ApproxInt_MultiLineTool<TheWLine>.Value(Line, i - 1, tabP2d);
                }
                else if (nbP3d != 0)
                {
                    ApproxInt_MultiLineTool<TheWLine>.Value(Line, i - 1, tabP);
                }

                if (nbP3d != 0 && nbP2d != 0)
                {
                    ApproxInt_MultiLineTool<TheWLine>.Value(Line, i, tabPP, tabPP2d);
                }
                else if (nbP2d != 0)
                {
                    ApproxInt_MultiLineTool<TheWLine>.Value(Line, i, tabPP2d);
                }
                else if (nbP3d != 0)
                {
                    ApproxInt_MultiLineTool<TheWLine>.Value(Line, i, tabPP);
                }
                dist = 0;
                for (j = 1; j <= nbP3d; j++)
                {
                    Point3D aP1 = tabP[j], aP2 = tabPP[j];
                    dist += aP2.SquareDistance(aP1);
                }
                for (j = 1; j <= nbP2d; j++)
                {
                    Point2D aP12d = tabP2d[j], aP22d = tabPP2d[j];

                    dist += aP22d.SquareDistance(aP12d);
                }

                dist = Math.Sqrt(dist);
                if (Par == Approx_ChordLength)
                {
                    TheParameters[i] = TheParameters[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    TheParameters[i] = TheParameters[i - 1] + Math.Sqrt(dist);
                }
            }
            for (i = firstP; i <= lastP; i++)
            {
                TheParameters[i] /= TheParameters[lastP];
            }
        }
        else
        {
            for (i = firstP; i <= lastP; i++)
            {
                TheParameters[i] = ((double)i - firstP) / ((double)lastP - (double)firstP);
            }
        }
    }

    //=================================================================================================

    public ApproxInt_Approx()
    {
        myComputeLine = new(4, 8, 0.001, 0.001, 5);
        myComputeLineBezier = new(4, 8, 0.001, 0.001, 5);
        myBezToBSpl = new();
        myWithTangency = true;
        myTol3d = 0.001;
        myTol2d = 0.001;
        myDegMin = 4;
        myDegMax = 8;
        myNbIterMax = 5;
        myTolReached3d = 0.0;
        myTolReached2d = 0.0;
        myData = new Approx_Data();
        myKnots = new();
        myComputeLine.SetContinuity(2);
        // myComputeLineBezier.SetContinuity(2);
    }

    /// <summary>
    /// Approximates the points <paramref name="indicemin"/> to <paramref name="indicemax"/> of the line alone,
    /// without the surfaces (no points are added): in 3D if <paramref name="ApproxXYZ"/>, and on the surfaces'
    /// parameter spaces if <paramref name="ApproxU1V1"/> and <paramref name="ApproxU2V2"/>.
    /// </summary>
    public void Perform(TheWLine theline, bool ApproxXYZ = true, bool ApproxU1V1 = true, bool ApproxU2V2 = true,
                        int indicemin = 0, int indicemax = 0)
    {
        // Prepare DS.
        prepareDS(ApproxXYZ, ApproxU1V1, ApproxU2V2, indicemin, indicemax);

        int nbpntbez = myData.indicemax - myData.indicemin;
        if (nbpntbez < aMinNbPointsForApprox)
        {
            myData.myBezierApprox = false;
        }
        else
        {
            myData.myBezierApprox = true;
        }

        // Fill data structure.
        fillData(theline);

        // Build knots.
        buildKnots(theline, null);
        if (myKnots.Length() == 2 && indicemax - indicemin > 2 * myData.myNbPntMax)
        {
            // At least 3 knots for BrepApprox.
            myKnots.ChangeLast() = (indicemax - indicemin) / 2; // OCCT: an index count, not indicemin + half of it
            myKnots.Append(indicemax);
        }

        myComputeLine.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, true, myData.parametrization);
        myComputeLineBezier.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, true, myData.parametrization);

        buildCurve(theline, null);
    }

    //=================================================================================================
    // function : Perform
    // purpose  : Definition of next steps according to surface types
    //            (i.e. coordination algorithm).
    //=================================================================================================

    /// <summary>
    /// Approximates the points <paramref name="indicemin"/> to <paramref name="indicemax"/> of the intersection
    /// line of <paramref name="Surf1"/> and <paramref name="Surf2"/>, adding points on the surfaces where needed:
    /// in 3D if <paramref name="ApproxXYZ"/>, and on the surfaces' parameter spaces if <paramref name="ApproxU1V1"/>
    /// and <paramref name="ApproxU2V2"/>.
    /// </summary>
    public void Perform(ThePSurface Surf1, ThePSurface Surf2, TheWLine theline, bool ApproxXYZ = true,
                        bool ApproxU1V1 = true, bool ApproxU2V2 = true, int indicemin = 0, int indicemax = 0)
    {
        myTolReached3d = myTolReached2d = 0.0;

        SurfaceKind typeS1 = ThePSurfaceTool.GetType(Surf1);
        SurfaceKind typeS2 = ThePSurfaceTool.GetType(Surf2);

        bool isQuadric = ((typeS1 == Plane) || (typeS1 == SurfaceKind.Cylinder) || (typeS1 == SurfaceKind.Sphere)
                          || (typeS1 == SurfaceKind.Cone) || (typeS2 == Plane) || (typeS2 == SurfaceKind.Cylinder)
                          || (typeS2 == SurfaceKind.Sphere) || (typeS2 == SurfaceKind.Cone));

        if (isQuadric)
        {
            IntSurf_Quadric Quad = new();
            bool SecondIsImplicit = false;
            switch (typeS1)
            {
                case Plane:
                    Quad.SetValue(ThePSurfaceTool.Plane(Surf1));
                    break;

                case SurfaceKind.Cylinder:
                    Quad.SetValue(ThePSurfaceTool.Cylinder(Surf1));
                    break;

                case SurfaceKind.Sphere:
                    Quad.SetValue(ThePSurfaceTool.Sphere(Surf1));
                    break;

                case SurfaceKind.Cone:
                    Quad.SetValue(ThePSurfaceTool.Cone(Surf1));
                    break;

                default:
                {
                    SecondIsImplicit = true;
                    switch (typeS2)
                    {
                        case Plane:
                            Quad.SetValue(ThePSurfaceTool.Plane(Surf2));
                            break;

                        case SurfaceKind.Cylinder:
                            Quad.SetValue(ThePSurfaceTool.Cylinder(Surf2));
                            break;

                        case SurfaceKind.Sphere:
                            Quad.SetValue(ThePSurfaceTool.Sphere(Surf2));
                            break;

                        case SurfaceKind.Cone:
                            Quad.SetValue(ThePSurfaceTool.Cone(Surf2));
                            break;

                        default:
                            break;
                    } // switch (typeS2)
                }

                    break;
            } // switch (typeS1)

            Perform(Quad, (SecondIsImplicit ? Surf1 : Surf2), theline, ApproxXYZ, ApproxU1V1, ApproxU2V2, indicemin,
                    indicemax, !SecondIsImplicit);

            return;
        }

        // Here, isQuadric == FALSE.

        // Prepare DS.
        prepareDS(ApproxXYZ, ApproxU1V1, ApproxU2V2, indicemin, indicemax);

        // Non-analytical case: Param-Param perform.
        ApproxInt_PrmPrmSvSurfaces<ThePSurface, ThePSurfaceTool> myPrmPrmSvSurfaces = new(Surf1, Surf2);

        int nbpntbez = indicemax - indicemin;

        if (nbpntbez < aMinNbPointsForApprox)
        {
            myData.myBezierApprox = false;
        }
        else
        {
            myData.myBezierApprox = true;
        }

        // Fill data structure.
        fillData(theline);

        bool cut = myData.myBezierApprox;
        ApproxInt_SvSurfaces ptrsvsurf = myPrmPrmSvSurfaces;

        // Build knots.
        buildKnots(theline, ptrsvsurf);

        myComputeLine.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, cut, myData.parametrization);
        myComputeLineBezier.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, cut, myData.parametrization);

        buildCurve(theline, ptrsvsurf);
    }

    //=================================================================================================
    // function : Perform
    // purpose  : Analytic-Param perform.
    //=================================================================================================

    private void Perform(in IntSurf_Quadric ISurf, ThePSurface PSurf, TheWLine theline, bool ApproxXYZ,
                         bool ApproxU1V1, bool ApproxU2V2, int indicemin, int indicemax, bool isTheQuadFirst)
    {
        // Prepare DS.
        prepareDS(ApproxXYZ, ApproxU1V1, ApproxU2V2, indicemin, indicemax);

        // Non-analytical case: Analytic-Param perform.
        ApproxInt_ImpPrmSvSurfaces<ThePSurface, ThePSurfaceTool> myImpPrmSvSurfaces =
            isTheQuadFirst ? new(ISurf, PSurf) : new(PSurf, ISurf);

        myImpPrmSvSurfaces.SetUseSolver(false);

        int nbpntbez = indicemax - indicemin;
        if (nbpntbez < aMinNbPointsForApprox)
        {
            myData.myBezierApprox = false;
        }
        else
        {
            myData.myBezierApprox = true;
        }

        bool cut = myData.myBezierApprox;
        ApproxInt_SvSurfaces ptrsvsurf = myImpPrmSvSurfaces;

        // Fill data structure.
        fillData(theline);

        // Build knots.
        buildKnots(theline, ptrsvsurf);

        myComputeLine.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, cut, myData.parametrization);
        myComputeLineBezier.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, cut, myData.parametrization);

        buildCurve(theline, ptrsvsurf);
    }

    /// <summary>
    /// Sets the tolerances (divided by 1.5 internally), the degrees, the number of iterations, the minimal number of
    /// points per knot interval, the tangency constraints at the ends and the parametrisation.
    /// </summary>
    public void SetParameters(double Tol3d, double Tol2d, int DegMin, int DegMax, int NbIterMax, int NbPntMax = 30,
                              bool ApproxWithTangency = true,
                              Approx_ParametrizationType Parametrization = Approx_ChordLength)
    {
        myData.myNbPntMax = NbPntMax;
        myWithTangency = ApproxWithTangency;
        myTol3d = Tol3d / RatioTol;
        myTol2d = Tol2d / RatioTol;
        myDegMin = DegMin;
        myDegMax = DegMax;
        myNbIterMax = NbIterMax;

        myComputeLine.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, true, Parametrization);
        myComputeLineBezier.Init(myDegMin, myDegMax, myTol3d, myTol2d, myNbIterMax, true, Parametrization);

        if (!ApproxWithTangency)
        {
            myComputeLine.SetConstraints(AppParCurves_PassPoint, AppParCurves_PassPoint);
            myComputeLineBezier.SetConstraints(AppParCurves_PassPoint, AppParCurves_PassPoint);
        }

        myData.myBezierApprox = true;
    }

    /// <summary>The number of MultiCurves: 1.</summary>
    public int NbMultiCurves() => 1;

    //=================================================================================================

    private void UpdateTolReached()
    {
        if (myData.myBezierApprox)
        {
            int NbCurves = myComputeLineBezier.NbMultiCurves();
            for (int ICur = 1; ICur <= NbCurves; ICur++)
            {
                myComputeLineBezier.Error(ICur, out double Tol3D, out double Tol2D);
                myTolReached3d = Max(myTolReached3d, Tol3D);
                myTolReached2d = Max(myTolReached2d, Tol2D);
            }
        }
        else
        {
            myComputeLine.Error(out myTolReached3d, out myTolReached2d);
        }
    }

    /// <summary>The largest 3D error of the approximation.</summary>
    public double TolReached3d() => myTolReached3d * RatioTol;

    /// <summary>The largest 2D error of the approximation.</summary>
    public double TolReached2d() => myTolReached2d * RatioTol;

    public bool IsDone()
    {
        if (myData.myBezierApprox)
        {
            return (myComputeLineBezier.NbMultiCurves() > 0);
        }
        else
        {
            return (myComputeLine.IsToleranceReached());
        }
    }

    /// <summary>The approximation (whatever the index): the 3D curve first, then the 2D curves.</summary>
    public AppParCurves_MultiBSpCurve Value(int Index)
    {
        if (myData.myBezierApprox)
        {
            return (myBezToBSpl.Value());
        }
        else
        {
            return (myComputeLine.Value());
        }
    }

    //=================================================================================================

    private void fillData(TheWLine theline)
    {
        if (myData.ApproxXYZ)
        {
            ComputeTrsf3d(theline, out myData.Xo, out myData.Yo, out myData.Zo);
        }
        else
        {
            myData.Xo = myData.Yo = myData.Zo = 0.0;
        }

        if (myData.ApproxU1V1)
        {
            ComputeTrsf2d(theline, true, out myData.U1o, out myData.V1o);
        }
        else
        {
            myData.U1o = myData.V1o = 0.0;
        }

        if (myData.ApproxU2V2)
        {
            ComputeTrsf2d(theline, false, out myData.U2o, out myData.V2o);
        }
        else
        {
            myData.U2o = myData.V2o = 0.0;
        }
    }

    //=================================================================================================

    private void prepareDS(bool theApproxXYZ, bool theApproxU1V1, bool theApproxU2V2, int theIndicemin,
                           int theIndicemax)
    {
        myTolReached3d = myTolReached2d = 0.0;
        myData.ApproxU1V1 = theApproxU1V1;
        myData.ApproxU2V2 = theApproxU2V2;
        myData.ApproxXYZ = theApproxXYZ;
        myData.indicemin = theIndicemin;
        myData.indicemax = theIndicemax;
        myData.parametrization = myComputeLineBezier.Parametrization();
    }

    //=================================================================================================

    private void buildKnots(TheWLine theline, ApproxInt_SvSurfaces? thePtrSVSurf)
    {
        myKnots.Clear();
        if (!myData.myBezierApprox)
        {
            myKnots.Append(myData.indicemin);
            myKnots.Append(myData.indicemax);
            return;
        }

        ApproxInt_MultiLine<TheWLine> aTestLine = new(theline, thePtrSVSurf, ((myData.ApproxXYZ) ? 1 : 0),
                                                      ((myData.ApproxU1V1) ? 1 : 0) + ((myData.ApproxU2V2) ? 1 : 0),
                                                      myData.ApproxU1V1, myData.ApproxU2V2, myData.Xo, myData.Yo,
                                                      myData.Zo, myData.U1o, myData.V1o, myData.U2o, myData.V2o,
                                                      myData.ApproxU1V1, myData.indicemin, myData.indicemax);

        int nbp3d = aTestLine.NbP3d(), nbp2d = aTestLine.NbP2d();
        NCollection_Array1<Point3D> aTabPnt3d = new(1, Max(1, nbp3d));
        NCollection_Array1<Point2D> aTabPnt2d = new(1, Max(1, nbp2d));
        NCollection_Array1<Point3D> aPntXYZ = new(myData.indicemin, myData.indicemax);
        NCollection_Array1<Point2D> aPntU1V1 = new(myData.indicemin, myData.indicemax);
        NCollection_Array1<Point2D> aPntU2V2 = new(myData.indicemin, myData.indicemax);

        for (int i = myData.indicemin; i <= myData.indicemax; ++i)
        {
            if (nbp3d != 0 && nbp2d != 0)
            {
                aTestLine.Value(i, aTabPnt3d, aTabPnt2d);
            }
            else if (nbp2d != 0)
            {
                aTestLine.Value(i, aTabPnt2d);
            }
            else if (nbp3d != 0)
            {
                aTestLine.Value(i, aTabPnt3d);
            }
            //
            if (nbp3d > 0)
            {
                aPntXYZ[i] = aTabPnt3d[1];
            }
            if (nbp2d > 1)
            {
                aPntU1V1[i] = aTabPnt2d[1];
                aPntU2V2[i] = aTabPnt2d[2];
            }
            else if (nbp2d > 0)
            {
                if (myData.ApproxU1V1)
                {
                    aPntU1V1[i] = aTabPnt2d[1];
                }
                else
                {
                    aPntU2V2[i] = aTabPnt2d[1];
                }
            }
        }

        int aMinNbPnts = myData.myNbPntMax;

        // Expected parametrization.
        math_Vector aPars = new(myData.indicemin, myData.indicemax);
        Parameters(aTestLine, myData.indicemin, myData.indicemax, myData.parametrization, aPars);

        ApproxInt_KnotTools.BuildKnots(aPntXYZ, aPntU1V1, aPntU2V2, aPars, myData.ApproxXYZ, myData.ApproxU1V1,
                                       myData.ApproxU2V2, aMinNbPnts, myKnots);
    }

    //=================================================================================================

    private void buildCurve(TheWLine theline, ApproxInt_SvSurfaces? thePtrSVSurf)
    {
        if (myData.myBezierApprox)
        {
            myBezToBSpl.Reset();
        }

        int kind = myKnots.Lower();
        int imin = 0, imax = 0;
        bool OtherInter = false;
        do
        {
            // Base cycle: iterate over knots.
            imin = myKnots[kind];
            imax = myKnots[kind + 1];
            ApproxInt_MultiLine<TheWLine> myMultiLine = new(theline, thePtrSVSurf, ((myData.ApproxXYZ) ? 1 : 0),
                                                            ((myData.ApproxU1V1) ? 1 : 0)
                                                                + ((myData.ApproxU2V2) ? 1 : 0),
                                                            myData.ApproxU1V1, myData.ApproxU2V2, myData.Xo,
                                                            myData.Yo, myData.Zo, myData.U1o, myData.V1o, myData.U2o,
                                                            myData.V2o, myData.ApproxU1V1, imin, imax);

            if (myData.myBezierApprox)
            {
                myComputeLineBezier.Perform(myMultiLine);
                if (myComputeLineBezier.NbMultiCurves() == 0)
                {
                    return;
                }
            }
            else
            {
                myComputeLine.Perform(myMultiLine);
            }

            UpdateTolReached();

            int indice3d = 1, indice2d1 = 2, indice2d2 = 3;
            if (!myData.ApproxXYZ)
            {
                indice2d1--;
                indice2d2--;
            }
            if (!myData.ApproxU1V1)
            {
                indice2d2--;
            }
            if (myData.ApproxXYZ)
            {
                if (myData.myBezierApprox)
                {
                    for (int nbmc = myComputeLineBezier.NbMultiCurves(); nbmc >= 1; nbmc--)
                    {
                        myComputeLineBezier.ChangeValue(nbmc)
                            .Transform(indice3d, -myData.Xo, 1.0, -myData.Yo, 1.0, -myData.Zo, 1.0);
                    }
                }
                else
                {
                    myComputeLine.ChangeValue().Transform(indice3d, -myData.Xo, 1.0, -myData.Yo, 1.0, -myData.Zo, 1.0);
                }
            }
            if (myData.ApproxU1V1)
            {
                if (myData.myBezierApprox)
                {
                    for (int nbmc = myComputeLineBezier.NbMultiCurves(); nbmc >= 1; nbmc--)
                    {
                        myComputeLineBezier.ChangeValue(nbmc).Transform2d(indice2d1, -myData.U1o, 1.0, -myData.V1o,
                                                                          1.0);
                    }
                }
                else
                {
                    myComputeLine.ChangeValue().Transform2d(indice2d1, -myData.U1o, 1.0, -myData.V1o, 1.0);
                }
            }
            if (myData.ApproxU2V2)
            {
                if (myData.myBezierApprox)
                {
                    for (int nbmc = myComputeLineBezier.NbMultiCurves(); nbmc >= 1; nbmc--)
                    {
                        myComputeLineBezier.ChangeValue(nbmc).Transform2d(indice2d2, -myData.U2o, 1.0, -myData.V2o,
                                                                          1.0);
                    }
                }
                else
                {
                    myComputeLine.ChangeValue().Transform2d(indice2d2, -myData.U2o, 1.0, -myData.V2o, 1.0);
                }
            }

            OtherInter = false;
            if (myData.myBezierApprox)
            {
                for (int nbmc = 1; nbmc <= myComputeLineBezier.NbMultiCurves(); nbmc++)
                {
                    myBezToBSpl.Append(myComputeLineBezier.Value(nbmc));
                }
                kind++;
                if (kind < myKnots.Upper())
                {
                    OtherInter = true;
                }
            }
        } while (OtherInter);

        if (myData.myBezierApprox)
        {
            myBezToBSpl.Perform();
        }
    }
}
