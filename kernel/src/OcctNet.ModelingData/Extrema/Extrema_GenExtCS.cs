// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The extrema of the distance between a curve and a surface in the general case: a global minimum search by
/// particle swarm optimisation (math_PSO) over sampled points, over (t, u, v), over (u, v) for a conic or over t for
/// a quadric, refined by math_FunctionSetRoot on <see cref="Extrema_FuncExtCS"/>.
/// </summary>
/// <remarks>
/// The surface (set by <see cref="Initialize(Adaptor3d_Surface, int, int, double)"/>) and the curve (given to
/// <see cref="Perform(Adaptor3d_Curve, int, double)"/>) are kept by reference, as OCCT keeps them: they must outlive
/// the algorithm. Infinite ranges are bounded at 1e10 (23 on hyperbolas and surfaces built on them).
/// </remarks>
public class Extrema_GenExtCS
{
    private const double MaxParamVal = 1.0e+10;
    private const double aBorderDivisor = 1.0e+4;
    private const double HyperbolaLimit = 23.0; // ln(MaxParamVal)

    private bool myDone;
    private double mytmin;
    private double mytsup;
    private double myumin;
    private double myusup;
    private double myvmin;
    private double myvsup;
    private int mytsample;
    private int myusample;
    private int myvsample;
    private double mytol1;
    private double mytol2;
    private readonly Extrema_FuncExtCS myF = new();
    private Adaptor3d_Surface? myS;
    private NCollection_HArray2<Point3D>? mySurfPnts;

    private static bool IsQuadric(SurfaceKind theSType)
    {
        if (theSType == Plane)
        {
            return true;
        }
        if (theSType == SurfaceKind.Cylinder)
        {
            return true;
        }
        if (theSType == SurfaceKind.Cone)
        {
            return true;
        }
        if (theSType == SurfaceKind.Sphere)
        {
            return true;
        }
        if (theSType == SurfaceKind.Torus)
        {
            return true;
        }
        return false;
    }

    private static bool IsConic(CurveKind theCType)
    {
        if (theCType == Line)
        {
            return true;
        }
        if (theCType == Circle)
        {
            return true;
        }
        if (theCType == Ellipse)
        {
            return true;
        }
        if (theCType == Hyperbola)
        {
            return true;
        }
        if (theCType == Parabola)
        {
            return true;
        }
        return false;
    }

    // restrict maximal parameter on hyperbola to avoid FPE
    private static double GetCurvMaxParamVal(Adaptor3d_Curve theC)
    {
        if (theC.GetType() == Hyperbola)
        {
            return HyperbolaLimit;
        }
        if (theC.GetType() == OffsetCurve)
        {
            Geom_Curve aBC = theC.OffsetCurve().BasisCurve();
            Geom_TrimmedCurve? aTC = aBC as Geom_TrimmedCurve;
            if (aTC is not null)
            {
                aBC = aTC.BasisCurve();
            }
            if (aBC is Geom_Hyperbola)
            {
                return HyperbolaLimit;
            }
        }
        return MaxParamVal;
    }

    // restrict maximal parameter on surfaces based on hyperbola to avoid FPE
    private static void GetSurfMaxParamVals(Adaptor3d_Surface theS, out double theUmax, out double theVmax)
    {
        theUmax = theVmax = MaxParamVal;

        if (theS.GetType() == SurfaceOfExtrusion)
        {
            theUmax = GetCurvMaxParamVal(theS.BasisCurve());
        }
        else if (theS.GetType() == SurfaceOfRevolution)
        {
            theVmax = GetCurvMaxParamVal(theS.BasisCurve());
        }
        else if (theS.GetType() == OffsetSurface)
        {
            GetSurfMaxParamVals(theS.BasisSurface(), out theUmax, out theVmax);
        }
    }

    /// <summary>An empty algorithm.</summary>
    public Extrema_GenExtCS()
    {
        myDone = false;
        mytmin = 0.0;
        mytsup = 0.0;
        myumin = 0.0;
        myusup = 0.0;
        myvmin = 0.0;
        myvsup = 0.0;
        mytsample = 0;
        myusample = 0;
        myvsample = 0;
        mytol1 = 0.0;
        mytol2 = 0.0;
        myS = null;
    }

    /// <summary>
    /// Computes all the extrema between C and S over their whole ranges. NbU and NbV sample the surface, NbT the
    /// curve, to locate the starting points.
    /// </summary>
    public Extrema_GenExtCS(Adaptor3d_Curve C, Adaptor3d_Surface S, int NbT, int NbU, int NbV, double Tol1, double Tol2)
    {
        Initialize(S, NbU, NbV, Tol2);
        Perform(C, NbT, Tol1);
    }

    /// <summary>Computes all the extrema between C on [tmin, tsup] and S on [Umin, Usup] x [Vmin, Vsup].</summary>
    public Extrema_GenExtCS(Adaptor3d_Curve C, Adaptor3d_Surface S, int NbT, int NbU, int NbV, double tmin,
        double tsup, double Umin, double Usup, double Vmin, double Vsup, double Tol1, double Tol2)
    {
        Initialize(S, NbU, NbV, Umin, Usup, Vmin, Vsup, Tol2);
        Perform(C, NbT, tmin, tsup, Tol1);
    }

    /// <summary>Sets the surface over its whole range and samples it.</summary>
    public void Initialize(Adaptor3d_Surface S, int NbU, int NbV, double Tol2)
    {
        myumin = S.FirstUParameter();
        myusup = S.LastUParameter();
        myvmin = S.FirstVParameter();
        myvsup = S.LastVParameter();
        Initialize(S, NbU, NbV, myumin, myusup, myvmin, myvsup, Tol2);
    }

    /// <summary>Sets the surface over [Umin, Usup] x [Vmin, Vsup] and samples it.</summary>
    public void Initialize(Adaptor3d_Surface S, int NbU, int NbV, double Umin, double Usup, double Vmin, double Vsup,
        double Tol2)
    {
        myS = S;
        myusample = NbU;
        myvsample = NbV;
        myumin = Umin;
        myusup = Usup;
        myvmin = Vmin;
        myvsup = Vsup;
        mytol2 = Tol2;

        GetSurfMaxParamVals(myS, out double umaxpar, out double vmaxpar);

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myusup))
        {
            myusup = umaxpar;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myumin))
        {
            myumin = -umaxpar;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myvsup))
        {
            myvsup = vmaxpar;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myvmin))
        {
            myvmin = -vmaxpar;
        }

        double du = (myusup - myumin) / aBorderDivisor;
        double dv = (myvsup - myvmin) / aBorderDivisor;
        double aMinU = myumin + du;
        double aMinV = myvmin + dv;
        double aMaxU = myusup - du;
        double aMaxV = myvsup - dv;

        double aStepSU = (aMaxU - aMinU) / myusample;
        double aStepSV = (aMaxV - aMinV) / myvsample;

        mySurfPnts = new NCollection_HArray2<Point3D>(0, myusample, 0, myvsample);

        // Build UV parameter arrays for batch evaluation
        NCollection_Array1<double> aUParams = new(0, myusample);
        NCollection_Array1<double> aVParams = new(0, myvsample);

        double aSU = aMinU;
        for (int aSUI = 0; aSUI <= myusample; aSUI++, aSU += aStepSU)
        {
            aUParams.SetValue(aSUI, aSU);
        }
        double aSV = aMinV;
        for (int aSVI = 0; aSVI <= myvsample; aSVI++, aSV += aStepSV)
        {
            aVParams.SetValue(aSVI, aSV);
        }

        // Use batch grid evaluation for optimized surface point computation
        GeomGridEval_Surface anEvaluator = new(myS);

        NCollection_Array2<Point3D> aGrid = anEvaluator.EvaluateGrid(aUParams, aVParams);

        // aGrid is 1-based, mySurfPnts is 0-based - adjust indexing
        for (int aSUI = 0; aSUI <= myusample; aSUI++)
        {
            for (int aSVI = 0; aSVI <= myvsample; aSVI++)
            {
                mySurfPnts.ChangeValue(aSUI, aSVI) = aGrid.Value(aSUI + 1, aSVI + 1);
            }
        }
    }

    /// <summary>Computes the extrema with C over its whole range (the surface must have been initialised).</summary>
    public void Perform(Adaptor3d_Curve C, int NbT, double Tol1)
    {
        mytmin = C.FirstParameter();
        mytsup = C.LastParameter();
        Perform(C, NbT, mytmin, mytsup, Tol1);
    }

    /// <summary>Computes the extrema with C over [tmin, tsup] (the surface must have been initialised).</summary>
    public void Perform(Adaptor3d_Curve C, int NbT, double tmin, double tsup, double Tol1)
    {
        myDone = false;
        if (myS is null)
        {
            throw new NotDoneException("Extrema_GenExtCS: the surface is not initialized");
        }
        myF.Initialize(C, myS);
        mytmin = tmin;
        mytsup = tsup;
        mytol1 = Tol1;
        mytsample = NbT;
        // Modification by lvt to trim the surface not to infinity but to +/- 10000

        double trimusup = myusup, trimumin = myumin, trimvsup = myvsup, trimvmin = myvmin;
        double aCMaxVal = GetCurvMaxParamVal(C);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(mytsup))
        {
            mytsup = aCMaxVal;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(mytmin))
        {
            mytmin = -aCMaxVal;
        }
        //
        int aNbVar = 3;
        SurfaceKind aSType = myS.GetType();
        if (IsQuadric(aSType))
        {
            aNbVar = 1;
        }
        else
        {
            CurveKind aCType = C.GetType();
            if (IsConic(aCType))
            {
                aNbVar = 2;
            }
        }

        math_Vector Tol = new(1, 3), TUV = new(1, 3), TUVinf = new(1, 3), TUVsup = new(1, 3);
        //
        Tol[1] = mytol1;
        Tol[2] = mytol2;
        Tol[3] = mytol2;
        //
        // Number of particles used in PSO algorithm (particle swarm optimization).
        const int aNbParticles = 48;

        int aNbIntC = 1;
        if (C.IsClosed() || C.IsPeriodic())
        {
            double aPeriod = C.Period();
            if (C.LastParameter() - C.FirstParameter() > 2.0 * aPeriod / 3.0)
            {
                aNbIntC = 2;
            }
        }

        int anInt;
        double dT = (mytsup - mytmin) / aNbIntC;
        for (anInt = 1; anInt <= aNbIntC; anInt++)
        {
            TUVinf[1] = mytmin + (anInt - 1) * dT;
            TUVinf[2] = trimumin;
            TUVinf[3] = trimvmin;
            //
            TUVsup[1] = TUVinf[1] + dT; // mytsup;
            TUVsup[2] = trimusup;
            TUVsup[3] = trimvsup;
            //
            if (aNbVar == 3)
            {
                GlobMinGenCS(C, aNbParticles, TUVinf, TUVsup, TUV);
            }
            else if (aNbVar == 2)
            {
                GlobMinConicS(C, aNbParticles, TUVinf, TUVsup, TUV);
            }
            else
            {
                GlobMinCQuadric(C, aNbParticles, TUVinf, TUVsup, TUV);
            }

            // Find min approximation
            FunctionSetRoot anA = new(myF, Tol);
            anA.Perform(myF, TUV, TUVinf, TUVsup);
        }
        if (aNbIntC > 1 && myF.NbExt() > 1)
        {
            // Try to remove "false" extrema caused by dividing curve interval
            NCollection_Sequence<double> aSqDists = myF.SquareDistances();
            NCollection_Sequence<Extrema_POnCurv> aPntsOnCrv = myF.PointsOnCurve();
            NCollection_Sequence<Extrema_POnSurf> aPntsOnSurf = myF.PointsOnSurf();
            NCollection_Sequence<double> aSqDists1 = new(aSqDists);
            NCollection_Sequence<Extrema_POnCurv> aPntsOnCrv1 = new(aPntsOnCrv);
            NCollection_Sequence<Extrema_POnSurf> aPntsOnSurf1 = new(aPntsOnSurf);

            double aMinDist = aSqDists[1];
            int i;
            for (i = 2; i <= aSqDists.Length(); ++i)
            {
                double aDist = aSqDists[i];
                if (aDist < aMinDist)
                {
                    aMinDist = aDist;
                }
            }
            aSqDists.Clear();
            aPntsOnCrv.Clear();
            aPntsOnSurf.Clear();
            double aTol = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
            for (i = 1; i <= aSqDists1.Length(); ++i)
            {
                double aDist = aSqDists1[i];
                if (Math.Abs(aDist - aMinDist) <= aTol)
                {
                    aSqDists.Append(aDist);
                    aPntsOnCrv.Append(aPntsOnCrv1[i]);
                    aPntsOnSurf.Append(aPntsOnSurf1[i]);
                }
            }
        }
        myDone = true;
    }

    private void GlobMinGenCS(Adaptor3d_Curve theC, int theNbParticles, math_Vector theTUVinf, math_Vector theTUVsup,
        math_Vector theTUV)
    {
        PsoParticlesPool aParticles = new(theNbParticles, 3);

        math_Vector aMinTUV = new(1, 3);
        aMinTUV.Initialized(theTUVinf + (theTUVsup - theTUVinf) / aBorderDivisor);

        math_Vector aMaxTUV = new(1, 3);
        aMaxTUV.Initialized(theTUVsup - (theTUVsup - theTUVinf) / aBorderDivisor);

        double aStepCU = (aMaxTUV[1] - aMinTUV[1]) / mytsample;
        double aStepSU = (aMaxTUV[2] - aMinTUV[2]) / myusample;
        double aStepSV = (aMaxTUV[3] - aMinTUV[3]) / myvsample;

        // Correct number of curve samples in case of low resolution
        int aNewCsample = mytsample;
        double aScaleFactor = 5.0;
        double aResolutionCU = aStepCU / theC.Resolution(1.0);

        double aMinResolution = aScaleFactor
                                * Min(aResolutionCU,
                                    Min(aStepSU / myS!.UResolution(1.0), aStepSV / myS.VResolution(1.0)));

        if (aMinResolution > Epsilon(1.0))
        {
            if (aResolutionCU > aMinResolution)
            {
                const int aMaxNbNodes = 50;

                aNewCsample = Min(aMaxNbNodes, RealToInt(mytsample * aResolutionCU / aMinResolution));

                aStepCU = (aMaxTUV[1] - aMinTUV[1]) / aNewCsample;
            }
        }

        // Pre-compute curve sample points.
        NCollection_Array1<Point3D> aCurvPnts = new(0, aNewCsample);

        double aCU1 = aMinTUV[1];
        for (int aCUI = 0; aCUI <= aNewCsample; aCUI++, aCU1 += aStepCU)
        {
            aCurvPnts.SetValue(aCUI, theC.Value(aCU1));
        }

        Particle aParticle = aParticles.GetWorstParticle();
        // Select specified number of particles from pre-computed set of samples
        double aSU = aMinTUV[2];
        for (int aSUI = 0; aSUI <= myusample; aSUI++, aSU += aStepSU)
        {
            double aSV = aMinTUV[3];
            for (int aSVI = 0; aSVI <= myvsample; aSVI++, aSV += aStepSV)
            {
                double aCU2 = aMinTUV[1];
                for (int aCUI = 0; aCUI <= aNewCsample; aCUI++, aCU2 += aStepCU)
                {
                    double aSqDist = mySurfPnts!.Value(aSUI, aSVI).SquareDistance(aCurvPnts.Value(aCUI));

                    if (aSqDist < aParticle.Distance)
                    {
                        aParticle.Position[0] = aCU2;
                        aParticle.Position[1] = aSU;
                        aParticle.Position[2] = aSV;

                        aParticle.BestPosition[0] = aCU2;
                        aParticle.BestPosition[1] = aSU;
                        aParticle.BestPosition[2] = aSV;

                        aParticle.Distance = aSqDist;
                        aParticle.BestDistance = aSqDist;

                        aParticle = aParticles.GetWorstParticle();
                    }
                }
            }
        }

        math_Vector aStep = new(1, 3);
        aStep[1] = aStepCU;
        aStep[2] = aStepSU;
        aStep[3] = aStepSV;

        // Find min approximation
        Extrema_GlobOptFuncCS aFunc = new(theC, myS);
        Pso aPSO = new(aFunc, theTUVinf, theTUVsup, aStep);
        aPSO.Perform(aParticles, theNbParticles, out _, theTUV);
    }

    private void GlobMinConicS(Adaptor3d_Curve theC, int theNbParticles, math_Vector theTUVinf, math_Vector theTUVsup,
        math_Vector theTUV)
    {
        int aNbVar = 2;
        math_Vector anUVinf = new(1, aNbVar), anUVsup = new(1, aNbVar), anUV = new(1, aNbVar);
        int i;
        for (i = 1; i <= aNbVar; ++i)
        {
            anUVinf[i] = theTUVinf[i + 1];
            anUVsup[i] = theTUVsup[i + 1];
        }
        //
        //
        PsoParticlesPool aParticles = new(theNbParticles, aNbVar);

        math_Vector aMinUV = new(1, aNbVar);
        aMinUV.Initialized(anUVinf + (anUVsup - anUVinf) / aBorderDivisor);

        math_Vector aMaxUV = new(1, aNbVar);
        aMaxUV.Initialized(anUVsup - (anUVsup - anUVinf) / aBorderDivisor);

        // Increase numbers of UV samples to improve searching global minimum
        int anAddsample = Max(mytsample / 2, 3);
        int anUsample = myusample + anAddsample;
        int aVsample = myvsample + anAddsample;
        //
        double aStepSU = (aMaxUV[1] - aMinUV[1]) / anUsample;
        double aStepSV = (aMaxUV[2] - aMinUV[2]) / aVsample;
        //
        Extrema_GlobOptFuncConicS aFunc = new(myS!, anUVinf[1], anUVsup[1], anUVinf[2], anUVsup[2]);
        aFunc.LoadConic(theC, theTUVinf[1], theTUVsup[1]);

        Particle aParticle = aParticles.GetWorstParticle();
        // Select specified number of particles from pre-computed set of samples
        double aSU = aMinUV[1];

        for (int aSUI = 0; aSUI <= anUsample; aSUI++, aSU += aStepSU)
        {
            anUV[1] = aSU;
            double aSV = aMinUV[2];
            for (int aSVI = 0; aSVI <= aVsample; aSVI++, aSV += aStepSV)
            {
                anUV[2] = aSV;
                if (!aFunc.Value(anUV, out double aSqDist))
                {
                    aSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                }

                if (aSqDist < aParticle.Distance)
                {
                    aParticle.Position[0] = aSU;
                    aParticle.Position[1] = aSV;

                    aParticle.BestPosition[0] = aSU;
                    aParticle.BestPosition[1] = aSV;

                    aParticle.Distance = aSqDist;
                    aParticle.BestDistance = aSqDist;

                    aParticle = aParticles.GetWorstParticle();
                }
            }
        }

        math_Vector aStep = new(1, aNbVar);
        aStep[1] = aStepSU;
        aStep[2] = aStepSV;

        // Find min approximation
        Pso aPSO = new(aFunc, anUVinf, anUVsup, aStep);
        aPSO.Perform(aParticles, theNbParticles, out _, anUV);
        //
        double aCT = aFunc.ConicParameter(anUV);
        if (theC.IsPeriodic())
        {
            if (aCT < theTUVinf[1] - global::Kerf.Engine.Foundation.Precision.PConfusion() || aCT > theTUVsup[1] + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                aCT = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aCT, theTUVinf[1], theTUVinf[1] + 2.0 * M_PI);
            }
        }

        theTUV[1] = aCT;
        theTUV[2] = anUV[1];
        theTUV[3] = anUV[2];

        bool isBadSol = false;
        myS!.D1(anUV[1], anUV[2], out Point3D aPOnS, out Vector3D aDU, out Vector3D aDV);
        theC.D1(aCT, out Point3D aPOnC, out Vector3D aDT);
        double aSqDist2 = aPOnC.SquareDistance(aPOnS);
        if (aSqDist2 <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return;
        }

        Vector3D aN = aDU.Crossed(aDV);
        if (aN.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return;
        }

        Vector3D PcPs = new(aPOnC, aPOnS);

        double anAngMin = M_PI_2 - M_PI_2 / 10.0;
        double anAngMax = M_PI_2 + M_PI_2 / 10.0;

        double anAngN = PcPs.Angle(aN);
        if (anAngN >= anAngMin && anAngN <= anAngMax)
        {
            // PcPs is perpendicular to surface normal, it means that
            // aPOnC can be on surface, but far from aPOnS
            isBadSol = true;
            int iu, iv;
            for (iu = -1; iu <= 1; ++iu)
            {
                double u = anUV[1] + iu * aStepSU;
                u = Max(anUVinf[1], u);
                u = Min(anUVsup[1], u);
                for (iv = -1; iv <= 1; ++iv)
                {
                    double v = anUV[2] + iv * aStepSV;
                    v = Max(anUVinf[2], v);
                    v = Min(anUVsup[2], v);
                    myS.D1(u, v, out aPOnS, out aDU, out aDV);
                    if (aPOnC.SquareDistance(aPOnS) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        isBadSol = false;
                        break;
                    }
                    aN = aDU.Crossed(aDV);
                    if (aN.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        isBadSol = false;
                        break;
                    }
                    PcPs.SetXYZ(aPOnS.XYZ() - aPOnC.XYZ());
                    anAngN = PcPs.Angle(aN);
                    if (anAngN < anAngMin || anAngN > anAngMax)
                    {
                        isBadSol = false;
                        break;
                    }
                }
                if (!isBadSol)
                {
                    break;
                }
            }
        }

        if (isBadSol)
        {
            // Try to precise solution with help of Extrema PS

            math_Vector aF = new(1, 3);
            aF[1] = PcPs.Dot(aDT);
            aF[2] = PcPs.Dot(aDU);
            aF[3] = PcPs.Dot(aDV);
            double aFF = aF.Norm2();

            Extrema_GenLocateExtPS anExtPS = new(myS, mytol2, mytol2);
            anExtPS.Perform(aPOnC, anUV[1], anUV[2], false);
            if (anExtPS.IsDone())
            {
                Extrema_POnSurf aPmin = anExtPS.Point();
                aPmin.Parameter(out anUV[1], out anUV[2]);
                math_Vector aTUV = new(theTUV);
                aTUV[2] = anUV[1];
                aTUV[3] = anUV[2];
                myF.Value(aTUV, aF);
                double aFF1 = aF.Norm2();

                if (anExtPS.SquareDistance() < aSqDist2 && aFF1 <= 1.1 * aFF)
                {
                    theTUV[2] = aTUV[2];
                    theTUV[3] = aTUV[3];
                }
            }
        }
    }

    private void GlobMinCQuadric(Adaptor3d_Curve theC, int theNbParticles, math_Vector theTUVinf,
        math_Vector theTUVsup, math_Vector theTUV)
    {
        int aNbVar = 1;
        math_Vector aTinf = new(1, aNbVar), aTsup = new(1, aNbVar), aT = new(1, aNbVar);
        aTinf[1] = theTUVinf[1];
        aTsup[1] = theTUVsup[1];
        //
        PsoParticlesPool aParticles = new(theNbParticles, aNbVar);

        math_Vector aMinT = new(1, aNbVar);
        aMinT.Initialized(aTinf + (aTsup - aTinf) / aBorderDivisor);

        math_Vector aMaxT = new(1, aNbVar);
        aMaxT.Initialized(aTsup - (aTsup - aTinf) / aBorderDivisor);
        //

        // Increase numbers of curve samples to improve searching global minimum
        // because dimension of optimisation task is reduced
        const int aMaxNbNodes = 50;
        int aNewCsample = mytsample;
        int anAddsample = Max(myusample / 2, 3);
        aNewCsample += anAddsample;
        aNewCsample = Min(aNewCsample, aMaxNbNodes);
        //
        // Correct number of curve samples in case of low resolution
        double aStepCT = (aMaxT[1] - aMinT[1]) / aNewCsample;
        double aStepSU = (theTUVsup[2] - theTUVinf[2]) / myusample;
        double aStepSV = (theTUVsup[3] - theTUVinf[3]) / myvsample;
        double aScaleFactor = 5.0;
        double aResolutionCU = aStepCT / theC.Resolution(1.0);

        double aMinResolution = aScaleFactor
                                * Min(aResolutionCU,
                                    Min(aStepSU / myS!.UResolution(1.0), aStepSV / myS.VResolution(1.0)));

        if (aMinResolution > Epsilon(1.0))
        {
            if (aResolutionCU > aMinResolution)
            {
                aNewCsample = Min(aMaxNbNodes, RealToInt(aNewCsample * aResolutionCU / aMinResolution));

                aStepCT = (aMaxT[1] - aMinT[1]) / aNewCsample;
            }
        }

        //
        Extrema_GlobOptFuncCQuadric aFunc = new(theC, aTinf[1], aTsup[1]);
        aFunc.LoadQuad(myS, theTUVinf[2], theTUVsup[2], theTUVinf[3], theTUVsup[3]);

        Particle aParticle = aParticles.GetWorstParticle();
        // Select specified number of particles from pre-computed set of samples
        double aCT = aMinT[1];
        for (int aCUI = 0; aCUI <= aNewCsample; aCUI++, aCT += aStepCT)
        {
            aT[1] = aCT;
            if (!aFunc.Value(aT, out double aSqDist))
            {
                aSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
            }

            if (aSqDist < aParticle.Distance)
            {
                aParticle.Position[0] = aCT;

                aParticle.BestPosition[0] = aCT;

                aParticle.Distance = aSqDist;
                aParticle.BestDistance = aSqDist;

                aParticle = aParticles.GetWorstParticle();
            }
        }
        //
        math_Vector aStep = new(1, aNbVar);
        aStep[1] = aStepCT;

        // Find min approximation
        Pso aPSO = new(aFunc, aTinf, aTsup, aStep);
        aPSO.Perform(aParticles, theNbParticles, out _, aT);
        //
        math_Vector anUV = new(1, 2);
        aFunc.QuadricParameters(aT, anUV);
        if (myS.IsUPeriodic())
        {
            if (anUV[1] < theTUVinf[2] - global::Kerf.Engine.Foundation.Precision.PConfusion() || anUV[1] > theTUVsup[2] + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                anUV[1] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anUV[1], theTUVinf[2], theTUVinf[2] + 2.0 * M_PI);
            }
        }
        //
        if (myS.IsVPeriodic())
        {
            if (anUV[2] < theTUVinf[3] - global::Kerf.Engine.Foundation.Precision.PConfusion() || anUV[2] > theTUVsup[3] + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                anUV[2] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anUV[2], theTUVinf[3], theTUVinf[3] + 2.0 * M_PI);
            }
        }
        //
        theTUV[1] = aT[1];
        theTUV[2] = anUV[1];
        theTUV[3] = anUV[2];
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of extrema.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myF.NbExt();
    }

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int N)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        return myF.SquareDistance(N);
    }

    /// <summary>The point on the curve of the Nth extremum.</summary>
    public Extrema_POnCurv PointOnCurve(int N)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        return myF.PointOnCurve(N);
    }

    /// <summary>The point on the surface of the Nth extremum.</summary>
    public Extrema_POnSurf PointOnSurface(int N)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        return myF.PointOnSurface(N);
    }
}
