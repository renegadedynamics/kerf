// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The extrema of the distance between two surfaces in the general case: the closest and the farthest pairs of
/// sample points of NbU x NbV grids on both surfaces are refined, the minimum by math_BFGS on the square distance
/// (math_FunctionSetRoot if BFGS fails), the maximum by math_FunctionSetRoot on <see cref="Extrema_FuncExtSS"/>.
/// </summary>
/// <remarks>
/// The surfaces are kept by reference, as OCCT keeps them: they must outlive the algorithm. NbU and NbV must be at
/// least 2.
/// </remarks>
public class Extrema_GenExtSS
{
    // This class represents distance objective function for surface / surface.
    private sealed class Extrema_FuncDistSS : MultipleVarFunctionWithGradient
    {
        private readonly Adaptor3d_Surface myS1;
        private readonly Adaptor3d_Surface myS2;

        public Extrema_FuncDistSS(Adaptor3d_Surface S1, Adaptor3d_Surface S2)
        {
            myS1 = S1;
            myS2 = S2;
        }

        public override int VariableCount => 4;

        public override bool Value(math_Vector X, out double F)
        {
            F = myS1.Value(X[1], X[2]).SquareDistance(myS2.Value(X[3], X[4]));
            return true;
        }

        public override bool Gradient(math_Vector X, math_Vector G)
        {
            myS1.D1(X[1], X[2], out Point3D P1, out Vector3D Du1s1, out Vector3D Dv1s1);
            myS2.D1(X[3], X[4], out Point3D P2, out Vector3D Du2s2, out Vector3D Dv2s2);

            Vector3D P1P2 = new(P2, P1);

            G[1] = P1P2.Dot(Du1s1);
            G[2] = P1P2.Dot(Dv1s1);
            G[3] = -P1P2.Dot(Du2s2);
            G[4] = -P1P2.Dot(Dv2s2);

            return true;
        }

        public override bool Values(math_Vector X, out double F, math_Vector G)
        {
            F = myS1.Value(X[1], X[2]).SquareDistance(myS2.Value(X[3], X[4]));

            myS1.D1(X[1], X[2], out Point3D P1, out Vector3D Du1s1, out Vector3D Dv1s1);
            myS2.D1(X[3], X[4], out Point3D P2, out Vector3D Du2s2, out Vector3D Dv2s2);

            Vector3D P1P2 = new(P2, P1);

            G[1] = P1P2.Dot(Du1s1);
            G[2] = P1P2.Dot(Dv1s1);
            G[3] = -P1P2.Dot(Du2s2);
            G[4] = -P1P2.Dot(Dv2s2);

            return true;
        }
    }

    private bool myDone;
#pragma warning disable CS0414 // set and never read in OCCT; kept (decision 221)
    private bool myInit;
#pragma warning restore CS0414
    private double myu1min;
    private double myu1sup;
    private double myv1min;
    private double myv1sup;
    private double myu2min;
    private double myu2sup;
    private double myv2min;
    private double myv2sup;
    private int myusample;
    private int myvsample;
    private NCollection_HArray2<Point3D>? mypoints1;
    private NCollection_HArray2<Point3D>? mypoints2;
    private double mytol1;
    private double mytol2;
    private readonly Extrema_FuncExtSS myF;
    private Adaptor3d_Surface? myS2;

    /// <summary>An empty algorithm.</summary>
    public Extrema_GenExtSS()
    {
        myF = new Extrema_FuncExtSS();
        myu1min = 0.0;
        myu1sup = 0.0;
        myv1min = 0.0;
        myv1sup = 0.0;
        myu2min = 0.0;
        myu2sup = 0.0;
        myv2min = 0.0;
        myv2sup = 0.0;
        myusample = 0;
        myvsample = 0;
        mytol1 = 0.0;
        mytol2 = 0.0;
        myS2 = null;
        myDone = false;
        myInit = false;
    }

    /// <summary>
    /// Computes the extrema between S1 and S2 over their whole ranges. NbU and NbV sample both surfaces to find the
    /// starting points.
    /// </summary>
    public Extrema_GenExtSS(Adaptor3d_Surface S1, Adaptor3d_Surface S2, int NbU, int NbV, double Tol1, double Tol2)
    {
        myF = new Extrema_FuncExtSS(S1, S2);
        Initialize(S2, NbU, NbV, Tol2);
        Perform(S1, Tol1);
    }

    /// <summary>Computes the extrema between S1 and S2 over the given ranges.</summary>
    public Extrema_GenExtSS(Adaptor3d_Surface S1, Adaptor3d_Surface S2, int NbU, int NbV, double U1min, double U1sup,
        double V1min, double V1sup, double U2min, double U2sup, double V2min, double V2sup, double Tol1, double Tol2)
    {
        myF = new Extrema_FuncExtSS(S1, S2);
        Initialize(S2, NbU, NbV, U2min, U2sup, V2min, V2sup, Tol2);
        Perform(S1, U1min, U1sup, V1min, V1sup, Tol1);
    }

    /// <summary>Sets S2 over its whole range and samples it.</summary>
    public void Initialize(Adaptor3d_Surface S2, int NbU, int NbV, double Tol2)
    {
        myu2min = S2.FirstUParameter();
        myu2sup = S2.LastUParameter();
        myv2min = S2.FirstVParameter();
        myv2sup = S2.LastVParameter();
        Initialize(S2, NbU, NbV, myu2min, myu2sup, myv2min, myv2sup, Tol2);
    }

    /// <summary>Sets S2 over [U2min, U2sup] x [V2min, V2sup] and samples it.</summary>
    public void Initialize(Adaptor3d_Surface S2, int NbU, int NbV, double U2min, double U2sup, double V2min,
        double V2sup, double Tol2)
    {
        myS2 = S2;
        mypoints1 = new NCollection_HArray2<Point3D>(0, NbU + 1, 0, NbV + 1);
        mypoints2 = new NCollection_HArray2<Point3D>(0, NbU + 1, 0, NbV + 1);
        myusample = NbU;
        myvsample = NbV;
        myu2min = U2min;
        myu2sup = U2sup;
        myv2min = V2min;
        myv2sup = V2sup;
        mytol2 = Tol2;

        // Parametrage de l echantillon sur S2

        double PasU = myu2sup - myu2min;
        double PasV = myv2sup - myv2min;
        double U0 = PasU / myusample / 100.0;
        double V0 = PasV / myvsample / 100.0;
        PasU = (PasU - U0) / (myusample - 1);
        PasV = (PasV - V0) / (myvsample - 1);
        U0 = myu2min + U0 / 2.0;
        V0 = myv2min + V0 / 2.0;

        // Build UV parameter arrays for batch evaluation
        NCollection_Array1<double> aUParams = new(1, myusample);
        NCollection_Array1<double> aVParams = new(1, myvsample);

        double U = U0;
        for (int NoU = 1; NoU <= myusample; NoU++, U += PasU)
        {
            aUParams.SetValue(NoU, U);
        }
        double V = V0;
        for (int NoV = 1; NoV <= myvsample; NoV++, V += PasV)
        {
            aVParams.SetValue(NoV, V);
        }

        // Use batch grid evaluation for optimized surface point computation
        GeomGridEval_Surface anEvaluator = new(myS2);

        NCollection_Array2<Point3D> aGrid = anEvaluator.EvaluateGrid(aUParams, aVParams);

        for (int NoU = 1; NoU <= myusample; NoU++)
        {
            for (int NoV = 1; NoV <= myvsample; NoV++)
            {
                mypoints2.SetValue(NoU, NoV, aGrid.Value(NoU, NoV));
            }
        }
    }

    /// <summary>Computes the extrema with S1 over its whole range (S2 must have been initialised).</summary>
    public void Perform(Adaptor3d_Surface S1, double Tol1)
    {
        myu1min = S1.FirstUParameter();
        myu1sup = S1.LastUParameter();
        myv1min = S1.FirstVParameter();
        myv1sup = S1.LastVParameter();
        Perform(S1, myu1min, myu1sup, myv1min, myv1sup, Tol1);
    }

    /// <summary>Computes the extrema with S1 over [U1min, U1sup] x [V1min, V1sup] (S2 must have been initialised).</summary>
    public void Perform(Adaptor3d_Surface S1, double U1min, double U1sup, double V1min, double V1sup, double Tol1)
    {
        if (myS2 is null)
        {
            throw new NotDoneException("Extrema_GenExtSS: the second surface is not initialized");
        }
        myF.Initialize(S1, myS2);
        myu1min = U1min;
        myu1sup = U1sup;
        myv1min = V1min;
        myv1sup = V1sup;
        mytol1 = Tol1;

        double U1, V1, U2, V2;
        int NoU1, NoV1, NoU2, NoV2;
        Point3D P1, P2;

        // Parametrage de l echantillon sur S1

        double PasU1 = myu1sup - myu1min;
        double PasV1 = myv1sup - myv1min;
        double U10 = PasU1 / myusample / 100.0;
        double V10 = PasV1 / myvsample / 100.0;
        PasU1 = (PasU1 - U10) / (myusample - 1);
        PasV1 = (PasV1 - V10) / (myvsample - 1);
        U10 = myu1min + U10 / 2.0;
        V10 = myv1min + V10 / 2.0;

        double PasU2 = myu2sup - myu2min;
        double PasV2 = myv2sup - myv2min;
        double U20 = PasU2 / myusample / 100.0;
        double V20 = PasV2 / myvsample / 100.0;
        PasU2 = (PasU2 - U20) / (myusample - 1);
        PasV2 = (PasV2 - V20) / (myvsample - 1);
        U20 = myu2min + U20 / 2.0;
        V20 = myv2min + V20 / 2.0;

        // Build UV parameter arrays for batch evaluation of S1
        NCollection_Array1<double> aU1Params = new(1, myusample);
        NCollection_Array1<double> aV1Params = new(1, myvsample);

        U1 = U10;
        for (NoU1 = 1; NoU1 <= myusample; NoU1++, U1 += PasU1)
        {
            aU1Params.SetValue(NoU1, U1);
        }
        V1 = V10;
        for (NoV1 = 1; NoV1 <= myvsample; NoV1++, V1 += PasV1)
        {
            aV1Params.SetValue(NoV1, V1);
        }

        // Use batch grid evaluation for optimized surface point computation
        GeomGridEval_Surface anEvaluator = new(S1);

        NCollection_Array2<Point3D> aGrid = anEvaluator.EvaluateGrid(aU1Params, aV1Params);

        for (NoU1 = 1; NoU1 <= myusample; NoU1++)
        {
            for (NoV1 = 1; NoV1 <= myvsample; NoV1++)
            {
                mypoints1!.SetValue(NoU1, NoV1, aGrid.Value(NoU1, NoV1));
            }
        }

        /*
        b- Calcul des minima:
           -----------------
           b.a) Initialisations:
        */

        math_Vector Tol = new(1, 4);
        Tol[1] = mytol1;
        Tol[2] = mytol1;
        Tol[3] = mytol2;
        Tol[4] = mytol2;
        math_Vector UV = new(1, 4), UVinf = new(1, 4), UVsup = new(1, 4);
        UVinf[1] = myu1min;
        UVinf[2] = myv1min;
        UVinf[3] = myu2min;
        UVinf[4] = myv2min;
        UVsup[1] = myu1sup;
        UVsup[2] = myv1sup;
        UVsup[3] = myu2sup;
        UVsup[4] = myv2sup;

        double distmin = RealLast(), distmax = 0.0, TheDist;

        int N1Umin = 0, N1Vmin = 0, N2Umin = 0, N2Vmin = 0;
        int N1Umax = 0, N1Vmax = 0, N2Umax = 0, N2Vmax = 0;
        // OCCT also keeps the points of both pairs (PP1min, PP2min, PP1max, PP2max), never read; left out

        for (NoU1 = 1, U1 = U10; NoU1 <= myusample; NoU1++, U1 += PasU1)
        {
            for (NoV1 = 1, V1 = V10; NoV1 <= myvsample; NoV1++, V1 += PasV1)
            {
                P1 = mypoints1!.Value(NoU1, NoV1);
                for (NoU2 = 1, U2 = U20; NoU2 <= myusample; NoU2++, U2 += PasU2)
                {
                    for (NoV2 = 1, V2 = V20; NoV2 <= myvsample; NoV2++, V2 += PasV2)
                    {
                        P2 = mypoints2!.Value(NoU2, NoV2);
                        TheDist = P1.SquareDistance(P2);
                        if (TheDist < distmin)
                        {
                            distmin = TheDist;
                            N1Umin = NoU1;
                            N1Vmin = NoV1;
                            N2Umin = NoU2;
                            N2Vmin = NoV2;
                        }
                        if (TheDist > distmax)
                        {
                            distmax = TheDist;
                            N1Umax = NoU1;
                            N1Vmax = NoV1;
                            N2Umax = NoU2;
                            N2Vmax = NoV2;
                        }
                    }
                }
            }
        }

        UV[1] = U10 + (N1Umin - 1) * PasU1;
        UV[2] = V10 + (N1Vmin - 1) * PasV1;
        UV[3] = U20 + (N2Umin - 1) * PasU2;
        UV[4] = V20 + (N2Vmin - 1) * PasV2;

        Extrema_FuncDistSS aGFSS = new(S1, myS2);
        Bfgs aBFGSSolver = new(4);
        aBFGSSolver.Perform(aGFSS, UV);
        if (aBFGSSolver.IsDone)
        {
            aBFGSSolver.Location(UV);

            //  Store result in myF (OCCT passes UV as both the variables and the values).
            myF.Value(UV, UV);
            myF.GetStateNumber();
        }
        else
        {
            // If optimum is not computed successfully then compute by old approach.

            // Restore initial point.
            UV[1] = U10 + (N1Umin - 1) * PasU1;
            UV[2] = V10 + (N1Vmin - 1) * PasV1;
            UV[3] = U20 + (N2Umin - 1) * PasU2;
            UV[4] = V20 + (N2Vmin - 1) * PasV2;

            FunctionSetRoot SR1 = new(myF, Tol);
            SR1.Perform(myF, UV, UVinf, UVsup);
        }

        UV[1] = U10 + (N1Umax - 1) * PasU1;
        UV[2] = V10 + (N1Vmax - 1) * PasV1;
        UV[3] = U20 + (N2Umax - 1) * PasU2;
        UV[4] = V20 + (N2Vmax - 1) * PasV2;

        // It is impossible to compute max distance in the same manner,
        // since for the distance functional for max have bad definition.
        // So, for max computation old approach is used.
        FunctionSetRoot SR2 = new(myF, Tol);
        SR2.Perform(myF, UV, UVinf, UVsup);

        myDone = true;
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

    /// <summary>The point on S1 of the Nth extremum.</summary>
    public Extrema_POnSurf PointOnS1(int N)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        return myF.PointOnS1(N);
    }

    /// <summary>The point on S2 of the Nth extremum.</summary>
    public Extrema_POnSurf PointOnS2(int N)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        return myF.PointOnS2(N);
    }
}
