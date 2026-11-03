// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvlinFunc.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvlinFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// An abstract curve with curvilinear parametrization (S in [0, 1]): a 3D curve (case 1), a 2D curve on a surface
/// (case 2), or two 2D curves on two surfaces (case 3), whose evaluations Approx_CurvilinearParameter approximates.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Trim"/> and <see cref="SetTol"/> change it, and the evaluations
/// remember the last parameter found (OCCT casts const away to write it). The adaptors are kept as OCCT keeps the
/// handles; <see cref="Trim"/> replaces them by trimmed adaptors and never changes the caller's.
/// </remarks>
public sealed class Approx_CurvlinFunc : Transient
{
    private Adaptor3d_Curve? myC3D;
    private Adaptor2d_Curve2d? myC2D1;
    private Adaptor2d_Curve2d? myC2D2;
    private Adaptor3d_Surface? mySurf1;
    private Adaptor3d_Surface? mySurf2;
    private readonly int myCase;
    private double myFirstS;
    private double myLastS;
    private double myFirstU1;
    private double myLastU1;
    private double myFirstU2;
    private double myLastU2;
    private double myLength;
    private double myLength1;
    private double myLength2;
    private double myTolLen;
    private double myPrevS;
    private double myPrevU;
    private NCollection_HArray1<double>? myUi_1;
    private NCollection_HArray1<double>? mySi_1;
    private NCollection_HArray1<double>? myUi_2;
    private NCollection_HArray1<double>? mySi_2;

    private static double cubic(double X, ReadOnlySpan<double> Xi, ReadOnlySpan<double> Yi)
    {
        double I1, I2, I3, I21, I22, I31, Result;

        I1 = (Yi[0] - Yi[1]) / (Xi[0] - Xi[1]);
        I2 = (Yi[1] - Yi[2]) / (Xi[1] - Xi[2]);
        I3 = (Yi[2] - Yi[3]) / (Xi[2] - Xi[3]);

        I21 = (I1 - I2) / (Xi[0] - Xi[2]);
        I22 = (I2 - I3) / (Xi[1] - Xi[3]);

        I31 = (I21 - I22) / (Xi[0] - Xi[3]);

        Result = Yi[0] + (X - Xi[0]) * (I1 + (X - Xi[1]) * (I21 + (X - Xi[2]) * I31));

        return Result;
    }

    private static void findfourpoints(double /*S*/ _,
                                       int NInterval,
                                       NCollection_HArray1<double> Si,
                                       NCollection_HArray1<double> Ui,
                                       double prevS,
                                       double prevU,
                                       Span<double> Xi,
                                       Span<double> Yi)
    {
        int i, j;
        int NbInt = Si.Length() - 1;
        if (NbInt < 3)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::GetUParameter");
        }

        if (NInterval < 1)
        {
            NInterval = 1;
        }
        else if (NInterval > NbInt - 2)
        {
            NInterval = NbInt - 2;
        }

        for (i = 0; i < 4; i++)
        {
            Xi[i] = Si.Value(NInterval - 1 + i);
            Yi[i] = Ui.Value(NInterval - 1 + i);
        }
        // try to insert (S, U)
        for (i = 0; i < 3; i++)
        {
            if (Xi[i] < prevS && prevS < Xi[i + 1])
            {
                for (j = 0; j < i; j++)
                {
                    Xi[j] = Xi[j + 1];
                    Yi[j] = Yi[j + 1];
                }
                Xi[i] = prevS;
                Yi[i] = prevU;
                break;
            }
        }
    }

    /// <summary>A 3D curve, with the tolerance of the length computations.</summary>
    public Approx_CurvlinFunc(Adaptor3d_Curve C, double Tol)
    {
        myC3D = C;
        myCase = 1;
        myFirstS = 0;
        myLastS = 1;
        myTolLen = Tol;
        myPrevS = 0.0;
        myPrevU = 0.0;
        Init();
    }

    /// <summary>A 2D curve on a surface, with the tolerance of the length computations.</summary>
    public Approx_CurvlinFunc(Adaptor2d_Curve2d C2D, Adaptor3d_Surface S, double Tol)
    {
        myC2D1 = C2D;
        mySurf1 = S;
        myCase = 2;
        myFirstS = 0;
        myLastS = 1;
        myTolLen = Tol;
        myPrevS = 0.0;
        myPrevU = 0.0;
        Init();
    }

    /// <summary>Two 2D curves on two surfaces, with the tolerance of the length computations.</summary>
    public Approx_CurvlinFunc(Adaptor2d_Curve2d C2D1,
                              Adaptor2d_Curve2d C2D2,
                              Adaptor3d_Surface S1,
                              Adaptor3d_Surface S2,
                              double Tol)
    {
        myC2D1 = C2D1;
        myC2D2 = C2D2;
        mySurf1 = S1;
        mySurf2 = S2;
        myCase = 3;
        myFirstS = 0;
        myLastS = 1;
        myTolLen = Tol;
        myPrevS = 0.0;
        myPrevU = 0.0;
        Init();
    }

    private void Init()
    {
        Adaptor3d_CurveOnSurface CurOnSur = new();

        switch (myCase)
        {
            case 1:
                Init(myC3D!, out mySi_1, out myUi_1);
                myFirstU1 = myC3D!.FirstParameter();
                myLastU1 = myC3D.LastParameter();
                myFirstU2 = myLastU2 = 0;
                break;
            case 2:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                Init(CurOnSur, out mySi_1, out myUi_1);
                myFirstU1 = CurOnSur.FirstParameter();
                myLastU1 = CurOnSur.LastParameter();
                myFirstU2 = myLastU2 = 0;
                break;
            case 3:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                Init(CurOnSur, out mySi_1, out myUi_1);
                myFirstU1 = CurOnSur.FirstParameter();
                myLastU1 = CurOnSur.LastParameter();
                // OCCT: the reloaded adaptor keeps the first curve's C3 intervals (see NbIntervals).
                CurOnSur.Load(myC2D2!);
                CurOnSur.Load(mySurf2!);
                Init(CurOnSur, out mySi_2, out myUi_2);
                myFirstU2 = CurOnSur.FirstParameter();
                myLastU2 = CurOnSur.LastParameter();
                break;
        }

        Length();
    }

    // Init the values. 23/10/1998 PMN: cut at the curve's discontinuities.
    private void Init(Adaptor3d_Curve C, out NCollection_HArray1<double> Si, out NCollection_HArray1<double> Ui)
    {
        double Step, FirstU, LastU;
        int i, j, k, NbInt, NbIntC3;
        FirstU = C.FirstParameter();
        LastU = C.LastParameter();

        NbInt = 10;
        NbIntC3 = C.NbIntervals(C3);
        NCollection_Array1<double> Disc = new(1, NbIntC3 + 1);

        if (NbIntC3 > 1)
        {
            C.Intervals(Disc, C3);
        }
        else
        {
            Disc[1] = FirstU;
            Disc[2] = LastU;
        }

        Ui = new NCollection_HArray1<double>(0, NbIntC3 * NbInt);
        Si = new NCollection_HArray1<double>(0, NbIntC3 * NbInt);

        Ui.SetValue(0, FirstU);
        Si.SetValue(0, 0);

        for (j = 1, i = 1; j <= NbIntC3; j++)
        {
            Step = (Disc[j + 1] - Disc[j]) / NbInt;
            for (k = 1; k <= NbInt; k++, i++)
            {
                Ui[i] = Ui.Value(i - 1) + Step;
                Si[i] = Si.Value(i - 1) + Length(C, Ui.Value(i - 1), Ui.Value(i));
            }
        }

        double Len = Si.Value(Si.Upper());
        for (i = Si.Lower(); i <= Si.Upper(); i++)
        {
            Si[i] /= Len;
        }

        myPrevS = myFirstS;
        myPrevU = FirstU;
    }

    /// <summary>Updates the tolerance of the length computations.</summary>
    public void SetTol(double Tol)
    {
        myTolLen = Tol;
    }

    public double FirstParameter() => myFirstS;

    public double LastParameter() => myLastS;

    /// <summary>The number of intervals of continuity S.</summary>
    public int NbIntervals(Continuity S)
    {
        Adaptor3d_CurveOnSurface CurOnSur = new();

        switch (myCase)
        {
            case 1:
                return myC3D!.NbIntervals(S);
            case 2:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                return CurOnSur.NbIntervals(S);
            case 3:
            {
                int NbInt;
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                NbInt = CurOnSur.NbIntervals(S);
                NCollection_Array1<double> T1 = new(1, NbInt + 1);
                CurOnSur.Intervals(T1, S);
                // OCCT: Adaptor3d_CurveOnSurface::Load keeps the intervals cached for the first curve, so these are
                // the first curve's intervals again (occt-issues.md).
                CurOnSur.Load(myC2D2!);
                CurOnSur.Load(mySurf2!);
                NbInt = CurOnSur.NbIntervals(S);
                NCollection_Array1<double> T2 = new(1, NbInt + 1);
                CurOnSur.Intervals(T2, S);

                NCollection_Sequence<double> Fusion = new();
                GeomLib.FuseIntervals(T1, T2, Fusion);
                return Fusion.Length() - 1;
            }
        }

        // POP pour WNT
        return 1;
    }

    /// <summary>Stores in T the curvilinear parameters bounding the intervals of continuity S.</summary>
    public void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        Adaptor3d_CurveOnSurface CurOnSur = new();
        int i;

        switch (myCase)
        {
            case 1:
                myC3D!.Intervals(T, S);
                break;
            case 2:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                CurOnSur.Intervals(T, S);
                break;
            case 3:
            {
                int NbInt;
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                NbInt = CurOnSur.NbIntervals(S);
                NCollection_Array1<double> T1 = new(1, NbInt + 1);
                CurOnSur.Intervals(T1, S);
                // OCCT: Adaptor3d_CurveOnSurface::Load keeps the intervals cached for the first curve, so these are
                // the first curve's intervals again (occt-issues.md).
                CurOnSur.Load(myC2D2!);
                CurOnSur.Load(mySurf2!);
                NbInt = CurOnSur.NbIntervals(S);
                NCollection_Array1<double> T2 = new(1, NbInt + 1);
                CurOnSur.Intervals(T2, S);

                NCollection_Sequence<double> Fusion = new();
                GeomLib.FuseIntervals(T1, T2, Fusion);

                for (i = 1; i <= Fusion.Length(); i++)
                {
                    T[i] = Fusion.Value(i);
                }
                break;
            }
        }

        for (i = 1; i <= T.Length(); i++)
        {
            T[i] = GetSParameter(T.Value(i));
        }
    }

    /// <summary>Restricts the curve to the curvilinear range [First, Last] (within [0, 1]).</summary>
    public void Trim(double First, double Last, double Tol)
    {
        if (First < 0 || Last > 1)
        {
            throw new OutOfRangeException("Approx_CurvlinFunc::Trim");
        }
        if ((Last - First) < Tol)
        {
            return;
        }

        double FirstU, LastU;
        Adaptor3d_CurveOnSurface CurOnSur = new();
        Adaptor3d_CurveOnSurface? HCurOnSur;

        switch (myCase)
        {
            case 1:
                myC3D = myC3D!.Trim(myFirstU1, myLastU1, Tol);
                FirstU = GetUParameter(myC3D, First, 1);
                LastU = GetUParameter(myC3D, Last, 1);
                myC3D = myC3D.Trim(FirstU, LastU, Tol);
                break;
            case 3:
                CurOnSur.Load(myC2D2!);
                CurOnSur.Load(mySurf2!);
                HCurOnSur = CurOnSur.Trim(myFirstU2, myLastU2, Tol) as Adaptor3d_CurveOnSurface;
                myC2D2 = HCurOnSur!.GetCurve();
                mySurf2 = HCurOnSur.GetSurface();
                CurOnSur.Load(myC2D2!);
                CurOnSur.Load(mySurf2!);

                // OCCT reads the parameter tables of the first curve (NumberOfCurve 1) for the second curve.
                FirstU = GetUParameter(CurOnSur, First, 1);
                LastU = GetUParameter(CurOnSur, Last, 1);
                HCurOnSur = CurOnSur.Trim(FirstU, LastU, Tol) as Adaptor3d_CurveOnSurface;
                myC2D2 = HCurOnSur!.GetCurve();
                mySurf2 = HCurOnSur.GetSurface();

                goto case 2;
            case 2:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                HCurOnSur = CurOnSur.Trim(myFirstU1, myLastU1, Tol) as Adaptor3d_CurveOnSurface;
                myC2D1 = HCurOnSur!.GetCurve();
                mySurf1 = HCurOnSur.GetSurface();
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);

                FirstU = GetUParameter(CurOnSur, First, 1);
                LastU = GetUParameter(CurOnSur, Last, 1);
                HCurOnSur = CurOnSur.Trim(FirstU, LastU, Tol) as Adaptor3d_CurveOnSurface;
                myC2D1 = HCurOnSur!.GetCurve();
                mySurf1 = HCurOnSur.GetSurface();
                break;
        }
        myFirstS = First;
        myLastS = Last;
    }

    /// <summary>Computes the length of the curve.</summary>
    public void Length()
    {
        Adaptor3d_CurveOnSurface CurOnSur = new();
        double FirstU, LastU;

        switch (myCase)
        {
            case 1:
                FirstU = myC3D!.FirstParameter();
                LastU = myC3D.LastParameter();
                myLength = Length(myC3D, FirstU, LastU);
                myLength1 = myLength2 = 0;
                break;
            case 2:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                FirstU = CurOnSur.FirstParameter();
                LastU = CurOnSur.LastParameter();
                myLength = Length(CurOnSur, FirstU, LastU);
                myLength1 = myLength2 = 0;
                break;
            case 3:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                FirstU = CurOnSur.FirstParameter();
                LastU = CurOnSur.LastParameter();
                myLength1 = Length(CurOnSur, FirstU, LastU);
                CurOnSur.Load(myC2D2!);
                CurOnSur.Load(mySurf2!);
                FirstU = CurOnSur.FirstParameter();
                LastU = CurOnSur.LastParameter();
                myLength2 = Length(CurOnSur, FirstU, LastU);
                myLength = (myLength1 + myLength2) / 2;
                break;
        }
    }

    /// <summary>Computes the length of the curve segment [FirstU, LastU].</summary>
    public double Length(Adaptor3d_Curve C, double FirstU, double LastU)
    {
        double Length;

        Length = GCPnts_AbscissaPoint.Length(C, FirstU, LastU, myTolLen);
        return Length;
    }

    public double GetLength() => myLength;

    /// <summary>The curvilinear parameter of the original parameter U.</summary>
    public double GetSParameter(double U)
    {
        double S = 0, S1, S2;
        Adaptor3d_CurveOnSurface CurOnSur = new();

        switch (myCase)
        {
            case 1:
                S = GetSParameter(myC3D!, U, myLength);
                break;
            case 2:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                S = GetSParameter(CurOnSur, U, myLength);
                break;
            case 3:
                CurOnSur.Load(myC2D1!);
                CurOnSur.Load(mySurf1!);
                S1 = GetSParameter(CurOnSur, U, myLength1);
                CurOnSur.Load(myC2D2!);
                CurOnSur.Load(mySurf2!);
                S2 = GetSParameter(CurOnSur, U, myLength2);
                S = (S1 + S2) / 2;
                break;
        }
        return S;
    }

    /// <summary>
    /// The original parameter of the curvilinear parameter S on C, read with the tables of the first curve
    /// (NumberOfCurve 1) or of the second.
    /// </summary>
    public double GetUParameter(Adaptor3d_Curve C, double S, int NumberOfCurve)
    {
        double deltaS, @base, U, Length;
        int NbInt, NInterval, i;
        NCollection_HArray1<double> InitUArray, InitSArray;
        if (S < 0 || S > 1)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::GetUParameter");
        }

        if (NumberOfCurve == 1)
        {
            InitUArray = myUi_1!;
            InitSArray = mySi_1!;
            if (myCase == 3)
            {
                Length = myLength1;
            }
            else
            {
                Length = myLength;
            }
        }
        else
        {
            InitUArray = myUi_2!;
            InitSArray = mySi_2!;
            Length = myLength2;
        }

        NbInt = InitUArray.Length() - 1;

        if (S == 1)
        {
            NInterval = NbInt - 1;
        }
        else
        {
            for (i = 0; i < NbInt; i++)
            {
                if ((InitSArray.Value(i) <= S && S < InitSArray.Value(i + 1)))
                {
                    break;
                }
            }
            NInterval = i;
        }
        if (S == InitSArray.Value(NInterval))
        {
            return InitUArray.Value(NInterval);
        }
        if (S == InitSArray.Value(NInterval + 1))
        {
            return InitUArray.Value(NInterval + 1);
        }

        @base = InitUArray.Value(NInterval);
        deltaS = (S - InitSArray.Value(NInterval)) * Length;

        // to find an initial point
        Span<double> Xi = stackalloc double[4], Yi = stackalloc double[4];
        double UGuess;
        findfourpoints(S, NInterval, InitSArray, InitUArray, myPrevS, myPrevU, Xi, Yi);
        UGuess = cubic(S, Xi, Yi);

        U = new GCPnts_AbscissaPoint(C, deltaS, @base, UGuess, myTolLen).Parameter();

        myPrevS = S;
        myPrevU = U;

        return U;
    }

    // The curvilinear parameter of U on C.
    private double GetSParameter(Adaptor3d_Curve C, double U, double Len)
    {
        double S, Origin;

        Origin = C.FirstParameter();
        S = myFirstS + Length(C, Origin, U) / Len;
        return S;
    }

    /// <summary>Evaluates the 3D curve (case 1) at S: the point or derivative Order in Result(0..2).</summary>
    public bool EvalCase1(double S, int Order, NCollection_Array1<double> Result)
    {
        if (myCase != 1)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCase1");
        }

        Point3D C;
        Vector3D dC_dU, dC_dS, d2C_dU2, d2C_dS2;
        double U, Mag, dU_dS, d2U_dS2;

        U = GetUParameter(myC3D!, S, 1);

        switch (Order)
        {
            case 0:
                myC3D!.D0(U, out C);

                Result[0] = C.X();
                Result[1] = C.Y();
                Result[2] = C.Z();
                break;

            case 1:
                myC3D!.D1(U, out C, out dC_dU);
                Mag = dC_dU.Magnitude();
                dU_dS = myLength / Mag;
                dC_dS = dC_dU * dU_dS;

                Result[0] = dC_dS.X();
                Result[1] = dC_dS.Y();
                Result[2] = dC_dS.Z();
                break;

            case 2:
                myC3D!.D2(U, out C, out dC_dU, out d2C_dU2);
                Mag = dC_dU.Magnitude();
                dU_dS = myLength / Mag;
                d2U_dS2 = -myLength * dC_dU.Dot(d2C_dU2) * dU_dS / (Mag * Mag * Mag);
                d2C_dS2 = d2C_dU2 * dU_dS * dU_dS + dC_dU * d2U_dS2;

                Result[0] = d2C_dS2.X();
                Result[1] = d2C_dS2.Y();
                Result[2] = d2C_dS2.Z();
                break;

            default:
                Result[0] = Result[1] = Result[2] = 0;
                return false;
        }
        return true;
    }

    /// <summary>Evaluates the curve on surface (case 2) at S: (u, v, x, y, z) or their derivatives in Result(0..4).</summary>
    public bool EvalCase2(double S, int Order, NCollection_Array1<double> Result)
    {
        if (myCase != 2)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCase2");
        }

        bool Done;

        Done = EvalCurOnSur(S, Order, Result, 1);

        return Done;
    }

    /// <summary>
    /// Evaluates the two curves on surfaces (case 3) at S: (u1, v1, u2, v2) and the mean 3D point, or their
    /// derivatives, in Result(0..6).
    /// </summary>
    public bool EvalCase3(double S, int Order, NCollection_Array1<double> Result)
    {
        if (myCase != 3)
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCase3");
        }

        NCollection_Array1<double> tmpRes1 = new(0, 4), tmpRes2 = new(0, 4);
        bool Done;

        Done = EvalCurOnSur(S, Order, tmpRes1, 1);

        Done = EvalCurOnSur(S, Order, tmpRes2, 2) && Done;

        Result[0] = tmpRes1[0];
        Result[1] = tmpRes1[1];
        Result[2] = tmpRes2[0];
        Result[3] = tmpRes2[1];
        Result[4] = 0.5 * (tmpRes1[2] + tmpRes2[2]);
        Result[5] = 0.5 * (tmpRes1[3] + tmpRes2[3]);
        Result[6] = 0.5 * (tmpRes1[4] + tmpRes2[4]);

        return Done;
    }

    private bool EvalCurOnSur(double S, int Order, NCollection_Array1<double> Result, int NumberOfCurve)
    {
        Adaptor2d_Curve2d Cur2D;
        Adaptor3d_Surface Surf;
        double U = 0, Length = 0;

        if (NumberOfCurve == 1)
        {
            Cur2D = myC2D1!;
            Surf = mySurf1!;
            Adaptor3d_CurveOnSurface CurOnSur = new(myC2D1!, mySurf1!);
            U = GetUParameter(CurOnSur, S, 1);
            if (myCase == 3)
            {
                Length = myLength1;
            }
            else
            {
                Length = myLength;
            }
        }
        else if (NumberOfCurve == 2)
        {
            Cur2D = myC2D2!;
            Surf = mySurf2!;
            Adaptor3d_CurveOnSurface CurOnSur = new(myC2D2!, mySurf2!);
            U = GetUParameter(CurOnSur, S, 2);
            Length = myLength2;
        }
        else
        {
            throw new ConstructionErrorException("Approx_CurvlinFunc::EvalCurOnSur");
        }

        double Mag, dU_dS, d2U_dS2, dV_dU, dW_dU, dV_dS, dW_dS, d2V_dS2, d2W_dS2, d2V_dU2, d2W_dU2;
        Point2D C2D;
        Point3D C;
        Vector2D dC2D_dU, d2C2D_dU2;
        Vector3D dC_dU, d2C_dU2, dC_dS, d2C_dS2, dS_dV, dS_dW, d2S_dV2, d2S_dW2, d2S_dVdW;

        switch (Order)
        {
            case 0:
                Cur2D.D0(U, out C2D);
                Surf.D0(C2D.X(), C2D.Y(), out C);

                Result[0] = C2D.X();
                Result[1] = C2D.Y();
                Result[2] = C.X();
                Result[3] = C.Y();
                Result[4] = C.Z();
                break;

            case 1:
                Cur2D.D1(U, out C2D, out dC2D_dU);
                dV_dU = dC2D_dU.X();
                dW_dU = dC2D_dU.Y();
                Surf.D1(C2D.X(), C2D.Y(), out C, out dS_dV, out dS_dW);
                dC_dU = dS_dV * dV_dU + dS_dW * dW_dU;
                Mag = dC_dU.Magnitude();
                dU_dS = Length / Mag;

                dV_dS = dV_dU * dU_dS;
                dW_dS = dW_dU * dU_dS;
                dC_dS = dC_dU * dU_dS;

                Result[0] = dV_dS;
                Result[1] = dW_dS;
                Result[2] = dC_dS.X();
                Result[3] = dC_dS.Y();
                Result[4] = dC_dS.Z();
                break;

            case 2:
                Cur2D.D2(U, out C2D, out dC2D_dU, out d2C2D_dU2);
                dV_dU = dC2D_dU.X();
                dW_dU = dC2D_dU.Y();
                d2V_dU2 = d2C2D_dU2.X();
                d2W_dU2 = d2C2D_dU2.Y();
                Surf.D2(C2D.X(), C2D.Y(), out C, out dS_dV, out dS_dW, out d2S_dV2, out d2S_dW2, out d2S_dVdW);
                dC_dU = dS_dV * dV_dU + dS_dW * dW_dU;
                d2C_dU2 = (d2S_dV2 * dV_dU + d2S_dVdW * dW_dU) * dV_dU + dS_dV * d2V_dU2
                          + (d2S_dVdW * dV_dU + d2S_dW2 * dW_dU) * dW_dU + dS_dW * d2W_dU2;
                Mag = dC_dU.Magnitude();
                dU_dS = Length / Mag;
                d2U_dS2 = -Length * dC_dU.Dot(d2C_dU2) * dU_dS / (Mag * Mag * Mag);

                dV_dS = dV_dU * dU_dS;
                dW_dS = dW_dU * dU_dS;
                d2V_dS2 = d2V_dU2 * dU_dS * dU_dS + dV_dU * d2U_dS2;
                d2W_dS2 = d2W_dU2 * dU_dS * dU_dS + dW_dU * d2U_dS2;

                // OCCT recomputes d2U_dS2 here (another formula) and never reads it again.
                d2C_dS2 = (d2S_dV2 * dV_dS + d2S_dVdW * dW_dS) * dV_dS + dS_dV * d2V_dS2
                          + (d2S_dW2 * dW_dS + d2S_dVdW * dV_dS) * dW_dS + dS_dW * d2W_dS2;

                Result[0] = d2V_dS2;
                Result[1] = d2W_dS2;
                Result[2] = d2C_dS2.X();
                Result[3] = d2C_dS2.Y();
                Result[4] = d2C_dS2.Z();
                break;

            default:
                Result[0] = Result[1] = Result[2] = Result[3] = Result[4] = 0;
                return false;
        }
        return true;
    }
}
