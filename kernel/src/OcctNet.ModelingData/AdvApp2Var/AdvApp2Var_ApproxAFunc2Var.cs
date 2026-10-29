// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_ApproxAFunc2Var.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_ApproxAFunc2Var.lxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_ApproxAFunc2Var.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.AdvApp2Var_CriterionRepartition;
using static Occt.AdvApp2Var_CriterionType;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Approximates a function F(U, V), made of 1D, 2D and 3D subspaces, by B-spline surfaces: polynomial patches on a
/// grid cut as needed, joined with the requested continuities, within the tolerances of each subspace (inside and
/// on the boundaries).
/// </summary>
/// <remarks>
/// <para>
/// The arguments: <c>Num1DSS</c>, <c>Num2DSS</c>, <c>Num3DSS</c>, the numbers of 1, 2 and 3-dimensional subspaces;
/// <c>OneDTol</c>, <c>TwoDTol</c>, <c>ThreeDTol</c>, their tolerances (one per subspace, 1-based; null for a
/// dimension without subspaces); <c>OneDTolFr</c>, ..., their tolerances on the four boundaries (subspace x 4);
/// [<c>FirstInU</c>, <c>LastInU</c>] x [<c>FirstInV</c>, <c>LastInV</c>], the domain; <c>FavorIso</c>, the iso-curves
/// to extract from F (which can help the evaluator); <c>ContInU</c>, <c>ContInV</c>, the continuities (C0, C1 or C2);
/// <c>PrecisCode</c>, the precision of the error measurement (1 fast, 2 average, 3 slow and precise);
/// <c>MaxDegInU</c>, <c>MaxDegInV</c>, the maximum degrees (at least 2 * order + 1, the order being 0, 1 or 2 for
/// C0, C1 or C2); <c>MaxPatch</c>, the maximum number of patches; <c>Func</c>, the function; <c>Crit</c>, an
/// optional criterion of convergence; <c>UChoice</c>, <c>VChoice</c>, where to cut.
/// </para>
/// <para>
/// For the moment the result is a 3D surface: <c>Num1DSS</c> and <c>Num2DSS</c> must be 0 and <c>Num3DSS</c> 1. The
/// evaluator writes, for each parameter, the 1D values, then the 2D ones, then the 3D ones.
/// </para>
/// <para>
/// The approximation is done in the constructor. <see cref="Surface"/> returns the stored surface, frozen
/// (decision 1906); the error accessors return copies, and null where OCCT returns a null handle (decision 146).
/// The approximation is thread-safe: each instance has its own state, and the f2c layer none (decisions 113 and
/// 1901).
/// </para>
/// </remarks>
public class AdvApp2Var_ApproxAFunc2Var
{
    private readonly int[] myNumSubSpaces = new int[3];
    private readonly NCollection_HArray1<double>? my1DTolerances;
    private readonly NCollection_HArray1<double>? my2DTolerances;
    private readonly NCollection_HArray1<double>? my3DTolerances;
    private readonly NCollection_HArray2<double>? my1DTolOnFront;
    private readonly NCollection_HArray2<double>? my2DTolOnFront;
    private readonly NCollection_HArray2<double>? my3DTolOnFront;
    private readonly double myFirstParInU;
    private readonly double myLastParInU;
    private readonly double myFirstParInV;
    private readonly double myLastParInV;
    private readonly GeomAbs_IsoType myFavoriteIso;
    private readonly GeomAbs_Shape myContInU;
    private readonly GeomAbs_Shape myContInV;
    private int myPrecisionCode;
    private readonly int myMaxDegInU;
    private readonly int myMaxDegInV;
    private readonly int myMaxPatches;
    private AdvApp2Var_Context myConditions = new();
    private AdvApp2Var_Network myResult = new();
    private AdvApp2Var_Framework myConstraints = new();
    private bool myDone;
    private bool myHasResult;
    private NCollection_HArray1<Geom_Surface?>? mySurfaces;
    private int myDegreeInU;
    private int myDegreeInV;
    private NCollection_HArray1<double>? my1DMaxError = null;
    private NCollection_HArray1<double>? my1DAverageError = null;
    private NCollection_HArray1<double>? my1DUFrontError = null;
    private NCollection_HArray1<double>? my1DVFrontError = null;
    private NCollection_HArray1<double>? my2DMaxError = null;
    private NCollection_HArray1<double>? my2DAverageError = null;
    private NCollection_HArray1<double>? my2DUFrontError = null;
    private NCollection_HArray1<double>? my2DVFrontError = null;
    private NCollection_HArray1<double>? my3DMaxError;
    private NCollection_HArray1<double>? my3DAverageError;
    private NCollection_HArray1<double>? my3DUFrontError;
    private NCollection_HArray1<double>? my3DVFrontError;
    private double myCriterionError;

    /// <summary>Approximates <paramref name="Func"/> (see the class remarks for the arguments).</summary>
    /// <exception cref="Standard_ConstructionError">
    /// A continuity other than C0, C1 or C2, or an approximation that fails (discretisation, curve or surface
    /// approximation error).
    /// </exception>
    public AdvApp2Var_ApproxAFunc2Var(int Num1DSS,
                                      int Num2DSS,
                                      int Num3DSS,
                                      NCollection_HArray1<double>? OneDTol,
                                      NCollection_HArray1<double>? TwoDTol,
                                      NCollection_HArray1<double>? ThreeDTol,
                                      NCollection_HArray2<double>? OneDTolFr,
                                      NCollection_HArray2<double>? TwoDTolFr,
                                      NCollection_HArray2<double>? ThreeDTolFr,
                                      double FirstInU,
                                      double LastInU,
                                      double FirstInV,
                                      double LastInV,
                                      GeomAbs_IsoType FavorIso,
                                      GeomAbs_Shape ContInU,
                                      GeomAbs_Shape ContInV,
                                      int PrecisCode,
                                      int MaxDegInU,
                                      int MaxDegInV,
                                      int MaxPatch,
                                      AdvApp2Var_EvaluatorFunc2Var Func,
                                      AdvApprox_Cutting UChoice,
                                      AdvApprox_Cutting VChoice)
    {
        my1DTolerances = OneDTol;
        my2DTolerances = TwoDTol;
        my3DTolerances = ThreeDTol;
        my1DTolOnFront = OneDTolFr;
        my2DTolOnFront = TwoDTolFr;
        my3DTolOnFront = ThreeDTolFr;
        myFirstParInU = FirstInU;
        myLastParInU = LastInU;
        myFirstParInV = FirstInV;
        myLastParInV = LastInV;
        myFavoriteIso = FavorIso;
        myContInU = ContInU;
        myContInV = ContInV;
        myPrecisionCode = PrecisCode;
        myMaxDegInU = MaxDegInU;
        myMaxDegInV = MaxDegInV;
        myMaxPatches = MaxPatch;
        myDone = false;
        myHasResult = false;
        myDegreeInU = 0;
        myDegreeInV = 0;
        myCriterionError = 0.0;
        myNumSubSpaces[0] = Num1DSS;
        myNumSubSpaces[1] = Num2DSS;
        myNumSubSpaces[2] = Num3DSS;
        Init();
        Perform(UChoice, VChoice, Func);
        ConvertBS();
    }

    /// <summary>Approximates <paramref name="Func"/> under the criterion <paramref name="Crit"/>.</summary>
    /// <exception cref="Standard_ConstructionError">
    /// A continuity other than C0, C1 or C2, or an approximation that fails.
    /// </exception>
    public AdvApp2Var_ApproxAFunc2Var(int Num1DSS,
                                      int Num2DSS,
                                      int Num3DSS,
                                      NCollection_HArray1<double>? OneDTol,
                                      NCollection_HArray1<double>? TwoDTol,
                                      NCollection_HArray1<double>? ThreeDTol,
                                      NCollection_HArray2<double>? OneDTolFr,
                                      NCollection_HArray2<double>? TwoDTolFr,
                                      NCollection_HArray2<double>? ThreeDTolFr,
                                      double FirstInU,
                                      double LastInU,
                                      double FirstInV,
                                      double LastInV,
                                      GeomAbs_IsoType FavorIso,
                                      GeomAbs_Shape ContInU,
                                      GeomAbs_Shape ContInV,
                                      int PrecisCode,
                                      int MaxDegInU,
                                      int MaxDegInV,
                                      int MaxPatch,
                                      AdvApp2Var_EvaluatorFunc2Var Func,
                                      AdvApp2Var_Criterion Crit,
                                      AdvApprox_Cutting UChoice,
                                      AdvApprox_Cutting VChoice)
    {
        my1DTolerances = OneDTol;
        my2DTolerances = TwoDTol;
        my3DTolerances = ThreeDTol;
        my1DTolOnFront = OneDTolFr;
        my2DTolOnFront = TwoDTolFr;
        my3DTolOnFront = ThreeDTolFr;
        myFirstParInU = FirstInU;
        myLastParInU = LastInU;
        myFirstParInV = FirstInV;
        myLastParInV = LastInV;
        myFavoriteIso = FavorIso;
        myContInU = ContInU;
        myContInV = ContInV;
        myPrecisionCode = PrecisCode;
        myMaxDegInU = MaxDegInU;
        myMaxDegInV = MaxDegInV;
        myMaxPatches = MaxPatch;
        myDone = false;
        myHasResult = false;
        myDegreeInU = 0;
        myDegreeInV = 0;
        myCriterionError = 0.0;
        myNumSubSpaces[0] = Num1DSS;
        myNumSubSpaces[1] = Num2DSS;
        myNumSubSpaces[2] = Num3DSS;
        Init();
        Perform(UChoice, VChoice, Func, Crit);
        ConvertBS();
    }

    // Initialisation of the approximation ; used by Create
    private void Init()
    {
        int ifav, iu = 0, iv = 0, ndu, ndv;
        switch (myFavoriteIso)
        {
            case GeomAbs_IsoU:
                ifav = 1;
                break;
            case GeomAbs_IsoV:
                ifav = 2;
                break;
            default:
                ifav = 2;
                break;
        }
        switch (myContInU)
        {
            case GeomAbs_C0:
                iu = 0;
                break;
            case GeomAbs_C1:
                iu = 1;
                break;
            case GeomAbs_C2:
                iu = 2;
                break;
            default:
                throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : UContinuity Error");
        }
        switch (myContInV)
        {
            case GeomAbs_C0:
                iv = 0;
                break;
            case GeomAbs_C1:
                iv = 1;
                break;
            case GeomAbs_C2:
                iv = 2;
                break;
            default:
                throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : VContinuity Error");
        }
        ndu = Max(myMaxDegInU + 1, 2 * iu + 2);
        ndv = Max(myMaxDegInV + 1, 2 * iv + 2);
        if (ndu < 2 * iu + 2)
        {
            throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : UMaxDegree Error");
        }
        if (ndv < 2 * iv + 2)
        {
            throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : VMaxDegree Error");
        }
        myPrecisionCode = Max(0, Min(myPrecisionCode, 3));
        AdvApp2Var_Context Conditions = new(ifav,
                                            iu,
                                            iv,
                                            ndu,
                                            ndv,
                                            myPrecisionCode,
                                            myNumSubSpaces[0],
                                            myNumSubSpaces[1],
                                            myNumSubSpaces[2],
                                            my1DTolerances,
                                            my2DTolerances,
                                            my3DTolerances,
                                            my1DTolOnFront,
                                            my2DTolOnFront,
                                            my3DTolOnFront);
        myConditions = Conditions;
        InitGrid(1);
    }

    // Initialisation of the approximation with a grid of regular cuttings ; used by Init and Perform
    private void InitGrid(int NbInt)
    {
        int iu = myConditions.UOrder(), iv = myConditions.VOrder(), iint;

        AdvApp2Var_Patch M0 = new(myFirstParInU, myLastParInU, myFirstParInV, myLastParInV, iu, iv);

        NCollection_Sequence<AdvApp2Var_Patch> Net = new();
        Net.Append(M0);

        NCollection_Sequence<double> TheU = new(), TheV = new();
        TheU.Append(myFirstParInU);
        TheV.Append(myFirstParInV);
        TheU.Append(myLastParInU);
        TheV.Append(myLastParInV);

        AdvApp2Var_Network Result = new(Net, TheU, TheV);

        gp_XY UV1 = new(myFirstParInU, myFirstParInV);
        AdvApp2Var_Node C1 = new(UV1, iu, iv);
        gp_XY UV2 = new(myLastParInU, myFirstParInV);
        AdvApp2Var_Node C2 = new(UV2, iu, iv);
        gp_XY UV4 = new(myLastParInU, myLastParInV);
        AdvApp2Var_Node C4 = new(UV4, iu, iv);
        gp_XY UV3 = new(myFirstParInU, myLastParInV);
        AdvApp2Var_Node C3 = new(UV3, iu, iv);
        NCollection_Sequence<AdvApp2Var_Node> Bag = new();
        Bag.Append(C1);
        Bag.Append(C2);
        Bag.Append(C3);
        Bag.Append(C4);

        AdvApp2Var_Iso V0 = new(GeomAbs_IsoV,
                                myFirstParInV,
                                myFirstParInU,
                                myLastParInU,
                                myFirstParInV,
                                myLastParInV,
                                1,
                                iu,
                                iv);
        AdvApp2Var_Iso V1 = new(GeomAbs_IsoV,
                                myLastParInV,
                                myFirstParInU,
                                myLastParInU,
                                myFirstParInV,
                                myLastParInV,
                                2,
                                iu,
                                iv);
        AdvApp2Var_Iso U0 = new(GeomAbs_IsoU,
                                myFirstParInU,
                                myFirstParInU,
                                myLastParInU,
                                myFirstParInV,
                                myLastParInV,
                                3,
                                iu,
                                iv);
        AdvApp2Var_Iso U1 = new(GeomAbs_IsoU,
                                myLastParInU,
                                myFirstParInU,
                                myLastParInU,
                                myFirstParInV,
                                myLastParInV,
                                4,
                                iu,
                                iv);

        NCollection_Sequence<AdvApp2Var_Iso> BU0 = new(), BV0 = new();
        BU0.Append(V0);
        BU0.Append(V1);
        BV0.Append(U0);
        BV0.Append(U1);

        NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> UStrip = new(), VStrip = new();
        UStrip.Append(BU0);
        VStrip.Append(BV0);

        AdvApp2Var_Framework Constraints = new(Bag, UStrip, VStrip);

        // regular cutting if NbInt>1
        double deltu = (myLastParInU - myFirstParInU) / NbInt,
               deltv = (myLastParInV - myFirstParInV) / NbInt;
        for (iint = 1; iint <= NbInt - 1; iint++)
        {
            Result.UpdateInU(myFirstParInU + iint * deltu);
            Constraints.UpdateInU(myFirstParInU + iint * deltu);
            Result.UpdateInV(myFirstParInV + iint * deltv);
            Constraints.UpdateInV(myFirstParInV + iint * deltv);
        }
        myResult = Result;
        myConstraints = Constraints;
    }

    // Computation of the approximation result ; used by Create
    private void Perform(AdvApprox_Cutting UChoice, AdvApprox_Cutting VChoice, AdvApp2Var_EvaluatorFunc2Var Func)
    {
        ComputePatches(UChoice, VChoice, Func);
        myHasResult = myDone = true;
        Compute3DErrors();
    }

    private void Perform(AdvApprox_Cutting UChoice,
                         AdvApprox_Cutting VChoice,
                         AdvApp2Var_EvaluatorFunc2Var Func,
                         AdvApp2Var_Criterion Crit)
    {
        ComputePatches(UChoice, VChoice, Func, Crit);
        myHasResult = myDone = true;
        Compute3DErrors();
        ComputeCritError();
    }

    // Computation of the polynomial approximations ; used by Perform
    private void ComputePatches(AdvApprox_Cutting UChoice,
                                AdvApprox_Cutting VChoice,
                                AdvApp2Var_EvaluatorFunc2Var Func)
    {
        double Udec, Vdec;
        bool Umore, Vmore;
        int NbPatch, NbU, NbV, NumDec;
        int FirstNA = 0;

        while (myResult.FirstNotApprox(ref FirstNA))
        {
            // complete the set of constraints
            ComputeConstraints(UChoice, VChoice, Func);

            // discretization of constraints relative to the square
            myResult[FirstNA].Discretise(myConditions, myConstraints, Func);
            if (!myResult[FirstNA].IsDiscretised())
            {
                myHasResult = myDone = false;
                throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : Surface Discretisation Error");
            }

            // calculate the number and the type of authorized cuts
            // depending on the max number of squares and the validity of next cuts.
            NbU = myResult.NbPatchInU();
            NbV = myResult.NbPatchInV();
            NbPatch = NbU * NbV;
            Umore = UChoice.Value(myResult[FirstNA].U0(), myResult[FirstNA].U1(), out Udec);
            Vmore = VChoice.Value(myResult[FirstNA].V0(), myResult[FirstNA].V1(), out Vdec);

            NumDec = 0;
            if (((NbPatch + NbV) <= myMaxPatches) && ((NbPatch + NbU) > myMaxPatches) && Umore)
            {
                NumDec = 1;
            }
            if (((NbPatch + NbV) > myMaxPatches) && ((NbPatch + NbU) <= myMaxPatches) && Vmore)
            {
                NumDec = 2;
            }
            if (((NbPatch + NbV) <= myMaxPatches) && ((NbPatch + NbU) <= myMaxPatches))
            {
                if (Umore)
                {
                    NumDec = 3;
                }
                if ((NbV > NbU) && Vmore)
                {
                    NumDec = 4;
                }
            }
            if ((NbU + 1) * (NbV + 1) <= myMaxPatches)
            {
                if (!Umore && !Vmore)
                {
                    NumDec = 0;
                }
                if (Umore && !Vmore)
                {
                    NumDec = 3;
                }
                if (!Umore && Vmore)
                {
                    NumDec = 4;
                }
                if (Umore && Vmore)
                {
                    NumDec = 5;
                }
            }

            // approximation of the square
            myResult[FirstNA].MakeApprox(myConditions, myConstraints, NumDec);

            if (!myResult[FirstNA].IsApproximated())
            {
                switch (myResult[FirstNA].CutSense())
                {
                    case 0:
                        //	It is not possible to cut : the result is preserved
                        if (myResult[FirstNA].HasResult())
                        {
                            myResult[FirstNA].OverwriteApprox();
                        }
                        else
                        {
                            myHasResult = myDone = false;
                            throw new Standard_ConstructionError(
                                "AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                        }
                        break;
                    case 1:
                        //      It is necessary to cut in U
                        myResult.UpdateInU(Udec);
                        myConstraints.UpdateInU(Udec);
                        break;
                    case 2:
                        //      It is necessary to cut in V
                        myResult.UpdateInV(Vdec);
                        myConstraints.UpdateInV(Vdec);
                        break;
                    case 3:
                        //      It is necessary to cut in U and V
                        myResult.UpdateInU(Udec);
                        myConstraints.UpdateInU(Udec);
                        myResult.UpdateInV(Vdec);
                        myConstraints.UpdateInV(Vdec);
                        break;
                    default:
                        myHasResult = myDone = false;
                        throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                }
            }
        }
    }

    private void ComputePatches(AdvApprox_Cutting UChoice,
                                AdvApprox_Cutting VChoice,
                                AdvApp2Var_EvaluatorFunc2Var Func,
                                AdvApp2Var_Criterion Crit)
    {
        double Udec, Vdec, CritValue, m1 = 0.0;
        bool Umore, Vmore, CritAbs = Crit.Type() == AdvApp2Var_Absolute;
        int NbPatch, NbU, NbV, NbInt, NumDec;
        int FirstNA = 0, decision = 0;

        while (myResult.FirstNotApprox(ref FirstNA))
        {
            // complete the set of constraints
            ComputeConstraints(UChoice, VChoice, Func, Crit);
            if (decision > 0)
            {
                m1 = 0.0;
            }

            // discretize the constraints relative to the square
            myResult[FirstNA].Discretise(myConditions, myConstraints, Func);
            if (!myResult[FirstNA].IsDiscretised())
            {
                myHasResult = myDone = false;
                throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : Surface Discretisation Error");
            }

            // calculate the number and type of authorized cuts
            // depending on the max number of squares and the validity of next cuts
            NbU = myResult.NbPatchInU();
            NbV = myResult.NbPatchInV();
            NbPatch = NbU * NbV;
            NbInt = NbU;
            Umore = UChoice.Value(myResult[FirstNA].U0(), myResult[FirstNA].U1(), out Udec);
            Vmore = VChoice.Value(myResult[FirstNA].V0(), myResult[FirstNA].V1(), out Vdec);

            NumDec = 0;
            if (((NbPatch + NbV) <= myMaxPatches) && ((NbPatch + NbU) > myMaxPatches) && Umore)
            {
                NumDec = 1;
            }
            if (((NbPatch + NbV) > myMaxPatches) && ((NbPatch + NbU) <= myMaxPatches) && Vmore)
            {
                NumDec = 2;
            }
            if (((NbPatch + NbV) <= myMaxPatches) && ((NbPatch + NbU) <= myMaxPatches))
            {
                if (Umore)
                {
                    NumDec = 3;
                }
                if ((NbV > NbU) && Vmore)
                {
                    NumDec = 4;
                }
            }
            if ((NbU + 1) * (NbV + 1) <= myMaxPatches)
            {
                if (!Umore && !Vmore)
                {
                    NumDec = 0;
                }
                if (Umore && !Vmore)
                {
                    NumDec = 1;
                }
                if (!Umore && Vmore)
                {
                    NumDec = 2;
                }
                if (Umore && Vmore)
                {
                    NumDec = 5;
                }
            }

            // approximation of the square
            if (CritAbs)
            {
                myResult[FirstNA].MakeApprox(myConditions, myConstraints, 0);
            }
            else
            {
                myResult[FirstNA].MakeApprox(myConditions, myConstraints, NumDec);
            }
            if (NumDec >= 3)
            {
                NumDec = NumDec - 2;
            }

            // evaluation of the criterion on the square
            if (myResult[FirstNA].HasResult())
            {
                Crit.Value(myResult[FirstNA], myConditions);
                CritValue = myResult[FirstNA].CritValue();
                if (m1 < CritValue)
                {
                    m1 = CritValue;
                }
            }
            // is it necessary to cut ?
            decision = myResult[FirstNA].CutSense(Crit, NumDec);
            bool Regular = Crit.Repartition() == AdvApp2Var_Regular;
            //    bool Regular = true;
            if (Regular && decision > 0)
            {
                NbInt++;
                InitGrid(NbInt);
            }
            else
            {
                switch (decision)
                {
                    case 0:
                        //	Impossible to cut : the result is preserved
                        if (myResult[FirstNA].HasResult())
                        {
                            myResult[FirstNA].OverwriteApprox();
                        }
                        else
                        {
                            myHasResult = myDone = false;
                            throw new Standard_ConstructionError(
                                "AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                        }
                        break;
                    case 1:
                        //      It is necessary to cut in U
                        myResult.UpdateInU(Udec);
                        myConstraints.UpdateInU(Udec);
                        break;
                    case 2:
                        //      It is necessary to cut in V
                        myResult.UpdateInV(Vdec);
                        myConstraints.UpdateInV(Vdec);
                        break;
                    case 3:
                        //      It is necessary to cut in U and V
                        myResult.UpdateInU(Udec);
                        myConstraints.UpdateInU(Udec);
                        myResult.UpdateInV(Vdec);
                        myConstraints.UpdateInV(Vdec);
                        break;
                    default:
                        myHasResult = myDone = false;
                        throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : Surface Approximation Error");
                }
            }
        }
    }

    // Approximation of the constraints ; used by ComputePatches
    private void ComputeConstraints(AdvApprox_Cutting UChoice,
                                    AdvApprox_Cutting VChoice,
                                    AdvApp2Var_EvaluatorFunc2Var Func)
    {
        double dec;
        bool more;
        int ind1 = 0, ind2 = 0, NbPatch, NbU, NbV;
        int iu = myConditions.UOrder(), iv = myConditions.VOrder();
        AdvApp2Var_Node N1 = new(iu, iv), N2 = new(iu, iv);

        for (AdvApp2Var_Iso? anIso = myConstraints.FirstNotApprox(ref ind1, ref ind2);
             anIso is not null;
             anIso = myConstraints.FirstNotApprox(ref ind1, ref ind2))
        {
            // approximation of iso and calculation of constraints at extremities
            int indN1 = myConstraints.FirstNode(anIso.Type(), ind1, ind2);
            N1.Assign(myConstraints.Node(indN1));
            int indN2 = myConstraints.LastNode(anIso.Type(), ind1, ind2);
            N2.Assign(myConstraints.Node(indN2));

            // note that old code attempted to make copy of anIso here (but copy was incomplete)
            anIso.MakeApprox(myConditions,
                             myFirstParInU,
                             myLastParInU,
                             myFirstParInV,
                             myLastParInV,
                             Func,
                             N1,
                             N2);

            if (anIso.IsApproximated())
            {
                // iso is approached at the required tolerance
                myConstraints.ChangeIso(ind1, ind2, anIso);
                myConstraints.Node(indN1).Assign(N1);
                myConstraints.Node(indN2).Assign(N2);
            }
            else
            {
                // Approximation is not satisfactory
                NbU = myResult.NbPatchInU();
                NbV = myResult.NbPatchInV();
                if (anIso.Type() == GeomAbs_IsoV)
                {
                    NbPatch = (NbU + 1) * NbV;
                    more = UChoice.Value(anIso.T0(), anIso.T1(), out dec);
                }
                else
                {
                    NbPatch = (NbV + 1) * NbU;
                    more = VChoice.Value(anIso.T0(), anIso.T1(), out dec);
                }

                if (NbPatch <= myMaxPatches && more)
                {
                    // It is possible to cut iso
                    if (anIso.Type() == GeomAbs_IsoV)
                    {
                        myResult.UpdateInU(dec);
                        myConstraints.UpdateInU(dec);
                    }
                    else
                    {
                        myResult.UpdateInV(dec);
                        myConstraints.UpdateInV(dec);
                    }
                }
                else
                {
                    // It is not possible to cut : the result is preserved
                    if (anIso.HasResult())
                    {
                        anIso.OverwriteApprox();
                        myConstraints.ChangeIso(ind1, ind2, anIso);
                        myConstraints.Node(indN1).Assign(N1);
                        myConstraints.Node(indN2).Assign(N2);
                    }
                    else
                    {
                        myHasResult = myDone = false;
                        throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : Curve Approximation Error");
                    }
                }
            }
        }
    }

    private void ComputeConstraints(AdvApprox_Cutting UChoice,
                                    AdvApprox_Cutting VChoice,
                                    AdvApp2Var_EvaluatorFunc2Var Func,
                                    AdvApp2Var_Criterion Crit)
    {
        double dec;
        bool more, CritRel = Crit.Type() == AdvApp2Var_Relative;
        int ind1 = 0, ind2 = 0, NbPatch, NbU, NbV;
        int indN1, indN2;
        int iu = myConditions.UOrder(), iv = myConditions.VOrder();
        AdvApp2Var_Node N1 = new(iu, iv), N2 = new(iu, iv);

        for (AdvApp2Var_Iso? anIso = myConstraints.FirstNotApprox(ref ind1, ref ind2);
             anIso is not null;
             anIso = myConstraints.FirstNotApprox(ref ind1, ref ind2))
        {
            // approximation of the iso and calculation of constraints at the extremities
            indN1 = myConstraints.FirstNode(anIso.Type(), ind1, ind2);
            N1.Assign(myConstraints.Node(indN1));
            indN2 = myConstraints.LastNode(anIso.Type(), ind1, ind2);
            N2.Assign(myConstraints.Node(indN2));

            // note that old code attempted to make copy of anIso here (but copy was incomplete)
            anIso.MakeApprox(myConditions,
                             myFirstParInU,
                             myLastParInU,
                             myFirstParInV,
                             myLastParInV,
                             Func,
                             N1,
                             N2);

            if (anIso.IsApproximated())
            {
                // iso is approached at the required tolerance
                myConstraints.ChangeIso(ind1, ind2, anIso);
                myConstraints.Node(indN1).Assign(N1);
                myConstraints.Node(indN2).Assign(N2);
            }
            else
            {
                // Approximation is not satisfactory
                NbU = myResult.NbPatchInU();
                NbV = myResult.NbPatchInV();
                if (anIso.Type() == GeomAbs_IsoV)
                {
                    NbPatch = (NbU + 1) * NbV;
                    more = UChoice.Value(anIso.T0(), anIso.T1(), out dec);
                }
                else
                {
                    NbPatch = (NbV + 1) * NbU;
                    more = VChoice.Value(anIso.T0(), anIso.T1(), out dec);
                }

                // To force Overwrite if the criterion is Absolute
                more = more && CritRel;

                if (NbPatch <= myMaxPatches && more)
                {
                    // It is possible to cut iso
                    if (anIso.Type() == GeomAbs_IsoV)
                    {
                        myResult.UpdateInU(dec);
                        myConstraints.UpdateInU(dec);
                    }
                    else
                    {
                        myResult.UpdateInV(dec);
                        myConstraints.UpdateInV(dec);
                    }
                }
                else
                {
                    // It is not possible to cut: the result is preserved
                    if (anIso.HasResult())
                    {
                        anIso.OverwriteApprox();
                        myConstraints.ChangeIso(ind1, ind2, anIso);
                        myConstraints.Node(indN1).Assign(N1);
                        myConstraints.Node(indN2).Assign(N2);
                    }
                    else
                    {
                        myHasResult = myDone = false;
                        throw new Standard_ConstructionError("AdvApp2Var_ApproxAFunc2Var : Curve Approximation Error");
                    }
                }
            }
        }
    }

    // Computation of the 3D errors on the approximation result ; used by Perform
    private void Compute3DErrors()
    {
        int iesp, ipat;
        double error_max, error_moy, error_U0, error_V0, error_U1, error_V1;
        double Tol, F1Tol, F2Tol, F3Tol, F4Tol;
        if (myNumSubSpaces[2] > 0)
        {
            my3DMaxError = new NCollection_HArray1<double>(1, myNumSubSpaces[2]);
            my3DAverageError = new NCollection_HArray1<double>(1, myNumSubSpaces[2]);
            my3DUFrontError = new NCollection_HArray1<double>(1, myNumSubSpaces[2]);
            my3DVFrontError = new NCollection_HArray1<double>(1, myNumSubSpaces[2]);
            for (iesp = 1; iesp <= myNumSubSpaces[2]; iesp++)
            {
                error_max = 0;
                error_moy = 0.0;
                error_U0 = 0.0;
                error_V0 = 0.0;
                error_U1 = 0.0;
                error_V1 = 0.0;
                Tol = my3DTolerances!.Value(iesp);
                F1Tol = my3DTolOnFront!.Value(iesp, 1);
                F2Tol = my3DTolOnFront.Value(iesp, 2);
                F3Tol = my3DTolOnFront.Value(iesp, 3);
                F4Tol = my3DTolOnFront.Value(iesp, 4);
                for (ipat = 1; ipat <= myResult.NbPatch(); ipat++)
                {
                    error_max = Max(myResult[ipat].MaxErrors()!.Value(iesp), error_max);
                    error_U0 = Max(myResult[ipat].IsoErrors()!.Value(iesp, 3), error_U0);
                    error_U1 = Max(myResult[ipat].IsoErrors()!.Value(iesp, 4), error_U1);
                    error_V0 = Max(myResult[ipat].IsoErrors()!.Value(iesp, 1), error_V0);
                    error_V1 = Max(myResult[ipat].IsoErrors()!.Value(iesp, 2), error_V1);
                    error_moy += myResult[ipat].AverageErrors()!.Value(iesp);
                }
                my3DMaxError.SetValue(iesp, error_max);
                my3DUFrontError.SetValue(iesp, Max(error_U0, error_U1));
                my3DVFrontError.SetValue(iesp, Max(error_V0, error_V1));
                error_moy /= (double)myResult.NbPatch();
                my3DAverageError.SetValue(iesp, error_moy);
                if (error_max > Tol || error_U0 > F3Tol || error_U1 > F4Tol || error_V0 > F1Tol || error_V1 > F2Tol)
                {
                    myDone = false;
                }
            }
        }
    }

    // Computation of the max value of the criterion on the approximation result ; used by Perform
    private void ComputeCritError()
    {
        int iesp, ipat;
        double crit_max;
        if (myNumSubSpaces[2] > 0)
        {
            for (iesp = 1; iesp <= myNumSubSpaces[2]; iesp++)
            {
                crit_max = 0.0;
                for (ipat = 1; ipat <= myResult.NbPatch(); ipat++)
                {
                    crit_max = Max(myResult[ipat].CritValue(), crit_max);
                }
                myCriterionError = crit_max;
            }
        }
    }

    // Conversion of the approximation result in BSpline; used by Create
    private void ConvertBS()
    {
        // Homogeneization of degrees
        int iu = myConditions.UOrder(), iv = myConditions.VOrder();
        int ncfu = myConditions.ULimit(), ncfv = myConditions.VLimit();
        myResult.SameDegree(iu, iv, out ncfu, out ncfv);
        myDegreeInU = ncfu - 1;
        myDegreeInV = ncfv - 1;

        // Calculate resulting surfaces
        mySurfaces = new NCollection_HArray1<Geom_Surface?>(1, myNumSubSpaces[2]);

        int j;
        NCollection_Array1<double> UKnots = new(1, myResult.NbPatchInU() + 1);
        for (j = 1; j <= UKnots.Length(); j++)
        {
            UKnots.SetValue(j, myResult.UParameter(j));
        }

        NCollection_Array1<double> VKnots = new(1, myResult.NbPatchInV() + 1);
        for (j = 1; j <= VKnots.Length(); j++)
        {
            VKnots.SetValue(j, myResult.VParameter(j));
        }

        // Prepare data for conversion grid of polynoms --> poles
        NCollection_HArray1<double> Uint1 = new(1, 2);
        Uint1.SetValue(1, -1);
        Uint1.SetValue(2, 1);
        NCollection_HArray1<double> Vint1 = new(1, 2);
        Vint1.SetValue(1, -1);
        Vint1.SetValue(2, 1);

        NCollection_HArray1<double> Uint2 = new(1, myResult.NbPatchInU() + 1);
        for (j = 1; j <= Uint2.Length(); j++)
        {
            Uint2.SetValue(j, myResult.UParameter(j));
        }
        NCollection_HArray1<double> Vint2 = new(1, myResult.NbPatchInV() + 1);
        for (j = 1; j <= Vint2.Length(); j++)
        {
            Vint2.SetValue(j, myResult.VParameter(j));
        }

        int nmax = myResult.NbPatchInU() * myResult.NbPatchInV(),
            Size_eq = myConditions.ULimit() * myConditions.VLimit() * 3;

        NCollection_HArray2<int> NbCoeff = new(1, nmax, 1, 2);
        NCollection_HArray1<double> Poly = new(1, nmax * Size_eq);

        int SSP, i;
        for (SSP = 1; SSP <= myNumSubSpaces[2]; SSP++)
        {
            // Creation of the grid of polynoms
            int n = 0, icf = 1, ieq;
            for (j = 1; j <= myResult.NbPatchInV(); j++)
            {
                for (i = 1; i <= myResult.NbPatchInU(); i++)
                {
                    n++;
                    NbCoeff.SetValue(n, 1, myResult.Patch(i, j).NbCoeffInU());
                    NbCoeff.SetValue(n, 2, myResult.Patch(i, j).NbCoeffInV());
                    for (ieq = 1; ieq <= Size_eq; ieq++)
                    {
                        Poly.SetValue(icf, myResult.Patch(i, j).Coefficients(SSP, myConditions)!.Value(ieq));
                        icf++;
                    }
                }
            }

            // Conversion into poles
            Convert_GridPolynomialToPoles CvP = new(myResult.NbPatchInU(),
                                                    myResult.NbPatchInV(),
                                                    iu,
                                                    iv,
                                                    myMaxDegInU,
                                                    myMaxDegInV,
                                                    NbCoeff.Array2(),
                                                    Poly.Array1(),
                                                    Uint1.Array1(),
                                                    Vint1.Array1(),
                                                    Uint2.Array1(),
                                                    Vint2.Array1());
            if (!CvP.IsDone())
            {
                myDone = false;
            }

            // Conversion into BSpline
            Geom_BSplineSurface aSurface = new(CvP.Poles(),
                                               CvP.UKnots(),
                                               CvP.VKnots(),
                                               CvP.UMultiplicities(),
                                               CvP.VMultiplicities(),
                                               CvP.UDegree(),
                                               CvP.VDegree());
            // The port: the result is handed to callers, so it is frozen (decision 1906)
            aSurface.Freeze();
            mySurfaces.ChangeValue(SSP) = aSurface;
        }
    }

    /// <summary>
    /// True if the approximation succeeded within the imposed tolerances and the wished continuities.
    /// </summary>
    public bool IsDone() => myDone;

    /// <summary>
    /// True if the approximation came out with a result, not necessarily within the required tolerance or with the
    /// wished continuities.
    /// </summary>
    public bool HasResult() => myHasResult;

    /// <summary>The B-spline surface of the subspace <paramref name="SSPIndex"/> (frozen; decision 1906).</summary>
    public Geom_BSplineSurface Surface(int SSPIndex) => (Geom_BSplineSurface)mySurfaces!.Value(SSPIndex)!;

    public int UDegree() => myDegreeInU;

    public int VDegree() => myDegreeInV;

    public int NumSubSpaces(int Dimension) => myNumSubSpaces[Dimension - 1];

    private static NCollection_HArray1<double>? copyOf(NCollection_HArray1<double>? theArray) =>
        theArray is null ? null : new NCollection_HArray1<double>(theArray);

    /// <summary>The maximum errors of the subspaces of dimension <paramref name="Dimension"/>: a copy, or null.</summary>
    /// <exception cref="Standard_OutOfRange">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? MaxError(int Dimension)
    {
        NCollection_HArray1<double>? EPtr = null;
        if (Dimension < 1 || Dimension > 3)
        {
            throw new Standard_OutOfRange(
                "AdvApp2Var_ApproxAFunc2Var::MaxError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (Dimension)
        {
            case 1:
                EPtr = my1DMaxError;
                break;
            case 2:
                EPtr = my2DMaxError;
                break;
            case 3:
                EPtr = my3DMaxError;
                break;
        }
        return copyOf(EPtr);
    }

    /// <summary>The average errors of the subspaces of dimension <paramref name="Dimension"/>: a copy, or null.</summary>
    /// <exception cref="Standard_OutOfRange">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? AverageError(int Dimension)
    {
        NCollection_HArray1<double>? EPtr = null;
        if (Dimension < 1 || Dimension > 3)
        {
            throw new Standard_OutOfRange(
                "AdvApp2Var_ApproxAFunc2Var::AverageError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (Dimension)
        {
            case 1:
                EPtr = my1DAverageError;
                break;
            case 2:
                EPtr = my2DAverageError;
                break;
            case 3:
                EPtr = my3DAverageError;
                break;
        }
        return copyOf(EPtr);
    }

    /// <summary>The maximum errors on the U frontiers (dimension 3 only is computed): a copy, or null.</summary>
    /// <exception cref="Standard_OutOfRange">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? UFrontError(int Dimension)
    {
        NCollection_HArray1<double>? EPtr = null;
        if (Dimension < 1 || Dimension > 3)
        {
            throw new Standard_OutOfRange(
                "AdvApp2Var_ApproxAFunc2Var::UFrontError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (Dimension)
        {
            case 1:
                EPtr = my1DUFrontError;
                break;
            case 2:
                EPtr = my2DUFrontError;
                break;
            case 3:
                EPtr = my3DUFrontError;
                break;
        }
        return copyOf(EPtr);
    }

    /// <summary>The maximum errors on the V frontiers (dimension 3 only is computed): a copy, or null.</summary>
    /// <exception cref="Standard_OutOfRange">A dimension other than 1, 2 or 3.</exception>
    public NCollection_HArray1<double>? VFrontError(int Dimension)
    {
        NCollection_HArray1<double>? EPtr = null;
        if (Dimension <= 0 || Dimension > 3)
        {
            throw new Standard_OutOfRange(
                "AdvApp2Var_ApproxAFunc2Var::VFrontError : Dimension must be equal to 1,2 or 3 !");
        }
        switch (Dimension)
        {
            case 1:
                EPtr = my1DVFrontError;
                break;
            case 2:
                EPtr = my2DVFrontError;
                break;
            case 3:
                EPtr = my3DVFrontError;
                break;
        }
        return copyOf(EPtr);
    }

    /// <summary>The maximum error of the surface <paramref name="SSPIndex"/>.</summary>
    /// <exception cref="Standard_OutOfRange">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double MaxError(int Dimension, int SSPIndex)
    {
        if (Dimension != 3 || SSPIndex != 1)
        {
            throw new Standard_OutOfRange("AdvApp2Var_ApproxAFunc2Var::MaxError: ONE Surface 3D only !");
        }
        return my3DMaxError!.Value(SSPIndex);
    }

    /// <summary>The average error of the surface <paramref name="SSPIndex"/>.</summary>
    /// <exception cref="Standard_OutOfRange">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double AverageError(int Dimension, int SSPIndex)
    {
        if (Dimension != 3 || SSPIndex != 1)
        {
            throw new Standard_OutOfRange("AdvApp2Var_ApproxAFunc2Var::AverageError : ONE Surface 3D only !");
        }
        return my3DAverageError!.Value(SSPIndex);
    }

    /// <summary>The maximum error on the U frontiers of the surface <paramref name="SSPIndex"/>.</summary>
    /// <exception cref="Standard_OutOfRange">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double UFrontError(int Dimension, int SSPIndex)
    {
        if (Dimension != 3 || SSPIndex != 1)
        {
            throw new Standard_OutOfRange("AdvApp2Var_ApproxAFunc2Var::UFrontError : ONE Surface 3D only !");
        }
        return my3DUFrontError!.Value(SSPIndex);
    }

    /// <summary>The maximum error on the V frontiers of the surface <paramref name="SSPIndex"/>.</summary>
    /// <exception cref="Standard_OutOfRange">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double VFrontError(int Dimension, int SSPIndex)
    {
        if (Dimension != 3 || SSPIndex != 1)
        {
            throw new Standard_OutOfRange("AdvApp2Var_ApproxAFunc2Var::VFrontError : ONE Surface 3D only !");
        }
        return my3DVFrontError!.Value(SSPIndex);
    }

    /// <summary>The maximum value of the criterion over the patches.</summary>
    /// <exception cref="Standard_OutOfRange">Unless Dimension is 3 and SSPIndex 1.</exception>
    public double CritError(int Dimension, int SSPIndex)
    {
        if (Dimension != 3 || SSPIndex != 1)
        {
            throw new Standard_OutOfRange("AdvApp2Var_ApproxAFunc2Var::CritError: ONE Surface 3D only !");
        }
        return myCriterionError;
    }

    /// <summary>Writes information on the result (C++: <c>Dump(Standard_OStream&amp;)</c>).</summary>
    public void Dump(TextWriter o)
    {
        int iesp = 1, NbKU, NbKV, ik;
        CultureInfo c = CultureInfo.InvariantCulture;
        o.Write('\n');
        if (!myHasResult)
        {
            o.Write("No result" + '\n');
        }
        else
        {
            o.Write("There is a result");
            if (myDone)
            {
                o.Write(" within the requested tolerance " + my3DTolerances!.Value(iesp).ToString(c) + '\n');
            }
            else if (my3DMaxError!.Value(iesp) > my3DTolerances!.Value(iesp))
            {
                o.Write(" WITHOUT the requested tolerance " + my3DTolerances.Value(iesp).ToString(c) + '\n');
            }
            else
            {
                o.Write(" WITHOUT the requested continuities " + '\n');
            }
            o.Write('\n');
            o.Write("Result max error :" + my3DMaxError!.Value(iesp).ToString(c) + '\n');
            o.Write("Result average error :" + my3DAverageError!.Value(iesp).ToString(c) + '\n');
            o.Write("Result max error on U frontiers :" + my3DUFrontError!.Value(iesp).ToString(c) + '\n');
            o.Write("Result max error on V frontiers :" + my3DVFrontError!.Value(iesp).ToString(c) + '\n');
            o.Write('\n');
            o.Write("Degree of Bezier patches in U : " + myDegreeInU.ToString(c) + "  in V : "
                    + myDegreeInV.ToString(c) + '\n');
            o.Write('\n');
            Geom_BSplineSurface S = (Geom_BSplineSurface)mySurfaces!.Value(iesp)!;
            o.Write("Number of poles in U : " + S.NbUPoles().ToString(c) + "  in V : " + S.NbVPoles().ToString(c)
                    + '\n');
            o.Write('\n');
            NbKU = S.NbUKnots();
            NbKV = S.NbVKnots();
            o.Write("Number of knots in U : " + NbKU.ToString(c) + '\n');
            for (ik = 1; ik <= NbKU; ik++)
            {
                o.Write("   " + ik.ToString(c) + " : " + S.UKnot(ik).ToString(c) + "   mult : "
                        + S.UMultiplicity(ik).ToString(c) + '\n');
            }
            o.Write('\n');
            o.Write("Number of knots in V : " + NbKV.ToString(c) + '\n');
            for (ik = 1; ik <= NbKV; ik++)
            {
                o.Write("   " + ik.ToString(c) + " : " + S.VKnot(ik).ToString(c) + "   mult : "
                        + S.VMultiplicity(ik).ToString(c) + '\n');
            }
            o.Write('\n');
        }
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
