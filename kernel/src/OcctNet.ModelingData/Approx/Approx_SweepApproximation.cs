// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepApproximation.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepApproximation.lxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SweepApproximation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Approximates a surface S(u, v) (and possibly associated 2D curves) defined by a law of sections: F(u, v) where
/// Ft(u) = F(u, t) is a B-spline curve, given by an <see cref="Approx_SweepFunction"/>. Used by blending and sweeping.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Perform"/> runs the approximation. The accessors <see cref="SurfPoles"/>,
/// <see cref="SurfWeights"/>, the knot and multiplicity accessors and <see cref="Curve2dPoles"/> return the object's
/// own arrays (OCCT's const references; shared: don't modify); <see cref="Surface"/> and <see cref="Curve2d"/> copy
/// into the caller's arrays.
/// </remarks>
public sealed class Approx_SweepApproximation
{
    private sealed class Approx_SweepApproximation_Eval : AdvApprox_EvaluatorFunction
    {
        private readonly Approx_SweepApproximation Tool;

        public Approx_SweepApproximation_Eval(Approx_SweepApproximation theTool)
        {
            Tool = theTool;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ErrorCode)
        {
            ErrorCode = Tool.Eval(Parameter, DerivativeRequest, StartEnd[0], StartEnd[1], Result);
        }
    }

    private readonly Approx_SweepFunction myFunc;
    private bool done;
    private int Num1DSS;
    private int Num2DSS;
    private int Num3DSS;
    private int udeg;
    private int vdeg;
    private int deg2d;
    private NCollection_HArray2<Point3D>? tabPoles;
    private NCollection_HArray2<double>? tabWeights;
    private NCollection_HArray1<double>? tabUKnots;
    private NCollection_HArray1<double>? tabVKnots;
    private NCollection_HArray1<double>? tab2dKnots;
    private NCollection_HArray1<int>? tabUMults;
    private NCollection_HArray1<int>? tabVMults;
    private NCollection_HArray1<int>? tab2dMults;
    private readonly NCollection_Sequence<NCollection_HArray1<Point2D>> seqPoles2d = new();
    private NCollection_HArray1<double>? MError1d;
    private NCollection_HArray1<double>? tab2dError;
    private NCollection_HArray1<double>? MError3d;
    private NCollection_HArray1<double>? AError1d;
    private NCollection_HArray1<double>? Ave2dError;
    private NCollection_HArray1<double>? AError3d;
    private NCollection_HArray1<GeneralTransform2D>? AAffin;
    private NCollection_HArray1<double>? COnSurfErr;
    private Vector3D Translation = new();
    private NCollection_HArray1<Point3D>? myPoles;
    private NCollection_HArray1<Point2D>? myPoles2d;
    private NCollection_HArray1<double>? myWeigths;
    private NCollection_HArray1<Vector3D>? myDPoles;
    private NCollection_HArray1<Vector3D>? myD2Poles;
    private NCollection_HArray1<Vector2D>? myDPoles2d;
    private NCollection_HArray1<Vector2D>? myD2Poles2d;
    private NCollection_HArray1<double>? myDWeigths;
    private NCollection_HArray1<double>? myD2Weigths;
    private int myOrder;
    private double myParam;
    private double first;
    private double last;

    /// <summary>An approximation of the sweep of Func (kept, and changed by the approximation as in OCCT).</summary>
    public Approx_SweepApproximation(Approx_SweepFunction Func)
    {
        myFunc = Func;
        //  Init of variables of control
        myParam = 0;
        myOrder = -1;
        first = 1.0e100;
        last = -1.0e100;
        done = false;
    }

    /// <summary>
    /// Performs the approximation on [First, Last]: Tol3d is the tolerance of the surface, BoundTol that at its
    /// boundaries, Tol2d the largest tolerance of the 2D curves (normally given by the resolutions of the support
    /// surfaces), TolAngular the angle (in radians) between the tangents of the section law and of the iso-v of the
    /// surface; Continuity, Degmax and Segmax the continuity, maximum degree and maximum number of spans in v.
    /// </summary>
    public void Perform(double First,
                        double Last,
                        double Tol3d,
                        double BoundTol,
                        double Tol2d,
                        double TolAngular,
                        Continuity Continuity = C0,
                        int Degmax = 11,
                        int Segmax = 50)
    {
        int NbPolSect, NbKnotSect, ii;
        double Tol, Tol3dMin = Tol3d, The3D2DTol = 0;
        Continuity continuity = Continuity;

        // (1) Characteristics of a section
        myFunc.SectionShape(out NbPolSect, out NbKnotSect, out udeg);
        Num2DSS = myFunc.Nb2dCurves();
        tabUKnots = new NCollection_HArray1<double>(1, NbKnotSect);
        tabUMults = new NCollection_HArray1<int>(1, NbKnotSect);
        myFunc.Knots(tabUKnots.ChangeArray1());
        myFunc.Mults(tabUMults.ChangeArray1());

        // (2) Decompositition into sub-spaces
        NCollection_HArray1<double>? OneDTol, TwoDTol, ThreeDTol;
        Num3DSS = NbPolSect;

        // (2.1) Tolerance 3d and 1d
        OneDTol = new NCollection_HArray1<double>(1, Num3DSS);
        ThreeDTol = new NCollection_HArray1<double>(1, Num3DSS);

        myFunc.GetTolerance(BoundTol, Tol3d, TolAngular, ThreeDTol.ChangeArray1());

        for (ii = 1; ii <= Num3DSS; ii++)
        {
            if (ThreeDTol.Value(ii) < Tol3dMin)
            {
                Tol3dMin = ThreeDTol.Value(ii);
            }
        }

        if (myFunc.IsRational())
        {
            double Size;
            Num1DSS = NbPolSect;
            NCollection_Array1<double> Wmin = new(1, Num1DSS);
            myFunc.GetMinimalWeight(Wmin);
            Size = myFunc.MaximalSection();
            Translation.SetXYZ(myFunc.BarycentreOfSurf().XYZ());
            for (ii = 1; ii <= Num3DSS; ii++)
            {
                Tol = ThreeDTol.Value(ii) / 2; // To take account of the error on the final result.
                OneDTol.SetValue(ii, Tol * Wmin[ii] / Size);
                Tol *= Wmin[ii]; // Factor of projection
                ThreeDTol.SetValue(ii, Max(Tol, 1.0e-20));
            }
        }
        else
        {
            Num1DSS = 0;
        }

        // (2.2) Tolerance and Transformation 2d.
        if (Num2DSS == 0)
        {
            TwoDTol = null;
        }
        else
        {
            // for 2d define affinity using resolutions, to
            // avoid homogeneous tolerance of approximation (u/v and 2d/3d)
            double res, tolu, tolv;
            TwoDTol = new NCollection_HArray1<double>(1, Num2DSS);
            AAffin = new NCollection_HArray1<GeneralTransform2D>(1, Num2DSS);
            The3D2DTol = 0.9 * BoundTol; // 10% of security
            for (ii = 1; ii <= Num2DSS; ii++)
            {
                myFunc.Resolution(ii, The3D2DTol, out tolu, out tolv);
                if (tolu > tolv)
                {
                    res = tolv;
                    AAffin[ii].SetValue(1, 1, tolv / tolu);
                }
                else
                {
                    res = tolu;
                    AAffin[ii].SetValue(2, 2, tolu / tolv);
                }
                TwoDTol.SetValue(ii, Min(Tol2d, res));
            }
        }

        // (3) Approximation

        // Init
        myPoles = new NCollection_HArray1<Point3D>(1, Num3DSS);
        myDPoles = new NCollection_HArray1<Vector3D>(1, Num3DSS);
        myD2Poles = new NCollection_HArray1<Vector3D>(1, Num3DSS);

        myWeigths = new NCollection_HArray1<double>(1, Num3DSS);
        myDWeigths = new NCollection_HArray1<double>(1, Num3DSS);
        myD2Weigths = new NCollection_HArray1<double>(1, Num3DSS);

        if (Num2DSS > 0)
        {
            myPoles2d = new NCollection_HArray1<Point2D>(1, Num2DSS);
            myDPoles2d = new NCollection_HArray1<Vector2D>(1, Num2DSS);
            myD2Poles2d = new NCollection_HArray1<Vector2D>(1, Num2DSS);
            COnSurfErr = new NCollection_HArray1<double>(1, Num2DSS);
        }
        else
        {
            myPoles2d = new NCollection_HArray1<Point2D>();
            myDPoles2d = new NCollection_HArray1<Vector2D>();
            myD2Poles2d = new NCollection_HArray1<Vector2D>();
            COnSurfErr = new NCollection_HArray1<double>();
        }

        // Checks if myFunc->D2 is implemented
        if (continuity >= C2)
        {
            bool B;
            B = myFunc.D2(First,
                          First,
                          Last,
                          myPoles.ChangeArray1(),
                          myDPoles.ChangeArray1(),
                          myD2Poles.ChangeArray1(),
                          myPoles2d.ChangeArray1(),
                          myDPoles2d.ChangeArray1(),
                          myD2Poles2d.ChangeArray1(),
                          myWeigths.ChangeArray1(),
                          myDWeigths.ChangeArray1(),
                          myD2Weigths.ChangeArray1());
            if (!B)
            {
                continuity = C1;
            }
        }
        // Checks if myFunc->D1 is implemented
        if (continuity == C1)
        {
            bool B;
            B = myFunc.D1(First,
                          First,
                          Last,
                          myPoles.ChangeArray1(),
                          myDPoles.ChangeArray1(),
                          myPoles2d.ChangeArray1(),
                          myDPoles2d.ChangeArray1(),
                          myWeigths.ChangeArray1(),
                          myDWeigths.ChangeArray1());
            if (!B)
            {
                continuity = C0;
            }
        }

        // So that F was at least 20 times more exact than its approx
        myFunc.SetTolerance(Tol3dMin / 20, Tol2d / 20);

        int NbIntervalC2 = myFunc.NbIntervals(C2);
        int NbIntervalC3 = myFunc.NbIntervals(C3);

        if (NbIntervalC3 > 1)
        {
            // (3.1) Approximation with preferential cut
            NCollection_Array1<double> Param_de_decoupeC2 = new(1, NbIntervalC2 + 1);
            myFunc.Intervals(Param_de_decoupeC2, C2);
            NCollection_Array1<double> Param_de_decoupeC3 = new(1, NbIntervalC3 + 1);
            myFunc.Intervals(Param_de_decoupeC3, C3);

            AdvApprox_PrefAndRec Preferentiel = new(Param_de_decoupeC2, Param_de_decoupeC3);

            Approx_SweepApproximation_Eval ev = new(this);
            Approximation(OneDTol,
                          TwoDTol,
                          ThreeDTol,
                          The3D2DTol,
                          First,
                          Last,
                          continuity,
                          Degmax,
                          Segmax,
                          ev,
                          Preferentiel);
        }
        else
        {
            // (3.2) Approximation without preferential cut
            AdvApprox_DichoCutting Dichotomie = new();
            Approx_SweepApproximation_Eval ev = new(this);
            Approximation(OneDTol,
                          TwoDTol,
                          ThreeDTol,
                          The3D2DTol,
                          First,
                          Last,
                          continuity,
                          Degmax,
                          Segmax,
                          ev,
                          Dichotomie);
        }
    }

    // Calls F(t) and stores the results.
    private void Approximation(NCollection_HArray1<double>? OneDTol,
                               NCollection_HArray1<double>? TwoDTol,
                               NCollection_HArray1<double>? ThreeDTol,
                               double BoundTol,
                               double First,
                               double Last,
                               Continuity Continuity,
                               int Degmax,
                               int Segmax,
                               AdvApprox_EvaluatorFunction TheApproxFunction,
                               AdvApprox_Cutting TheCuttingTool)
    {
        AdvApprox_ApproxAFunction Approx = new(Num1DSS,
                                               Num2DSS,
                                               Num3DSS,
                                               OneDTol,
                                               TwoDTol,
                                               ThreeDTol,
                                               First,
                                               Last,
                                               Continuity,
                                               Degmax,
                                               Segmax,
                                               TheApproxFunction,
                                               TheCuttingTool);
        done = Approx.HasResult();

        if (done)
        {
            // --> Fill Champs of the surface ----
            int ii, jj;

            vdeg = Approx.Degree();
            int aNbPoles = Approx.NbPoles();
            // Unfortunately Adv_Approx stores the transposition of the required
            // so, writing tabPoles = Approx.Poles() will give an erroneous result
            // It is only possible to allocate and recopy term by term...
            tabPoles = new NCollection_HArray2<Point3D>(1, Num3DSS, 1, aNbPoles);
            tabWeights = new NCollection_HArray2<double>(1, Num3DSS, 1, aNbPoles);

            // OCCT reads Approx.Poles() (a handle) in the loops; the port's accessor copies, so it is read once.
            NCollection_HArray2<Point3D> aPoles = Approx.Poles()!;
            if (Num1DSS == Num3DSS)
            {
                NCollection_HArray2<double> aPoles1d = Approx.Poles1d()!;
                double wpoid;
                Point3D P;
                for (ii = 1; ii <= Num3DSS; ii++)
                {
                    for (jj = 1; jj <= aNbPoles; jj++)
                    {
                        P = aPoles.Value(jj, ii);
                        wpoid = aPoles1d.Value(jj, ii);
                        P.ChangeCoord().Divide(wpoid); // It is necessary to divide poles by weight
                        P.Translate(Translation);
                        tabPoles.SetValue(ii, jj, P);
                        tabWeights.SetValue(ii, jj, wpoid);
                    }
                }
            }
            else
            {
                tabWeights.Init(1);
                for (ii = 1; ii <= Num3DSS; ii++)
                {
                    for (jj = 1; jj <= aNbPoles; jj++)
                    {
                        tabPoles.SetValue(ii, jj, aPoles.Value(jj, ii));
                    }
                }
            }

            // this is better
            tabVKnots = Approx.Knots();
            tabVMults = Approx.Multiplicities();

            // --> Filling of curves 2D  ----------
            if (Num2DSS > 0)
            {
                GeneralTransform2D TrsfInv;
                deg2d = vdeg;
                tab2dKnots = Approx.Knots();
                tab2dMults = Approx.Multiplicities();

                for (ii = 1; ii <= Num2DSS; ii++)
                {
                    TrsfInv = AAffin!.Value(ii).Inverted();
                    NCollection_HArray1<Point2D> P2d = new(1, aNbPoles);
                    Approx.Poles2d(ii, P2d.ChangeArray1());
                    // do not forget to apply inverted homothety.
                    for (jj = 1; jj <= aNbPoles; jj++)
                    {
                        TrsfInv.Transforms(ref P2d[jj].ChangeCoord());
                    }
                    seqPoles2d.Append(P2d);
                }
            }
            // ---> Filling of errors
            MError3d = new NCollection_HArray1<double>(1, Num3DSS);
            AError3d = new NCollection_HArray1<double>(1, Num3DSS);
            for (ii = 1; ii <= Num3DSS; ii++)
            {
                MError3d.SetValue(ii, Approx.MaxError(3, ii));
                AError3d.SetValue(ii, Approx.AverageError(3, ii));
            }

            if (myFunc.IsRational())
            {
                MError1d = new NCollection_HArray1<double>(1, Num3DSS);
                AError1d = new NCollection_HArray1<double>(1, Num3DSS);
                for (ii = 1; ii <= Num1DSS; ii++)
                {
                    MError1d.SetValue(ii, Approx.MaxError(1, ii));
                    AError1d.SetValue(ii, Approx.AverageError(1, ii));
                }
            }

            if (Num2DSS > 0)
            {
                tab2dError = new NCollection_HArray1<double>(1, Num2DSS);
                Ave2dError = new NCollection_HArray1<double>(1, Num2DSS);
                for (ii = 1; ii <= Num2DSS; ii++)
                {
                    tab2dError.SetValue(ii, Approx.MaxError(2, ii));
                    Ave2dError.SetValue(ii, Approx.AverageError(2, ii));
                    COnSurfErr!.SetValue(ii, (tab2dError.Value(ii) / TwoDTol!.Value(ii)) * BoundTol);
                }
            }
        }
    }

    /// <summary>
    /// The evaluator of AdvApprox: writes the derivative DerivativeRequest of the 1D, 2D and 3D poles at Parameter
    /// into Result, and returns the error code (0 when done).
    /// </summary>
    public int Eval(double Parameter, int DerivativeRequest, double First, double Last, Span<double> Result)
    {
        int ier = 0;
        switch (DerivativeRequest)
        {
            case 0:
                ier = (!D0(Parameter, First, Last, Result)) ? 1 : 0;
                break;
            case 1:
                ier = (!D1(Parameter, First, Last, Result)) ? 1 : 0;
                break;
            case 2:
                ier = (!D2(Parameter, First, Last, Result)) ? 1 : 0;
                break;
            default:
                ier = 2;
                break;
        }
        return ier;
    }

    private bool D0(double Param, double First, double Last, Span<double> LocalResult)
    {
        int index, ii;
        bool Ok = true;

        // Management of limits
        if ((first != First) || (Last != last))
        {
            myFunc.SetInterval(First, Last);
        }

        if ((Param != myParam) || (myOrder < 0) || (first != First) || (Last != last))
        {
            // Positioning in case when the last operation is not repeated.
            Ok = myFunc.D0(Param,
                           First,
                           Last,
                           myPoles!.ChangeArray1(),
                           myPoles2d!.ChangeArray1(),
                           myWeigths!.ChangeArray1());

            //  poles3d are multiplied by weight after translation.
            for (ii = 1; ii <= Num1DSS; ii++)
            {
                myPoles[ii].ChangeCoord().Subtract(Translation.XYZ());
                myPoles[ii].ChangeCoord().Multiply(myWeigths.Value(ii));
            }

            //  The transformation is applied to poles 2d.
            for (ii = 1; ii <= Num2DSS; ii++)
            {
                AAffin!.Value(ii).Transforms(ref myPoles2d[ii].ChangeCoord());
            }

            // Update variables of control and return
            first = First;
            last = Last;
            myOrder = 0;
            myParam = Param;
        }

        //  Extraction of results
        index = 0;
        for (ii = 1; ii <= Num1DSS; ii++)
        {
            LocalResult[index] = myWeigths!.Value(ii);
            index++;
        }
        for (ii = 1; ii <= Num2DSS; ii++)
        {
            LocalResult[index] = myPoles2d!.Value(ii).X();
            LocalResult[index + 1] = myPoles2d.Value(ii).Y();
            index += 2;
        }
        for (ii = 1; ii <= Num3DSS; ii++, index += 3)
        {
            LocalResult[index] = myPoles!.Value(ii).X();
            LocalResult[index + 1] = myPoles.Value(ii).Y();
            LocalResult[index + 2] = myPoles.Value(ii).Z();
        }

        return Ok;
    }

    private bool D1(double Param, double First, double Last, Span<double> LocalResult)
    {
        Xy Vcoord;
        Vector3D Vaux = new();
        int index, ii;
        bool Ok = true;

        if ((first != First) || (Last != last))
        {
            myFunc.SetInterval(First, Last);
        }

        if ((Param != myParam) || (myOrder < 1) || (first != First) || (Last != last))
        {
            // Positioning
            Ok = myFunc.D1(Param,
                           First,
                           Last,
                           myPoles!.ChangeArray1(),
                           myDPoles!.ChangeArray1(),
                           myPoles2d!.ChangeArray1(),
                           myDPoles2d!.ChangeArray1(),
                           myWeigths!.ChangeArray1(),
                           myDWeigths!.ChangeArray1());

            //  Take into account the multiplication of poles3d by weights.
            //  and the translation.
            for (ii = 1; ii <= Num1DSS; ii++)
            {
                // Translation on the section
                myPoles[ii].ChangeCoord().Subtract(Translation.XYZ());
                // Homothety on all.
                double aWeight = myWeigths.Value(ii);
                myDPoles[ii].Multiply(aWeight);
                Vaux.SetXYZ(myPoles.Value(ii).Coord());
                myDPoles[ii].Add(myDWeigths.Value(ii) * Vaux);
                myPoles[ii].ChangeCoord().Multiply(aWeight); // for the cash
            }

            //  Apply transformation 2d to suitable vectors
            for (ii = 1; ii <= Num2DSS; ii++)
            {
                Vcoord = myDPoles2d.Value(ii).XY();
                AAffin!.Value(ii).Transforms(ref Vcoord);
                myDPoles2d[ii].SetXY(Vcoord);
                AAffin.Value(ii).Transforms(ref myPoles2d[ii].ChangeCoord());
            }

            // Update control variables and return
            first = First;
            last = Last;
            myOrder = 1;
            myParam = Param;
        }

        //  Extraction of results
        index = 0;
        for (ii = 1; ii <= Num1DSS; ii++)
        {
            LocalResult[index] = myDWeigths!.Value(ii);
            index++;
        }
        for (ii = 1; ii <= Num2DSS; ii++)
        {
            LocalResult[index] = myDPoles2d!.Value(ii).X();
            LocalResult[index + 1] = myDPoles2d.Value(ii).Y();
            index += 2;
        }
        for (ii = 1; ii <= Num3DSS; ii++, index += 3)
        {
            LocalResult[index] = myDPoles!.Value(ii).X();
            LocalResult[index + 1] = myDPoles.Value(ii).Y();
            LocalResult[index + 2] = myDPoles.Value(ii).Z();
        }
        return Ok;
    }

    private bool D2(double Param, double First, double Last, Span<double> LocalResult)
    {
        Xy Vcoord;
        Vector3D Vaux = new();
        int index, ii;
        bool Ok = true;

        // management of limits
        if ((first != First) || (Last != last))
        {
            myFunc.SetInterval(First, Last);
        }

        if ((Param != myParam) || (myOrder < 2) || (first != First) || (Last != last))
        {
            // Positioning in case when the last operation is not repeated
            Ok = myFunc.D2(Param,
                           First,
                           Last,
                           myPoles!.ChangeArray1(),
                           myDPoles!.ChangeArray1(),
                           myD2Poles!.ChangeArray1(),
                           myPoles2d!.ChangeArray1(),
                           myDPoles2d!.ChangeArray1(),
                           myD2Poles2d!.ChangeArray1(),
                           myWeigths!.ChangeArray1(),
                           myDWeigths!.ChangeArray1(),
                           myD2Weigths!.ChangeArray1());

            //  Multiply poles3d by the weight after translations.
            for (ii = 1; ii <= Num1DSS; ii++)
            {
                // First translate
                myPoles[ii].ChangeCoord().Subtract(Translation.XYZ());

                // Calculate the second derivative
                myD2Poles[ii].Multiply(myWeigths.Value(ii));
                Vaux.SetXYZ(myDPoles.Value(ii).XYZ());
                myD2Poles[ii].Add((2 * myDWeigths.Value(ii)) * Vaux);
                Vaux.SetXYZ(myPoles.Value(ii).Coord());
                myD2Poles[ii].Add(myD2Weigths.Value(ii) * Vaux);

                // Then the remainder for the cash
                myDPoles[ii].Multiply(myWeigths.Value(ii));
                Vaux.SetXYZ(myPoles.Value(ii).Coord());
                myDPoles[ii].Add(myDWeigths.Value(ii) * Vaux);
                myPoles[ii].ChangeCoord().Multiply(myWeigths.Value(ii));
            }

            //  Apply transformation to poles 2d.
            for (ii = 1; ii <= Num2DSS; ii++)
            {
                Vcoord = myD2Poles2d.Value(ii).XY();
                AAffin!.Value(ii).Transforms(ref Vcoord);
                myD2Poles2d[ii].SetXY(Vcoord);
                Vcoord = myDPoles2d.Value(ii).XY();
                AAffin.Value(ii).Transforms(ref Vcoord);
                myDPoles2d[ii].SetXY(Vcoord);
                AAffin.Value(ii).Transforms(ref myPoles2d[ii].ChangeCoord());
            }

            // Update variables of control and return
            first = First;
            last = Last;
            myOrder = 2;
            myParam = Param;
        }

        //  Extraction of results
        index = 0;
        for (ii = 1; ii <= Num1DSS; ii++)
        {
            LocalResult[index] = myD2Weigths!.Value(ii);
            index++;
        }
        for (ii = 1; ii <= Num2DSS; ii++)
        {
            LocalResult[index] = myD2Poles2d!.Value(ii).X();
            LocalResult[index + 1] = myD2Poles2d.Value(ii).Y();
            index += 2;
        }
        for (ii = 1; ii <= Num3DSS; ii++, index += 3)
        {
            LocalResult[index] = myD2Poles!.Value(ii).X();
            LocalResult[index + 1] = myD2Poles.Value(ii).Y();
            LocalResult[index + 2] = myD2Poles.Value(ii).Z();
        }

        return Ok;
    }

    /// <summary>Whether the approximation has a result.</summary>
    public bool IsDone() => done;

    /// <summary>The shape of the approximated surface: degrees and numbers of poles and knots in U and V.</summary>
    public void SurfShape(out int UDegree, out int VDegree, out int NbUPoles, out int NbVPoles, out int NbUKnots,
                          out int NbVKnots)
    {
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        UDegree = udeg;
        VDegree = vdeg;
        NbUPoles = tabPoles!.ColLength();
        NbVPoles = tabPoles.RowLength();
        NbUKnots = tabUKnots!.Length();
        NbVKnots = tabVKnots!.Length();
    }

    /// <summary>Copies the approximated surface into the caller's arrays (sized by <see cref="SurfShape"/>).</summary>
    public void Surface(NCollection_Array2<Point3D> TPoles,
                        NCollection_Array2<double> TWeights,
                        NCollection_Array1<double> TUKnots,
                        NCollection_Array1<double> TVKnots,
                        NCollection_Array1<int> TUMults,
                        NCollection_Array1<int> TVMults)
    {
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        TPoles.Assign(tabPoles!.Array2());
        TWeights.Assign(tabWeights!.Array2());
        TUKnots.Assign(tabUKnots!.Array1());
        TUMults.Assign(tabUMults!.Array1());
        TVKnots.Assign(tabVKnots!.Array1());
        TVMults.Assign(tabVMults!.Array1());
    }

    public int UDegree()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return udeg;
    }

    public int VDegree()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return vdeg;
    }

    /// <summary>The poles of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array2<Point3D> SurfPoles()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return tabPoles!.Array2();
    }

    /// <summary>The weights of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array2<double> SurfWeights()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return tabWeights!.Array2();
    }

    /// <summary>The U knots of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<double> SurfUKnots()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return tabUKnots!.Array1();
    }

    /// <summary>The V knots of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<double> SurfVKnots()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return tabVKnots!.Array1();
    }

    /// <summary>The U multiplicities of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<int> SurfUMults()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return tabUMults!.Array1();
    }

    /// <summary>The V multiplicities of the surface (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<int> SurfVMults()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return tabVMults!.Array1();
    }

    /// <summary>The maximum error of the surface approximation.</summary>
    public double MaxErrorOnSurf()
    {
        int ii;
        double MaxError = 0, err;
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }

        if (myFunc.IsRational())
        {
            NCollection_Array1<double> Wmin = new(1, Num1DSS);
            myFunc.GetMinimalWeight(Wmin);
            double Size = myFunc.MaximalSection();
            for (ii = 1; ii <= Num3DSS; ii++)
            {
                err = (Size * MError1d!.Value(ii) + MError3d!.Value(ii)) / Wmin[ii];
                if (err > MaxError)
                {
                    MaxError = err;
                }
            }
        }
        else
        {
            for (ii = 1; ii <= Num3DSS; ii++)
            {
                err = MError3d!.Value(ii);
                if (err > MaxError)
                {
                    MaxError = err;
                }
            }
        }
        return MaxError;
    }

    /// <summary>The average error of the surface approximation.</summary>
    public double AverageErrorOnSurf()
    {
        int ii;
        double MoyError = 0, err;
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }

        if (myFunc.IsRational())
        {
            NCollection_Array1<double> Wmin = new(1, Num1DSS);
            myFunc.GetMinimalWeight(Wmin);
            double Size = myFunc.MaximalSection();
            for (ii = 1; ii <= Num3DSS; ii++)
            {
                err = (Size * AError1d!.Value(ii) + AError3d!.Value(ii)) / Wmin[ii];
                MoyError += err;
            }
        }
        else
        {
            for (ii = 1; ii <= Num3DSS; ii++)
            {
                err = AError3d!.Value(ii);
                MoyError += err;
            }
        }
        return MoyError / Num3DSS;
    }

    public int NbCurves2d()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        return Num2DSS;
    }

    /// <summary>The shape of the 2D curves: degree and numbers of poles and knots.</summary>
    public void Curves2dShape(out int Degree, out int NbPoles, out int NbKnots)
    {
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException("Approx_SweepApproximation");
        }
        Degree = deg2d;
        NbPoles = seqPoles2d.Value(1).Length();
        NbKnots = tab2dKnots!.Length();
    }

    /// <summary>Copies the 2D curve Index into the caller's arrays (sized by <see cref="Curves2dShape"/>).</summary>
    public void Curve2d(int Index,
                        NCollection_Array1<Point2D> TPoles,
                        NCollection_Array1<double> TKnots,
                        NCollection_Array1<int> TMults)
    {
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException("Approx_SweepApproximation");
        }
        TPoles.Assign(seqPoles2d.Value(Index).Array1());
        TKnots.Assign(tab2dKnots!.Array1());
        TMults.Assign(tab2dMults!.Array1());
    }

    public int Curves2dDegree()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return deg2d;
    }

    /// <summary>The poles of the 2D curve Index (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<Point2D> Curve2dPoles(int Index)
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return seqPoles2d.Value(Index).Array1();
    }

    /// <summary>The knots of the 2D curves (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<double> Curves2dKnots()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return tab2dKnots!.Array1();
    }

    /// <summary>The multiplicities of the 2D curves (the object's own array; shared: don't modify).</summary>
    public NCollection_Array1<int> Curves2dMults()
    {
        if (!done)
        {
            throw new NotDoneException(" Approx_SweepApproximation");
        }
        if (seqPoles2d.Length() == 0)
        {
            throw new DomainErrorException();
        }
        return tab2dMults!.Array1();
    }

    /// <summary>The maximum error of the approximation of the 2D curve Index.</summary>
    public double Max2dError(int Index)
    {
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        return tab2dError!.Value(Index);
    }

    /// <summary>The average error of the approximation of the 2D curve Index.</summary>
    public double Average2dError(int Index)
    {
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        return Ave2dError!.Value(Index);
    }

    /// <summary>The maximum 3D error, on the surface, of the approximation of the 2D curve Index.</summary>
    public double TolCurveOnSurf(int Index)
    {
        if (!done)
        {
            throw new NotDoneException("Approx_SweepApproximation");
        }
        return COnSurfErr!.Value(Index);
    }

    /// <summary>Writes information on the approximation.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("Dump of SweepApproximation" + '\n');
        if (done)
        {
            o.Write("Error 3d = " + MaxErrorOnSurf().ToString(CultureInfo.InvariantCulture) + '\n');

            if (Num2DSS > 0)
            {
                o.Write("Error 2d = ");
                for (int ii = 1; ii <= Num2DSS; ii++)
                {
                    o.Write(Max2dError(ii).ToString(CultureInfo.InvariantCulture));
                    if (ii < Num2DSS)
                    {
                        o.Write(" , " + '\n');
                    }
                }
                // OCCT writes this line end and " Not Done " to std::cout instead of o; the port writes them to o.
                o.Write('\n');
            }
            o.Write((tabVKnots!.Length() - 1).ToString(CultureInfo.InvariantCulture) + " Segment(s) of degree "
                    + vdeg.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        else
        {
            o.Write(" Not Done " + '\n');
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
