// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CLProps3d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.LProp_Status;

namespace Occt;

/// <summary>
/// The local properties of a curve adaptor at a parameter: point, derivatives up to order 3, tangent, curvature, normal and
/// centre of curvature. Derivatives are computed when the parameter is set, up to the order given to the
/// constructor, and later on demand.
/// </summary>
/// <remarks>
/// OCCT's <c>LProp_CLProps3d</c>, <c>GeomLProp_CLPropsBase&lt;gp_Pnt, gp_Vec, gp_Dir, occ::handle&lt;Adaptor3d_Curve&gt;&gt;</c>:
/// <see cref="GeomLProp_CLProps"/> on an adaptor (generated from it by substituting the type). A working object: its
/// setters are public. The resolution is the linear tolerance used to decide that a vector is null.
/// </remarks>
public class LProp_CLProps3d
{
    private Adaptor3d_Curve? myCurve;
    private double myU;
    private int myDerOrder;
    private int myCN;
    private double myLinTol;
    private gp_Pnt myPnt;
    private readonly gp_Vec[] myDerivArr = new gp_Vec[3];
    private double myCurvature = 0.0;
    private LProp_Status myTangentStatus;
    private int mySignificantFirstDerivativeOrder = 0;

    /// <summary>
    /// The properties of the curve C, computing derivatives up to order N (0 to 3) when a parameter is set, with the
    /// linear tolerance Resolution.
    /// </summary>
    public LProp_CLProps3d(Adaptor3d_Curve C, int N, double Resolution)
    {
        myCurve = C;
        myU = RealLast();
        myDerOrder = N;
        myCN = 4;
        myLinTol = Resolution;
        myTangentStatus = LProp_Undecided;
        Standard_OutOfRange_Raise_if(N < 0 || N > 3, "GeomLProp_CLProps::GeomLProp_CLProps()");
    }

    /// <summary>As the previous constructor, and sets the parameter U.</summary>
    public LProp_CLProps3d(Adaptor3d_Curve C, double U, int N, double Resolution)
    {
        myCurve = C;
        myDerOrder = N;
        myCN = 4;
        myLinTol = Resolution;
        myTangentStatus = LProp_Undecided;
        Standard_OutOfRange_Raise_if(N < 0 || N > 3, "GeomLProp_CLProps::GeomLProp_CLProps()");
        SetParameter(U);
    }

    /// <summary>Properties without a curve yet: <see cref="SetCurve"/> gives it.</summary>
    public LProp_CLProps3d(int N, double Resolution)
    {
        myCurve = null;
        myU = RealLast();
        myDerOrder = N;
        myCN = 0;
        myLinTol = Resolution;
        myTangentStatus = LProp_Undecided;
        Standard_OutOfRange_Raise_if(N < 0 || N > 3, "GeomLProp_CLProps() - invalid input");
    }

    /// <summary>Sets the parameter, computing the point and the derivatives up to the order given.</summary>
    public void SetParameter(double U)
    {
        // LProp_CurveUtils::SetParameter
        myU = U;
        EvalDerivatives(myU, myDerOrder);
        myTangentStatus = LProp_Undecided;
    }

    /// <summary>Sets the curve; the parameter must then be set again.</summary>
    public void SetCurve(Adaptor3d_Curve C)
    {
        myCurve = C;
        myCN = 4;
    }

    /// <summary>The point.</summary>
    public gp_Pnt Value() => myPnt;

    /// <summary>The first derivative, computed if it hasn't been.</summary>
    public gp_Vec D1() => EnsureDeriv(1);

    /// <summary>The second derivative, computed if it hasn't been.</summary>
    public gp_Vec D2() => EnsureDeriv(2);

    /// <summary>The third derivative, computed if it hasn't been.</summary>
    public gp_Vec D3() => EnsureDeriv(3);

    /// <summary>
    /// True if the tangent is defined: one of the first three derivatives is longer than the resolution (the first
    /// such is the "significant" one).
    /// </summary>
    public bool IsTangentDefined()
    {
        // LProp_CurveUtils::IsTangentDefined
        if (myTangentStatus == LProp_Undefined)
        {
            return false;
        }

        if (myTangentStatus >= LProp_Defined)
        {
            return true;
        }

        double aTolSq = myLinTol * myLinTol;
        int anOrder = 0;
        while (anOrder++ < 4)
        {
            if (myCN >= anOrder)
            {
                gp_Vec aV;
                switch (anOrder)
                {
                    case 1:
                        aV = D1();
                        break;
                    case 2:
                        aV = D2();
                        break;
                    case 3:
                        aV = D3();
                        break;
                    default:
                        myTangentStatus = LProp_Undefined;
                        return false;
                }

                if (aV.SquareMagnitude() > aTolSq)
                {
                    mySignificantFirstDerivativeOrder = anOrder;
                    myTangentStatus = LProp_Defined;
                    return true;
                }
            }
            else
            {
                myTangentStatus = LProp_Undefined;
                return false;
            }
        }

        return false;
    }

    /// <summary>The tangent direction; throws LProp_NotDefined if it isn't defined.</summary>
    public void Tangent(out gp_Dir D)
    {
        // LProp_CurveUtils::Tangent
        if (!IsTangentDefined())
        {
            throw new LProp_NotDefined();
        }

        ComputeTangent(out D);
    }

    /// <summary>
    /// The curvature: RealLast() if the first derivative is null (the tangent comes from a higher derivative), 0 if
    /// the second derivative is null or collinear with the first. Throws LProp_NotDefined if the tangent isn't
    /// defined. Uses the second derivative as computed: give an order of at least 2.
    /// </summary>
    public double Curvature()
    {
        // LProp_CurveUtils::Curvature
        bool anIsDefined = IsTangentDefined();
        if (!anIsDefined)
        {
            throw new LProp_NotDefined("CLProps::Curvature()");
        }

        if (mySignificantFirstDerivativeOrder > 1)
        {
            return RealLast();
        }

        myCurvature = LProp_CurveUtils.ComputeCurvature(myDerivArr[0], myDerivArr[1], myLinTol * myLinTol);
        return myCurvature;
    }

    /// <summary>The normal direction; throws LProp_NotDefined if the curvature is null or infinite.</summary>
    public void Normal(out gp_Dir N)
    {
        // LProp_CurveUtils::Normal
        double aCurvature = Curvature();
        if (aCurvature == RealLast() || Math.Abs(aCurvature) <= myLinTol)
        {
            throw new LProp_NotDefined("CLProps::Normal(): Curvature is null or infinity");
        }

        LProp_CurveUtils.ComputeNormal(myDerivArr[0], myDerivArr[1], out N);
    }

    /// <summary>The centre of curvature; throws LProp_NotDefined if the curvature is null.</summary>
    public void CentreOfCurvature(out gp_Pnt P)
    {
        // LProp_CurveUtils::CentreOfCurvature
        if (Math.Abs(Curvature()) <= myLinTol)
        {
            throw new LProp_NotDefined();
        }

        // OCCT: when Curvature() returns RealLast() (a null first derivative) it doesn't update myCurvature, so the
        // centre is computed with the curvature of an earlier call (0 if none)
        LProp_CurveUtils.ComputeCentreOfCurvature(myPnt, myDerivArr[0], myDerivArr[1], myCurvature, out P);
    }

    // LProp_CurveUtils::EvalDerivatives, with OCCT's DirectAccess
    private void EvalDerivatives(double theU, int theOrder)
    {
        switch (theOrder)
        {
            case 0:
                myCurve!.D0(theU, out myPnt);
                break;
            case 1:
                myCurve!.D1(theU, out myPnt, out myDerivArr[0]);
                break;
            case 2:
                myCurve!.D2(theU, out myPnt, out myDerivArr[0], out myDerivArr[1]);
                break;
            case 3:
                myCurve!.D3(theU, out myPnt, out myDerivArr[0], out myDerivArr[1], out myDerivArr[2]);
                break;
        }
    }

    // LProp_CurveUtils::EnsureDeriv
    private gp_Vec EnsureDeriv(int theRequired)
    {
        if (myDerOrder < theRequired)
        {
            myDerOrder = theRequired;
            EvalDerivatives(myU, myDerOrder);
        }

        return myDerivArr[theRequired - 1];
    }

    // LProp_CurveUtils::ComputeTangent: when the first significant derivative has order > 1, its sign is taken from
    // the chord to a nearby point
    private void ComputeTangent(out gp_Dir theDir)
    {
        int theSigOrder = mySignificantFirstDerivativeOrder;
        if (theSigOrder == 1)
        {
            theDir = new gp_Dir(myDerivArr[0]);
            return;
        }

        const double THE_DIVISION_FACTOR = 1.0e-3;
        const double THE_MIN_STEP = 1.0e-7;

        double anUsupremum = myCurve!.LastParameter();
        double anUinfimum = myCurve.FirstParameter();

        double aDu;
        if ((anUsupremum >= RealLast()) || (anUinfimum <= RealFirst()))
        {
            aDu = 0.0;
        }
        else
        {
            aDu = anUsupremum - anUinfimum;
        }

        double aDelta = Max(aDu * THE_DIVISION_FACTOR, THE_MIN_STEP);

        gp_Vec aV = myDerivArr[theSigOrder - 1];

        double anOtherU;
        if (myU - anUinfimum < aDelta)
        {
            anOtherU = myU + aDelta;
        }
        else
        {
            anOtherU = myU - aDelta;
        }

        myCurve.D0(Min(myU, anOtherU), out gp_Pnt aP1);
        myCurve.D0(Max(myU, anOtherU), out gp_Pnt aP2);

        gp_Vec aChord = new(aP1, aP2);
        if (aV.Dot(aChord) < 0.0)
        {
            aV = -aV;
        }

        theDir = new gp_Dir(aV);
    }
}
