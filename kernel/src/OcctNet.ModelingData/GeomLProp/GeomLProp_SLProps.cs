// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_SLProps.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.LProp_Status;

namespace Occt;

/// <summary>
/// The local properties of a surface at a pair of parameters: point, first and second derivatives, tangent
/// directions, normal, and the principal, mean and Gaussian curvatures with their directions. Derivatives are
/// computed when the parameters are set, up to the order given to the constructor, and later on demand.
/// </summary>
/// <remarks>
/// OCCT's <c>GeomLProp_SLPropsBase&lt;occ::handle&lt;Geom_Surface&gt;&gt;</c>, with the code of
/// <c>LProp_SurfaceUtils</c>'s templates written out (one class per instantiation: this one and
/// <see cref="LProp_SLProps3d"/>, with identical bodies but for the bounds). A working object: its setters are
/// public. The resolution is the linear tolerance used to decide that a vector is null.
/// </remarks>
public class GeomLProp_SLProps
{
    private Geom_Surface? mySurf;
    private double myU;
    private double myV;
    private int myDerOrder;
    private int myCN;
    private double myLinTol;
    private gp_Pnt myPnt;
    private gp_Vec myD1u;
    private gp_Vec myD1v;
    private gp_Vec myD2u;
    private gp_Vec myD2v;
    private gp_Vec myDuv;
    private gp_Dir myNormal = new();
    private double myMinCurv = 0.0;
    private double myMaxCurv = 0.0;
    private gp_Dir myDirMinCurv = new();
    private gp_Dir myDirMaxCurv = new();
    private double myMeanCurv = 0.0;
    private double myGausCurv = 0.0;
    private int mySignificantFirstDerivativeOrderU = 0;
    private int mySignificantFirstDerivativeOrderV = 0;
    private LProp_Status myUTangentStatus;
    private LProp_Status myVTangentStatus;
    private LProp_Status myNormalStatus;
    private LProp_Status myCurvatureStatus;

    /// <summary>
    /// The properties of the surface S at (U, V), computing derivatives up to order N (0 to 2), with the linear
    /// tolerance Resolution.
    /// </summary>
    public GeomLProp_SLProps(Geom_Surface S, double U, double V, int N, double Resolution)
    {
        mySurf = S;
        myU = RealLast();
        myV = RealLast();
        myDerOrder = N;
        myCN = 4;
        myLinTol = Resolution;
        myUTangentStatus = LProp_Undecided;
        myVTangentStatus = LProp_Undecided;
        myNormalStatus = LProp_Undecided;
        myCurvatureStatus = LProp_Undecided;
        Standard_OutOfRange_Raise_if(N < 0 || N > 2, "GeomLProp_SLPropsBase::GeomLProp_SLPropsBase()");
        SetParameters(U, V);
    }

    /// <summary>As the previous constructor, without setting the parameters.</summary>
    public GeomLProp_SLProps(Geom_Surface S, int N, double Resolution)
    {
        mySurf = S;
        myU = RealLast();
        myV = RealLast();
        myDerOrder = N;
        myCN = 4;
        myLinTol = Resolution;
        myUTangentStatus = LProp_Undecided;
        myVTangentStatus = LProp_Undecided;
        myNormalStatus = LProp_Undecided;
        myCurvatureStatus = LProp_Undecided;
        Standard_OutOfRange_Raise_if(N < 0 || N > 2, "GeomLProp_SLPropsBase::GeomLProp_SLPropsBase()");
    }

    /// <summary>Properties without a surface yet: <see cref="SetSurface"/> gives it.</summary>
    public GeomLProp_SLProps(int N, double Resolution)
    {
        mySurf = null;
        myU = RealLast();
        myV = RealLast();
        myDerOrder = N;
        myCN = 0;
        myLinTol = Resolution;
        myUTangentStatus = LProp_Undecided;
        myVTangentStatus = LProp_Undecided;
        myNormalStatus = LProp_Undecided;
        myCurvatureStatus = LProp_Undecided;
        Standard_OutOfRange_Raise_if(N < 0 || N > 2, "GeomLProp_SLPropsBase::GeomLProp_SLPropsBase() bad level");
    }

    /// <summary>Sets the surface; the parameters must then be set again.</summary>
    public void SetSurface(Geom_Surface S)
    {
        mySurf = S;
        myCN = 4;
    }

    /// <summary>Sets the parameters, computing the point and the derivatives up to the order given.</summary>
    public void SetParameters(double U, double V)
    {
        // LProp_SurfaceUtils::SetParameters
        myU = U;
        myV = V;
        EvalSurfDerivatives(myU, myV, myDerOrder);
        myUTangentStatus = LProp_Undecided;
        myVTangentStatus = LProp_Undecided;
        myNormalStatus = LProp_Undecided;
        myCurvatureStatus = LProp_Undecided;
    }

    /// <summary>The point.</summary>
    public gp_Pnt Value() => myPnt;

    /// <summary>The first U derivative, computed if it hasn't been.</summary>
    public gp_Vec D1U()
    {
        EnsureSurfDeriv(1);
        return myD1u;
    }

    /// <summary>The first V derivative, computed if it hasn't been.</summary>
    public gp_Vec D1V()
    {
        EnsureSurfDeriv(1);
        return myD1v;
    }

    /// <summary>The second U derivative, computed if it hasn't been.</summary>
    public gp_Vec D2U()
    {
        EnsureSurfDeriv(2);
        return myD2u;
    }

    /// <summary>The second V derivative, computed if it hasn't been.</summary>
    public gp_Vec D2V()
    {
        EnsureSurfDeriv(2);
        return myD2v;
    }

    /// <summary>The second cross derivative, computed if it hasn't been.</summary>
    public gp_Vec DUV()
    {
        EnsureSurfDeriv(2);
        return myDuv;
    }

    /// <summary>True if the U tangent is defined (one of the first two U derivatives isn't null).</summary>
    public bool IsTangentUDefined()
    {
        // LProp_SurfaceUtils::IsTangentUDefined
        if (myUTangentStatus == LProp_Undefined)
        {
            return false;
        }

        if (myUTangentStatus >= LProp_Defined)
        {
            return true;
        }

        // OCCT passes D1U() and D2U() as references to the fields, read after both are evaluated
        D1U();
        D2U();
        return LProp_SurfaceUtils.FindSurfTangentOrder(myD1u,
                                                       myD2u,
                                                       myCN,
                                                       myLinTol * myLinTol,
                                                       out mySignificantFirstDerivativeOrderU,
                                                       out myUTangentStatus);
    }

    /// <summary>The tangent direction on the iso-V; throws LProp_NotDefined if it isn't defined.</summary>
    public void TangentU(out gp_Dir D)
    {
        // LProp_SurfaceUtils::TangentU
        if (!IsTangentUDefined())
        {
            throw new LProp_NotDefined();
        }

        ComputeSurfTangent(myD1u, myD2u, mySignificantFirstDerivativeOrderU, true, out D);
    }

    /// <summary>True if the V tangent is defined (one of the first two V derivatives isn't null).</summary>
    public bool IsTangentVDefined()
    {
        // LProp_SurfaceUtils::IsTangentVDefined
        if (myVTangentStatus == LProp_Undefined)
        {
            return false;
        }

        if (myVTangentStatus >= LProp_Defined)
        {
            return true;
        }

        // OCCT passes D1V() and D2V() as references to the fields, read after both are evaluated
        D1V();
        D2V();
        return LProp_SurfaceUtils.FindSurfTangentOrder(myD1v,
                                                       myD2v,
                                                       myCN,
                                                       myLinTol * myLinTol,
                                                       out mySignificantFirstDerivativeOrderV,
                                                       out myVTangentStatus);
    }

    /// <summary>The tangent direction on the iso-U; throws LProp_NotDefined if it isn't defined.</summary>
    public void TangentV(out gp_Dir D)
    {
        // LProp_SurfaceUtils::TangentV
        if (!IsTangentVDefined())
        {
            throw new LProp_NotDefined();
        }

        ComputeSurfTangent(myD1v, myD2v, mySignificantFirstDerivativeOrderV, false, out D);
    }

    /// <summary>True if the normal is defined (by CSLib::Normal on the first derivatives).</summary>
    public bool IsNormalDefined()
    {
        return LProp_SurfaceUtils.IsNormalDefined(myD1u, myD1v, myLinTol, ref myNormal, ref myNormalStatus);
    }

    /// <summary>The normal direction; throws LProp_NotDefined if it isn't defined.</summary>
    public gp_Dir Normal()
    {
        // LProp_SurfaceUtils::Normal
        if (!IsNormalDefined())
        {
            throw new LProp_NotDefined();
        }

        return myNormal;
    }

    /// <summary>
    /// True if the curvatures are defined: the surface is at least C2 here, the normal and both tangents are defined,
    /// and the principal curvatures can be computed.
    /// </summary>
    public bool IsCurvatureDefined()
    {
        // LProp_SurfaceUtils::IsCurvatureDefined
        if (myCurvatureStatus == LProp_Undefined)
        {
            return false;
        }

        if (myCurvatureStatus >= LProp_Defined)
        {
            return true;
        }

        if (myCN < 2)
        {
            myCurvatureStatus = LProp_Undefined;
            return false;
        }

        if (!IsNormalDefined())
        {
            myCurvatureStatus = LProp_Undefined;
            return false;
        }

        if (!IsTangentUDefined() || !IsTangentVDefined())
        {
            myCurvatureStatus = LProp_Undefined;
            return false;
        }

        if (myDerOrder < 2)
        {
            D2U();
        }

        if (LProp_SurfaceUtils.ComputeSurfCurvatures(myD1u,
                                                     myD1v,
                                                     myD2u,
                                                     myD2v,
                                                     myDuv,
                                                     myNormal,
                                                     ref myMinCurv,
                                                     ref myMaxCurv,
                                                     ref myDirMinCurv,
                                                     ref myDirMaxCurv,
                                                     ref myMeanCurv,
                                                     ref myGausCurv))
        {
            myCurvatureStatus = LProp_Computed;
            return true;
        }

        myCurvatureStatus = LProp_Undefined;
        return false;
    }

    /// <summary>
    /// True if the point is umbilic (equal principal curvatures). Throws LProp_NotDefined if the curvatures aren't
    /// defined.
    /// </summary>
    /// <remarks>
    /// OCCT passes the curvatures by value before computing them, so the first call after a parameter change compares
    /// the previous parameters' curvatures (0 and 0, so true, on a new object). Call <see cref="IsCurvatureDefined"/>
    /// first.
    /// </remarks>
    public bool IsUmbilic()
    {
        // OCCT: LProp_SurfaceUtils::IsUmbilic(*this, myMaxCurv, myMinCurv) copies the fields before it calls
        // IsCurvatureDefined(), which computes them
        double theMaxCurv = myMaxCurv;
        double theMinCurv = myMinCurv;
        if (!IsCurvatureDefined())
        {
            throw new LProp_NotDefined();
        }

        return Math.Abs(theMaxCurv - theMinCurv) < Math.Abs(Epsilon(theMaxCurv));
    }

    /// <summary>The maximum curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    /// <remarks>
    /// As in OCCT, the first call after a parameter change returns the previous parameters' value (0 on a new
    /// object): call <see cref="IsCurvatureDefined"/> first.
    /// </remarks>
    public double MaxCurvature() => RequireCurvature(myMaxCurv);

    /// <summary>The minimum curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    /// <remarks>As <see cref="MaxCurvature"/>, call <see cref="IsCurvatureDefined"/> first.</remarks>
    public double MinCurvature() => RequireCurvature(myMinCurv);

    /// <summary>The directions of the maximum and minimum curvatures; throws LProp_NotDefined if not defined.</summary>
    public void CurvatureDirections(out gp_Dir MaxD, out gp_Dir MinD)
    {
        // LProp_SurfaceUtils::CurvatureDirections
        if (!IsCurvatureDefined())
        {
            throw new LProp_NotDefined();
        }

        MaxD = myDirMaxCurv;
        MinD = myDirMinCurv;
    }

    /// <summary>The mean curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    /// <remarks>As <see cref="MaxCurvature"/>, call <see cref="IsCurvatureDefined"/> first.</remarks>
    public double MeanCurvature() => RequireCurvature(myMeanCurv);

    /// <summary>The Gaussian curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    /// <remarks>As <see cref="MaxCurvature"/>, call <see cref="IsCurvatureDefined"/> first.</remarks>
    public double GaussianCurvature() => RequireCurvature(myGausCurv);

    // LProp_SurfaceUtils::RequireCurvature. OCCT: the value is passed by value, before IsCurvatureDefined() computes it
    private double RequireCurvature(double theValue)
    {
        if (!IsCurvatureDefined())
        {
            throw new LProp_NotDefined();
        }

        return theValue;
    }

    // LProp_SurfaceUtils::EvalSurfDerivatives, with OCCT's DirectAccess
    private void EvalSurfDerivatives(double theU, double theV, int theOrder)
    {
        switch (theOrder)
        {
            case 0:
                mySurf!.D0(theU, theV, out myPnt);
                break;
            case 1:
                mySurf!.D1(theU, theV, out myPnt, out myD1u, out myD1v);
                break;
            case 2:
                mySurf!.D2(theU, theV, out myPnt, out myD1u, out myD1v, out myD2u, out myD2v, out myDuv);
                break;
        }
    }

    // LProp_SurfaceUtils::EnsureSurfDeriv
    private void EnsureSurfDeriv(int theRequired)
    {
        if (myDerOrder < theRequired)
        {
            myDerOrder = theRequired;
            EvalSurfDerivatives(myU, myV, myDerOrder);
        }
    }

    // LProp_SurfaceUtils::GetSurfBounds (Geom_Surface: Bounds in the order U1, U2, V1, V2)
    private void GetSurfBounds(out double theU1, out double theV1, out double theU2, out double theV2)
    {
        mySurf!.Bounds(out theU1, out theU2, out theV1, out theV2);
    }

    // LProp_SurfaceUtils::ComputeSurfTangent: when the first significant derivative has order 2, its sign is taken
    // from the chord to a nearby point
    private void ComputeSurfTangent(in gp_Vec theFirstDeriv, in gp_Vec theSecDeriv, int theSigOrder, bool theIsU,
                                    out gp_Dir theDir)
    {
        if (theSigOrder == 1)
        {
            theDir = new gp_Dir(theFirstDeriv);
            return;
        }

        const double THE_DIVISION_FACTOR = 1.0e-3;
        const double THE_MIN_STEP = 1.0e-7;

        GetSurfBounds(out double anUinfimum, out double anVinfimum, out double anUsupremum, out double anVsupremum);

        if (theIsU)
        {
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

            gp_Vec aV = theSecDeriv;

            double anOtherU;
            if (myU - anUinfimum < aDelta)
            {
                anOtherU = myU + aDelta;
            }
            else
            {
                anOtherU = myU - aDelta;
            }

            mySurf!.D0(Min(myU, anOtherU), myV, out gp_Pnt aP1);
            mySurf.D0(Max(myU, anOtherU), myV, out gp_Pnt aP2);

            gp_Vec aChord = new(aP1, aP2);
            if (aV.Dot(aChord) < 0.0)
            {
                aV = -aV;
            }

            theDir = new gp_Dir(aV);
        }
        else
        {
            double aDv;
            if ((anVsupremum >= RealLast()) || (anVinfimum <= RealFirst()))
            {
                aDv = 0.0;
            }
            else
            {
                aDv = anVsupremum - anVinfimum;
            }

            double aDelta = Max(aDv * THE_DIVISION_FACTOR, THE_MIN_STEP);

            gp_Vec aV = theSecDeriv;

            double anOtherV;
            if (myV - anVinfimum < aDelta)
            {
                anOtherV = myV + aDelta;
            }
            else
            {
                anOtherV = myV - aDelta;
            }

            mySurf!.D0(myU, Min(myV, anOtherV), out gp_Pnt aP1);
            mySurf.D0(myU, Max(myV, anOtherV), out gp_Pnt aP2);

            gp_Vec aChord = new(aP1, aP2);
            if (aV.Dot(aChord) < 0.0)
            {
                aV = -aV;
            }

            theDir = new gp_Dir(aV);
        }
    }
}
