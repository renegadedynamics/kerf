// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_RepUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Dispatch and validation of evaluation representations (<see cref="Geom2dEval_RepCurveDesc"/>) for the
/// Geom2d curves that accept one. An internal C++ namespace in OCCT. The <c>TryEval</c> functions return false,
/// with their result set to zero, when the descriptor is null or can't evaluate the request; the caller then
/// evaluates the geometry itself.
/// </summary>
internal static class Geom2dEval_RepUtils
{
    internal static void ValidateCurveDesc(Geom2dEval_RepCurveDesc.Base? theDesc, Geom2d_Curve theOwner)
    {
        if (theDesc is null)
        {
            return;
        }
        if (theDesc.Representation is null)
        {
            throw new ProgramErrorException("SetEvalRepresentation: null Representation");
        }
        if (theDesc.GetKind() == Geom2dEval_RepCurveDesc.Base.Kind.Mapped)
        {
            Geom2dEval_RepCurveDesc.Mapped aMapped = (Geom2dEval_RepCurveDesc.Mapped)theDesc;
            if (!aMapped.ParamMap.IsValid())
            {
                throw new ProgramErrorException("SetEvalRepresentation: invalid parameter map");
            }
        }
        if (ReferenceEquals(theDesc.Representation, theOwner))
        {
            throw new ProgramErrorException("SetEvalRepresentation: invalid descriptor chain");
        }
    }

    /// <summary>A deep copy of the descriptor (its representation copied too), or null.</summary>
    internal static Geom2dEval_RepCurveDesc.Base? CloneCurveDesc(Geom2dEval_RepCurveDesc.Base? theDesc)
    {
        if (theDesc is null || theDesc.Representation is null)
        {
            return null;
        }

        Geom2d_Curve? aRepCopy = theDesc.Representation.Copy() as Geom2d_Curve;
        if (aRepCopy is null)
        {
            return null;
        }

        switch (theDesc.GetKind())
        {
            case Geom2dEval_RepCurveDesc.Base.Kind.Full:
            {
                Geom2dEval_RepCurveDesc.Full aClone = new Geom2dEval_RepCurveDesc.Full();
                aClone.Representation = aRepCopy;
                return aClone;
            }
            case Geom2dEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                Geom2dEval_RepCurveDesc.DerivBounded aSrc = (Geom2dEval_RepCurveDesc.DerivBounded)theDesc;
                Geom2dEval_RepCurveDesc.DerivBounded aClone = new Geom2dEval_RepCurveDesc.DerivBounded();
                aClone.Representation = aRepCopy;
                aClone.MaxDerivOrder = aSrc.MaxDerivOrder;
                return aClone;
            }
            case Geom2dEval_RepCurveDesc.Base.Kind.Mapped:
            {
                Geom2dEval_RepCurveDesc.Mapped aSrc = (Geom2dEval_RepCurveDesc.Mapped)theDesc;
                Geom2dEval_RepCurveDesc.Mapped aClone = new Geom2dEval_RepCurveDesc.Mapped();
                aClone.Representation = aRepCopy;
                aClone.MaxDerivOrder = aSrc.MaxDerivOrder;
                aClone.Domain = aSrc.Domain;
                aClone.ParamMap = aSrc.ParamMap;
                return aClone;
            }
        }
        return null;
    }

    // Curves

    internal static bool TryEvalCurveD0(Geom2dEval_RepCurveDesc.Base? theDesc, double theU, out Point2D theResult)
    {
        theResult = new Point2D();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case Geom2dEval_RepCurveDesc.Base.Kind.Full:
            case Geom2dEval_RepCurveDesc.Base.Kind.DerivBounded:
                theResult = theDesc.Representation!.EvalD0(theU);
                return true;
            case Geom2dEval_RepCurveDesc.Base.Kind.Mapped:
                return TryEvalCurveD0Mapped((Geom2dEval_RepCurveDesc.Mapped)theDesc, theU, out theResult);
        }
        return false;
    }

    internal static bool TryEvalCurveD1(Geom2dEval_RepCurveDesc.Base? theDesc, double theU, out Geom2d_Curve.ResD1 theResult)
    {
        theResult = new Geom2d_Curve.ResD1();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case Geom2dEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD1(theU);
                return true;
            case Geom2dEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                Geom2dEval_RepCurveDesc.DerivBounded aBounded = (Geom2dEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 1)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD1(theU);
                return true;
            }
            case Geom2dEval_RepCurveDesc.Base.Kind.Mapped:
            {
                Geom2dEval_RepCurveDesc.Mapped aMapped = (Geom2dEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 1)
                {
                    return false;
                }
                return TryEvalCurveD1Mapped(aMapped, theU, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD2(Geom2dEval_RepCurveDesc.Base? theDesc, double theU, out Geom2d_Curve.ResD2 theResult)
    {
        theResult = new Geom2d_Curve.ResD2();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case Geom2dEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD2(theU);
                return true;
            case Geom2dEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                Geom2dEval_RepCurveDesc.DerivBounded aBounded = (Geom2dEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 2)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD2(theU);
                return true;
            }
            case Geom2dEval_RepCurveDesc.Base.Kind.Mapped:
            {
                Geom2dEval_RepCurveDesc.Mapped aMapped = (Geom2dEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 2)
                {
                    return false;
                }
                return TryEvalCurveD2Mapped(aMapped, theU, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD3(Geom2dEval_RepCurveDesc.Base? theDesc, double theU, out Geom2d_Curve.ResD3 theResult)
    {
        theResult = new Geom2d_Curve.ResD3();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case Geom2dEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD3(theU);
                return true;
            case Geom2dEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                Geom2dEval_RepCurveDesc.DerivBounded aBounded = (Geom2dEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 3)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD3(theU);
                return true;
            }
            case Geom2dEval_RepCurveDesc.Base.Kind.Mapped:
            {
                Geom2dEval_RepCurveDesc.Mapped aMapped = (Geom2dEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 3)
                {
                    return false;
                }
                return TryEvalCurveD3Mapped(aMapped, theU, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveDN(Geom2dEval_RepCurveDesc.Base? theDesc, double theU, int theN, out Vector2D theResult)
    {
        theResult = new Vector2D();
        if (theDesc is null || theN < 1)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case Geom2dEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalDN(theU, theN);
                return true;
            case Geom2dEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                Geom2dEval_RepCurveDesc.DerivBounded aBounded = (Geom2dEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < theN)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalDN(theU, theN);
                return true;
            }
            case Geom2dEval_RepCurveDesc.Base.Kind.Mapped:
            {
                Geom2dEval_RepCurveDesc.Mapped aMapped = (Geom2dEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < theN)
                {
                    return false;
                }
                return TryEvalCurveDNMapped(aMapped, theU, theN, out theResult);
            }
        }
        return false;
    }

    // Mapped curves

    internal static bool TryEvalCurveD0Mapped(Geom2dEval_RepCurveDesc.Mapped theDesc, double theU, out Point2D theResult)
    {
        theResult = new Point2D();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        theResult = theDesc.Representation!.EvalD0(aURep);
        return true;
    }

    internal static bool TryEvalCurveD1Mapped(Geom2dEval_RepCurveDesc.Mapped theDesc, double theU, out Geom2d_Curve.ResD1 theResult)
    {
        theResult = new Geom2d_Curve.ResD1();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Geom2d_Curve.ResD1 aRepRes = theDesc.Representation!.EvalD1(aURep);
        double aScale = theDesc.ParamMap.Scale;
        theResult.Point = aRepRes.Point;
        theResult.D1 = aRepRes.D1 * aScale;
        return true;
    }

    internal static bool TryEvalCurveD2Mapped(Geom2dEval_RepCurveDesc.Mapped theDesc, double theU, out Geom2d_Curve.ResD2 theResult)
    {
        theResult = new Geom2d_Curve.ResD2();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Geom2d_Curve.ResD2 aRepRes = theDesc.Representation!.EvalD2(aURep);
        double aScale = theDesc.ParamMap.Scale;
        double aScale2 = aScale * aScale;
        theResult.Point = aRepRes.Point;
        theResult.D1 = aRepRes.D1 * aScale;
        theResult.D2 = aRepRes.D2 * aScale2;
        return true;
    }

    internal static bool TryEvalCurveD3Mapped(Geom2dEval_RepCurveDesc.Mapped theDesc, double theU, out Geom2d_Curve.ResD3 theResult)
    {
        theResult = new Geom2d_Curve.ResD3();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Geom2d_Curve.ResD3 aRepRes = theDesc.Representation!.EvalD3(aURep);
        double aScale = theDesc.ParamMap.Scale;
        double aScale2 = aScale * aScale;
        double aScale3 = aScale2 * aScale;
        theResult.Point = aRepRes.Point;
        theResult.D1 = aRepRes.D1 * aScale;
        theResult.D2 = aRepRes.D2 * aScale2;
        theResult.D3 = aRepRes.D3 * aScale3;
        return true;
    }

    internal static bool TryEvalCurveDNMapped(Geom2dEval_RepCurveDesc.Mapped theDesc, double theU, int theN, out Vector2D theResult)
    {
        theResult = new Vector2D();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Vector2D aRepVec = theDesc.Representation!.EvalDN(aURep, theN);
        double aScaleN = 1.0;
        for (int i = 0; i < theN; ++i)
        {
            aScaleN *= theDesc.ParamMap.Scale;
        }
        theResult = aRepVec * aScaleN;
        return true;
    }

}
