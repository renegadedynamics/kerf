// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_RepUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Dispatch and validation of evaluation representations (<see cref="GeomEval_RepCurveDesc"/>,
/// <see cref="GeomEval_RepSurfaceDesc"/>) for the Geom curves and surfaces that accept one. An internal C++
/// namespace in OCCT. The <c>TryEval</c> functions return false, with their result set to zero, when the
/// descriptor is null or can't evaluate the request; the caller then evaluates the geometry itself.
/// </summary>
internal static class GeomEval_RepUtils
{
    internal static void ValidateCurveDesc(GeomEval_RepCurveDesc.Base? theDesc, Geom_Curve theOwner)
    {
        if (theDesc is null)
        {
            return;
        }
        if (theDesc.Representation is null)
        {
            throw new ProgramErrorException("SetEvalRepresentation: null Representation");
        }
        if (theDesc.GetKind() == GeomEval_RepCurveDesc.Base.Kind.Mapped)
        {
            GeomEval_RepCurveDesc.Mapped aMapped = (GeomEval_RepCurveDesc.Mapped)theDesc;
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

    internal static void ValidateSurfaceDesc(GeomEval_RepSurfaceDesc.Base? theDesc, Geom_Surface theOwner)
    {
        if (theDesc is null)
        {
            return;
        }
        if (theDesc.Representation is null)
        {
            throw new ProgramErrorException("SetEvalRepresentation: null Representation");
        }
        if (theDesc.GetKind() == GeomEval_RepSurfaceDesc.Base.Kind.Mapped)
        {
            GeomEval_RepSurfaceDesc.Mapped aMapped = (GeomEval_RepSurfaceDesc.Mapped)theDesc;
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
    internal static GeomEval_RepCurveDesc.Base? CloneCurveDesc(GeomEval_RepCurveDesc.Base? theDesc)
    {
        if (theDesc is null || theDesc.Representation is null)
        {
            return null;
        }

        Geom_Curve? aRepCopy = theDesc.Representation.Copy() as Geom_Curve;
        if (aRepCopy is null)
        {
            return null;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepCurveDesc.Base.Kind.Full:
            {
                GeomEval_RepCurveDesc.Full aClone = new GeomEval_RepCurveDesc.Full();
                aClone.Representation = aRepCopy;
                return aClone;
            }
            case GeomEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepCurveDesc.DerivBounded aSrc = (GeomEval_RepCurveDesc.DerivBounded)theDesc;
                GeomEval_RepCurveDesc.DerivBounded aClone = new GeomEval_RepCurveDesc.DerivBounded();
                aClone.Representation = aRepCopy;
                aClone.MaxDerivOrder = aSrc.MaxDerivOrder;
                return aClone;
            }
            case GeomEval_RepCurveDesc.Base.Kind.Mapped:
            {
                GeomEval_RepCurveDesc.Mapped aSrc = (GeomEval_RepCurveDesc.Mapped)theDesc;
                GeomEval_RepCurveDesc.Mapped aClone = new GeomEval_RepCurveDesc.Mapped();
                aClone.Representation = aRepCopy;
                aClone.MaxDerivOrder = aSrc.MaxDerivOrder;
                aClone.Domain = aSrc.Domain;
                aClone.ParamMap = aSrc.ParamMap;
                return aClone;
            }
        }
        return null;
    }

    /// <summary>A deep copy of the descriptor (its representation copied too), or null.</summary>
    internal static GeomEval_RepSurfaceDesc.Base? CloneSurfaceDesc(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        if (theDesc is null || theDesc.Representation is null)
        {
            return null;
        }

        Geom_Surface? aRepCopy = theDesc.Representation.Copy() as Geom_Surface;
        if (aRepCopy is null)
        {
            return null;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepSurfaceDesc.Base.Kind.Full:
            {
                GeomEval_RepSurfaceDesc.Full aClone = new GeomEval_RepSurfaceDesc.Full();
                aClone.Representation = aRepCopy;
                return aClone;
            }
            case GeomEval_RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepSurfaceDesc.DerivBounded aSrc = (GeomEval_RepSurfaceDesc.DerivBounded)theDesc;
                GeomEval_RepSurfaceDesc.DerivBounded aClone = new GeomEval_RepSurfaceDesc.DerivBounded();
                aClone.Representation = aRepCopy;
                aClone.MaxDerivOrder = aSrc.MaxDerivOrder;
                return aClone;
            }
            case GeomEval_RepSurfaceDesc.Base.Kind.Mapped:
            {
                GeomEval_RepSurfaceDesc.Mapped aSrc = (GeomEval_RepSurfaceDesc.Mapped)theDesc;
                GeomEval_RepSurfaceDesc.Mapped aClone = new GeomEval_RepSurfaceDesc.Mapped();
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

    internal static bool TryEvalCurveD0(GeomEval_RepCurveDesc.Base? theDesc, double theU, out Point3D theResult)
    {
        theResult = new Point3D();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepCurveDesc.Base.Kind.Full:
            case GeomEval_RepCurveDesc.Base.Kind.DerivBounded:
                theResult = theDesc.Representation!.EvalD0(theU);
                return true;
            case GeomEval_RepCurveDesc.Base.Kind.Mapped:
                return TryEvalCurveD0Mapped((GeomEval_RepCurveDesc.Mapped)theDesc, theU, out theResult);
        }
        return false;
    }

    internal static bool TryEvalCurveD1(GeomEval_RepCurveDesc.Base? theDesc, double theU, out Geom_Curve.ResD1 theResult)
    {
        theResult = new Geom_Curve.ResD1();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD1(theU);
                return true;
            case GeomEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepCurveDesc.DerivBounded aBounded = (GeomEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 1)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD1(theU);
                return true;
            }
            case GeomEval_RepCurveDesc.Base.Kind.Mapped:
            {
                GeomEval_RepCurveDesc.Mapped aMapped = (GeomEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 1)
                {
                    return false;
                }
                return TryEvalCurveD1Mapped(aMapped, theU, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD2(GeomEval_RepCurveDesc.Base? theDesc, double theU, out Geom_Curve.ResD2 theResult)
    {
        theResult = new Geom_Curve.ResD2();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD2(theU);
                return true;
            case GeomEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepCurveDesc.DerivBounded aBounded = (GeomEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 2)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD2(theU);
                return true;
            }
            case GeomEval_RepCurveDesc.Base.Kind.Mapped:
            {
                GeomEval_RepCurveDesc.Mapped aMapped = (GeomEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 2)
                {
                    return false;
                }
                return TryEvalCurveD2Mapped(aMapped, theU, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD3(GeomEval_RepCurveDesc.Base? theDesc, double theU, out Geom_Curve.ResD3 theResult)
    {
        theResult = new Geom_Curve.ResD3();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD3(theU);
                return true;
            case GeomEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepCurveDesc.DerivBounded aBounded = (GeomEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 3)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD3(theU);
                return true;
            }
            case GeomEval_RepCurveDesc.Base.Kind.Mapped:
            {
                GeomEval_RepCurveDesc.Mapped aMapped = (GeomEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 3)
                {
                    return false;
                }
                return TryEvalCurveD3Mapped(aMapped, theU, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveDN(GeomEval_RepCurveDesc.Base? theDesc, double theU, int theN, out Vector3D theResult)
    {
        theResult = new Vector3D();
        if (theDesc is null || theN < 1)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepCurveDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalDN(theU, theN);
                return true;
            case GeomEval_RepCurveDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepCurveDesc.DerivBounded aBounded = (GeomEval_RepCurveDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < theN)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalDN(theU, theN);
                return true;
            }
            case GeomEval_RepCurveDesc.Base.Kind.Mapped:
            {
                GeomEval_RepCurveDesc.Mapped aMapped = (GeomEval_RepCurveDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < theN)
                {
                    return false;
                }
                return TryEvalCurveDNMapped(aMapped, theU, theN, out theResult);
            }
        }
        return false;
    }

    // Surfaces

    internal static bool TryEvalSurfaceD0(GeomEval_RepSurfaceDesc.Base? theDesc, double theU, double theV, out Point3D theResult)
    {
        theResult = new Point3D();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepSurfaceDesc.Base.Kind.Full:
            case GeomEval_RepSurfaceDesc.Base.Kind.DerivBounded:
                theResult = theDesc.Representation!.EvalD0(theU, theV);
                return true;
            case GeomEval_RepSurfaceDesc.Base.Kind.Mapped:
                return TryEvalSurfaceD0Mapped((GeomEval_RepSurfaceDesc.Mapped)theDesc, theU, theV, out theResult);
        }
        return false;
    }

    internal static bool TryEvalSurfaceD1(GeomEval_RepSurfaceDesc.Base? theDesc, double theU, double theV,
                                          out Geom_Surface.ResD1 theResult)
    {
        theResult = new Geom_Surface.ResD1();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepSurfaceDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD1(theU, theV);
                return true;
            case GeomEval_RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepSurfaceDesc.DerivBounded aBounded = (GeomEval_RepSurfaceDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 1)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD1(theU, theV);
                return true;
            }
            case GeomEval_RepSurfaceDesc.Base.Kind.Mapped:
            {
                GeomEval_RepSurfaceDesc.Mapped aMapped = (GeomEval_RepSurfaceDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 1)
                {
                    return false;
                }
                return TryEvalSurfaceD1Mapped(aMapped, theU, theV, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalSurfaceD2(GeomEval_RepSurfaceDesc.Base? theDesc, double theU, double theV,
                                          out Geom_Surface.ResD2 theResult)
    {
        theResult = new Geom_Surface.ResD2();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepSurfaceDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD2(theU, theV);
                return true;
            case GeomEval_RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepSurfaceDesc.DerivBounded aBounded = (GeomEval_RepSurfaceDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 2)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD2(theU, theV);
                return true;
            }
            case GeomEval_RepSurfaceDesc.Base.Kind.Mapped:
            {
                GeomEval_RepSurfaceDesc.Mapped aMapped = (GeomEval_RepSurfaceDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 2)
                {
                    return false;
                }
                return TryEvalSurfaceD2Mapped(aMapped, theU, theV, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalSurfaceD3(GeomEval_RepSurfaceDesc.Base? theDesc, double theU, double theV,
                                          out Geom_Surface.ResD3 theResult)
    {
        theResult = new Geom_Surface.ResD3();
        if (theDesc is null)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepSurfaceDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalD3(theU, theV);
                return true;
            case GeomEval_RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepSurfaceDesc.DerivBounded aBounded = (GeomEval_RepSurfaceDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < 3)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalD3(theU, theV);
                return true;
            }
            case GeomEval_RepSurfaceDesc.Base.Kind.Mapped:
            {
                GeomEval_RepSurfaceDesc.Mapped aMapped = (GeomEval_RepSurfaceDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < 3)
                {
                    return false;
                }
                return TryEvalSurfaceD3Mapped(aMapped, theU, theV, out theResult);
            }
        }
        return false;
    }

    internal static bool TryEvalSurfaceDN(GeomEval_RepSurfaceDesc.Base? theDesc, double theU, double theV,
                                          int theNu, int theNv, out Vector3D theResult)
    {
        theResult = new Vector3D();
        int aTotalOrder = theNu + theNv;
        if (theDesc is null || theNu < 0 || theNv < 0 || aTotalOrder < 1)
        {
            return false;
        }

        switch (theDesc.GetKind())
        {
            case GeomEval_RepSurfaceDesc.Base.Kind.Full:
                theResult = theDesc.Representation!.EvalDN(theU, theV, theNu, theNv);
                return true;
            case GeomEval_RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                GeomEval_RepSurfaceDesc.DerivBounded aBounded = (GeomEval_RepSurfaceDesc.DerivBounded)theDesc;
                if (aBounded.MaxDerivOrder < aTotalOrder)
                {
                    return false;
                }
                theResult = theDesc.Representation!.EvalDN(theU, theV, theNu, theNv);
                return true;
            }
            case GeomEval_RepSurfaceDesc.Base.Kind.Mapped:
            {
                GeomEval_RepSurfaceDesc.Mapped aMapped = (GeomEval_RepSurfaceDesc.Mapped)theDesc;
                if (aMapped.MaxDerivOrder < aTotalOrder)
                {
                    return false;
                }
                return TryEvalSurfaceDNMapped(aMapped, theU, theV, theNu, theNv, out theResult);
            }
        }
        return false;
    }

    // Mapped curves

    internal static bool TryEvalCurveD0Mapped(GeomEval_RepCurveDesc.Mapped theDesc, double theU, out Point3D theResult)
    {
        theResult = new Point3D();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        theResult = theDesc.Representation!.EvalD0(aURep);
        return true;
    }

    internal static bool TryEvalCurveD1Mapped(GeomEval_RepCurveDesc.Mapped theDesc, double theU, out Geom_Curve.ResD1 theResult)
    {
        theResult = new Geom_Curve.ResD1();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Geom_Curve.ResD1 aRepRes = theDesc.Representation!.EvalD1(aURep);
        double aScale = theDesc.ParamMap.Scale;
        theResult.Point = aRepRes.Point;
        theResult.D1 = aRepRes.D1 * aScale;
        return true;
    }

    internal static bool TryEvalCurveD2Mapped(GeomEval_RepCurveDesc.Mapped theDesc, double theU, out Geom_Curve.ResD2 theResult)
    {
        theResult = new Geom_Curve.ResD2();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Geom_Curve.ResD2 aRepRes = theDesc.Representation!.EvalD2(aURep);
        double aScale = theDesc.ParamMap.Scale;
        double aScale2 = aScale * aScale;
        theResult.Point = aRepRes.Point;
        theResult.D1 = aRepRes.D1 * aScale;
        theResult.D2 = aRepRes.D2 * aScale2;
        return true;
    }

    internal static bool TryEvalCurveD3Mapped(GeomEval_RepCurveDesc.Mapped theDesc, double theU, out Geom_Curve.ResD3 theResult)
    {
        theResult = new Geom_Curve.ResD3();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Geom_Curve.ResD3 aRepRes = theDesc.Representation!.EvalD3(aURep);
        double aScale = theDesc.ParamMap.Scale;
        double aScale2 = aScale * aScale;
        double aScale3 = aScale2 * aScale;
        theResult.Point = aRepRes.Point;
        theResult.D1 = aRepRes.D1 * aScale;
        theResult.D2 = aRepRes.D2 * aScale2;
        theResult.D3 = aRepRes.D3 * aScale3;
        return true;
    }

    internal static bool TryEvalCurveDNMapped(GeomEval_RepCurveDesc.Mapped theDesc, double theU, int theN, out Vector3D theResult)
    {
        theResult = new Vector3D();
        double aURep = theDesc.ParamMap.Map(theU);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep))
        {
            return false;
        }
        Vector3D aRepVec = theDesc.Representation!.EvalDN(aURep, theN);
        double aScaleN = Math.Pow(theDesc.ParamMap.Scale, theN);
        theResult = aRepVec * aScaleN;
        return true;
    }

    // Mapped surfaces

    internal static bool TryEvalSurfaceD0Mapped(GeomEval_RepSurfaceDesc.Mapped theDesc, double theU, double theV, out Point3D theResult)
    {
        theResult = new Point3D();
        double aURep = 0.0;
        double aVRep = 0.0;
        theDesc.ParamMap.Map(theU, theV, out aURep, out aVRep);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep, aVRep))
        {
            return false;
        }
        theResult = theDesc.Representation!.EvalD0(aURep, aVRep);
        return true;
    }

    internal static bool TryEvalSurfaceD1Mapped(GeomEval_RepSurfaceDesc.Mapped theDesc, double theU, double theV,
                                                out Geom_Surface.ResD1 theResult)
    {
        theResult = new Geom_Surface.ResD1();
        double aURep = 0.0;
        double aVRep = 0.0;
        theDesc.ParamMap.Map(theU, theV, out aURep, out aVRep);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep, aVRep))
        {
            return false;
        }
        Geom_Surface.ResD1 aRepRes = theDesc.Representation!.EvalD1(aURep, aVRep);
        theResult.Point = aRepRes.Point;
        if (theDesc.ParamMap.SwapUV)
        {
            theResult.D1U = aRepRes.D1V * theDesc.ParamMap.ScaleV;
            theResult.D1V = aRepRes.D1U * theDesc.ParamMap.ScaleU;
        }
        else
        {
            theResult.D1U = aRepRes.D1U * theDesc.ParamMap.ScaleU;
            theResult.D1V = aRepRes.D1V * theDesc.ParamMap.ScaleV;
        }
        return true;
    }

    internal static bool TryEvalSurfaceD2Mapped(GeomEval_RepSurfaceDesc.Mapped theDesc, double theU, double theV,
                                                out Geom_Surface.ResD2 theResult)
    {
        theResult = new Geom_Surface.ResD2();
        double aURep = 0.0;
        double aVRep = 0.0;
        theDesc.ParamMap.Map(theU, theV, out aURep, out aVRep);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep, aVRep))
        {
            return false;
        }
        Geom_Surface.ResD2 aRepRes = theDesc.Representation!.EvalD2(aURep, aVRep);
        double aSu = theDesc.ParamMap.ScaleU;
        double aSv = theDesc.ParamMap.ScaleV;
        double aSu2 = aSu * aSu;
        double aSv2 = aSv * aSv;
        double aSuSv = aSu * aSv;
        theResult.Point = aRepRes.Point;
        if (theDesc.ParamMap.SwapUV)
        {
            theResult.D1U = aRepRes.D1V * aSv;
            theResult.D1V = aRepRes.D1U * aSu;
            theResult.D2U = aRepRes.D2V * aSv2;
            theResult.D2V = aRepRes.D2U * aSu2;
            theResult.D2UV = aRepRes.D2UV * aSuSv;
        }
        else
        {
            theResult.D1U = aRepRes.D1U * aSu;
            theResult.D1V = aRepRes.D1V * aSv;
            theResult.D2U = aRepRes.D2U * aSu2;
            theResult.D2V = aRepRes.D2V * aSv2;
            theResult.D2UV = aRepRes.D2UV * aSuSv;
        }
        return true;
    }

    internal static bool TryEvalSurfaceD3Mapped(GeomEval_RepSurfaceDesc.Mapped theDesc, double theU, double theV,
                                                out Geom_Surface.ResD3 theResult)
    {
        theResult = new Geom_Surface.ResD3();
        double aURep = 0.0;
        double aVRep = 0.0;
        theDesc.ParamMap.Map(theU, theV, out aURep, out aVRep);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep, aVRep))
        {
            return false;
        }
        Geom_Surface.ResD3 aRepRes = theDesc.Representation!.EvalD3(aURep, aVRep);
        double aSu = theDesc.ParamMap.ScaleU;
        double aSv = theDesc.ParamMap.ScaleV;
        theResult.Point = aRepRes.Point;
        if (theDesc.ParamMap.SwapUV)
        {
            theResult.D1U = aRepRes.D1V * aSv;
            theResult.D1V = aRepRes.D1U * aSu;
            theResult.D2U = aRepRes.D2V * (aSv * aSv);
            theResult.D2V = aRepRes.D2U * (aSu * aSu);
            theResult.D2UV = aRepRes.D2UV * (aSu * aSv);
            theResult.D3U = aRepRes.D3V * (aSv * aSv * aSv);
            theResult.D3V = aRepRes.D3U * (aSu * aSu * aSu);
            theResult.D3UUV = aRepRes.D3UVV * (aSv * aSv * aSu);
            theResult.D3UVV = aRepRes.D3UUV * (aSu * aSu * aSv);
        }
        else
        {
            theResult.D1U = aRepRes.D1U * aSu;
            theResult.D1V = aRepRes.D1V * aSv;
            theResult.D2U = aRepRes.D2U * (aSu * aSu);
            theResult.D2V = aRepRes.D2V * (aSv * aSv);
            theResult.D2UV = aRepRes.D2UV * (aSu * aSv);
            theResult.D3U = aRepRes.D3U * (aSu * aSu * aSu);
            theResult.D3V = aRepRes.D3V * (aSv * aSv * aSv);
            theResult.D3UUV = aRepRes.D3UUV * (aSu * aSu * aSv);
            theResult.D3UVV = aRepRes.D3UVV * (aSu * aSv * aSv);
        }
        return true;
    }

    internal static bool TryEvalSurfaceDNMapped(GeomEval_RepSurfaceDesc.Mapped theDesc, double theU, double theV,
                                                int theNu, int theNv, out Vector3D theResult)
    {
        theResult = new Vector3D();
        double aURep = 0.0;
        double aVRep = 0.0;
        theDesc.ParamMap.Map(theU, theV, out aURep, out aVRep);
        if (theDesc.Domain.HasValue && !theDesc.Domain.Value.Contains(aURep, aVRep))
        {
            return false;
        }
        int aNuRep = theNu;
        int aNvRep = theNv;
        if (theDesc.ParamMap.SwapUV)
        {
            aNuRep = theNv;
            aNvRep = theNu;
        }
        Vector3D aRepVec = theDesc.Representation!.EvalDN(aURep, aVRep, aNuRep, aNvRep);
        double aScale = Math.Pow(theDesc.ParamMap.ScaleU, aNuRep) * Math.Pow(theDesc.ParamMap.ScaleV, aNvRep);
        theResult = aRepVec * aScale;
        return true;
    }
}
