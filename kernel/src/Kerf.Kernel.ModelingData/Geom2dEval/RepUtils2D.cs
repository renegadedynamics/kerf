// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_RepUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// Dispatch and validation of evaluation representations (<see cref="RepCurveDesc2D"/>) for the
/// Geom2d curves that accept one. An internal C++ namespace in OCCT. The <c>TryEval</c> functions return false,
/// with their result set to zero, when the descriptor is null or can't evaluate the request; the caller then
/// evaluates the geometry itself.
/// </summary>
internal static class RepUtils2D
{
    internal static void ValidateCurveDesc(RepCurveDesc2D.Base? desc, Curve2D owner)
    {
        if (desc is null)
        {
            return;
        }
        if (desc.Representation is null)
        {
            throw new ProgramErrorException("SetEvalRepresentation: null Representation");
        }
        if (desc.GetKind() == RepCurveDesc2D.Base.Kind.Mapped)
        {
            RepCurveDesc2D.Mapped mapped = (RepCurveDesc2D.Mapped)desc;
            if (!mapped.ParamMap.IsValid)
            {
                throw new ProgramErrorException("SetEvalRepresentation: invalid parameter map");
            }
        }
        if (ReferenceEquals(desc.Representation, owner))
        {
            throw new ProgramErrorException("SetEvalRepresentation: invalid descriptor chain");
        }
    }

    /// <summary>A deep copy of the descriptor (its representation copied too), or null.</summary>
    internal static RepCurveDesc2D.Base? CloneCurveDesc(RepCurveDesc2D.Base? desc)
    {
        if (desc is null || desc.Representation is null)
        {
            return null;
        }

        Curve2D? repCopy = desc.Representation.Copy() as Curve2D;
        if (repCopy is null)
        {
            return null;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc2D.Base.Kind.Full:
            {
                RepCurveDesc2D.Full clone = new RepCurveDesc2D.Full();
                clone.Representation = repCopy;
                return clone;
            }
            case RepCurveDesc2D.Base.Kind.DerivBounded:
            {
                RepCurveDesc2D.DerivBounded src = (RepCurveDesc2D.DerivBounded)desc;
                RepCurveDesc2D.DerivBounded clone = new RepCurveDesc2D.DerivBounded();
                clone.Representation = repCopy;
                clone.MaxDerivOrder = src.MaxDerivOrder;
                return clone;
            }
            case RepCurveDesc2D.Base.Kind.Mapped:
            {
                RepCurveDesc2D.Mapped src = (RepCurveDesc2D.Mapped)desc;
                RepCurveDesc2D.Mapped clone = new RepCurveDesc2D.Mapped();
                clone.Representation = repCopy;
                clone.MaxDerivOrder = src.MaxDerivOrder;
                clone.Domain = src.Domain;
                clone.ParamMap = src.ParamMap;
                return clone;
            }
        }
        return null;
    }

    // Curves

    internal static bool TryEvalCurveD0(RepCurveDesc2D.Base? desc, double u, out Point2D result)
    {
        result = new Point2D();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc2D.Base.Kind.Full:
            case RepCurveDesc2D.Base.Kind.DerivBounded:
                result = desc.Representation!.EvalD0(u);
                return true;
            case RepCurveDesc2D.Base.Kind.Mapped:
                return TryEvalCurveD0Mapped((RepCurveDesc2D.Mapped)desc, u, out result);
        }
        return false;
    }

    internal static bool TryEvalCurveD1(RepCurveDesc2D.Base? desc, double u, out Curve2D.ResD1 result)
    {
        result = new Curve2D.ResD1();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc2D.Base.Kind.Full:
                result = desc.Representation!.EvalD1(u);
                return true;
            case RepCurveDesc2D.Base.Kind.DerivBounded:
            {
                RepCurveDesc2D.DerivBounded bounded = (RepCurveDesc2D.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 1)
                {
                    return false;
                }
                result = desc.Representation!.EvalD1(u);
                return true;
            }
            case RepCurveDesc2D.Base.Kind.Mapped:
            {
                RepCurveDesc2D.Mapped mapped = (RepCurveDesc2D.Mapped)desc;
                if (mapped.MaxDerivOrder < 1)
                {
                    return false;
                }
                return TryEvalCurveD1Mapped(mapped, u, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD2(RepCurveDesc2D.Base? desc, double u, out Curve2D.ResD2 result)
    {
        result = new Curve2D.ResD2();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc2D.Base.Kind.Full:
                result = desc.Representation!.EvalD2(u);
                return true;
            case RepCurveDesc2D.Base.Kind.DerivBounded:
            {
                RepCurveDesc2D.DerivBounded bounded = (RepCurveDesc2D.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 2)
                {
                    return false;
                }
                result = desc.Representation!.EvalD2(u);
                return true;
            }
            case RepCurveDesc2D.Base.Kind.Mapped:
            {
                RepCurveDesc2D.Mapped mapped = (RepCurveDesc2D.Mapped)desc;
                if (mapped.MaxDerivOrder < 2)
                {
                    return false;
                }
                return TryEvalCurveD2Mapped(mapped, u, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD3(RepCurveDesc2D.Base? desc, double u, out Curve2D.ResD3 result)
    {
        result = new Curve2D.ResD3();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc2D.Base.Kind.Full:
                result = desc.Representation!.EvalD3(u);
                return true;
            case RepCurveDesc2D.Base.Kind.DerivBounded:
            {
                RepCurveDesc2D.DerivBounded bounded = (RepCurveDesc2D.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 3)
                {
                    return false;
                }
                result = desc.Representation!.EvalD3(u);
                return true;
            }
            case RepCurveDesc2D.Base.Kind.Mapped:
            {
                RepCurveDesc2D.Mapped mapped = (RepCurveDesc2D.Mapped)desc;
                if (mapped.MaxDerivOrder < 3)
                {
                    return false;
                }
                return TryEvalCurveD3Mapped(mapped, u, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveDN(RepCurveDesc2D.Base? desc, double u, int n, out Vector2D result)
    {
        result = new Vector2D();
        if (desc is null || n < 1)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc2D.Base.Kind.Full:
                result = desc.Representation!.EvalDN(u, n);
                return true;
            case RepCurveDesc2D.Base.Kind.DerivBounded:
            {
                RepCurveDesc2D.DerivBounded bounded = (RepCurveDesc2D.DerivBounded)desc;
                if (bounded.MaxDerivOrder < n)
                {
                    return false;
                }
                result = desc.Representation!.EvalDN(u, n);
                return true;
            }
            case RepCurveDesc2D.Base.Kind.Mapped:
            {
                RepCurveDesc2D.Mapped mapped = (RepCurveDesc2D.Mapped)desc;
                if (mapped.MaxDerivOrder < n)
                {
                    return false;
                }
                return TryEvalCurveDNMapped(mapped, u, n, out result);
            }
        }
        return false;
    }

    // Mapped curves

    internal static bool TryEvalCurveD0Mapped(RepCurveDesc2D.Mapped desc, double u, out Point2D result)
    {
        result = new Point2D();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        result = desc.Representation!.EvalD0(uRep);
        return true;
    }

    internal static bool TryEvalCurveD1Mapped(RepCurveDesc2D.Mapped desc, double u, out Curve2D.ResD1 result)
    {
        result = new Curve2D.ResD1();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Curve2D.ResD1 repRes = desc.Representation!.EvalD1(uRep);
        double scale = desc.ParamMap.Scale;
        result.Point = repRes.Point;
        result.D1 = repRes.D1 * scale;
        return true;
    }

    internal static bool TryEvalCurveD2Mapped(RepCurveDesc2D.Mapped desc, double u, out Curve2D.ResD2 result)
    {
        result = new Curve2D.ResD2();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Curve2D.ResD2 repRes = desc.Representation!.EvalD2(uRep);
        double scale = desc.ParamMap.Scale;
        double scale2 = scale * scale;
        result.Point = repRes.Point;
        result.D1 = repRes.D1 * scale;
        result.D2 = repRes.D2 * scale2;
        return true;
    }

    internal static bool TryEvalCurveD3Mapped(RepCurveDesc2D.Mapped desc, double u, out Curve2D.ResD3 result)
    {
        result = new Curve2D.ResD3();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Curve2D.ResD3 repRes = desc.Representation!.EvalD3(uRep);
        double scale = desc.ParamMap.Scale;
        double scale2 = scale * scale;
        double scale3 = scale2 * scale;
        result.Point = repRes.Point;
        result.D1 = repRes.D1 * scale;
        result.D2 = repRes.D2 * scale2;
        result.D3 = repRes.D3 * scale3;
        return true;
    }

    internal static bool TryEvalCurveDNMapped(RepCurveDesc2D.Mapped desc, double u, int n, out Vector2D result)
    {
        result = new Vector2D();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Vector2D repVec = desc.Representation!.EvalDN(uRep, n);
        double scaleN = 1.0;
        for (int i = 0; i < n; ++i)
        {
            scaleN *= desc.ParamMap.Scale;
        }
        result = repVec * scaleN;
        return true;
    }

}
