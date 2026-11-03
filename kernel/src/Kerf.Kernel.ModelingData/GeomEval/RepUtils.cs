// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_RepUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// Dispatch and validation of evaluation representations (<see cref="RepCurveDesc"/>,
/// <see cref="RepSurfaceDesc"/>) for the Geom curves and surfaces that accept one. An internal C++
/// namespace in OCCT. The <c>TryEval</c> functions return false, with their result set to zero, when the
/// descriptor is null or can't evaluate the request; the caller then evaluates the geometry itself.
/// </summary>
internal static class RepUtils
{
    internal static void ValidateCurveDesc(RepCurveDesc.Base? desc, Curve3D owner)
    {
        if (desc is null)
        {
            return;
        }
        if (desc.Representation is null)
        {
            throw new ProgramErrorException("SetEvalRepresentation: null Representation");
        }
        if (desc.GetKind() == RepCurveDesc.Base.Kind.Mapped)
        {
            RepCurveDesc.Mapped mapped = (RepCurveDesc.Mapped)desc;
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

    internal static void ValidateSurfaceDesc(RepSurfaceDesc.Base? desc, Surface owner)
    {
        if (desc is null)
        {
            return;
        }
        if (desc.Representation is null)
        {
            throw new ProgramErrorException("SetEvalRepresentation: null Representation");
        }
        if (desc.GetKind() == RepSurfaceDesc.Base.Kind.Mapped)
        {
            RepSurfaceDesc.Mapped mapped = (RepSurfaceDesc.Mapped)desc;
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
    internal static RepCurveDesc.Base? CloneCurveDesc(RepCurveDesc.Base? desc)
    {
        if (desc is null || desc.Representation is null)
        {
            return null;
        }

        Curve3D? repCopy = desc.Representation.Copy() as Curve3D;
        if (repCopy is null)
        {
            return null;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc.Base.Kind.Full:
            {
                RepCurveDesc.Full clone = new RepCurveDesc.Full();
                clone.Representation = repCopy;
                return clone;
            }
            case RepCurveDesc.Base.Kind.DerivBounded:
            {
                RepCurveDesc.DerivBounded src = (RepCurveDesc.DerivBounded)desc;
                RepCurveDesc.DerivBounded clone = new RepCurveDesc.DerivBounded();
                clone.Representation = repCopy;
                clone.MaxDerivOrder = src.MaxDerivOrder;
                return clone;
            }
            case RepCurveDesc.Base.Kind.Mapped:
            {
                RepCurveDesc.Mapped src = (RepCurveDesc.Mapped)desc;
                RepCurveDesc.Mapped clone = new RepCurveDesc.Mapped();
                clone.Representation = repCopy;
                clone.MaxDerivOrder = src.MaxDerivOrder;
                clone.Domain = src.Domain;
                clone.ParamMap = src.ParamMap;
                return clone;
            }
        }
        return null;
    }

    /// <summary>A deep copy of the descriptor (its representation copied too), or null.</summary>
    internal static RepSurfaceDesc.Base? CloneSurfaceDesc(RepSurfaceDesc.Base? desc)
    {
        if (desc is null || desc.Representation is null)
        {
            return null;
        }

        Surface? repCopy = desc.Representation.Copy() as Surface;
        if (repCopy is null)
        {
            return null;
        }

        switch (desc.GetKind())
        {
            case RepSurfaceDesc.Base.Kind.Full:
            {
                RepSurfaceDesc.Full clone = new RepSurfaceDesc.Full();
                clone.Representation = repCopy;
                return clone;
            }
            case RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                RepSurfaceDesc.DerivBounded src = (RepSurfaceDesc.DerivBounded)desc;
                RepSurfaceDesc.DerivBounded clone = new RepSurfaceDesc.DerivBounded();
                clone.Representation = repCopy;
                clone.MaxDerivOrder = src.MaxDerivOrder;
                return clone;
            }
            case RepSurfaceDesc.Base.Kind.Mapped:
            {
                RepSurfaceDesc.Mapped src = (RepSurfaceDesc.Mapped)desc;
                RepSurfaceDesc.Mapped clone = new RepSurfaceDesc.Mapped();
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

    internal static bool TryEvalCurveD0(RepCurveDesc.Base? desc, double u, out Point3D result)
    {
        result = new Point3D();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc.Base.Kind.Full:
            case RepCurveDesc.Base.Kind.DerivBounded:
                result = desc.Representation!.EvalD0(u);
                return true;
            case RepCurveDesc.Base.Kind.Mapped:
                return TryEvalCurveD0Mapped((RepCurveDesc.Mapped)desc, u, out result);
        }
        return false;
    }

    internal static bool TryEvalCurveD1(RepCurveDesc.Base? desc, double u, out Curve3D.ResD1 result)
    {
        result = new Curve3D.ResD1();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc.Base.Kind.Full:
                result = desc.Representation!.EvalD1(u);
                return true;
            case RepCurveDesc.Base.Kind.DerivBounded:
            {
                RepCurveDesc.DerivBounded bounded = (RepCurveDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 1)
                {
                    return false;
                }
                result = desc.Representation!.EvalD1(u);
                return true;
            }
            case RepCurveDesc.Base.Kind.Mapped:
            {
                RepCurveDesc.Mapped mapped = (RepCurveDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < 1)
                {
                    return false;
                }
                return TryEvalCurveD1Mapped(mapped, u, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD2(RepCurveDesc.Base? desc, double u, out Curve3D.ResD2 result)
    {
        result = new Curve3D.ResD2();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc.Base.Kind.Full:
                result = desc.Representation!.EvalD2(u);
                return true;
            case RepCurveDesc.Base.Kind.DerivBounded:
            {
                RepCurveDesc.DerivBounded bounded = (RepCurveDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 2)
                {
                    return false;
                }
                result = desc.Representation!.EvalD2(u);
                return true;
            }
            case RepCurveDesc.Base.Kind.Mapped:
            {
                RepCurveDesc.Mapped mapped = (RepCurveDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < 2)
                {
                    return false;
                }
                return TryEvalCurveD2Mapped(mapped, u, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveD3(RepCurveDesc.Base? desc, double u, out Curve3D.ResD3 result)
    {
        result = new Curve3D.ResD3();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc.Base.Kind.Full:
                result = desc.Representation!.EvalD3(u);
                return true;
            case RepCurveDesc.Base.Kind.DerivBounded:
            {
                RepCurveDesc.DerivBounded bounded = (RepCurveDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 3)
                {
                    return false;
                }
                result = desc.Representation!.EvalD3(u);
                return true;
            }
            case RepCurveDesc.Base.Kind.Mapped:
            {
                RepCurveDesc.Mapped mapped = (RepCurveDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < 3)
                {
                    return false;
                }
                return TryEvalCurveD3Mapped(mapped, u, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalCurveDN(RepCurveDesc.Base? desc, double u, int n, out Vector3D result)
    {
        result = new Vector3D();
        if (desc is null || n < 1)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepCurveDesc.Base.Kind.Full:
                result = desc.Representation!.EvalDN(u, n);
                return true;
            case RepCurveDesc.Base.Kind.DerivBounded:
            {
                RepCurveDesc.DerivBounded bounded = (RepCurveDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < n)
                {
                    return false;
                }
                result = desc.Representation!.EvalDN(u, n);
                return true;
            }
            case RepCurveDesc.Base.Kind.Mapped:
            {
                RepCurveDesc.Mapped mapped = (RepCurveDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < n)
                {
                    return false;
                }
                return TryEvalCurveDNMapped(mapped, u, n, out result);
            }
        }
        return false;
    }

    // Surfaces

    internal static bool TryEvalSurfaceD0(RepSurfaceDesc.Base? desc, double u, double v, out Point3D result)
    {
        result = new Point3D();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepSurfaceDesc.Base.Kind.Full:
            case RepSurfaceDesc.Base.Kind.DerivBounded:
                result = desc.Representation!.EvalD0(u, v);
                return true;
            case RepSurfaceDesc.Base.Kind.Mapped:
                return TryEvalSurfaceD0Mapped((RepSurfaceDesc.Mapped)desc, u, v, out result);
        }
        return false;
    }

    internal static bool TryEvalSurfaceD1(RepSurfaceDesc.Base? desc, double u, double v,
                                          out Surface.ResD1 result)
    {
        result = new Surface.ResD1();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepSurfaceDesc.Base.Kind.Full:
                result = desc.Representation!.EvalD1(u, v);
                return true;
            case RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                RepSurfaceDesc.DerivBounded bounded = (RepSurfaceDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 1)
                {
                    return false;
                }
                result = desc.Representation!.EvalD1(u, v);
                return true;
            }
            case RepSurfaceDesc.Base.Kind.Mapped:
            {
                RepSurfaceDesc.Mapped mapped = (RepSurfaceDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < 1)
                {
                    return false;
                }
                return TryEvalSurfaceD1Mapped(mapped, u, v, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalSurfaceD2(RepSurfaceDesc.Base? desc, double u, double v,
                                          out Surface.ResD2 result)
    {
        result = new Surface.ResD2();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepSurfaceDesc.Base.Kind.Full:
                result = desc.Representation!.EvalD2(u, v);
                return true;
            case RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                RepSurfaceDesc.DerivBounded bounded = (RepSurfaceDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 2)
                {
                    return false;
                }
                result = desc.Representation!.EvalD2(u, v);
                return true;
            }
            case RepSurfaceDesc.Base.Kind.Mapped:
            {
                RepSurfaceDesc.Mapped mapped = (RepSurfaceDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < 2)
                {
                    return false;
                }
                return TryEvalSurfaceD2Mapped(mapped, u, v, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalSurfaceD3(RepSurfaceDesc.Base? desc, double u, double v,
                                          out Surface.ResD3 result)
    {
        result = new Surface.ResD3();
        if (desc is null)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepSurfaceDesc.Base.Kind.Full:
                result = desc.Representation!.EvalD3(u, v);
                return true;
            case RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                RepSurfaceDesc.DerivBounded bounded = (RepSurfaceDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < 3)
                {
                    return false;
                }
                result = desc.Representation!.EvalD3(u, v);
                return true;
            }
            case RepSurfaceDesc.Base.Kind.Mapped:
            {
                RepSurfaceDesc.Mapped mapped = (RepSurfaceDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < 3)
                {
                    return false;
                }
                return TryEvalSurfaceD3Mapped(mapped, u, v, out result);
            }
        }
        return false;
    }

    internal static bool TryEvalSurfaceDN(RepSurfaceDesc.Base? desc, double u, double v,
                                          int nu, int nv, out Vector3D result)
    {
        result = new Vector3D();
        int totalOrder = nu + nv;
        if (desc is null || nu < 0 || nv < 0 || totalOrder < 1)
        {
            return false;
        }

        switch (desc.GetKind())
        {
            case RepSurfaceDesc.Base.Kind.Full:
                result = desc.Representation!.EvalDN(u, v, nu, nv);
                return true;
            case RepSurfaceDesc.Base.Kind.DerivBounded:
            {
                RepSurfaceDesc.DerivBounded bounded = (RepSurfaceDesc.DerivBounded)desc;
                if (bounded.MaxDerivOrder < totalOrder)
                {
                    return false;
                }
                result = desc.Representation!.EvalDN(u, v, nu, nv);
                return true;
            }
            case RepSurfaceDesc.Base.Kind.Mapped:
            {
                RepSurfaceDesc.Mapped mapped = (RepSurfaceDesc.Mapped)desc;
                if (mapped.MaxDerivOrder < totalOrder)
                {
                    return false;
                }
                return TryEvalSurfaceDNMapped(mapped, u, v, nu, nv, out result);
            }
        }
        return false;
    }

    // Mapped curves

    internal static bool TryEvalCurveD0Mapped(RepCurveDesc.Mapped desc, double u, out Point3D result)
    {
        result = new Point3D();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        result = desc.Representation!.EvalD0(uRep);
        return true;
    }

    internal static bool TryEvalCurveD1Mapped(RepCurveDesc.Mapped desc, double u, out Curve3D.ResD1 result)
    {
        result = new Curve3D.ResD1();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Curve3D.ResD1 repRes = desc.Representation!.EvalD1(uRep);
        double scale = desc.ParamMap.Scale;
        result.Point = repRes.Point;
        result.D1 = repRes.D1 * scale;
        return true;
    }

    internal static bool TryEvalCurveD2Mapped(RepCurveDesc.Mapped desc, double u, out Curve3D.ResD2 result)
    {
        result = new Curve3D.ResD2();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Curve3D.ResD2 repRes = desc.Representation!.EvalD2(uRep);
        double scale = desc.ParamMap.Scale;
        double scale2 = scale * scale;
        result.Point = repRes.Point;
        result.D1 = repRes.D1 * scale;
        result.D2 = repRes.D2 * scale2;
        return true;
    }

    internal static bool TryEvalCurveD3Mapped(RepCurveDesc.Mapped desc, double u, out Curve3D.ResD3 result)
    {
        result = new Curve3D.ResD3();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Curve3D.ResD3 repRes = desc.Representation!.EvalD3(uRep);
        double scale = desc.ParamMap.Scale;
        double scale2 = scale * scale;
        double scale3 = scale2 * scale;
        result.Point = repRes.Point;
        result.D1 = repRes.D1 * scale;
        result.D2 = repRes.D2 * scale2;
        result.D3 = repRes.D3 * scale3;
        return true;
    }

    internal static bool TryEvalCurveDNMapped(RepCurveDesc.Mapped desc, double u, int n, out Vector3D result)
    {
        result = new Vector3D();
        double uRep = desc.ParamMap.Map(u);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep))
        {
            return false;
        }
        Vector3D repVec = desc.Representation!.EvalDN(uRep, n);
        double scaleN = Math.Pow(desc.ParamMap.Scale, n);
        result = repVec * scaleN;
        return true;
    }

    // Mapped surfaces

    internal static bool TryEvalSurfaceD0Mapped(RepSurfaceDesc.Mapped desc, double u, double v, out Point3D result)
    {
        result = new Point3D();
        double uRep = 0.0;
        double vRep = 0.0;
        desc.ParamMap.Map(u, v, out uRep, out vRep);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep, vRep))
        {
            return false;
        }
        result = desc.Representation!.EvalD0(uRep, vRep);
        return true;
    }

    internal static bool TryEvalSurfaceD1Mapped(RepSurfaceDesc.Mapped desc, double u, double v,
                                                out Surface.ResD1 result)
    {
        result = new Surface.ResD1();
        double uRep = 0.0;
        double vRep = 0.0;
        desc.ParamMap.Map(u, v, out uRep, out vRep);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep, vRep))
        {
            return false;
        }
        Surface.ResD1 repRes = desc.Representation!.EvalD1(uRep, vRep);
        result.Point = repRes.Point;
        if (desc.ParamMap.SwapUV)
        {
            result.D1U = repRes.D1V * desc.ParamMap.ScaleV;
            result.D1V = repRes.D1U * desc.ParamMap.ScaleU;
        }
        else
        {
            result.D1U = repRes.D1U * desc.ParamMap.ScaleU;
            result.D1V = repRes.D1V * desc.ParamMap.ScaleV;
        }
        return true;
    }

    internal static bool TryEvalSurfaceD2Mapped(RepSurfaceDesc.Mapped desc, double u, double v,
                                                out Surface.ResD2 result)
    {
        result = new Surface.ResD2();
        double uRep = 0.0;
        double vRep = 0.0;
        desc.ParamMap.Map(u, v, out uRep, out vRep);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep, vRep))
        {
            return false;
        }
        Surface.ResD2 repRes = desc.Representation!.EvalD2(uRep, vRep);
        double su = desc.ParamMap.ScaleU;
        double sv = desc.ParamMap.ScaleV;
        double su2 = su * su;
        double sv2 = sv * sv;
        double suSv = su * sv;
        result.Point = repRes.Point;
        if (desc.ParamMap.SwapUV)
        {
            result.D1U = repRes.D1V * sv;
            result.D1V = repRes.D1U * su;
            result.D2U = repRes.D2V * sv2;
            result.D2V = repRes.D2U * su2;
            result.D2UV = repRes.D2UV * suSv;
        }
        else
        {
            result.D1U = repRes.D1U * su;
            result.D1V = repRes.D1V * sv;
            result.D2U = repRes.D2U * su2;
            result.D2V = repRes.D2V * sv2;
            result.D2UV = repRes.D2UV * suSv;
        }
        return true;
    }

    internal static bool TryEvalSurfaceD3Mapped(RepSurfaceDesc.Mapped desc, double u, double v,
                                                out Surface.ResD3 result)
    {
        result = new Surface.ResD3();
        double uRep = 0.0;
        double vRep = 0.0;
        desc.ParamMap.Map(u, v, out uRep, out vRep);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep, vRep))
        {
            return false;
        }
        Surface.ResD3 repRes = desc.Representation!.EvalD3(uRep, vRep);
        double su = desc.ParamMap.ScaleU;
        double sv = desc.ParamMap.ScaleV;
        result.Point = repRes.Point;
        if (desc.ParamMap.SwapUV)
        {
            result.D1U = repRes.D1V * sv;
            result.D1V = repRes.D1U * su;
            result.D2U = repRes.D2V * (sv * sv);
            result.D2V = repRes.D2U * (su * su);
            result.D2UV = repRes.D2UV * (su * sv);
            result.D3U = repRes.D3V * (sv * sv * sv);
            result.D3V = repRes.D3U * (su * su * su);
            result.D3UUV = repRes.D3UVV * (sv * sv * su);
            result.D3UVV = repRes.D3UUV * (su * su * sv);
        }
        else
        {
            result.D1U = repRes.D1U * su;
            result.D1V = repRes.D1V * sv;
            result.D2U = repRes.D2U * (su * su);
            result.D2V = repRes.D2V * (sv * sv);
            result.D2UV = repRes.D2UV * (su * sv);
            result.D3U = repRes.D3U * (su * su * su);
            result.D3V = repRes.D3V * (sv * sv * sv);
            result.D3UUV = repRes.D3UUV * (su * su * sv);
            result.D3UVV = repRes.D3UVV * (su * sv * sv);
        }
        return true;
    }

    internal static bool TryEvalSurfaceDNMapped(RepSurfaceDesc.Mapped desc, double u, double v,
                                                int nu, int nv, out Vector3D result)
    {
        result = new Vector3D();
        double uRep = 0.0;
        double vRep = 0.0;
        desc.ParamMap.Map(u, v, out uRep, out vRep);
        if (desc.Domain.HasValue && !desc.Domain.Value.Contains(uRep, vRep))
        {
            return false;
        }
        int nuRep = nu;
        int nvRep = nv;
        if (desc.ParamMap.SwapUV)
        {
            nuRep = nv;
            nvRep = nu;
        }
        Vector3D repVec = desc.Representation!.EvalDN(uRep, vRep, nuRep, nvRep);
        double scale = Math.Pow(desc.ParamMap.ScaleU, nuRep) * Math.Pow(desc.ParamMap.ScaleV, nvRep);
        result = repVec * scale;
        return true;
    }
}
