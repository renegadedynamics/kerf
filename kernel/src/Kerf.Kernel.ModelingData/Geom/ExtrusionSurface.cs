// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfLinearExtrusion.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfLinearExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A surface of linear extrusion: a basis curve swept along a direction. The U parameter is the parameter of
/// the basis curve, the V parameter (infinite) the distance along the direction.
/// P(U, V) = BasisCurve(U) + V * Direction.
/// </summary>
/// <remarks>
/// The surface keeps its own copy of the basis curve; <see cref="SweptSurface.BasisCurve"/> returns that copy
/// (shared, as in OCCT; callers can't change it). The evaluation representation (<see cref="HasEvalRepresentation"/>)
/// is set by <see cref="WithEvalRepresentation"/>, which stores a copy of the descriptor; every modification drops it.
/// </remarks>
public class ExtrusionSurface : SweptSurface
{
    private RepSurfaceDesc.Base? _evalRep;

    /// <summary>The extrusion of a copy of C along V. Nothing is checked (C may be a line parallel to V).</summary>
    public ExtrusionSurface(Curve3D c, in Direction3D v)
    {
        _basisCurve = (Curve3D)c.Copy();
        _direction = v;
        _smooth = c.Continuity();
    }

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation => _evalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal RepSurfaceDesc.Base? EvalRepresentation() => _evalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        CheckMutable();
        RepUtils.ValidateSurfaceDesc(desc, this);
        _evalRep = desc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        _evalRep = null;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public ExtrusionSurface WithEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        RepUtils.ValidateSurfaceDesc(desc, this);
        ExtrusionSurface copy = (ExtrusionSurface)Copy();
        copy.SetEvalRepresentation(RepUtils.CloneSurfaceDesc(desc));
        return copy;
    }

    /// <summary>Changes the direction of extrusion.</summary>
    internal void SetDirection(in Direction3D v)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _direction = v;
    }

    /// <summary>Changes the extruded curve (to a copy of C).</summary>
    internal void SetBasisCurve(Curve3D c)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _smooth = c.Continuity();
        _basisCurve = (Curve3D)c.Copy();
    }

    /// <summary>A copy extruded along V.</summary>
    public ExtrusionSurface WithDirection(in Direction3D v)
    {
        ExtrusionSurface copy = (ExtrusionSurface)Copy();
        copy.SetDirection(v);
        return copy;
    }

    /// <summary>A copy extruding (a copy of) C.</summary>
    public ExtrusionSurface WithBasisCurve(Curve3D c)
    {
        ExtrusionSurface copy = (ExtrusionSurface)Copy();
        copy.SetBasisCurve(c);
        return copy;
    }

    /// <summary>Reverses the U direction (the basis curve).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisCurve();
        _basisCurve!.Reverse();
    }

    /// <summary>The reversed parameter of the basis curve.</summary>
    public sealed override double UReversedParameter(double u) => _basisCurve!.ReversedParameter(u);

    /// <summary>Reverses the V direction (the direction of extrusion).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        _direction = _direction.Reversed();
    }

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double v) => (-v);

    /// <summary>U between the bounds of the basis curve; V infinite.</summary>
    public sealed override void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        v1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        v2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        u1 = _basisCurve!.FirstParameter();
        u2 = _basisCurve.LastParameter();
    }

    public sealed override Point3D EvalD0(double u, double v)
    {
        Point3D evalRepResult;
        if (RepUtils.TryEvalSurfaceD0(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Point3D basisD0 = _basisCurve!.EvalD0(u);
        Point3D p;
        ExtrusionUtils.CalculateD0(basisD0, v, _direction.XYZ(), out p);
        return p;
    }

    public sealed override ResD1 EvalD1(double u, double v)
    {
        ResD1 evalRepResult;
        if (RepUtils.TryEvalSurfaceD1(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve3D.ResD1 basisD1 = _basisCurve!.EvalD1(u);
        ResD1 result;
        ExtrusionUtils.CalculateD1(basisD1.Point,
                                        basisD1.D1,
                                        v,
                                        _direction.XYZ(),
                                        out result.Point,
                                        out result.D1U,
                                        out result.D1V);
        return result;
    }

    public sealed override ResD2 EvalD2(double u, double v)
    {
        ResD2 evalRepResult;
        if (RepUtils.TryEvalSurfaceD2(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve3D.ResD2 basisD2 = _basisCurve!.EvalD2(u);
        ResD2 result;
        ExtrusionUtils.CalculateD2(basisD2.Point,
                                        basisD2.D1,
                                        basisD2.D2,
                                        v,
                                        _direction.XYZ(),
                                        out result.Point,
                                        out result.D1U,
                                        out result.D1V,
                                        out result.D2U,
                                        out result.D2V,
                                        out result.D2UV);
        return result;
    }

    public sealed override ResD3 EvalD3(double u, double v)
    {
        ResD3 evalRepResult;
        if (RepUtils.TryEvalSurfaceD3(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve3D.ResD3 basisD3 = _basisCurve!.EvalD3(u);
        ResD3 result;
        ExtrusionUtils.CalculateD3(basisD3.Point,
                                        basisD3.D1,
                                        basisD3.D2,
                                        basisD3.D3,
                                        v,
                                        _direction.XYZ(),
                                        out result.Point,
                                        out result.D1U,
                                        out result.D1V,
                                        out result.D2U,
                                        out result.D2V,
                                        out result.D2UV,
                                        out result.D3U,
                                        out result.D3V,
                                        out result.D3UUV,
                                        out result.D3UVV);
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException();
        }

        Vector3D evalRepResult;
        if (RepUtils.TryEvalSurfaceDN(_evalRep, u, v, nu, nv, out evalRepResult))
        {
            return evalRepResult;
        }

        if (nv == 0)
        {
            Vector3D dn = _basisCurve!.EvalDN(u, nu);
            return ExtrusionUtils.CalculateDN(dn, _direction.XYZ(), nu, nv);
        }
        else if (nu == 0 && nv == 1)
        {
            return new Vector3D(_direction.XYZ());
        }
        return new Vector3D(0.0, 0.0, 0.0);
    }

    /// <summary>The U iso-curve: the line through the point of parameter U of the basis curve, along the direction.</summary>
    public sealed override Curve3D UIso(double u)
    {
        LineCurve3D l;
        l = new LineCurve3D(_basisCurve!.Value(u), _direction);
        return l;
    }

    /// <summary>The V iso-curve: a copy of the basis curve translated by V * Direction.</summary>
    public sealed override Curve3D VIso(double v)
    {
        Vector3D vdir = new Vector3D(_direction);
        vdir = vdir.Multiplied(v);
        Curve3D c;
        c = (Curve3D)_basisCurve!.Translated(vdir);
        return c;
    }

    /// <summary>True if the basis curve is CN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public sealed override bool IsCNu(int n)
    {
        Standard_RangeError_Raise_if(n < 0, " ");
        return _basisCurve!.IsCN(n);
    }

    /// <summary>True.</summary>
    public sealed override bool IsCNv(int n) => true;

    /// <summary>Transforms the direction and the basis curve.</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _direction = _direction.Transformed(t);
        OwnBasisCurve();
        _basisCurve!.Transform(t);
    }

    /// <summary>True if the basis curve is closed.</summary>
    public sealed override bool IsUClosed => _basisCurve!.IsClosed;

    /// <summary>True if the basis curve is periodic.</summary>
    public sealed override bool IsUPeriodic => _basisCurve!.IsPeriodic;

    /// <summary>False.</summary>
    public sealed override bool IsVClosed => false;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic => false;

    /// <summary>U changed into the transformed parameter of the basis curve; V scaled by |T.ScaleFactor()| unless infinite.</summary>
    public sealed override void TransformParameters(ref double u, ref double v, in Transform3D t)
    {
        u = _basisCurve!.TransformedParameter(u, t);
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v))
        {
            v *= Math.Abs(t.ScaleFactor());
        }
    }

    /// <summary>The product of an affinity along U (the basis curve's) and one along V (the scale factor).</summary>
    public sealed override GeneralTransform2D ParametricTransformation(in Transform3D t)
    {
        // transformation in the V Direction
        GeneralTransform2D tv = new GeneralTransform2D();
        Axis2D axis = new Axis2D(Constants.Origin2d(), Constants.DX2d());
        tv = tv.WithAffinity(axis, Math.Abs(t.ScaleFactor()));
        // transformation in the U Direction
        GeneralTransform2D tu = new GeneralTransform2D();
        axis = new Axis2D(Constants.Origin2d(), Constants.DY2d());
        tu = tu.WithAffinity(axis, _basisCurve!.ParametricTransformation(t));
        return tu * tv;
    }

    /// <summary>A new, unfrozen surface on a copy of the basis curve, with a copy of the evaluation representation.</summary>
    public sealed override Geometry3D Copy()
    {
        ExtrusionSurface copy = new ExtrusionSurface(_basisCurve!, _direction);
        copy._evalRep = RepUtils.CloneSurfaceDesc(_evalRep);
        return copy;
    }
}
