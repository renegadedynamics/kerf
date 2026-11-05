// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>
/// A surface of revolution: a basis curve (the meridian) rotated about an axis. The U parameter is the angle of
/// rotation, in [0, 2*PI], and the V parameter is the parameter of the meridian.
/// P(U, V) = Rotation(Axis, U) * BasisCurve(V).
/// </summary>
/// <remarks>
/// The surface keeps its own copy of the basis curve; <see cref="SweptSurface.BasisCurve"/> returns that copy
/// (shared, as in OCCT; callers can't change it). The evaluation representation (<see cref="HasEvalRepresentation"/>)
/// is set by <see cref="WithEvalRepresentation"/>, which stores a copy of the descriptor; every modification drops it.
/// </remarks>
public sealed class RevolutionSurface : SweptSurface
{
    private RepSurfaceDesc.Base? _evalRep;
    private Point3D _loc;

    /// <summary>
    /// The revolution of a copy of C about A1. C must be planar and lie in the same plane as A1 (not checked);
    /// the surface may be self-intersecting otherwise.
    /// </summary>
    public RevolutionSurface(Curve3D c, in Axis3D a1)
    {
        _loc = a1.Location;
        _direction = a1.Direction;
        SetBasisCurve(c);
    }

    /// <summary>True if an evaluation representation is attached.</summary>
    internal bool HasEvalRepresentation => _evalRep is not null;

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
    internal RevolutionSurface WithEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        RepUtils.ValidateSurfaceDesc(desc, this);
        RevolutionSurface copy = (RevolutionSurface)Copy();
        copy.SetEvalRepresentation(RepUtils.CloneSurfaceDesc(desc));
        return copy;
    }

    /// <summary>Changes the axis of revolution.</summary>
    internal void SetAxis(in Axis3D a1)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _direction = a1.Direction;
        _loc = a1.Location;
    }

    /// <summary>Changes the direction of the axis of revolution.</summary>
    internal void SetDirection(in Direction3D v)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _direction = v;
    }

    /// <summary>Changes the revolved curve (to a copy of C).</summary>
    internal void SetBasisCurve(Curve3D c)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _basisCurve = (Curve3D)c.Copy();
        _smooth = c.Continuity;
    }

    /// <summary>Changes the location point of the axis of revolution.</summary>
    internal void SetLocation(in Point3D p)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _loc = p;
    }

    /// <summary>A copy revolved about A1.</summary>
    public RevolutionSurface WithAxis(in Axis3D a1)
    {
        RevolutionSurface copy = (RevolutionSurface)Copy();
        copy.SetAxis(a1);
        return copy;
    }

    /// <summary>A copy whose axis of revolution has the direction V.</summary>
    public RevolutionSurface WithDirection(in Direction3D v)
    {
        RevolutionSurface copy = (RevolutionSurface)Copy();
        copy.SetDirection(v);
        return copy;
    }

    /// <summary>A copy revolving (a copy of) C.</summary>
    public RevolutionSurface WithBasisCurve(Curve3D c)
    {
        RevolutionSurface copy = (RevolutionSurface)Copy();
        copy.SetBasisCurve(c);
        return copy;
    }

    /// <summary>A copy whose axis of revolution passes through P.</summary>
    public RevolutionSurface WithLocation(in Point3D p)
    {
        RevolutionSurface copy = (RevolutionSurface)Copy();
        copy.SetLocation(p);
        return copy;
    }

    /// <summary>The axis of revolution.</summary>
    public Axis3D Axis => new Axis3D(_loc, _direction);

    /// <summary>The location point of the axis of revolution.</summary>
    public Point3D Location => _loc;

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public Frame3D ReferencePlane
    {
        get
        {
            throw Refusal.NotSupported(new MethodNotImplementedException());
        }
    }

    /// <summary>Reverses the U direction (the direction of the axis of revolution).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        _direction = _direction.Reversed();
    }

    /// <summary>2*PI - U.</summary>
    public sealed override double UReversedParameter(double u) => (2.0 * M_PI - u);

    /// <summary>Reverses the V direction (the basis curve).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisCurve();
        _basisCurve!.Reverse();
    }

    /// <summary>The reversed parameter of the basis curve.</summary>
    public sealed override double VReversedParameter(double v) => _basisCurve!.ReversedParameter(v);

    /// <summary>V changed into the transformed parameter of the basis curve; U unchanged.</summary>
    internal sealed override void TransformParameters(ref double u, ref double v, in Transform3D t)
    {
        v = _basisCurve!.TransformedParameter(v, t);
    }

    /// <summary>An affinity along V of the basis curve's parametric transformation.</summary>
    internal sealed override GeneralTransform2D ParametricTransformation(in Transform3D t)
    {
        GeneralTransform2D t2 = new GeneralTransform2D();
        Axis2D axis = new Axis2D(Constants.Origin2d(), Constants.DX2d());
        t2 = t2.WithAffinity(axis, _basisCurve!.ParametricTransformation(t));
        return t2;
    }

    /// <summary>U in [0, 2*PI]; V between the bounds of the basis curve.</summary>
    internal sealed override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = 2.0 * M_PI;
        v1 = _basisCurve!.FirstParameter;
        v2 = _basisCurve.LastParameter;
    }

    /// <summary>True.</summary>
    public sealed override bool IsUClosed => true;

    /// <summary>True if the basis curve is closed.</summary>
    public sealed override bool IsVClosed => _basisCurve!.IsClosed;

    /// <summary>True.</summary>
    internal sealed override bool IsCNu(int n) => true;

    /// <summary>True if the basis curve is CN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCNv(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, " ");
        return _basisCurve!.IsCN(n);
    }

    /// <summary>True (period 2*PI).</summary>
    public sealed override bool IsUPeriodic => true;

    /// <summary>True if the basis curve is periodic.</summary>
    public sealed override bool IsVPeriodic => _basisCurve!.IsPeriodic;

    /// <summary>The U iso-curve: a copy of the basis curve rotated by U.</summary>
    public sealed override Curve3D UIso(double u)
    {
        Curve3D c = (Curve3D)_basisCurve!.Copy();
        Axis3D rotAxis = new Axis3D(_loc, _direction);
        c.Rotate(rotAxis, u);
        return c;
    }

    /// <summary>The V iso-curve: the circle described by the point of parameter V of the basis curve.</summary>
    public sealed override Curve3D VIso(double v)
    {
        CircleCurve3D circ;
        Point3D pc = _basisCurve!.Value(v);
        Line3D l1 = new Line3D(_loc, _direction);
        double rad = l1.DistanceTo(pc);
        Frame3D rep;
        if (rad > Constants.Resolution())
        {
            Xyz p = pc.XYZ();
            Xyz c = new Xyz();
            c.SetLinearForm((p - _loc.XYZ()).Dot(_direction.XYZ()), _direction.XYZ(), _loc.XYZ());
            p = p - c;
            if (p.Modulus() > Constants.Resolution())
            {
                Direction3D d = new Direction3D(p.Normalized());
                rep = new Frame3D(new Point3D(c), _direction, d);
            }
            else
            {
                rep = new Frame3D(new Point3D(c), _direction);
            }
        }
        else
        {
            rep = new Frame3D(pc, _direction);
        }
        circ = new CircleCurve3D(rep, rad);
        return circ;
    }

    internal sealed override Point3D EvalD0(double u, double v)
    {
        Point3D evalRepResult;
        if (RepUtils.TryEvalSurfaceD0(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Point3D basisD0 = _basisCurve!.EvalD0(v);
        Point3D p;
        RevolutionUtils.CalculateD0(basisD0, u, new Axis3D(_loc, _direction), out p);
        return p;
    }

    internal sealed override ResD1 EvalD1(double u, double v)
    {
        ResD1 evalRepResult;
        if (RepUtils.TryEvalSurfaceD1(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve3D.ResD1 basisD1 = _basisCurve!.EvalD1(v);
        ResD1 result;
        RevolutionUtils.CalculateD1(basisD1.Point,
                                         basisD1.D1,
                                         u,
                                         new Axis3D(_loc, _direction),
                                         out result.Point,
                                         out result.D1U,
                                         out result.D1V);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u, double v)
    {
        ResD2 evalRepResult;
        if (RepUtils.TryEvalSurfaceD2(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve3D.ResD2 basisD2 = _basisCurve!.EvalD2(v);
        ResD2 result;
        RevolutionUtils.CalculateD2(basisD2.Point,
                                         basisD2.D1,
                                         basisD2.D2,
                                         u,
                                         new Axis3D(_loc, _direction),
                                         out result.Point,
                                         out result.D1U,
                                         out result.D1V,
                                         out result.D2U,
                                         out result.D2V,
                                         out result.D2UV);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u, double v)
    {
        ResD3 evalRepResult;
        if (RepUtils.TryEvalSurfaceD3(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve3D.ResD3 basisD3 = _basisCurve!.EvalD3(v);
        ResD3 result;
        RevolutionUtils.CalculateD3(basisD3.Point,
                                         basisD3.D1,
                                         basisD3.D2,
                                         basisD3.D3,
                                         u,
                                         new Axis3D(_loc, _direction),
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
    internal sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }

        Vector3D evalRepResult;
        if (RepUtils.TryEvalSurfaceDN(_evalRep, u, v, nu, nv, out evalRepResult))
        {
            return evalRepResult;
        }

        Vector3D curvePtOrDN;
        if (nu == 0)
        {
            curvePtOrDN = _basisCurve!.EvalDN(v, nv);
        }
        else if (nv == 0)
        {
            Point3D d0 = _basisCurve!.EvalD0(v);
            curvePtOrDN = new Vector3D(d0.XYZ() - _loc.XYZ());
        }
        else
        {
            curvePtOrDN = _basisCurve!.EvalDN(v, nv);
        }
        return RevolutionUtils.CalculateDN(curvePtOrDN, u, new Axis3D(_loc, _direction), nu, nv);
    }

    /// <summary>Transforms the axis and the basis curve; a transformation reversing orientation also reverses U.</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _loc = _loc.Transformed(t);
        _direction = _direction.Transformed(t);
        OwnBasisCurve();
        _basisCurve!.Transform(t);
        if (t.ScaleFactor * t.RotationPart.Determinant < 0.0)
        {
            UReverse();
        }
    }

    /// <summary>A new, unfrozen surface on a copy of the basis curve, with a copy of the evaluation representation.</summary>
    public sealed override Geometry3D Copy()
    {
        RevolutionSurface copy = new RevolutionSurface(_basisCurve!, Axis);
        copy._evalRep = RepUtils.CloneSurfaceDesc(_evalRep);
        return copy;
    }
}
