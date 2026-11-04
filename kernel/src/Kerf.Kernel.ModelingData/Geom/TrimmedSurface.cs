// Port of: src/ModelingData/TKG3d/Geom/Geom_RectangularTrimmedSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_RectangularTrimmedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A portion of a basis surface limited by two values of U, two values of V, or both (a rectangle of its
/// parametric domain). The trimmed surface has the parametrization of its basis surface, and keeps a copy of it
/// (a trimmed basis is replaced by its own basis).
/// </summary>
/// <remarks>
/// <see cref="BasisSurface"/> returns the stored copy frozen, as OCCT returns its handle; this surface's internal
/// mutators then copy the basis surface again before changing it (decision 159).
/// </remarks>
public sealed class TrimmedSurface : BoundedSurface
{
    private Surface _basisSurf;
    private double _utrim1;
    private double _vtrim1;
    private double _utrim2;
    private double _vtrim2;
    private bool _isutrimmed;
    private bool _isvtrimmed;

    /// <summary>
    /// The part of a copy of S between U1 and U2 and between V1 and V2. USense and VSense choose, for a periodic
    /// direction, between the two portions (false: opposite to the basis orientation); for a non-periodic one, they
    /// say whether the trimmed surface keeps the basis orientation (U1 &lt; U2 with USense true, or U1 &gt; U2 with
    /// USense false), otherwise it is reversed.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// U1 == U2 or V1 == V2, or a non-periodic direction is trimmed outside the bounds of S.
    /// </exception>
    public TrimmedSurface(Surface s,
                                          double u1,
                                          double u2,
                                          double v1,
                                          double v2,
                                          bool uSense = true,
                                          bool vSense = true)
    {
        _utrim1 = u1;
        _vtrim1 = v1;
        _utrim2 = u2;
        _vtrim2 = v2;
        _isutrimmed = true;
        _isvtrimmed = true;

        // kill trimmed basis surfaces
        TrimmedSurface? t = s as TrimmedSurface;
        if (t is not null)
        {
            _basisSurf = (Surface)t._basisSurf.Copy(); // OCCT.NET: the field, so that T's basis surface isn't frozen
        }
        else
        {
            _basisSurf = (Surface)s.Copy();
        }

        OffsetSurface? o = _basisSurf as OffsetSurface;
        if (o is not null)
        {
            TrimmedSurface s2 =
                new TrimmedSurface(o.BasisSurface, u1, u2, v1, v2, uSense, vSense);
            _basisSurf = new OffsetSurface(s2, o.Offset, true);
        }

        SetTrim(u1, u2, v1, v2, uSense, vSense);
    }

    /// <summary>
    /// The part of a copy of S between Param1 and Param2 in U (UTrim true) or in V (UTrim false); the other
    /// direction is not trimmed, unless S is itself trimmed in it. Sense as in the other constructor.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// Param1 == Param2, or the direction is not periodic and is trimmed outside the bounds of S.
    /// </exception>
    public TrimmedSurface(Surface s,
                                          double param1,
                                          double param2,
                                          bool uTrim,
                                          bool sense = true)
    {
        // kill trimmed basis surfaces
        TrimmedSurface? t = s as TrimmedSurface;
        if (t is not null)
        {
            _basisSurf = (Surface)t._basisSurf.Copy(); // OCCT.NET: the field, so that T's basis surface isn't frozen
        }
        else
        {
            _basisSurf = (Surface)s.Copy();
        }

        OffsetSurface? o = _basisSurf as OffsetSurface;
        if (o is not null)
        {
            TrimmedSurface s2 =
                new TrimmedSurface(o.BasisSurface, param1, param2, uTrim, sense);
            _basisSurf = new OffsetSurface(s2, o.Offset, true);
        }

        if (t is not null)
        {
            if (uTrim && t._isvtrimmed)
            {
                SetTrim(param1, param2, t._vtrim1, t._vtrim2, sense, true);
                return;
            }
            else if (!uTrim && t._isutrimmed)
            {
                SetTrim(t._utrim1, t._utrim2, param1, param2, true, sense);
                return;
            }
        }

        // OCCT reads isutrimmed/isvtrimmed (and utrim*/vtrim*) here before this constructor initialises them. OCCT's
        // allocator clears memory by default (MMGT_CLEAR), so they are false/0 there as in C#.
        SetTrim(param1, param2, uTrim, sense);
    }

    /// <summary>Changes the trim values in both directions (see the first constructor).</summary>
    /// <exception cref="ConstructionErrorException">As the first constructor.</exception>
    internal void SetTrim(double u1,
                          double u2,
                          double v1,
                          double v2,
                          bool uSense = true,
                          bool vSense = true)
    {
        CheckMutable();
        SetTrim(u1, u2, v1, v2, true, true, uSense, vSense);
    }

    /// <summary>Changes the trim values in one direction (see the second constructor).</summary>
    /// <exception cref="ConstructionErrorException">As the second constructor.</exception>
    internal void SetTrim(double param1, double param2, bool uTrim, bool sense = true)
    {
        CheckMutable();
        // dummy arguments to call general SetTrim
        double dummy_a = 0.0;
        double dummy_b = 0.0;
        bool dummy_Sense = true;

        if (uTrim)
        {
            if (_isvtrimmed)
            {
                SetTrim(param1, param2, _vtrim1, _vtrim2, true, true, sense, dummy_Sense);
            }
            else
            {
                SetTrim(param1, param2, dummy_a, dummy_b, true, false, sense, dummy_Sense);
            }
        }
        else
        {
            if (_isutrimmed)
            {
                SetTrim(_utrim1, _utrim2, param1, param2, true, true, dummy_Sense, sense);
            }
            else
            {
                SetTrim(dummy_a, dummy_b, param1, param2, false, true, dummy_Sense, sense);
            }
        }
    }

    private void SetTrim(double u1,
                         double u2,
                         double v1,
                         double v2,
                         bool uTrim,
                         bool vTrim,
                         bool uSense,
                         bool vSense)
    {
        // A NaN compares false either way, and UReverse or VReverse would come back here for ever
        Refusal.ArgumentIf(uTrim && (double.IsNaN(u1) || double.IsNaN(u2)) || vTrim && (double.IsNaN(v1) || double.IsNaN(v2)), FailureKind.ConstructionError, "The trimming parameters aren't numbers");
        bool usameSense = true;
        bool vsameSense = true;
        double udeb, ufin, vdeb, vfin;

        _basisSurf.ParameterBounds(out udeb, out ufin, out vdeb, out vfin);

        // Trimming the U-Direction
        _isutrimmed = uTrim;
        if (!uTrim)
        {
            _utrim1 = udeb;
            _utrim2 = ufin;
        }
        else
        {
            if (u1 == u2)
            {
                throw Refusal.Argument(new ConstructionErrorException("Geom_RectangularTrimmedSurface::U1==U2"));
            }

            if (_basisSurf.IsUPeriodic)
            {
                usameSense = uSense;

                // set uTrim1 in the range Udeb , Ufin
                // set uTrim2 in the range uTrim1 , uTrim1 + Period()
                _utrim1 = u1;
                _utrim2 = u2;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(udeb,
                                      ufin,
                                      Min(Math.Abs(_utrim2 - _utrim1) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                                      ref _utrim1,
                                      ref _utrim2);
            }
            else
            {
                if (u1 < u2)
                {
                    usameSense = uSense;
                    _utrim1 = u1;
                    _utrim2 = u2;
                }
                else
                {
                    usameSense = !uSense;
                    _utrim1 = u2;
                    _utrim2 = u1;
                }

                if ((udeb - _utrim1 > global::Kerf.Engine.Foundation.Precision.PConfusion()) || (_utrim2 - ufin > global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    throw Refusal.Argument(new ConstructionErrorException(
                        "Geom_RectangularTrimmedSurface::U parameters out of range"));
                }
            }
        }

        // Trimming the V-Direction
        _isvtrimmed = vTrim;
        if (!vTrim)
        {
            _vtrim1 = vdeb;
            _vtrim2 = vfin;
        }
        else
        {
            if (v1 == v2)
            {
                throw Refusal.Argument(new ConstructionErrorException("Geom_RectangularTrimmedSurface::V1==V2"));
            }

            if (_basisSurf.IsVPeriodic)
            {
                vsameSense = vSense;

                // set vTrim1 in the range Vdeb , Vfin
                // set vTrim2 in the range vTrim1 , vTrim1 + Period()
                _vtrim1 = v1;
                _vtrim2 = v2;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(vdeb,
                                      vfin,
                                      Min(Math.Abs(_vtrim2 - _vtrim1) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                                      ref _vtrim1,
                                      ref _vtrim2);
            }
            else
            {
                if (v1 < v2)
                {
                    vsameSense = vSense;
                    _vtrim1 = v1;
                    _vtrim2 = v2;
                }
                else
                {
                    vsameSense = !vSense;
                    _vtrim1 = v2;
                    _vtrim2 = v1;
                }

                if ((vdeb - _vtrim1 > global::Kerf.Engine.Foundation.Precision.PConfusion()) || (_vtrim2 - vfin > global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    throw Refusal.Argument(new ConstructionErrorException(
                        "Geom_RectangularTrimmedSurface::V parameters out of range"));
                }
            }
        }

        if (!usameSense)
        {
            UReverse();
        }
        if (!vsameSense)
        {
            VReverse();
        }
    }

    /// <summary>A copy trimmed in both directions (see the first constructor).</summary>
    /// <exception cref="ConstructionErrorException">As the first constructor.</exception>
    public TrimmedSurface WithTrim(double u1,
                                                   double u2,
                                                   double v1,
                                                   double v2,
                                                   bool uSense = true,
                                                   bool vSense = true)
    {
        TrimmedSurface copy = (TrimmedSurface)Copy();
        copy.SetTrim(u1, u2, v1, v2, uSense, vSense);
        return copy;
    }

    /// <summary>A copy trimmed in one direction (see the second constructor).</summary>
    /// <exception cref="ConstructionErrorException">As the second constructor.</exception>
    public TrimmedSurface WithTrim(double param1, double param2, bool uTrim, bool sense = true)
    {
        TrimmedSurface copy = (TrimmedSurface)Copy();
        copy.SetTrim(param1, param2, uTrim, sense);
        return copy;
    }

    /// <summary>The basis surface (the trimmed surface's own copy, frozen; shared, not copied).</summary>
    public Surface BasisSurface
    {
        get
        {
            _basisSurf.Freeze();
            return _basisSurf;
        }
    }

    // OCCT.NET: the basis surface may have been handed out (and frozen) by BasisSurface(); copy it before changing it.
    private void OwnBasisSurface()
    {
        if (_basisSurf.IsFrozen)
        {
            _basisSurf = (Surface)_basisSurf.Copy();
        }
    }

    /// <summary>Reverses the U direction: the basis surface is reversed and the U trim values recomputed.</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        OwnBasisSurface();
        double u1 = _basisSurf.UReversedParameter(_utrim2);
        double u2 = _basisSurf.UReversedParameter(_utrim1);
        _basisSurf.UReverse();
        SetTrim(u1, u2, _vtrim1, _vtrim2, _isutrimmed, _isvtrimmed, true, true);
    }

    /// <summary>The reversed U parameter of the basis surface.</summary>
    public sealed override double UReversedParameter(double u) => _basisSurf.UReversedParameter(u);

    /// <summary>Reverses the V direction: the basis surface is reversed and the V trim values recomputed.</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        OwnBasisSurface();
        double v1 = _basisSurf.VReversedParameter(_vtrim2);
        double v2 = _basisSurf.VReversedParameter(_vtrim1);
        _basisSurf.VReverse();
        SetTrim(_utrim1, _utrim2, v1, v2, _isutrimmed, _isvtrimmed, true, true);
    }

    /// <summary>The reversed V parameter of the basis surface.</summary>
    public sealed override double VReversedParameter(double v) => _basisSurf.VReversedParameter(v);

    /// <summary>The trim values (the bounds of the basis surface in a direction that is not trimmed).</summary>
    internal sealed override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = _utrim1;
        u2 = _utrim2;
        v1 = _vtrim1;
        v2 = _vtrim2;
    }

    /// <summary>The continuity of the basis surface.</summary>
    public sealed override Continuity Continuity => _basisSurf.Continuity;

    /// <summary>
    /// The basis surface's closure if U is not trimmed; otherwise true only if the basis is U-periodic and the
    /// trimmed range is a non-zero multiple of the period (within Precision::PConfusion()).
    /// </summary>
    public sealed override bool IsUClosed
    {
        get
        {
            if (!_isutrimmed)
            {
                return _basisSurf.IsUClosed;
            }

            if (_basisSurf.IsUPeriodic)
            {
                double period = _basisSurf.UPeriod;
                double length = _utrim2 - _utrim1;
                if (length > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(Math.IEEERemainder(length, period)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return true;
                }
            }
            return false;
        }
    }

    /// <summary>As <see cref="IsUClosed"/>, in V.</summary>
    public sealed override bool IsVClosed
    {
        get
        {
            if (!_isvtrimmed)
            {
                return _basisSurf.IsVClosed;
            }

            if (_basisSurf.IsVPeriodic)
            {
                double period = _basisSurf.VPeriod;
                double length = _vtrim2 - _vtrim1;
                if (length > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(Math.IEEERemainder(length, period)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return true;
                }
            }
            return false;
        }
    }

    /// <summary>True if the basis surface is CN in U.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCNu(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, " ");
        return _basisSurf.IsCNu(n);
    }

    /// <summary>True if the basis surface is CN in V.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCNv(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, " ");
        return _basisSurf.IsCNv(n);
    }

    /// <summary>
    /// True if the basis surface is U-periodic and U is not trimmed, or trimmed over a non-zero multiple of the
    /// period (within Precision::PConfusion()).
    /// </summary>
    public sealed override bool IsUPeriodic
    {
        get
        {
            if (!_basisSurf.IsUPeriodic)
            {
                return false;
            }
            if (!_isutrimmed)
            {
                return true;
            }
            double period = _basisSurf.UPeriod;
            double length = _utrim2 - _utrim1;
            return length > global::Kerf.Engine.Foundation.Precision.PConfusion()
                   && Math.Abs(Math.IEEERemainder(length, period)) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
        }
    }

    /// <summary>The U period of the basis surface.</summary>
    /// <exception cref="NoSuchObjectException">The basis surface is not U-periodic.</exception>
    public sealed override double UPeriod => _basisSurf.UPeriod;

    /// <summary>As <see cref="IsUPeriodic"/>, in V.</summary>
    public sealed override bool IsVPeriodic
    {
        get
        {
            if (!_basisSurf.IsVPeriodic)
            {
                return false;
            }
            if (!_isvtrimmed)
            {
                return true;
            }
            double period = _basisSurf.VPeriod;
            double length = _vtrim2 - _vtrim1;
            return length > global::Kerf.Engine.Foundation.Precision.PConfusion()
                   && Math.Abs(Math.IEEERemainder(length, period)) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
        }
    }

    /// <summary>The V period of the basis surface.</summary>
    /// <exception cref="NoSuchObjectException">The basis surface is not V-periodic.</exception>
    public sealed override double VPeriod => _basisSurf.VPeriod;

    /// <summary>The U iso-curve of the basis surface, trimmed in V if V is trimmed.</summary>
    public sealed override Curve3D UIso(double u)
    {
        Curve3D c = _basisSurf.UIso(u);

        if (_isvtrimmed)
        {
            TrimmedCurve3D ct;
            ct = new TrimmedCurve3D(c, _vtrim1, _vtrim2, true);
            return ct;
        }
        else
        {
            return c;
        }
    }

    /// <summary>The V iso-curve of the basis surface, trimmed in U if U is trimmed.</summary>
    public sealed override Curve3D VIso(double v)
    {
        Curve3D c = _basisSurf.VIso(v);

        if (_isutrimmed)
        {
            TrimmedCurve3D ct;
            ct = new TrimmedCurve3D(c, _utrim1, _utrim2, true);
            return ct;
        }
        else
        {
            return c;
        }
    }

    internal sealed override Point3D EvalD0(double u, double v) => _basisSurf.EvalD0(u, v);

    internal sealed override ResD1 EvalD1(double u, double v) => _basisSurf.EvalD1(u, v);

    internal sealed override ResD2 EvalD2(double u, double v) => _basisSurf.EvalD2(u, v);

    internal sealed override ResD3 EvalD3(double u, double v) => _basisSurf.EvalD3(u, v);

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }
        return _basisSurf.EvalDN(u, v, nu, nv);
    }

    /// <summary>Transforms the basis surface and the trim values (through its TransformParameters).</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        OwnBasisSurface();
        _basisSurf.Transform(t);
        _basisSurf.TransformParameters(ref _utrim1, ref _vtrim1, t);
        _basisSurf.TransformParameters(ref _utrim2, ref _vtrim2, t);
    }

    /// <summary>The basis surface's TransformParameters.</summary>
    internal sealed override void TransformParameters(ref double u, ref double v, in Transform3D t)
    {
        _basisSurf.TransformParameters(ref u, ref v, t);
    }

    /// <summary>The basis surface's ParametricTransformation.</summary>
    internal sealed override GeneralTransform2D ParametricTransformation(in Transform3D t) => _basisSurf.ParametricTransformation(t);

    /// <summary>A new, unfrozen trimmed surface on a copy of the basis surface.</summary>
    public sealed override Geometry3D Copy()
    {
        TrimmedSurface s;

        if (_isutrimmed && _isvtrimmed)
        {
            s = new TrimmedSurface(_basisSurf, _utrim1, _utrim2, _vtrim1, _vtrim2, true, true);
        }
        else if (_isutrimmed)
        {
            s = new TrimmedSurface(_basisSurf, _utrim1, _utrim2, true, true);
        }
        else if (_isvtrimmed)
        {
            s = new TrimmedSurface(_basisSurf, _vtrim1, _vtrim2, false, true);
        }
        else
        {
            // OCCT returns a null handle here; no constructor leaves both directions untrimmed (the one-direction
            // SetTrim can only add a trimmed direction), so this is unreachable.
            throw Refusal.State(new ProgramErrorException("Geom_RectangularTrimmedSurface::Copy: not trimmed"));
        }

        return s;
    }
}
