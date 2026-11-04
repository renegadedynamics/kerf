// Port of: src/ModelingData/TKG3d/Geom/Geom_TrimmedCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_TrimmedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A portion of a basis curve limited by two parameter values inside its parametric domain. The trimmed curve
/// has the parametrization of its basis curve, between its first and last parameters; it keeps a copy of the
/// basis curve (a trimmed basis is replaced by its own basis). The derivatives have the orientation of the
/// basis curve.
/// </summary>
/// <remarks>
/// <see cref="BasisCurve"/> returns the stored copy frozen, as OCCT returns its handle; this curve's internal
/// mutators then copy the basis curve again before changing it (decision 127).
/// </remarks>
public sealed class TrimmedCurve3D : BoundedCurve3D
{
    private Curve3D _basisCurve;
    private double _uTrim1;
    private double _uTrim2;

    /// <summary>
    /// The part of a copy of C between U1 and U2, oriented from U1 to U2. On a periodic basis, Sense chooses
    /// between the two arcs (false: opposite to the basis orientation), and theAdjustPeriodic brings the
    /// parameters into the first period (they may then differ from U1 and U2 by a period).
    /// </summary>
    /// <exception cref="ConstructionErrorException">U1 == U2, or C is not periodic and U1 or U2 is out of its bounds.</exception>
    public TrimmedCurve3D(Curve3D c, double u1, double u2, bool sense = true, bool adjustPeriodic = true)
    {
        _uTrim1 = u1;
        _uTrim2 = u2;
        // kill trimmed basis curves
        TrimmedCurve3D? t = c as TrimmedCurve3D;
        if (t is not null)
        {
            _basisCurve = (Curve3D)t._basisCurve.Copy(); // OCCT.NET: the field, so that T's basis curve isn't frozen
        }
        else
        {
            _basisCurve = (Curve3D)c.Copy();
        }
        SetTrim(u1, u2, sense, adjustPeriodic);
    }

    /// <summary>A new, unfrozen trimmed curve equal to this one (on a copy of the basis curve).</summary>
    public sealed override Geometry3D Copy()
    {
        TrimmedCurve3D tc = new TrimmedCurve3D(_basisCurve, _uTrim1, _uTrim2);
        return tc;
    }

    // OCCT.NET: the basis curve may have been handed out (and frozen) by BasisCurve(); copy it before changing it.
    private void OwnBasisCurve()
    {
        if (_basisCurve.IsFrozen)
        {
            _basisCurve = (Curve3D)_basisCurve.Copy();
        }
    }

    /// <summary>
    /// Reverses the orientation: the basis curve is reversed and the trim parameters are recomputed
    /// (1 - U2 and 1 - U1 for a basis on [0, 1]).
    /// </summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        OwnBasisCurve();
        double u1 = _basisCurve.ReversedParameter(_uTrim2);
        double u2 = _basisCurve.ReversedParameter(_uTrim1);
        _basisCurve.Reverse();
        SetTrim(u1, u2, true, false);
    }

    /// <summary>The parameter on the reversed curve (the basis curve's).</summary>
    public sealed override double ReversedParameter(double u) => _basisCurve.ReversedParameter(u);

    /// <summary>Changes the trim parameters (see the constructor).</summary>
    /// <exception cref="ConstructionErrorException">U1 == U2, or the basis is not periodic and U1 or U2 is out of its bounds.</exception>
    internal void SetTrim(double u1, double u2, bool sense = true, bool adjustPeriodic = true)
    {
        CheckMutable();
        // A NaN compares false either way, and Reverse would come back here for ever
        Refusal.ArgumentIf(double.IsNaN(u1) || double.IsNaN(u2), FailureKind.ConstructionError, "The trimming parameters aren't numbers");
        bool sameSense = true;
        if (u1 == u2)
        {
            throw Refusal.Argument(new ConstructionErrorException("Geom_TrimmedCurve::U1 == U2"));
        }

        double udeb = _basisCurve.FirstParameter;
        double ufin = _basisCurve.LastParameter;

        if (_basisCurve.IsPeriodic)
        {
            sameSense = sense;

            // set uTrim1 in the range Udeb , Ufin
            // set uTrim2 in the range uTrim1 , uTrim1 + Period()
            _uTrim1 = u1;
            _uTrim2 = u2;
            if (adjustPeriodic)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(udeb,
                                      ufin,
                                      Min(Math.Abs(_uTrim2 - _uTrim1) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                                      ref _uTrim1,
                                      ref _uTrim2);
            }
        }
        else
        {
            if (u1 < u2)
            {
                sameSense = sense;
                _uTrim1 = u1;
                _uTrim2 = u2;
            }
            else
            {
                sameSense = !sense;
                _uTrim1 = u2;
                _uTrim2 = u1;
            }

            if ((udeb - _uTrim1 > global::Kerf.Engine.Foundation.Precision.PConfusion()) || (_uTrim2 - ufin > global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                throw Refusal.Argument(new ConstructionErrorException("Geom_TrimmedCurve::parameters out of range"));
            }
        }
        if (!sameSense)
        {
            Reverse();
        }
    }

    /// <summary>
    /// True if the basis is periodic and the trim spans a whole number of periods, or if the start and end
    /// points are within Precision::Computational() (squared distance).
    /// </summary>
    public sealed override bool IsClosed
    {
        get
        {
            if (_basisCurve.IsPeriodic)
            {
                double period = _basisCurve.Period;
                double length = LastParameter - FirstParameter;
                if (length > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(Math.IEEERemainder(length, period)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return true;
                }
            }
            return StartPoint.SquareDistanceTo(EndPoint) <= global::Kerf.Engine.Foundation.Precision.Computational();
        }
    }

    /// <summary>True if the basis is periodic and the trim spans a whole number of periods.</summary>
    public sealed override bool IsPeriodic
    {
        get
        {
            if (_basisCurve.IsPeriodic)
            {
                double period = _basisCurve.Period;
                double length = LastParameter - FirstParameter;
                if (length > global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(Math.IEEERemainder(length, period)) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return true;
                }
            }
            return false;
        }
    }

    /// <summary>The period of the basis curve.</summary>
    /// <exception cref="NoSuchObjectException">The basis curve is not periodic.</exception>
    public sealed override double Period => _basisCurve.Period;

    public sealed override Continuity Continuity => _basisCurve.Continuity;

    /// <summary>The basis curve, frozen (it is shared with this trimmed curve; decision 127).</summary>
    public Curve3D BasisCurve
    {
        get
        {
            _basisCurve.Freeze();
            return _basisCurve;
        }
    }

    internal sealed override Point3D EvalD0(double u) => _basisCurve.EvalD0(u);

    internal sealed override ResD1 EvalD1(double u) => _basisCurve.EvalD1(u);

    internal sealed override ResD2 EvalD2(double u) => _basisCurve.EvalD2(u);

    internal sealed override ResD3 EvalD3(double u) => _basisCurve.EvalD3(u);

    internal sealed override Vector3D EvalDN(double u, int n) => _basisCurve.EvalDN(u, n);

    public sealed override Point3D EndPoint => _basisCurve.Value(_uTrim2);

    public sealed override double FirstParameter => _uTrim1;

    public sealed override double LastParameter => _uTrim2;

    public sealed override Point3D StartPoint => _basisCurve.Value(_uTrim1);

    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCN(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, " ");
        return _basisCurve.IsCN(n);
    }

    /// <summary>Applies T to this trimmed curve and its basis curve.</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        OwnBasisCurve();
        _basisCurve.Transform(t);
        double u1 = _basisCurve.TransformedParameter(_uTrim1, t);
        double u2 = _basisCurve.TransformedParameter(_uTrim2, t);
        SetTrim(u1, u2, true, false);
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    internal sealed override double TransformedParameter(double u, in Transform3D t) => _basisCurve.TransformedParameter(u, t);

    /// <summary>The basis curve's parametric transformation.</summary>
    internal sealed override double ParametricTransformation(in Transform3D t) => _basisCurve.ParametricTransformation(t);

    // Non-mutating twin of SetTrim (OCCT.NET)

    /// <summary>A copy trimmed between U1 and U2 of the same basis curve (see the constructor).</summary>
    public TrimmedCurve3D WithTrim(double u1, double u2, bool sense = true, bool adjustPeriodic = true)
    {
        TrimmedCurve3D copy = (TrimmedCurve3D)Copy();
        copy.SetTrim(u1, u2, sense, adjustPeriodic);
        return copy;
    }
}
