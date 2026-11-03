// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_TrimmedCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_TrimmedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A portion of a curve between two parameters U1 and U2 of a basis curve, with the basis curve's
/// parametrization. The trimmed curve is oriented as the basis curve when <c>Sense</c> is true, and opposite
/// otherwise (the basis curve is then reversed). For a periodic basis curve the parameters are first adjusted to
/// its period.
/// </summary>
/// <remarks>
/// The trimmed curve keeps its own copy of the basis curve (OCCT copies it in the constructor; a trimmed basis
/// curve is replaced by its own basis). <see cref="BasisCurve"/> returns that copy frozen, as OCCT returns its
/// handle; this curve's internal mutators then copy the basis curve again before changing it (decision 132).
/// </remarks>
public sealed class TrimmedCurve2D : BoundedCurve2D
{
    private Curve2D _basisCurve;
    private double _uTrim1;
    private double _uTrim2;

    /// <summary>A new, unfrozen trimmed curve equal to this one (with its own copy of the basis curve).</summary>
    public sealed override Geometry2D Copy()
    {
        TrimmedCurve2D tc;
        tc = new TrimmedCurve2D(_basisCurve, _uTrim1, _uTrim2);
        return tc;
    }

    /// <summary>
    /// The portion of C between U1 and U2. If C is periodic and theAdjustPeriodic is true, U1 and U2 are brought
    /// into C's period (U2 within one period after U1); Sense false gives the curve from U1 to U2 the other way.
    /// If C is not periodic, the curve goes from min(U1, U2) to max(U1, U2), in C's sense if Sense is true
    /// and U1 &lt; U2.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// C is null, U1 == U2, or (C not periodic) U1 or U2 is outside C's bounds.
    /// </exception>
    public TrimmedCurve2D(Curve2D c, double u1, double u2, bool sense = true, bool adjustPeriodic = true)
    {
        _uTrim1 = u1;
        _uTrim2 = u2;
        if (c is null)
        {
            throw new ConstructionErrorException("Geom2d_TrimmedCurve:: C is null");
        }
        // kill trimmed basis curves
        TrimmedCurve2D? t = c as TrimmedCurve2D;
        if (t is not null)
        {
            _basisCurve = (Curve2D)t._basisCurve.Copy(); // OCCT.NET: the field, so that T's basis curve isn't frozen
        }
        else
        {
            _basisCurve = (Curve2D)c.Copy();
        }
        SetTrim(u1, u2, sense, adjustPeriodic);
    }

    // OCCT.NET: the basis curve may have been handed out (and frozen) by BasisCurve(); copy it before changing it.
    private void OwnBasisCurve()
    {
        if (_basisCurve.IsFrozen)
        {
            _basisCurve = (Curve2D)_basisCurve.Copy();
        }
    }

    /// <summary>Reverses the trimmed curve: the basis curve is reversed and the trim parameters changed accordingly.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        OwnBasisCurve();
        double u1 = _basisCurve.ReversedParameter(_uTrim2);
        double u2 = _basisCurve.ReversedParameter(_uTrim1);
        _basisCurve.Reverse();
        SetTrim(u1, u2, true, false);
    }

    /// <summary>The basis curve's reversed parameter of U.</summary>
    public sealed override double ReversedParameter(double u) => _basisCurve.ReversedParameter(u);

    /// <summary>Changes the trim parameters, as the constructor sets them.</summary>
    /// <exception cref="ConstructionErrorException">
    /// U1 == U2, or (basis curve not periodic) U1 or U2 is outside its bounds.
    /// </exception>
    internal void SetTrim(double u1, double u2, bool sense = true, bool adjustPeriodic = true)
    {
        CheckMutable();
        bool sameSense = true;
        if (u1 == u2)
        {
            throw new ConstructionErrorException("Geom2d_TrimmedCurve::U1 == U2");
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
                // OCCT: the message names Geom_TrimmedCurve (cosmetic).
                throw new ConstructionErrorException("Geom_TrimmedCurve::parameters out of range");
            }
        }
        if (!sameSense)
        {
            Reverse();
        }
    }

    /// <summary>A copy with the trim parameters U1 and U2 (see the constructor).</summary>
    public TrimmedCurve2D WithTrim(double u1, double u2, bool sense = true, bool adjustPeriodic = true)
    {
        TrimmedCurve2D copy = (TrimmedCurve2D)Copy();
        copy.SetTrim(u1, u2, sense, adjustPeriodic);
        return copy;
    }

    /// <summary>The basis curve, frozen (it is shared with this trimmed curve).</summary>
    public Curve2D BasisCurve
    {
        get
        {
            _basisCurve.Freeze();
            return _basisCurve;
        }
    }

    /// <summary>
    /// The basis curve, not frozen: for ported code that changes it in place through the trimmed curve, as OCCT does
    /// (MAT2d's <c>Bisector_BisecAna::SetTrim</c> on a Bisector_Bisec's curve), and for reads that must not freeze it.
    /// </summary>
    internal Curve2D ChangeBasisCurve() => _basisCurve;

    /// <summary>The basis curve's continuity.</summary>
    public sealed override Continuity Continuity => _basisCurve.Continuity;

    /// <summary>The basis curve's IsCN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCN(int n)
    {
        Standard_RangeError_Raise_if(n < 0, " ");
        return _basisCurve.IsCN(n);
    }

    /// <summary>The point of the basis curve at the last trim parameter.</summary>
    public sealed override Point2D EndPoint => _basisCurve.Value(_uTrim2);

    /// <summary>The first trim parameter.</summary>
    public sealed override double FirstParameter => _uTrim1;

    /// <summary>
    /// True if the trimmed range of a periodic basis curve is a whole number of periods, or if the end points are
    /// within Precision::Computational() (squared distance).
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
            return Value(FirstParameter).SquareDistanceTo(Value(LastParameter))
                   <= global::Kerf.Engine.Foundation.Precision.Computational();
        }
    }

    /// <summary>True if the basis curve is periodic and the trimmed range is a whole number of periods.</summary>
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

    /// <summary>The basis curve's period.</summary>
    /// <exception cref="NoSuchObjectException">The basis curve is not periodic.</exception>
    public sealed override double Period => _basisCurve.Period;

    /// <summary>The last trim parameter.</summary>
    public sealed override double LastParameter => _uTrim2;

    /// <summary>The point of the basis curve at the first trim parameter.</summary>
    public sealed override Point2D StartPoint
    {
        get
        {
            Point2D p;
            p = _basisCurve.Value(_uTrim1);
            return p;
        }
    }

    internal sealed override Point2D EvalD0(double u) => _basisCurve.EvalD0(u);

    internal sealed override ResD1 EvalD1(double u) => _basisCurve.EvalD1(u);

    internal sealed override ResD2 EvalD2(double u) => _basisCurve.EvalD2(u);

    internal sealed override ResD3 EvalD3(double u) => _basisCurve.EvalD3(u);

    internal sealed override Vector2D EvalDN(double u, int n) => _basisCurve.EvalDN(u, n);

    /// <summary>Transforms the basis curve and the trim parameters.</summary>
    protected internal sealed override void Transform(in Transform2D t)
    {
        CheckMutable();
        OwnBasisCurve();
        _basisCurve.Transform(t);
        double u1 = _basisCurve.TransformedParameter(_uTrim1, t);
        double u2 = _basisCurve.TransformedParameter(_uTrim2, t);
        SetTrim(u1, u2, true, false);
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    internal sealed override double TransformedParameter(double u, in Transform2D t) =>
        _basisCurve.TransformedParameter(u, t);

    /// <summary>The basis curve's parametric transformation.</summary>
    internal sealed override double ParametricTransformation(in Transform2D t) =>
        _basisCurve.ParametricTransformation(t);
}
