// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_OffsetCurve.hxx
// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// The offset of an adapted 2D curve, between two parameters: the point at distance Offset from the curve
/// along its normal (the tangent rotated by -90 degrees).
/// </summary>
/// <remarks>An adaptor is a working object: <see cref="Load(Curve2DAdaptorBase)"/> is public, as in OCCT.</remarks>
internal class OffsetCurveAdaptor2D : Curve2DAdaptorBase
{
    private Curve2DAdaptorBase? _curve;
    private double _offset;
    private double _first;
    private double _last;

    /// <summary>An empty adaptor; load a curve before use.</summary>
    public OffsetCurveAdaptor2D()
    {
        _offset = 0.0;
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>The curve theCurve with offset 0 (and parameters 0 to 0 until an offset is loaded).</summary>
    public OffsetCurveAdaptor2D(Curve2DAdaptorBase curve)
    {
        _curve = curve;
        _offset = 0.0;
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>The offset of theCurve by theOffset, over the whole curve.</summary>
    public OffsetCurveAdaptor2D(Curve2DAdaptorBase curve, double offset)
    {
        _curve = curve;
        _offset = offset;
        _first = curve.FirstParameter();
        _last = curve.LastParameter();
    }

    /// <summary>The offset of theCurve by theOffset, between theWFirst and theWLast.</summary>
    public OffsetCurveAdaptor2D(Curve2DAdaptorBase curve, double offset, double wFirst, double wLast)
    {
        _curve = curve;
        _offset = offset;
        _first = wFirst;
        _last = wLast;
    }

    // OCCT's implicit copy constructor: shares the basis adaptor.
    private OffsetCurveAdaptor2D(OffsetCurveAdaptor2D other)
    {
        _curve = other._curve;
        _offset = other._offset;
        _first = other._first;
        _last = other._last;
    }

    /// <summary>A copy with a shallow copy of the basis adaptor.</summary>
    public override Curve2DAdaptorBase ShallowCopy()
    {
        OffsetCurveAdaptor2D copy = new OffsetCurveAdaptor2D();

        if (_curve is not null)
        {
            copy._curve = _curve.ShallowCopy();
        }
        copy._offset = _offset;
        copy._first = _first;
        copy._last = _last;

        return copy;
    }

    /// <summary>Loads the basis curve S, with offset 0 (the parameter range is unchanged).</summary>
    public void Load(Curve2DAdaptorBase s)
    {
        _curve = s;
        _offset = 0.0;
    }

    /// <summary>Loads the offset, over the whole basis curve.</summary>
    public void Load(double offset)
    {
        _offset = offset;
        _first = _curve!.FirstParameter();
        _last = _curve.LastParameter();
    }

    /// <summary>Loads the offset and the parameter range.</summary>
    public void Load(double offset, double wFirst, double wLast)
    {
        _offset = offset;
        _first = wFirst;
        _last = wLast;
    }

    /// <summary>The basis curve (null if none is loaded).</summary>
    public Curve2DAdaptorBase? Curve() => _curve;

    /// <summary>The offset value.</summary>
    public double Offset() => _offset;

    public override double FirstParameter() => _first;

    public override double LastParameter() => _last;

    /// <summary>The continuity, lowered from the basis curve's by one step of C3, C2, G2, C1, G1, C0.</summary>
    /// <exception cref="TypeMismatchException">The basis curve is C0.</exception>
    public override Continuity Continuity()
    {
        switch (_curve!.Continuity())
        {
            case CN:
                return CN;
            case C3:
                return C2;
            case C2:
                return G2;
            case G2:
                return C1;
            case C1:
                return G1;
            case G1:
                return C0;
            case C0:
                // No Continuity !!
                throw new TypeMismatchException("Adaptor2d_OffsetCurve::IntervalContinuity");
        }

        // portage WNT
        return C0;
    }

    public override int NbIntervals(Continuity s)
    {
        Continuity sh;
        if (s >= C2)
        {
            sh = CN;
        }
        else
        {
            sh = (Continuity)((int)s + 2);
        }

        int nbInter = _curve!.NbIntervals(sh);

        if (nbInter == 1)
        {
            return nbInter;
        }

        NCollection_Array1<double> t = new NCollection_Array1<double>(1, nbInter + 1);

        _curve.Intervals(t, sh);

        int first = 1;
        while (t[first] <= _first)
        {
            first++;
        }
        int last = nbInter + 1;
        while (t[last] >= _last)
        {
            last--;
        }
        return (last - first + 2);
    }

    public override void Intervals(NCollection_Array1<double> ti, Continuity s)
    {
        Continuity sh;
        if (s >= C2)
        {
            sh = CN;
        }
        else
        {
            sh = (Continuity)((int)s + 2);
        }

        int nbInter = _curve!.NbIntervals(sh);

        if (nbInter == 1)
        {
            ti[ti.Lower()] = _first;
            ti[ti.Lower() + 1] = _last;
            return;
        }

        NCollection_Array1<double> t = new NCollection_Array1<double>(1, nbInter + 1);
        _curve.Intervals(t, sh);

        int first = 1;
        while (t[first] <= _first)
        {
            first++;
        }
        int last = nbInter + 1;
        while (t[last] >= _last)
        {
            last--;
        }

        int i = ti.Lower(), j;
        for (j = first - 1; j <= last + 1; j++)
        {
            ti[i] = t[j];
            i++;
        }

        ti[ti.Lower()] = _first;
        ti[ti.Lower() + last - first + 2] = _last;
    }

    /// <summary>The same offset between First and Last (sharing the basis adaptor).</summary>
    public override Curve2DAdaptorBase Trim(double first, double last, double tol)
    {
        OffsetCurveAdaptor2D ho = new OffsetCurveAdaptor2D(this);
        ho.Load(_offset, first, last);
        return ho;
    }

    /// <summary>
    /// For a non-zero offset: true if the basis curve is closed and its end tangents are parallel and in the
    /// same sense (false for a C0 basis).
    /// </summary>
    public override bool IsClosed
    {
        get
        {
            if (_offset == 0.0)
            {
                return _curve!.IsClosed;
            }
            else
            {
                if (_curve!.Continuity() == C0)
                {
                    return false;
                }
                else
                {
                    if (_curve.IsClosed)
                    {
                        Vector2D dummy0, dummy1;
                        Point2D p;
                        _curve.D1(_curve.FirstParameter(), out p, out dummy0);
                        _curve.D1(_curve.LastParameter(), out p, out dummy1);
                        return dummy0.IsParallel(dummy1, global::Kerf.Engine.Foundation.Precision.Angular())
                               && !(dummy0.IsOpposite(dummy1, global::Kerf.Engine.Foundation.Precision.Angular()));
                    }
                    else
                    {
                        return false;
                    }
                }
            }
        }
    }

    public override bool IsPeriodic => _curve!.IsPeriodic;

    public override double Period() => _curve!.Period();

    public override Point2D Value(double u)
    {
        if (_offset != 0.0)
        {
            Point2D p;
            Vector2D v;
            _curve!.D1(u, out p, out v);
            OffsetCurveUtils2D.CalculateD0(ref p, v, _offset);
            return p;
        }
        else
        {
            return _curve!.Value(u);
        }
    }

    public override void D0(double u, out Point2D p) => p = Value(u);

    public override void D1(double u, out Point2D p, out Vector2D v)
    {
        if (_offset != 0.0)
        {
            Vector2D v2;
            _curve!.D2(u, out p, out v, out v2);
            OffsetCurveUtils2D.CalculateD1(ref p, ref v, v2, _offset);
        }
        else
        {
            _curve!.D1(u, out p, out v);
        }
    }

    public override void D2(double u, out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        if (_offset != 0.0)
        {
            Vector2D v3;
            _curve!.D3(u, out p, out v1, out v2, out v3);
            OffsetCurveUtils2D.CalculateD2(ref p, ref v1, ref v2, v3, false, _offset);
        }
        else
        {
            _curve!.D2(u, out p, out v1, out v2);
        }
    }

    /// <remarks>D3 is the derivative of D2 (OCCT's D2 and D3 here were wrong where the basis curve's speed varies; Kerf decision 1487).</remarks>
    public override void D3(double u, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        if (_offset != 0.0)
        {
            Vector2D v4 = _curve!.DN(u, 4);
            _curve.D3(u, out p, out v1, out v2, out v3);
            OffsetCurveUtils2D.CalculateD3(ref p, ref v1, ref v2, ref v3, v4, false, _offset);
        }
        else
        {
            _curve!.D3(u, out p, out v1, out v2, out v3);
        }
    }

    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public override Vector2D DN(double u, int n) => throw new MethodNotImplementedException("Adaptor2d_OffsetCurve::DN");

    /// <summary>Precision::PConfusion(R3d).</summary>
    public override double Resolution(double r3d) => global::Kerf.Engine.Foundation.Precision.PConfusion(r3d);

    /// <summary>The basis curve's type for a zero offset; Line or Circle for the offset of one; OffsetCurve otherwise.</summary>
    public override CurveKind GetType()
    {
        if (_offset == 0.0)
        {
            return _curve!.GetType();
        }
        else
        {
            switch (_curve!.GetType())
            {
                case global::Kerf.Geometry.CurveKind.Line:
                    return global::Kerf.Geometry.CurveKind.Line;

                case global::Kerf.Geometry.CurveKind.Circle:
                    return global::Kerf.Geometry.CurveKind.Circle;

                default:
                    return OffsetCurve;
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The offset curve is not a line.</exception>
    public override Line2D Line()
    {
        if (GetType() == global::Kerf.Geometry.CurveKind.Line)
        {
            Point2D p;
            Vector2D v;
            D1(0, out p, out v);
            return new Line2D(p, new Direction2D(v));
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Line");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset curve is not a circle, or its radius is 0.</exception>
    public override Circle2D Circle()
    {
        if (GetType() == global::Kerf.Geometry.CurveKind.Circle)
        {
            if (_offset == 0.0)
            {
                return _curve!.Circle();
            }
            else
            {
                Circle2D c1 = _curve!.Circle();
                double radius = c1.Radius;
                Frame2D axes = c1.Axis;
                Direction2D xd = axes.XDirection;
                Direction2D yd = axes.YDirection;
                double crossed = xd.X * yd.Y - xd.Y * yd.X;
                double signe = (crossed > 0.0) ? 1.0 : -1.0;

                radius += signe * _offset;
                if (radius > 0.0)
                {
                    return new Circle2D(axes, radius);
                }
                else if (radius < 0.0)
                {
                    radius = -radius;
                    axes = axes.WithXDirection((axes.XDirection).Reversed());
                    return new Circle2D(axes, radius);
                }
                else
                { // Cercle de rayon Nul
                    throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Circle");
                }
            }
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Circle");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not an ellipse.</exception>
    public override Ellipse2D Ellipse()
    {
        if (_curve!.GetType() == global::Kerf.Geometry.CurveKind.Ellipse && _offset == 0.0)
        {
            return _curve.Ellipse();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve:Ellipse");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a hyperbola.</exception>
    public override Hyperbola2D Hyperbola()
    {
        if (_curve!.GetType() == global::Kerf.Geometry.CurveKind.Hyperbola && _offset == 0.0)
        {
            return _curve.Hyperbola();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve:Hyperbola");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a parabola.</exception>
    public override Parabola2D Parabola()
    {
        if (_curve!.GetType() == global::Kerf.Geometry.CurveKind.Parabola && _offset == 0.0)
        {
            return _curve.Parabola();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve:Parabola");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a Bezier or B-spline curve.</exception>
    public override int Degree()
    {
        CurveKind type = _curve!.GetType();
        if ((type == BezierCurve || type == BSplineCurve) && _offset == 0.0)
        {
            return _curve.Degree();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Degree");
        }
    }

    /// <summary>The basis curve's IsRational for a zero offset, false otherwise.</summary>
    public override bool IsRational
    {
        get
        {
            if (_offset == 0.0)
            {
                return _curve!.IsRational;
            }
            return false;
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a Bezier or B-spline curve.</exception>
    public override int PoleCount
    {
        get
        {
            CurveKind type = _curve!.GetType();
            if ((type == BezierCurve || type == BSplineCurve) && _offset == 0.0)
            {
                return _curve.PoleCount;
            }
            else
            {
                throw new NoSuchObjectException("Adaptor2d_OffsetCurve::NbPoles");
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0.</exception>
    public override int KnotCount
    {
        get
        {
            if (_offset == 0.0)
            {
                return _curve!.KnotCount;
            }
            else
            {
                throw new NoSuchObjectException("Adaptor2d_OffsetCurve::NbKnots");
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the curve is not a Bezier curve.</exception>
    public override BezierCurve2D Bezier()
    {
        Standard_NoSuchObject_Raise_if(_offset != 0.0e0 || GetType() != BezierCurve,
                                       "Adaptor2d_OffsetCurve::Bezier() - wrong curve type");
        return _curve!.Bezier();
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the curve is not a B-spline curve.</exception>
    public override BSplineCurve2D BSpline()
    {
        Standard_NoSuchObject_Raise_if(_offset != 0.0e0 || GetType() != BSplineCurve,
                                       "Adaptor2d_OffsetCurve::BSpline() - wrong curve type");
        return _curve!.BSpline();
    }

    private static int NbPoints(Curve2DAdaptorBase curve)
    {
        int nbs = 20;

        if (curve.GetType() == BezierCurve)
        {
            nbs = Max(nbs, 3 + curve.PoleCount);
        }
        else if (curve.GetType() == BSplineCurve)
        {
            nbs = Max(nbs, curve.KnotCount * curve.Degree());
        }

        if (nbs > 300)
        {
            nbs = 300;
        }
        return nbs;
    }

    /// <summary>The basis curve's number of samples: 20, or more for Bezier and B-spline curves (300 at most).</summary>
    public override int SampleCount => NbPoints(_curve!);
}
