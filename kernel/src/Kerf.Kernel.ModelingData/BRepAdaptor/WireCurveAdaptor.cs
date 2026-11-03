// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_CompCurve.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_CompCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>
/// A wire of the BRep topology seen as a 3D curve: its non-degenerated edges in connection order
/// (<see cref="WireExplorer"/>), each over a unit of parameter, or over its length with
/// <c>KnotByCurvilinearAbcissa</c>.
/// </summary>
/// <remarks>
/// C0 and C1 continuity are not assumed between edges. The curve is never periodic, even for a single periodic
/// edge. The wire must be valid: a chain of connected edges. A working object (decision 214).
/// </remarks>
public class WireCurveAdaptor : CurveAdaptorBase
{
    private Wire _wire = new();
    private double _tFirst;
    private double _tLast;
    private double _pTol;
    private NCollection_HArray1<EdgeCurveAdaptor>? _curves;
    private NCollection_HArray1<double>? _knots;
    private int _curIndex;
    private bool _forward;
    private bool _isbyAC;

    /// <summary>An undefined curve, with no wire loaded.</summary>
    public WireCurveAdaptor()
    {
        _tFirst = 0.0;
        _tLast = 0.0;
        _pTol = 0.0;
        _curIndex = -1;
        _forward = false;
        _isbyAC = false;
    }

    /// <summary>The wire, its edges parametrised by unit parameters or by length (<paramref name="isAC"/>).</summary>
    public WireCurveAdaptor(in Wire wire, bool isAC = false)
    {
        _wire = wire;
        _tFirst = 0.0;
        _tLast = 0.0;
        _pTol = 0.0;
        _curIndex = -1;
        _forward = false;
        _isbyAC = isAC;
        Initialize(wire, isAC);
    }

    /// <summary>The wire trimmed to [theFirst, theLast].</summary>
    public WireCurveAdaptor(in Wire wire, bool isAC, double first, double last, double tolerance)
    {
        _wire = wire;
        _tFirst = first;
        _tLast = last;
        _pTol = tolerance;
        _curIndex = -1;
        _forward = false;
        _isbyAC = isAC;
        Initialize(wire, isAC, first, last, tolerance);
    }

    /// <summary>A shallow copy: shallow copies of the edges' adaptors, the knots shared.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        WireCurveAdaptor copy = new WireCurveAdaptor();

        copy._wire = _wire;
        copy._tFirst = _tFirst;
        copy._tLast = _tLast;
        copy._pTol = _pTol;
        copy._curves = new NCollection_HArray1<EdgeCurveAdaptor>(1, _curves!.Length());
        for (int i = 1; i <= _curves.Length(); ++i)
        {
            CurveAdaptorBase curve = _curves.Value(i).ShallowCopy();
            EdgeCurveAdaptor brepCurve = (EdgeCurveAdaptor)curve;
            copy._curves.SetValue(i, brepCurve);
        }
        copy._knots = _knots;
        copy._curIndex = _curIndex;
        copy._forward = _forward;
        copy._isbyAC = _isbyAC;

        return copy;
    }

    /// <summary>Loads the wire; AC parametrises the edges by their lengths.</summary>
    public void Initialize(in Wire w, bool ac)
    {
        int ii, nbEdge;
        var wexp = new WireExplorer();
        Edge e;

        _wire = w;
        _pTol = 0.0;
        _isbyAC = ac;

        for (nbEdge = 0, wexp.Init(_wire); wexp.More(); wexp.Next())
        {
            if (!Tool.Degenerated(wexp.Current()))
            {
                nbEdge++;
            }
        }

        if (nbEdge == 0)
        {
            return;
        }

        _curIndex = (nbEdge + 1) / 2;
        _curves = new NCollection_HArray1<EdgeCurveAdaptor>(1, nbEdge);
        // OCCT's array holds adaptors by value, default-constructed
        for (int i = 1; i <= nbEdge; i++)
        {
            _curves.SetValue(i, new EdgeCurveAdaptor());
        }
        _knots = new NCollection_HArray1<double>(1, nbEdge + 1);
        _knots.SetValue(1, 0.0);

        for (ii = 0, wexp.Init(_wire); wexp.More(); wexp.Next())
        {
            e = wexp.Current();
            if (!Tool.Degenerated(e))
            {
                ii++;
                _curves.ChangeValue(ii).Initialize(e);
                if (ac)
                {
                    _knots.SetValue(ii + 1, _knots.Value(ii));
                    _knots.ChangeValue(ii + 1) += AbscissaPoint.Length(_curves.ChangeValue(ii));
                }
                else
                {
                    _knots.SetValue(ii + 1, (double)ii);
                }
            }
        }

        _forward = true; // Default ; The Reverse Edges are parsed.
        if ((nbEdge > 2) || ((nbEdge == 2) && (!_wire.IsClosed)))
        {
            Orientation or = _curves.Value(1).Edge().Orientation;
            Vertex vi = new Vertex(), vl;
            Exploration.CommonVertex(_curves.Value(1).Edge(), _curves.Value(2).Edge(), ref vi);
            vl = Exploration.LastVertex(_curves.Value(1).Edge());
            if (vi.IsSame(vl))
            { // The direction of parsing is always preserved
                if (or == Reversed)
                {
                    _forward = false;
                }
            }
            else
            { // The direction of parsing is always reversed
                if (or != Reversed)
                {
                    _forward = false;
                }
            }
        }

        _tFirst = 0;
        _tLast = _knots.Value(_knots.Length());
    }

    /// <summary>Loads the wire, trimmed to [First, Last] with the tolerance Tol.</summary>
    public void Initialize(in Wire w, bool ac, double first, double last, double tol)
    {
        Initialize(w, ac);
        _tFirst = first;
        _tLast = last;
        _pTol = tol;

        // Trim the extremal curves.
        EdgeCurveAdaptor hc;
        int i1, i2;
        double f = _tFirst, l = _tLast, d;
        i1 = i2 = _curIndex;
        Prepare(ref f, out d, ref i1);
        Prepare(ref l, out d, ref i2);
        _curIndex = (i1 + i2) / 2; // Small optimization
        if (i1 == i2)
        {
            if (l > f)
            {
                hc = (EdgeCurveAdaptor)_curves!.Value(i1).Trim(f, l, _pTol);
            }
            else
            {
                hc = (EdgeCurveAdaptor)_curves!.Value(i1).Trim(l, f, _pTol);
            }
            _curves.SetValue(i1, hc);
        }
        else
        {
            EdgeCurveAdaptor c1 = _curves!.Value(i1);
            EdgeCurveAdaptor c2 = _curves.Value(i2);
            double k;

            k = c1.LastParameter();
            if (k > f)
            {
                hc = (EdgeCurveAdaptor)c1.Trim(f, k, _pTol);
            }
            else
            {
                hc = (EdgeCurveAdaptor)c1.Trim(k, f, _pTol);
            }
            _curves.SetValue(i1, hc);

            k = c2.FirstParameter();
            if (k <= l)
            {
                hc = (EdgeCurveAdaptor)c2.Trim(k, l, _pTol);
            }
            else
            {
                hc = (EdgeCurveAdaptor)c2.Trim(l, k, _pTol);
            }
            _curves.SetValue(i2, hc);
        }
    }

    /// <summary>The wire.</summary>
    public Wire Wire() => _wire;

    /// <summary>The edge holding the parameter U, and the parameter on it.</summary>
    public void Edge(double u, out Edge e, out double uonE)
    {
        int index = _curIndex;
        uonE = u;
        Prepare(ref uonE, out _, ref index);
        e = _curves!.Value(index).Edge();
    }

    public override double FirstParameter() => _tFirst;

    public override double LastParameter() => _tLast;

    public override Continuity Continuity()
    {
        if (_curves!.Length() > 1)
        {
            return C0;
        }
        return _curves.Value(1).Continuity();
    }

    public override int NbIntervals(Continuity s)
    {
        int nbInt, ii;
        for (ii = 1, nbInt = 0; ii <= _curves!.Length(); ii++)
        {
            nbInt += _curves.ChangeValue(ii).NbIntervals(s);
        }

        return nbInt;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int ii, jj, kk, n;
        double f, F, delta;

        // First curve (direction of parsing of the edge)
        n = _curves!.ChangeValue(1).NbIntervals(s);
        NCollection_HArray1<double> ti = new NCollection_HArray1<double>(1, n + 1);
        _curves.ChangeValue(1).Intervals(ti.ChangeArray1(), s);
        InvPrepare(1, out f, out delta);
        F = _knots!.Value(1);
        if (delta < 0)
        {
            // invert the direction of parsing
            for (kk = 1, jj = ti.Length(); jj > 0; kk++, jj--)
            {
                t[kk] = F + (ti.Value(jj) - f) * delta;
            }
        }
        else
        {
            for (kk = 1; kk <= ti.Length(); kk++)
            {
                t[kk] = F + (ti.Value(kk) - f) * delta;
            }
        }

        // and the next
        for (ii = 2; ii <= _curves.Length(); ii++)
        {
            n = _curves.ChangeValue(ii).NbIntervals(s);
            if (n != ti.Length() - 1)
            {
                ti = new NCollection_HArray1<double>(1, n + 1);
            }
            _curves.ChangeValue(ii).Intervals(ti.ChangeArray1(), s);
            InvPrepare(ii, out f, out delta);
            F = _knots.Value(ii);
            if (delta < 0)
            {
                // invert the direction of parcing
                for (jj = ti.Length() - 1; jj > 0; kk++, jj--)
                {
                    t[kk] = F + (ti.Value(jj) - f) * delta;
                }
            }
            else
            {
                for (jj = 2; jj <= ti.Length(); kk++, jj++)
                {
                    t[kk] = F + (ti.Value(jj) - f) * delta;
                }
            }
        }
    }

    /// <summary>The wire trimmed to [First, Last], a new adaptor.</summary>
    public override CurveAdaptorBase Trim(double first, double last, double tol)
    {
        WireCurveAdaptor hc = new WireCurveAdaptor(_wire, _isbyAC, first, last, tol);
        return hc;
    }

    public override bool IsClosed => _wire.IsClosed;

    public override bool IsPeriodic => false;

    public override double Period() => (_tLast - _tFirst);

    public sealed override Point3D EvalD0(double theU)
    {
        double u = theU;
        int index = _curIndex;
        Prepare(ref u, out _, ref index);
        return _curves!.Value(index).EvalD0(u);
    }

    internal sealed override Curve3D.ResD1 EvalD1(double theU)
    {
        double u = theU;
        int index = _curIndex;
        Prepare(ref u, out double d, ref index);
        Curve3D.ResD1 res = _curves!.Value(index).EvalD1(u);
        res.D1 *= d;
        return res;
    }

    internal sealed override Curve3D.ResD2 EvalD2(double theU)
    {
        double u = theU;
        int index = _curIndex;
        Prepare(ref u, out double d, ref index);
        Curve3D.ResD2 res = _curves!.Value(index).EvalD2(u);
        res.D1 *= d;
        res.D2 *= d * d;
        return res;
    }

    internal sealed override Curve3D.ResD3 EvalD3(double theU)
    {
        double u = theU;
        int index = _curIndex;
        Prepare(ref u, out double d, ref index);
        Curve3D.ResD3 res = _curves!.Value(index).EvalD3(u);
        res.D1 *= d;
        res.D2 *= d * d;
        res.D3 *= d * d * d;
        return res;
    }

    public sealed override Vector3D EvalDN(double theU, int n)
    {
        double u = theU;
        int index = _curIndex;
        Prepare(ref u, out double d, ref index);
        return _curves!.Value(index).EvalDN(u, n) * Math.Pow(d, n);
    }

    /// <summary>The smallest parametric resolution of the edges.</summary>
    public override double Resolution(double r3d)
    {
        double res = 1.0e200, r;
        int ii, l = _curves!.Length();
        for (ii = 1; ii <= l; ii++)
        {
            r = _curves.Value(ii).Resolution(r3d);
            if (r < res)
            {
                res = r;
            }
        }
        return res;
    }

    /// <summary>Always GeomAbs_OtherCurve.</summary>
    public override CurveKind GetType()
    {
        return OtherCurve; // temporary
        //  if ( myCurves->Length() > 1) return GeomAbs_OtherCurve;
        //  return myCurves->Value(1).GetType();
    }

    public override Line3D Line() => _curves!.Value(1).Line();

    public override Circle3D Circle() => _curves!.Value(1).Circle();

    public override Ellipse3D Ellipse() => _curves!.Value(1).Ellipse();

    public override Hyperbola3D Hyperbola() => _curves!.Value(1).Hyperbola();

    public override Parabola3D Parabola() => _curves!.Value(1).Parabola();

    public override int Degree() => _curves!.Value(1).Degree();

    public override bool IsRational => _curves!.Value(1).IsRational;

    public override int PoleCount => _curves!.Value(1).PoleCount;

    public override int KnotCount => _curves!.Value(1).KnotCount;

    public override BezierCurve3D Bezier() => _curves!.Value(1).Bezier();

    public override BSplineCurve3D BSpline() => _curves!.Value(1).BSpline();

    // When the parameter is close to "node" the rule is determined
    // depending on the sign of tol:
    //   - negative -> Rule preceding to the node.
    //   - positive -> Rule following after the node.
    private void Prepare(ref double w, out double delta, ref int curIndex)
    {
        double f, l, wtest, eps;
        int ii;
        if (w - _tFirst < _tLast - w)
        {
            eps = _pTol;
        }
        else
        {
            eps = -_pTol;
        }

        wtest = w + eps; // Offset to discriminate the nodes

        // Find the index
        bool trouve = false;
        if (_knots!.Value(curIndex) > wtest)
        {
            for (ii = curIndex - 1; ii > 0 && !trouve; ii--)
            {
                if (_knots.Value(ii) <= wtest)
                {
                    curIndex = ii;
                    trouve = true;
                }
            }
            if (!trouve)
            {
                curIndex = 1; // Out of limits...
            }
        }

        else if (_knots.Value(curIndex + 1) <= wtest)
        {
            for (ii = curIndex + 1; ii <= _curves!.Length() && !trouve; ii++)
            {
                if (_knots.Value(ii + 1) > wtest)
                {
                    curIndex = ii;
                    trouve = true;
                }
            }
            if (!trouve)
            {
                curIndex = _curves.Length(); // Out of limits...
            }
        }

        // Invert ?
        Edge e = _curves!.Value(curIndex).Edge();
        Orientation or = e.Orientation;
        bool reverse;
        reverse = (_forward && (or == Reversed)) || (!_forward && (or != Reversed));

        // Calculate the local parameter
        Tool.Range(e, out f, out l);
        delta = _knots.Value(curIndex + 1) - _knots.Value(curIndex);
        if (delta > _pTol * 1.0e-9)
        {
            delta = (l - f) / delta;
        }

        if (reverse)
        {
            delta *= -1;
            w = l + (w - _knots.Value(curIndex)) * delta;
        }
        else
        {
            w = f + (w - _knots.Value(curIndex)) * delta;
        }
    }

    private void InvPrepare(int index, out double first, out double delta)
    {
        // Invert?
        Edge e = _curves!.Value(index).Edge();
        Orientation or = e.Orientation;
        bool reverse;
        reverse = (_forward && (or == Reversed)) || (!_forward && (or != Reversed));

        // Calculate the parameters of reparametrisation
        // such as : T = Ti + (t-First)*Delta
        Tool.Range(e, out double f, out double l);
        delta = _knots!.Value(index + 1) - _knots.Value(index);
        if (l - f > _pTol * 1.0e-9)
        {
            delta /= (l - f);
        }

        if (reverse)
        {
            delta *= -1;
            first = l;
        }
        else
        {
            first = f;
        }
    }
}
