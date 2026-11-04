// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ElSpine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ElSpine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// An elementary spine: the smooth guide curve (C2) of one part of a spine, which the walking and the
/// approximation of a fillet follow, with its end points and tangents, its vertices with tangents and the fillet
/// surfaces before and after it.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): they set its curve, bounds and neighbours through the
/// internal mutators. As an <see cref="CurveAdaptorBase"/> it evaluates its curve between its own bounds.
/// </remarks>
internal sealed class ElSpine : CurveAdaptorBase
{
    private CurveAdaptor _curve = new();
    private Point3D _ptfirst = new();
    private Point3D _ptlast = new();
    private Vector3D _tgfirst = new();
    private Vector3D _tglast = new();
    private NCollection_Sequence<Axis3D> _verticesWithTangents = new();
    private SurfData? _previous;
    private SurfData? _next;
    private double _pfirst;
    private double _plast;
    private double _period;
    private bool _periodic;
    private double _pfirstsav;
    private double _plastsav;

    public ElSpine()
    {
        _pfirst = 0.0;
        _plast = 0.0;
        _period = 0.0;
        _periodic = false;
        _pfirstsav = global::Kerf.Engine.Foundation.Precision.Infinite();
        _plastsav = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Shallow copy of the adaptor.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        ElSpine copy = new();

        CurveAdaptorBase curve = _curve.ShallowCopy();
        copy._curve = (CurveAdaptor)curve;

        copy._ptfirst = _ptfirst;
        copy._ptlast = _ptlast;
        copy._tgfirst = _tgfirst;
        copy._tglast = _tglast;
        copy._verticesWithTangents = new NCollection_Sequence<Axis3D>(_verticesWithTangents);
        copy._previous = _previous;
        copy._next = _next;
        copy._pfirst = _pfirst;
        copy._plast = _plast;
        copy._period = _period;
        copy._periodic = _periodic;
        copy._pfirstsav = _pfirstsav;
        copy._plastsav = _plastsav;

        return copy;
    }

    public override double FirstParameter()
    {
        return _pfirst;
    }

    public override double LastParameter()
    {
        return _plast;
    }

    public double GetSavedFirstParameter()
    {
        return _pfirstsav;
    }

    public double GetSavedLastParameter()
    {
        return _plastsav;
    }

    public override Continuity Continuity()
    {
        return _curve.Continuity();
    }

    public override int NbIntervals(Continuity s)
    {
        return _curve.NbIntervals(s);
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _curve.Intervals(t, s);
    }

    /// <summary>The curve between <paramref name="first"/> and <paramref name="last"/>.</summary>
    public override CurveAdaptorBase Trim(double first, double last, double tol)
    {
        return _curve.Trim(first, last, tol);
    }

    public override double Resolution(double r3d)
    {
        return _curve.Resolution(r3d);
    }

    public override CurveKind GetType()
    {
        return _curve.GetType();
    }

    public override bool IsPeriodic
    {
        get
        {
            return _periodic;
        }
    }

    internal void SetPeriodic(bool i)
    {
        _periodic = i;
        _period = _plast - _pfirst;
    }

    public override double Period()
    {
        if (!_periodic)
        {
            throw new FailureException("ElSpine non periodique");
        }

        return _period;
    }

    public override Point3D EvalD0(double absC)
    {
        return _curve.EvalD0(absC);
    }

    internal override Curve3D.ResD1 EvalD1(double absC)
    {
        return _curve.EvalD1(absC);
    }

    internal override Curve3D.ResD2 EvalD2(double absC)
    {
        return _curve.EvalD2(absC);
    }

    internal override Curve3D.ResD3 EvalD3(double absC)
    {
        return _curve.EvalD3(absC);
    }

    internal void FirstParameter(double p)
    {
        _pfirst = p;
    }

    internal void LastParameter(double p)
    {
        _plast = p;
    }

    internal void SaveFirstParameter()
    {
        _pfirstsav = _pfirst;
    }

    internal void SaveLastParameter()
    {
        _plastsav = _plast;
    }

    /// <summary>
    /// Moves the origin of the periodic B-spline curve to <paramref name="o"/>. OCCT edits the curve in place: the
    /// builders' own approximated curve, never a shape's (a frozen curve throws).
    /// </summary>
    internal void SetOrigin(double o)
    {
        if (!_periodic)
        {
            throw new FailureException("Elspine non periodique");
        }

        BSplineCurve3D? bs = _curve.Curve() as BSplineCurve3D;
        if (bs is not null)
        {
            bs.SetOrigin(o, global::Kerf.Engine.Foundation.Precision.PConfusion());
            _curve.Load(bs);
        }
    }

    internal void SetFirstPointAndTgt(in Point3D p, in Vector3D t)
    {
        _ptfirst = p;
        _tgfirst = t;
    }

    internal void SetLastPointAndTgt(in Point3D p, in Vector3D t)
    {
        _ptlast = p;
        _tglast = t;
    }

    internal void AddVertexWithTangent(in Axis3D anAx1)
    {
        _verticesWithTangents.Append(anAx1);
    }

    public void FirstPointAndTgt(out Point3D p, out Vector3D t)
    {
        p = _ptfirst;
        t = _tgfirst;
    }

    public void LastPointAndTgt(out Point3D p, out Vector3D t)
    {
        p = _ptlast;
        t = _tglast;
    }

    public int VertexCount
    {
        get
        {
            return _verticesWithTangents.Length();
        }
    }

    public Axis3D VertexWithTangent(int index)
    {
        return _verticesWithTangents[index];
    }

    internal void SetCurve(Curve3D c)
    {
        _curve.Load(c);
    }

    public SurfData? Previous()
    {
        return _previous;
    }

    internal ref SurfData? ChangePrevious() => ref _previous;

    public SurfData? Next()
    {
        return _next;
    }

    internal ref SurfData? ChangeNext() => ref _next;

    public override Line3D Line()
    {
        return _curve.Line();
    }

    public override Circle3D Circle()
    {
        return _curve.Circle();
    }

    public override Ellipse3D Ellipse()
    {
        return _curve.Ellipse();
    }

    public override Hyperbola3D Hyperbola()
    {
        return _curve.Hyperbola();
    }

    public override Parabola3D Parabola()
    {
        return _curve.Parabola();
    }

    public override BezierCurve3D Bezier()
    {
        return _curve.Bezier();
    }

    public override BSplineCurve3D BSpline()
    {
        return _curve.BSpline();
    }
}
