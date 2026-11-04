// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Spine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Spine.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Spine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified by isg, Thu Mar 17 09:21:31 1994

using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Data.ChamfMode;
using static Kerf.Engine.Fillets.Data.ErrorStatus;
using static Kerf.Engine.Fillets.Data.State;
using static Kerf.Engine.Fillets.Data.TypeOfConcavity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// The guideline of a 3D fillet or chamfer: a chain of edges of the solid, tangent to each other, with the nature of
/// its ends (free boundary, section, closed) and its parameterisation by curvilinear abscissa.
/// </summary>
/// <remarks>
/// <para>
/// OCCT notes that the guideline so represented is not C2, although the path claims it; the elementary spines
/// (<c>ElSpine</c>) are the C2 curves the walking follows.
/// </para>
/// <para>
/// A working object of the fillet builders (decision 21). Its edges are the input shape's own (frozen) edges; the
/// builders change the spine's state (status, parameters, elementary spines, error status) through the internal
/// members. Evaluating it by abscissa (<c>Value</c>, <c>D1</c>, <c>Absc</c>, <c>Parameter</c>) moves its current
/// elementary curve, so those members are internal too; the members OCCT declares <c>const</c> are public.
/// </para>
/// </remarks>
internal class Spine : Transient
{
    protected bool _splitdone;
    protected readonly NCollection_List<ElSpine> _elspines = new();
    protected readonly NCollection_List<ElSpine> _offsetElspines = new();
    protected ChamfMode _mode;

    private readonly EdgeCurveAdaptor _curve = new();
    // OCCT's myOffsetCurve is never used: left out.
    private int _indexofcurve;
    private TypeOfConcavity _typeOfConcavity;
    private State _firstState;
    private State _lastState;
    private readonly NCollection_Sequence<Shape> _spine = new();
    private readonly NCollection_Sequence<Shape> _offsetspine = new();
    private NCollection_HArray1<double>? _abscissa;
    // OCCT's offset_abscissa is never used: left out.
    private double _tolesp;
    private double _firstparam;
    private double _lastparam;
    private bool _firstprolon;
    private bool _lastprolon;
    private bool _firstistgt;
    private bool _lastistgt;
    private double _firsttgtpar;
    private double _lasttgtpar;
    private bool _hasfirsttgt;
    private bool _haslasttgt;
    private Point3D _firstori = new();
    private Point3D _lastori = new();
    private Vector3D _firsttgt = new();
    private Vector3D _lasttgt = new();
    private double _valref;
    private bool _hasref;
    private ErrorStatus _errorstate;

    public Spine()
        : this(global::Kerf.Engine.Foundation.Precision.Confusion())
    {
    }

    public Spine(double tol)
    {
        _splitdone = false;
        _mode = ChamfMode.ClassicChamfer;
        _indexofcurve = 0;
        _typeOfConcavity = TypeOfConcavity.Other;
        _firstState = State.OnSame;
        _lastState = State.OnSame;
        _tolesp = tol;
        _firstparam = 0.0;
        _lastparam = 0.0;
        _firstprolon = false;
        _lastprolon = false;
        _firstistgt = false;
        _lastistgt = false;
        _firsttgtpar = 0.0;
        _lasttgtpar = 0.0;
        _hasfirsttgt = false;
        _haslasttgt = false;
        _valref = 0.0;
        _hasref = false;
        _errorstate = global::Kerf.Engine.Fillets.Data.ErrorStatus.Ok;
    }

    /// <summary>Appends an edge to the guideline.</summary>
    internal void SetEdges(in Edge e)
    {
        _spine.Append(e);
    }

    /// <summary>Appends an edge to the offset guideline.</summary>
    internal void SetOffsetEdges(in Edge e)
    {
        _offsetspine.Append(e);
    }

    /// <summary>Puts the edge first, before all the others.</summary>
    internal void PutInFirst(in Edge e)
    {
        _spine.InsertBefore(1, e);
    }

    /// <summary>Puts the offset edge first, before all the others.</summary>
    internal void PutInFirstOffset(in Edge e)
    {
        _offsetspine.InsertBefore(1, e);
    }

    public int EdgeCount
    {
        get
        {
            return _spine.Length();
        }
    }

    public Edge Edges(int i)
    {
        return ShapeCasts.Edge(_spine.Value(i));
    }

    public Edge OffsetEdges(int i)
    {
        return ShapeCasts.Edge(_offsetspine.Value(i));
    }

    /// <summary>Stores whether the chain starts on a section, a free boundary or is closed.</summary>
    internal void SetFirstStatus(State s)
    {
        _firstState = s;
    }

    /// <summary>Stores whether the chain ends on a section, a free boundary or is closed.</summary>
    internal void SetLastStatus(State s)
    {
        _lastState = s;
    }

    internal virtual void AppendElSpine(ElSpine els)
    {
        _elspines.Append(els);
    }

    internal virtual void AppendOffsetElSpine(ElSpine els)
    {
        _offsetElspines.Append(els);
    }

    /// <summary>The elementary spine of the edge <paramref name="e"/>.</summary>
    public ElSpine? ElSpine(in Edge e)
    {
        return ElSpine(Index(e));
    }

    /// <summary>The elementary spine of the edge of index <paramref name="ie"/>.</summary>
    public ElSpine? ElSpine(int ie)
    {
        double wmil = 0.5 * (FirstParameter(ie) + LastParameter(ie));
        if (IsPeriodic)
        {
            wmil = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wmil, FirstParameter(), LastParameter());
        }

        return ElSpine(wmil);
    }

    /// <summary>The elementary spine at the abscissa <paramref name="w"/>, or null.</summary>
    public ElSpine? ElSpine(double w)
    {
        if (_elspines.Extent() == 1)
        {
            return _elspines.First();
        }
        else
        {
            NCollection_List<ElSpine>.Iterator it = new(_elspines);
            for (; it.More(); it.Next())
            {
                ElSpine cur = it.Value();
                double uf = cur.FirstParameter();
                double ul = cur.LastParameter();
                if (uf <= w && w <= ul)
                {
                    return cur;
                }
            }

            return null;
        }
    }

    internal NCollection_List<ElSpine> ChangeElSpines()
    {
        return _elspines;
    }

    internal NCollection_List<ElSpine> ChangeOffsetElSpines()
    {
        return _offsetElspines;
    }

    /// <summary>Forgets the elementary spines; with <paramref name="allData"/>, also the extensions.</summary>
    internal virtual void Reset(bool allData = false)
    {
        _splitdone = false;
        // if(AllData && !isconstant.IsNull()) isconstant->ChangeArray1().Init(0);
        _elspines.Clear();
        if (allData)
        {
            _firstparam = 0.0;
            _lastparam = _abscissa!.Value(_abscissa.Upper());
            _firstprolon = _lastprolon = false;
        }
    }

    public bool SplitDone()
    {
        return _splitdone;
    }

    internal void SplitDone(bool b)
    {
        _splitdone = b;
    }

    /// <summary>
    /// Computes the abscissae of the edges, which the evaluation by abscissa needs, and lowers the tolerance to the
    /// curve's parameter range.
    /// </summary>
    internal void Load()
    {
        if (_abscissa is not null)
        {
            // OCCT_DEBUG trace left out ("new load of CE").
        }

        int len = _spine.Length();
        _abscissa = new NCollection_HArray1<double>(1, len);
        double a1 = 0.0;
        for (int i = 1; i <= len; i++)
        {
            _curve.Initialize(ShapeCasts.Edge(_spine.Value(i)));
            a1 += AbscissaPoint.Length(_curve);
            _abscissa.SetValue(i, a1);
        }

        _indexofcurve = 1;
        _curve.Initialize(ShapeCasts.Edge(_spine.Value(1)));

        // Here, we should update tolesp according to curve parameter range
        // if tolesp candidate less than default initial value.
        double umin = FirstParameter();
        double umax = LastParameter();

        double new_tolesp = 5.0e-5 * (umax - umin);
        if (_tolesp > new_tolesp)
        {
            _tolesp = new_tolesp;
        }
    }

    /// <summary>Returns <paramref name="r3d"/>.</summary>
    public double Resolution(double r3d)
    {
        return r3d;
    }

    /// <summary>True if the first and last vertices are the same.</summary>
    public bool IsClosed
    {
        get
        {
            return (FirstVertex().IsSame(LastVertex()));
        }
    }

    public double FirstParameter()
    {
        if (_firstprolon)
        {
            return _firstparam;
        }

        return 0.0;
    }

    public double LastParameter()
    {
        if (_lastprolon)
        {
            return _lastparam;
        }

        return _abscissa!.Value(_abscissa.Upper());
    }

    /// <summary>Extends the guideline at its start to <paramref name="par"/>.</summary>
    internal void SetFirstParameter(double par)
    {
        // OCCT_DEBUG traces left out.
        _firstprolon = true;
        _firstparam = par;
    }

    /// <summary>Extends the guideline at its end to <paramref name="par"/>.</summary>
    internal void SetLastParameter(double par)
    {
        // OCCT_DEBUG traces left out.
        _lastprolon = true;
        _lastparam = par;
    }

    /// <summary>The total length of the edges before the edge <paramref name="indexSpine"/>.</summary>
    public double FirstParameter(int indexSpine)
    {
        if (indexSpine == 1)
        {
            return 0.0;
        }

        return _abscissa!.Value(indexSpine - 1);
    }

    /// <summary>The total length of the edges up to the edge <paramref name="indexSpine"/> (included).</summary>
    public double LastParameter(int indexSpine)
    {
        return _abscissa!.Value(indexSpine);
    }

    /// <summary>The length of the edge <paramref name="indexSpine"/>.</summary>
    public double Length(int indexSpine)
    {
        if (indexSpine == 1)
        {
            return _abscissa!.Value(indexSpine);
        }

        return _abscissa!.Value(indexSpine) - _abscissa.Value(indexSpine - 1);
    }

    /// <summary>True if the chain is closed (its first status is <see cref="Closed"/>).</summary>
    public bool IsPeriodic
    {
        get
        {
            return (_firstState == State.Closed);
        }
    }

    /// <summary>The length of the closed chain; throws <see cref="FailureException"/> if it is not periodic.</summary>
    public double Period()
    {
        if (!IsPeriodic)
        {
            throw new FailureException("Non-periodic Spine");
        }

        return _abscissa!.Value(_abscissa.Upper());
    }

    /// <summary>The abscissa of the parameter <paramref name="u"/> on the current edge.</summary>
    internal double Absc(double u)
    {
        return Absc(u, _indexofcurve);
    }

    /// <summary>The abscissa of the parameter <paramref name="u"/> on the edge <paramref name="i"/>.</summary>
    internal double Absc(double u, int i)
    {
        if (_indexofcurve != i)
        {
            _indexofcurve = i;
            _curve.Initialize(ShapeCasts.Edge(_spine.Value(i)));
        }

        double l = FirstParameter(i);
        if (_spine.Value(i).Orientation == Reversed)
        {
            l += AbscissaPoint.Length(_curve, u, _curve.LastParameter());
        }
        else
        {
            l += AbscissaPoint.Length(_curve, _curve.FirstParameter(), u);
        }

        return l;
    }

    /// <summary>The parameter <paramref name="u"/>, on its edge, of the abscissa <paramref name="absC"/>.</summary>
    internal void Parameter(double absC, out double u, bool oriented = true)
    {
        int index;
        for (index = 1; index < _abscissa!.Length(); index++)
        {
            if (absC < _abscissa.Value(index))
            {
                break;
            }
        }

        Parameter(index, absC, out u, oriented);
    }

    /// <summary>The parameter <paramref name="u"/>, on the edge <paramref name="index"/>, of the abscissa
    /// <paramref name="absC"/>.</summary>
    internal void Parameter(int index, double absC, out double u, bool oriented = true)
    {
        if (index != _indexofcurve)
        {
            _indexofcurve = index;
            _curve.Initialize(ShapeCasts.Edge(_spine.Value(index)));
        }

        double l;
        Orientation or = _spine.Value(index).Orientation;
        if (or == Reversed)
        {
            l = _abscissa!.Value(_indexofcurve) - absC;
        }
        else if (_indexofcurve == 1)
        {
            l = absC;
        }
        else
        {
            l = absC - _abscissa!.Value(_indexofcurve - 1);
        }

        double t = l / Length(index);
        double uapp = (1.0 - t) * _curve.FirstParameter() + t * _curve.LastParameter();
        AbscissaPoint gcp = new(_curve, l, _curve.FirstParameter(), uapp);
        u = gcp.Parameter();
        if (or == Reversed && oriented)
        {
            u = (_curve.LastParameter() + _curve.FirstParameter()) - u;
        }
    }

    internal Point3D Value(double absC)
    {
        int index;
        double l = absC;

        Prepare(ref l, out index);

        if (index == -1)
        {
            Point3D pp = _firstori;
            Vector3D vp = _firsttgt;
            vp = vp.Multiplied(l);
            pp = pp.Translated(vp);
            return pp;
        }
        else if (index == (_abscissa!.Length() + 1))
        {
            Point3D pp = _lastori;
            Vector3D vp = _lasttgt;
            vp = vp.Multiplied(l);
            pp = pp.Translated(vp);
            return pp;
        }

        if (index != _indexofcurve)
        {
            _indexofcurve = index;
            _curve.Initialize(ShapeCasts.Edge(_spine.Value(index)));
        }

        double t = l / Length(index);
        double uapp = (1.0 - t) * _curve.FirstParameter() + t * _curve.LastParameter();
        AbscissaPoint gcp = new(_curve, l, _curve.FirstParameter(), uapp);
        return _curve.Value(gcp.Parameter());
    }

    internal void D0(double absC, out Point3D p)
    {
        p = Value(absC);
    }

    /// <summary>The point and the unit tangent at the abscissa <paramref name="absC"/>.</summary>
    internal void D1(double absC, out Point3D p, out Vector3D v1)
    {
        int index;
        double l = absC;

        Prepare(ref l, out index);

        if (index == -1)
        {
            p = _firstori;
            v1 = _firsttgt;
            Vector3D vp = _firsttgt;
            vp = vp.Multiplied(l);
            p = p.Translated(vp);
        }
        else if (index == (_abscissa!.Length() + 1))
        {
            p = _lastori;
            v1 = _lasttgt;
            Vector3D vp = _lasttgt;
            vp = vp.Multiplied(l);
            p = p.Translated(vp);
        }
        else
        {
            if (index != _indexofcurve)
            {
                _indexofcurve = index;
                _curve.Initialize(ShapeCasts.Edge(_spine.Value(index)));
            }

            double t = l / Length(index);
            double uapp = (1.0 - t) * _curve.FirstParameter() + t * _curve.LastParameter();
            AbscissaPoint gcp = new(_curve, l, _curve.FirstParameter(), uapp);
            _curve.D1(gcp.Parameter(), out p, out v1);
            double d1 = 1.0 / v1.Magnitude;
            if (_spine.Value(index).Orientation == Reversed)
            {
                d1 = -d1;
            }

            v1 = v1.Multiplied(d1);
        }
    }

    internal void D2(double absC, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        int index;
        double l = absC;

        Prepare(ref l, out index);

        if (index == -1)
        {
            p = _firstori;
            v1 = _firsttgt;
            v2 = new Vector3D();
            v2 = v2.WithCoord(0.0, 0.0, 0.0);
            Vector3D vp = _firsttgt;
            vp = vp.Multiplied(l);
            p = p.Translated(vp);
        }
        else if (index == (_abscissa!.Length() + 1))
        {
            p = _lastori;
            v1 = _lasttgt;
            v2 = new Vector3D();
            v2 = v2.WithCoord(0.0, 0.0, 0.0);
            Vector3D vp = _lasttgt;
            vp = vp.Multiplied(l);
            p = p.Translated(vp);
        }
        else
        {
            if (index != _indexofcurve)
            {
                _indexofcurve = index;
                _curve.Initialize(ShapeCasts.Edge(_spine.Value(index)));
            }

            double t = l / Length(index);
            double uapp = (1.0 - t) * _curve.FirstParameter() + t * _curve.LastParameter();
            AbscissaPoint gcp = new(_curve, l, _curve.FirstParameter(), uapp);
            _curve.D2(gcp.Parameter(), out p, out v1, out v2);
            double n1 = v1.SquareMagnitude;
            double d2 = -(v1.Dot(v2)) * (1.0 / n1) * (1.0 / n1);
            v2 = v2.Multiplied(1.0 / n1);
            n1 = Math.Sqrt(n1);
            Vector3D va = v1.Multiplied(d2);
            v2 = v2.Added(va);
            double d1 = 1.0 / n1;
            if (_spine.Value(index).Orientation == Reversed)
            {
                d1 = -d1;
            }

            v1 = v1.Multiplied(d1);
        }
    }

    internal void SetCurrent(int index)
    {
        if (index != _indexofcurve)
        {
            _indexofcurve = index;
            _curve.Initialize(ShapeCasts.Edge(_spine.Value(_indexofcurve)));
        }
    }

    /// <summary>Sets the current curve to the edge <paramref name="index"/> and returns it.</summary>
    internal EdgeCurveAdaptor CurrentElementarySpine(int index)
    {
        if (index != _indexofcurve)
        {
            _indexofcurve = index;
            _curve.Initialize(ShapeCasts.Edge(_spine.Value(_indexofcurve)));
        }

        return _curve;
    }

    public int CurrentIndexOfElementarySpine()
    {
        return _indexofcurve;
    }

    /// <summary>The type of the current curve.</summary>
    public new CurveKind GetType()
    {
        return _curve.GetType();
    }

    /// <summary>The current curve as an oriented line starting at the edge's start.</summary>
    public Line3D Line()
    {
        Line3D ll = _curve.Line();
        if (_spine.Value(_indexofcurve).Orientation == Reversed)
        {
            ll = ll.Reversed();
            ll = ll.WithLocation(_curve.Value(_curve.LastParameter()));
        }
        else
        {
            ll = ll.WithLocation(_curve.Value(_curve.FirstParameter()));
        }

        return ll;
    }

    /// <summary>The current curve as an oriented circle starting at the edge's start.</summary>
    public Circle3D Circle()
    {
        Frame3D ac = _curve.Circle().Position;
        Direction3D dc = new Direction3D(new Vector3D(ac.Location, _curve.Value(_curve.FirstParameter())));
        Direction3D zz = ac.Direction;

        if (_spine.Value(_indexofcurve).Orientation == Reversed)
        {
            dc = new Direction3D(new Vector3D(ac.Location, _curve.Value(_curve.LastParameter())));
            zz = zz.Reversed();
        }

        Frame3D a = new Frame3D(ac.Location, zz, dc);
        return new Circle3D(a, _curve.Circle().Radius);
    }

    /// <summary>Whether the chain starts on a free boundary, on a break point, or is closed.</summary>
    public State FirstStatus()
    {
        return _firstState;
    }

    /// <summary>The state at the end of the chain.</summary>
    public State LastStatus()
    {
        return _lastState;
    }

    public State Status(bool isFirst)
    {
        if (isFirst)
        {
            return _firstState;
        }
        else
        {
            return _lastState;
        }
    }

    /// <summary>The type of concavity of the connection.</summary>
    public TypeOfConcavity GetTypeOfConcavity()
    {
        return _typeOfConcavity;
    }

    internal void SetStatus(State s, bool isFirst)
    {
        if (isFirst)
        {
            _firstState = s;
        }
        else
        {
            _lastState = s;
        }
    }

    internal void SetTypeOfConcavity(TypeOfConcavity type)
    {
        _typeOfConcavity = type;
    }

    /// <summary>True if the chain starts (or ends) on a tangency point.</summary>
    public bool IsTangencyExtremity(bool isFirst)
    {
        if (isFirst)
        {
            return _firstistgt;
        }
        else
        {
            return _lastistgt;
        }
    }

    internal void SetTangencyExtremity(bool isTangency, bool isFirst)
    {
        if (isFirst)
        {
            _firstistgt = isTangency;
        }
        else
        {
            _lastistgt = isTangency;
        }
    }

    /// <summary>The abscissa of the vertex <paramref name="v"/>, or -1 if it is not on the chain.</summary>
    public double Absc(in Vertex v)
    {
        Vertex d, f;
        Edge e;
        for (int i = 1; i <= _spine.Length(); i++)
        {
            e = ShapeCasts.Edge(_spine.Value(i));
            Exploration.Vertices(e, out d, out f);
            if (d.IsSame(v) && e.Orientation == Forward)
            {
                return FirstParameter(i);
            }

            if (d.IsSame(v) && e.Orientation == Reversed)
            {
                return LastParameter(i);
            }

            if (f.IsSame(v) && e.Orientation == Forward)
            {
                return LastParameter(i);
            }

            if (f.IsSame(v) && e.Orientation == Reversed)
            {
                return FirstParameter(i);
            }
        }

        return -1.0;
    }

    public Vertex FirstVertex()
    {
        Edge e = ShapeCasts.Edge(_spine.First());
        if (e.Orientation == Forward)
        {
            return Exploration.FirstVertex(e);
        }

        return Exploration.LastVertex(e);
    }

    public Vertex LastVertex()
    {
        Edge e = ShapeCasts.Edge(_spine.Last());
        if (e.Orientation == Forward)
        {
            return Exploration.LastVertex(e);
        }

        return Exploration.FirstVertex(e);
    }

    /// <summary>Extends the guideline at its start by its tangent at <paramref name="w"/>.</summary>
    internal void SetFirstTgt(double w)
    {
        if (IsPeriodic)
        {
            throw new FailureException("No extension by tangent on periodic contours");
        }

        // OCCT_DEBUG trace left out.

        // The flag is suspended if is already positioned to avoid
        // stopping d1
        _hasfirsttgt = false;
        D1(w, out _firstori, out _firsttgt);
        // and it is reset.
        _hasfirsttgt = true;
        _firsttgtpar = w;
    }

    /// <summary>Extends the guideline at its end by its tangent at <paramref name="w"/>.</summary>
    internal void SetLastTgt(double w)
    {
        if (IsPeriodic)
        {
            throw new FailureException("No extension by tangent periodic contours");
        }

        // OCCT_DEBUG trace left out.

        // The flag is suspended if is already positioned to avoid
        // stopping d1
        _haslasttgt = false;
        D1(w, out _lastori, out _lasttgt);
        // and it is reset.
        _haslasttgt = true;
        _lasttgtpar = w;
    }

    public bool HasFirstTgt
    {
        get
        {
            return _hasfirsttgt;
        }
    }

    public bool HasLastTgt
    {
        get
        {
            return _haslasttgt;
        }
    }

    /// <summary>Sets a parameter reference for the approximation.</summary>
    internal void SetReference(double w)
    {
        _hasref = true;
        double lll = _abscissa!.Value(_abscissa.Upper());
        if (IsPeriodic)
        {
            _valref = global::Kerf.Engine.Geometry.ElCLib.InPeriod(w, 0.0, lll);
        }
        else
        {
            _valref = w;
        }
    }

    /// <summary>Sets a parameter reference for the approximation at the middle of the edge <paramref name="i"/>.</summary>
    internal void SetReference(int i)
    {
        _hasref = true;
        if (i == 1)
        {
            _valref = _abscissa!.Value(1) * 0.5;
        }
        else
        {
            _valref = (_abscissa!.Value(i) + _abscissa.Value(i - 1)) * 0.5;
        }
    }

    /// <summary>The index of the edge at the abscissa <paramref name="w"/>, the next one at a junction when
    /// <paramref name="forward"/>.</summary>
    public int Index(double w, bool forward = true)
    {
        int ind, len = _abscissa!.Length();
        double par = w, last = _abscissa.Value(_abscissa.Upper());
        double f = 0.0, l = 0.0, t = Max(_tolesp, global::Kerf.Engine.Foundation.Precision.Confusion());

        if (IsPeriodic && Math.Abs(par) >= t && Math.Abs(par - last) >= t)
        {
            par = global::Kerf.Engine.Geometry.ElCLib.InPeriod(par, 0.0, last);
        }

        for (ind = 1; ind <= len; ind++)
        {
            f = l;
            l = _abscissa.Value(ind);
            if (par < l || ind == len)
            {
                break;
            }
        }

        if (forward && ind < len && Math.Abs(par - l) < t)
        {
            ind++;
        }
        else if (!forward && ind > 1 && Math.Abs(par - f) < t)
        {
            ind--;
        }
        else if (forward && IsPeriodic && ind == len && Math.Abs(par - l) < t)
        {
            ind = 1;
        }
        else if (!forward && IsPeriodic && ind == 1 && Math.Abs(par - f) < t)
        {
            ind = len;
        }

        return ind;
    }

    /// <summary>The index of the edge <paramref name="e"/>, or 0.</summary>
    public int Index(in Edge e)
    {
        for (int ie = 1; ie <= _spine.Length(); ie++)
        {
            if (e.IsSame(_spine.Value(ie)))
            {
                return ie;
            }
        }

        return 0;
    }

    internal void UnsetReference()
    {
        _hasref = false;
    }

    internal void SetErrorStatus(ErrorStatus state)
    {
        _errorstate = state;
    }

    /// <summary>The error status of the spine (renvoie le statut d'erreur concernant la spine).</summary>
    public ErrorStatus ErrorStatus()
    {
        return _errorstate;
    }

    /// <summary>The mode of the chamfers.</summary>
    public ChamfMode Mode()
    {
        return _mode;
    }

    /// <summary>The tolerance of the spine.</summary>
    public double GetTolesp()
    {
        return _tolesp;
    }

    // Moves the abscissa L to its edge Ind (-1 or NbEdges + 1 on the tangent extensions) and makes it local.
    private void Prepare(ref double l, out int ind)
    {
        double tol = Max(_tolesp, global::Kerf.Engine.Foundation.Precision.Confusion());
        double last = _abscissa!.Value(_abscissa.Upper());
        int len = _abscissa.Length();
        if (IsPeriodic && Math.Abs(l) >= tol && Math.Abs(l - last) >= tol)
        {
            l = global::Kerf.Engine.Geometry.ElCLib.InPeriod(l, 0.0, last);
        }

        if (_hasfirsttgt && (l <= _firsttgtpar))
        {
            if (_hasref && _valref >= l && Math.Abs(l - _firsttgtpar) <= tol)
            {
                ind = Index(l);
            }
            else
            {
                ind = -1;
                l -= _firsttgtpar;
            }
        }
        else if (l <= 0.0)
        {
            ind = 1;
        }
        else if (_haslasttgt && (l >= _lasttgtpar))
        {
            if (_hasref && _valref <= l && Math.Abs(l - _lasttgtpar) <= tol)
            {
                ind = Index(l);
            }
            else
            {
                ind = len + 1;
                l -= _lasttgtpar;
            }
        }
        else if (l >= last)
        {
            ind = len;
        }
        else
        {
            for (ind = 1; ind < len; ind++)
            {
                if (l < _abscissa.Value(ind))
                {
                    break;
                }
            }

            if (_hasref)
            {
                if (l >= _valref && ind != 1)
                {
                    if (Math.Abs(l - _abscissa.Value(ind - 1)) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        ind--;
                    }
                }
                else if (l <= _valref && ind != len)
                {
                    if (Math.Abs(l - _abscissa.Value(ind)) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        ind++;
                    }
                }
            }
        }

        if (ind >= 1 && ind <= len)
        {
            if (_spine.Value(ind).Orientation == Reversed)
            {
                l = _abscissa.Value(ind) - l;
            }
            else if (ind != 1)
            {
                l -= _abscissa.Value(ind - 1);
            }
        }
    }
}
