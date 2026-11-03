// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Elements.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Elements.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// The elements of a 2D hatcher, indexed by integers, and the face explorer that
/// <see cref="Classifier2D"/> runs over them: one wire holding every element, and half-lines from the point
/// classified through sample points of the elements.
/// </summary>
/// <remarks>
/// A working object (decision 21). The map is an <see cref="NCollection_IntDataMap{TValue}"/>, so the edges are
/// explored in OCCT's order (decisions 543 and 666).
/// </remarks>
public sealed class Elements2D : IFaceExplorer<CurveAdaptor2D>
{
    //  Modified by skv - Fri Jul 14 17:03:47 2006 OCC12627
    private const double Probing_Start = 0.123;
    private const double Probing_End = 0.8;
    private const double Probing_Step = 0.2111;

    private readonly NCollection_IntDataMap<Element2D> _map = new();
    private readonly NCollection_IntDataMap<Element2D>.Iterator _iter = new();
    private int _numWire;
#pragma warning disable CS0414 // OCCT resets the edge counter without reading it
    private int _numEdge;
#pragma warning restore CS0414
    private int _curEdge;
    private double _curEdgePar;

    /// <summary>An empty set of elements.</summary>
    public Elements2D()
    {
        _numWire = 0;
        _numEdge = 0;
        _curEdge = 1;
        _curEdgePar = Probing_Start;
    }

    /// <summary>
    /// OCCT's copy constructor, which copies nothing: the new set is empty, its current edge 0 and its probing
    /// parameter 0 (OCCT's "magic constructor"; occt-issues.md).
    /// </summary>
    public Elements2D(Elements2D other)
    {
        _ = other; // OCCT ignores its argument: the copy is empty (occt-issues.md)
        _numWire = 0;
        _numEdge = 0;
        _curEdge = 0;
        _curEdgePar = 0.0;
    }

    /// <summary>Removes all the elements.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>Binds the element <paramref name="i"/> to the index <paramref name="k"/>; false if K was bound (it is rebound).</summary>
    public bool Bind(int k, in Element2D i) => _map.Bind(k, i);

    /// <summary>True if the index <paramref name="k"/> is bound.</summary>
    public bool IsBound(int k) => _map.IsBound(k);

    /// <summary>Unbinds the index <paramref name="k"/>; false if it wasn't bound.</summary>
    public bool UnBind(int k) => _map.UnBind(k);

    /// <summary>The element bound to <paramref name="k"/>.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="k"/> is not bound.</exception>
    public Element2D Find(int k) => _map.Find(k);

    /// <summary>The element bound to <paramref name="k"/>, for writing.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="k"/> is not bound.</exception>
    internal ref Element2D ChangeFind(int k) => ref _map.ChangeFind(k);

    /// <summary>OCCT's <c>operator()</c>: the element bound to <paramref name="k"/>.</summary>
    public Element2D this[int k] => Find(k);

    /// <summary>Accepts every point.</summary>
    public bool CheckPoint(ref Point2D p) => true;

    /// <summary>Rejects no point.</summary>
    public bool Reject(in Point2D p) => false;

    /// <summary>The first half-line from <paramref name="p"/>: through the first suitable sample point of an element.</summary>
    public bool Segment(in Point2D p, out Line2D l, out double par)
    {
        _curEdge = 1;
        _curEdgePar = Probing_Start;
        return OtherSegment(p, out l, out par);
    }

    /// <summary>
    /// The next half-line from <paramref name="p"/>: through the next sample point of the elements, not too close to
    /// tangent to the element and passing through neither end of it; false when there is none.
    /// </summary>
    public bool OtherSegment(in Point2D p, out Line2D l, out double par)
    {
        NCollection_IntDataMap<Element2D>.Iterator itertemp = new();
        int i;
        par = 0.0;
        for (itertemp.Initialize(_map), i = 1; itertemp.More(); itertemp.Next(), i++)
        {
            if (i < _curEdge)
            {
                continue;
            }

            ref Element2D item = ref _map.ChangeFind(itertemp.Key());
            CurveAdaptor2D e = item.ChangeCurve();
            Orientation or = item.Orientation();
            if (or == Forward || or == Reversed)
            {
                double fPar = e.FirstParameter(), lPar = e.LastParameter();
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(fPar))
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lPar))
                    {
                        fPar = -1.0;
                        lPar = 1.0;
                    }
                    else
                    {
                        fPar = lPar - 1.0;
                    }
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lPar))
                {
                    lPar = fPar + 1.0;
                }

                for (; _curEdgePar < Probing_End; _curEdgePar += Probing_Step)
                {
                    double param = _curEdgePar * fPar + (1.0 - _curEdgePar) * lPar;
                    e.D1(param, out Point2D pOnC, out Vector2D tanVec);
                    Vector2D linVec = new(p, pOnC);
                    par = linVec.SquareMagnitude();
                    if (par > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                    {
                        Direction2D linDir = new(linVec);
                        double tanMod = tanVec.SquareMagnitude();
                        if (tanMod < global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                        {
                            continue;
                        }

                        tanVec /= Math.Sqrt(tanMod);
                        double sinA = tanVec.Crossed(new Vector2D(linDir));
                        if (Math.Abs(sinA) < 0.001)
                        {
                            // too small angle - line and edge may be considered
                            // as tangent which is bad for classifier
                            if (_curEdgePar + Probing_Step < Probing_End)
                            {
                                continue;
                            }
                        }

                        l = new Line2D(p, linDir);

                        pOnC = e.Value(fPar);
                        if (l.SquareDistance(pOnC) > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                        {
                            pOnC = e.Value(lPar);
                            if (l.SquareDistance(pOnC) > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                            {
                                _curEdgePar += Probing_Step;
                                if (_curEdgePar >= Probing_End)
                                {
                                    _curEdge++;
                                    _curEdgePar = Probing_Start;
                                }
                                par = Math.Sqrt(par);
                                return true;
                            }
                        }
                    }
                }
            }
            _curEdge++;
            _curEdgePar = Probing_Start;
        }

        par = RealLast();
        l = new Line2D(p, new Direction2D(Direction2D.D.X));
        return false;
    }

    /// <summary>Starts the exploration of the wires (there is one).</summary>
    public void InitWires()
    {
        _numWire = 0;
    }

    /// <summary>True while there is a wire to explore.</summary>
    public bool MoreWires() => (_numWire == 0);

    /// <summary>Moves to the next wire.</summary>
    public void NextWire()
    {
        _numWire++;
    }

    /// <summary>Rejects no wire.</summary>
    public bool RejectWire(in Line2D l, double par) => false;

    /// <summary>Starts the exploration of the edges (the elements, in OCCT's map order).</summary>
    public void InitEdges()
    {
        _numEdge = 0;
        _iter.Initialize(_map);
    }

    /// <summary>True while there is an edge to explore.</summary>
    public bool MoreEdges() => (_iter.More());

    /// <summary>Moves to the next edge.</summary>
    public void NextEdge()
    {
        _iter.Next();
    }

    /// <summary>Rejects no edge.</summary>
    public bool RejectEdge(in Line2D l, double par) => false;

    /// <summary>
    /// The current edge: a shallow copy of the element's curve (OCCT copies it into <paramref name="e"/>), and its
    /// orientation.
    /// </summary>
    public void CurrentEdge(ref CurveAdaptor2D e, out Orientation or)
    {
        Element2D item = _map.Find(_iter.Key());
        e = (CurveAdaptor2D)item.Curve().ShallowCopy();
        or = item.Orientation();
    }
}
