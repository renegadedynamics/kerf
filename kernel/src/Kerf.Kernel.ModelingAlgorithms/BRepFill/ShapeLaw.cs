// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ShapeLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ShapeLaw.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ShapeLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// A section law built on a vertex or a wire: constant (a GeomFill_UniformSection per edge) or scaled along the path by
/// a law function (a GeomFill_EvolvedSection per edge).
/// </summary>
/// <remarks>
/// The section's edges are the caller's, read only: their curves are reversed into new curves and trimmed into new
/// trimmed curves, as in OCCT. <see cref="D0"/> gives the caller's wire itself for a constant law and a frozen scaled
/// copy otherwise; <see cref="Vertex"/> the caller's vertex or a frozen scaled copy (decision 901).
/// </remarks>
public class ShapeLaw : SectionLaw
{
    protected bool _vertex;
    private readonly Shape _shape;
    private NCollection_HArray1<Shape>? _edges;
    private readonly LawsFunction? _theLaw;

    //=======================================================================
    // function : Create
    // purpose  : Process the case of Vertex by constructing a line
    //           with the vertex in the origin
    //=======================================================================

    /// <summary>A constant law on the vertex <paramref name="v"/> (a tiny segment along X through it).</summary>
    public ShapeLaw(in Vertex v, bool build = true)
    {
        _vertex = true;
        _shape = v;
        _theLaw = null;
        _uclosed = false;
        _vclosed = true; // constant law
        _edges = new NCollection_HArray1<Shape>(1, 1);
        _edges.SetValue(1, v);

        if (build)
        {
            _laws = new NCollection_HArray1<SurfaceSectionLaw>(1, 1);
            //    gp_Pnt Origine;
            Direction3D d = new(Direction3D.D.X); // Following the normal
            LineCurve3D l = new(Tool.Pnt(v), d);
            double last = 2 * Tool.Tolerance(v) + global::Kerf.Engine.Foundation.Precision.PConfusion();
            TrimmedCurve3D tc = new(l, 0, last);

            _laws.ChangeValue(1) = new UniformSection(tc);
        }
        _done = true;
    }

    //=================================================================================================

    /// <summary>A constant law on the wire <paramref name="w"/>.</summary>
    public ShapeLaw(in Wire w, bool build = true)
    {
        _vertex = false;
        _shape = w;
        _theLaw = null;
        Init(build);
        _done = true;
    }

    //=================================================================================================

    /// <summary>A law on the wire <paramref name="w"/> scaled about the origin by <paramref name="l"/>.</summary>
    public ShapeLaw(in Wire w, LawsFunction l, bool build = true)
    {
        _vertex = false;
        _shape = w;
        _theLaw = l;
        Init(build);
        _done = true;
    }

    //=======================================================================
    // function : Init
    // purpose  : Case of the wire : Create a table of GeomFill_SectionLaw
    //=======================================================================
    private void Init(bool build)
    {
        _vclosed = true;
        WireExplorer wexp = new();
        Edge e;
        int nbEdge, ii;
        double first, last;
        Wire w;
        w = ShapeCasts.Wire(_shape);

        for (nbEdge = 0, wexp.Init(w); wexp.More(); wexp.Next())
        {
            e = wexp.Current();
            if (!e.IsNull && !Tool.Degenerated(e))
            {
                Curve3D? c = Tool.Curve(e, out first, out last);
                if (c is not null)
                {
                    nbEdge++;
                }
            }
        }

        _laws = new NCollection_HArray1<SurfaceSectionLaw>(1, nbEdge);
        _edges = new NCollection_HArray1<Shape>(1, nbEdge);

        ii = 1;

        for (wexp.Init(w); wexp.More(); wexp.Next())
        {
            e = wexp.Current();
            if (!e.IsNull && !Tool.Degenerated(wexp.Current()))
            {
                Curve3D? c = Tool.Curve(e, out first, out last);
                if (c is not null)
                {
                    _edges.SetValue(ii, e);
                    _indices[e] = ii;
                    if (build)
                    {
                        // occ::handle<Geom_Curve> C = BRep_Tool::Curve(E,First,Last);
                        if (e.Orientation() == Reversed)
                        {
                            double aux;
                            Curve3D cBis;
                            cBis = c.Reversed(); // To avoid the deterioration of the topology
                            aux = c.ReversedParameter(first);
                            first = c.ReversedParameter(last);
                            last = aux;
                            c = cBis;
                        }

                        bool isClosed = Tool.IsClosed(e);
                        if (isClosed && Math.Abs(c.FirstParameter() - first) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            isClosed = false; // trimmed curve differs
                        }

                        if ((ii > 1) || !isClosed)
                        { // Trim C
                            TrimmedCurve3D tc = new(c, first, last);
                            c = tc;
                        }
                        // otherwise preserve the integrity of the curve
                        if (_theLaw is null)
                        {
                            _laws.ChangeValue(ii) = new UniformSection(c);
                        }
                        else
                        {
                            _laws.ChangeValue(ii) = new EvolvedSection(c, _theLaw);
                        }
                    }
                    ii++;
                }
            }
        }

        //  std::cout << "new law" << std::endl;

        //  Is the law closed by U ?
        _uclosed = w.Closed();
        if (!_uclosed)
        {
            // if not sure about the flag, make check
            Edge edge1, edge2;
            Vertex v1, v2;
            edge1 = ShapeCasts.Edge(_edges.Value(_edges.Length()));
            edge2 = ShapeCasts.Edge(_edges.Value(1));

            if (edge1.Orientation() == Reversed)
            {
                v1 = Exploration.FirstVertex(edge1);
            }
            else
            {
                v1 = Exploration.LastVertex(edge1);
            }

            if (edge2.Orientation() == Reversed)
            {
                v2 = Exploration.LastVertex(edge2);
            }
            else
            {
                v2 = Exploration.FirstVertex(edge2);
            }
            if (v1.IsSame(v2))
            {
                _uclosed = true;
            }
            else
            {
                EdgeCurveAdaptor curve1 = new(edge1);
                EdgeCurveAdaptor curve2 = new(edge2);
                double u1 = Tool.Parameter(v1, edge1);
                double u2 = Tool.Parameter(v2, edge2);
                double eps = Tool.Tolerance(v2) + Tool.Tolerance(v1);

                _uclosed = curve1.Value(u1).IsEqual(curve2.Value(u2), eps);
            }
        }
    }

    //=================================================================================================

    /// <summary>True if the input shape is a vertex.</summary>
    public override bool IsVertex => _vertex;

    //=================================================================================================

    /// <summary>True if the law is constant (no law function).</summary>
    public override bool IsConstant => _theLaw is null;

    //=================================================================================================

    /// <summary>
    /// The section's vertex of rank <paramref name="index"/> (in [1, NbLaw() + 1]): the caller's vertex for a constant
    /// law, a frozen copy scaled by the law at <paramref name="param"/> otherwise.
    /// </summary>
    public override Vertex Vertex(int index, double param)
    {
        Edge e;
        Vertex v = new();
        if (index <= _edges!.Length())
        {
            e = ShapeCasts.Edge(_edges.Value(index));
            if (e.Orientation() == Reversed)
            {
                v = Exploration.LastVertex(e);
            }
            else
            {
                v = Exploration.FirstVertex(e);
            }
        }
        else if (index == _edges.Length() + 1)
        {
            e = ShapeCasts.Edge(_edges.Value(index - 1));
            if (e.Orientation() == Reversed)
            {
                v = Exploration.FirstVertex(e);
            }
            else
            {
                v = Exploration.LastVertex(e);
            }
        }

        if (_theLaw is not null)
        {
            Transform3D t = new Transform3D().WithScale(new Point3D(0, 0, 0), _theLaw.Value(param));
            // TopLoc_Location L(T);
            // V.Move(L);
            v = ShapeCasts.Vertex(new Transform(v, t).Shape());
        }
        return v;
    }

    //=======================================================================
    // function : VertexTol
    // purpose  : Evaluate the hole between 2 edges of the section
    //=======================================================================

    /// <summary>The gap between the end of the edge of rank <paramref name="index"/> and the start of the next one.</summary>
    public override double VertexTol(int index, double param)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int i1, i2;
        if ((index == 0) || (index == _edges!.Length()))
        {
            if (!_uclosed)
            {
                return tol; // The least possible error
            }
            i1 = _edges!.Length();
            i2 = 1;
        }
        else
        {
            i1 = index;
            i2 = i1 + 1;
        }

        SurfaceSectionLaw loi;
        int nbPoles, nbKnots, degree;
        NCollection_HArray1<Point3D> poles;
        NCollection_HArray1<double> knots, weigth;
        NCollection_HArray1<int> mults;
        BSplineCurve3D bs;
        Point3D pFirst;

        loi = _laws!.Value(i1);
        loi.SectionShape(out nbPoles, out nbKnots, out degree);
        poles = new NCollection_HArray1<Point3D>(1, nbPoles);
        weigth = new NCollection_HArray1<double>(1, nbPoles);
        loi.D0(param, poles.ChangeArray1(), weigth.ChangeArray1());
        knots = new NCollection_HArray1<double>(1, nbKnots);
        loi.Knots(knots.ChangeArray1());
        mults = new NCollection_HArray1<int>(1, nbKnots);
        loi.Mults(mults.ChangeArray1());
        bs = new BSplineCurve3D(poles.Array1(),
                                   weigth.Array1(),
                                   knots.Array1(),
                                   mults.Array1(),
                                   degree,
                                   loi.IsUPeriodic);
        pFirst = bs.Value(knots.Value(knots.Length()));

        loi = _laws.Value(i2);
        loi.SectionShape(out nbPoles, out nbKnots, out degree);
        poles = new NCollection_HArray1<Point3D>(1, nbPoles);
        weigth = new NCollection_HArray1<double>(1, nbPoles);
        loi.D0(param, poles.ChangeArray1(), weigth.ChangeArray1());
        knots = new NCollection_HArray1<double>(1, nbKnots);
        loi.Knots(knots.ChangeArray1());
        mults = new NCollection_HArray1<int>(1, nbKnots);
        loi.Mults(mults.ChangeArray1());
        bs = new BSplineCurve3D(poles.Array1(),
                                   weigth.Array1(),
                                   knots.Array1(),
                                   mults.Array1(),
                                   degree,
                                   loi.IsUPeriodic);
        tol += pFirst.Distance(bs.Value(knots.Value(1)));
        return tol;
    }

    //=================================================================================================

    /// <summary>
    /// The law on the section's edges concatenated into one B-spline (the law's own elementary law when there is one
    /// edge; null for a vertex law of several edges, which can't happen).
    /// </summary>
    public override SurfaceSectionLaw? ConcatenedLaw()
    {
        SurfaceSectionLaw? law = null;
        if (_laws!.Length() == 1)
        {
            return _laws.Value(1);
        }
        else
        {
            Wire w;
            Vertex v = new();
            w = ShapeCasts.Wire(_shape);
            if (!w.IsNull)
            {
                //  Concatenation of edges
                int ii;
                double epsV;
                bool bof;
                Curve3D composite;
                TrimmedCurve3D tc;
                composite = Tool.Curve(Edge(1), out double f, out double l)!;
                tc = new TrimmedCurve3D(composite, f, l);
                CompCurveToBSplineCurve concat = new(tc);

                for (ii = 2, bof = true; ii <= _edges!.Length() && bof; ii++)
                {
                    composite = Tool.Curve(Edge(ii), out f, out l)!;
                    tc = new TrimmedCurve3D(composite, f, l);
                    bof = Exploration.CommonVertex(Edge(ii - 1), Edge(ii), ref v);
                    if (bof)
                    {
                        epsV = Tool.Tolerance(v);
                    }
                    else
                    {
                        epsV = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
                    }
                    bof = concat.Add(tc, epsV, true, false, 20);
                    if (!bof)
                    {
                        bof = concat.Add(tc, 200 * epsV, true, false, 20);
                    }
                }
                composite = concat.InternalBSplineCurve()!;

                if (_theLaw is null)
                {
                    law = new UniformSection(composite);
                }
                else
                {
                    law = new EvolvedSection(composite, _theLaw);
                }
            }
        }
        return law;
    }

    //=================================================================================================

    /// <summary>The continuity of the section at the vertex between its edges of ranks <paramref name="index"/> and next.</summary>
    public override Continuity Continuity(int index, double tolAngular)
    {

        Edge edge1, edge2;
        if ((index == 0) || (index == _edges!.Length()))
        {
            if (!_uclosed)
            {
                return C0; // The least possible error
            }

            edge1 = ShapeCasts.Edge(_edges!.Value(_edges.Length()));
            edge2 = ShapeCasts.Edge(_edges.Value(1));
        }
        else
        {
            edge1 = ShapeCasts.Edge(_edges.Value(index));
            edge2 = ShapeCasts.Edge(_edges.Value(index + 1));
        }

        Vertex v1, v2; // common vertex
        Exploration.Vertices(edge1, out Vertex vv1, out Vertex vv2);
        Exploration.Vertices(edge2, out Vertex vv3, out Vertex vv4);
        if (vv1.IsSame(vv3))
        {
            v1 = vv1;
            v2 = vv3;
        }
        else if (vv1.IsSame(vv4))
        {
            v1 = vv1;
            v2 = vv4;
        }
        else if (vv2.IsSame(vv3))
        {
            v1 = vv2;
            v2 = vv3;
        }
        else
        {
            v1 = vv2;
            v2 = vv4;
        }

        double u1 = Tool.Parameter(v1, edge1);
        double u2 = Tool.Parameter(v2, edge2);
        EdgeCurveAdaptor curve1 = new(edge1);
        EdgeCurveAdaptor curve2 = new(edge2);
        double eps = Tool.Tolerance(v2) + Tool.Tolerance(v1);
        Continuity cont;
        cont = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(curve1, curve2, u1, u2, eps, tolAngular);

        return cont;
    }

    //=================================================================================================

    /// <summary>
    /// The section at the law's parameter <paramref name="u"/>: the caller's shape for a constant law, a frozen copy
    /// scaled about the origin otherwise.
    /// </summary>
    public override void D0(double u, ref Shape s)
    {
        s = _shape;
        if (_theLaw is not null)
        {
            Transform3D t = new Transform3D().WithScale(new Point3D(0, 0, 0), _theLaw.Value(u));
            // TopLoc_Location L(T);
            // S.Move(L);
            s = new Transform(s, t).Shape();
        }
    }

    /// <summary>The section's edge of rank <paramref name="index"/>.</summary>
    public Edge Edge(int index) => ShapeCasts.Edge(_edges!.Value(index));
}
