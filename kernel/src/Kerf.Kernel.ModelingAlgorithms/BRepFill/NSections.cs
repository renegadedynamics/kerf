// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_NSections.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_NSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
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
/// A section law through N wires (sections) with the same number of edges, placed at given parameters along the path:
/// one GeomFill_NSections per edge, cut from a B-spline surface approximated through all the sections.
/// </summary>
/// <remarks>
/// The sections' edges are the caller's, read only: their curves are converted, reversed and trimmed into new curves,
/// as in OCCT (and GeomFill_NSections reparametrises its own copy of a B-spline section, decision 838). The elementary
/// laws share the law's approximated surface, which stays unfrozen: the law reads it through
/// <c>GeomFill_NSections.InternalBSplineSurface</c> (decision 902). <see cref="D0"/> and <see cref="Vertex"/> hand
/// out new frozen shapes.
/// </remarks>
internal sealed class NSections : SectionLaw
{
    private readonly double _vFirst;
    private readonly double _vLast;
    private readonly NCollection_Sequence<Shape> _shapes = new();
    private readonly NCollection_Sequence<Transform3D> _trsfs = new();
    private readonly NCollection_Sequence<double> _params = new();
    private NCollection_HArray2<Shape>? _edges;
    private BSplineSurface? _surface;

    //=======================================================================
    // function : EdgeToBSpline
    // purpose  : auxiliary -- get curve from edge and convert it to bspline
    //           parameterized from 0 to 1
    //=======================================================================

    // NOTE: this code duplicates the same function in BRepOffsetAPI_ThruSections.cxx
    private static BSplineCurve3D EdgeToBSpline(in Edge edge)
    {
        BSplineCurve3D? bsCurve = null;
        if (Tool.Degenerated(edge))
        {
            // degenerated edge : construction of a point curve
            NCollection_Array1<double> knots = new(1, 2);
            knots[1] = 0.0;
            knots[2] = 1.0;

            NCollection_Array1<int> mults = new(1, 2);
            mults[1] = 2;
            mults[2] = 2;

            NCollection_Array1<Point3D> poles = new(1, 2);
            Exploration.Vertices(edge, out Vertex vl, out Vertex vf);
            poles[1] = Tool.Pnt(vf);
            poles[2] = Tool.Pnt(vl);

            bsCurve = new BSplineCurve3D(poles, knots, mults, 1);
        }
        else
        {
            // get the curve of the edge
            Curve3D curve = Tool.Curve(edge, out Placement loc, out double first, out double last)!;

            // convert its part used by edge to bspline; note that if edge curve is bspline,
            // approximation or conversion made via trimmed curve is still needed -- it will copy it,
            // segment as appropriate, and remove periodicity if it is periodic (deadly for approximator)
            TrimmedCurve3D trimCurve = new(curve, first, last);

            Curve3D curveTemp = trimCurve; // to avoid ambiguity
            ApproxCurve appr = new(curveTemp, global::Kerf.Engine.Foundation.Precision.Confusion(), C1, 16, 14);
            if (appr.HasResult)
            {
                bsCurve = appr.InternalCurve();
            }

            if (bsCurve is null)
            {
                bsCurve = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(trimCurve);
            }

            // apply transformation if needed
            if (!loc.IsIdentity)
            {
                bsCurve.Transform(loc.Transformation);
            }

            // reparameterize to [0,1]
            NCollection_Array1<double> knots = new(bsCurve.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, knots);
            bsCurve.SetKnots(knots);
        }

        // reverse curve if edge is reversed
        if (edge.Orientation == Reversed)
        {
            bsCurve.Reverse();
        }

        return bsCurve;
    }

    //=================================================================================================

    private static BSplineSurface Totalsurf(NCollection_Array2<Shape> shapes,
                                                 int nbSects,
                                                 int nbEdges,
                                                 NCollection_Sequence<double> @params,
                                                 bool w1Point,
                                                 bool w2Point,
                                                 bool uClosed,
                                                 bool vClosed,
                                                 double myPres3d)
    {
        int i, j, jdeb = 1, jfin = nbSects;
        Edge edge;

        SectionGenerator section = new();
        BSplineSurface surface;
        BSplineCurve3D? bs, bs1 = null;

        if (w1Point)
        {
            jdeb++;
            edge = ShapeCasts.Edge(shapes.Value(1, 1));
            Exploration.Vertices(edge, out Vertex vl, out Vertex vf);
            NCollection_Array1<Point3D> extremities = new(1, 2);
            extremities[1] = Tool.Pnt(vf);
            extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> bounds = new(1, 2);
            bounds[1] = 0.0;
            bounds[2] = 1.0;
            NCollection_Array1<int> mult = new(1, 2);
            mult[1] = 2;
            mult[2] = 2;
            BSplineCurve3D bsPoint = new(extremities, bounds, mult, 1);
            section.AddCurve(bsPoint);
        }

        if (w2Point)
        {
            jfin--;
        }

        for (j = jdeb; j <= jfin; j++)
        {

            // case of looping sections
            if (j == jfin && vClosed)
            {
                section.AddCurve(bs1!);
            }

            else
            {
                // read the first edge to initialise CompBS;
                Edge prevEdge = ShapeCasts.Edge(shapes.Value(1, j));
                BSplineCurve3D curvBS = EdgeToBSpline(prevEdge);

                // initialization
                CompCurveToBSplineCurve compBS = new(curvBS);

                for (i = 2; i <= nbEdges; i++)
                {
                    // read the edge
                    Edge nextEdge = ShapeCasts.Edge(shapes.Value(i, j));
                    curvBS = EdgeToBSpline(nextEdge);

                    // concatenation
                    Vertex comV = new();
                    double epsV;
                    bool bof = Exploration.CommonVertex(prevEdge, nextEdge, ref comV);
                    if (bof)
                    {
                        epsV = Tool.Tolerance(comV);
                    }
                    else
                    {
                        epsV = global::Kerf.Engine.Foundation.Precision.Confusion();
                    }
                    bof = compBS.Add(curvBS, epsV, true, false, 1);
                    if (!bof)
                    {
                        bof = compBS.Add(curvBS, 200 * epsV, true, false, 1);
                    }

                    // remember previous edge
                    prevEdge = nextEdge;
                }

                // return the final section
                bs = compBS.InternalBSplineCurve();
                section.AddCurve(bs!);

                // case of looping sections
                if (j == jdeb && vClosed)
                {
                    bs1 = bs;
                }
            }
        }

        if (w2Point)
        {
            edge = ShapeCasts.Edge(shapes.Value(nbEdges, nbSects));
            Exploration.Vertices(edge, out Vertex vl, out Vertex vf);
            NCollection_Array1<Point3D> extremities = new(1, 2);
            extremities[1] = Tool.Pnt(vf);
            extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> bounds = new(1, 2);
            bounds[1] = 0.0;
            bounds[2] = 1.0;
            NCollection_Array1<int> mult = new(1, 2);
            mult[1] = 2;
            mult[2] = 2;
            BSplineCurve3D bsPoint = new(extremities, bounds, mult, 1);
            section.AddCurve(bsPoint);
        }

        NCollection_HArray1<double> hPar = new(1, @params.Length());
        for (i = 1; i <= @params.Length(); i++)
        {
            hPar.SetValue(i, @params[i]);
        }
        section.SetParam(hPar);
        section.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        SurfaceLine line = new(nbSects);
        int nbIt = 0, degmin = 2, degmax = 6;
        bool knownP = true;
        SurfaceAppSurf approx = new(degmin, degmax, myPres3d, myPres3d, nbIt, knownP);
        bool spApprox = true;
        approx.Perform(line, section, spApprox);
        bool uperiodic = uClosed;
        bool vperiodic = vClosed;
        int nup = approx.SurfPoles().ColLength(), nvp = approx.SurfPoles().RowLength();
        NCollection_Array1<int> umults = new(1, approx.SurfUKnots().Length());
        umults.Assign(approx.SurfUMults());
        NCollection_Array1<int> vmults = new(1, approx.SurfVKnots().Length());
        vmults.Assign(approx.SurfVMults());

        if (uperiodic)
        {
            int nbuk = approx.SurfUKnots().Length();
            umults[1]--;
            umults[nbuk]--;
            nup--;
        }

        if (vperiodic)
        {
            int nbvk = approx.SurfVKnots().Length();
            vmults[1]--;
            vmults[nbvk]--;
            nvp--;
        }

        NCollection_Array2<Point3D> poles = new(1, nup, 1, nvp);
        NCollection_Array2<double> weights = new(1, nup, 1, nvp);
        for (j = 1; j <= nvp; j++)
        {
            for (i = 1; i <= nup; i++)
            {
                poles[i, j] = approx.SurfPoles()[i, j];
                weights[i, j] = approx.SurfWeights()[i, j];
            }
        }

        // To create non-rational surface if possible
        double tolEps = 1.0e-13;
        bool vrational = false, urational = false;
        for (j = 1; j <= weights.UpperCol(); j++)
        {
            if (!vrational)
            {
                for (i = 1; i <= weights.UpperRow() - 1; i++)
                {
                    // double signeddelta = weights(i,j) - weights(i+1,j);
                    double delta = Math.Abs(weights[i, j] - weights[i + 1, j]);
                    //	  double eps = Epsilon( std::abs(weights(i,j)) );
                    if (delta > tolEps /* || delta > 3.*eps*/)
                    {
                        vrational = true;
                        break;
                    }
                }
            }
        }
        for (i = 1; i <= weights.UpperRow(); i++)
        {
            if (!urational)
            {
                for (j = 1; j <= weights.UpperCol() - 1; j++)
                {
                    // double signeddelta = weights(i,j) - weights(i,j+1);
                    double delta = Math.Abs(weights[i, j] - weights[i, j + 1]);
                    //	  double eps = Epsilon( std::abs(weights(i,j)) );
                    if (delta > tolEps /* || delta > 3.*eps*/)
                    {
                        urational = true;
                        break;
                    }
                }
            }
        }
        if (!vrational && !urational)
        {
            double theWeight = weights[1, 1];
            for (i = 1; i <= weights.UpperRow(); i++)
            {
                for (j = 1; j <= weights.UpperCol(); j++)
                {
                    weights[i, j] = theWeight;
                }
            }
        }

        surface = new BSplineSurface(poles,
                                          weights,
                                          approx.SurfUKnots(),
                                          approx.SurfVKnots(),
                                          umults,
                                          vmults,
                                          approx.UDegree(),
                                          approx.VDegree(),
                                          uperiodic,
                                          vperiodic);
        return surface;
    }

    //=================================================================================================

    /// <summary>The law through the wires or vertices <paramref name="s"/>, at the parameters 0, 1, 2, ...</summary>
    public NSections(NCollection_Sequence<Shape> s, bool build = true)

    {
        _shapes.Assign(s);
        _vFirst = 0.0;
        _vLast = 1.0;
        NCollection_Sequence<double> par = new();
        par.Clear();
        for (int i = 1; i <= s.Length(); i++)
        {
            par.Append(i - 1);
        }
        _params.Assign(par);
        Init(par, build);
        _done = true;
    }

    //=======================================================================
    // function : Create
    // purpose  : WSeq + Param
    //=======================================================================

    /// <summary>
    /// The law through the sections <paramref name="s"/> placed by <paramref name="transformations"/> at the increasing
    /// parameters <paramref name="p"/>, on the path's parameter range [<paramref name="vf"/>, <paramref name="vl"/>].
    /// Not done if the parameters don't increase.
    /// </summary>
    public NSections(NCollection_Sequence<Shape> s,
                              NCollection_Sequence<Transform3D> transformations,
                              NCollection_Sequence<double> p,
                              double vf,
                              double vl,
                              bool build = true)

    {
        bool ok = true;
        for (int iseq = 1; iseq < p.Length(); iseq++)
        {
            ok = ok && (p.Value(iseq) < p.Value(iseq + 1));
        }
        if (ok)
        {
            _params.Assign(p);
            _shapes.Assign(s);
            _trsfs.Assign(transformations);
            _vFirst = vf;
            _vLast = vl;
            Init(p, build);
            _done = true;
        }
        else
        {
            _done = false;
        }
    }

    //=======================================================================
    // function : Init
    // purpose  : Create a table of GeomFill_SectionLaw
    //=======================================================================
    private void Init(NCollection_Sequence<double> p, bool build)
    {
        WireExplorer wexp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        Edge e;
        int ii, nbEdge, jj, nbSects = p.Length();
        int ideb = 1, ifin = nbSects;
        bool wClosed, w1Point = true, w2Point = true;
        Wire w;

        // Check if the start and end wires are punctual
        w = ShapeCasts.Wire(_shapes[1]);
        for (wexp.Init(w); wexp.More(); wexp.Next())
        {
            //    w1Point = w1Point && B.Degenerated(wexp.Current());
            w1Point = w1Point && Tool.Degenerated(wexp.Current());
        }
        if (w1Point)
        {
            ideb++;
        }
        w = ShapeCasts.Wire(_shapes[nbSects]);
        for (wexp.Init(w); wexp.More(); wexp.Next())
        {
            //    w2Point = w2Point && B.Degenerated(wexp.Current());
            w2Point = w2Point && Tool.Degenerated(wexp.Current());
        }
        if (w2Point)
        {
            ifin--;
        }

        // Check if the start and end wires are identical
        _vclosed = _shapes[1].IsSame(_shapes[nbSects]);

        // Count the number of non-degenerated edges
        w = ShapeCasts.Wire(_shapes[ideb]);
        for (nbEdge = 0, wexp.Init(w); wexp.More(); wexp.Next())
        {
            //    if (! B.Degenerated(wexp.Current())) NbEdge++;
            if (!Tool.Degenerated(wexp.Current()))
            {
                nbEdge++;
            }
        }

        _edges = new NCollection_HArray2<Shape>(1, nbEdge, 1, nbSects);

        // Fill tables
        _uclosed = true;
        for (jj = ideb; jj <= ifin; jj++)
        {

            w = ShapeCasts.Wire(_shapes[jj]);

            for (ii = 1, wexp.Init(w); ii <= nbEdge; wexp.Next(), ii++)
            {
                e = wexp.Current();

                //      if ( ! B.Degenerated(E)) {
                if (!Tool.Degenerated(e))
                {
                    _edges.SetValue(ii, jj, e);
                    if (e.Orientation == Forward)
                    {
                        _indices[e] = ii;
                    }
                    else
                    {
                        _indices[e] = -ii;
                    }
                }
            }

            // Is the law closed by U ?

            wClosed = w.IsClosed;
            if (!wClosed)
            {
                // if unsure about the flag, make check
                Edge edge1, edge2;
                Vertex v1, v2;
                edge1 = ShapeCasts.Edge(_edges.Value(nbEdge, jj));
                edge2 = ShapeCasts.Edge(_edges.Value(1, jj));

                if (edge1.Orientation == Reversed)
                {
                    v1 = Exploration.FirstVertex(edge1);
                }
                else
                {
                    v1 = Exploration.LastVertex(edge1);
                }
                if (edge2.Orientation == Reversed)
                {
                    v2 = Exploration.LastVertex(edge2);
                }
                else
                {
                    v2 = Exploration.FirstVertex(edge2);
                }
                if (v1.IsSame(v2))
                {
                    wClosed = true;
                }
                else
                {
                    EdgeCurveAdaptor curve1 = new(edge1);
                    EdgeCurveAdaptor curve2 = new(edge2);
                    double u1 = Tool.Parameter(v1, edge1);
                    double u2 = Tool.Parameter(v2, edge2);
                    double eps = Tool.Tolerance(v2) + Tool.Tolerance(v1);

                    wClosed = curve1.Value(u1).IsEqual(curve2.Value(u2), eps);
                }
            }
            if (!wClosed)
            {
                _uclosed = false;
            }
        }

        // point sections at end
        if (w1Point)
        {
            w = ShapeCasts.Wire(_shapes[1]);
            wexp.Init(w);
            e = wexp.Current();
            for (ii = 1; ii <= nbEdge; ii++)
            {
                _edges.SetValue(ii, 1, e);
            }
        }

        if (w2Point)
        {
            w = ShapeCasts.Wire(_shapes[nbSects]);
            wexp.Init(w);
            e = wexp.Current();
            for (ii = 1; ii <= nbEdge; ii++)
            {
                _edges.SetValue(ii, nbSects, e);
            }
        }

        _laws = new NCollection_HArray1<SurfaceSectionLaw>(1, nbEdge);

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        _surface = Totalsurf(_edges.Array2(),
                              _shapes.Length(),
                              nbEdge,
                              _params,
                              w1Point,
                              w2Point,
                              _uclosed,
                              _vclosed,
                              tol);

        // Increase the degree so that the position D2
        // on GeomFill_NSections could be correct
        // see comments in GeomFill_NSections
        if (_surface.VDegree < 2)
        {
            _surface.IncreaseDegree(_surface.UDegree, 2);
        }

        // Fill tables
        if (build)
        {
            for (ii = 1; ii <= nbEdge; ii++)
            {
                NCollection_Sequence<Curve3D> nc = new();
                nc.Clear();
                for (jj = 1; jj <= nbSects; jj++)
                {
                    e = ShapeCasts.Edge(_edges.Value(ii, jj));
                    Curve3D c;
                    //	if (B.Degenerated(E)) {
                    if (Tool.Degenerated(e))
                    {
                        Exploration.Vertices(e, out Vertex vl, out Vertex vf);
                        NCollection_Array1<Point3D> extremities = new(1, 2);
                        extremities[1] = Tool.Pnt(vf);
                        extremities[2] = Tool.Pnt(vl);
                        NCollection_Array1<double> bounds = new(1, 2);
                        bounds[1] = 0.0;
                        bounds[2] = 1.0;
                        NCollection_Array1<int> mult = new(1, 2);
                        mult[1] = 2;
                        mult[2] = 2;
                        BSplineCurve3D bsPoint = new(extremities, bounds, mult, 1);
                        c = bsPoint;
                    }
                    else
                    {
                        c = Tool.Curve(e, out double first, out double last)!;

                        if (e.Orientation == Reversed)
                        {
                            double aux;
                            Curve3D cBis;
                            cBis = c.Reversed(); // To avoid the spoiling of the topology
                            aux = c.ReversedParameter(first);
                            first = c.ReversedParameter(last);
                            last = aux;
                            c = cBis;
                        }
                        if ((ii > 1) || (!Tool.IsClosed(e)))
                        { // Cut C
                            TrimmedCurve3D tc = new(c, first, last);
                            c = tc;
                        }
                        //  otherwise preserve the integrity of the curve
                    }
                    nc.Append(c);
                }

                double ufirst = ii - 1;
                double ulast = ii;
                _laws.ChangeValue(ii) =
                    new SurfaceNSections(nc, _trsfs, _params, ufirst, ulast, _vFirst, _vLast, _surface);
            }
        }
    }

    //=================================================================================================

    /// <summary>False: the input shapes are wires.</summary>
    public override bool IsVertex => false;

    //=================================================================================================

    /// <summary>False.</summary>
    public override bool IsConstant => false;

    // The surface of the elementary law of rank Index, unfrozen (OCCT reads BSplineSurface(), decision 902).
    private BSplineSurface LawSurface(int index) => ((SurfaceNSections)_laws!.Value(index)).InternalBSplineSurface()!;

    //=================================================================================================

    /// <summary>
    /// A new frozen vertex at the start of the section's edge of rank <paramref name="index"/> (the end of the last
    /// edge for NbLaw() + 1) at the path parameter <paramref name="param"/>.
    /// </summary>
    public override Vertex Vertex(int index, double param)
    {
        BrepBuilder b = new();
        Vertex v = new();
        b.MakeVertex(ref v);
        Point3D p;

        if (index <= _edges!.ColLength())
        {
            BSplineCurve3D curve = (BSplineCurve3D)LawSurface(index).VIso(param);
            double first = curve.FirstParameter;
            curve.D0(first, out p);
            b.UpdateVertex(v, p, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else if (index == _edges.ColLength() + 1)
        {
            BSplineCurve3D curve = (BSplineCurve3D)LawSurface(index - 1).VIso(param);
            double last = curve.LastParameter;
            curve.D0(last, out p);
            b.UpdateVertex(v, p, global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        v.TShape()?.FreezeGraph();
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
        if ((index == 0) || (index == _edges!.ColLength()))
        {
            if (!_uclosed)
            {
                return tol; // The least possible error
            }
            i1 = _edges!.ColLength();
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
        tol += pFirst.DistanceTo(bs.Value(knots.Value(1)));
        return tol;
    }

    //=================================================================================================

    /// <summary>
    /// The law on the whole sections (the law's own elementary law when the sections have one edge, else a new
    /// GeomFill_NSections through the iso-curves of the law's surface).
    /// </summary>
    public override SurfaceSectionLaw? ConcatenedLaw()
    {
        SurfaceSectionLaw? law;
        if (_laws!.Length() == 1)
        {
            return _laws.Value(1);
        }
        else
        {
            _surface!.ParameterBounds(out double ufirst, out double ulast, out double vfirst, out double vlast);
            NCollection_Sequence<Curve3D> nCompo = new();
            nCompo.Clear();
            for (int jj = 1; jj <= _shapes.Length(); jj++)
            {
                nCompo.Append(_surface.VIso(_params[jj]));
            }
            law = new SurfaceNSections(nCompo, _trsfs, _params, ufirst, ulast, vfirst, vlast, _surface);
        }
        return law;
    }

    //=================================================================================================

    /// <summary>The least continuity over the sections at the vertex between their edges of ranks <paramref name="index"/> and next.</summary>
    public override Continuity Continuity(int index, double tolAngular)
    {

        int jj;
        Continuity cont_jj;
        Continuity cont = C0;

        for (jj = 1; jj <= _shapes.Length(); jj++)
        {

            Edge edge1, edge2;
            if ((index == 0) || (index == _edges!.ColLength()))
            {
                if (!_uclosed)
                {
                    return C0; // The least possible error
                }

                edge1 = ShapeCasts.Edge(_edges!.Value(_edges.ColLength(), jj));
                edge2 = ShapeCasts.Edge(_edges.Value(1, jj));
            }
            else
            {
                edge1 = ShapeCasts.Edge(_edges.Value(index, jj));
                edge2 = ShapeCasts.Edge(_edges.Value(index + 1, jj));
            }

            Vertex v1, v2;
            if (edge1.Orientation == Reversed)
            {
                v1 = Exploration.FirstVertex(edge1);
            }
            else
            {
                v1 = Exploration.LastVertex(edge1);
            }
            if (edge2.Orientation == Reversed)
            {
                v2 = Exploration.LastVertex(edge2);
            }
            else
            {
                v2 = Exploration.FirstVertex(edge2);
            }

            if (Tool.Degenerated(edge1) || Tool.Degenerated(edge2))
            {
                cont_jj = CN;
            }
            else
            {
                double u1 = Tool.Parameter(v1, edge1);
                double u2 = Tool.Parameter(v2, edge2);
                EdgeCurveAdaptor curve1 = new(edge1);
                EdgeCurveAdaptor curve2 = new(edge2);
                double eps = Tool.Tolerance(v2) + Tool.Tolerance(v1);
                cont_jj = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(curve1, curve2, u1, u2, eps, tolAngular);
            }

            if (jj == 1)
            {
                cont = cont_jj;
            }
            if (cont > cont_jj)
            {
                cont = cont_jj;
            }
        }

        return cont;
    }

    //=================================================================================================

    /// <summary>The section at the path parameter <paramref name="v"/>: a new frozen wire of the law's iso-curves.</summary>
    public override void D0(double v, ref Shape s)
    {
        LibraryMakeWire mw = new();
        int ii, nbEdge = _laws!.Length();
        for (ii = 1; ii <= nbEdge; ii++)
        {
            BSplineCurve3D curve = (BSplineCurve3D)LawSurface(ii).VIso(v);
            double first = curve.FirstParameter, last = curve.LastParameter;
            Edge e = new LibraryMakeEdge(curve, first, last).InternalEdge();
            mw.Add(e);
        }
        Orientation orien = Forward;
        Shape localShape = mw.Wire().Oriented(orien);
        s = ShapeCasts.Wire(localShape);
        //  S = TopoDS::Wire(MW.Wire().Oriented(Orien));
    }
}
