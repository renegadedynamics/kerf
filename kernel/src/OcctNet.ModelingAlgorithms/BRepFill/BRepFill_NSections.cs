// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_NSections.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_NSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Occt;

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
public sealed class BRepFill_NSections : BRepFill_SectionLaw
{
    private readonly double VFirst;
    private readonly double VLast;
    private readonly NCollection_Sequence<Shape> myShapes = new();
    private readonly NCollection_Sequence<Transform3D> myTrsfs = new();
    private readonly NCollection_Sequence<double> myParams = new();
    private NCollection_HArray2<Shape>? myEdges;
    private BSplineSurface? mySurface;

    //=======================================================================
    // function : EdgeToBSpline
    // purpose  : auxiliary -- get curve from edge and convert it to bspline
    //           parameterized from 0 to 1
    //=======================================================================

    // NOTE: this code duplicates the same function in BRepOffsetAPI_ThruSections.cxx
    private static BSplineCurve3D EdgeToBSpline(in Edge theEdge)
    {
        BSplineCurve3D? aBSCurve = null;
        if (Tool.Degenerated(theEdge))
        {
            // degenerated edge : construction of a point curve
            NCollection_Array1<double> aKnots = new(1, 2);
            aKnots[1] = 0.0;
            aKnots[2] = 1.0;

            NCollection_Array1<int> aMults = new(1, 2);
            aMults[1] = 2;
            aMults[2] = 2;

            NCollection_Array1<Point3D> aPoles = new(1, 2);
            Exploration.Vertices(theEdge, out Vertex vl, out Vertex vf);
            aPoles[1] = Tool.Pnt(vf);
            aPoles[2] = Tool.Pnt(vl);

            aBSCurve = new BSplineCurve3D(aPoles, aKnots, aMults, 1);
        }
        else
        {
            // get the curve of the edge
            Curve3D aCurve = Tool.Curve(theEdge, out Placement aLoc, out double aFirst, out double aLast)!;

            // convert its part used by edge to bspline; note that if edge curve is bspline,
            // approximation or conversion made via trimmed curve is still needed -- it will copy it,
            // segment as appropriate, and remove periodicity if it is periodic (deadly for approximator)
            TrimmedCurve3D aTrimCurve = new(aCurve, aFirst, aLast);

            Curve3D aCurveTemp = aTrimCurve; // to avoid ambiguity
            ApproxCurve anAppr = new(aCurveTemp, global::Kerf.Engine.Foundation.Precision.Confusion(), C1, 16, 14);
            if (anAppr.HasResult)
            {
                aBSCurve = anAppr.InternalCurve();
            }

            if (aBSCurve is null)
            {
                aBSCurve = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrimCurve);
            }

            // apply transformation if needed
            if (!aLoc.IsIdentity)
            {
                aBSCurve.Transform(aLoc.Transformation());
            }

            // reparameterize to [0,1]
            NCollection_Array1<double> aKnots = new(aBSCurve.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, aKnots);
            aBSCurve.SetKnots(aKnots);
        }

        // reverse curve if edge is reversed
        if (theEdge.Orientation() == Reversed)
        {
            aBSCurve.Reverse();
        }

        return aBSCurve;
    }

    //=================================================================================================

    private static BSplineSurface totalsurf(NCollection_Array2<Shape> shapes,
                                                 int NbSects,
                                                 int NbEdges,
                                                 NCollection_Sequence<double> @params,
                                                 bool w1Point,
                                                 bool w2Point,
                                                 bool uClosed,
                                                 bool vClosed,
                                                 double myPres3d)
    {
        int i, j, jdeb = 1, jfin = NbSects;
        Edge edge;

        GeomFill_SectionGenerator section = new();
        BSplineSurface surface;
        BSplineCurve3D? BS, BS1 = null;

        if (w1Point)
        {
            jdeb++;
            edge = ShapeCasts.Edge(shapes.Value(1, 1));
            Exploration.Vertices(edge, out Vertex vl, out Vertex vf);
            NCollection_Array1<Point3D> Extremities = new(1, 2);
            Extremities[1] = Tool.Pnt(vf);
            Extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            BSplineCurve3D BSPoint = new(Extremities, Bounds, Mult, 1);
            section.AddCurve(BSPoint);
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
                section.AddCurve(BS1!);
            }

            else
            {
                // read the first edge to initialise CompBS;
                Edge aPrevEdge = ShapeCasts.Edge(shapes.Value(1, j));
                BSplineCurve3D curvBS = EdgeToBSpline(aPrevEdge);

                // initialization
                CompCurveToBSplineCurve CompBS = new(curvBS);

                for (i = 2; i <= NbEdges; i++)
                {
                    // read the edge
                    Edge aNextEdge = ShapeCasts.Edge(shapes.Value(i, j));
                    curvBS = EdgeToBSpline(aNextEdge);

                    // concatenation
                    Vertex ComV = new();
                    double epsV;
                    bool Bof = Exploration.CommonVertex(aPrevEdge, aNextEdge, ref ComV);
                    if (Bof)
                    {
                        epsV = Tool.Tolerance(ComV);
                    }
                    else
                    {
                        epsV = global::Kerf.Engine.Foundation.Precision.Confusion();
                    }
                    Bof = CompBS.Add(curvBS, epsV, true, false, 1);
                    if (!Bof)
                    {
                        Bof = CompBS.Add(curvBS, 200 * epsV, true, false, 1);
                    }

                    // remember previous edge
                    aPrevEdge = aNextEdge;
                }

                // return the final section
                BS = CompBS.InternalBSplineCurve();
                section.AddCurve(BS!);

                // case of looping sections
                if (j == jdeb && vClosed)
                {
                    BS1 = BS;
                }
            }
        }

        if (w2Point)
        {
            edge = ShapeCasts.Edge(shapes.Value(NbEdges, NbSects));
            Exploration.Vertices(edge, out Vertex vl, out Vertex vf);
            NCollection_Array1<Point3D> Extremities = new(1, 2);
            Extremities[1] = Tool.Pnt(vf);
            Extremities[2] = Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            BSplineCurve3D BSPoint = new(Extremities, Bounds, Mult, 1);
            section.AddCurve(BSPoint);
        }

        NCollection_HArray1<double> HPar = new(1, @params.Length());
        for (i = 1; i <= @params.Length(); i++)
        {
            HPar.SetValue(i, @params[i]);
        }
        section.SetParam(HPar);
        section.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        GeomFill_Line line = new(NbSects);
        int nbIt = 0, degmin = 2, degmax = 6;
        bool knownP = true;
        GeomFill_AppSurf anApprox = new(degmin, degmax, myPres3d, myPres3d, nbIt, knownP);
        bool SpApprox = true;
        anApprox.Perform(line, section, SpApprox);
        bool uperiodic = uClosed;
        bool vperiodic = vClosed;
        int nup = anApprox.SurfPoles().ColLength(), nvp = anApprox.SurfPoles().RowLength();
        NCollection_Array1<int> Umults = new(1, anApprox.SurfUKnots().Length());
        Umults.Assign(anApprox.SurfUMults());
        NCollection_Array1<int> Vmults = new(1, anApprox.SurfVKnots().Length());
        Vmults.Assign(anApprox.SurfVMults());

        if (uperiodic)
        {
            int nbuk = anApprox.SurfUKnots().Length();
            Umults[1]--;
            Umults[nbuk]--;
            nup--;
        }

        if (vperiodic)
        {
            int nbvk = anApprox.SurfVKnots().Length();
            Vmults[1]--;
            Vmults[nbvk]--;
            nvp--;
        }

        NCollection_Array2<Point3D> poles = new(1, nup, 1, nvp);
        NCollection_Array2<double> weights = new(1, nup, 1, nvp);
        for (j = 1; j <= nvp; j++)
        {
            for (i = 1; i <= nup; i++)
            {
                poles[i, j] = anApprox.SurfPoles()[i, j];
                weights[i, j] = anApprox.SurfWeights()[i, j];
            }
        }

        // To create non-rational surface if possible
        double TolEps = 1.0e-13;
        bool Vrational = false, Urational = false;
        for (j = 1; j <= weights.UpperCol(); j++)
        {
            if (!Vrational)
            {
                for (i = 1; i <= weights.UpperRow() - 1; i++)
                {
                    // double signeddelta = weights(i,j) - weights(i+1,j);
                    double delta = Math.Abs(weights[i, j] - weights[i + 1, j]);
                    //	  double eps = Epsilon( std::abs(weights(i,j)) );
                    if (delta > TolEps /* || delta > 3.*eps*/)
                    {
                        Vrational = true;
                        break;
                    }
                }
            }
        }
        for (i = 1; i <= weights.UpperRow(); i++)
        {
            if (!Urational)
            {
                for (j = 1; j <= weights.UpperCol() - 1; j++)
                {
                    // double signeddelta = weights(i,j) - weights(i,j+1);
                    double delta = Math.Abs(weights[i, j] - weights[i, j + 1]);
                    //	  double eps = Epsilon( std::abs(weights(i,j)) );
                    if (delta > TolEps /* || delta > 3.*eps*/)
                    {
                        Urational = true;
                        break;
                    }
                }
            }
        }
        if (!Vrational && !Urational)
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
                                          anApprox.SurfUKnots(),
                                          anApprox.SurfVKnots(),
                                          Umults,
                                          Vmults,
                                          anApprox.UDegree(),
                                          anApprox.VDegree(),
                                          uperiodic,
                                          vperiodic);
        return surface;
    }

    //=================================================================================================

    /// <summary>The law through the wires or vertices <paramref name="S"/>, at the parameters 0, 1, 2, ...</summary>
    public BRepFill_NSections(NCollection_Sequence<Shape> S, bool Build = true)

    {
        myShapes.Assign(S);
        VFirst = 0.0;
        VLast = 1.0;
        NCollection_Sequence<double> par = new();
        par.Clear();
        for (int i = 1; i <= S.Length(); i++)
        {
            par.Append(i - 1);
        }
        myParams.Assign(par);
        Init(par, Build);
        myDone = true;
    }

    //=======================================================================
    // function : Create
    // purpose  : WSeq + Param
    //=======================================================================

    /// <summary>
    /// The law through the sections <paramref name="S"/> placed by <paramref name="Transformations"/> at the increasing
    /// parameters <paramref name="P"/>, on the path's parameter range [<paramref name="VF"/>, <paramref name="VL"/>].
    /// Not done if the parameters don't increase.
    /// </summary>
    public BRepFill_NSections(NCollection_Sequence<Shape> S,
                              NCollection_Sequence<Transform3D> Transformations,
                              NCollection_Sequence<double> P,
                              double VF,
                              double VL,
                              bool Build = true)

    {
        bool ok = true;
        for (int iseq = 1; iseq < P.Length(); iseq++)
        {
            ok = ok && (P.Value(iseq) < P.Value(iseq + 1));
        }
        if (ok)
        {
            myParams.Assign(P);
            myShapes.Assign(S);
            myTrsfs.Assign(Transformations);
            VFirst = VF;
            VLast = VL;
            Init(P, Build);
            myDone = true;
        }
        else
        {
            myDone = false;
        }
    }

    //=======================================================================
    // function : Init
    // purpose  : Create a table of GeomFill_SectionLaw
    //=======================================================================
    private void Init(NCollection_Sequence<double> P, bool Build)
    {
        WireExplorer wexp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        Edge E;
        int ii, NbEdge, jj, NbSects = P.Length();
        int ideb = 1, ifin = NbSects;
        bool wClosed, w1Point = true, w2Point = true;
        Wire W;

        // Check if the start and end wires are punctual
        W = ShapeCasts.Wire(myShapes[1]);
        for (wexp.Init(W); wexp.More(); wexp.Next())
        {
            //    w1Point = w1Point && B.Degenerated(wexp.Current());
            w1Point = w1Point && Tool.Degenerated(wexp.Current());
        }
        if (w1Point)
        {
            ideb++;
        }
        W = ShapeCasts.Wire(myShapes[NbSects]);
        for (wexp.Init(W); wexp.More(); wexp.Next())
        {
            //    w2Point = w2Point && B.Degenerated(wexp.Current());
            w2Point = w2Point && Tool.Degenerated(wexp.Current());
        }
        if (w2Point)
        {
            ifin--;
        }

        // Check if the start and end wires are identical
        vclosed = myShapes[1].IsSame(myShapes[NbSects]);

        // Count the number of non-degenerated edges
        W = ShapeCasts.Wire(myShapes[ideb]);
        for (NbEdge = 0, wexp.Init(W); wexp.More(); wexp.Next())
        {
            //    if (! B.Degenerated(wexp.Current())) NbEdge++;
            if (!Tool.Degenerated(wexp.Current()))
            {
                NbEdge++;
            }
        }

        myEdges = new NCollection_HArray2<Shape>(1, NbEdge, 1, NbSects);

        // Fill tables
        uclosed = true;
        for (jj = ideb; jj <= ifin; jj++)
        {

            W = ShapeCasts.Wire(myShapes[jj]);

            for (ii = 1, wexp.Init(W); ii <= NbEdge; wexp.Next(), ii++)
            {
                E = wexp.Current();

                //      if ( ! B.Degenerated(E)) {
                if (!Tool.Degenerated(E))
                {
                    myEdges.SetValue(ii, jj, E);
                    if (E.Orientation() == Forward)
                    {
                        myIndices.Bind(E, ii);
                    }
                    else
                    {
                        myIndices.Bind(E, -ii);
                    }
                }
            }

            // Is the law closed by U ?

            wClosed = W.Closed();
            if (!wClosed)
            {
                // if unsure about the flag, make check
                Edge Edge1, Edge2;
                Vertex V1, V2;
                Edge1 = ShapeCasts.Edge(myEdges.Value(NbEdge, jj));
                Edge2 = ShapeCasts.Edge(myEdges.Value(1, jj));

                if (Edge1.Orientation() == Reversed)
                {
                    V1 = Exploration.FirstVertex(Edge1);
                }
                else
                {
                    V1 = Exploration.LastVertex(Edge1);
                }
                if (Edge2.Orientation() == Reversed)
                {
                    V2 = Exploration.LastVertex(Edge2);
                }
                else
                {
                    V2 = Exploration.FirstVertex(Edge2);
                }
                if (V1.IsSame(V2))
                {
                    wClosed = true;
                }
                else
                {
                    EdgeCurveAdaptor Curve1 = new(Edge1);
                    EdgeCurveAdaptor Curve2 = new(Edge2);
                    double U1 = Tool.Parameter(V1, Edge1);
                    double U2 = Tool.Parameter(V2, Edge2);
                    double Eps = Tool.Tolerance(V2) + Tool.Tolerance(V1);

                    wClosed = Curve1.Value(U1).IsEqual(Curve2.Value(U2), Eps);
                }
            }
            if (!wClosed)
            {
                uclosed = false;
            }
        }

        // point sections at end
        if (w1Point)
        {
            W = ShapeCasts.Wire(myShapes[1]);
            wexp.Init(W);
            E = wexp.Current();
            for (ii = 1; ii <= NbEdge; ii++)
            {
                myEdges.SetValue(ii, 1, E);
            }
        }

        if (w2Point)
        {
            W = ShapeCasts.Wire(myShapes[NbSects]);
            wexp.Init(W);
            E = wexp.Current();
            for (ii = 1; ii <= NbEdge; ii++)
            {
                myEdges.SetValue(ii, NbSects, E);
            }
        }

        myLaws = new NCollection_HArray1<GeomFill_SectionLaw>(1, NbEdge);

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        mySurface = totalsurf(myEdges.Array2(),
                              myShapes.Length(),
                              NbEdge,
                              myParams,
                              w1Point,
                              w2Point,
                              uclosed,
                              vclosed,
                              tol);

        // Increase the degree so that the position D2
        // on GeomFill_NSections could be correct
        // see comments in GeomFill_NSections
        if (mySurface.VDegree() < 2)
        {
            mySurface.IncreaseDegree(mySurface.UDegree(), 2);
        }

        // Fill tables
        if (Build)
        {
            for (ii = 1; ii <= NbEdge; ii++)
            {
                NCollection_Sequence<Curve3D> NC = new();
                NC.Clear();
                for (jj = 1; jj <= NbSects; jj++)
                {
                    E = ShapeCasts.Edge(myEdges.Value(ii, jj));
                    Curve3D C;
                    //	if (B.Degenerated(E)) {
                    if (Tool.Degenerated(E))
                    {
                        Exploration.Vertices(E, out Vertex vl, out Vertex vf);
                        NCollection_Array1<Point3D> Extremities = new(1, 2);
                        Extremities[1] = Tool.Pnt(vf);
                        Extremities[2] = Tool.Pnt(vl);
                        NCollection_Array1<double> Bounds = new(1, 2);
                        Bounds[1] = 0.0;
                        Bounds[2] = 1.0;
                        NCollection_Array1<int> Mult = new(1, 2);
                        Mult[1] = 2;
                        Mult[2] = 2;
                        BSplineCurve3D BSPoint = new(Extremities, Bounds, Mult, 1);
                        C = BSPoint;
                    }
                    else
                    {
                        C = Tool.Curve(E, out double First, out double Last)!;

                        if (E.Orientation() == Reversed)
                        {
                            double aux;
                            Curve3D CBis;
                            CBis = C.Reversed(); // To avoid the spoiling of the topology
                            aux = C.ReversedParameter(First);
                            First = C.ReversedParameter(Last);
                            Last = aux;
                            C = CBis;
                        }
                        if ((ii > 1) || (!Tool.IsClosed(E)))
                        { // Cut C
                            TrimmedCurve3D TC = new(C, First, Last);
                            C = TC;
                        }
                        //  otherwise preserve the integrity of the curve
                    }
                    NC.Append(C);
                }

                double Ufirst = ii - 1;
                double Ulast = ii;
                myLaws.ChangeValue(ii) =
                    new GeomFill_NSections(NC, myTrsfs, myParams, Ufirst, Ulast, VFirst, VLast, mySurface);
            }
        }
    }

    //=================================================================================================

    /// <summary>False: the input shapes are wires.</summary>
    public override bool IsVertex() => false;

    //=================================================================================================

    /// <summary>False.</summary>
    public override bool IsConstant() => false;

    // The surface of the elementary law of rank Index, unfrozen (OCCT reads BSplineSurface(), decision 902).
    private BSplineSurface LawSurface(int Index) => ((GeomFill_NSections)myLaws!.Value(Index)).InternalBSplineSurface()!;

    //=================================================================================================

    /// <summary>
    /// A new frozen vertex at the start of the section's edge of rank <paramref name="Index"/> (the end of the last
    /// edge for NbLaw() + 1) at the path parameter <paramref name="Param"/>.
    /// </summary>
    public override Vertex Vertex(int Index, double Param)
    {
        BrepBuilder B = new();
        Vertex V = new();
        B.MakeVertex(ref V);
        Point3D P;

        if (Index <= myEdges!.ColLength())
        {
            BSplineCurve3D Curve = (BSplineCurve3D)LawSurface(Index).VIso(Param);
            double first = Curve.FirstParameter();
            Curve.D0(first, out P);
            B.UpdateVertex(V, P, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else if (Index == myEdges.ColLength() + 1)
        {
            BSplineCurve3D Curve = (BSplineCurve3D)LawSurface(Index - 1).VIso(Param);
            double last = Curve.LastParameter();
            Curve.D0(last, out P);
            B.UpdateVertex(V, P, global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        V.TShape()?.FreezeGraph();
        return V;
    }

    //=======================================================================
    // function : VertexTol
    // purpose  : Evaluate the hole between 2 edges of the section
    //=======================================================================

    /// <summary>The gap between the end of the edge of rank <paramref name="Index"/> and the start of the next one.</summary>
    public override double VertexTol(int Index, double Param)
    {
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int I1, I2;
        if ((Index == 0) || (Index == myEdges!.ColLength()))
        {
            if (!uclosed)
            {
                return Tol; // The least possible error
            }
            I1 = myEdges!.ColLength();
            I2 = 1;
        }
        else
        {
            I1 = Index;
            I2 = I1 + 1;
        }

        GeomFill_SectionLaw Loi;
        int NbPoles, NbKnots, Degree;
        NCollection_HArray1<Point3D> Poles;
        NCollection_HArray1<double> Knots, Weigth;
        NCollection_HArray1<int> Mults;
        BSplineCurve3D BS;
        Point3D PFirst;

        Loi = myLaws!.Value(I1);
        Loi.SectionShape(out NbPoles, out NbKnots, out Degree);
        Poles = new NCollection_HArray1<Point3D>(1, NbPoles);
        Weigth = new NCollection_HArray1<double>(1, NbPoles);
        Loi.D0(Param, Poles.ChangeArray1(), Weigth.ChangeArray1());
        Knots = new NCollection_HArray1<double>(1, NbKnots);
        Loi.Knots(Knots.ChangeArray1());
        Mults = new NCollection_HArray1<int>(1, NbKnots);
        Loi.Mults(Mults.ChangeArray1());
        BS = new BSplineCurve3D(Poles.Array1(),
                                   Weigth.Array1(),
                                   Knots.Array1(),
                                   Mults.Array1(),
                                   Degree,
                                   Loi.IsUPeriodic());
        PFirst = BS.Value(Knots.Value(Knots.Length()));

        Loi = myLaws.Value(I2);
        Loi.SectionShape(out NbPoles, out NbKnots, out Degree);
        Poles = new NCollection_HArray1<Point3D>(1, NbPoles);
        Weigth = new NCollection_HArray1<double>(1, NbPoles);
        Loi.D0(Param, Poles.ChangeArray1(), Weigth.ChangeArray1());
        Knots = new NCollection_HArray1<double>(1, NbKnots);
        Loi.Knots(Knots.ChangeArray1());
        Mults = new NCollection_HArray1<int>(1, NbKnots);
        Loi.Mults(Mults.ChangeArray1());
        BS = new BSplineCurve3D(Poles.Array1(),
                                   Weigth.Array1(),
                                   Knots.Array1(),
                                   Mults.Array1(),
                                   Degree,
                                   Loi.IsUPeriodic());
        Tol += PFirst.Distance(BS.Value(Knots.Value(1)));
        return Tol;
    }

    //=================================================================================================

    /// <summary>
    /// The law on the whole sections (the law's own elementary law when the sections have one edge, else a new
    /// GeomFill_NSections through the iso-curves of the law's surface).
    /// </summary>
    public override GeomFill_SectionLaw? ConcatenedLaw()
    {
        GeomFill_SectionLaw? Law;
        if (myLaws!.Length() == 1)
        {
            return myLaws.Value(1);
        }
        else
        {
            mySurface!.Bounds(out double Ufirst, out double Ulast, out double Vfirst, out double Vlast);
            NCollection_Sequence<Curve3D> NCompo = new();
            NCompo.Clear();
            for (int jj = 1; jj <= myShapes.Length(); jj++)
            {
                NCompo.Append(mySurface.VIso(myParams[jj]));
            }
            Law = new GeomFill_NSections(NCompo, myTrsfs, myParams, Ufirst, Ulast, Vfirst, Vlast, mySurface);
        }
        return Law;
    }

    //=================================================================================================

    /// <summary>The least continuity over the sections at the vertex between their edges of ranks <paramref name="Index"/> and next.</summary>
    public override Continuity Continuity(int Index, double TolAngular)
    {

        int jj;
        Continuity cont_jj;
        Continuity cont = C0;

        for (jj = 1; jj <= myShapes.Length(); jj++)
        {

            Edge Edge1, Edge2;
            if ((Index == 0) || (Index == myEdges!.ColLength()))
            {
                if (!uclosed)
                {
                    return C0; // The least possible error
                }

                Edge1 = ShapeCasts.Edge(myEdges!.Value(myEdges.ColLength(), jj));
                Edge2 = ShapeCasts.Edge(myEdges.Value(1, jj));
            }
            else
            {
                Edge1 = ShapeCasts.Edge(myEdges.Value(Index, jj));
                Edge2 = ShapeCasts.Edge(myEdges.Value(Index + 1, jj));
            }

            Vertex V1, V2;
            if (Edge1.Orientation() == Reversed)
            {
                V1 = Exploration.FirstVertex(Edge1);
            }
            else
            {
                V1 = Exploration.LastVertex(Edge1);
            }
            if (Edge2.Orientation() == Reversed)
            {
                V2 = Exploration.LastVertex(Edge2);
            }
            else
            {
                V2 = Exploration.FirstVertex(Edge2);
            }

            if (Tool.Degenerated(Edge1) || Tool.Degenerated(Edge2))
            {
                cont_jj = CN;
            }
            else
            {
                double U1 = Tool.Parameter(V1, Edge1);
                double U2 = Tool.Parameter(V2, Edge2);
                EdgeCurveAdaptor Curve1 = new(Edge1);
                EdgeCurveAdaptor Curve2 = new(Edge2);
                double Eps = Tool.Tolerance(V2) + Tool.Tolerance(V1);
                cont_jj = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(Curve1, Curve2, U1, U2, Eps, TolAngular);
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

    /// <summary>The section at the path parameter <paramref name="V"/>: a new frozen wire of the law's iso-curves.</summary>
    public override void D0(double V, ref Shape S)
    {
        BRepLib_MakeWire MW = new();
        int ii, NbEdge = myLaws!.Length();
        for (ii = 1; ii <= NbEdge; ii++)
        {
            BSplineCurve3D Curve = (BSplineCurve3D)LawSurface(ii).VIso(V);
            double first = Curve.FirstParameter(), last = Curve.LastParameter();
            Edge E = new BRepLib_MakeEdge(Curve, first, last).InternalEdge();
            MW.Add(E);
        }
        Orientation Orien = Forward;
        Shape aLocalShape = MW.Wire().Oriented(Orien);
        S = ShapeCasts.Wire(aLocalShape);
        //  S = TopoDS::Wire(MW.Wire().Oriented(Orien));
    }
}
