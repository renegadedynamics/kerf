// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_NSections.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_NSections.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// A section law through N wires (sections) with the same number of edges, placed at given parameters along the path:
/// one GeomFill_NSections per edge, cut from a B-spline surface approximated through all the sections.
/// </summary>
/// <remarks>
/// The sections' edges are the caller's, read only: their curves are converted, reversed and trimmed into new curves,
/// as in OCCT (and GeomFill_NSections reparametrises its own copy of a B-spline section, decision 838). The elementary
/// laws share the law's approximated surface, which stays unfrozen: the law reads it through
/// <c>GeomFill_NSections.InternalBSplineSurface</c> (decision 5864). <see cref="D0"/> and <see cref="Vertex"/> hand
/// out new frozen shapes.
/// </remarks>
public sealed class BRepFill_NSections : BRepFill_SectionLaw
{
    private readonly double VFirst;
    private readonly double VLast;
    private readonly NCollection_Sequence<TopoDS_Shape> myShapes = new();
    private readonly NCollection_Sequence<gp_Trsf> myTrsfs = new();
    private readonly NCollection_Sequence<double> myParams = new();
    private NCollection_HArray2<TopoDS_Shape>? myEdges;
    private Geom_BSplineSurface? mySurface;

    //=======================================================================
    // function : EdgeToBSpline
    // purpose  : auxiliary -- get curve from edge and convert it to bspline
    //           parameterized from 0 to 1
    //=======================================================================

    // NOTE: this code duplicates the same function in BRepOffsetAPI_ThruSections.cxx
    private static Geom_BSplineCurve EdgeToBSpline(in TopoDS_Edge theEdge)
    {
        Geom_BSplineCurve? aBSCurve = null;
        if (BRep_Tool.Degenerated(theEdge))
        {
            // degenerated edge : construction of a point curve
            NCollection_Array1<double> aKnots = new(1, 2);
            aKnots[1] = 0.0;
            aKnots[2] = 1.0;

            NCollection_Array1<int> aMults = new(1, 2);
            aMults[1] = 2;
            aMults[2] = 2;

            NCollection_Array1<gp_Pnt> aPoles = new(1, 2);
            TopExp.Vertices(theEdge, out TopoDS_Vertex vl, out TopoDS_Vertex vf);
            aPoles[1] = BRep_Tool.Pnt(vf);
            aPoles[2] = BRep_Tool.Pnt(vl);

            aBSCurve = new Geom_BSplineCurve(aPoles, aKnots, aMults, 1);
        }
        else
        {
            // get the curve of the edge
            Geom_Curve aCurve = BRep_Tool.Curve(theEdge, out TopLoc_Location aLoc, out double aFirst, out double aLast)!;

            // convert its part used by edge to bspline; note that if edge curve is bspline,
            // approximation or conversion made via trimmed curve is still needed -- it will copy it,
            // segment as appropriate, and remove periodicity if it is periodic (deadly for approximator)
            Geom_TrimmedCurve aTrimCurve = new(aCurve, aFirst, aLast);

            Geom_Curve aCurveTemp = aTrimCurve; // to avoid ambiguity
            GeomConvert_ApproxCurve anAppr = new(aCurveTemp, Precision.Confusion(), GeomAbs_C1, 16, 14);
            if (anAppr.HasResult())
            {
                aBSCurve = anAppr.InternalCurve();
            }

            if (aBSCurve is null)
            {
                aBSCurve = GeomConvert.CurveToBSplineCurve(aTrimCurve);
            }

            // apply transformation if needed
            if (!aLoc.IsIdentity())
            {
                aBSCurve.Transform(aLoc.Transformation());
            }

            // reparameterize to [0,1]
            NCollection_Array1<double> aKnots = new(aBSCurve.InternalKnots());
            BSplCLib.Reparametrize(0.0, 1.0, aKnots);
            aBSCurve.SetKnots(aKnots);
        }

        // reverse curve if edge is reversed
        if (theEdge.Orientation() == TopAbs_REVERSED)
        {
            aBSCurve.Reverse();
        }

        return aBSCurve;
    }

    //=================================================================================================

    private static Geom_BSplineSurface totalsurf(NCollection_Array2<TopoDS_Shape> shapes,
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
        TopoDS_Edge edge;

        GeomFill_SectionGenerator section = new();
        Geom_BSplineSurface surface;
        Geom_BSplineCurve? BS, BS1 = null;

        if (w1Point)
        {
            jdeb++;
            edge = TopoDS.Edge(shapes.Value(1, 1));
            TopExp.Vertices(edge, out TopoDS_Vertex vl, out TopoDS_Vertex vf);
            NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
            Extremities[1] = BRep_Tool.Pnt(vf);
            Extremities[2] = BRep_Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            Geom_BSplineCurve BSPoint = new(Extremities, Bounds, Mult, 1);
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
                TopoDS_Edge aPrevEdge = TopoDS.Edge(shapes.Value(1, j));
                Geom_BSplineCurve curvBS = EdgeToBSpline(aPrevEdge);

                // initialization
                GeomConvert_CompCurveToBSplineCurve CompBS = new(curvBS);

                for (i = 2; i <= NbEdges; i++)
                {
                    // read the edge
                    TopoDS_Edge aNextEdge = TopoDS.Edge(shapes.Value(i, j));
                    curvBS = EdgeToBSpline(aNextEdge);

                    // concatenation
                    TopoDS_Vertex ComV = new();
                    double epsV;
                    bool Bof = TopExp.CommonVertex(aPrevEdge, aNextEdge, ref ComV);
                    if (Bof)
                    {
                        epsV = BRep_Tool.Tolerance(ComV);
                    }
                    else
                    {
                        epsV = Precision.Confusion();
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
            edge = TopoDS.Edge(shapes.Value(NbEdges, NbSects));
            TopExp.Vertices(edge, out TopoDS_Vertex vl, out TopoDS_Vertex vf);
            NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
            Extremities[1] = BRep_Tool.Pnt(vf);
            Extremities[2] = BRep_Tool.Pnt(vl);
            NCollection_Array1<double> Bounds = new(1, 2);
            Bounds[1] = 0.0;
            Bounds[2] = 1.0;
            NCollection_Array1<int> Mult = new(1, 2);
            Mult[1] = 2;
            Mult[2] = 2;
            Geom_BSplineCurve BSPoint = new(Extremities, Bounds, Mult, 1);
            section.AddCurve(BSPoint);
        }

        NCollection_HArray1<double> HPar = new(1, @params.Length());
        for (i = 1; i <= @params.Length(); i++)
        {
            HPar.SetValue(i, @params[i]);
        }
        section.SetParam(HPar);
        section.Perform(Precision.PConfusion());
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

        NCollection_Array2<gp_Pnt> poles = new(1, nup, 1, nvp);
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

        surface = new Geom_BSplineSurface(poles,
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
    public BRepFill_NSections(NCollection_Sequence<TopoDS_Shape> S, bool Build = true)

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
    public BRepFill_NSections(NCollection_Sequence<TopoDS_Shape> S,
                              NCollection_Sequence<gp_Trsf> Transformations,
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
        BRepTools_WireExplorer wexp = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        TopoDS_Edge E;
        int ii, NbEdge, jj, NbSects = P.Length();
        int ideb = 1, ifin = NbSects;
        bool wClosed, w1Point = true, w2Point = true;
        TopoDS_Wire W;

        // Check if the start and end wires are punctual
        W = TopoDS.Wire(myShapes[1]);
        for (wexp.Init(W); wexp.More(); wexp.Next())
        {
            //    w1Point = w1Point && B.Degenerated(wexp.Current());
            w1Point = w1Point && BRep_Tool.Degenerated(wexp.Current());
        }
        if (w1Point)
        {
            ideb++;
        }
        W = TopoDS.Wire(myShapes[NbSects]);
        for (wexp.Init(W); wexp.More(); wexp.Next())
        {
            //    w2Point = w2Point && B.Degenerated(wexp.Current());
            w2Point = w2Point && BRep_Tool.Degenerated(wexp.Current());
        }
        if (w2Point)
        {
            ifin--;
        }

        // Check if the start and end wires are identical
        vclosed = myShapes[1].IsSame(myShapes[NbSects]);

        // Count the number of non-degenerated edges
        W = TopoDS.Wire(myShapes[ideb]);
        for (NbEdge = 0, wexp.Init(W); wexp.More(); wexp.Next())
        {
            //    if (! B.Degenerated(wexp.Current())) NbEdge++;
            if (!BRep_Tool.Degenerated(wexp.Current()))
            {
                NbEdge++;
            }
        }

        myEdges = new NCollection_HArray2<TopoDS_Shape>(1, NbEdge, 1, NbSects);

        // Fill tables
        uclosed = true;
        for (jj = ideb; jj <= ifin; jj++)
        {

            W = TopoDS.Wire(myShapes[jj]);

            for (ii = 1, wexp.Init(W); ii <= NbEdge; wexp.Next(), ii++)
            {
                E = wexp.Current();

                //      if ( ! B.Degenerated(E)) {
                if (!BRep_Tool.Degenerated(E))
                {
                    myEdges.SetValue(ii, jj, E);
                    if (E.Orientation() == TopAbs_FORWARD)
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
                TopoDS_Edge Edge1, Edge2;
                TopoDS_Vertex V1, V2;
                Edge1 = TopoDS.Edge(myEdges.Value(NbEdge, jj));
                Edge2 = TopoDS.Edge(myEdges.Value(1, jj));

                if (Edge1.Orientation() == TopAbs_REVERSED)
                {
                    V1 = TopExp.FirstVertex(Edge1);
                }
                else
                {
                    V1 = TopExp.LastVertex(Edge1);
                }
                if (Edge2.Orientation() == TopAbs_REVERSED)
                {
                    V2 = TopExp.LastVertex(Edge2);
                }
                else
                {
                    V2 = TopExp.FirstVertex(Edge2);
                }
                if (V1.IsSame(V2))
                {
                    wClosed = true;
                }
                else
                {
                    BRepAdaptor_Curve Curve1 = new(Edge1);
                    BRepAdaptor_Curve Curve2 = new(Edge2);
                    double U1 = BRep_Tool.Parameter(V1, Edge1);
                    double U2 = BRep_Tool.Parameter(V2, Edge2);
                    double Eps = BRep_Tool.Tolerance(V2) + BRep_Tool.Tolerance(V1);

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
            W = TopoDS.Wire(myShapes[1]);
            wexp.Init(W);
            E = wexp.Current();
            for (ii = 1; ii <= NbEdge; ii++)
            {
                myEdges.SetValue(ii, 1, E);
            }
        }

        if (w2Point)
        {
            W = TopoDS.Wire(myShapes[NbSects]);
            wexp.Init(W);
            E = wexp.Current();
            for (ii = 1; ii <= NbEdge; ii++)
            {
                myEdges.SetValue(ii, NbSects, E);
            }
        }

        myLaws = new NCollection_HArray1<GeomFill_SectionLaw>(1, NbEdge);

        double tol = Precision.Confusion();
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
                NCollection_Sequence<Geom_Curve> NC = new();
                NC.Clear();
                for (jj = 1; jj <= NbSects; jj++)
                {
                    E = TopoDS.Edge(myEdges.Value(ii, jj));
                    Geom_Curve C;
                    //	if (B.Degenerated(E)) {
                    if (BRep_Tool.Degenerated(E))
                    {
                        TopExp.Vertices(E, out TopoDS_Vertex vl, out TopoDS_Vertex vf);
                        NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
                        Extremities[1] = BRep_Tool.Pnt(vf);
                        Extremities[2] = BRep_Tool.Pnt(vl);
                        NCollection_Array1<double> Bounds = new(1, 2);
                        Bounds[1] = 0.0;
                        Bounds[2] = 1.0;
                        NCollection_Array1<int> Mult = new(1, 2);
                        Mult[1] = 2;
                        Mult[2] = 2;
                        Geom_BSplineCurve BSPoint = new(Extremities, Bounds, Mult, 1);
                        C = BSPoint;
                    }
                    else
                    {
                        C = BRep_Tool.Curve(E, out double First, out double Last)!;

                        if (E.Orientation() == TopAbs_REVERSED)
                        {
                            double aux;
                            Geom_Curve CBis;
                            CBis = C.Reversed(); // To avoid the spoiling of the topology
                            aux = C.ReversedParameter(First);
                            First = C.ReversedParameter(Last);
                            Last = aux;
                            C = CBis;
                        }
                        if ((ii > 1) || (!BRep_Tool.IsClosed(E)))
                        { // Cut C
                            Geom_TrimmedCurve TC = new(C, First, Last);
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

    // The surface of the elementary law of rank Index, unfrozen (OCCT reads BSplineSurface(), decision 5864).
    private Geom_BSplineSurface LawSurface(int Index) => ((GeomFill_NSections)myLaws!.Value(Index)).InternalBSplineSurface()!;

    //=================================================================================================

    /// <summary>
    /// A new frozen vertex at the start of the section's edge of rank <paramref name="Index"/> (the end of the last
    /// edge for NbLaw() + 1) at the path parameter <paramref name="Param"/>.
    /// </summary>
    public override TopoDS_Vertex Vertex(int Index, double Param)
    {
        BRep_Builder B = new();
        TopoDS_Vertex V = new();
        B.MakeVertex(ref V);
        gp_Pnt P;

        if (Index <= myEdges!.ColLength())
        {
            Geom_BSplineCurve Curve = (Geom_BSplineCurve)LawSurface(Index).VIso(Param);
            double first = Curve.FirstParameter();
            Curve.D0(first, out P);
            B.UpdateVertex(V, P, Precision.Confusion());
        }
        else if (Index == myEdges.ColLength() + 1)
        {
            Geom_BSplineCurve Curve = (Geom_BSplineCurve)LawSurface(Index - 1).VIso(Param);
            double last = Curve.LastParameter();
            Curve.D0(last, out P);
            B.UpdateVertex(V, P, Precision.Confusion());
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
        double Tol = Precision.Confusion();
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
        NCollection_HArray1<gp_Pnt> Poles;
        NCollection_HArray1<double> Knots, Weigth;
        NCollection_HArray1<int> Mults;
        Geom_BSplineCurve BS;
        gp_Pnt PFirst;

        Loi = myLaws!.Value(I1);
        Loi.SectionShape(out NbPoles, out NbKnots, out Degree);
        Poles = new NCollection_HArray1<gp_Pnt>(1, NbPoles);
        Weigth = new NCollection_HArray1<double>(1, NbPoles);
        Loi.D0(Param, Poles.ChangeArray1(), Weigth.ChangeArray1());
        Knots = new NCollection_HArray1<double>(1, NbKnots);
        Loi.Knots(Knots.ChangeArray1());
        Mults = new NCollection_HArray1<int>(1, NbKnots);
        Loi.Mults(Mults.ChangeArray1());
        BS = new Geom_BSplineCurve(Poles.Array1(),
                                   Weigth.Array1(),
                                   Knots.Array1(),
                                   Mults.Array1(),
                                   Degree,
                                   Loi.IsUPeriodic());
        PFirst = BS.Value(Knots.Value(Knots.Length()));

        Loi = myLaws.Value(I2);
        Loi.SectionShape(out NbPoles, out NbKnots, out Degree);
        Poles = new NCollection_HArray1<gp_Pnt>(1, NbPoles);
        Weigth = new NCollection_HArray1<double>(1, NbPoles);
        Loi.D0(Param, Poles.ChangeArray1(), Weigth.ChangeArray1());
        Knots = new NCollection_HArray1<double>(1, NbKnots);
        Loi.Knots(Knots.ChangeArray1());
        Mults = new NCollection_HArray1<int>(1, NbKnots);
        Loi.Mults(Mults.ChangeArray1());
        BS = new Geom_BSplineCurve(Poles.Array1(),
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
            NCollection_Sequence<Geom_Curve> NCompo = new();
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
    public override GeomAbs_Shape Continuity(int Index, double TolAngular)
    {

        int jj;
        GeomAbs_Shape cont_jj;
        GeomAbs_Shape cont = GeomAbs_C0;

        for (jj = 1; jj <= myShapes.Length(); jj++)
        {

            TopoDS_Edge Edge1, Edge2;
            if ((Index == 0) || (Index == myEdges!.ColLength()))
            {
                if (!uclosed)
                {
                    return GeomAbs_C0; // The least possible error
                }

                Edge1 = TopoDS.Edge(myEdges!.Value(myEdges.ColLength(), jj));
                Edge2 = TopoDS.Edge(myEdges.Value(1, jj));
            }
            else
            {
                Edge1 = TopoDS.Edge(myEdges.Value(Index, jj));
                Edge2 = TopoDS.Edge(myEdges.Value(Index + 1, jj));
            }

            TopoDS_Vertex V1, V2;
            if (Edge1.Orientation() == TopAbs_REVERSED)
            {
                V1 = TopExp.FirstVertex(Edge1);
            }
            else
            {
                V1 = TopExp.LastVertex(Edge1);
            }
            if (Edge2.Orientation() == TopAbs_REVERSED)
            {
                V2 = TopExp.LastVertex(Edge2);
            }
            else
            {
                V2 = TopExp.FirstVertex(Edge2);
            }

            if (BRep_Tool.Degenerated(Edge1) || BRep_Tool.Degenerated(Edge2))
            {
                cont_jj = GeomAbs_CN;
            }
            else
            {
                double U1 = BRep_Tool.Parameter(V1, Edge1);
                double U2 = BRep_Tool.Parameter(V2, Edge2);
                BRepAdaptor_Curve Curve1 = new(Edge1);
                BRepAdaptor_Curve Curve2 = new(Edge2);
                double Eps = BRep_Tool.Tolerance(V2) + BRep_Tool.Tolerance(V1);
                cont_jj = BRepLProp.Continuity(Curve1, Curve2, U1, U2, Eps, TolAngular);
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
    public override void D0(double V, ref TopoDS_Shape S)
    {
        BRepLib_MakeWire MW = new();
        int ii, NbEdge = myLaws!.Length();
        for (ii = 1; ii <= NbEdge; ii++)
        {
            Geom_BSplineCurve Curve = (Geom_BSplineCurve)LawSurface(ii).VIso(V);
            double first = Curve.FirstParameter(), last = Curve.LastParameter();
            TopoDS_Edge E = new BRepLib_MakeEdge(Curve, first, last).InternalEdge();
            MW.Add(E);
        }
        TopAbs_Orientation Orien = TopAbs_FORWARD;
        TopoDS_Shape aLocalShape = MW.Wire().Oriented(Orien);
        S = TopoDS.Wire(aLocalShape);
        //  S = TopoDS::Wire(MW.Wire().Oriented(Orien));
    }
}
