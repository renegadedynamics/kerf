// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdgeLoop.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdgeLoop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using static Occt.StepToTopoDS_TranslateEdgeLoopError;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Translates the edge_loop of a face bound into a wire on the face, with the edges' pcurves.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1259).</remarks>
public sealed class StepToTopoDS_TranslateEdgeLoop : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateEdgeLoopError myError;
    private TopoDS_Shape myResult = new TopoDS_Shape();

    private static void RemoveSinglePCurve(in TopoDS_Edge aEdge, in TopoDS_Face aFace)
    {
        new ShapeBuild_Edge().RemovePCurve(aEdge, aFace);
    }

    private static void RemovePCurves(in TopoDS_Wire aWire, in TopoDS_Face aFace)
    {
        var EdgeExp = new TopExp_Explorer(aWire, TopAbs_EDGE);
        while (EdgeExp.More())
        {
            TopoDS_Edge myEdge = TopoDS.Edge(EdgeExp.Current());
            RemoveSinglePCurve(myEdge, aFace);
            EdgeExp.Next();
        }
    }

    // Checks the pcurves topological trimming parameter consistency and deviation between 2D ans 3D
    private static void CheckPCurves(in TopoDS_Wire aWire, in TopoDS_Face aFace, bool isPlane, double preci, bool theStdSameParameterMode)
    {
        if (isPlane)
        {
            RemovePCurves(aWire, aFace);
            return;
        }

        var B = new BRep_Builder();
        double w1, w2, cf, cl;
        Geom_Surface mySurf = BRep_Tool.Surface(aFace);

        var sbwd = new ShapeExtend_WireData(aWire);
        for (int i = 1; i <= sbwd.NbEdges(); i++)
        {
            TopoDS_Edge myEdge = sbwd.Edge(i);

            // First Check : 2D Parameters on Edge :
            // Case 1 : w1 == w2 illegal => Drop the PCurve
            // Case 2 : on bounded curve w1 < FirstParameter => w1 = FirstParameter
            //                           w2 > LastParameter  => w2 = LastParameter

            Geom2d_Curve thePC;
            var sae = new ShapeAnalysis_Edge();
            if (!sae.PCurve(myEdge, aFace, out thePC, out w1, out w2, false))
            {
                continue;
            }

            cf = thePC.FirstParameter();
            cl = thePC.LastParameter();

            if (w1 == w2)
            {
                RemoveSinglePCurve(myEdge, aFace);
                continue;
            }

            // check parameters of range only for not periodic curves
            if (!thePC.IsPeriodic())
            {
                if (w1 < cf)
                {
                    B.Range(myEdge, aFace, cf, w2);
                    w1 = cf;
                }

                if (w2 > cl)
                {
                    B.Range(myEdge, aFace, w1, cl);
                    w2 = cl;
                }
            }

            if (w1 > w2 && mySurf.IsUPeriodic())
            {
                mySurf.Bounds(out double u1, out double u2, out double v1, out double v2);
                ElCLib.AdjustPeriodic(u1, u2, Min(Math.Abs(w2 - w1) / 2, Occt.Precision.PConfusion()), ref w1, ref w2);
                B.Range(myEdge, aFace, w1, w2);
            }

            // advanced check
            XSAlgo_ShapeProcessor.CheckPCurve(myEdge, aFace, preci, sbwd.IsSeam(i), theStdSameParameterMode);
        }
    }

    public StepToTopoDS_TranslateEdgeLoop()
    {
        myError = StepToTopoDS_TranslateEdgeLoopOther;
        done = false;
    }

    public StepToTopoDS_TranslateEdgeLoop(StepShape_FaceBound FB, in TopoDS_Face Face, Geom_Surface GeomSurf, StepGeom_Surface StepSurf,
                                          bool sameSense, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        Init(FB, Face, GeomSurf, StepSurf, sameSense, T, NMTool, theLocalFactors);
    }

    public StepToTopoDS_TranslateEdgeLoop(StepShape_FaceBound FB, in TopoDS_Face Face, Geom_Surface GeomSurf, StepGeom_Surface StepSurf,
                                          bool sameSense, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool)
        : this(FB, Face, GeomSurf, StepSurf, sameSense, T, NMTool, new StepData_Factors())
    {
    }

    /// <remarks>
    /// Reads <c>ReadSurfaceCurveMode</c> and, for the pcurves' check, <c>ReadSameParamMode</c> in the model's
    /// <see cref="StepData_StepModel.InternalParameters"/> (OCCT: the static read.stdsameparameter.mode, decision
    /// 13003). Needs <see cref="ShapeAlgo.Init"/>.
    /// </remarks>
    public void Init(StepShape_FaceBound FaceBound, in TopoDS_Face Face, Geom_Surface GeomSurf, StepGeom_Surface StepSurf, bool sameSense,
                     StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        done = true;
        StepShape_EdgeLoop EL = FaceBound.Bound() as StepShape_EdgeLoop;

        if (aTool.IsBound(EL))
        {
            myResult = TopoDS.Wire(aTool.Find(EL));
            myError = StepToTopoDS_TranslateEdgeLoopDone;
            done = true;
            return;
        }

        var B = new BRep_Builder();
        Transfer_TransientProcess TP = aTool.TransientProcess();

        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        int modepcurve = (int)aStepModel.InternalParameters.ReadSurfaceCurveMode;
        bool aStdSameParameterMode = aStepModel.InternalParameters.ReadSameParamMode;

        //  0,1 : suivre le code,  2 : ne prendre que pcurve,  3 : ne prendre que C3D

        double preci = Precision();
        TopoDS_Wire W = new TopoDS_Wire();
        TopoDS_Edge E = new TopoDS_Edge();
        TopoDS_Vertex V = new TopoDS_Vertex();

        bool isSeam, isLikeSeam;

        StepShape_OrientedEdge OrEdge1, OrEdge2;
        StepGeom_Curve StepCurve, StepCurve1, StepCurve2;
        StepRepr_DefinitionalRepresentation DRI, Dri1, Dri2;

        Geom2d_Curve C2d = null, C2d1 = null, C2d2 = null, WhichC2d1, WhichC2d2;
        TopoDS_Edge suspectE = new TopoDS_Edge(); //: f1, degEdge;

        int j, NbEdge = EL.NbEdgeList();
        if (NbEdge == 0)
        {
            TP.AddWarning(EL, "Wire not done. EdgeLoop does not contain edges.");
            done = false;
            return;
        }

        // PTV 16.09.2000
        // default value set as true (if not correct see logic of algorithm).
        bool hasPcurve = true;
        bool isPlane = GeomSurf is Geom_Plane;
        Geom_Surface ConvSurf = GeomSurf;
        if (GeomSurf is Geom_RectangularTrimmedSurface theRTS)
        {
            ConvSurf = theRTS.BasisSurface();
        }

        aTool.ComputePCurve(false);

        // What is the Wire Orientation
        bool ForwardWire = FaceBound.Orientation();

        // --- Initialize target Wire ---

        B.MakeWire(ref W);

        // -----------------------------------------------
        // Preparation : Make Vertices + Curves3d
        //  Hence, a closed curve limited by distinct vertices
        //  will give a unique vertex (if same coords)
        //
        // In addition : detect shared PCurve (cf SYRKO)
        //  This case may not be processed, PCurves has to be recomputed from scratch
        // -----------------------------------------------
        //  int theSame = 1; //gka 15.12.98

        for (j = 1; j <= NbEdge; j++)
        {
            OrEdge1 = EL.EdgeListValue(j);

            if (OrEdge1 is null || OrEdge1.EdgeElement() is null)
            {
                TP.AddWarning(OrEdge1, "Incorrect Oriented_Edge is not translated. Oriented_Edge definition is not correct");
                continue;
            }

            // see bug #29979: oriented edge contains another oriented edge
            if (OrEdge1.EdgeElement() is StepShape_OrientedEdge)
            {
                OrEdge1 = OrEdge1.EdgeElement() as StepShape_OrientedEdge;
            }

            StepShape_EdgeCurve EC = OrEdge1.EdgeElement() as StepShape_EdgeCurve;
            if (EC is null)
            {
                TP.AddWarning(OrEdge1, "Edge does not contain EDGE_CURVE, skipped");
                continue;
            }

            StepGeom_Curve C = EC.EdgeGeometry();
            if (C is not null)
            {
                if (C is StepGeom_SurfaceCurve)
                {
                    StepGeom_SurfaceCurve Sc = C as StepGeom_SurfaceCurve;
                    C = Sc.Curve3d();
                }
            }

            Geom_Curve C1 = null;
            if (C is not null)
            {
                try
                {
                    C1 = TP.FindTransient(C) as Geom_Curve;
                    if (C1 is null)
                    {
                        C1 = StepToGeom.MakeCurve(C, theLocalFactors);
                        if (C1 is not null)
                        {
                            TP.BindTransient(C, C1);
                        }
                        else
                        {
                            TP.AddWarning(C, "Could not convert a curve. Curve definition is incorrect");
                        }
                    }
                }
                catch (Exception anException) when (CaughtBySignals(anException))
                {
                    // OCCT_DEBUG output left out
                    TP.AddFail(C, "Exception was raised. Curve geometry definition is incorrect");
                }
            }

            StepShape_Vertex Vstart, Vend;
            if (EC.SameSense())
            {
                Vstart = EC.EdgeStart();
                Vend = EC.EdgeEnd();
            }
            else
            {
                Vend = EC.EdgeStart();
                Vstart = EC.EdgeEnd();
            }

            bool istV = aTool.IsBound(Vstart);
            bool iseV = aTool.IsBound(Vend);
            TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();
            var myTranVertex1 = new StepToTopoDS_TranslateVertex(Vstart, aTool, NMTool, theLocalFactors);
            var myTranVertex2 = new StepToTopoDS_TranslateVertex(Vend, aTool, NMTool, theLocalFactors);

            if (myTranVertex1.IsDone())
            {
                V1 = TopoDS.Vertex(myTranVertex1.InternalValue());
            }

            if (myTranVertex2.IsDone())
            {
                V2 = TopoDS.Vertex(myTranVertex2.InternalValue());
                gp_Pnt p1 = BRep_Tool.Pnt(V1);
                gp_Pnt p2 = BRep_Tool.Pnt(V2);
                if (p1.Distance(p2) <= Occt.Precision.Confusion())
                { //: S4136: preci) {
                    bool Fixed = true;
                    if (!iseV)
                    {
                        aTool.Bind(Vend, V1); // gka 21.08.1998 bug PRO7656
                    }
                    else if (!istV)
                    {
                        aTool.Bind(Vstart, V2);
                    }
                    else
                    {
                        aTool.Bind(Vend, V1);
                    }

                    if (C1 is not null && !C1.IsClosed() && Fixed)
                    {
                        TP.AddWarning(EL.EdgeListValue(j), "Vertex of same coordinates, set confused");
                    }
                }
            }
        }

        //: f6 abv 29 Apr 98: BUC50070 #3815: make sure that each two edges are
        // connected by the same vertex; else check that vertices confuse
        // and make it be one vertex
        // NOTE: this is done only for the case if at least one of edges
        // was not yet translated; else nothing will help
        for (j = 1; j <= NbEdge; j++)
        {
            OrEdge1 = EL.EdgeListValue(j);
            OrEdge2 = EL.EdgeListValue(j < NbEdge ? j + 1 : 1);
            if (OrEdge1 is null || OrEdge2 is null)
            {
                continue;
            }

            StepShape_EdgeCurve EC1 = OrEdge1.EdgeElement() as StepShape_EdgeCurve;
            StepShape_EdgeCurve EC2 = OrEdge2.EdgeElement() as StepShape_EdgeCurve;
            if (EC1 is null || EC2 is null) // see #29979
            {
                continue;
            }

            StepShape_Vertex Vs1, Vs2, Vs11, Vs22;
            Vs1 = (OrEdge1.Orientation() ? EC1.EdgeEnd() : EC1.EdgeStart());
            Vs2 = (OrEdge2.Orientation() ? EC2.EdgeStart() : EC2.EdgeEnd());

            Vs11 = (OrEdge1.Orientation() ? EC1.EdgeStart() : EC1.EdgeEnd());
            Vs22 = (OrEdge2.Orientation() ? EC2.EdgeEnd() : EC2.EdgeStart());

            if ((Vs1 == Vs2) || (Vs1 == Vs22) || (Vs2 == Vs11) || (Vs22 == Vs11))
            {
                continue;
            }

            var myTranVertex1 = new StepToTopoDS_TranslateVertex(Vs1, aTool, NMTool, theLocalFactors);
            var myTranVertex2 = new StepToTopoDS_TranslateVertex(Vs2, aTool, NMTool, theLocalFactors);

            TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();
            if (myTranVertex1.IsDone())
            {
                V1 = TopoDS.Vertex(myTranVertex1.InternalValue());
            }

            if (myTranVertex2.IsDone())
            {
                V2 = TopoDS.Vertex(myTranVertex2.InternalValue());
            }

            if (V1.IsNull() || V2.IsNull())
            {
                continue; // not treated
            }

            if (V1.IsSame(V2))
            {
                continue; // OK
            }

            gp_Pnt p1 = BRep_Tool.Pnt(V1);
            gp_Pnt p2 = BRep_Tool.Pnt(V2);
            bool locFixed = true;
            if (p1.Distance(p2) <= preci)
            {
                if (!aTool.IsBound(EC1))
                {
                    aTool.Bind(Vs1, V2);
                }
                else if (!aTool.IsBound(EC2))
                {
                    aTool.Bind(Vs2, V1);
                }
                else
                {
                    locFixed = false;
                }
            }
            else
            {
                locFixed = false;
            }

            if (locFixed)
            {
                TP.AddWarning(EL, "Adjacent edges do not have common vertex; set confused");
            }
            else
            {
                TP.AddWarning(EL, "Adjacent edges are not connected");
            }
        }

        // -----------------------------------------------
        // Iteration on each Oriented Edge of the EdgeLoop
        // -----------------------------------------------

        for (j = 1; j <= NbEdge; j++)
        {
            bool ThereIsLikeSeam = false;

            OrEdge1 = EL.EdgeListValue(j);
            if (OrEdge1 is null || OrEdge1.EdgeElement() is null)
            {
                continue;
            }

            // see bug #29979: oriented edge contains another oriented edge
            if (OrEdge1.EdgeElement() is StepShape_OrientedEdge)
            {
                OrEdge1 = OrEdge1.EdgeElement() as StepShape_OrientedEdge;
            }

            StepShape_EdgeCurve EC = OrEdge1.EdgeElement() as StepShape_EdgeCurve;
            if (EC is null)
            {
                continue;
            }

            // ----------------
            // Map the StepEdge
            // ----------------

            var myTranEdge = new StepToTopoDS_TranslateEdge();

            myTranEdge.SetPrecision(preci);
            myTranEdge.SetMaxTol(MaxTol());
            myTranEdge.Init(OrEdge1, aTool, NMTool, theLocalFactors);

            if (myTranEdge.IsDone())
            {
                E = TopoDS.Edge(myTranEdge.InternalValue());
                if (E.IsNull())
                {
                    continue; // NULL, on saute
                }

                StepGeom_Curve C = EC.EdgeGeometry();

                if (OrEdge1.Orientation() && EC.SameSense())
                {
                    E.Orientation(TopAbs_FORWARD);
                }
                else if (!OrEdge1.Orientation() && !EC.SameSense())
                {
                    E.Orientation(TopAbs_FORWARD);
                }
                else
                {
                    E.Orientation(TopAbs_REVERSED);
                }

                isSeam = isLikeSeam = false;

                // ------------------------------------------
                // Map the StepEdge parametric representation
                // ------------------------------------------

                // --------------------------------------------
                // CASE 1 : The Edge Geometry is of Pcurve Type
                // --------------------------------------------
                if (C is null)
                {
                    aTool.ComputePCurve(true);
                    hasPcurve = false;
                }
                else if (C is StepGeom_Pcurve)
                {
                    StepGeom_Pcurve StepPCurve = C as StepGeom_Pcurve;
                    C2d = myTranEdge.MakePCurve(StepPCurve, ConvSurf, theLocalFactors);
                    // -- Statistics --
                    aTool.AddContinuity(C2d);
                }

                // -----------------------------------------
                // CASE 2 : The curve is a SurfaceCurve i.e.
                //           - a 3D Curve (mandatory)
                //           - 2 PCurveOrSurface
                //   If modepcurve = 3, PCurve are ignored here
                // -----------------------------------------

                else if (modepcurve == 3)
                {
                    aTool.ComputePCurve(true);
                    hasPcurve = false;
                }
                else if (C is StepGeom_SurfaceCurve)
                {
                    // recouvre les cas SeamCurve et IntersectionCurve

                    StepGeom_SurfaceCurve SurfCurve = C as StepGeom_SurfaceCurve;

                    StepGeom_Pcurve StepPCurve, StepPCurve1, StepPCurve2;
                    int lastpcurve = StepToTopoDS_GeometricTool.PCurve(SurfCurve, StepSurf, out StepPCurve, 0);
                    hasPcurve = StepPCurve is not null;

                    // De toute facon, on recalcule

                    if (isPlane)
                    {
                        hasPcurve = false;
                    }

                    // -------------------------------------------
                    // ---        Special Mapping Cases :      ---
                    // ---   the SurfaceCurve is a SeamCurve   ---
                    // ---        or is like a seam curve      ---
                    // ---         (see CATIA cylinder)        ---
                    // -------------------------------------------
                    isLikeSeam = StepToTopoDS_GeometricTool.IsLikeSeam(SurfCurve, StepSurf, EC, EL);

                    isSeam = StepToTopoDS_GeometricTool.IsSeamCurve(SurfCurve, StepSurf, EC, EL);

                    if (isSeam || isLikeSeam)
                    {
                        // isLikeSeam = Two faces on the same Surface
                        StepPCurve1 = SurfCurve.AssociatedGeometryValue(1)?.Pcurve();
                        StepPCurve2 = SurfCurve.AssociatedGeometryValue(2)?.Pcurve();
                        if (StepPCurve1 is null || StepPCurve2 is null)
                        {
                            hasPcurve = false; // smh : BUC60810
                        }
                        else
                        {
                            C2d1 = myTranEdge.MakePCurve(StepPCurve1, ConvSurf, theLocalFactors);
                            C2d2 = myTranEdge.MakePCurve(StepPCurve2, ConvSurf, theLocalFactors);
                            hasPcurve = (C2d1 is not null && C2d2 is not null);
                        }

                        if (isLikeSeam)
                        {
                            suspectE = E;
                            ThereIsLikeSeam = true;
                            hasPcurve = true;
                        }
                    }
                    else if (hasPcurve)
                    {
                        //  GeometricTool : Pcurve a retourne StepPCurve
                        while (lastpcurve > 0)
                        {
                            C2d1 = myTranEdge.MakePCurve(StepPCurve, ConvSurf, theLocalFactors);
                            if (C2d1 is null)
                            {
                                TP.AddWarning(EC, "Incorrect pcurve is not translated. Pcurve definition is not correct");
                                hasPcurve = false;
                                break;
                            }
                            else
                            {
                                C2d = C2d1;
                            }

                            lastpcurve = StepToTopoDS_GeometricTool.PCurve(SurfCurve, StepSurf, out StepPCurve, lastpcurve);
                            // -- Statistics --
                            aTool.AddContinuity(C2d);
                        }
                    }

                    if (!hasPcurve)
                    {
                        // The edge geometry has no 2D representation
                        aTool.ComputePCurve(true);
                    }
                }

                // ----------------------------------------------------------
                // CASE 3 : The EdgeCurve Geometry is not a Pcurve
                //          nor a SurfaceCurve (i.e. it is a single 3D curve)
                // ----------------------------------------------------------

                else
                {
                    aTool.ComputePCurve(true);
                    hasPcurve = false;
                }

                // ----------------------------------
                // update the edge with the pcurve(s)
                // ----------------------------------

                if (hasPcurve && (isSeam || ThereIsLikeSeam))
                {
                    // -----------------------------------------------------------
                    // The Edge is a Seam Edge : The pcurve which is FORWARD has
                    //                           to be identified
                    // -----------------------------------------------------------

                    if ((C2d1 is not null) && (C2d2 is not null))
                    {
                        TopAbs_Orientation CumulO, EdgeO, WireO, FaceO;
                        EdgeO = E.Orientation();
                        if (ForwardWire)
                        {
                            WireO = TopAbs_FORWARD;
                        }
                        else
                        {
                            WireO = TopAbs_REVERSED;
                        }

                        if (sameSense)
                        {
                            FaceO = TopAbs_FORWARD;
                        }
                        else
                        {
                            FaceO = TopAbs_REVERSED;
                        }

                        CumulO = TopAbs.Compose(EdgeO, WireO);
                        CumulO = TopAbs.Compose(CumulO, FaceO);

                        bool ForwardEdge = (CumulO == TopAbs_FORWARD);

                        int forwardPC = new ShapeAnalysis_Curve().SelectForwardSeam(C2d1, C2d2);
                        if (forwardPC == 0)
                        {
                            TP.AddFail(EC, " Seam curve not mapped");
                            done = false;
                            myError = StepToTopoDS_TranslateEdgeLoopOther;
                            continue;
                        }
                        else if (!ForwardEdge)
                        {
                            forwardPC = 3 - forwardPC; // inverser 1-2
                        }

                        if (forwardPC == 1)
                        {
                            if (isSeam)
                            {
                                // When the edge is a Seam, it is better to find the topological
                                // trimming right now.
                                // Remarque : pour bien faire, il faudrait, si necessaire, recalculer
                                //            les trois courbes de maniere a ce qu`elles soient
                                //            immediatement Same Range et Same Parameter.
                                B.UpdateEdge(E, C2d1, C2d2, Face, 0.0);
                                //: S4136	      FindParameter(C2d1, C2d2, E, Face, preci);
                            }
                            else
                            {
                                B.UpdateEdge(E, C2d1, Face, 0.0); // preci
                            }
                        }
                        else
                        {
                            if (isSeam)
                            {
                                // When the edge is a Seam, it is better to find the topological
                                // trimming right now.
                                B.UpdateEdge(E, C2d2, C2d1, Face, 0.0);
                                //: S4136	      FindParameter(C2d1, C2d2, E, Face, preci);
                            }
                            else
                            {
                                B.UpdateEdge(E, C2d2, Face, 0.0);
                            }
                        }
                    }
                    else
                    {
                        TP.AddFail(EC, " Seam curve not mapped");
                        done = false;
                        myError = StepToTopoDS_TranslateEdgeLoopOther;
                        continue;
                    }
                }
                else
                {
                    // ---------------------------
                    // The Edge is a "normal" edge
                    // ---------------------------

                    if (hasPcurve)
                    {
                        if (C2d is not null && !isLikeSeam)
                        {
                            B.UpdateEdge(E, C2d, Face, 0.0);
                        }
                        else
                        {
                            TP.AddFail(EC, " Edge: Trimming of 2D curve failed");
                            done = false;
                            myError = StepToTopoDS_TranslateEdgeLoopOther;
                            continue;
                        }
                    }
                }

                if (E.IsNull())
                {
                    TP.AddFail(EC, " an Edge not mapped");
                    done = false;
                    myError = StepToTopoDS_TranslateEdgeLoopOther;
                }
            }
            else
            { // The Edge is Not mapped => switch to next wire ?
                TP.AddFail(EC, " an Edge not mapped");
                done = false;
                myError = StepToTopoDS_TranslateEdgeLoopOther;
            }

            if (done)
            {
                B.Add(W, E); // on le fait ici. Sauf si erreur rencontree ... !
            }
            else
            {
                StepShape_Vertex Vs1, Vs2;
                Vs1 = EC.EdgeStart();
                Vs2 = EC.EdgeEnd();
                if (Vs1 is not null && Vs2 is not null && Vs1 == Vs2)
                {
                    done = true;
                    TP.AddFail(EL, " Edge with equal vertices failed, scipped");
                }
            }
        }

        // The EdgeLoop is binded in the Wire

        if (!done)
        {
            TP.AddFail(EL, "At least one edge failed : wire not done");
            return;
        }

        W.Closed(BRep_Tool.IsClosed(W));
        aTool.Bind(EL, W);

        // ----------------------------------------------
        // Computes the 2D parameter of Vertices on Edges
        // ----------------------------------------------
        // pdn compute parameter of Vertices using projecting
        if (!aTool.ComputePCurve())
        {
            for (var EdgeIt = new TopoDS_Iterator(W); EdgeIt.More(); EdgeIt.Next())
            {
                TopoDS_Edge edge = TopoDS.Edge(EdgeIt.Value());
                ShapeFix_EdgeProjAux myEdgePro = ShapeAlgo.AlgoContainer().ToolContainer().EdgeProjAux();
                myEdgePro.Init(Face, edge);
                myEdgePro.Compute(preci);
                if (myEdgePro.IsFirstDone() && myEdgePro.IsLastDone())
                {
                    if (Math.Abs(myEdgePro.FirstParam() - myEdgePro.LastParam()) < Occt.Precision.PConfusion())
                    {
                        continue;
                    }

                    B.Range(edge, Face, myEdgePro.FirstParam(), myEdgePro.LastParam());
                }
                else
                {
                    RemoveSinglePCurve(edge, Face);
                }
            }
        }

        myResult = W;
        myError = StepToTopoDS_TranslateEdgeLoopDone;
        done = true;

        //  Check des PCurves SYSTEMATIQUE, s il n y en a que quelques unes
        CheckPCurves(W, Face, isPlane, preci, aStdSameParameterMode);
    }

    public void Init(StepShape_FaceBound FaceBound, in TopoDS_Face Face, Geom_Surface GeomSurf, StepGeom_Surface StepSurf, bool sameSense,
                     StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool)
        => Init(FaceBound, Face, GeomSurf, StepSurf, sameSense, aTool, NMTool, new StepData_Factors());

    /// <summary>The wire, frozen.</summary>
    /// <exception cref="StdFail_NotDone">The translation wasn't done.</exception>
    public TopoDS_Shape Value()
    {
        TopoDS_Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal TopoDS_Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateEdgeLoop::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateEdgeLoopError Error() => myError;
}
