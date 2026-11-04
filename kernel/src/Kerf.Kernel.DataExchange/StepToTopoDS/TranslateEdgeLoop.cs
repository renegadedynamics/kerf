// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdgeLoop.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdgeLoop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Import.TranslateEdgeLoopError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates the edge_loop of a face bound into a wire on the face, with the edges' pcurves.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
internal sealed class TranslateEdgeLoop : ImportRoot
{
    private TranslateEdgeLoopError _error;
    private Shape _result = new Shape();

    private static void RemoveSinglePCurve(in Edge aEdge, in Face aFace)
    {
        new BuildEdgeBuilder().RemovePCurve(aEdge, aFace);
    }

    private static void RemovePCurves(in Wire aWire, in Face aFace)
    {
        var edgeExp = new Explorer(aWire, ShapeType.Edge);
        while (edgeExp.More())
        {
            Edge myEdge = ShapeCasts.Edge(edgeExp.Current());
            RemoveSinglePCurve(myEdge, aFace);
            edgeExp.Next();
        }
    }

    // Checks the pcurves topological trimming parameter consistency and deviation between 2D ans 3D
    private static void CheckPCurves(in Wire aWire, in Face aFace, bool isPlane, double preci, bool stdSameParameterMode)
    {
        if (isPlane)
        {
            RemovePCurves(aWire, aFace);
            return;
        }

        var b = new BrepBuilder();
        double w1, w2, cf, cl;
        Surface mySurf = Tool.Surface(aFace);

        var sbwd = new ExtendWireData(aWire);
        for (int i = 1; i <= sbwd.EdgeCount; i++)
        {
            Edge myEdge = sbwd.Edge(i);

            // First Check : 2D Parameters on Edge :
            // Case 1 : w1 == w2 illegal => Drop the PCurve
            // Case 2 : on bounded curve w1 < FirstParameter => w1 = FirstParameter
            //                           w2 > LastParameter  => w2 = LastParameter

            Curve2D thePC;
            var sae = new EdgeAnalysis();
            if (!sae.PCurve(myEdge, aFace, out thePC, out w1, out w2, false))
            {
                continue;
            }

            cf = thePC.FirstParameter;
            cl = thePC.LastParameter;

            if (w1 == w2)
            {
                RemoveSinglePCurve(myEdge, aFace);
                continue;
            }

            // check parameters of range only for not periodic curves
            if (!thePC.IsPeriodic)
            {
                if (w1 < cf)
                {
                    b.Range(myEdge, aFace, cf, w2);
                    w1 = cf;
                }

                if (w2 > cl)
                {
                    b.Range(myEdge, aFace, w1, cl);
                    w2 = cl;
                }
            }

            if (w1 > w2 && mySurf.IsUPeriodic)
            {
                mySurf.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(u1, u2, Min(Math.Abs(w2 - w1) / 2, Kerf.Engine.Foundation.Precision.PConfusion()), ref w1, ref w2);
                b.Range(myEdge, aFace, w1, w2);
            }

            // advanced check
            ShapeProcessor.CheckPCurve(myEdge, aFace, preci, sbwd.IsSeam(i), stdSameParameterMode);
        }
    }

    public TranslateEdgeLoop()
    {
        _error = TranslateEdgeLoopError.TranslateEdgeLoopOther;
        _done = false;
    }

    public TranslateEdgeLoop(StepFaceBound fb, in Face face, Surface geomSurf, StepSurface stepSurf,
                                          bool sameSense, ImportTool t, NMTool nmTool, in Factors localFactors)
    {
        Init(fb, face, geomSurf, stepSurf, sameSense, t, nmTool, localFactors);
    }

    public TranslateEdgeLoop(StepFaceBound fb, in Face face, Surface geomSurf, StepSurface stepSurf,
                                          bool sameSense, ImportTool t, NMTool nmTool)
        : this(fb, face, geomSurf, stepSurf, sameSense, t, nmTool, new Factors())
    {
    }

    /// <remarks>
    /// Reads <c>ReadSurfaceCurveMode</c> and, for the pcurves' check, <c>ReadSameParamMode</c> in the model's
    /// <see cref="StepModel.InternalParameters"/> (OCCT: the static read.stdsameparameter.mode, decision
    /// 13003). Needs <see cref="ShapeAlgo.Init"/>.
    /// </remarks>
    public void Init(StepFaceBound faceBound, in Face face, Surface geomSurf, StepSurface stepSurf, bool sameSense,
                     ImportTool aTool, NMTool nmTool, in Factors localFactors)
    {
        _done = true;
        StepEdgeLoop el = faceBound.Bound() as StepEdgeLoop;

        if (aTool.IsBound(el))
        {
            _result = ShapeCasts.Wire(aTool.Find(el));
            _error = TranslateEdgeLoopError.TranslateEdgeLoopDone;
            _done = true;
            return;
        }

        var b = new BrepBuilder();
        TransientProcess tp = aTool.TransientProcess();

        StepModel stepModel = tp.Model() as StepModel;
        int modepcurve = (int)stepModel.InternalParameters.ReadSurfaceCurveMode;
        bool stdSameParameterMode = stepModel.InternalParameters.ReadSameParamMode;

        //  0,1 : suivre le code,  2 : ne prendre que pcurve,  3 : ne prendre que C3D

        double preci = Precision();
        Wire w = new Wire();
        Edge e = new Edge();
        Vertex v = new Vertex();

        bool isSeam, isLikeSeam;

        StepOrientedEdge orEdge1, orEdge2;
        StepCurve stepCurve, stepCurve1, stepCurve2;
        DefinitionalRepresentation dri, dri1, dri2;

        Curve2D c2d = null, c2d1 = null, c2d2 = null, whichC2d1, whichC2d2;
        Edge suspectE = new Edge(); //: f1, degEdge;

        int j, nbEdge = el.NbEdgeList();
        if (nbEdge == 0)
        {
            tp.AddWarning(el, "Wire not done. EdgeLoop does not contain edges.");
            _done = false;
            return;
        }

        // PTV 16.09.2000
        // default value set as true (if not correct see logic of algorithm).
        bool hasPcurve = true;
        bool isPlane = geomSurf is PlaneSurface;
        Surface convSurf = geomSurf;
        if (geomSurf is TrimmedSurface theRTS)
        {
            convSurf = theRTS.BasisSurface;
        }

        aTool.ComputePCurve(false);

        // What is the Wire Orientation
        bool forwardWire = faceBound.Orientation();

        // --- Initialize target Wire ---

        b.MakeWire(ref w);

        // -----------------------------------------------
        // Preparation : Make Vertices + Curves3d
        //  Hence, a closed curve limited by distinct vertices
        //  will give a unique vertex (if same coords)
        //
        // In addition : detect shared PCurve (cf SYRKO)
        //  This case may not be processed, PCurves has to be recomputed from scratch
        // -----------------------------------------------
        //  int theSame = 1; //gka 15.12.98

        for (j = 1; j <= nbEdge; j++)
        {
            orEdge1 = el.EdgeListValue(j);

            if (orEdge1 is null || orEdge1.EdgeElement() is null)
            {
                tp.AddWarning(orEdge1, "Incorrect Oriented_Edge is not translated. Oriented_Edge definition is not correct");
                continue;
            }

            // see bug #29979: oriented edge contains another oriented edge
            if (orEdge1.EdgeElement() is StepOrientedEdge)
            {
                orEdge1 = orEdge1.EdgeElement() as StepOrientedEdge;
            }

            StepEdgeCurve ec = orEdge1.EdgeElement() as StepEdgeCurve;
            if (ec is null)
            {
                tp.AddWarning(orEdge1, "Edge does not contain EDGE_CURVE, skipped");
                continue;
            }

            StepCurve c = ec.EdgeGeometry();
            if (c is not null)
            {
                if (c is StepSurfaceCurve)
                {
                    StepSurfaceCurve sc = c as StepSurfaceCurve;
                    c = sc.Curve3d();
                }
            }

            Curve3D c1 = null;
            if (c is not null)
            {
                try
                {
                    c1 = tp.FindTransient(c) as Curve3D;
                    if (c1 is null)
                    {
                        c1 = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve(c, localFactors);
                        if (c1 is not null)
                        {
                            tp.BindTransient(c, c1);
                        }
                        else
                        {
                            tp.AddWarning(c, "Could not convert a curve. Curve definition is incorrect");
                        }
                    }
                }
                catch (Exception exception) when (CaughtBySignals(exception))
                {
                    // OCCT_DEBUG output left out
                    tp.AddFail(c, "Exception was raised. Curve geometry definition is incorrect");
                }
            }

            StepVertex vstart, vend;
            if (ec.SameSense())
            {
                vstart = ec.EdgeStart();
                vend = ec.EdgeEnd();
            }
            else
            {
                vend = ec.EdgeStart();
                vstart = ec.EdgeEnd();
            }

            bool istV = aTool.IsBound(vstart);
            bool iseV = aTool.IsBound(vend);
            Vertex v1 = new Vertex(), v2 = new Vertex();
            var myTranVertex1 = new TranslateVertex(vstart, aTool, nmTool, localFactors);
            var myTranVertex2 = new TranslateVertex(vend, aTool, nmTool, localFactors);

            if (myTranVertex1.IsDone)
            {
                v1 = ShapeCasts.Vertex(myTranVertex1.InternalValue());
            }

            if (myTranVertex2.IsDone)
            {
                v2 = ShapeCasts.Vertex(myTranVertex2.InternalValue());
                Point3D p1 = Tool.Pnt(v1);
                Point3D p2 = Tool.Pnt(v2);
                if (p1.DistanceTo(p2) <= Kerf.Engine.Foundation.Precision.Confusion())
                { //: S4136: preci) {
                    bool Fixed = true;
                    if (!iseV)
                    {
                        aTool.Bind(vend, v1); // gka 21.08.1998 bug PRO7656
                    }
                    else if (!istV)
                    {
                        aTool.Bind(vstart, v2);
                    }
                    else
                    {
                        aTool.Bind(vend, v1);
                    }

                    if (c1 is not null && !c1.IsClosed && Fixed)
                    {
                        tp.AddWarning(el.EdgeListValue(j), "Vertex of same coordinates, set confused");
                    }
                }
            }
        }

        //: f6 abv 29 Apr 98: BUC50070 #3815: make sure that each two edges are
        // connected by the same vertex; else check that vertices confuse
        // and make it be one vertex
        // NOTE: this is done only for the case if at least one of edges
        // was not yet translated; else nothing will help
        for (j = 1; j <= nbEdge; j++)
        {
            orEdge1 = el.EdgeListValue(j);
            orEdge2 = el.EdgeListValue(j < nbEdge ? j + 1 : 1);
            if (orEdge1 is null || orEdge2 is null)
            {
                continue;
            }

            StepEdgeCurve ec1 = orEdge1.EdgeElement() as StepEdgeCurve;
            StepEdgeCurve ec2 = orEdge2.EdgeElement() as StepEdgeCurve;
            if (ec1 is null || ec2 is null) // see #29979
            {
                continue;
            }

            StepVertex vs1, vs2, vs11, vs22;
            vs1 = (orEdge1.Orientation() ? ec1.EdgeEnd() : ec1.EdgeStart());
            vs2 = (orEdge2.Orientation() ? ec2.EdgeStart() : ec2.EdgeEnd());

            vs11 = (orEdge1.Orientation() ? ec1.EdgeStart() : ec1.EdgeEnd());
            vs22 = (orEdge2.Orientation() ? ec2.EdgeEnd() : ec2.EdgeStart());

            if ((vs1 == vs2) || (vs1 == vs22) || (vs2 == vs11) || (vs22 == vs11))
            {
                continue;
            }

            var myTranVertex1 = new TranslateVertex(vs1, aTool, nmTool, localFactors);
            var myTranVertex2 = new TranslateVertex(vs2, aTool, nmTool, localFactors);

            Vertex v1 = new Vertex(), v2 = new Vertex();
            if (myTranVertex1.IsDone)
            {
                v1 = ShapeCasts.Vertex(myTranVertex1.InternalValue());
            }

            if (myTranVertex2.IsDone)
            {
                v2 = ShapeCasts.Vertex(myTranVertex2.InternalValue());
            }

            if (v1.IsNull || v2.IsNull)
            {
                continue; // not treated
            }

            if (v1.IsSame(v2))
            {
                continue; // OK
            }

            Point3D p1 = Tool.Pnt(v1);
            Point3D p2 = Tool.Pnt(v2);
            bool locFixed = true;
            if (p1.DistanceTo(p2) <= preci)
            {
                if (!aTool.IsBound(ec1))
                {
                    aTool.Bind(vs1, v2);
                }
                else if (!aTool.IsBound(ec2))
                {
                    aTool.Bind(vs2, v1);
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
                tp.AddWarning(el, "Adjacent edges do not have common vertex; set confused");
            }
            else
            {
                tp.AddWarning(el, "Adjacent edges are not connected");
            }
        }

        // -----------------------------------------------
        // Iteration on each Oriented Edge of the EdgeLoop
        // -----------------------------------------------

        for (j = 1; j <= nbEdge; j++)
        {
            bool thereIsLikeSeam = false;

            orEdge1 = el.EdgeListValue(j);
            if (orEdge1 is null || orEdge1.EdgeElement() is null)
            {
                continue;
            }

            // see bug #29979: oriented edge contains another oriented edge
            if (orEdge1.EdgeElement() is StepOrientedEdge)
            {
                orEdge1 = orEdge1.EdgeElement() as StepOrientedEdge;
            }

            StepEdgeCurve ec = orEdge1.EdgeElement() as StepEdgeCurve;
            if (ec is null)
            {
                continue;
            }

            // ----------------
            // Map the StepEdge
            // ----------------

            var myTranEdge = new TranslateEdge();

            myTranEdge.SetPrecision(preci);
            myTranEdge.SetMaxTol(MaxTol());
            myTranEdge.Init(orEdge1, aTool, nmTool, localFactors);

            if (myTranEdge.IsDone)
            {
                e = ShapeCasts.Edge(myTranEdge.InternalValue());
                if (e.IsNull)
                {
                    continue; // NULL, on saute
                }

                StepCurve c = ec.EdgeGeometry();

                if (orEdge1.Orientation() && ec.SameSense())
                {
                    e.Orient(Forward);
                }
                else if (!orEdge1.Orientation() && !ec.SameSense())
                {
                    e.Orient(Forward);
                }
                else
                {
                    e.Orient(Reversed);
                }

                isSeam = isLikeSeam = false;

                // ------------------------------------------
                // Map the StepEdge parametric representation
                // ------------------------------------------

                // --------------------------------------------
                // CASE 1 : The Edge Geometry is of Pcurve Type
                // --------------------------------------------
                if (c is null)
                {
                    aTool.ComputePCurve(true);
                    hasPcurve = false;
                }
                else if (c is StepPcurve)
                {
                    StepPcurve stepPCurve = c as StepPcurve;
                    c2d = myTranEdge.MakePCurve(stepPCurve, convSurf, localFactors);
                    // -- Statistics --
                    aTool.AddContinuity(c2d);
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
                else if (c is StepSurfaceCurve)
                {
                    // recouvre les cas SeamCurve et IntersectionCurve

                    StepSurfaceCurve surfCurve = c as StepSurfaceCurve;

                    StepPcurve stepPCurve, stepPCurve1, stepPCurve2;
                    int lastpcurve = GeometricTool.PCurve(surfCurve, stepSurf, out stepPCurve, 0);
                    hasPcurve = stepPCurve is not null;

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
                    isLikeSeam = GeometricTool.IsLikeSeam(surfCurve, stepSurf, ec, el);

                    isSeam = GeometricTool.IsSeamCurve(surfCurve, stepSurf, ec, el);

                    if (isSeam || isLikeSeam)
                    {
                        // isLikeSeam = Two faces on the same Surface
                        stepPCurve1 = surfCurve.AssociatedGeometryValue(1)?.Pcurve();
                        stepPCurve2 = surfCurve.AssociatedGeometryValue(2)?.Pcurve();
                        if (stepPCurve1 is null || stepPCurve2 is null)
                        {
                            hasPcurve = false; // smh : BUC60810
                        }
                        else
                        {
                            c2d1 = myTranEdge.MakePCurve(stepPCurve1, convSurf, localFactors);
                            c2d2 = myTranEdge.MakePCurve(stepPCurve2, convSurf, localFactors);
                            hasPcurve = (c2d1 is not null && c2d2 is not null);
                        }

                        if (isLikeSeam)
                        {
                            suspectE = e;
                            thereIsLikeSeam = true;
                            hasPcurve = true;
                        }
                    }
                    else if (hasPcurve)
                    {
                        //  GeometricTool : Pcurve a retourne StepPCurve
                        while (lastpcurve > 0)
                        {
                            c2d1 = myTranEdge.MakePCurve(stepPCurve, convSurf, localFactors);
                            if (c2d1 is null)
                            {
                                tp.AddWarning(ec, "Incorrect pcurve is not translated. Pcurve definition is not correct");
                                hasPcurve = false;
                                break;
                            }
                            else
                            {
                                c2d = c2d1;
                            }

                            lastpcurve = GeometricTool.PCurve(surfCurve, stepSurf, out stepPCurve, lastpcurve);
                            // -- Statistics --
                            aTool.AddContinuity(c2d);
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

                if (hasPcurve && (isSeam || thereIsLikeSeam))
                {
                    // -----------------------------------------------------------
                    // The Edge is a Seam Edge : The pcurve which is FORWARD has
                    //                           to be identified
                    // -----------------------------------------------------------

                    if ((c2d1 is not null) && (c2d2 is not null))
                    {
                        Orientation cumulO, edgeO, wireO, faceO;
                        edgeO = e.Orientation;
                        if (forwardWire)
                        {
                            wireO = Forward;
                        }
                        else
                        {
                            wireO = Reversed;
                        }

                        if (sameSense)
                        {
                            faceO = Forward;
                        }
                        else
                        {
                            faceO = Reversed;
                        }

                        cumulO = global::Kerf.Engine.Topology.TopAbs.Compose(edgeO, wireO);
                        cumulO = global::Kerf.Engine.Topology.TopAbs.Compose(cumulO, faceO);

                        bool forwardEdge = (cumulO == Forward);

                        int forwardPC = new CurveAnalysis().SelectForwardSeam(c2d1, c2d2);
                        if (forwardPC == 0)
                        {
                            tp.AddFail(ec, " Seam curve not mapped");
                            _done = false;
                            _error = TranslateEdgeLoopError.TranslateEdgeLoopOther;
                            continue;
                        }
                        else if (!forwardEdge)
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
                                b.UpdateEdge(e, c2d1, c2d2, face, 0.0);
                                //: S4136	      FindParameter(C2d1, C2d2, E, Face, preci);
                            }
                            else
                            {
                                b.UpdateEdge(e, c2d1, face, 0.0); // preci
                            }
                        }
                        else
                        {
                            if (isSeam)
                            {
                                // When the edge is a Seam, it is better to find the topological
                                // trimming right now.
                                b.UpdateEdge(e, c2d2, c2d1, face, 0.0);
                                //: S4136	      FindParameter(C2d1, C2d2, E, Face, preci);
                            }
                            else
                            {
                                b.UpdateEdge(e, c2d2, face, 0.0);
                            }
                        }
                    }
                    else
                    {
                        tp.AddFail(ec, " Seam curve not mapped");
                        _done = false;
                        _error = TranslateEdgeLoopError.TranslateEdgeLoopOther;
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
                        if (c2d is not null && !isLikeSeam)
                        {
                            b.UpdateEdge(e, c2d, face, 0.0);
                        }
                        else
                        {
                            tp.AddFail(ec, " Edge: Trimming of 2D curve failed");
                            _done = false;
                            _error = TranslateEdgeLoopError.TranslateEdgeLoopOther;
                            continue;
                        }
                    }
                }

                if (e.IsNull)
                {
                    tp.AddFail(ec, " an Edge not mapped");
                    _done = false;
                    _error = TranslateEdgeLoopError.TranslateEdgeLoopOther;
                }
            }
            else
            { // The Edge is Not mapped => switch to next wire ?
                tp.AddFail(ec, " an Edge not mapped");
                _done = false;
                _error = TranslateEdgeLoopError.TranslateEdgeLoopOther;
            }

            if (_done)
            {
                b.Add(w, e); // on le fait ici. Sauf si erreur rencontree ... !
            }
            else
            {
                StepVertex vs1, vs2;
                vs1 = ec.EdgeStart();
                vs2 = ec.EdgeEnd();
                if (vs1 is not null && vs2 is not null && vs1 == vs2)
                {
                    _done = true;
                    tp.AddFail(el, " Edge with equal vertices failed, scipped");
                }
            }
        }

        // The EdgeLoop is binded in the Wire

        if (!_done)
        {
            tp.AddFail(el, "At least one edge failed : wire not done");
            return;
        }

        w.Closed(Tool.IsClosed(w));
        aTool.Bind(el, w);

        // ----------------------------------------------
        // Computes the 2D parameter of Vertices on Edges
        // ----------------------------------------------
        // pdn compute parameter of Vertices using projecting
        if (!aTool.ComputePCurve())
        {
            for (var edgeIt = new Iterator(w); edgeIt.More(); edgeIt.Next())
            {
                Edge edge = ShapeCasts.Edge(edgeIt.Value());
                EdgeProjAuxFixer myEdgePro = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().ToolContainer().EdgeProjAux();
                myEdgePro.Init(face, edge);
                myEdgePro.Compute(preci);
                if (myEdgePro.IsFirstDone && myEdgePro.IsLastDone)
                {
                    if (Math.Abs(myEdgePro.FirstParam() - myEdgePro.LastParam()) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }

                    b.Range(edge, face, myEdgePro.FirstParam(), myEdgePro.LastParam());
                }
                else
                {
                    RemoveSinglePCurve(edge, face);
                }
            }
        }

        _result = w;
        _error = TranslateEdgeLoopError.TranslateEdgeLoopDone;
        _done = true;

        //  Check des PCurves SYSTEMATIQUE, s il n y en a que quelques unes
        CheckPCurves(w, face, isPlane, preci, stdSameParameterMode);
    }

    public void Init(StepFaceBound faceBound, in Face face, Surface geomSurf, StepSurface stepSurf, bool sameSense,
                     ImportTool aTool, NMTool nmTool)
        => Init(faceBound, face, geomSurf, stepSurf, sameSense, aTool, nmTool, new Factors());

    /// <summary>The wire, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslateEdgeLoop::Value() - no result");
        return _result;
    }

    public TranslateEdgeLoopError Error() => _error;
}
