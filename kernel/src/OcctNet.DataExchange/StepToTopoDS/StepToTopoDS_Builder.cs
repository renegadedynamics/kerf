// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Builder.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using System.Globalization;
using static Occt.StepToTopoDS_BuilderError;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The transfer of a STEP shape representation item into a BRep: a manifold solid brep, a brep with voids, a faceted
/// brep (with voids), a shell based surface model, an edge based wireframe model, a face based surface model, a
/// geometric set, or a tessellated solid, shell, face or surface set.
/// </summary>
/// <remarks>
/// <para>
/// Each <c>Init</c> translates its item with a new <see cref="StepToTopoDS_Tool"/> on the transient process, which
/// receives the shapes made from the topological items as their results. Units go through
/// <see cref="StepData_Factors"/>; the model's <see cref="StepData_StepModel.InternalParameters"/> gives the modes.
/// Needs <see cref="ShapeAlgo.Init"/> (as STEPControl_Controller's initialisation does natively).
/// </para>
/// <para>
/// <see cref="Value"/> freezes the result. A reader that goes on working on the shapes of a transfer (sharing them
/// between items, through the transient process or a non-manifold tool) uses <c>InternalValue</c> and freezes once
/// at the end of the transfer (decision 1259).
/// </para>
/// </remarks>
public sealed class StepToTopoDS_Builder : StepToTopoDS_Root
{
    private StepToTopoDS_BuilderError myError;
    private TopoDS_Shape myResult = new TopoDS_Shape();

    private static void ResetPreci(StepData_StepModel theStepModel, in TopoDS_Shape S, double maxtol)
    {
        //: S4136
        int modetol = (int)theStepModel.InternalParameters.ReadMaxPrecisionMode;
        if (modetol != 0)
        {
            var STU = new ShapeFix_ShapeTolerance();
            STU.LimitTolerance(S, Occt.Precision.Confusion(), maxtol);
        }
    }

    public StepToTopoDS_Builder()
    {
        myError = StepToTopoDS_BuilderOther;
        done = false;
    }

    // The geometric statistics, written when the trace level is above 2
    private static void SendStatistics(Message_Messenger.StreamBuffer sout, StepToTopoDS_Tool aTool)
    {
        sout.Append("Geometric Statistics : ").Append('\n');
        sout.Append("   Surface Continuity : - C0 : ").Append(aTool.C0Surf()).Append('\n');
        sout.Append("                        - C1 : ").Append(aTool.C1Surf()).Append('\n');
        sout.Append("                        - C2 : ").Append(aTool.C2Surf()).Append('\n');
        sout.Append("   Curve Continuity :   - C0 : ").Append(aTool.C0Cur3()).Append('\n');
        sout.Append("                        - C1 : ").Append(aTool.C1Cur3()).Append('\n');
        sout.Append("                        - C2 : ").Append(aTool.C2Cur3()).Append('\n');
        sout.Append("   PCurve Continuity :  - C0 : ").Append(aTool.C0Cur2()).Append('\n');
        sout.Append("                        - C1 : ").Append(aTool.C1Cur2()).Append('\n');
        sout.Append("                        - C2 : ").Append(aTool.C2Cur2()).Append('\n');
    }

    /// <summary>Translates a manifold_solid_brep into a solid.</summary>
    public void Init(StepShape_ManifoldSolidBrep theManifoldSolid, Transfer_TransientProcess theTP, in StepData_Factors theLocalFactors,
                     Message_ProgressRange theProgress = default)
    {
        using Message_Messenger.StreamBuffer sout = theTP.Messenger().SendInfo();
        StepData_StepModel aStepModel = theTP.Model() as StepData_StepModel;
        // Initialisation of the Tool

        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        // Start Mapping
        StepShape_ConnectedFaceSet aShell = theManifoldSolid.Outer();

        var myTranShell = new StepToTopoDS_TranslateShell();
        myTranShell.SetPrecision(Precision());
        myTranShell.SetMaxTol(MaxTol());
        var dummyNMTool = new StepToTopoDS_NMTool();
        myTranShell.Init(aShell, aTool, dummyNMTool, theLocalFactors, theProgress);

        if (!myTranShell.IsDone())
        {
            theTP.AddWarning(aShell, " OuterShell from ManifoldSolidBrep not mapped to TopoDS");
            myError = StepToTopoDS_BuilderOther;
            done = false;
            return;
        }

        TopoDS_Shape aShape = myTranShell.InternalValue();
        aShape.Closed(true);
        TopoDS_Solid aSolid = new TopoDS_Solid();
        var aBuilder = new BRep_Builder();
        aBuilder.MakeSolid(ref aSolid);
        aBuilder.Add(aSolid, aShape);
        myResult = aSolid;
        myError = StepToTopoDS_BuilderDone;
        done = true;

        // Get Statistics :
        if (theTP.TraceLevel() > 2)
        {
            SendStatistics(sout, aTool);
        }

        ResetPreci(aStepModel, aSolid, MaxTol());
    }

    public void Init(StepShape_ManifoldSolidBrep theManifoldSolid, Transfer_TransientProcess theTP)
        => Init(theManifoldSolid, theTP, new StepData_Factors());

    /// <summary>Translates a brep_with_voids into a solid with an outer shell and voids.</summary>
    public void Init(StepShape_BrepWithVoids theBRepWithVoids, Transfer_TransientProcess theTP, in StepData_Factors theLocalFactors,
                     Message_ProgressRange theProgress = default)
    {
        using Message_Messenger.StreamBuffer sout = theTP.Messenger().SendInfo();
        // Initialisation of the Tool
        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        // Start Mapping
        TopoDS_Solid aSolid = new TopoDS_Solid();
        var aBuilder = new BRep_Builder();
        aBuilder.MakeSolid(ref aSolid);

        using var aPS = new Message_ProgressScope(theProgress, "Shell", theBRepWithVoids.NbVoids() + 1);

        var aTranShell = new StepToTopoDS_TranslateShell();
        aTranShell.SetPrecision(Precision()); // gka
        aTranShell.SetMaxTol(MaxTol());
        // OuterBound
        StepShape_ClosedShell aCShell = theBRepWithVoids.Outer() as StepShape_ClosedShell;
        // Non-manifold topology is not referenced by BrepWithVoids (ssv; 14.11.2010)
        var aDummyNMTool = new StepToTopoDS_NMTool();
        aTranShell.Init(aCShell, aTool, aDummyNMTool, theLocalFactors, aPS.Next());

        if (!aTranShell.IsDone())
        {
            theTP.AddWarning(aCShell, " OuterShell from BrepWithVoids not mapped to TopoDS");
            myError = StepToTopoDS_BuilderOther;
            done = false;
            return;
        }

        TopoDS_Shape aShape = aTranShell.InternalValue();
        aShape.Closed(true);
        aBuilder.MakeSolid(ref aSolid);
        aBuilder.Add(aSolid, aShape);

        myResult = aSolid;
        myError = StepToTopoDS_BuilderDone;
        done = true;

        // Voids
        for (var anIt = new NCollection_Array1<StepShape_OrientedClosedShell>.Iterator(theBRepWithVoids.Voids());
             anIt.More() && aPS.More(); anIt.Next())
        {
            StepShape_OrientedClosedShell anOCShell = anIt.Value();
            aTranShell.Init(anOCShell, aTool, aDummyNMTool, theLocalFactors, aPS.Next());
            if (aTranShell.IsDone())
            {
                aShape = aTranShell.InternalValue();
                aShape.Closed(true);
                //: e0 abv 25 Mar 98: void should be an OrientedShell
                if (anOCShell is not null && !anOCShell.Orientation())
                {
                    aShape.Reverse();
                }

                aBuilder.Add(aSolid, aShape);
            }
            else
            {
                theTP.AddWarning(anOCShell, " A Void from BrepWithVoids not mapped to TopoDS");
            }
        }

        myResult = aSolid;

        // Get Statistics :
        if (theTP.TraceLevel() > 2)
        {
            SendStatistics(sout, aTool);
        }

        //: S4136  ShapeFix::SameParameter (S,false);
        StepData_StepModel aStepModel = theTP.Model() as StepData_StepModel;
        ResetPreci(aStepModel, aSolid, MaxTol());
    }

    public void Init(StepShape_BrepWithVoids theBRepWithVoids, Transfer_TransientProcess theTP)
        => Init(theBRepWithVoids, theTP, new StepData_Factors());

    /// <summary>Translates a faceted_brep into a solid.</summary>
    public void Init(StepShape_FacetedBrep theFB, Transfer_TransientProcess theTP, in StepData_Factors theLocalFactors,
                     Message_ProgressRange theProgress = default)
    {
        // Initialisation of the Tool
        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        // Start Mapping
        StepShape_ClosedShell aCShell = theFB.Outer() as StepShape_ClosedShell;

        var aTranShell = new StepToTopoDS_TranslateShell();
        aTranShell.SetPrecision(Precision()); // gka
        aTranShell.SetMaxTol(MaxTol());
        // Non-manifold topology is not referenced by FacetedBrep (ss; 14.11.2010)
        var aDummyNMTool = new StepToTopoDS_NMTool();
        aTranShell.Init(aCShell, aTool, aDummyNMTool, theLocalFactors, theProgress);

        if (!aTranShell.IsDone())
        {
            theTP.AddWarning(aCShell, " OuterShell from FacetedBrep not mapped to TopoDS");
            myError = StepToTopoDS_BuilderOther;
            done = false;
            return;
        }

        TopoDS_Shape aShape = aTranShell.InternalValue();
        aShape.Closed(true);

        aTool.ClearEdgeMap();
        aTool.ClearVertexMap();
        TopoDS_Solid aSolid = new TopoDS_Solid();
        var aBuilder = new BRep_Builder();
        aBuilder.MakeSolid(ref aSolid);
        aBuilder.Add(aSolid, aShape);
        myResult = aSolid;
        myError = StepToTopoDS_BuilderDone;
        done = true;
    }

    public void Init(StepShape_FacetedBrep theFB, Transfer_TransientProcess theTP) => Init(theFB, theTP, new StepData_Factors());

    /// <summary>Translates a faceted_brep_and_brep_with_voids into a solid with voids.</summary>
    public void Init(StepShape_FacetedBrepAndBrepWithVoids theFBABWV, Transfer_TransientProcess theTP, in StepData_Factors theLocalFactors,
                     Message_ProgressRange theProgress = default)
    {
        // Initialisation of the Tool
        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        // Start Mapping
        StepShape_ClosedShell aCShell = theFBABWV.Outer() as StepShape_ClosedShell;

        using var aPSRoot = new Message_ProgressScope(theProgress, null, 2);

        var aTranShell = new StepToTopoDS_TranslateShell();
        aTranShell.SetPrecision(Precision()); // gka
        aTranShell.SetMaxTol(MaxTol());
        // Non-manifold topology is not referenced by FacetedBrepAndBrepWithVoids (ss; 14.11.2010)
        var aDummyNMTool = new StepToTopoDS_NMTool();
        aTranShell.Init(aCShell, aTool, aDummyNMTool, theLocalFactors, aPSRoot.Next());

        if (!aTranShell.IsDone())
        {
            theTP.AddWarning(aCShell, " OuterShell from FacetedBrepAndBrepWithVoids not mapped to TopoDS");
            done = false;
            myError = StepToTopoDS_BuilderOther;
            return;
        }

        TopoDS_Shape aShape = aTranShell.InternalValue();
        aShape.Closed(true);

        TopoDS_Solid aSolid = new TopoDS_Solid();
        var aBuilder = new BRep_Builder();
        aBuilder.MakeSolid(ref aSolid);
        aBuilder.Add(aSolid, aShape);

        using var aPS = new Message_ProgressScope(aPSRoot.Next(), null, theFBABWV.NbVoids());
        for (var anIt = new NCollection_Array1<StepShape_OrientedClosedShell>.Iterator(theFBABWV.Voids());
             anIt.More() && aPS.More(); anIt.Next())
        {
            StepShape_OrientedClosedShell anOCShell = anIt.Value();
            aTranShell.Init(anOCShell, aTool, aDummyNMTool, theLocalFactors, aPS.Next());
            if (aTranShell.IsDone())
            {
                aShape = aTranShell.InternalValue();
                aShape.Closed(true);
                aBuilder.Add(aSolid, aShape);
            }
            else
            {
                theTP.AddWarning(anOCShell, " A Void from FacetedBrepAndBrepWithVoids not mapped to TopoDS");
            }
        }

        myResult = aSolid;
        myError = StepToTopoDS_BuilderDone;
        done = true;
    }

    public void Init(StepShape_FacetedBrepAndBrepWithVoids theFBABWV, Transfer_TransientProcess theTP)
        => Init(theFBABWV, theTP, new StepData_Factors());

    /// <summary>
    /// Translates a shell_based_surface_model into a shell, or a compound of shells when it has several.
    /// </summary>
    public void Init(StepShape_ShellBasedSurfaceModel aSBSM, Transfer_TransientProcess TP, StepToTopoDS_NMTool NMTool,
                     in StepData_Factors theLocalFactors, Message_ProgressRange theProgress = default)
    {
        using Message_Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
        // Initialisation of the Tool

        var myTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        myTool.Init(aMap, TP);

        // Start Mapping

        int Nb = aSBSM.NbSbsmBoundary();
        StepShape_Shell aShell;
        StepShape_OpenShell aOpenShell;
        StepShape_ClosedShell aClosedShell;
        TopoDS_Compound S = new TopoDS_Compound();
        TopoDS_Shape Shl = new TopoDS_Shape();
        var B = new BRep_Builder();
        B.MakeCompound(ref S);
        var myTranShell = new StepToTopoDS_TranslateShell();

        myTranShell.SetPrecision(Precision());
        myTranShell.SetMaxTol(MaxTol());

        using var PS = new Message_ProgressScope(theProgress, "Shell", Nb);
        for (int i = 1; i <= Nb && PS.More(); i++)
        {
            Message_ProgressRange aRange = PS.Next();
            aShell = aSBSM.SbsmBoundaryValue(i);
            aOpenShell = aShell?.OpenShell();
            aClosedShell = aShell?.ClosedShell();
            if (aOpenShell is not null)
            {
                myTranShell.Init(aOpenShell, myTool, NMTool, theLocalFactors, aRange);
                if (myTranShell.IsDone())
                {
                    Shl = TopoDS.Shell(myTranShell.InternalValue());
                    Shl.Closed(false);
                    B.Add(S, Shl);
                }
                else
                {
                    TP.AddWarning(aOpenShell, " OpenShell from ShellBasedSurfaceModel not mapped to TopoDS");
                }
            }
            else if (aClosedShell is not null)
            {
                myTranShell.Init(aClosedShell, myTool, NMTool, theLocalFactors, aRange);
                if (myTranShell.IsDone())
                {
                    Shl = TopoDS.Shell(myTranShell.InternalValue());
                    Shl.Closed(true);
                    B.Add(S, Shl);
                }
                else
                {
                    TP.AddWarning(aClosedShell, " ClosedShell from ShellBasedSurfaceModel not mapped to TopoDS");
                }
            }
        }

        if (Nb > 1)
        {
            myResult = S;
        }
        else
        {
            myResult = Shl;
        }

        myError = StepToTopoDS_BuilderDone;
        done = true;

        // Get Statistics :
        if (TP.TraceLevel() > 2)
        {
            SendStatistics(sout, myTool);
        }

        //: S4136  ShapeFix::SameParameter (S,false);
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        ResetPreci(aStepModel, S, MaxTol());
        ResetPreci(aStepModel, Shl, MaxTol()); // skl
    }

    public void Init(StepShape_ShellBasedSurfaceModel aSBSM, Transfer_TransientProcess TP, StepToTopoDS_NMTool NMTool)
        => Init(aSBSM, TP, NMTool, new StepData_Factors());

    /// <summary>Translates an edge_based_wireframe_model into a wire, or a compound of wires.</summary>
    public void Init(StepShape_EdgeBasedWireframeModel aEBWM, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors)
    {
        myResult.Nullify();

        NCollection_HArray1<StepShape_ConnectedEdgeSet> boundary = aEBWM.EbwmBoundary();
        if (boundary is null || boundary.Length() < 1)
        {
            TP.AddWarning(aEBWM, "List of boundaries is empty");
            return;
        }

        var myTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        myTool.Init(aMap, TP);

        var myTranEdge = new StepToTopoDS_TranslateEdge();
        myTranEdge.SetPrecision(Precision());
        myTranEdge.SetMaxTol(MaxTol());

        TopoDS_Compound C = new TopoDS_Compound();
        var B = new BRep_Builder();
        B.MakeCompound(ref C);

        var dummyNMTool = new StepToTopoDS_NMTool();

        for (int i = 1; i <= boundary.Length(); i++)
        {
            StepShape_ConnectedEdgeSet ces = boundary.Value(i);
            if (ces is null)
            {
                continue;
            }

            NCollection_HArray1<StepShape_Edge> edges = ces.CesEdges();
            if (edges is null || edges.Length() < 1)
            {
                TP.AddWarning(ces, "No edges in connected_edge_set");
                continue;
            }

            TopoDS_Wire W = new TopoDS_Wire();
            for (int j = 1; j <= edges.Length(); j++)
            {
                myTranEdge.Init(edges.Value(j), myTool, dummyNMTool, theLocalFactors);
                if (!myTranEdge.IsDone())
                {
                    continue;
                }

                TopoDS_Edge E = TopoDS.Edge(myTranEdge.InternalValue());
                if (E.IsNull())
                {
                    continue; // NULL, on saute
                }

                if (W.IsNull())
                {
                    B.MakeWire(ref W);
                }

                B.Add(W, E);
            }

            if (W.IsNull())
            {
                continue;
            }

            W.Closed(BRep_Tool.IsClosed(W));
            B.Add(C, W);
            if (myResult.IsNull())
            {
                myResult = W;
            }
            else
            {
                myResult = C;
            }
        }

        myError = (myResult.IsNull() ? StepToTopoDS_BuilderDone : StepToTopoDS_BuilderOther); // OCCT: the codes are swapped
        done = !myResult.IsNull();

        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        ResetPreci(aStepModel, myResult, MaxTol());
    }

    public void Init(StepShape_EdgeBasedWireframeModel aEBWM, Transfer_TransientProcess TP) => Init(aEBWM, TP, new StepData_Factors());

    /// <summary>Translates a face_based_surface_model into a shell, or a compound of shells.</summary>
    public void Init(StepShape_FaceBasedSurfaceModel aFBSM, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors)
    {
        myResult.Nullify();

        NCollection_HArray1<StepShape_ConnectedFaceSet> boundary = aFBSM.FbsmFaces();
        if (boundary is null || boundary.Length() < 1)
        {
            TP.AddWarning(aFBSM, "List of faces is empty");
            return;
        }

        var myTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        myTool.Init(aMap, TP);

        var myTranFace = new StepToTopoDS_TranslateFace();
        myTranFace.SetPrecision(Precision());
        myTranFace.SetMaxTol(MaxTol());

        TopoDS_Compound C = new TopoDS_Compound();
        var B = new BRep_Builder();
        B.MakeCompound(ref C);

        // Non-manifold topology is not mapped via FaceBasedSurfaceModel (ssv; 14.11.2010)
        var dummyNMTool = new StepToTopoDS_NMTool();

        for (int i = 1; i <= boundary.Length(); i++)
        {
            StepShape_ConnectedFaceSet cfs = boundary.Value(i);
            if (cfs is null)
            {
                continue;
            }

            NCollection_HArray1<StepShape_Face> faces = cfs.CfsFaces();
            if (faces is null || faces.Length() < 1)
            {
                TP.AddWarning(cfs, "No faces in connected_face_set");
                continue;
            }

            TopoDS_Shell S = new TopoDS_Shell();
            for (int j = 1; j <= faces.Length(); j++)
            {
                StepShape_FaceSurface fs = faces.Value(j) as StepShape_FaceSurface;
                myTranFace.Init(fs, myTool, dummyNMTool, theLocalFactors);
                if (!myTranFace.IsDone())
                {
                    continue;
                }

                TopoDS_Face F = TopoDS.Face(myTranFace.InternalValue());
                if (F.IsNull())
                {
                    continue; // NULL, on saute
                }

                if (S.IsNull())
                {
                    B.MakeShell(ref S);
                }

                B.Add(S, F);
            }

            if (S.IsNull())
            {
                continue;
            }

            S.Closed(BRep_Tool.IsClosed(S));
            B.Add(C, S);
            if (myResult.IsNull())
            {
                myResult = S;
            }
            else
            {
                myResult = C;
            }
        }

        myError = (myResult.IsNull() ? StepToTopoDS_BuilderDone : StepToTopoDS_BuilderOther); // OCCT: the codes are swapped
        done = !myResult.IsNull();

        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        ResetPreci(aStepModel, myResult, MaxTol());
    }

    public void Init(StepShape_FaceBasedSurfaceModel aFBSM, Transfer_TransientProcess TP) => Init(aFBSM, TP, new StepData_Factors());

    // ***start DTH Apr/6
    //: i6 abv 17 Sep 98: ProSTEP TR9 r0601-ct.stp: to be able read GS: GeometricCurveSet ->
    //: GeometricSet
    private static TopoDS_Face TranslateBoundedSurf(StepGeom_Surface surf, double TolDegen, in StepData_Factors theLocalFactors)
    {
        TopoDS_Face res = new TopoDS_Face();

        Geom_Surface theSurf = StepToGeom.MakeSurface(surf, theLocalFactors);
        if (theSurf is null || //: i6: protection
            theSurf is not Geom_BoundedSurface)
        {
            return res;
        }

        var myMkFace = new BRepBuilderAPI_MakeFace();
        Geom_RectangularTrimmedSurface RS = theSurf as Geom_RectangularTrimmedSurface;

        if (RS is not null)
        {
            theSurf.Bounds(out double umin, out double umax, out double vmin, out double vmax);

            myMkFace.Init(RS.BasisSurface(), umin, umax, vmin, vmax, TolDegen);
        }
        else
        {
            myMkFace.Init(theSurf, true, TolDegen);
        }

        return myMkFace.InternalFace();
    }

    /// <summary>
    /// Translates a geometric_set (or geometric_curve_set) into a compound: curves (composite curves as wires) as
    /// edges, points as vertices, bounded surfaces as faces, and other geometric representation items through the
    /// reading actor <paramref name="RA"/> (STEPControl_ActorRead's TransferShape, unit 8F).
    /// </summary>
    public void Init(StepShape_GeometricSet GCS, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors,
                     Transfer_ActorOfTransientProcess RA = null, bool isManifold = false, Message_ProgressRange theProgress = default)
    {
        // Start Mapping
        TopoDS_Compound S = new TopoDS_Compound();
        var B = new BRep_Builder();
        B.MakeCompound(ref S);
        TopoDS_Edge theEdge;
        int i;
        double preci = Precision(); // gka
        double maxtol = MaxTol();
        int nbElem = GCS.NbElements();
        using var aPS = new Message_ProgressScope(theProgress, null, nbElem);
        for (i = 1; i <= nbElem && aPS.More(); i++)
        {
            Message_ProgressRange aRange = aPS.Next();
            StepShape_GeometricSetSelect aGSS = GCS.ElementsValue(i);
            Standard_Transient ent = aGSS?.Value();

            TopoDS_Shape res = TransferBRep.ShapeResult(TP, ent);
            if (!res.IsNull())
            { // already translated
                B.Add(S, res);
                continue;
            }

            //: o7 abv 18 Feb 99: bm1_sy_fuel.stp #1427(1,2) protection against null entity
            if (ent is null)
            {
                string buff = string.Format(CultureInfo.InvariantCulture, "Entity {0} is a Null entity", i);
                TP.AddWarning(GCS, buff);
                continue;
            }
            // try curve
            else if (ent is StepGeom_Curve)
            {
                StepGeom_Curve aCrv = ent as StepGeom_Curve;

                // try composite_curve
                StepGeom_CompositeCurve CC = aCrv as StepGeom_CompositeCurve;
                if (CC is not null)
                {
                    var TrCC = new StepToTopoDS_TranslateCompositeCurve();
                    TrCC.SetPrecision(preci);
                    TrCC.SetMaxTol(maxtol);
                    TrCC.Init(CC, TP, theLocalFactors);
                    if (TrCC.IsDone())
                    {
                        if (TrCC.IsInfiniteSegment())
                        {
                            var aB = new BRep_Builder();
                            TopoDS_Compound aComp = new TopoDS_Compound();
                            aB.MakeCompound(ref aComp);
                            var anExp = new TopExp_Explorer();
                            for (anExp.Init(TrCC.InternalValue(), TopAbs_EDGE); anExp.More(); anExp.Next())
                            {
                                aB.Add(aComp, anExp.Current());
                            }

                            res = aComp;
                        }
                        else
                        {
                            res = TrCC.InternalValue();
                        }
                    }
                }
                else
                { // try other curves
                    Geom_Curve aGeomCrv = null;
                    try
                    {
                        aGeomCrv = StepToGeom.MakeCurve(aCrv, theLocalFactors);
                    }
                    catch (Exception anException) when (CaughtBySignals(anException))
                    {
                        using Message_Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
                        sout.Append("StepToTopoDS, GeometricSet, elem ").Append(i).Append(" of ").Append(nbElem).Append(": exception ");
                        sout.Append(Transfer_ProcessForTransient.What(anException)).Append('\n');
                    }

                    if (aGeomCrv is not null)
                    {
                        var anEdge = new BRepBuilderAPI_MakeEdge(aGeomCrv, aGeomCrv.FirstParameter(), aGeomCrv.LastParameter());
                        if (anEdge.IsDone())
                        {
                            res = anEdge.InternalEdge();
                        }
                    }
                }
            }
            // try point
            else if (ent is StepGeom_CartesianPoint)
            {
                StepGeom_CartesianPoint aPnt = ent as StepGeom_CartesianPoint;
                Geom_CartesianPoint thePnt = StepToGeom.MakeCartesianPoint(aPnt, theLocalFactors);
                if (thePnt is not null)
                {
                    var myMkVtx = new BRepBuilderAPI_MakeVertex(thePnt.Pnt());
                    if (myMkVtx.IsDone())
                    {
                        res = myMkVtx.InternalVertex();
                    }
                }
            }
            // Element should finally be a Surface
            else if (ent is StepGeom_Surface)
            {
                StepGeom_Surface aSurf = ent as StepGeom_Surface;

                // try curve_bounded_surf
                if (ent is StepGeom_CurveBoundedSurface)
                {
                    StepGeom_CurveBoundedSurface CBS = aSurf as StepGeom_CurveBoundedSurface;
                    var TrCBS = new StepToTopoDS_TranslateCurveBoundedSurface();
                    TrCBS.SetPrecision(preci);
                    TrCBS.SetMaxTol(maxtol);

                    TrCBS.Init(CBS, TP, theLocalFactors);
                    if (TrCBS.IsDone())
                    {
                        res = TrCBS.InternalValue();
                    }
                }
                // try RectangularCompositeSurface
                else if (ent is StepGeom_RectangularCompositeSurface)
                {
                    StepGeom_RectangularCompositeSurface RCS = aSurf as StepGeom_RectangularCompositeSurface;
                    int nbi = RCS.NbSegmentsI();
                    int nbj = RCS.NbSegmentsJ();
                    TopoDS_Compound C = new TopoDS_Compound();
                    B.MakeCompound(ref C);
                    for (int ii = 1; ii <= nbi; ii++)
                    {
                        for (int j = 1; j <= nbj; j++)
                        {
                            StepGeom_SurfacePatch patch = RCS.SegmentsValue(ii, j);
                            TopoDS_Face f = TranslateBoundedSurf(patch.ParentSurface(), preci, theLocalFactors);
                            if (!f.IsNull())
                            {
                                B.Add(C, f);
                            }
                        }
                    }

                    res = C;
                }
                // try other surfs
                else
                {
                    res = TranslateBoundedSurf(aSurf, preci, theLocalFactors);
                }
            }
            else if (ent is StepGeom_GeometricRepresentationItem)
            {
                StepGeom_GeometricRepresentationItem GRI = ent as StepGeom_GeometricRepresentationItem;
                if (RA is not null)
                {
                    // OCCT down-casts the actor to STEPControl_ActorRead (unit 8F), which implements this (decision 1261)
                    StepToTopoDS_IActorRead anActor = RA as StepToTopoDS_IActorRead;
                    Transfer_Binder binder = null;
                    if (anActor is not null)
                    {
                        binder = anActor.TransferShape(GRI, TP, theLocalFactors, isManifold, false, aRange);
                    }

                    if (binder is not null)
                    {
                        res = TransferBRep.ShapeResult(binder);
                    }
                }
            }
            else
            {
                TP.AddWarning(ent, " Entity is not a Curve, Point, Surface or GeometricRepresentationItem");
            }

            if (!res.IsNull())
            {
                B.Add(S, res);
                TransferBRep.SetShapeResult(TP, ent, res);
            }
            else
            {
                TP.AddWarning(ent, " Entity not mapped to TopoDS");
            }
        }

        myResult = S;
        myError = StepToTopoDS_BuilderDone;
        done = true;
    }

    public void Init(StepShape_GeometricSet GCS, Transfer_TransientProcess TP) => Init(GCS, TP, new StepData_Factors());

    // ***end DTH Apr/6

    /// <summary>Builds a TopoDS_Solid from a tessellated_solid.</summary>
    public void Init(StepVisual_TessellatedSolid theTSo, Transfer_TransientProcess theTP, bool theReadTessellatedWhenNoBRepOnly,
                     ref bool theHasGeom, in StepData_Factors theLocalFactors, Message_ProgressRange theProgress = default)
    {
        var aTranSolid = new StepToTopoDS_TranslateSolid();
        aTranSolid.SetPrecision(Precision());
        aTranSolid.SetMaxTol(MaxTol());

        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        var aDummyNMTool = new StepToTopoDS_NMTool();
        aTranSolid.Init(theTSo, theTP, aTool, aDummyNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, theLocalFactors, theProgress);

        if (!aTranSolid.IsDone())
        {
            theTP.AddWarning(theTSo, " TessellatedSolid not mapped to TopoDS");
            myError = StepToTopoDS_BuilderOther;
            done = false;
            return;
        }

        TopoDS_Shape aS = aTranSolid.InternalValue();
        myResult = TopoDS.Solid(aS);
        myError = StepToTopoDS_BuilderDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedSolid theTSo, Transfer_TransientProcess theTP, bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        => Init(theTSo, theTP, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors());

    /// <summary>Builds a TopoDS_Shell from a tessellated_shell.</summary>
    public void Init(StepVisual_TessellatedShell theTSh, Transfer_TransientProcess theTP, bool theReadTessellatedWhenNoBRepOnly,
                     ref bool theHasGeom, in StepData_Factors theLocalFactors, Message_ProgressRange theProgress = default)
    {
        var aTranShell = new StepToTopoDS_TranslateShell();
        aTranShell.SetPrecision(Precision());
        aTranShell.SetMaxTol(MaxTol());

        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        var aDummyNMTool = new StepToTopoDS_NMTool();
        aTranShell.Init(theTSh, aTool, aDummyNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, theLocalFactors, theProgress);

        if (!aTranShell.IsDone())
        {
            theTP.AddWarning(theTSh, " TessellatedShell not mapped to TopoDS");
            myError = StepToTopoDS_BuilderOther;
            done = false;
            return;
        }

        TopoDS_Shape aS = aTranShell.InternalValue();
        myResult = TopoDS.Shell(aS);
        myError = StepToTopoDS_BuilderDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedShell theTSh, Transfer_TransientProcess theTP, bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        => Init(theTSh, theTP, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors());

    /// <summary>Builds a TopoDS_Face from a tessellated_face.</summary>
    public void Init(StepVisual_TessellatedFace theTF, Transfer_TransientProcess theTP, bool theReadTessellatedWhenNoBRepOnly,
                     ref bool theHasGeom, in StepData_Factors theLocalFactors)
    {
        var aTranFace = new StepToTopoDS_TranslateFace();
        aTranFace.SetPrecision(Precision());
        aTranFace.SetMaxTol(MaxTol());

        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        var aDummyNMTool = new StepToTopoDS_NMTool();
        aTranFace.Init(theTF, aTool, aDummyNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, theLocalFactors);

        if (!aTranFace.IsDone())
        {
            theTP.AddWarning(theTF, " TessellatedFace not mapped to TopoDS");
            myError = StepToTopoDS_BuilderOther;
            done = false;
            return;
        }

        TopoDS_Shape aS = aTranFace.InternalValue();
        myResult = TopoDS.Face(aS);
        myError = StepToTopoDS_BuilderDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedFace theTF, Transfer_TransientProcess theTP, bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        => Init(theTF, theTP, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors());

    /// <summary>Builds a TopoDS_Face from a tessellated_surface_set.</summary>
    public void Init(StepVisual_TessellatedSurfaceSet theTSS, Transfer_TransientProcess theTP, ref bool theHasGeom, in StepData_Factors theLocalFactors)
    {
        var aTranFace = new StepToTopoDS_TranslateFace();
        aTranFace.SetPrecision(Precision());
        aTranFace.SetMaxTol(MaxTol());

        var aTool = new StepToTopoDS_Tool();
        var aMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();
        aTool.Init(aMap, theTP);

        var aDummyNMTool = new StepToTopoDS_NMTool();
        aTranFace.Init(theTSS, aTool, aDummyNMTool, theLocalFactors);
        theHasGeom = false;

        if (!aTranFace.IsDone())
        {
            theTP.AddWarning(theTSS, " TessellatedSurfaceSet not mapped to TopoDS");
            myError = StepToTopoDS_BuilderOther;
            done = false;
            return;
        }

        TopoDS_Shape aS = aTranFace.InternalValue();
        myResult = TopoDS.Face(aS);
        myError = StepToTopoDS_BuilderDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedSurfaceSet theTSS, Transfer_TransientProcess theTP, ref bool theHasGeom)
        => Init(theTSS, theTP, ref theHasGeom, new StepData_Factors());

    /// <summary>The result of the mapping, frozen.</summary>
    /// <exception cref="StdFail_NotDone">The translation wasn't done.</exception>
    public TopoDS_Shape Value()
    {
        TopoDS_Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape (decision 1259).</summary>
    internal TopoDS_Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_Builder::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_BuilderError Error() => myError;
}

/// <summary>
/// What StepToTopoDS_Builder asks of STEPControl_ActorRead (unit 8F) for the geometric representation items of a
/// geometric set: OCCT down-casts the actor to that class; the port asks for this interface, which the actor
/// implements (decision 1261).
/// </summary>
internal interface StepToTopoDS_IActorRead
{
    /// <summary>STEPControl_ActorRead::TransferShape.</summary>
    Transfer_Binder? TransferShape(Standard_Transient start, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors,
                                   bool isManifold, bool theUseTrsf, Message_ProgressRange theProgress);
}
