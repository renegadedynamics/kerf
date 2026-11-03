// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Builder.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Globalization;
using static Kerf.Engine.Exchange.Step.Import.BuilderError;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>
/// The transfer of a STEP shape representation item into a BRep: a manifold solid brep, a brep with voids, a faceted
/// brep (with voids), a shell based surface model, an edge based wireframe model, a face based surface model, a
/// geometric set, or a tessellated solid, shell, face or surface set.
/// </summary>
/// <remarks>
/// <para>
/// Each <c>Init</c> translates its item with a new <see cref="ImportTool"/> on the transient process, which
/// receives the shapes made from the topological items as their results. Units go through
/// <see cref="Factors"/>; the model's <see cref="StepModel.InternalParameters"/> gives the modes.
/// Needs <see cref="ShapeAlgo.Init"/> (as STEPControl_Controller's initialisation does natively).
/// </para>
/// <para>
/// <see cref="Value"/> freezes the result. A reader that goes on working on the shapes of a transfer (sharing them
/// between items, through the transient process or a non-manifold tool) uses <c>InternalValue</c> and freezes once
/// at the end of the transfer (decision 1247).
/// </para>
/// </remarks>
public sealed class ImportBuilder : ImportRoot
{
    private BuilderError _error;
    private Shape _result = new Shape();

    private static void ResetPreci(StepModel stepModel, in Shape s, double maxtol)
    {
        //: S4136
        int modetol = (int)stepModel.InternalParameters.ReadMaxPrecisionMode;
        if (modetol != 0)
        {
            var stu = new ShapeToleranceFixer();
            stu.LimitTolerance(s, Kerf.Engine.Foundation.Precision.Confusion(), maxtol);
        }
    }

    public ImportBuilder()
    {
        _error = BuilderError.BuilderOther;
        _done = false;
    }

    // The geometric statistics, written when the trace level is above 2
    private static void SendStatistics(Messenger.StreamBuffer sout, ImportTool aTool)
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
    public void Init(StepManifoldSolidBrep manifoldSolid, TransientProcess tp, in Factors localFactors,
                     ProgressRange progress = default)
    {
        using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
        StepModel stepModel = tp.Model() as StepModel;
        // Initialisation of the Tool

        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        // Start Mapping
        StepConnectedFaceSet shell = manifoldSolid.Outer();

        var myTranShell = new TranslateShell();
        myTranShell.SetPrecision(Precision());
        myTranShell.SetMaxTol(MaxTol());
        var dummyNMTool = new NMTool();
        myTranShell.Init(shell, tool, dummyNMTool, localFactors, progress);

        if (!myTranShell.IsDone)
        {
            tp.AddWarning(shell, " OuterShell from ManifoldSolidBrep not mapped to TopoDS");
            _error = BuilderError.BuilderOther;
            _done = false;
            return;
        }

        Shape shape = myTranShell.InternalValue();
        shape.Closed(true);
        Solid solid = new Solid();
        var builder = new BrepBuilder();
        builder.MakeSolid(ref solid);
        builder.Add(solid, shape);
        _result = solid;
        _error = BuilderError.BuilderDone;
        _done = true;

        // Get Statistics :
        if (tp.TraceLevel() > 2)
        {
            SendStatistics(sout, tool);
        }

        ResetPreci(stepModel, solid, MaxTol());
    }

    public void Init(StepManifoldSolidBrep manifoldSolid, TransientProcess tp)
        => Init(manifoldSolid, tp, new Factors());

    /// <summary>Translates a brep_with_voids into a solid with an outer shell and voids.</summary>
    public void Init(StepBrepWithVoids bRepWithVoids, TransientProcess tp, in Factors localFactors,
                     ProgressRange progress = default)
    {
        using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
        // Initialisation of the Tool
        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        // Start Mapping
        Solid solid = new Solid();
        var builder = new BrepBuilder();
        builder.MakeSolid(ref solid);

        using var ps = new ProgressScope(progress, "Shell", bRepWithVoids.NbVoids() + 1);

        var tranShell = new TranslateShell();
        tranShell.SetPrecision(Precision()); // gka
        tranShell.SetMaxTol(MaxTol());
        // OuterBound
        StepClosedShell cShell = bRepWithVoids.Outer() as StepClosedShell;
        // Non-manifold topology is not referenced by BrepWithVoids (ssv; 14.11.2010)
        var dummyNMTool = new NMTool();
        tranShell.Init(cShell, tool, dummyNMTool, localFactors, ps.Next());

        if (!tranShell.IsDone)
        {
            tp.AddWarning(cShell, " OuterShell from BrepWithVoids not mapped to TopoDS");
            _error = BuilderError.BuilderOther;
            _done = false;
            return;
        }

        Shape shape = tranShell.InternalValue();
        shape.Closed(true);
        builder.MakeSolid(ref solid);
        builder.Add(solid, shape);

        _result = solid;
        _error = BuilderError.BuilderDone;
        _done = true;

        // Voids
        for (var it = new NCollection_Array1<StepOrientedClosedShell>.Iterator(bRepWithVoids.Voids());
             it.More() && ps.More(); it.Next())
        {
            StepOrientedClosedShell ocShell = it.Value();
            tranShell.Init(ocShell, tool, dummyNMTool, localFactors, ps.Next());
            if (tranShell.IsDone)
            {
                shape = tranShell.InternalValue();
                shape.Closed(true);
                //: e0 abv 25 Mar 98: void should be an OrientedShell
                if (ocShell is not null && !ocShell.Orientation())
                {
                    shape.Reverse();
                }

                builder.Add(solid, shape);
            }
            else
            {
                tp.AddWarning(ocShell, " A Void from BrepWithVoids not mapped to TopoDS");
            }
        }

        _result = solid;

        // Get Statistics :
        if (tp.TraceLevel() > 2)
        {
            SendStatistics(sout, tool);
        }

        //: S4136  ShapeFix::SameParameter (S,false);
        StepModel stepModel = tp.Model() as StepModel;
        ResetPreci(stepModel, solid, MaxTol());
    }

    public void Init(StepBrepWithVoids bRepWithVoids, TransientProcess tp)
        => Init(bRepWithVoids, tp, new Factors());

    /// <summary>Translates a faceted_brep into a solid.</summary>
    public void Init(StepFacetedBrep fb, TransientProcess tp, in Factors localFactors,
                     ProgressRange progress = default)
    {
        // Initialisation of the Tool
        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        // Start Mapping
        StepClosedShell cShell = fb.Outer() as StepClosedShell;

        var tranShell = new TranslateShell();
        tranShell.SetPrecision(Precision()); // gka
        tranShell.SetMaxTol(MaxTol());
        // Non-manifold topology is not referenced by FacetedBrep (ss; 14.11.2010)
        var dummyNMTool = new NMTool();
        tranShell.Init(cShell, tool, dummyNMTool, localFactors, progress);

        if (!tranShell.IsDone)
        {
            tp.AddWarning(cShell, " OuterShell from FacetedBrep not mapped to TopoDS");
            _error = BuilderError.BuilderOther;
            _done = false;
            return;
        }

        Shape shape = tranShell.InternalValue();
        shape.Closed(true);

        tool.ClearEdgeMap();
        tool.ClearVertexMap();
        Solid solid = new Solid();
        var builder = new BrepBuilder();
        builder.MakeSolid(ref solid);
        builder.Add(solid, shape);
        _result = solid;
        _error = BuilderError.BuilderDone;
        _done = true;
    }

    public void Init(StepFacetedBrep fb, TransientProcess tp) => Init(fb, tp, new Factors());

    /// <summary>Translates a faceted_brep_and_brep_with_voids into a solid with voids.</summary>
    public void Init(StepFacetedBrepAndBrepWithVoids fbabwv, TransientProcess tp, in Factors localFactors,
                     ProgressRange progress = default)
    {
        // Initialisation of the Tool
        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        // Start Mapping
        StepClosedShell cShell = fbabwv.Outer() as StepClosedShell;

        using var psRoot = new ProgressScope(progress, null, 2);

        var tranShell = new TranslateShell();
        tranShell.SetPrecision(Precision()); // gka
        tranShell.SetMaxTol(MaxTol());
        // Non-manifold topology is not referenced by FacetedBrepAndBrepWithVoids (ss; 14.11.2010)
        var dummyNMTool = new NMTool();
        tranShell.Init(cShell, tool, dummyNMTool, localFactors, psRoot.Next());

        if (!tranShell.IsDone)
        {
            tp.AddWarning(cShell, " OuterShell from FacetedBrepAndBrepWithVoids not mapped to TopoDS");
            _done = false;
            _error = BuilderError.BuilderOther;
            return;
        }

        Shape shape = tranShell.InternalValue();
        shape.Closed(true);

        Solid solid = new Solid();
        var builder = new BrepBuilder();
        builder.MakeSolid(ref solid);
        builder.Add(solid, shape);

        using var ps = new ProgressScope(psRoot.Next(), null, fbabwv.VoidCount);
        for (var it = new NCollection_Array1<StepOrientedClosedShell>.Iterator(fbabwv.Voids());
             it.More() && ps.More(); it.Next())
        {
            StepOrientedClosedShell ocShell = it.Value();
            tranShell.Init(ocShell, tool, dummyNMTool, localFactors, ps.Next());
            if (tranShell.IsDone)
            {
                shape = tranShell.InternalValue();
                shape.Closed(true);
                builder.Add(solid, shape);
            }
            else
            {
                tp.AddWarning(ocShell, " A Void from FacetedBrepAndBrepWithVoids not mapped to TopoDS");
            }
        }

        _result = solid;
        _error = BuilderError.BuilderDone;
        _done = true;
    }

    public void Init(StepFacetedBrepAndBrepWithVoids fbabwv, TransientProcess tp)
        => Init(fbabwv, tp, new Factors());

    /// <summary>
    /// Translates a shell_based_surface_model into a shell, or a compound of shells when it has several.
    /// </summary>
    public void Init(StepShellBasedSurfaceModel aSBSM, TransientProcess tp, NMTool nmTool,
                     in Factors localFactors, ProgressRange progress = default)
    {
        using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
        // Initialisation of the Tool

        var myTool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        myTool.Init(map, tp);

        // Start Mapping

        int nb = aSBSM.SbsmBoundaryCount;
        StepShell shell;
        StepOpenShell openShell;
        StepClosedShell closedShell;
        Compound s = new Compound();
        Shape shl = new Shape();
        var b = new BrepBuilder();
        b.MakeCompound(ref s);
        var myTranShell = new TranslateShell();

        myTranShell.SetPrecision(Precision());
        myTranShell.SetMaxTol(MaxTol());

        using var ps = new ProgressScope(progress, "Shell", nb);
        for (int i = 1; i <= nb && ps.More(); i++)
        {
            ProgressRange range = ps.Next();
            shell = aSBSM.SbsmBoundaryValue(i);
            openShell = shell?.OpenShell();
            closedShell = shell?.ClosedShell();
            if (openShell is not null)
            {
                myTranShell.Init(openShell, myTool, nmTool, localFactors, range);
                if (myTranShell.IsDone)
                {
                    shl = ShapeCasts.Shell(myTranShell.InternalValue());
                    shl.Closed(false);
                    b.Add(s, shl);
                }
                else
                {
                    tp.AddWarning(openShell, " OpenShell from ShellBasedSurfaceModel not mapped to TopoDS");
                }
            }
            else if (closedShell is not null)
            {
                myTranShell.Init(closedShell, myTool, nmTool, localFactors, range);
                if (myTranShell.IsDone)
                {
                    shl = ShapeCasts.Shell(myTranShell.InternalValue());
                    shl.Closed(true);
                    b.Add(s, shl);
                }
                else
                {
                    tp.AddWarning(closedShell, " ClosedShell from ShellBasedSurfaceModel not mapped to TopoDS");
                }
            }
        }

        if (nb > 1)
        {
            _result = s;
        }
        else
        {
            _result = shl;
        }

        _error = BuilderError.BuilderDone;
        _done = true;

        // Get Statistics :
        if (tp.TraceLevel() > 2)
        {
            SendStatistics(sout, myTool);
        }

        //: S4136  ShapeFix::SameParameter (S,false);
        StepModel stepModel = tp.Model() as StepModel;
        ResetPreci(stepModel, s, MaxTol());
        ResetPreci(stepModel, shl, MaxTol()); // skl
    }

    public void Init(StepShellBasedSurfaceModel aSBSM, TransientProcess tp, NMTool nmTool)
        => Init(aSBSM, tp, nmTool, new Factors());

    /// <summary>Translates an edge_based_wireframe_model into a wire, or a compound of wires.</summary>
    public void Init(StepEdgeBasedWireframeModel aEBWM, TransientProcess tp, in Factors localFactors)
    {
        _result.Nullify();

        NCollection_HArray1<StepConnectedEdgeSet> boundary = aEBWM.EbwmBoundary();
        if (boundary is null || boundary.Length() < 1)
        {
            tp.AddWarning(aEBWM, "List of boundaries is empty");
            return;
        }

        var myTool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        myTool.Init(map, tp);

        var myTranEdge = new TranslateEdge();
        myTranEdge.SetPrecision(Precision());
        myTranEdge.SetMaxTol(MaxTol());

        Compound c = new Compound();
        var b = new BrepBuilder();
        b.MakeCompound(ref c);

        var dummyNMTool = new NMTool();

        for (int i = 1; i <= boundary.Length(); i++)
        {
            StepConnectedEdgeSet ces = boundary.Value(i);
            if (ces is null)
            {
                continue;
            }

            NCollection_HArray1<StepEdge> edges = ces.CesEdges();
            if (edges is null || edges.Length() < 1)
            {
                tp.AddWarning(ces, "No edges in connected_edge_set");
                continue;
            }

            Wire w = new Wire();
            for (int j = 1; j <= edges.Length(); j++)
            {
                myTranEdge.Init(edges.Value(j), myTool, dummyNMTool, localFactors);
                if (!myTranEdge.IsDone)
                {
                    continue;
                }

                Edge e = ShapeCasts.Edge(myTranEdge.InternalValue());
                if (e.IsNull)
                {
                    continue; // NULL, on saute
                }

                if (w.IsNull)
                {
                    b.MakeWire(ref w);
                }

                b.Add(w, e);
            }

            if (w.IsNull)
            {
                continue;
            }

            w.Closed(Tool.IsClosed(w));
            b.Add(c, w);
            if (_result.IsNull)
            {
                _result = w;
            }
            else
            {
                _result = c;
            }
        }

        _error = (_result.IsNull ? BuilderError.BuilderDone : BuilderError.BuilderOther); // OCCT: the codes are swapped
        _done = !_result.IsNull;

        StepModel stepModel = tp.Model() as StepModel;
        ResetPreci(stepModel, _result, MaxTol());
    }

    public void Init(StepEdgeBasedWireframeModel aEBWM, TransientProcess tp) => Init(aEBWM, tp, new Factors());

    /// <summary>Translates a face_based_surface_model into a shell, or a compound of shells.</summary>
    public void Init(StepFaceBasedSurfaceModel aFBSM, TransientProcess tp, in Factors localFactors)
    {
        _result.Nullify();

        NCollection_HArray1<StepConnectedFaceSet> boundary = aFBSM.FbsmFaces();
        if (boundary is null || boundary.Length() < 1)
        {
            tp.AddWarning(aFBSM, "List of faces is empty");
            return;
        }

        var myTool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        myTool.Init(map, tp);

        var myTranFace = new TranslateFace();
        myTranFace.SetPrecision(Precision());
        myTranFace.SetMaxTol(MaxTol());

        Compound c = new Compound();
        var b = new BrepBuilder();
        b.MakeCompound(ref c);

        // Non-manifold topology is not mapped via FaceBasedSurfaceModel (ssv; 14.11.2010)
        var dummyNMTool = new NMTool();

        for (int i = 1; i <= boundary.Length(); i++)
        {
            StepConnectedFaceSet cfs = boundary.Value(i);
            if (cfs is null)
            {
                continue;
            }

            NCollection_HArray1<StepFace> faces = cfs.CfsFaces();
            if (faces is null || faces.Length() < 1)
            {
                tp.AddWarning(cfs, "No faces in connected_face_set");
                continue;
            }

            Shell s = new Shell();
            for (int j = 1; j <= faces.Length(); j++)
            {
                StepFaceSurface fs = faces.Value(j) as StepFaceSurface;
                myTranFace.Init(fs, myTool, dummyNMTool, localFactors);
                if (!myTranFace.IsDone)
                {
                    continue;
                }

                Face f = ShapeCasts.Face(myTranFace.InternalValue());
                if (f.IsNull)
                {
                    continue; // NULL, on saute
                }

                if (s.IsNull)
                {
                    b.MakeShell(ref s);
                }

                b.Add(s, f);
            }

            if (s.IsNull)
            {
                continue;
            }

            s.Closed(Tool.IsClosed(s));
            b.Add(c, s);
            if (_result.IsNull)
            {
                _result = s;
            }
            else
            {
                _result = c;
            }
        }

        _error = (_result.IsNull ? BuilderError.BuilderDone : BuilderError.BuilderOther); // OCCT: the codes are swapped
        _done = !_result.IsNull;

        StepModel stepModel = tp.Model() as StepModel;
        ResetPreci(stepModel, _result, MaxTol());
    }

    public void Init(StepFaceBasedSurfaceModel aFBSM, TransientProcess tp) => Init(aFBSM, tp, new Factors());

    // ***start DTH Apr/6
    //: i6 abv 17 Sep 98: ProSTEP TR9 r0601-ct.stp: to be able read GS: GeometricCurveSet ->
    //: GeometricSet
    private static Face TranslateBoundedSurf(StepSurface surf, double tolDegen, in Factors localFactors)
    {
        Face res = new Face();

        Surface theSurf = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurface(surf, localFactors);
        if (theSurf is null || //: i6: protection
            theSurf is not BoundedSurface)
        {
            return res;
        }

        var myMkFace = new MakeFace();
        TrimmedSurface rs = theSurf as TrimmedSurface;

        if (rs is not null)
        {
            theSurf.Bounds(out double umin, out double umax, out double vmin, out double vmax);

            myMkFace.Init(rs.BasisSurface(), umin, umax, vmin, vmax, tolDegen);
        }
        else
        {
            myMkFace.Init(theSurf, true, tolDegen);
        }

        return myMkFace.InternalFace();
    }

    /// <summary>
    /// Translates a geometric_set (or geometric_curve_set) into a compound: curves (composite curves as wires) as
    /// edges, points as vertices, bounded surfaces as faces, and other geometric representation items through the
    /// reading actor <paramref name="ra"/> (STEPControl_ActorRead's TransferShape, unit 8F).
    /// </summary>
    public void Init(StepGeometricSet gcs, TransientProcess tp, in Factors localFactors,
                     ActorOfTransientProcess ra = null, bool isManifold = false, ProgressRange progress = default)
    {
        // Start Mapping
        Compound s = new Compound();
        var B = new BrepBuilder();
        B.MakeCompound(ref s);
        Edge theEdge;
        int i;
        double preci = Precision(); // gka
        double maxtol = MaxTol();
        int nbElem = gcs.NbElements();
        using var ps = new ProgressScope(progress, null, nbElem);
        for (i = 1; i <= nbElem && ps.More(); i++)
        {
            ProgressRange range = ps.Next();
            StepGeometricSetSelect gss = gcs.ElementsValue(i);
            Transient ent = gss?.Value();

            Shape res = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(tp, ent);
            if (!res.IsNull)
            { // already translated
                B.Add(s, res);
                continue;
            }

            //: o7 abv 18 Feb 99: bm1_sy_fuel.stp #1427(1,2) protection against null entity
            if (ent is null)
            {
                string buff = string.Format(CultureInfo.InvariantCulture, "Entity {0} is a Null entity", i);
                tp.AddWarning(gcs, buff);
                continue;
            }
            // try curve
            else if (ent is StepCurve)
            {
                StepCurve crv = ent as StepCurve;

                // try composite_curve
                StepCompositeCurve cc = crv as StepCompositeCurve;
                if (cc is not null)
                {
                    var trCC = new TranslateCompositeCurve();
                    trCC.SetPrecision(preci);
                    trCC.SetMaxTol(maxtol);
                    trCC.Init(cc, tp, localFactors);
                    if (trCC.IsDone)
                    {
                        if (trCC.IsInfiniteSegment)
                        {
                            var b = new BrepBuilder();
                            Compound comp = new Compound();
                            b.MakeCompound(ref comp);
                            var exp = new Explorer();
                            for (exp.Init(trCC.InternalValue(), ShapeType.Edge); exp.More(); exp.Next())
                            {
                                b.Add(comp, exp.Current());
                            }

                            res = comp;
                        }
                        else
                        {
                            res = trCC.InternalValue();
                        }
                    }
                }
                else
                { // try other curves
                    Curve3D geomCrv = null;
                    try
                    {
                        geomCrv = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve(crv, localFactors);
                    }
                    catch (Exception exception) when (CaughtBySignals(exception))
                    {
                        using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
                        sout.Append("StepToTopoDS, GeometricSet, elem ").Append(i).Append(" of ").Append(nbElem).Append(": exception ");
                        sout.Append(ProcessForTransient.What(exception)).Append('\n');
                    }

                    if (geomCrv is not null)
                    {
                        var edge = new MakeEdge(geomCrv, geomCrv.FirstParameter(), geomCrv.LastParameter());
                        if (edge.IsDone)
                        {
                            res = edge.InternalEdge();
                        }
                    }
                }
            }
            // try point
            else if (ent is StepCartesianPoint)
            {
                StepCartesianPoint pnt = ent as StepCartesianPoint;
                CartesianPoint thePnt = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(pnt, localFactors);
                if (thePnt is not null)
                {
                    var myMkVtx = new MakeVertex(thePnt.Pnt());
                    if (myMkVtx.IsDone)
                    {
                        res = myMkVtx.InternalVertex();
                    }
                }
            }
            // Element should finally be a Surface
            else if (ent is StepSurface)
            {
                StepSurface surf = ent as StepSurface;

                // try curve_bounded_surf
                if (ent is StepCurveBoundedSurface)
                {
                    StepCurveBoundedSurface cbs = surf as StepCurveBoundedSurface;
                    var trCBS = new TranslateCurveBoundedSurface();
                    trCBS.SetPrecision(preci);
                    trCBS.SetMaxTol(maxtol);

                    trCBS.Init(cbs, tp, localFactors);
                    if (trCBS.IsDone)
                    {
                        res = trCBS.InternalValue();
                    }
                }
                // try RectangularCompositeSurface
                else if (ent is StepRectangularCompositeSurface)
                {
                    StepRectangularCompositeSurface rcs = surf as StepRectangularCompositeSurface;
                    int nbi = rcs.NbSegmentsI();
                    int nbj = rcs.NbSegmentsJ();
                    Compound c = new Compound();
                    B.MakeCompound(ref c);
                    for (int ii = 1; ii <= nbi; ii++)
                    {
                        for (int j = 1; j <= nbj; j++)
                        {
                            StepSurfacePatch patch = rcs.SegmentsValue(ii, j);
                            Face f = TranslateBoundedSurf(patch.ParentSurface(), preci, localFactors);
                            if (!f.IsNull)
                            {
                                B.Add(c, f);
                            }
                        }
                    }

                    res = c;
                }
                // try other surfs
                else
                {
                    res = TranslateBoundedSurf(surf, preci, localFactors);
                }
            }
            else if (ent is StepGeometricRepresentationItem)
            {
                StepGeometricRepresentationItem gri = ent as StepGeometricRepresentationItem;
                if (ra is not null)
                {
                    // OCCT down-casts the actor to STEPControl_ActorRead (unit 8F), which implements this (decision 1249)
                    IActorRead actor = ra as IActorRead;
                    Binder binder = null;
                    if (actor is not null)
                    {
                        binder = actor.TransferShape(gri, tp, localFactors, isManifold, false, range);
                    }

                    if (binder is not null)
                    {
                        res = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
                    }
                }
            }
            else
            {
                tp.AddWarning(ent, " Entity is not a Curve, Point, Surface or GeometricRepresentationItem");
            }

            if (!res.IsNull)
            {
                B.Add(s, res);
                global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetShapeResult(tp, ent, res);
            }
            else
            {
                tp.AddWarning(ent, " Entity not mapped to TopoDS");
            }
        }

        _result = s;
        _error = BuilderError.BuilderDone;
        _done = true;
    }

    public void Init(StepGeometricSet gcs, TransientProcess tp) => Init(gcs, tp, new Factors());

    // ***end DTH Apr/6

    /// <summary>Builds a TopoDS_Solid from a tessellated_solid.</summary>
    public void Init(TessellatedSolid tSo, TransientProcess tp, bool readTessellatedWhenNoBRepOnly,
                     ref bool hasGeom, in Factors localFactors, ProgressRange progress = default)
    {
        var tranSolid = new TranslateSolid();
        tranSolid.SetPrecision(Precision());
        tranSolid.SetMaxTol(MaxTol());

        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        var dummyNMTool = new NMTool();
        tranSolid.Init(tSo, tp, tool, dummyNMTool, readTessellatedWhenNoBRepOnly, ref hasGeom, localFactors, progress);

        if (!tranSolid.IsDone)
        {
            tp.AddWarning(tSo, " TessellatedSolid not mapped to TopoDS");
            _error = BuilderError.BuilderOther;
            _done = false;
            return;
        }

        Shape s = tranSolid.InternalValue();
        _result = ShapeCasts.Solid(s);
        _error = BuilderError.BuilderDone;
        _done = true;
    }

    public void Init(TessellatedSolid tSo, TransientProcess tp, bool readTessellatedWhenNoBRepOnly, ref bool hasGeom)
        => Init(tSo, tp, readTessellatedWhenNoBRepOnly, ref hasGeom, new Factors());

    /// <summary>Builds a TopoDS_Shell from a tessellated_shell.</summary>
    public void Init(TessellatedShell tSh, TransientProcess tp, bool readTessellatedWhenNoBRepOnly,
                     ref bool hasGeom, in Factors localFactors, ProgressRange progress = default)
    {
        var tranShell = new TranslateShell();
        tranShell.SetPrecision(Precision());
        tranShell.SetMaxTol(MaxTol());

        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        var dummyNMTool = new NMTool();
        tranShell.Init(tSh, tool, dummyNMTool, readTessellatedWhenNoBRepOnly, ref hasGeom, localFactors, progress);

        if (!tranShell.IsDone)
        {
            tp.AddWarning(tSh, " TessellatedShell not mapped to TopoDS");
            _error = BuilderError.BuilderOther;
            _done = false;
            return;
        }

        Shape s = tranShell.InternalValue();
        _result = ShapeCasts.Shell(s);
        _error = BuilderError.BuilderDone;
        _done = true;
    }

    public void Init(TessellatedShell tSh, TransientProcess tp, bool readTessellatedWhenNoBRepOnly, ref bool hasGeom)
        => Init(tSh, tp, readTessellatedWhenNoBRepOnly, ref hasGeom, new Factors());

    /// <summary>Builds a TopoDS_Face from a tessellated_face.</summary>
    public void Init(TessellatedFace tf, TransientProcess tp, bool readTessellatedWhenNoBRepOnly,
                     ref bool hasGeom, in Factors localFactors)
    {
        var tranFace = new TranslateFace();
        tranFace.SetPrecision(Precision());
        tranFace.SetMaxTol(MaxTol());

        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        var dummyNMTool = new NMTool();
        tranFace.Init(tf, tool, dummyNMTool, readTessellatedWhenNoBRepOnly, ref hasGeom, localFactors);

        if (!tranFace.IsDone)
        {
            tp.AddWarning(tf, " TessellatedFace not mapped to TopoDS");
            _error = BuilderError.BuilderOther;
            _done = false;
            return;
        }

        Shape s = tranFace.InternalValue();
        _result = ShapeCasts.Face(s);
        _error = BuilderError.BuilderDone;
        _done = true;
    }

    public void Init(TessellatedFace tf, TransientProcess tp, bool readTessellatedWhenNoBRepOnly, ref bool hasGeom)
        => Init(tf, tp, readTessellatedWhenNoBRepOnly, ref hasGeom, new Factors());

    /// <summary>Builds a TopoDS_Face from a tessellated_surface_set.</summary>
    public void Init(TessellatedSurfaceSet tss, TransientProcess tp, ref bool hasGeom, in Factors localFactors)
    {
        var tranFace = new TranslateFace();
        tranFace.SetPrecision(Precision());
        tranFace.SetMaxTol(MaxTol());

        var tool = new ImportTool();
        var map = new Dictionary<StepTopologicalRepresentationItem, Shape>();
        tool.Init(map, tp);

        var dummyNMTool = new NMTool();
        tranFace.Init(tss, tool, dummyNMTool, localFactors);
        hasGeom = false;

        if (!tranFace.IsDone)
        {
            tp.AddWarning(tss, " TessellatedSurfaceSet not mapped to TopoDS");
            _error = BuilderError.BuilderOther;
            _done = false;
            return;
        }

        Shape s = tranFace.InternalValue();
        _result = ShapeCasts.Face(s);
        _error = BuilderError.BuilderDone;
        _done = true;
    }

    public void Init(TessellatedSurfaceSet tss, TransientProcess tp, ref bool hasGeom)
        => Init(tss, tp, ref hasGeom, new Factors());

    /// <summary>The result of the mapping, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape (decision 1247).</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_Builder::Value() - no result");
        return _result;
    }

    public BuilderError Error() => _error;
}

/// <summary>
/// What StepToTopoDS_Builder asks of STEPControl_ActorRead (unit 8F) for the geometric representation items of a
/// geometric set: OCCT down-casts the actor to that class; the port asks for this interface, which the actor
/// implements (decision 1249).
/// </summary>
internal interface IActorRead
{
    /// <summary>STEPControl_ActorRead::TransferShape.</summary>
    Binder? TransferShape(Transient start, TransientProcess tp, in Factors localFactors,
                                   bool isManifold, bool useTrsf, ProgressRange progress);
}
