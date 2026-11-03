// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorWrite.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorWrite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Process;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.ModelType;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Step.Export.FacetedError;

namespace Kerf.Engine.Exchange.Step;

/// <summary>This class performs the transfer of a Shape from TopoDS to AP203 or AP214 (CD2 or DIS)</summary>
/// <remarks>
/// OCCT.NET (plan 2.7, decisions 1266, 1269): the writer never changes the shape it is given. OCCT marks the shells
/// of a non-manifold solid closed in place (<c>Closed(true)</c>); the port marks a copy of a shell that isn't closed
/// (the same faces). The writer numbers its assembly occurrences from its context's counter
/// (<see cref="ContextTool.NauoId"/>), where OCCT uses a process-wide one. <see cref="ContextTool"/>
/// (not in OCCT) gives access to the context, for instance to set AP203's default date and person.
/// </remarks>
public class ActorWrite : ActorOfFinderProcess
{
    //: k8 abv 6 Jan 99: unique names for PRODUCTs
    //: k9 abv 6 Jan 99: TR10: eliminating duplicated APPLICATION_CONTEXT entities
    // abv,gka 05.04.99: S4136: change parameters and update units
    //  PTV    22.08.2002 OCC609 transfer solo vertices into step file.
    //  PTV    16.09.2002 OCC725 transfer compound of vertices into one geometrical curve set.

    private static bool HasGeometry(in Shape shape)
    {
        ShapeType type = shape.ShapeType();

        if (type == ShapeType.Vertex)
        {
            return true;
        }
        else if (type == ShapeType.Edge)
        {
            BrepTEdge te = shape.TShape() as BrepTEdge;
            foreach (CurveRepresentation cr in te.Curves())
            {
                bool hasGeometry = (cr.IsCurve3D && cr.Curve3D() is not null) || cr.IsCurveOnSurface()
                                    || cr.IsRegularity() || (cr.IsPolygon3D && cr.Polygon3D() is not null)
                                    || cr.IsPolygonOnTriangulation() || cr.IsPolygonOnSurface();
                if (!hasGeometry)
                {
                    return false;
                }
            }
            return true;
        }
        else if (type == ShapeType.Face)
        {
            BrepTFace tf = shape.TShape() as BrepTFace;
            if (tf.Surface() is not null)
            {
                return true;
            }
        }
        else
        {
            Iterator it = new Iterator(shape, false, false);
            for (; it.More(); it.Next())
            {
                Shape aShape = it.Value();
                bool hasGeometry = HasGeometry(aShape);
                if (!hasGeometry)
                {
                    return false;
                }
            }
            return true;
        }

        return false;
    }

    //=======================================================================
    // Function : IsManifoldShape
    // Purpose  : Used to define whether the passed shape has manifold
    //            topology or not
    //=======================================================================
    private static bool IsManifoldShape(in Shape shape)
    {
        bool result = true;

        // Do not check nested Compounds
        Compound directShapes = new Compound();
        BrepBuilder brepBuilder = new BrepBuilder();
        brepBuilder.MakeCompound(ref directShapes);

        Iterator it = new Iterator(shape);
        for (; it.More(); it.Next())
        {
            Shape directChild = it.Value();
            if (directChild.ShapeType() != ShapeType.Compound)
            {
                brepBuilder.Add(directShapes, directChild);
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapEdgeFaces =
            new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(directShapes, ShapeType.Edge, ShapeType.Face, mapEdgeFaces);

        int nbEdges = mapEdgeFaces.Extent();

        // Check topology
        for (int i = 1; i <= nbEdges; i++)
        {
            Edge currentEdge = ShapeCasts.Edge(mapEdgeFaces.FindKey(i));
            if (!Tool.Degenerated(currentEdge))
            {
                int nbAncestors = mapEdgeFaces.FindFromIndex(i).Extent();
                if (nbAncestors > 2)
                {
                    result = false;
                    break;
                }
            }
        }

        return result;
    }

    // OCCT.NET: OCCT marks a non-manifold solid's shell closed in place (aSubShell.Closed(true)), changing the caller's
    // shape; the port gives a shell that isn't closed as a copy (the same faces) marked closed (decision 1269)
    private static Shell ClosedShell(Shell shell)
    {
        if (shell.Closed())
        {
            return shell;
        }
        Shape copy = shell.EmptyCopied();
        BrepBuilder builder = new BrepBuilder();
        for (Iterator it = new Iterator(shell, false, false); it.More(); it.Next())
        {
            builder.Add(copy, it.Value());
        }
        Shell aShell = ShapeCasts.Shell(copy);
        aShell.Closed(true);
        return aShell;
    }

    public ActorWrite()
    {
        _mygroup = 0;
        _mytoler = -1.0;
        SetMode(ModelType.ShellBasedSurfaceModel);
    }

    //=======================================================================
    // method: getNMSSRForGroup
    // purpose: allows to get NMSSR (NON_MANIFOLD_SURFACE_SHAPE_REPRESENTATION)
    //         STEP's entity for the group of shells (!) passed
    //=======================================================================
    private StepNonManifoldSurfaceShapeRepresentation GetNMSSRForGroup(NCollection_HSequence<Shape> shapeGroup,
                                                                             FinderProcess fp, out bool isNMSSRCreated)
    {
        StepNonManifoldSurfaceShapeRepresentation result = null;

        if (shapeGroup is not null)
        {
            for (int i = 1; i <= shapeGroup.Length(); i++)
            {
                Shape currentShape = shapeGroup.Value(i);
                ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, currentShape);
                if (fp.FindTypedTransient(mapper, typeof(StepNonManifoldSurfaceShapeRepresentation), ref result))
                {
                    break;
                }
            }
        }

        if (result is null)
        {
            result = new StepNonManifoldSurfaceShapeRepresentation();
            isNMSSRCreated = true;
        }
        else
        {
            isNMSSRCreated = false;
        }

        return result;
    }

    //=======================================================================
    // function : mergeInfoForNM
    // purpose  : bind already written shared faces to STEP entity for non-manifold
    //=======================================================================
    private void MergeInfoForNM(FinderProcess fp, Transient info)
    {
        ShapeContext context = info as ShapeContext;
        if (context is null)
        {
            return;
        }

        // OCCT iterates the map in its hash order; the port in the order of the shape processing (decision 1204)
        foreach (KeyValuePair<Shape, Shape> shapeShapeIt in context.Map())
        {
            Shape orig = shapeShapeIt.Key, res = shapeShapeIt.Value;
            if (orig.ShapeType() != ShapeType.Face)
            {
                continue;
            }

            ShapeMapper origMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, orig);
            Binder origBinder = fp.Find(origMapper);
            if (origBinder is null)
            {
                continue;
            }

            ShapeMapper resMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, res);
            fp.Bind(resMapper, origBinder);
        }
    }

    /// <summary>
    /// Gets sequence of vertices of all compounds level by recursive @param[in] theShape shape to iterate, checked
    /// for compound type and sub shapes vertex type @param[out] theVertices sequence of found vertices via
    /// recursively iterate of shape @return TRUE if one or more vertex was found and all shapes were compound or
    /// vertex
    /// </summary>
    private bool SeparateShapeToSoloVertex(in Shape shape, NCollection_Sequence<Shape> vertices)
    {
        if (shape.IsNull)
        {
            return false;
        }
        switch (shape.ShapeType())
        {
            case ShapeType.Compound:
                {
                    for (Iterator iter = new Iterator(shape); iter.More(); iter.Next())
                    {
                        if (!SeparateShapeToSoloVertex(iter.Value(), vertices))
                        {
                            return false;
                        }
                    }
                    break;
                }
            case ShapeType.Vertex:
                {
                    vertices.Append(shape);
                    break;
                }
            default:
                {
                    vertices.Clear();
                    return false;
                }
        }
        return true;
    }

    public void SetMode(ModelType m)
    {
        switch (m)
        {
            case ModelType.AsIs:
                ModeTrans() = 0;
                break;
            case ModelType.ManifoldSolidBrep:
                ModeTrans() = 3;
                break;
            case ModelType.BrepWithVoids:
                ModeTrans() = 5;
                break;
            case ModelType.FacetedBrep:
                ModeTrans() = 1;
                break;
            case ModelType.FacetedBrepAndBrepWithVoids:
                ModeTrans() = 6;
                break;
            case ModelType.ShellBasedSurfaceModel:
                ModeTrans() = 2;
                break;
            case ModelType.GeometricCurveSet:
                ModeTrans() = 4;
                break;
            case ModelType.Hybrid:
                ModeTrans() = 0;
                break; // PAS IMPLEMENTE !!
            default:
                break;
        }
    }

    public ModelType Mode()
    {
        switch (ModeTrans())
        {
            case 0:
                return ModelType.AsIs;
            case 1:
                return ModelType.FacetedBrep;
            case 2:
                return ModelType.ShellBasedSurfaceModel;
            case 3:
                return ModelType.ManifoldSolidBrep;
            case 4:
                return ModelType.GeometricCurveSet;
            case 5:
                return ModelType.BrepWithVoids;
            case 6:
                return ModelType.FacetedBrepAndBrepWithVoids;
            default:
                break;
        }
        return ModelType.AsIs;
    }

    public void SetGroupMode(int mode)
    {
        if (mode >= 0)
        {
            _mygroup = mode;
        }
    }

    public int GroupMode()
    {
        return _mygroup;
    }

    public void SetTolerance(double tol)
    {
        _mytoler = tol;
    }

    /// <summary>The writing context (OCCT.NET: OCCT keeps it private).</summary>
    public ContextTool ContextTool()
    {
        return _context;
    }

    //=======================================================================
    // function : Recognize
    //  ATTENTION, Recognize doit s aligner sur ce que Transfer sait faire
    // purpose  :
    //=======================================================================
    public override bool Recognize(Finder start)
    {
        ModelType mymode = Mode();
        ShapeMapper mapper = start as ShapeMapper;
        if (mapper is null)
        {
            return false;
        }
        if (mymode == ModelType.AsIs)
        {
            return true;
        }

        bool yasolid = false, yashell = false, yaface = false;

        Shape theShape;
        //  theShape = TopoDSToStep::DirectFaces(mapper->Value());
        theShape = mapper.Value(); // pour une reconnaissance c est bien assez

        if (theShape.ShapeType() == ShapeType.Compound)
        {
            Explorer solidExp = new Explorer(), shellExp = new Explorer(), faceExp = new Explorer();

            for (solidExp.Init(theShape, ShapeType.Solid); solidExp.More(); solidExp.Next())
            {
                yasolid = true;
            }
            for (shellExp.Init(theShape, ShapeType.Shell, ShapeType.Solid); shellExp.More(); shellExp.Next())
            {
                yashell = true;
            }
            for (faceExp.Init(theShape, ShapeType.Face, ShapeType.Shell); faceExp.More(); faceExp.Next())
            {
                yaface = true;
            }
        }
        else if (theShape.ShapeType() == ShapeType.Solid)
        {
            yasolid = true;
        }
        else if (theShape.ShapeType() == ShapeType.Shell)
        {
            yashell = true;
        }
        else if (theShape.ShapeType() == ShapeType.Face)
        {
            yaface = true;
        }
        else if (mymode != ModelType.GeometricCurveSet)
        {
            return false;
        }
        //  pour wireframe ?

        //  Faceted : il est OBLIGATOIRE d avoir des surfaces support Plane et des
        //   courbes 3D Line (pcurves ignorees)

        if (mymode == ModelType.FacetedBrep || mymode == ModelType.FacetedBrepAndBrepWithVoids)
        {
            for (Explorer ffac = new Explorer(theShape, ShapeType.Face); ffac.More(); ffac.Next())
            {
                Face f = ShapeCasts.Face(ffac.Current());
                Placement locbid = new Placement();
                Surface surf = Tool.Surface(f, out locbid);
                if (surf is null || !surf.IsKind(typeof(PlaneSurface)))
                {
                    return false;
                }
            }
            for (Explorer fedg = new Explorer(theShape, ShapeType.Edge); fedg.More(); fedg.Next())
            {
                Edge e = ShapeCasts.Edge(fedg.Current());
                Placement locbid;
                Curve3D curv = Tool.Curve(e, out locbid, out double first, out double last);
                if (curv is null || !curv.IsKind(typeof(LineCurve3D)))
                {
                    return false;
                }
            }
        }

        switch (mymode)
        {
            case ModelType.ManifoldSolidBrep:
                return (yasolid || yashell);
            case ModelType.BrepWithVoids:
            case ModelType.FacetedBrep:
            case ModelType.FacetedBrepAndBrepWithVoids:
                return yasolid;
            case ModelType.ShellBasedSurfaceModel:
                return (yasolid || yashell || yaface);
            case ModelType.GeometricCurveSet:
                return true; // tout OK
            default:
                break;
        }
        return false;
    }

    //  ########    MAKE PRODUCT DATA + CONTEXT    ########

    public override Binder Transfer(Finder start, FinderProcess fp, ProgressRange progress = default)
    {
        ShapeMapper mapper = start as ShapeMapper;

        if (mapper is null)
        {
            return NullResult();
        }
        Shape shape = mapper.Value();

        // init context
        StepModel model = fp.Model() as StepModel;
        if (model is not null)
        {
            _context.SetModel(model); //: abv 04.11.00: take APD from model
        }
        _context.AddAPD(false); // update APD
        _context.SetLevel(1);   // set assembly level to 1 (to ensure)
        if (!model.IsInitializedUnit)
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; the port has no
            // process-wide unit: millimetres (STEPControl_Writer initialises the model's unit from its statics)
            model.SetLocalLengthUnit(global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(null));
        }
        double lFactor = model.WriteLengthUnit();
        lFactor /= model.LocalLengthUnit();
        int anglemode = (int)model.InternalParameters.AngleUnit;
        Factors localFactors = new Factors();
        localFactors.InitializeFactors(lFactor, (anglemode <= 1 ? 1.0 : Math.PI / 180.0), 1.0);
        // create SDR
        Part sdrTool = new Part();
        sdrTool.MakeSDR(null, _context.GetProductName(), _context.GetAPD().Application(), model);
        StepShapeDefinitionRepresentation sdr = sdrTool.SDRValue();
        // transfer shape

        Binder resbind = TransferShape(mapper, sdr, fp, localFactors, null, true, progress);

        // create binder with all root entities produced from shape
        NCollection_HSequence<Transient> roots = _context.GetRootsForPart(sdrTool);
        Binder resprod = TransientResult(_context.GetAPD());
        for (int i = 1; i <= roots.Length(); i++)
        {
            resprod.AddResult(TransientResult(roots.Value(i)));
        }
        resprod.AddResult(resbind);

        // bind and exit
        // FP->Bind (mapper,resprod);
        _context.NextIndex();
        return resprod;
    }

    //==========================================

    private static double UsedTolerance(StepModel stepModel, double mytoler, in Shape shape)
    {
        //    COMPUTING 3D TOLERANCE
        //    Either from Session, or Computed (Least,Average, or Greatest)
        //    Then given to TopoDSToStep_Tool
        double tol = mytoler;
        int tolmod = (int)stepModel.InternalParameters.WritePrecisionMode;
        if (tol <= 0 && tolmod == 2)
        {
            tol = stepModel.InternalParameters.WritePrecisionVal;
        }
        if (tol <= 0)
        {
            ShapeToleranceAnalysis stu = new ShapeToleranceAnalysis();
            tol = stu.Tolerance(shape, tolmod);
            //  Par defaut, on prend une tolerance moyenne, on elimine les aberrations
            tol = FrameworkMsg.Intervalled(tol * 1.5); // arrondi a 1 2 5 ...
        }
        if (tol == 0)
        {
            tol = 1.0e-07; // minimum ...
        }

        return tol;
    }

    /// <summary>
    /// Customizable method to check whether shape S should be written as assembly or not Default implementation uses
    /// flag GroupMode and analyses the shape itself NOTE: this method can modify shape
    /// </summary>
    // if GroupMode is >1 downgrades all compounds having single subshape to that
    // subshape
    public virtual bool IsAssembly(StepModel model, ref Shape s)
    {
        if (GroupMode() == 0 || s.ShapeType() != ShapeType.Compound)
        {
            return false;
        }
        // PTV 16.09.2002  OCC725 for storing compound of vertices
        if (model.InternalParameters.WriteVertexMode == 0)
        { // bug 23950
            if (s.ShapeType() == ShapeType.Compound)
            {
                bool isOnlyVertices = true;
                Iterator itr = new Iterator(s);
                for (; itr.More(); itr.Next())
                {
                    if (itr.Value().ShapeType() != ShapeType.Vertex)
                    {
                        isOnlyVertices = false;
                        break;
                    }
                }
                if (isOnlyVertices)
                {
                    return false;
                }
            }
        }
        if (GroupMode() == 1)
        {
            return true;
        }
        Iterator it = new Iterator(s);
        if (!it.More())
        {
            return false;
        }
        Shape shape = it.Value();
        it.Next();
        if (it.More())
        {
            return true;
        }
        s = shape;
        return IsAssembly(model, ref s);
    }

    // PTV 16.09.2002 added for transferring vertices.
    private static bool TransferVertex(FinderProcess fp, NCollection_HArray1<StepGeometricSetSelect> aGSS,
                                       in Shape aShVrtx, int num, in Factors localFactors)
    {
        bool isDone = false;
        NCollection_DataMap<Shape, Transient> map =
            new NCollection_DataMap<Shape, Transient>(ShapeHasher.Instance);
        StepModel stepModel = fp.Model() as StepModel;
        ExportTool tool = new ExportTool(map, true, stepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        MakeStepVertex mkVrtx = new MakeStepVertex(ShapeCasts.Vertex(aShVrtx), tool, fp, localFactors);

        if (!mkVrtx.IsDone)
        {
            return isDone;
        }

        StepVertexPoint vp = tool.Find(aShVrtx) as StepVertexPoint;
        if (vp is null)
        {
            return isDone;
        }

        StepGeometricSetSelect select = new StepGeometricSetSelect();
        select.SetValue(vp.VertexGeometry());
        // add current result
        aGSS.SetValue(num, select);
        isDone = true;
        return isDone;
    }

    public Binder TransferShape(Finder start, StepShapeDefinitionRepresentation sdr0, FinderProcess fp,
                                         in Factors localFactors, NCollection_HSequence<Shape> shapeGroup = null,
                                         bool isManifold = true, ProgressRange progress = default)
    {
        ModelType mymode = Mode();
        ShapeMapper mapper = start as ShapeMapper;
        Binder binder = null;
        StepModel stepModel = fp.Model() as StepModel;

        // Indicates whether to use an existing NMSSR to write items to (ss; 13.11.2010)
        bool useExistingNMSSR = false;

        if (mapper is null)
        {
            return binder;
        }
        Shape theShape = mapper.Value();

        if (theShape.IsNull)
        {
            return binder;
        }

        // INDIVIDUAL SHAPE ALREADY TRANSFERRED : RETURN IT !
        binder = fp.Find(start);
        if (binder is not null)
        {
            if (!binder.HasResult)
            {
                binder = null;
            }
        }
        if (binder is not null)
        {
            //: abv 20.05.02: writing box & face from it (shared) in one compound
            // as assembly - while face already translated, it should be
            // re-translated to break sharing
            //    return binder;
        }

        // MODE ASSEMBLY : if Compound, (sub-)assembly
        if (IsAssembly(stepModel, ref theShape))
        {
            return TransferCompound(start, sdr0, fp, localFactors, progress);
        }

        using ProgressScope psRoot = new ProgressScope(progress, null, 2);

        // [BEGIN] Separate manifold topology from non-manifold in group mode 0 (ssv; 18.11.2010)
        bool isNMMode = stepModel.InternalParameters.WriteNonmanifold;
        Binder nmBinder = null;
        if (isNMMode && GroupMode() == 0 && theShape.ShapeType() == ShapeType.Compound)
        {
            Compound nmCompound = new Compound();
            Compound manifoldCompound = new Compound();
            BrepBuilder brepBuilder = new BrepBuilder();

            // Create empty Compounds
            brepBuilder.MakeCompound(ref manifoldCompound);
            brepBuilder.MakeCompound(ref nmCompound);

            // Indicates whether there is only non-manifold topology detected
            // (there is no manifold topology found in the Compound passed)
            bool isOnlyNonManifold = false;

            // Find a Compound containing non-manifold topology.
            // NOTE: only one such Compound must exist in the entire Compound passed
            if (!IsManifoldShape(theShape))
            {
                nmCompound = ShapeCasts.Compound(theShape);
                isOnlyNonManifold = true;
            }
            else
            {
                // OCCT appends to the list while iterating it: an index walks the growing list
                List<Shape> listOfShapes = new List<Shape>();
                NCollection_List<Shape> listOfManifoldShapes = new NCollection_List<Shape>();
                listOfShapes.Add(theShape);

                for (int iL = 0; iL < listOfShapes.Count; iL++)
                {
                    Shape parentShape = listOfShapes[iL];
                    Iterator it = new Iterator(parentShape);
                    for (; it.More(); it.Next())
                    {
                        Shape subShape = it.Value();
                        if (subShape.ShapeType() == ShapeType.Compound && !IsManifoldShape(subShape))
                        {
                            nmCompound = ShapeCasts.Compound(subShape);
                        }
                        else if (subShape.ShapeType() == ShapeType.Compound)
                        {
                            listOfShapes.Add(subShape);
                        }
                        else
                        {
                            listOfManifoldShapes.Append(subShape);
                        }
                    }
                }

                // Group manifold topology together.
                // NOTE: there is no sense that initial Compound structure was lost as
                //       group mode is set to 0 (no Assemblies are mapped)
                foreach (Shape currentManiShape in listOfManifoldShapes)
                {
                    brepBuilder.Add(manifoldCompound, currentManiShape);
                }
            }

            // Process only manifold topology in the current TransferShape invocation.
            // Invoke TransferShape for non-manifold topology separately (see below)
            theShape = manifoldCompound;

            // List of items to transfer
            NCollection_HSequence<Shape> repItemSeq1 = new NCollection_HSequence<Shape>();
            // Non-manifold group to pass into TransferShape with each shape from RepItemSeq
            NCollection_HSequence<Shape> nonManifoldGroup = new NCollection_HSequence<Shape>();

            // Transfer Solids to closed Shells. Prepare RepItemSeq & NonManifoldGroup
            for (Iterator iter = new Iterator(nmCompound); iter.More(); iter.Next())
            {
                Shape subShape = iter.Value();
                if (subShape.ShapeType() == ShapeType.Solid)
                {
                    for (Iterator subIter = new Iterator(subShape); subIter.More(); subIter.Next())
                    {
                        Shell subShell = ClosedShell(ShapeCasts.Shell(subIter.Value())); // OCCT: aSubShell.Closed(true)
                        repItemSeq1.Append(subShell);
                        nonManifoldGroup.Append(subShell);
                    }
                }
                else if (!isManifold && (subShape.ShapeType() == ShapeType.Shell))
                {
                    repItemSeq1.Append(subShape);
                    nonManifoldGroup.Append(subShape);
                }
                else
                {
                    repItemSeq1.Append(iter.Value());
                }
            }

            int nmItemsNb = repItemSeq1.Length();

            // In case of pure manifold topology do nothing; theShape is processed as usual (see below)
            if (nmItemsNb > 0)
            {
                // Prepare SDR for non-manifold group. This SDR will be linked to NMSSR by means
                // of TransferShape invocation. SDR is not created if there is no any manifold
                // topology in the passed Compound. If topology is pure non-manifold, SDR0 (passed)
                // is used
                StepShapeDefinitionRepresentation sdr;
                if (isOnlyNonManifold)
                {
                    sdr = sdr0;
                }
                else
                {
                    Part sdrTool = new Part();
                    sdrTool.MakeSDR(null, _context.GetProductName(), _context.GetAPD().Application(), stepModel);
                    sdr = sdrTool.SDRValue();
                }

                nmBinder = TransientResult(sdr);

                // Complete SDR with shape representations.
                // NOTE: aNMBinder is connected now with this SDR. It will be added to the resulting
                //       binder in the end of this invocation of TransferShape
                using ProgressScope ps0 = new ProgressScope(psRoot.Next(), null, nmItemsNb);
                for (int i = 1; i <= nmItemsNb && ps0.More(); i++)
                {
                    ShapeMapper aMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, repItemSeq1.Value(i));
                    TransferShape(aMapper, sdr, fp, localFactors, nonManifoldGroup, false, ps0.Next());
                }

                // Nothing else needed for pure non-manifold topology, return
                if (isOnlyNonManifold)
                {
                    return nmBinder;
                }
            }
        }
        // [END] Separate manifold topology from non-manifold in group mode 0 (ssv; 18.11.2010)

        if (psRoot.UserBreak())
        {
            return null;
        }

        // create a list of items to translate
        NCollection_HSequence<Shape> repItemSeq = new NCollection_HSequence<Shape>();

        bool isSeparateVertices = stepModel.InternalParameters.WriteVertexMode == 0; // bug 23950
        // PTV 16.09.2002 OCC725 separate shape from solo vertices.
        bool isOnlyVertices = false;
        if (theShape.ShapeType() == ShapeType.Compound && isSeparateVertices)
        {
            Compound newShape = new Compound(), compOfVrtx = new Compound();
            BrepBuilder builder = new BrepBuilder();
            builder.MakeCompound(ref newShape);
            builder.MakeCompound(ref compOfVrtx);
            NCollection_Sequence<Shape> vertices = new NCollection_Sequence<Shape>();
            isOnlyVertices = SeparateShapeToSoloVertex(theShape, vertices);
            if (!isOnlyVertices)
            {
                for (Iterator compIt = new Iterator(theShape); compIt.More(); compIt.Next())
                {
                    Shape curSh = compIt.Value();
                    NCollection_Sequence<Shape> verticesOfSubSh = new NCollection_Sequence<Shape>();
                    if (SeparateShapeToSoloVertex(curSh, verticesOfSubSh))
                    {
                        vertices.Append(verticesOfSubSh);
                    }
                    else
                    {
                        builder.Add(newShape, curSh);
                    }
                }
                theShape = newShape;
            }
            foreach (Shape vertex in vertices)
            {
                builder.Add(compOfVrtx, vertex);
            }
            if (!vertices.IsEmpty())
            {
                repItemSeq.Append(compOfVrtx);
            }
        }

        if (theShape.ShapeType() == ShapeType.Compound)
        {
            Explorer solidExp = new Explorer(), shellExp = new Explorer(), faceExp = new Explorer();
            if (mymode != ModelType.GeometricCurveSet)
            {
                for (solidExp.Init(theShape, ShapeType.Solid); solidExp.More(); solidExp.Next())
                {
                    repItemSeq.Append(ShapeCasts.Solid(solidExp.Current()));
                }
                for (shellExp.Init(theShape, ShapeType.Shell, ShapeType.Solid); shellExp.More(); shellExp.Next())
                {
                    repItemSeq.Append(ShapeCasts.Shell(shellExp.Current()));
                }

                for (faceExp.Init(theShape, ShapeType.Face, ShapeType.Shell); faceExp.More(); faceExp.Next())
                {
                    repItemSeq.Append(ShapeCasts.Face(faceExp.Current()));
                }
            }
            else
            {
                if (!isOnlyVertices)
                {
                    repItemSeq.Append(theShape); //: j1
                }
            }
            if (mymode == ModelType.AsIs)
            {
                Explorer wireExp = new Explorer(), edgeExp = new Explorer();
                for (wireExp.Init(theShape, ShapeType.Wire, ShapeType.Face); wireExp.More(); wireExp.Next())
                {
                    repItemSeq.Append(ShapeCasts.Wire(wireExp.Current()));
                }
                for (edgeExp.Init(theShape, ShapeType.Edge, ShapeType.Wire); edgeExp.More(); edgeExp.Next())
                {
                    repItemSeq.Append(ShapeCasts.Edge(edgeExp.Current()));
                }
            }
        }
        else if (theShape.ShapeType() == ShapeType.Solid)
        {
            repItemSeq.Append(ShapeCasts.Solid(theShape));
        }
        else if (theShape.ShapeType() == ShapeType.Shell)
        {
            repItemSeq.Append(ShapeCasts.Shell(theShape));
        }
        else if (theShape.ShapeType() == ShapeType.Face)
        {
            repItemSeq.Append(ShapeCasts.Face(theShape));
        }
        else if (theShape.ShapeType() == ShapeType.CompSolid)
        {
            fp.AddWarning(start, "NonManifold COMPSOLID was translated like a set of SOLIDs");
            if (GroupMode() > 0)
            {
                return TransferCompound(start, sdr0, fp, localFactors, psRoot.Next());
            }
            else
            {
                Explorer solidExp = new Explorer();
                for (solidExp.Init(theShape, ShapeType.Solid); solidExp.More(); solidExp.Next())
                {
                    repItemSeq.Append(ShapeCasts.Solid(solidExp.Current()));
                }
            }
        }

        else if (mymode != ModelType.GeometricCurveSet && mymode != ModelType.AsIs)
        {
            fp.AddFail(start, "The Shape is not a SOLID, nor a SHELL, nor a FACE");
            return binder;
        }
        else
        {
            repItemSeq.Append(theShape);
        }

        //    COMPUTING 3D TOLERANCE
        //    Either from Session, or Computed (Least,Average, or Greatest)
        //    Then given to TopoDSToStep_Tool
        double tol = UsedTolerance(stepModel, _mytoler, theShape);

        // Create a STEP-Entity for each TopoDS_Shape
        // according to the current StepModelMode

        int nbs = repItemSeq.Length();
        NCollection_HSequence<Transient> itemSeq = new NCollection_HSequence<Transient>();

        // ptv 10.11.00: allow to write empty Compound:  if (GroupMode() >0)
        itemSeq.Append(_context.GetDefaultAxis());
        ModelType trmode = mymode;
        using ProgressScope ps = new ProgressScope(psRoot.Next(), null, nbs);
        for (int i = 1; i <= nbs && ps.More(); i++)
        {
            Shape xShape = repItemSeq.Value(i);

            if (mymode == ModelType.AsIs)
            {
                switch (xShape.ShapeType())
                {
                    case ShapeType.Solid:
                        trmode = ModelType.ManifoldSolidBrep;
                        break;
                    case ShapeType.Shell:
                        trmode = ModelType.ShellBasedSurfaceModel;
                        break;
                    case ShapeType.Face:
                        trmode = ModelType.ShellBasedSurfaceModel;
                        break;
                    default:
                        trmode = ModelType.GeometricCurveSet;
                        break;
                }
            }

            using ProgressScope ps1 = new ProgressScope(ps.Next(), null, 2);

            Shape shape = xShape;

            NCollection_DataMap<string, string> parameters = GetShapeFixParameters();
            ShapeProcessor.SetParameter("FixShape.Tolerance3d", tol, true, parameters);
            ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", stepModel.InternalParameters.ReadMaxPrecisionVal, true,
                                               parameters);
            ShapeProcessor shapeProcessor = new ShapeProcessor(parameters);

            if (HasGeometry(shape))
            {
                shape = shapeProcessor.ProcessShape(xShape, GetShapeProcessFlags().Item1, ps1.Next());

                if (ps1.UserBreak())
                {
                    return null;
                }
            }

            if (!isManifold)
            {
                MergeInfoForNM(fp, shapeProcessor.GetContext());
            }

            // create a STEP entity corresponding to shape
            StepGeometricRepresentationItem item = null, itemTess = null;
            switch (trmode)
            {
                case ModelType.ManifoldSolidBrep:
                    {
                        if (shape.ShapeType() == ShapeType.Solid)
                        {
                            Solid solid = ShapeCasts.Solid(shape);

                            //: d6 abv 13 Mar 98: if solid has more than 1 shell,
                            // try to treat it as solid with voids
                            int nbShells = 0;
                            for (Iterator It = new Iterator(solid); It.More(); It.Next())
                            {
                                if (It.Value().ShapeType() == ShapeType.Shell)
                                {
                                    nbShells++;
                                }
                            }
                            if (nbShells > 1)
                            {
                                MakeBrepWithVoids mkBRepWithVoids = new MakeBrepWithVoids(solid, fp, localFactors, ps1.Next());
                                mkBRepWithVoids.Tolerance() = tol;
                                if (mkBRepWithVoids.IsDone)
                                {
                                    item = mkBRepWithVoids.Value();
                                    itemTess = mkBRepWithVoids.TessellatedValue();
                                }
                                else
                                {
                                    nbShells = 1; // smth went wrong; let it will be just Manifold
                                }
                            }
                            if (nbShells == 1)
                            {
                                ManifoldSolidBrepMaker mkManifoldSolidBrep = new ManifoldSolidBrepMaker(solid, fp, localFactors, ps1.Next());
                                mkManifoldSolidBrep.Tolerance() = tol;
                                if (mkManifoldSolidBrep.IsDone)
                                {
                                    item = mkManifoldSolidBrep.Value();
                                    itemTess = mkManifoldSolidBrep.TessellatedValue();
                                }
                            }
                        }
                        else if (shape.ShapeType() == ShapeType.Shell)
                        {
                            Shell shell = ShapeCasts.Shell(shape);
                            ManifoldSolidBrepMaker mkManifoldSolidBrep = new ManifoldSolidBrepMaker(shell, fp, localFactors, ps1.Next());
                            mkManifoldSolidBrep.Tolerance() = tol;
                            if (mkManifoldSolidBrep.IsDone)
                            {
                                item = mkManifoldSolidBrep.Value();
                                itemTess = mkManifoldSolidBrep.TessellatedValue();
                            }
                        }
                        break;
                    }
                case ModelType.BrepWithVoids:
                    {
                        if (shape.ShapeType() == ShapeType.Solid)
                        {
                            Solid solid = ShapeCasts.Solid(shape);
                            MakeBrepWithVoids mkBRepWithVoids = new MakeBrepWithVoids(solid, fp, localFactors, ps1.Next());
                            mkBRepWithVoids.Tolerance() = tol;
                            if (mkBRepWithVoids.IsDone)
                            {
                                item = mkBRepWithVoids.Value();
                                itemTess = mkBRepWithVoids.TessellatedValue();
                            }
                        }
                        break;
                    }
                case ModelType.FacetedBrep:
                    {
                        FacetedError facErr = FacetedTool.CheckTopoDSShape(shape);
                        if (facErr != FacetedError.FacetedDone)
                        {
                            fp.AddFail(start, "Error in Faceted Shape from TopoDS");
                            if (facErr == FacetedError.SurfaceNotPlane)
                            {
                                fp.AddFail(start, "-- The TopoDS_Face is not plane");
                            }
                            else if (facErr == FacetedError.PCurveNotLinear)
                            {
                                fp.AddFail(start, "-- The Face contains non linear PCurves");
                            }
                            return binder;
                        }
                        if (shape.ShapeType() == ShapeType.Solid)
                        {
                            Solid solid = ShapeCasts.Solid(shape);
                            MakeFacetedBrep mkFacetedBrep = new MakeFacetedBrep(solid, fp, localFactors, ps1.Next());
                            mkFacetedBrep.Tolerance() = tol;
                            if (mkFacetedBrep.IsDone)
                            {
                                item = mkFacetedBrep.Value();
                                itemTess = mkFacetedBrep.TessellatedValue();
                            }
                        }
                        break;
                    }
                case ModelType.FacetedBrepAndBrepWithVoids:
                    {
                        FacetedError facErr = FacetedTool.CheckTopoDSShape(shape);
                        if (facErr != FacetedError.FacetedDone)
                        {
                            fp.AddFail(start, "Error in Faceted Shape from TopoDS");
                            if (facErr == FacetedError.SurfaceNotPlane)
                            {
                                fp.AddFail(start, "-- The TopoDS_Face is not plane");
                            }
                            else if (facErr == FacetedError.PCurveNotLinear)
                            {
                                fp.AddFail(start, "-- The Face contains non linear PCurves");
                            }
                            return binder;
                        }
                        if (shape.ShapeType() == ShapeType.Solid)
                        {
                            Solid solid = ShapeCasts.Solid(shape);
                            MakeFacetedBrepAndBrepWithVoids mkFacetedBrepAndBrepWithVoids =
                                new MakeFacetedBrepAndBrepWithVoids(solid, fp, localFactors, ps1.Next());
                            mkFacetedBrepAndBrepWithVoids.Tolerance() = tol;
                            if (mkFacetedBrepAndBrepWithVoids.IsDone)
                            {
                                item = mkFacetedBrepAndBrepWithVoids.Value();
                                itemTess = mkFacetedBrepAndBrepWithVoids.TessellatedValue();
                            }
                        }
                        break;
                    }
                case ModelType.ShellBasedSurfaceModel:
                    {
                        if (shape.ShapeType() == ShapeType.Solid)
                        {
                            Solid solid = ShapeCasts.Solid(shape);
                            MakeShellBasedSurfaceModel mkShellBasedSurfaceModel =
                                new MakeShellBasedSurfaceModel(solid, fp, localFactors, ps1.Next());
                            mkShellBasedSurfaceModel.Tolerance() = tol;
                            if (mkShellBasedSurfaceModel.IsDone)
                            {
                                item = mkShellBasedSurfaceModel.Value();
                                itemTess = mkShellBasedSurfaceModel.TessellatedValue();
                            }
                        }
                        else if (shape.ShapeType() == ShapeType.Shell)
                        {
                            Shell shell = ShapeCasts.Shell(shape);
                            // Non-manifold topology is stored via NMSSR containing series of SBSM (ssv; 13.11.2010)
                            MakeShellBasedSurfaceModel mkShellBasedSurfaceModel =
                                new MakeShellBasedSurfaceModel(shell, fp, localFactors, ps1.Next());
                            mkShellBasedSurfaceModel.Tolerance() = tol;
                            if (mkShellBasedSurfaceModel.IsDone)
                            {
                                item = mkShellBasedSurfaceModel.Value();
                                itemTess = mkShellBasedSurfaceModel.TessellatedValue();
                            }
                        }
                        else if (shape.ShapeType() == ShapeType.Face)
                        {
                            Face face = ShapeCasts.Face(shape);
                            MakeShellBasedSurfaceModel mkShellBasedSurfaceModel =
                                new MakeShellBasedSurfaceModel(face, fp, localFactors, ps1.Next());
                            mkShellBasedSurfaceModel.Tolerance() = tol;
                            if (mkShellBasedSurfaceModel.IsDone)
                            {
                                item = mkShellBasedSurfaceModel.Value();
                                itemTess = mkShellBasedSurfaceModel.TessellatedValue();
                            }
                        }
                        break;
                    }
                case ModelType.GeometricCurveSet:
                    {
                        MakeGeometricCurveSet mkGeometricCurveSet = new MakeGeometricCurveSet(shape, fp, localFactors);
                        mkGeometricCurveSet.Tolerance() = tol;
                        if (mkGeometricCurveSet.IsDone)
                        {
                            item = mkGeometricCurveSet.Value();
                        }
                        // PTV 22.08.2002 OCC609 ------------------------- begin --------------------
                        // modified by PTV 16.09.2002 OCC725
                        else if (shape.ShapeType() == ShapeType.Compound || shape.ShapeType() == ShapeType.Vertex)
                        {
                            // it is compound with solo vertices.
                            int nbVrtx = 0;
                            int curNb = 0;
                            Explorer exp = new Explorer(shape, ShapeType.Vertex);
                            for (; exp.More(); exp.Next())
                            {
                                if (exp.Current().ShapeType() != ShapeType.Vertex)
                                {
                                    continue;
                                }
                                nbVrtx++;
                            }
                            if (nbVrtx != 0)
                            {
                                // create new geometric curve set for all vertices
                                NCollection_HArray1<StepGeometricSetSelect> gss =
                                    new NCollection_HArray1<StepGeometricSetSelect>(1, nbVrtx);
                                TCollection_HAsciiString empty = new TCollection_HAsciiString("");
                                StepGeometricCurveSet gcSet = new StepGeometricCurveSet();
                                gcSet.SetName(empty);
                                // iterates on compound with vertices and traces each vertex
                                for (exp.ReInit(); exp.More(); exp.Next())
                                {
                                    Shape vertex = exp.Current();
                                    if (vertex.ShapeType() != ShapeType.Vertex)
                                    {
                                        continue;
                                    }
                                    curNb++;
                                    TransferVertex(fp, gss, vertex, curNb, localFactors);
                                } // end of iteration on compound with vertices.
                                gcSet.SetElements(gss);
                                item = gcSet;
                            } // end of check that number of vertices is not null
                        }
                        // PTV 22.08.2002 OCC609-------------------------  end  --------------------
                        break;
                    }
                default:
                    break;
            }
            if (item is null && itemTess is null)
            {
                continue;
            }

            // add resulting item to the FP
            if (item is not null)
            {
                itemSeq.Append(item);
                ShapeMapper submapper = null;
                if (xShape.IsSame(mapper.Value()))
                {
                    submapper = start as ShapeMapper;
                }
                if (submapper is null)
                {
                    submapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, xShape);
                }
                Binder subbind = fp.Find(submapper);
                if (subbind is null)
                {
                    subbind = TransientResult(item);
                    fp.Bind(submapper, subbind);
                }
                else
                {
                    subbind.AddResult(TransientResult(item));
                }
            }
            if (itemTess is not null)
            {
                itemSeq.Append(itemTess);
                ShapeMapper submapper = null;
                if (xShape.IsSame(mapper.Value()))
                {
                    submapper = start as ShapeMapper;
                }
                if (submapper is null)
                {
                    submapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, xShape);
                }
                Binder subbind = fp.Find(submapper);
                if (subbind is null)
                {
                    subbind = TransientResult(itemTess);
                    fp.Bind(submapper, subbind);
                }
                else
                {
                    subbind.AddResult(TransientResult(itemTess));
                }
            }

            // If shape is not processed, this function does nothing. Intentionally.
            shapeProcessor.MergeTransferInfo(fp);
        }

        // - Make Shape Representation
        int nCc1 = itemSeq.Length();
        if (nCc1 < 1)
        {
            fp.AddFail(start, "The Shape has not the appropriate type");
            return binder;
        }
        StepShapeRepresentation shapeRep = null;
        if (theShape.ShapeType() == ShapeType.Shape)
        { // for external references
            shapeRep = new StepShapeRepresentation();
        }
        else
        {
            switch (mymode)
            {
                case ModelType.ManifoldSolidBrep:
                    shapeRep = new StepAdvancedBrepShapeRepresentation();
                    break;
                case ModelType.FacetedBrep:
                    shapeRep = new StepFacetedBrepShapeRepresentation();
                    break;
                // NOTE: STEPControl_AsIs mode is normally used to transfer non-manifold topology.
                //       However, as ShellBasedSurfaceModel is used in non-manifold processing
                //       internally, STEPControl_ShellBasedSurfaceModel is also adjusted to
                //       be able to work with non-manifold cases
                case ModelType.ShellBasedSurfaceModel:
                    if (isManifold)
                    {
                        shapeRep = new StepManifoldSurfaceShapeRepresentation();
                    }
                    else
                    {
                        shapeRep = GetNMSSRForGroup(shapeGroup, fp, out bool isNewNMSSRCreated);
                        useExistingNMSSR = !isNewNMSSRCreated;
                    }
                    break;
                case ModelType.GeometricCurveSet:
                    shapeRep = new StepGeometricallyBoundedWireframeShapeRepresentation();
                    break;
                case ModelType.AsIs:
                    {
                        if (nbs == 1)
                        {
                            if (trmode == ModelType.ManifoldSolidBrep)
                            {
                                shapeRep = new StepAdvancedBrepShapeRepresentation();
                            }
                            else if (trmode == ModelType.ShellBasedSurfaceModel)
                            {
                                // Process non-manifold topology separately (ssv; 13.11.2010)
                                if (isManifold)
                                {
                                    shapeRep = new StepManifoldSurfaceShapeRepresentation();
                                }
                                else
                                {
                                    shapeRep = GetNMSSRForGroup(shapeGroup, fp, out bool isNewNMSSRCreated);
                                    useExistingNMSSR = !isNewNMSSRCreated;
                                }
                            }
                            else if (trmode == ModelType.GeometricCurveSet)
                            {
                                shapeRep = new StepGeometricallyBoundedWireframeShapeRepresentation();
                            }
                            else if (trmode == ModelType.FacetedBrep)
                            {
                                shapeRep = new StepFacetedBrepShapeRepresentation();
                            }
                        }
                        else
                        {
                            shapeRep = new StepShapeRepresentation();
                        }
                    }
                    break;
                default:
                    break;
            }
        }
        if (shapeRep is null)
        {
            Binder resb = null;
            return resb;
        }

        NCollection_HArray1<RepresentationItem> items = new NCollection_HArray1<RepresentationItem>(1, nCc1);

        for (int rep = 1; rep <= nCc1; rep++)
        {
            RepresentationItem repit = itemSeq.Value(rep) as RepresentationItem;
            items.SetValue(rep, repit);
        }
        int ap = (int)stepModel.InternalParameters.WriteSchema;
        NCollection_Sequence<Binder> seqBindRelation = new NCollection_Sequence<Binder>();
        if (ap == 3 && nbs > 1)
        {
            int j = 1;
            if (items.Value(j).IsKind(typeof(StepAxis2Placement3d)))
            {
                NCollection_HArray1<RepresentationItem> axis = new NCollection_HArray1<RepresentationItem>(1, 1);
                axis.SetValue(1, items.Value(j++));
                shapeRep.SetItems(axis);
            }
            for (; j <= items.Length(); j++)
            {
                StepShapeRepresentation shapeRepr1;
                if (items.Value(j).IsKind(typeof(StepManifoldSolidBrep)))
                {
                    shapeRepr1 = new StepAdvancedBrepShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(StepShellBasedSurfaceModel)))
                {
                    shapeRepr1 = new StepManifoldSurfaceShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(StepGeometricCurveSet)))
                {
                    shapeRepr1 = new StepGeometricallyBoundedWireframeShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(StepFacetedBrep)))
                {
                    shapeRepr1 = new StepFacetedBrepShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(TessellatedItem)))
                {
                    shapeRepr1 = new TessellatedShapeRepresentation();
                }
                else
                {
                    shapeRepr1 = new StepShapeRepresentation();
                }

                NCollection_HArray1<RepresentationItem> repr1 = new NCollection_HArray1<RepresentationItem>(1, 2);
                repr1.SetValue(1, _context.GetDefaultAxis());
                repr1.SetValue(2, items.Value(j));
                shapeRepr1.SetItems(repr1);
                UnitContext mk1 = new UnitContext();
                mk1.Init(tol, stepModel, localFactors);
                shapeRepr1.SetContextOfItems(mk1.Value()); // la tolerance, voir au debut
                shapeRepr1.SetName(new TCollection_HAsciiString(""));

                ShapeRepresentationRelationship shapeRel = new ShapeRepresentationRelationship();
                TCollection_HAsciiString name = new TCollection_HAsciiString("");
                TCollection_HAsciiString descr = new TCollection_HAsciiString("");
                shapeRel.SetName(name);
                shapeRel.SetDescription(descr);
                shapeRel.SetRep2(shapeRep);
                shapeRel.SetRep1(shapeRepr1);

                seqBindRelation.Append(TransientResult(shapeRel));
            }
        }
        else
        {
            int nC = 0;
            for (int i = 1; i <= items.Length(); i++)
            {
                if (!items.Value(i).IsKind(typeof(TessellatedItem)))
                {
                    continue;
                }
                ++nC;
            }
            if (nC > 0)
            {
                NCollection_HArray1<RepresentationItem> itemsTess = new NCollection_HArray1<RepresentationItem>(1, nC);
                int i = 1;
                for (int j = 1; j <= items.Length(); j++)
                {
                    if (!items.Value(j).IsKind(typeof(TessellatedItem)))
                    {
                        continue;
                    }
                    itemsTess.SetValue(i++, items.Value(j));
                }

                StepShapeRepresentation shapeTessRepr = new TessellatedShapeRepresentation();
                shapeTessRepr.SetItems(itemsTess);
                UnitContext mk1 = new UnitContext();
                mk1.Init(tol, stepModel, localFactors);
                shapeTessRepr.SetContextOfItems(mk1.Value());
                shapeTessRepr.SetName(new TCollection_HAsciiString(""));

                seqBindRelation.Append(TransientResult(shapeTessRepr));
            }
            if (!useExistingNMSSR)
            {
                shapeRep.SetItems(items);
            }
            else
            {
                // Add new representation item to the NMSSR's existing collection (ssv; 13.11.2010)
                NCollection_HArray1<RepresentationItem> oldItems = shapeRep.Items();
                NCollection_HArray1<RepresentationItem> newItems =
                    new NCollection_HArray1<RepresentationItem>(1, oldItems.Length() + 1);
                int el = 1;
                for (int i = 1; i <= oldItems.Length(); i++)
                {
                    newItems.SetValue(el++, oldItems.Value(i));
                }
                newItems.SetValue(el, items.Value(items.Length()));
                shapeRep.SetItems(newItems);
            }
        }

        // init representation
        UnitContext mk = new UnitContext();
        mk.Init(tol, stepModel, localFactors);
        shapeRep.SetContextOfItems(mk.Value()); // la tolerance, voir au debut
        shapeRep.SetName(new TCollection_HAsciiString(""));

        // Create SDR (only once for non-manifold group)
        if (!useExistingNMSSR)
        {
            sdr0.SetUsedRepresentation(shapeRep);
            // create binder for SR and attach to it binder for RepItem (if exists)
            Binder resbind = TransientResult(shapeRep);
            binder = fp.Find(start);
            if (binder is not null)
            {
                resbind.AddResult(binder);
                fp.Rebind(start, resbind);
                // binder->AddResult ( resbind );
                // resbind = binder;
            }
            for (int k = 1; k <= seqBindRelation.Length(); k++)
            {
                resbind.AddResult(seqBindRelation.Value(k));
            }

            // Add SDR for non-manifold topology in group mode 0 (ssv; 18.11.2010)
            if (nmBinder is not null)
            {
                resbind.AddResult(nmBinder);
            }

            return resbind;
        }
        else
        {
            return fp.Find(start);
        }
    }

    public Binder TransferShape(Finder start, StepShapeDefinitionRepresentation sdr, FinderProcess fp)
        => TransferShape(start, sdr, fp, new Factors());

    //=======================================================================
    // function : TransferCompound
    //    ####    TRANSFER COMPOUND AS (SUB-)ASSEMBLY
    // purpose  :
    //=======================================================================
    public Binder TransferCompound(Finder start, StepShapeDefinitionRepresentation sdr0, FinderProcess fp,
                                            in Factors localFactors, ProgressRange progress = default)
    {
        ShapeMapper mapper = start as ShapeMapper;
        Binder binder = null;
        if (mapper is null)
        {
            return binder;
        }
        Shape theShape = mapper.Value();

        StepModel stepModel = fp.Model() as StepModel;

        // Inspect non-manifold topology case (ssv; 10.11.2010)
        bool isNMMode = stepModel.InternalParameters.WriteNonmanifold;
        bool isManifold;
        if (isNMMode)
        {
            isManifold = IsManifoldShape(theShape);
        }
        else
        {
            isManifold = true;
        }

        // get a sequence of components (subshapes)
        NCollection_HSequence<Shape> repItemSeq = new NCollection_HSequence<Shape>();
        // Prepare a collection for non-manifold group of shapes
        NCollection_HSequence<Shape> nonManifoldGroup = new NCollection_HSequence<Shape>();
        bool isSeparateVertices = (stepModel.InternalParameters.WriteVertexMode == 0); // bug 23950
        // PTV OCC725 17.09.2002 -- begin --
        int nbFreeVrtx = 0;
        Compound compOfVrtx = new Compound();
        BrepBuilder b = new BrepBuilder();
        b.MakeCompound(ref compOfVrtx);

        for (Iterator iter = new Iterator(theShape); iter.More(); iter.Next())
        {
            Shape subShape = iter.Value();
            if (subShape.ShapeType() != ShapeType.Vertex || !isSeparateVertices)
            {
                // Store non-manifold topology as shells (ssv; 10.11.2010)
                if (!isManifold && subShape.ShapeType() == ShapeType.Solid)
                {
                    for (Iterator subIter = new Iterator(subShape); subIter.More(); subIter.Next())
                    {
                        Shell subShell = ClosedShell(ShapeCasts.Shell(subIter.Value())); // OCCT: aSubShell.Closed(true)
                        repItemSeq.Append(subShell);
                        nonManifoldGroup.Append(subShell);
                    }
                }
                else if (!isManifold && (subShape.ShapeType() == ShapeType.Shell || subShape.ShapeType() == ShapeType.Face))
                {
                    repItemSeq.Append(subShape);
                    nonManifoldGroup.Append(subShape);
                }
                else
                {
                    repItemSeq.Append(subShape);
                }

                continue;
            }
            b.Add(compOfVrtx, iter.Value());
            nbFreeVrtx++;
        }
        if (nbFreeVrtx != 0)
        {
            repItemSeq.Append(compOfVrtx);
        }

        // PTV OCC725 17.09.2002 -- end --

        // Constitution : liste d axes, le premier est l origine, les suivants : 1
        // par sous-item
        StepShapeRepresentation shapeRep = sdr0.UsedRepresentation() as StepShapeRepresentation;
        if (shapeRep is null)
        {
            shapeRep = new StepShapeRepresentation();
            sdr0.SetUsedRepresentation(shapeRep); // to be used by MakeItem
        }
        binder = TransientResult(sdr0); // set SDR as first item in order to be found first (but not SDR of subshape!)
        binder.AddResult(TransientResult(shapeRep));

        // translate components
        int i, nbs = repItemSeq.Length();
        NCollection_HSequence<Transient> itemSeq = new NCollection_HSequence<Transient>();
        itemSeq.Append(_context.GetDefaultAxis());
        _context.NextLevel();
        using ProgressScope ps = new ProgressScope(progress, null, nbs);
        for (i = 1; i <= nbs && ps.More(); i++)
        {
            ShapeMapper subs = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, repItemSeq.Value(i));
            StepGeometricRepresentationItem ax1 = null;

            Binder bnd = TransferSubShape(subs, sdr0, ref ax1, fp, localFactors, nonManifoldGroup, isManifold, ps.Next());

            if (ax1 is not null)
            {
                itemSeq.Append(ax1);
            }
            // copy binders so as to have all roots in upper binder, but do not conflict
            while (bnd is not null)
            {
                SimpleBinderOfTransient bx = bnd as SimpleBinderOfTransient;
                if (bx is not null)
                {
                    binder.AddResult(TransientResult(bx.Result()));
                }
                bnd = bnd.NextResult();
            }
        }
        _context.PrevLevel();

        int nsub = itemSeq.Length();
        NCollection_HArray1<RepresentationItem> items = new NCollection_HArray1<RepresentationItem>(1, nsub);

        // initialize representation
        for (int rep = 1; rep <= nsub; rep++)
        {
            items.SetValue(rep, itemSeq.Value(rep) as RepresentationItem);
        }
        shapeRep.SetItems(items);
        double tol = UsedTolerance(stepModel, _mytoler, theShape);
        UnitContext mk = new UnitContext();
        mk.Init(tol, stepModel, localFactors);
        shapeRep.SetContextOfItems(mk.Value()); // la tolerance, voir au debut
        shapeRep.SetName(new TCollection_HAsciiString(""));

        // set it to SDR
        //  SDR0->SetUsedRepresentation (shapeRep);

        return binder;
    }

    public Binder TransferSubShape(Finder start, StepShapeDefinitionRepresentation sdr0,
                                            ref StepGeometricRepresentationItem ax1, FinderProcess fp,
                                            in Factors localFactors, NCollection_HSequence<Shape> shapeGroup = null,
                                            bool isManifold = true, ProgressRange progress = default)
    {
        ShapeMapper mapper = start as ShapeMapper;
        if (mapper is null)
        {
            return NullResult();
        }
        Shape shape = mapper.Value();

        //   SHAPE EN POSITION VENANT D UN ASSEMBLAGE
        //   Il faut alors distinguer la transformation de la shape meme
        //   laquelle est consideree a l origine, puis transferee
        //   A part, un item decrivant une occurence en position est cree
        //   SINON, la shape est prise et transferee telle quelle
        Shape sh0 = shape;
        Transform3D loc = new Transform3D();
        bool isShapeLocated = false;
        if (GroupMode() > 0)
        {
            Placement shloc = shape.Location();
            isShapeLocated = !shloc.IsIdentity;
            loc = shloc.Transformation();
            Placement shident = new Placement();
            sh0.Location(shident);
            mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, sh0);
            mapper.SameAttributes(start);
        }

        Binder resbind = fp.Find(mapper);
        StepShapeDefinitionRepresentation sdr = null;
        Part sdrTool = new Part();
        StepModel stepModel = fp.Model() as StepModel;

        // Already SDR and SR available : take them as are
        bool iasdr = fp.GetTypedTransient(resbind, typeof(StepShapeDefinitionRepresentation), ref sdr);
        if (iasdr)
        {
            sdrTool.ReadSDR(sdr);
        }
        else
        {
            sdrTool.MakeSDR(null, _context.GetProductName(), _context.GetAPD().Application(), stepModel);
            sdr = sdrTool.SDRValue();
        }

        // if shape itself not yet translated, do it now
        //: abv 20.05.02: see comment in TransferShape(): added "! iasdr ||"
        Binder resprod = TransientResult(sdr); // KA - OCC7141(skl 10.11.2004)
        bool isJustTransferred = false;
        if (!iasdr || resbind is null)
        {
            Binder resbind1 = TransferShape(mapper, sdr, fp, localFactors, shapeGroup, isManifold, progress);
            if (resbind1 is null || sdr.UsedRepresentation() is null)
            {
                return null;
            }
            resbind = resbind1;
            Binder oldbind = fp.Find(mapper);
            if (oldbind is not null && resbind is not null)
            {
                resbind.AddResult(oldbind);
            }
            isJustTransferred = true;
        }

        // create assembly structures (CDSR, NAUO etc.)
        ConstructAssembly mkitem = new ConstructAssembly();

        // make location for assembly placement
        if (Math.Abs(loc.ScaleFactor() - 1.0) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            if (stepModel.InternalParameters.WriteScalingTrsf)
            {
                fp.AddWarning(start, "The shape has a scaling factor, exported as cartesian_transformation_operator");
            }
            else
            {
                fp.AddWarning(start, "The shape has a scaling factor, skipped");
            }
        }
        if (Math.Abs(loc.ScaleFactor() - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion() || !stepModel.InternalParameters.WriteScalingTrsf)
        {
            // create a new axis2placement3d
            Axis2Placement3dMaker mkax = new Axis2Placement3dMaker(loc, localFactors);
            mkitem.Init(sdr, sdr0, _context.GetDefaultAxis(), mkax.Value());
            ax1 = mkax.Value();
        }
        else
        {
            // create a new cartesian transformation
            MakeCartesianTransformationOperator maker = new MakeCartesianTransformationOperator(loc, localFactors);
            if (!maker.IsDone)
            {
                fp.AddFail(start, "The transformation relation creation failed");
            }
            mkitem.Init(sdr, sdr0, maker.Value());
            ax1 = maker.Value();
        }

        // OCCT numbers the occurrence from a process-wide counter: here the writer's (decision 1266)
        mkitem.MakeRelationship(ref _context.NauoId());
        NCollection_HSequence<Transient> roots = _context.GetRootsForAssemblyLink(mkitem);

        // add roots corresponding to assembly and product structures to binder
        // KA: we need only the current subshape in resprod, since the binder is copied
        //    in Transfershape which calls Transfersubshape   [ OCC7141(skl 10.11.2004) ]
        if (!iasdr)
        {
            resprod.AddResult(TransientResult(sdr));
            if (resprod != resbind)
            {
                resbind.AddResult(TransientResult(sdr)); // KA - OCC7141(skl 10.11.2004)
            }
            roots.Append(_context.GetRootsForPart(sdrTool));
        }
        for (int i = 1; i <= roots.Length(); i++)
        {
            resprod.AddResult(TransientResult(roots.Value(i)));
            if (resprod != resbind)
            {
                resbind.AddResult(TransientResult(roots.Value(i))); // KA - OCC7141(skl 10.11.2004)
            }
        }
        if (isJustTransferred)
        {
            // We make CDSR of the current shape preceding CDSR of any subshapes,
            // therefore add resbind at the end.
            resprod.AddResult(resbind);
            fp.Bind(mapper, resprod);
        }

        _context.NextIndex();

        // abv 16.10.00: bind CDSR (et al) to located shape in order to be able to track instances
        if (mapper != start && isShapeLocated)
        {
            Binder bnd = fp.Find(start);
            for (int j = 1; j <= roots.Length(); j++)
            {
                if (bnd is null)
                {
                    bnd = TransientResult(roots.Value(j));
                }
                else
                {
                    bnd.AddResult(TransientResult(roots.Value(j)));
                }
            }
            fp.Bind(start, bnd);
        }

        return resprod;
    }

    private int _mygroup;
    private double _mytoler;
    private ContextTool _context = new ContextTool();
}
