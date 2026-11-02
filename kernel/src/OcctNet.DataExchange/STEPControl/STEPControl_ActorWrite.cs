// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorWrite.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorWrite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.STEPControl_StepModelType;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopoDSToStep_FacetedError;

namespace Occt;

/// <summary>This class performs the transfer of a Shape from TopoDS to AP203 or AP214 (CD2 or DIS)</summary>
/// <remarks>
/// OCCT.NET (plan 2.7, decisions 1266, 1269): the writer never changes the shape it is given. OCCT marks the shells
/// of a non-manifold solid closed in place (<c>Closed(true)</c>); the port marks a copy of a shell that isn't closed
/// (the same faces). The writer numbers its assembly occurrences from its context's counter
/// (<see cref="STEPConstruct_ContextTool.NauoId"/>), where OCCT uses a process-wide one. <see cref="ContextTool"/>
/// (not in OCCT) gives access to the context, for instance to set AP203's default date and person.
/// </remarks>
public class STEPControl_ActorWrite : Transfer_ActorOfFinderProcess
{
    //: k8 abv 6 Jan 99: unique names for PRODUCTs
    //: k9 abv 6 Jan 99: TR10: eliminating duplicated APPLICATION_CONTEXT entities
    // abv,gka 05.04.99: S4136: change parameters and update units
    //  PTV    22.08.2002 OCC609 transfer solo vertices into step file.
    //  PTV    16.09.2002 OCC725 transfer compound of vertices into one geometrical curve set.

    private static bool hasGeometry(in TopoDS_Shape theShape)
    {
        TopAbs_ShapeEnum aType = theShape.ShapeType();

        if (aType == TopAbs_VERTEX)
        {
            return true;
        }
        else if (aType == TopAbs_EDGE)
        {
            BRep_TEdge TE = theShape.TShape() as BRep_TEdge;
            foreach (BRep_CurveRepresentation CR in TE.Curves())
            {
                bool aHasGeometry = (CR.IsCurve3D() && CR.Curve3D() is not null) || CR.IsCurveOnSurface()
                                    || CR.IsRegularity() || (CR.IsPolygon3D() && CR.Polygon3D() is not null)
                                    || CR.IsPolygonOnTriangulation() || CR.IsPolygonOnSurface();
                if (!aHasGeometry)
                {
                    return false;
                }
            }
            return true;
        }
        else if (aType == TopAbs_FACE)
        {
            BRep_TFace TF = theShape.TShape() as BRep_TFace;
            if (TF.Surface() is not null)
            {
                return true;
            }
        }
        else
        {
            TopoDS_Iterator anIt = new TopoDS_Iterator(theShape, false, false);
            for (; anIt.More(); anIt.Next())
            {
                TopoDS_Shape aShape = anIt.Value();
                bool aHasGeometry = hasGeometry(aShape);
                if (!aHasGeometry)
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
    private static bool IsManifoldShape(in TopoDS_Shape theShape)
    {
        bool aResult = true;

        // Do not check nested Compounds
        TopoDS_Compound aDirectShapes = new TopoDS_Compound();
        BRep_Builder aBrepBuilder = new BRep_Builder();
        aBrepBuilder.MakeCompound(ref aDirectShapes);

        TopoDS_Iterator anIt = new TopoDS_Iterator(theShape);
        for (; anIt.More(); anIt.Next())
        {
            TopoDS_Shape aDirectChild = anIt.Value();
            if (aDirectChild.ShapeType() != TopAbs_COMPOUND)
            {
                aBrepBuilder.Add(aDirectShapes, aDirectChild);
            }
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapEdgeFaces =
            new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aDirectShapes, TopAbs_EDGE, TopAbs_FACE, aMapEdgeFaces);

        int aNbEdges = aMapEdgeFaces.Extent();

        // Check topology
        for (int i = 1; i <= aNbEdges; i++)
        {
            TopoDS_Edge aCurrentEdge = TopoDS.Edge(aMapEdgeFaces.FindKey(i));
            if (!BRep_Tool.Degenerated(aCurrentEdge))
            {
                int aNbAncestors = aMapEdgeFaces.FindFromIndex(i).Extent();
                if (aNbAncestors > 2)
                {
                    aResult = false;
                    break;
                }
            }
        }

        return aResult;
    }

    // OCCT.NET: OCCT marks a non-manifold solid's shell closed in place (aSubShell.Closed(true)), changing the caller's
    // shape; the port gives a shell that isn't closed as a copy (the same faces) marked closed (decision 1269)
    private static TopoDS_Shell ClosedShell(TopoDS_Shell theShell)
    {
        if (theShell.Closed())
        {
            return theShell;
        }
        TopoDS_Shape aCopy = theShell.EmptyCopied();
        BRep_Builder aBuilder = new BRep_Builder();
        for (TopoDS_Iterator anIt = new TopoDS_Iterator(theShell, false, false); anIt.More(); anIt.Next())
        {
            aBuilder.Add(aCopy, anIt.Value());
        }
        TopoDS_Shell aShell = TopoDS.Shell(aCopy);
        aShell.Closed(true);
        return aShell;
    }

    public STEPControl_ActorWrite()
    {
        mygroup = 0;
        mytoler = -1.0;
        SetMode(STEPControl_ShellBasedSurfaceModel);
    }

    //=======================================================================
    // method: getNMSSRForGroup
    // purpose: allows to get NMSSR (NON_MANIFOLD_SURFACE_SHAPE_REPRESENTATION)
    //         STEP's entity for the group of shells (!) passed
    //=======================================================================
    private StepShape_NonManifoldSurfaceShapeRepresentation getNMSSRForGroup(NCollection_HSequence<TopoDS_Shape> shapeGroup,
                                                                             Transfer_FinderProcess FP, out bool isNMSSRCreated)
    {
        StepShape_NonManifoldSurfaceShapeRepresentation aResult = null;

        if (shapeGroup is not null)
        {
            for (int i = 1; i <= shapeGroup.Length(); i++)
            {
                TopoDS_Shape aCurrentShape = shapeGroup.Value(i);
                TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(FP, aCurrentShape);
                if (FP.FindTypedTransient(mapper, typeof(StepShape_NonManifoldSurfaceShapeRepresentation), ref aResult))
                {
                    break;
                }
            }
        }

        if (aResult is null)
        {
            aResult = new StepShape_NonManifoldSurfaceShapeRepresentation();
            isNMSSRCreated = true;
        }
        else
        {
            isNMSSRCreated = false;
        }

        return aResult;
    }

    //=======================================================================
    // function : mergeInfoForNM
    // purpose  : bind already written shared faces to STEP entity for non-manifold
    //=======================================================================
    private void mergeInfoForNM(Transfer_FinderProcess theFP, Standard_Transient theInfo)
    {
        ShapeProcess_ShapeContext aContext = theInfo as ShapeProcess_ShapeContext;
        if (aContext is null)
        {
            return;
        }

        // OCCT iterates the map in its hash order; the port in the order of the shape processing (decision 1204)
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> aShapeShapeIt in aContext.Map())
        {
            TopoDS_Shape anOrig = aShapeShapeIt.Key, aRes = aShapeShapeIt.Value;
            if (anOrig.ShapeType() != TopAbs_FACE)
            {
                continue;
            }

            TransferBRep_ShapeMapper anOrigMapper = TransferBRep.ShapeMapper(theFP, anOrig);
            Transfer_Binder anOrigBinder = theFP.Find(anOrigMapper);
            if (anOrigBinder is null)
            {
                continue;
            }

            TransferBRep_ShapeMapper aResMapper = TransferBRep.ShapeMapper(theFP, aRes);
            theFP.Bind(aResMapper, anOrigBinder);
        }
    }

    /// <summary>
    /// Gets sequence of vertices of all compounds level by recursive @param[in] theShape shape to iterate, checked
    /// for compound type and sub shapes vertex type @param[out] theVertices sequence of found vertices via
    /// recursively iterate of shape @return TRUE if one or more vertex was found and all shapes were compound or
    /// vertex
    /// </summary>
    private bool separateShapeToSoloVertex(in TopoDS_Shape theShape, NCollection_Sequence<TopoDS_Shape> theVertices)
    {
        if (theShape.IsNull())
        {
            return false;
        }
        switch (theShape.ShapeType())
        {
            case TopAbs_COMPOUND:
                {
                    for (TopoDS_Iterator anIter = new TopoDS_Iterator(theShape); anIter.More(); anIter.Next())
                    {
                        if (!separateShapeToSoloVertex(anIter.Value(), theVertices))
                        {
                            return false;
                        }
                    }
                    break;
                }
            case TopAbs_VERTEX:
                {
                    theVertices.Append(theShape);
                    break;
                }
            default:
                {
                    theVertices.Clear();
                    return false;
                }
        }
        return true;
    }

    public void SetMode(STEPControl_StepModelType M)
    {
        switch (M)
        {
            case STEPControl_AsIs:
                ModeTrans() = 0;
                break;
            case STEPControl_ManifoldSolidBrep:
                ModeTrans() = 3;
                break;
            case STEPControl_BrepWithVoids:
                ModeTrans() = 5;
                break;
            case STEPControl_FacetedBrep:
                ModeTrans() = 1;
                break;
            case STEPControl_FacetedBrepAndBrepWithVoids:
                ModeTrans() = 6;
                break;
            case STEPControl_ShellBasedSurfaceModel:
                ModeTrans() = 2;
                break;
            case STEPControl_GeometricCurveSet:
                ModeTrans() = 4;
                break;
            case STEPControl_Hybrid:
                ModeTrans() = 0;
                break; // PAS IMPLEMENTE !!
            default:
                break;
        }
    }

    public STEPControl_StepModelType Mode()
    {
        switch (ModeTrans())
        {
            case 0:
                return STEPControl_AsIs;
            case 1:
                return STEPControl_FacetedBrep;
            case 2:
                return STEPControl_ShellBasedSurfaceModel;
            case 3:
                return STEPControl_ManifoldSolidBrep;
            case 4:
                return STEPControl_GeometricCurveSet;
            case 5:
                return STEPControl_BrepWithVoids;
            case 6:
                return STEPControl_FacetedBrepAndBrepWithVoids;
            default:
                break;
        }
        return STEPControl_AsIs;
    }

    public void SetGroupMode(int mode)
    {
        if (mode >= 0)
        {
            mygroup = mode;
        }
    }

    public int GroupMode()
    {
        return mygroup;
    }

    public void SetTolerance(double Tol)
    {
        mytoler = Tol;
    }

    /// <summary>The writing context (OCCT.NET: OCCT keeps it private).</summary>
    public STEPConstruct_ContextTool ContextTool()
    {
        return myContext;
    }

    //=======================================================================
    // function : Recognize
    //  ATTENTION, Recognize doit s aligner sur ce que Transfer sait faire
    // purpose  :
    //=======================================================================
    public override bool Recognize(Transfer_Finder start)
    {
        STEPControl_StepModelType mymode = Mode();
        TransferBRep_ShapeMapper mapper = start as TransferBRep_ShapeMapper;
        if (mapper is null)
        {
            return false;
        }
        if (mymode == STEPControl_AsIs)
        {
            return true;
        }

        bool yasolid = false, yashell = false, yaface = false;

        TopoDS_Shape theShape;
        //  theShape = TopoDSToStep::DirectFaces(mapper->Value());
        theShape = mapper.Value(); // pour une reconnaissance c est bien assez

        if (theShape.ShapeType() == TopAbs_COMPOUND)
        {
            TopExp_Explorer SolidExp = new TopExp_Explorer(), ShellExp = new TopExp_Explorer(), FaceExp = new TopExp_Explorer();

            for (SolidExp.Init(theShape, TopAbs_SOLID); SolidExp.More(); SolidExp.Next())
            {
                yasolid = true;
            }
            for (ShellExp.Init(theShape, TopAbs_SHELL, TopAbs_SOLID); ShellExp.More(); ShellExp.Next())
            {
                yashell = true;
            }
            for (FaceExp.Init(theShape, TopAbs_FACE, TopAbs_SHELL); FaceExp.More(); FaceExp.Next())
            {
                yaface = true;
            }
        }
        else if (theShape.ShapeType() == TopAbs_SOLID)
        {
            yasolid = true;
        }
        else if (theShape.ShapeType() == TopAbs_SHELL)
        {
            yashell = true;
        }
        else if (theShape.ShapeType() == TopAbs_FACE)
        {
            yaface = true;
        }
        else if (mymode != STEPControl_GeometricCurveSet)
        {
            return false;
        }
        //  pour wireframe ?

        //  Faceted : il est OBLIGATOIRE d avoir des surfaces support Plane et des
        //   courbes 3D Line (pcurves ignorees)

        if (mymode == STEPControl_FacetedBrep || mymode == STEPControl_FacetedBrepAndBrepWithVoids)
        {
            for (TopExp_Explorer ffac = new TopExp_Explorer(theShape, TopAbs_FACE); ffac.More(); ffac.Next())
            {
                TopoDS_Face F = TopoDS.Face(ffac.Current());
                TopLoc_Location locbid = new TopLoc_Location();
                Geom_Surface surf = BRep_Tool.Surface(F, out locbid);
                if (surf is null || !surf.IsKind(typeof(Geom_Plane)))
                {
                    return false;
                }
            }
            for (TopExp_Explorer fedg = new TopExp_Explorer(theShape, TopAbs_EDGE); fedg.More(); fedg.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(fedg.Current());
                TopLoc_Location locbid;
                Geom_Curve curv = BRep_Tool.Curve(E, out locbid, out double first, out double last);
                if (curv is null || !curv.IsKind(typeof(Geom_Line)))
                {
                    return false;
                }
            }
        }

        switch (mymode)
        {
            case STEPControl_ManifoldSolidBrep:
                return (yasolid || yashell);
            case STEPControl_BrepWithVoids:
            case STEPControl_FacetedBrep:
            case STEPControl_FacetedBrepAndBrepWithVoids:
                return yasolid;
            case STEPControl_ShellBasedSurfaceModel:
                return (yasolid || yashell || yaface);
            case STEPControl_GeometricCurveSet:
                return true; // tout OK
            default:
                break;
        }
        return false;
    }

    //  ########    MAKE PRODUCT DATA + CONTEXT    ########

    public override Transfer_Binder Transfer(Transfer_Finder start, Transfer_FinderProcess FP, Message_ProgressRange theProgress = default)
    {
        TransferBRep_ShapeMapper mapper = start as TransferBRep_ShapeMapper;

        if (mapper is null)
        {
            return NullResult();
        }
        TopoDS_Shape shape = mapper.Value();

        // init context
        StepData_StepModel model = FP.Model() as StepData_StepModel;
        if (model is not null)
        {
            myContext.SetModel(model); //: abv 04.11.00: take APD from model
        }
        myContext.AddAPD(false); // update APD
        myContext.SetLevel(1);   // set assembly level to 1 (to ensure)
        if (!model.IsInitializedUnit())
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; the port has no
            // process-wide unit: millimetres (STEPControl_Writer initialises the model's unit from its statics)
            model.SetLocalLengthUnit(UnitsMethods.GetCasCadeLengthUnit(null));
        }
        double aLFactor = model.WriteLengthUnit();
        aLFactor /= model.LocalLengthUnit();
        int anglemode = (int)model.InternalParameters.AngleUnit;
        StepData_Factors aLocalFactors = new StepData_Factors();
        aLocalFactors.InitializeFactors(aLFactor, (anglemode <= 1 ? 1.0 : Math.PI / 180.0), 1.0);
        // create SDR
        STEPConstruct_Part SDRTool = new STEPConstruct_Part();
        SDRTool.MakeSDR(null, myContext.GetProductName(), myContext.GetAPD().Application(), model);
        StepShape_ShapeDefinitionRepresentation sdr = SDRTool.SDRValue();
        // transfer shape

        Transfer_Binder resbind = TransferShape(mapper, sdr, FP, aLocalFactors, null, true, theProgress);

        // create binder with all root entities produced from shape
        NCollection_HSequence<Standard_Transient> roots = myContext.GetRootsForPart(SDRTool);
        Transfer_Binder resprod = TransientResult(myContext.GetAPD());
        for (int i = 1; i <= roots.Length(); i++)
        {
            resprod.AddResult(TransientResult(roots.Value(i)));
        }
        resprod.AddResult(resbind);

        // bind and exit
        // FP->Bind (mapper,resprod);
        myContext.NextIndex();
        return resprod;
    }

    //==========================================

    private static double UsedTolerance(StepData_StepModel theStepModel, double mytoler, in TopoDS_Shape theShape)
    {
        //    COMPUTING 3D TOLERANCE
        //    Either from Session, or Computed (Least,Average, or Greatest)
        //    Then given to TopoDSToStep_Tool
        double Tol = mytoler;
        int tolmod = (int)theStepModel.InternalParameters.WritePrecisionMode;
        if (Tol <= 0 && tolmod == 2)
        {
            Tol = theStepModel.InternalParameters.WritePrecisionVal;
        }
        if (Tol <= 0)
        {
            ShapeAnalysis_ShapeTolerance stu = new ShapeAnalysis_ShapeTolerance();
            Tol = stu.Tolerance(theShape, tolmod);
            //  Par defaut, on prend une tolerance moyenne, on elimine les aberrations
            Tol = Interface_MSG.Intervalled(Tol * 1.5); // arrondi a 1 2 5 ...
        }
        if (Tol == 0)
        {
            Tol = 1.0e-07; // minimum ...
        }

        return Tol;
    }

    /// <summary>
    /// Customizable method to check whether shape S should be written as assembly or not Default implementation uses
    /// flag GroupMode and analyses the shape itself NOTE: this method can modify shape
    /// </summary>
    // if GroupMode is >1 downgrades all compounds having single subshape to that
    // subshape
    public virtual bool IsAssembly(StepData_StepModel theModel, ref TopoDS_Shape S)
    {
        if (GroupMode() == 0 || S.ShapeType() != TopAbs_COMPOUND)
        {
            return false;
        }
        // PTV 16.09.2002  OCC725 for storing compound of vertices
        if (theModel.InternalParameters.WriteVertexMode == 0)
        { // bug 23950
            if (S.ShapeType() == TopAbs_COMPOUND)
            {
                bool IsOnlyVertices = true;
                TopoDS_Iterator anItr = new TopoDS_Iterator(S);
                for (; anItr.More(); anItr.Next())
                {
                    if (anItr.Value().ShapeType() != TopAbs_VERTEX)
                    {
                        IsOnlyVertices = false;
                        break;
                    }
                }
                if (IsOnlyVertices)
                {
                    return false;
                }
            }
        }
        if (GroupMode() == 1)
        {
            return true;
        }
        TopoDS_Iterator it = new TopoDS_Iterator(S);
        if (!it.More())
        {
            return false;
        }
        TopoDS_Shape shape = it.Value();
        it.Next();
        if (it.More())
        {
            return true;
        }
        S = shape;
        return IsAssembly(theModel, ref S);
    }

    // PTV 16.09.2002 added for transferring vertices.
    private static bool transferVertex(Transfer_FinderProcess FP, NCollection_HArray1<StepShape_GeometricSetSelect> aGSS,
                                       in TopoDS_Shape aShVrtx, int theNum, in StepData_Factors theLocalFactors)
    {
        bool IsDone = false;
        NCollection_DataMap<TopoDS_Shape, Standard_Transient> aMap =
            new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);
        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;
        TopoDSToStep_Tool aTool = new TopoDSToStep_Tool(aMap, true, aStepModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0);
        TopoDSToStep_MakeStepVertex aMkVrtx = new TopoDSToStep_MakeStepVertex(TopoDS.Vertex(aShVrtx), aTool, FP, theLocalFactors);

        if (!aMkVrtx.IsDone())
        {
            return IsDone;
        }

        StepShape_VertexPoint aVP = aTool.Find(aShVrtx) as StepShape_VertexPoint;
        if (aVP is null)
        {
            return IsDone;
        }

        StepShape_GeometricSetSelect select = new StepShape_GeometricSetSelect();
        select.SetValue(aVP.VertexGeometry());
        // add current result
        aGSS.SetValue(theNum, select);
        IsDone = true;
        return IsDone;
    }

    public Transfer_Binder TransferShape(Transfer_Finder start, StepShape_ShapeDefinitionRepresentation SDR0, Transfer_FinderProcess FP,
                                         in StepData_Factors theLocalFactors, NCollection_HSequence<TopoDS_Shape> shapeGroup = null,
                                         bool isManifold = true, Message_ProgressRange theProgress = default)
    {
        STEPControl_StepModelType mymode = Mode();
        TransferBRep_ShapeMapper mapper = start as TransferBRep_ShapeMapper;
        Transfer_Binder binder = null;
        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;

        // Indicates whether to use an existing NMSSR to write items to (ss; 13.11.2010)
        bool useExistingNMSSR = false;

        if (mapper is null)
        {
            return binder;
        }
        TopoDS_Shape theShape = mapper.Value();

        if (theShape.IsNull())
        {
            return binder;
        }

        // INDIVIDUAL SHAPE ALREADY TRANSFERRED : RETURN IT !
        binder = FP.Find(start);
        if (binder is not null)
        {
            if (!binder.HasResult())
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
        if (IsAssembly(aStepModel, ref theShape))
        {
            return TransferCompound(start, SDR0, FP, theLocalFactors, theProgress);
        }

        using Message_ProgressScope aPSRoot = new Message_ProgressScope(theProgress, null, 2);

        // [BEGIN] Separate manifold topology from non-manifold in group mode 0 (ssv; 18.11.2010)
        bool isNMMode = aStepModel.InternalParameters.WriteNonmanifold;
        Transfer_Binder aNMBinder = null;
        if (isNMMode && GroupMode() == 0 && theShape.ShapeType() == TopAbs_COMPOUND)
        {
            TopoDS_Compound aNMCompound = new TopoDS_Compound();
            TopoDS_Compound aManifoldCompound = new TopoDS_Compound();
            BRep_Builder brepBuilder = new BRep_Builder();

            // Create empty Compounds
            brepBuilder.MakeCompound(ref aManifoldCompound);
            brepBuilder.MakeCompound(ref aNMCompound);

            // Indicates whether there is only non-manifold topology detected
            // (there is no manifold topology found in the Compound passed)
            bool isOnlyNonManifold = false;

            // Find a Compound containing non-manifold topology.
            // NOTE: only one such Compound must exist in the entire Compound passed
            if (!IsManifoldShape(theShape))
            {
                aNMCompound = TopoDS.Compound(theShape);
                isOnlyNonManifold = true;
            }
            else
            {
                // OCCT appends to the list while iterating it: an index walks the growing list
                List<TopoDS_Shape> aListOfShapes = new List<TopoDS_Shape>();
                NCollection_List<TopoDS_Shape> aListOfManifoldShapes = new NCollection_List<TopoDS_Shape>();
                aListOfShapes.Add(theShape);

                for (int iL = 0; iL < aListOfShapes.Count; iL++)
                {
                    TopoDS_Shape aParentShape = aListOfShapes[iL];
                    TopoDS_Iterator it = new TopoDS_Iterator(aParentShape);
                    for (; it.More(); it.Next())
                    {
                        TopoDS_Shape aSubShape = it.Value();
                        if (aSubShape.ShapeType() == TopAbs_COMPOUND && !IsManifoldShape(aSubShape))
                        {
                            aNMCompound = TopoDS.Compound(aSubShape);
                        }
                        else if (aSubShape.ShapeType() == TopAbs_COMPOUND)
                        {
                            aListOfShapes.Add(aSubShape);
                        }
                        else
                        {
                            aListOfManifoldShapes.Append(aSubShape);
                        }
                    }
                }

                // Group manifold topology together.
                // NOTE: there is no sense that initial Compound structure was lost as
                //       group mode is set to 0 (no Assemblies are mapped)
                foreach (TopoDS_Shape aCurrentManiShape in aListOfManifoldShapes)
                {
                    brepBuilder.Add(aManifoldCompound, aCurrentManiShape);
                }
            }

            // Process only manifold topology in the current TransferShape invocation.
            // Invoke TransferShape for non-manifold topology separately (see below)
            theShape = aManifoldCompound;

            // List of items to transfer
            NCollection_HSequence<TopoDS_Shape> RepItemSeq1 = new NCollection_HSequence<TopoDS_Shape>();
            // Non-manifold group to pass into TransferShape with each shape from RepItemSeq
            NCollection_HSequence<TopoDS_Shape> NonManifoldGroup = new NCollection_HSequence<TopoDS_Shape>();

            // Transfer Solids to closed Shells. Prepare RepItemSeq & NonManifoldGroup
            for (TopoDS_Iterator iter = new TopoDS_Iterator(aNMCompound); iter.More(); iter.Next())
            {
                TopoDS_Shape aSubShape = iter.Value();
                if (aSubShape.ShapeType() == TopAbs_SOLID)
                {
                    for (TopoDS_Iterator aSubIter = new TopoDS_Iterator(aSubShape); aSubIter.More(); aSubIter.Next())
                    {
                        TopoDS_Shell aSubShell = ClosedShell(TopoDS.Shell(aSubIter.Value())); // OCCT: aSubShell.Closed(true)
                        RepItemSeq1.Append(aSubShell);
                        NonManifoldGroup.Append(aSubShell);
                    }
                }
                else if (!isManifold && (aSubShape.ShapeType() == TopAbs_SHELL))
                {
                    RepItemSeq1.Append(aSubShape);
                    NonManifoldGroup.Append(aSubShape);
                }
                else
                {
                    RepItemSeq1.Append(iter.Value());
                }
            }

            int aNMItemsNb = RepItemSeq1.Length();

            // In case of pure manifold topology do nothing; theShape is processed as usual (see below)
            if (aNMItemsNb > 0)
            {
                // Prepare SDR for non-manifold group. This SDR will be linked to NMSSR by means
                // of TransferShape invocation. SDR is not created if there is no any manifold
                // topology in the passed Compound. If topology is pure non-manifold, SDR0 (passed)
                // is used
                StepShape_ShapeDefinitionRepresentation sdr;
                if (isOnlyNonManifold)
                {
                    sdr = SDR0;
                }
                else
                {
                    STEPConstruct_Part SDRTool = new STEPConstruct_Part();
                    SDRTool.MakeSDR(null, myContext.GetProductName(), myContext.GetAPD().Application(), aStepModel);
                    sdr = SDRTool.SDRValue();
                }

                aNMBinder = TransientResult(sdr);

                // Complete SDR with shape representations.
                // NOTE: aNMBinder is connected now with this SDR. It will be added to the resulting
                //       binder in the end of this invocation of TransferShape
                using Message_ProgressScope aPS0 = new Message_ProgressScope(aPSRoot.Next(), null, aNMItemsNb);
                for (int i = 1; i <= aNMItemsNb && aPS0.More(); i++)
                {
                    TransferBRep_ShapeMapper aMapper = TransferBRep.ShapeMapper(FP, RepItemSeq1.Value(i));
                    TransferShape(aMapper, sdr, FP, theLocalFactors, NonManifoldGroup, false, aPS0.Next());
                }

                // Nothing else needed for pure non-manifold topology, return
                if (isOnlyNonManifold)
                {
                    return aNMBinder;
                }
            }
        }
        // [END] Separate manifold topology from non-manifold in group mode 0 (ssv; 18.11.2010)

        if (aPSRoot.UserBreak())
        {
            return null;
        }

        // create a list of items to translate
        NCollection_HSequence<TopoDS_Shape> RepItemSeq = new NCollection_HSequence<TopoDS_Shape>();

        bool isSeparateVertices = aStepModel.InternalParameters.WriteVertexMode == 0; // bug 23950
        // PTV 16.09.2002 OCC725 separate shape from solo vertices.
        bool isOnlyVertices = false;
        if (theShape.ShapeType() == TopAbs_COMPOUND && isSeparateVertices)
        {
            TopoDS_Compound aNewShape = new TopoDS_Compound(), aCompOfVrtx = new TopoDS_Compound();
            BRep_Builder aBuilder = new BRep_Builder();
            aBuilder.MakeCompound(ref aNewShape);
            aBuilder.MakeCompound(ref aCompOfVrtx);
            NCollection_Sequence<TopoDS_Shape> aVertices = new NCollection_Sequence<TopoDS_Shape>();
            isOnlyVertices = separateShapeToSoloVertex(theShape, aVertices);
            if (!isOnlyVertices)
            {
                for (TopoDS_Iterator anCompIt = new TopoDS_Iterator(theShape); anCompIt.More(); anCompIt.Next())
                {
                    TopoDS_Shape aCurSh = anCompIt.Value();
                    NCollection_Sequence<TopoDS_Shape> aVerticesOfSubSh = new NCollection_Sequence<TopoDS_Shape>();
                    if (separateShapeToSoloVertex(aCurSh, aVerticesOfSubSh))
                    {
                        aVertices.Append(aVerticesOfSubSh);
                    }
                    else
                    {
                        aBuilder.Add(aNewShape, aCurSh);
                    }
                }
                theShape = aNewShape;
            }
            foreach (TopoDS_Shape aVertex in aVertices)
            {
                aBuilder.Add(aCompOfVrtx, aVertex);
            }
            if (!aVertices.IsEmpty())
            {
                RepItemSeq.Append(aCompOfVrtx);
            }
        }

        if (theShape.ShapeType() == TopAbs_COMPOUND)
        {
            TopExp_Explorer SolidExp = new TopExp_Explorer(), ShellExp = new TopExp_Explorer(), FaceExp = new TopExp_Explorer();
            if (mymode != STEPControl_GeometricCurveSet)
            {
                for (SolidExp.Init(theShape, TopAbs_SOLID); SolidExp.More(); SolidExp.Next())
                {
                    RepItemSeq.Append(TopoDS.Solid(SolidExp.Current()));
                }
                for (ShellExp.Init(theShape, TopAbs_SHELL, TopAbs_SOLID); ShellExp.More(); ShellExp.Next())
                {
                    RepItemSeq.Append(TopoDS.Shell(ShellExp.Current()));
                }

                for (FaceExp.Init(theShape, TopAbs_FACE, TopAbs_SHELL); FaceExp.More(); FaceExp.Next())
                {
                    RepItemSeq.Append(TopoDS.Face(FaceExp.Current()));
                }
            }
            else
            {
                if (!isOnlyVertices)
                {
                    RepItemSeq.Append(theShape); //: j1
                }
            }
            if (mymode == STEPControl_AsIs)
            {
                TopExp_Explorer WireExp = new TopExp_Explorer(), EdgeExp = new TopExp_Explorer();
                for (WireExp.Init(theShape, TopAbs_WIRE, TopAbs_FACE); WireExp.More(); WireExp.Next())
                {
                    RepItemSeq.Append(TopoDS.Wire(WireExp.Current()));
                }
                for (EdgeExp.Init(theShape, TopAbs_EDGE, TopAbs_WIRE); EdgeExp.More(); EdgeExp.Next())
                {
                    RepItemSeq.Append(TopoDS.Edge(EdgeExp.Current()));
                }
            }
        }
        else if (theShape.ShapeType() == TopAbs_SOLID)
        {
            RepItemSeq.Append(TopoDS.Solid(theShape));
        }
        else if (theShape.ShapeType() == TopAbs_SHELL)
        {
            RepItemSeq.Append(TopoDS.Shell(theShape));
        }
        else if (theShape.ShapeType() == TopAbs_FACE)
        {
            RepItemSeq.Append(TopoDS.Face(theShape));
        }
        else if (theShape.ShapeType() == TopAbs_COMPSOLID)
        {
            FP.AddWarning(start, "NonManifold COMPSOLID was translated like a set of SOLIDs");
            if (GroupMode() > 0)
            {
                return TransferCompound(start, SDR0, FP, theLocalFactors, aPSRoot.Next());
            }
            else
            {
                TopExp_Explorer SolidExp = new TopExp_Explorer();
                for (SolidExp.Init(theShape, TopAbs_SOLID); SolidExp.More(); SolidExp.Next())
                {
                    RepItemSeq.Append(TopoDS.Solid(SolidExp.Current()));
                }
            }
        }

        else if (mymode != STEPControl_GeometricCurveSet && mymode != STEPControl_AsIs)
        {
            FP.AddFail(start, "The Shape is not a SOLID, nor a SHELL, nor a FACE");
            return binder;
        }
        else
        {
            RepItemSeq.Append(theShape);
        }

        //    COMPUTING 3D TOLERANCE
        //    Either from Session, or Computed (Least,Average, or Greatest)
        //    Then given to TopoDSToStep_Tool
        double Tol = UsedTolerance(aStepModel, mytoler, theShape);

        // Create a STEP-Entity for each TopoDS_Shape
        // according to the current StepModelMode

        int nbs = RepItemSeq.Length();
        NCollection_HSequence<Standard_Transient> ItemSeq = new NCollection_HSequence<Standard_Transient>();

        // ptv 10.11.00: allow to write empty Compound:  if (GroupMode() >0)
        ItemSeq.Append(myContext.GetDefaultAxis());
        STEPControl_StepModelType trmode = mymode;
        using Message_ProgressScope aPS = new Message_ProgressScope(aPSRoot.Next(), null, nbs);
        for (int i = 1; i <= nbs && aPS.More(); i++)
        {
            TopoDS_Shape xShape = RepItemSeq.Value(i);

            if (mymode == STEPControl_AsIs)
            {
                switch (xShape.ShapeType())
                {
                    case TopAbs_SOLID:
                        trmode = STEPControl_ManifoldSolidBrep;
                        break;
                    case TopAbs_SHELL:
                        trmode = STEPControl_ShellBasedSurfaceModel;
                        break;
                    case TopAbs_FACE:
                        trmode = STEPControl_ShellBasedSurfaceModel;
                        break;
                    default:
                        trmode = STEPControl_GeometricCurveSet;
                        break;
                }
            }

            using Message_ProgressScope aPS1 = new Message_ProgressScope(aPS.Next(), null, 2);

            TopoDS_Shape aShape = xShape;

            NCollection_DataMap<string, string> aParameters = GetShapeFixParameters();
            XSAlgo_ShapeProcessor.SetParameter("FixShape.Tolerance3d", Tol, true, aParameters);
            XSAlgo_ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", aStepModel.InternalParameters.ReadMaxPrecisionVal, true,
                                               aParameters);
            XSAlgo_ShapeProcessor aShapeProcessor = new XSAlgo_ShapeProcessor(aParameters);

            if (hasGeometry(aShape))
            {
                aShape = aShapeProcessor.ProcessShape(xShape, GetShapeProcessFlags().Item1, aPS1.Next());

                if (aPS1.UserBreak())
                {
                    return null;
                }
            }

            if (!isManifold)
            {
                mergeInfoForNM(FP, aShapeProcessor.GetContext());
            }

            // create a STEP entity corresponding to shape
            StepGeom_GeometricRepresentationItem item = null, itemTess = null;
            switch (trmode)
            {
                case STEPControl_ManifoldSolidBrep:
                    {
                        if (aShape.ShapeType() == TopAbs_SOLID)
                        {
                            TopoDS_Solid aSolid = TopoDS.Solid(aShape);

                            //: d6 abv 13 Mar 98: if solid has more than 1 shell,
                            // try to treat it as solid with voids
                            int nbShells = 0;
                            for (TopoDS_Iterator It = new TopoDS_Iterator(aSolid); It.More(); It.Next())
                            {
                                if (It.Value().ShapeType() == TopAbs_SHELL)
                                {
                                    nbShells++;
                                }
                            }
                            if (nbShells > 1)
                            {
                                TopoDSToStep_MakeBrepWithVoids MkBRepWithVoids = new TopoDSToStep_MakeBrepWithVoids(aSolid, FP, theLocalFactors, aPS1.Next());
                                MkBRepWithVoids.Tolerance() = Tol;
                                if (MkBRepWithVoids.IsDone())
                                {
                                    item = MkBRepWithVoids.Value();
                                    itemTess = MkBRepWithVoids.TessellatedValue();
                                }
                                else
                                {
                                    nbShells = 1; // smth went wrong; let it will be just Manifold
                                }
                            }
                            if (nbShells == 1)
                            {
                                TopoDSToStep_MakeManifoldSolidBrep MkManifoldSolidBrep = new TopoDSToStep_MakeManifoldSolidBrep(aSolid, FP, theLocalFactors, aPS1.Next());
                                MkManifoldSolidBrep.Tolerance() = Tol;
                                if (MkManifoldSolidBrep.IsDone())
                                {
                                    item = MkManifoldSolidBrep.Value();
                                    itemTess = MkManifoldSolidBrep.TessellatedValue();
                                }
                            }
                        }
                        else if (aShape.ShapeType() == TopAbs_SHELL)
                        {
                            TopoDS_Shell aShell = TopoDS.Shell(aShape);
                            TopoDSToStep_MakeManifoldSolidBrep MkManifoldSolidBrep = new TopoDSToStep_MakeManifoldSolidBrep(aShell, FP, theLocalFactors, aPS1.Next());
                            MkManifoldSolidBrep.Tolerance() = Tol;
                            if (MkManifoldSolidBrep.IsDone())
                            {
                                item = MkManifoldSolidBrep.Value();
                                itemTess = MkManifoldSolidBrep.TessellatedValue();
                            }
                        }
                        break;
                    }
                case STEPControl_BrepWithVoids:
                    {
                        if (aShape.ShapeType() == TopAbs_SOLID)
                        {
                            TopoDS_Solid aSolid = TopoDS.Solid(aShape);
                            TopoDSToStep_MakeBrepWithVoids MkBRepWithVoids = new TopoDSToStep_MakeBrepWithVoids(aSolid, FP, theLocalFactors, aPS1.Next());
                            MkBRepWithVoids.Tolerance() = Tol;
                            if (MkBRepWithVoids.IsDone())
                            {
                                item = MkBRepWithVoids.Value();
                                itemTess = MkBRepWithVoids.TessellatedValue();
                            }
                        }
                        break;
                    }
                case STEPControl_FacetedBrep:
                    {
                        TopoDSToStep_FacetedError facErr = TopoDSToStep_FacetedTool.CheckTopoDSShape(aShape);
                        if (facErr != TopoDSToStep_FacetedDone)
                        {
                            FP.AddFail(start, "Error in Faceted Shape from TopoDS");
                            if (facErr == TopoDSToStep_SurfaceNotPlane)
                            {
                                FP.AddFail(start, "-- The TopoDS_Face is not plane");
                            }
                            else if (facErr == TopoDSToStep_PCurveNotLinear)
                            {
                                FP.AddFail(start, "-- The Face contains non linear PCurves");
                            }
                            return binder;
                        }
                        if (aShape.ShapeType() == TopAbs_SOLID)
                        {
                            TopoDS_Solid aSolid = TopoDS.Solid(aShape);
                            TopoDSToStep_MakeFacetedBrep MkFacetedBrep = new TopoDSToStep_MakeFacetedBrep(aSolid, FP, theLocalFactors, aPS1.Next());
                            MkFacetedBrep.Tolerance() = Tol;
                            if (MkFacetedBrep.IsDone())
                            {
                                item = MkFacetedBrep.Value();
                                itemTess = MkFacetedBrep.TessellatedValue();
                            }
                        }
                        break;
                    }
                case STEPControl_FacetedBrepAndBrepWithVoids:
                    {
                        TopoDSToStep_FacetedError facErr = TopoDSToStep_FacetedTool.CheckTopoDSShape(aShape);
                        if (facErr != TopoDSToStep_FacetedDone)
                        {
                            FP.AddFail(start, "Error in Faceted Shape from TopoDS");
                            if (facErr == TopoDSToStep_SurfaceNotPlane)
                            {
                                FP.AddFail(start, "-- The TopoDS_Face is not plane");
                            }
                            else if (facErr == TopoDSToStep_PCurveNotLinear)
                            {
                                FP.AddFail(start, "-- The Face contains non linear PCurves");
                            }
                            return binder;
                        }
                        if (aShape.ShapeType() == TopAbs_SOLID)
                        {
                            TopoDS_Solid aSolid = TopoDS.Solid(aShape);
                            TopoDSToStep_MakeFacetedBrepAndBrepWithVoids MkFacetedBrepAndBrepWithVoids =
                                new TopoDSToStep_MakeFacetedBrepAndBrepWithVoids(aSolid, FP, theLocalFactors, aPS1.Next());
                            MkFacetedBrepAndBrepWithVoids.Tolerance() = Tol;
                            if (MkFacetedBrepAndBrepWithVoids.IsDone())
                            {
                                item = MkFacetedBrepAndBrepWithVoids.Value();
                                itemTess = MkFacetedBrepAndBrepWithVoids.TessellatedValue();
                            }
                        }
                        break;
                    }
                case STEPControl_ShellBasedSurfaceModel:
                    {
                        if (aShape.ShapeType() == TopAbs_SOLID)
                        {
                            TopoDS_Solid aSolid = TopoDS.Solid(aShape);
                            TopoDSToStep_MakeShellBasedSurfaceModel MkShellBasedSurfaceModel =
                                new TopoDSToStep_MakeShellBasedSurfaceModel(aSolid, FP, theLocalFactors, aPS1.Next());
                            MkShellBasedSurfaceModel.Tolerance() = Tol;
                            if (MkShellBasedSurfaceModel.IsDone())
                            {
                                item = MkShellBasedSurfaceModel.Value();
                                itemTess = MkShellBasedSurfaceModel.TessellatedValue();
                            }
                        }
                        else if (aShape.ShapeType() == TopAbs_SHELL)
                        {
                            TopoDS_Shell aShell = TopoDS.Shell(aShape);
                            // Non-manifold topology is stored via NMSSR containing series of SBSM (ssv; 13.11.2010)
                            TopoDSToStep_MakeShellBasedSurfaceModel MkShellBasedSurfaceModel =
                                new TopoDSToStep_MakeShellBasedSurfaceModel(aShell, FP, theLocalFactors, aPS1.Next());
                            MkShellBasedSurfaceModel.Tolerance() = Tol;
                            if (MkShellBasedSurfaceModel.IsDone())
                            {
                                item = MkShellBasedSurfaceModel.Value();
                                itemTess = MkShellBasedSurfaceModel.TessellatedValue();
                            }
                        }
                        else if (aShape.ShapeType() == TopAbs_FACE)
                        {
                            TopoDS_Face aFace = TopoDS.Face(aShape);
                            TopoDSToStep_MakeShellBasedSurfaceModel MkShellBasedSurfaceModel =
                                new TopoDSToStep_MakeShellBasedSurfaceModel(aFace, FP, theLocalFactors, aPS1.Next());
                            MkShellBasedSurfaceModel.Tolerance() = Tol;
                            if (MkShellBasedSurfaceModel.IsDone())
                            {
                                item = MkShellBasedSurfaceModel.Value();
                                itemTess = MkShellBasedSurfaceModel.TessellatedValue();
                            }
                        }
                        break;
                    }
                case STEPControl_GeometricCurveSet:
                    {
                        TopoDSToStep_MakeGeometricCurveSet MkGeometricCurveSet = new TopoDSToStep_MakeGeometricCurveSet(aShape, FP, theLocalFactors);
                        MkGeometricCurveSet.Tolerance() = Tol;
                        if (MkGeometricCurveSet.IsDone())
                        {
                            item = MkGeometricCurveSet.Value();
                        }
                        // PTV 22.08.2002 OCC609 ------------------------- begin --------------------
                        // modified by PTV 16.09.2002 OCC725
                        else if (aShape.ShapeType() == TopAbs_COMPOUND || aShape.ShapeType() == TopAbs_VERTEX)
                        {
                            // it is compound with solo vertices.
                            int aNbVrtx = 0;
                            int curNb = 0;
                            TopExp_Explorer anExp = new TopExp_Explorer(aShape, TopAbs_VERTEX);
                            for (; anExp.More(); anExp.Next())
                            {
                                if (anExp.Current().ShapeType() != TopAbs_VERTEX)
                                {
                                    continue;
                                }
                                aNbVrtx++;
                            }
                            if (aNbVrtx != 0)
                            {
                                // create new geometric curve set for all vertices
                                NCollection_HArray1<StepShape_GeometricSetSelect> aGSS =
                                    new NCollection_HArray1<StepShape_GeometricSetSelect>(1, aNbVrtx);
                                TCollection_HAsciiString empty = new TCollection_HAsciiString("");
                                StepShape_GeometricCurveSet aGCSet = new StepShape_GeometricCurveSet();
                                aGCSet.SetName(empty);
                                // iterates on compound with vertices and traces each vertex
                                for (anExp.ReInit(); anExp.More(); anExp.Next())
                                {
                                    TopoDS_Shape aVertex = anExp.Current();
                                    if (aVertex.ShapeType() != TopAbs_VERTEX)
                                    {
                                        continue;
                                    }
                                    curNb++;
                                    transferVertex(FP, aGSS, aVertex, curNb, theLocalFactors);
                                } // end of iteration on compound with vertices.
                                aGCSet.SetElements(aGSS);
                                item = aGCSet;
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
                ItemSeq.Append(item);
                TransferBRep_ShapeMapper submapper = null;
                if (xShape.IsSame(mapper.Value()))
                {
                    submapper = start as TransferBRep_ShapeMapper;
                }
                if (submapper is null)
                {
                    submapper = TransferBRep.ShapeMapper(FP, xShape);
                }
                Transfer_Binder subbind = FP.Find(submapper);
                if (subbind is null)
                {
                    subbind = TransientResult(item);
                    FP.Bind(submapper, subbind);
                }
                else
                {
                    subbind.AddResult(TransientResult(item));
                }
            }
            if (itemTess is not null)
            {
                ItemSeq.Append(itemTess);
                TransferBRep_ShapeMapper submapper = null;
                if (xShape.IsSame(mapper.Value()))
                {
                    submapper = start as TransferBRep_ShapeMapper;
                }
                if (submapper is null)
                {
                    submapper = TransferBRep.ShapeMapper(FP, xShape);
                }
                Transfer_Binder subbind = FP.Find(submapper);
                if (subbind is null)
                {
                    subbind = TransientResult(itemTess);
                    FP.Bind(submapper, subbind);
                }
                else
                {
                    subbind.AddResult(TransientResult(itemTess));
                }
            }

            // If shape is not processed, this function does nothing. Intentionally.
            aShapeProcessor.MergeTransferInfo(FP);
        }

        // - Make Shape Representation
        int nCc1 = ItemSeq.Length();
        if (nCc1 < 1)
        {
            FP.AddFail(start, "The Shape has not the appropriate type");
            return binder;
        }
        StepShape_ShapeRepresentation shapeRep = null;
        if (theShape.ShapeType() == TopAbs_SHAPE)
        { // for external references
            shapeRep = new StepShape_ShapeRepresentation();
        }
        else
        {
            switch (mymode)
            {
                case STEPControl_ManifoldSolidBrep:
                    shapeRep = new StepShape_AdvancedBrepShapeRepresentation();
                    break;
                case STEPControl_FacetedBrep:
                    shapeRep = new StepShape_FacetedBrepShapeRepresentation();
                    break;
                // NOTE: STEPControl_AsIs mode is normally used to transfer non-manifold topology.
                //       However, as ShellBasedSurfaceModel is used in non-manifold processing
                //       internally, STEPControl_ShellBasedSurfaceModel is also adjusted to
                //       be able to work with non-manifold cases
                case STEPControl_ShellBasedSurfaceModel:
                    if (isManifold)
                    {
                        shapeRep = new StepShape_ManifoldSurfaceShapeRepresentation();
                    }
                    else
                    {
                        shapeRep = getNMSSRForGroup(shapeGroup, FP, out bool isNewNMSSRCreated);
                        useExistingNMSSR = !isNewNMSSRCreated;
                    }
                    break;
                case STEPControl_GeometricCurveSet:
                    shapeRep = new StepShape_GeometricallyBoundedWireframeShapeRepresentation();
                    break;
                case STEPControl_AsIs:
                    {
                        if (nbs == 1)
                        {
                            if (trmode == STEPControl_ManifoldSolidBrep)
                            {
                                shapeRep = new StepShape_AdvancedBrepShapeRepresentation();
                            }
                            else if (trmode == STEPControl_ShellBasedSurfaceModel)
                            {
                                // Process non-manifold topology separately (ssv; 13.11.2010)
                                if (isManifold)
                                {
                                    shapeRep = new StepShape_ManifoldSurfaceShapeRepresentation();
                                }
                                else
                                {
                                    shapeRep = getNMSSRForGroup(shapeGroup, FP, out bool isNewNMSSRCreated);
                                    useExistingNMSSR = !isNewNMSSRCreated;
                                }
                            }
                            else if (trmode == STEPControl_GeometricCurveSet)
                            {
                                shapeRep = new StepShape_GeometricallyBoundedWireframeShapeRepresentation();
                            }
                            else if (trmode == STEPControl_FacetedBrep)
                            {
                                shapeRep = new StepShape_FacetedBrepShapeRepresentation();
                            }
                        }
                        else
                        {
                            shapeRep = new StepShape_ShapeRepresentation();
                        }
                    }
                    break;
                default:
                    break;
            }
        }
        if (shapeRep is null)
        {
            Transfer_Binder resb = null;
            return resb;
        }

        NCollection_HArray1<StepRepr_RepresentationItem> items = new NCollection_HArray1<StepRepr_RepresentationItem>(1, nCc1);

        for (int rep = 1; rep <= nCc1; rep++)
        {
            StepRepr_RepresentationItem repit = ItemSeq.Value(rep) as StepRepr_RepresentationItem;
            items.SetValue(rep, repit);
        }
        int ap = (int)aStepModel.InternalParameters.WriteSchema;
        NCollection_Sequence<Transfer_Binder> aSeqBindRelation = new NCollection_Sequence<Transfer_Binder>();
        if (ap == 3 && nbs > 1)
        {
            int j = 1;
            if (items.Value(j).IsKind(typeof(StepGeom_Axis2Placement3d)))
            {
                NCollection_HArray1<StepRepr_RepresentationItem> axis = new NCollection_HArray1<StepRepr_RepresentationItem>(1, 1);
                axis.SetValue(1, items.Value(j++));
                shapeRep.SetItems(axis);
            }
            for (; j <= items.Length(); j++)
            {
                StepShape_ShapeRepresentation ShapeRepr1;
                if (items.Value(j).IsKind(typeof(StepShape_ManifoldSolidBrep)))
                {
                    ShapeRepr1 = new StepShape_AdvancedBrepShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(StepShape_ShellBasedSurfaceModel)))
                {
                    ShapeRepr1 = new StepShape_ManifoldSurfaceShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(StepShape_GeometricCurveSet)))
                {
                    ShapeRepr1 = new StepShape_GeometricallyBoundedWireframeShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(StepShape_FacetedBrep)))
                {
                    ShapeRepr1 = new StepShape_FacetedBrepShapeRepresentation();
                }
                else if (items.Value(j).IsKind(typeof(StepVisual_TessellatedItem)))
                {
                    ShapeRepr1 = new StepVisual_TessellatedShapeRepresentation();
                }
                else
                {
                    ShapeRepr1 = new StepShape_ShapeRepresentation();
                }

                NCollection_HArray1<StepRepr_RepresentationItem> repr1 = new NCollection_HArray1<StepRepr_RepresentationItem>(1, 2);
                repr1.SetValue(1, myContext.GetDefaultAxis());
                repr1.SetValue(2, items.Value(j));
                ShapeRepr1.SetItems(repr1);
                STEPConstruct_UnitContext mk1 = new STEPConstruct_UnitContext();
                mk1.Init(Tol, aStepModel, theLocalFactors);
                ShapeRepr1.SetContextOfItems(mk1.Value()); // la tolerance, voir au debut
                ShapeRepr1.SetName(new TCollection_HAsciiString(""));

                StepRepr_ShapeRepresentationRelationship aShapeRel = new StepRepr_ShapeRepresentationRelationship();
                TCollection_HAsciiString aName = new TCollection_HAsciiString("");
                TCollection_HAsciiString aDescr = new TCollection_HAsciiString("");
                aShapeRel.SetName(aName);
                aShapeRel.SetDescription(aDescr);
                aShapeRel.SetRep2(shapeRep);
                aShapeRel.SetRep1(ShapeRepr1);

                aSeqBindRelation.Append(TransientResult(aShapeRel));
            }
        }
        else
        {
            int nC = 0;
            for (int i = 1; i <= items.Length(); i++)
            {
                if (!items.Value(i).IsKind(typeof(StepVisual_TessellatedItem)))
                {
                    continue;
                }
                ++nC;
            }
            if (nC > 0)
            {
                NCollection_HArray1<StepRepr_RepresentationItem> itemsTess = new NCollection_HArray1<StepRepr_RepresentationItem>(1, nC);
                int i = 1;
                for (int j = 1; j <= items.Length(); j++)
                {
                    if (!items.Value(j).IsKind(typeof(StepVisual_TessellatedItem)))
                    {
                        continue;
                    }
                    itemsTess.SetValue(i++, items.Value(j));
                }

                StepShape_ShapeRepresentation shapeTessRepr = new StepVisual_TessellatedShapeRepresentation();
                shapeTessRepr.SetItems(itemsTess);
                STEPConstruct_UnitContext mk1 = new STEPConstruct_UnitContext();
                mk1.Init(Tol, aStepModel, theLocalFactors);
                shapeTessRepr.SetContextOfItems(mk1.Value());
                shapeTessRepr.SetName(new TCollection_HAsciiString(""));

                aSeqBindRelation.Append(TransientResult(shapeTessRepr));
            }
            if (!useExistingNMSSR)
            {
                shapeRep.SetItems(items);
            }
            else
            {
                // Add new representation item to the NMSSR's existing collection (ssv; 13.11.2010)
                NCollection_HArray1<StepRepr_RepresentationItem> oldItems = shapeRep.Items();
                NCollection_HArray1<StepRepr_RepresentationItem> newItems =
                    new NCollection_HArray1<StepRepr_RepresentationItem>(1, oldItems.Length() + 1);
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
        STEPConstruct_UnitContext mk = new STEPConstruct_UnitContext();
        mk.Init(Tol, aStepModel, theLocalFactors);
        shapeRep.SetContextOfItems(mk.Value()); // la tolerance, voir au debut
        shapeRep.SetName(new TCollection_HAsciiString(""));

        // Create SDR (only once for non-manifold group)
        if (!useExistingNMSSR)
        {
            SDR0.SetUsedRepresentation(shapeRep);
            // create binder for SR and attach to it binder for RepItem (if exists)
            Transfer_Binder resbind = TransientResult(shapeRep);
            binder = FP.Find(start);
            if (binder is not null)
            {
                resbind.AddResult(binder);
                FP.Rebind(start, resbind);
                // binder->AddResult ( resbind );
                // resbind = binder;
            }
            for (int k = 1; k <= aSeqBindRelation.Length(); k++)
            {
                resbind.AddResult(aSeqBindRelation.Value(k));
            }

            // Add SDR for non-manifold topology in group mode 0 (ssv; 18.11.2010)
            if (aNMBinder is not null)
            {
                resbind.AddResult(aNMBinder);
            }

            return resbind;
        }
        else
        {
            return FP.Find(start);
        }
    }

    public Transfer_Binder TransferShape(Transfer_Finder start, StepShape_ShapeDefinitionRepresentation SDR, Transfer_FinderProcess FP)
        => TransferShape(start, SDR, FP, new StepData_Factors());

    //=======================================================================
    // function : TransferCompound
    //    ####    TRANSFER COMPOUND AS (SUB-)ASSEMBLY
    // purpose  :
    //=======================================================================
    public Transfer_Binder TransferCompound(Transfer_Finder start, StepShape_ShapeDefinitionRepresentation SDR0, Transfer_FinderProcess FP,
                                            in StepData_Factors theLocalFactors, Message_ProgressRange theProgress = default)
    {
        TransferBRep_ShapeMapper mapper = start as TransferBRep_ShapeMapper;
        Transfer_Binder binder = null;
        if (mapper is null)
        {
            return binder;
        }
        TopoDS_Shape theShape = mapper.Value();

        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;

        // Inspect non-manifold topology case (ssv; 10.11.2010)
        bool isNMMode = aStepModel.InternalParameters.WriteNonmanifold;
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
        NCollection_HSequence<TopoDS_Shape> RepItemSeq = new NCollection_HSequence<TopoDS_Shape>();
        // Prepare a collection for non-manifold group of shapes
        NCollection_HSequence<TopoDS_Shape> NonManifoldGroup = new NCollection_HSequence<TopoDS_Shape>();
        bool isSeparateVertices = (aStepModel.InternalParameters.WriteVertexMode == 0); // bug 23950
        // PTV OCC725 17.09.2002 -- begin --
        int nbFreeVrtx = 0;
        TopoDS_Compound aCompOfVrtx = new TopoDS_Compound();
        BRep_Builder aB = new BRep_Builder();
        aB.MakeCompound(ref aCompOfVrtx);

        for (TopoDS_Iterator iter = new TopoDS_Iterator(theShape); iter.More(); iter.Next())
        {
            TopoDS_Shape aSubShape = iter.Value();
            if (aSubShape.ShapeType() != TopAbs_VERTEX || !isSeparateVertices)
            {
                // Store non-manifold topology as shells (ssv; 10.11.2010)
                if (!isManifold && aSubShape.ShapeType() == TopAbs_SOLID)
                {
                    for (TopoDS_Iterator aSubIter = new TopoDS_Iterator(aSubShape); aSubIter.More(); aSubIter.Next())
                    {
                        TopoDS_Shell aSubShell = ClosedShell(TopoDS.Shell(aSubIter.Value())); // OCCT: aSubShell.Closed(true)
                        RepItemSeq.Append(aSubShell);
                        NonManifoldGroup.Append(aSubShell);
                    }
                }
                else if (!isManifold && (aSubShape.ShapeType() == TopAbs_SHELL || aSubShape.ShapeType() == TopAbs_FACE))
                {
                    RepItemSeq.Append(aSubShape);
                    NonManifoldGroup.Append(aSubShape);
                }
                else
                {
                    RepItemSeq.Append(aSubShape);
                }

                continue;
            }
            aB.Add(aCompOfVrtx, iter.Value());
            nbFreeVrtx++;
        }
        if (nbFreeVrtx != 0)
        {
            RepItemSeq.Append(aCompOfVrtx);
        }

        // PTV OCC725 17.09.2002 -- end --

        // Constitution : liste d axes, le premier est l origine, les suivants : 1
        // par sous-item
        StepShape_ShapeRepresentation shapeRep = SDR0.UsedRepresentation() as StepShape_ShapeRepresentation;
        if (shapeRep is null)
        {
            shapeRep = new StepShape_ShapeRepresentation();
            SDR0.SetUsedRepresentation(shapeRep); // to be used by MakeItem
        }
        binder = TransientResult(SDR0); // set SDR as first item in order to be found first (but not SDR of subshape!)
        binder.AddResult(TransientResult(shapeRep));

        // translate components
        int i, nbs = RepItemSeq.Length();
        NCollection_HSequence<Standard_Transient> ItemSeq = new NCollection_HSequence<Standard_Transient>();
        ItemSeq.Append(myContext.GetDefaultAxis());
        myContext.NextLevel();
        using Message_ProgressScope aPS = new Message_ProgressScope(theProgress, null, nbs);
        for (i = 1; i <= nbs && aPS.More(); i++)
        {
            TransferBRep_ShapeMapper subs = TransferBRep.ShapeMapper(FP, RepItemSeq.Value(i));
            StepGeom_GeometricRepresentationItem AX1 = null;

            Transfer_Binder bnd = TransferSubShape(subs, SDR0, ref AX1, FP, theLocalFactors, NonManifoldGroup, isManifold, aPS.Next());

            if (AX1 is not null)
            {
                ItemSeq.Append(AX1);
            }
            // copy binders so as to have all roots in upper binder, but do not conflict
            while (bnd is not null)
            {
                Transfer_SimpleBinderOfTransient bx = bnd as Transfer_SimpleBinderOfTransient;
                if (bx is not null)
                {
                    binder.AddResult(TransientResult(bx.Result()));
                }
                bnd = bnd.NextResult();
            }
        }
        myContext.PrevLevel();

        int nsub = ItemSeq.Length();
        NCollection_HArray1<StepRepr_RepresentationItem> items = new NCollection_HArray1<StepRepr_RepresentationItem>(1, nsub);

        // initialize representation
        for (int rep = 1; rep <= nsub; rep++)
        {
            items.SetValue(rep, ItemSeq.Value(rep) as StepRepr_RepresentationItem);
        }
        shapeRep.SetItems(items);
        double Tol = UsedTolerance(aStepModel, mytoler, theShape);
        STEPConstruct_UnitContext mk = new STEPConstruct_UnitContext();
        mk.Init(Tol, aStepModel, theLocalFactors);
        shapeRep.SetContextOfItems(mk.Value()); // la tolerance, voir au debut
        shapeRep.SetName(new TCollection_HAsciiString(""));

        // set it to SDR
        //  SDR0->SetUsedRepresentation (shapeRep);

        return binder;
    }

    public Transfer_Binder TransferSubShape(Transfer_Finder start, StepShape_ShapeDefinitionRepresentation SDR0,
                                            ref StepGeom_GeometricRepresentationItem AX1, Transfer_FinderProcess FP,
                                            in StepData_Factors theLocalFactors, NCollection_HSequence<TopoDS_Shape> shapeGroup = null,
                                            bool isManifold = true, Message_ProgressRange theProgress = default)
    {
        TransferBRep_ShapeMapper mapper = start as TransferBRep_ShapeMapper;
        if (mapper is null)
        {
            return NullResult();
        }
        TopoDS_Shape shape = mapper.Value();

        //   SHAPE EN POSITION VENANT D UN ASSEMBLAGE
        //   Il faut alors distinguer la transformation de la shape meme
        //   laquelle est consideree a l origine, puis transferee
        //   A part, un item decrivant une occurence en position est cree
        //   SINON, la shape est prise et transferee telle quelle
        TopoDS_Shape sh0 = shape;
        gp_Trsf aLoc = new gp_Trsf();
        bool isShapeLocated = false;
        if (GroupMode() > 0)
        {
            TopLoc_Location shloc = shape.Location();
            isShapeLocated = !shloc.IsIdentity();
            aLoc = shloc.Transformation();
            TopLoc_Location shident = new TopLoc_Location();
            sh0.Location(shident);
            mapper = TransferBRep.ShapeMapper(FP, sh0);
            mapper.SameAttributes(start);
        }

        Transfer_Binder resbind = FP.Find(mapper);
        StepShape_ShapeDefinitionRepresentation sdr = null;
        STEPConstruct_Part SDRTool = new STEPConstruct_Part();
        StepData_StepModel aStepModel = FP.Model() as StepData_StepModel;

        // Already SDR and SR available : take them as are
        bool iasdr = FP.GetTypedTransient(resbind, typeof(StepShape_ShapeDefinitionRepresentation), ref sdr);
        if (iasdr)
        {
            SDRTool.ReadSDR(sdr);
        }
        else
        {
            SDRTool.MakeSDR(null, myContext.GetProductName(), myContext.GetAPD().Application(), aStepModel);
            sdr = SDRTool.SDRValue();
        }

        // if shape itself not yet translated, do it now
        //: abv 20.05.02: see comment in TransferShape(): added "! iasdr ||"
        Transfer_Binder resprod = TransientResult(sdr); // KA - OCC7141(skl 10.11.2004)
        bool isJustTransferred = false;
        if (!iasdr || resbind is null)
        {
            Transfer_Binder resbind1 = TransferShape(mapper, sdr, FP, theLocalFactors, shapeGroup, isManifold, theProgress);
            if (resbind1 is null || sdr.UsedRepresentation() is null)
            {
                return null;
            }
            resbind = resbind1;
            Transfer_Binder oldbind = FP.Find(mapper);
            if (oldbind is not null && resbind is not null)
            {
                resbind.AddResult(oldbind);
            }
            isJustTransferred = true;
        }

        // create assembly structures (CDSR, NAUO etc.)
        STEPConstruct_Assembly mkitem = new STEPConstruct_Assembly();

        // make location for assembly placement
        if (Math.Abs(aLoc.ScaleFactor() - 1.0) > Precision.Confusion())
        {
            if (aStepModel.InternalParameters.WriteScalingTrsf)
            {
                FP.AddWarning(start, "The shape has a scaling factor, exported as cartesian_transformation_operator");
            }
            else
            {
                FP.AddWarning(start, "The shape has a scaling factor, skipped");
            }
        }
        if (Math.Abs(aLoc.ScaleFactor() - 1.0) < Precision.Confusion() || !aStepModel.InternalParameters.WriteScalingTrsf)
        {
            // create a new axis2placement3d
            GeomToStep_MakeAxis2Placement3d mkax = new GeomToStep_MakeAxis2Placement3d(aLoc, theLocalFactors);
            mkitem.Init(sdr, SDR0, myContext.GetDefaultAxis(), mkax.Value());
            AX1 = mkax.Value();
        }
        else
        {
            // create a new cartesian transformation
            GeomToStep_MakeCartesianTransformationOperator aMaker = new GeomToStep_MakeCartesianTransformationOperator(aLoc, theLocalFactors);
            if (!aMaker.IsDone())
            {
                FP.AddFail(start, "The transformation relation creation failed");
            }
            mkitem.Init(sdr, SDR0, aMaker.Value());
            AX1 = aMaker.Value();
        }

        // OCCT numbers the occurrence from a process-wide counter: here the writer's (decision 1266)
        mkitem.MakeRelationship(ref myContext.NauoId());
        NCollection_HSequence<Standard_Transient> roots = myContext.GetRootsForAssemblyLink(mkitem);

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
            roots.Append(myContext.GetRootsForPart(SDRTool));
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
            FP.Bind(mapper, resprod);
        }

        myContext.NextIndex();

        // abv 16.10.00: bind CDSR (et al) to located shape in order to be able to track instances
        if (mapper != start && isShapeLocated)
        {
            Transfer_Binder bnd = FP.Find(start);
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
            FP.Bind(start, bnd);
        }

        return resprod;
    }

    private int mygroup;
    private double mytoler;
    private STEPConstruct_ContextTool myContext = new STEPConstruct_ContextTool();
}
