// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorRead.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorRead.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Occt.BRepCheck_Status;
using static Kerf.Geometry.TransformForm;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// This class performs the transfer of an Entity from AP214 and AP203, either Geometric or Topologic. I.E. for each
/// type of Entity, it invokes the appropriate Tool then returns the Binder which contains the Result
/// </summary>
/// <remarks>
/// <para>
/// OCCT.NET (decisions 1247, 1249, 1265, 1268):
/// </para>
/// <list type="bullet">
/// <item>the shapes are made unfrozen (StepToTopoDS's <c>InternalValue</c>), bound and reused through the transient
/// process, and frozen when the reader publishes them (XSControl_Reader, decision 1224);</item>
/// <item>OCCT's file-level <c>NM_DETECTED</c> (a process-wide flag telling the callers that the representation just
/// translated was non-manifold) is the actor's own;</item>
/// <item>a model whose length unit isn't initialised is read in millimetres (OCCT: the process-wide unit, from the
/// static <c>xstep.cascade.unit</c>); STEPControl_Reader initialises it from its session's statics;</item>
/// <item>OCCT recurses along a file's assembly links (NAUO to product definition and back) without a guard, so a cycle
/// overflows the stack; the port checks the stack (<c>EnsureSufficientExecutionStack</c>) at each shape transfer,
/// which raises an exception the transient process records as a fail (occt-issues.md, 8F).</item>
/// </list>
/// </remarks>
public class STEPControl_ActorRead : Transfer_ActorOfTransientProcess, StepToTopoDS_IActorRead
{
    // Set to inform outer methods that current representation item is non-manifold (OCCT: a file-level global).
    private bool NM_DETECTED;

    public STEPControl_ActorRead(Interface_InterfaceModel theModel)
    {
        myPrecision = 0.0;
        myMaxTol = 0.0;
        myModel = theModel;
    }

    public override bool Recognize(Transient start)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(StepBasic_ProductDefinition)))
        {
            return true;
        }

        if (start.IsKind(typeof(StepRepr_NextAssemblyUsageOccurrence)))
        {
            return true;
        }

        StepData_StepModel aStepModel = myModel as StepData_StepModel;
        if (aStepModel is null)
        {
            return false;
        }
        bool aProdMode = aStepModel.InternalParameters.ReadProductMode;
        if (!aProdMode)
        {
            if (start.IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
            {
                return true;
            }
        }

        StepShape_ShapeRepresentation sr = start as StepShape_ShapeRepresentation;
        if (sr is not null)
        {
            int i, nb = sr.NbItems();
            for (i = 1; i <= nb; i++)
            {
                if (Recognize(sr.ItemsValue(i)))
                {
                    return true;
                }
            }
            return false;
        }

        bool aCanReadTessGeom = aStepModel.InternalParameters.ReadTessellated != 0;

        if (start.IsKind(typeof(StepShape_FacetedBrep)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_BrepWithVoids)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_ManifoldSolidBrep)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_ShellBasedSurfaceModel)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_FacetedBrepAndBrepWithVoids)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_GeometricSet)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepRepr_MappedItem)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_FaceSurface)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_EdgeBasedWireframeModel)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShape_FaceBasedSurfaceModel)))
        {
            return true;
        }
        if (aCanReadTessGeom && start.IsKind(typeof(StepVisual_TessellatedFace)))
        {
            return true;
        }
        if (aCanReadTessGeom && start.IsKind(typeof(StepVisual_TessellatedSurfaceSet)))
        {
            return true;
        }
        if (aCanReadTessGeom && start.IsKind(typeof(StepVisual_TessellatedShell)))
        {
            return true;
        }
        if (aCanReadTessGeom && start.IsKind(typeof(StepVisual_TessellatedSolid)))
        {
            return true;
        }
        if (aCanReadTessGeom && start.IsKind(typeof(StepVisual_TessellatedShapeRepresentation)))
        {
            return true;
        }

        //  REPRESENTATION_RELATIONSHIP et consorts : on regarde le contenu ...
        //  On prend WithTransformation ou non ...

        if (start.IsKind(typeof(StepRepr_ShapeRepresentationRelationship)))
        {
            StepRepr_ShapeRepresentationRelationship und = start as StepRepr_ShapeRepresentationRelationship;

            //  On prend son contenu

            return Recognize(und.Rep1()) || Recognize(und.Rep2());
        }

        if (start.IsKind(typeof(StepShape_ContextDependentShapeRepresentation)))
        {
            return true;
            //  on fait le pari que, si ce n est pas transferable tel quel,
            //  des CDSR implicitement references le sont ...
            //  Sinon cette entite n aurait pas grand sens ...
        }

        return false;
    }

    /// <summary>recursive method that accesses the root entities and starts the mapping</summary>
    public override Transfer_Binder Transfer(Transient start, Transfer_TransientProcess TP,
                                             ProgressRange theProgress = default)
    {
        // [BEGIN] Get version of preprocessor (to detect I-Deas case) (ssv; 23.11.2010)
        StepData_Factors aLocalFactors = new StepData_Factors();
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        if (!aStepModel.IsInitializedUnit())
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; the port has no
            // process-wide unit: millimetres (decision 1265)
            double aCascadeUnit = UnitsMethods.GetCasCadeLengthUnit(null);
            aStepModel.SetLocalLengthUnit(aCascadeUnit);
            aLocalFactors.SetCascadeUnit(aCascadeUnit);
        }
        aLocalFactors.SetCascadeUnit(aStepModel.LocalLengthUnit());
        Interface_EntityIterator anEntIt = aStepModel.Header();
        for (anEntIt.Start(); anEntIt.More(); anEntIt.Next())
        {
            HeaderSection_FileName aFileNameEntity = anEntIt.Value() as HeaderSection_FileName;
            if (aFileNameEntity is not null)
            {
                TCollection_HAsciiString aPPVersion = aFileNameEntity.PreprocessorVersion();
                if (aPPVersion is null)
                {
                    continue;
                }
                int anIDeasResult = aPPVersion.Search("I-DEAS");
                if (anIDeasResult != -1)
                {
                    myNMTool.SetIDEASCase(true);
                }
            }
        }
        // [END] Get version of preprocessor (to detect I-Deas case) (ssv; 23.11.2010)
        bool aTrsfUse = aStepModel.InternalParameters.ReadRootTransformation;
        return TransferShape(start, TP, aLocalFactors, true, aTrsfUse, theProgress);
    }

    // ============================================================================
    // auxiliary function : ApplyTransformation
    // ============================================================================
    private static void ApplyTransformation(ref TopoDS_Shape shape, in Transform3D Trsf)
    {
        if (Trsf.Form() == Identity)
        {
            return;
        }
        Placement theLoc = new Placement(Trsf);
        shape.Move(theLoc);
    }

    private static void ApplyTransformation(ref TopoDS_Compound shape, in Transform3D Trsf)
    {
        if (Trsf.Form() == Identity)
        {
            return;
        }
        Placement theLoc = new Placement(Trsf);
        shape.Move(theLoc);
    }

    // ============================================================================
    // auxiliary function : FindContext
    // ============================================================================
    private static StepRepr_Representation FindContext(Transient start, Transfer_TransientProcess TP, int level = 10)
    {
        StepRepr_Representation rep = null;
        Interface_Graph graph = TP.Graph();
        Interface_EntityIterator subs = graph.Sharings(start);
        for (subs.Start(); subs.More() && rep is null; subs.Next())
        {
            rep = subs.Value() as StepRepr_Representation;
            if (rep is null && level > 0)
            {
                rep = FindContext(subs.Value(), TP, level - 1);
            }
        }
        return rep;
    }

    //=======================================================================
    // function : FindShapeReprType
    // purpose  : Returns integer corresponding to the type of the representation
    //           as defined in read.step.shape.repr_level parameter
    //=======================================================================
    private static int FindShapeReprType(Transient start)
    {
        if (start.IsKind(typeof(StepShape_AdvancedBrepShapeRepresentation)))
        {
            return 2;
        }
        if (start.IsKind(typeof(StepShape_ManifoldSurfaceShapeRepresentation)))
        {
            return 3;
        }
        if (start.IsKind(typeof(StepShape_GeometricallyBoundedSurfaceShapeRepresentation)))
        {
            return 4;
        }
        if (start.IsKind(typeof(StepShape_FacetedBrepShapeRepresentation)))
        {
            return 5;
        }
        if (start.IsKind(typeof(StepShape_EdgeBasedWireframeShapeRepresentation)))
        {
            return 6;
        }
        if (start.IsKind(typeof(StepShape_GeometricallyBoundedWireframeShapeRepresentation)))
        {
            return 7;
        }
        return 8;
    }

    //=======================================================================
    // function : getListSDR
    // purpose  : Get SDRs assigned to ShapeAspect, which may potentially
    //           correspond to hybrid models in AP203 before 1998
    //=======================================================================
    private static void getListSDR(StepRepr_ShapeAspect sa, NCollection_HSequence<Transient> listSDR, Transfer_TransientProcess TP)
    {
        Interface_Graph graph = TP.Graph();

        // check whether this ShapeAspect is used in G&DT, and if yes, ignore it
        if (sa.IsKind(typeof(StepDimTol_DatumFeature)))
        {
            return;
        }
        Interface_EntityIterator subs5 = graph.Sharings(sa);
        for (subs5.Start(); subs5.More(); subs5.Next())
        {
            if (subs5.Value().IsKind(typeof(StepDimTol_GeometricTolerance)))
            {
                return;
            }
        }

        // reiterate by referred entities and take all SDRs; note that SDRs that
        // refer to sub-shapes of main SDR will be filtered out further during translation
        subs5 = graph.Sharings(sa);
        for (subs5.Start(); subs5.More(); subs5.Next())
        {
            StepRepr_PropertyDefinition propd = subs5.Value() as StepRepr_PropertyDefinition;
            if (propd is null)
            {
                continue;
            }
            Interface_EntityIterator subs6 = graph.Sharings(propd);
            for (subs6.Start(); subs6.More(); subs6.Next())
            {
                StepShape_ShapeDefinitionRepresentation sdr = subs6.Value() as StepShape_ShapeDefinitionRepresentation;
                if (sdr is not null)
                {
                    listSDR.Append(sdr);
                }
            }
        }
    }

    //=======================================================================
    // function : getSDR
    // purpose  : Find all SDRs related to given PDS
    //=======================================================================
    private static void getSDR(StepRepr_ProductDefinitionShape PDS, NCollection_HSequence<Transient> listSDR,
                               NCollection_HSequence<Transient> listNAUO, NCollection_HSequence<Transient> listSDRAspect,
                               Transfer_TransientProcess TP)
    {
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;

        // Flag indicating preferred shape representation type, to be chosen if
        // several different representations are attached to the same shape
        int delta = 100;
        int ICS = (int)aStepModel.InternalParameters.ReadShapeRepr;
        int nbSDR0 = listSDR.Length();

        // Iterate by entities referring PDS
        Interface_Graph graph = TP.Graph();
        Interface_EntityIterator subs4 = graph.Sharings(PDS);
        for (subs4.Start(); subs4.More(); subs4.Next())
        {
            // Directly assigned SDR
            StepShape_ShapeDefinitionRepresentation sdr = subs4.Value() as StepShape_ShapeDefinitionRepresentation;
            if (sdr is not null)
            {
                StepRepr_Representation rep = sdr.UsedRepresentation();

                if (rep is not null)
                {
                    // if specific representation type is preferred, choose the
                    // representations of the closest type
                    if (ICS == 1)
                    { // translate all SDRs
                        listSDR.Append(sdr);
                    }
                    else
                    {
                        int iDiff = Math.Abs(FindShapeReprType(rep) - ICS);
                        // if more suitable representation is found, drop previous if any selected
                        if (iDiff < delta)
                        {
                            while (listSDR.Length() > nbSDR0)
                            {
                                listSDR.Remove(listSDR.Length());
                            }
                            delta = iDiff;
                        }
                        // take all representations of preferred or closest type
                        if (iDiff <= delta)
                        {
                            listSDR.Append(sdr);
                        }
                    }
                }
                continue;
            }

            // SDRs assigned via ShapeAspect; may correspond to hybrid model in AP203 before 1998
            StepRepr_ShapeAspect sa = subs4.Value() as StepRepr_ShapeAspect;
            if (sa is not null)
            {
                getListSDR(sa, listSDRAspect, TP);
                continue;
            }

            // NAUO is used to find sub-assemblies
            StepRepr_NextAssemblyUsageOccurrence NAUO = subs4.Value() as StepRepr_NextAssemblyUsageOccurrence;
            if (NAUO is not null)
            {
                if (PDS.Definition().ProductDefinition() == NAUO.RelatingProductDefinition())
                {
                    listNAUO.Append(NAUO);
                }
                continue;
            }
        }
    }

    /// <summary>Transfers product definition entity</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepBasic_ProductDefinition PD, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, bool theUseTrsf = false,
                                                      ProgressRange theProgress = default)
    {
        TransferBRep_ShapeBinder shbinder = null;

        TopoDS_Compound Cund = new TopoDS_Compound();
        TopoDS_Shape Result1 = new TopoDS_Shape();
        BRep_Builder B = new BRep_Builder();
        B.MakeCompound(ref Cund);

        // Find subcomponents of assembly (NAUO)
        // and definitions of shape of the current product (SDR)
        NCollection_HSequence<Transient> listSDR = new NCollection_HSequence<Transient>();
        NCollection_HSequence<Transient> listNAUO = new NCollection_HSequence<Transient>();
        NCollection_HSequence<Transient> listSDRAspect = new NCollection_HSequence<Transient>();
        Interface_Graph graph = TP.Graph();
        Interface_EntityIterator subs3 = graph.Sharings(PD);
        for (subs3.Start(); subs3.More(); subs3.Next())
        {
            // PDS is used to find shape definitions attached to this product
            StepRepr_ProductDefinitionShape PDS = subs3.Value() as StepRepr_ProductDefinitionShape;
            if (PDS is not null)
            {
                getSDR(PDS, listSDR, listNAUO, listSDRAspect, TP);
                continue;
            }
            // NAUO is used to find sub-assemblies
            StepRepr_NextAssemblyUsageOccurrence NAUO1 = subs3.Value() as StepRepr_NextAssemblyUsageOccurrence;
            if (NAUO1 is not null)
            {
                if (PD == NAUO1.RelatingProductDefinition())
                {
                    listNAUO.Append(NAUO1);
                }
                continue;
            }
        }

        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;

        // Flag indicating whether SDRs associated with the product`s main SDR
        // by SRRs (which correspond to hybrid model representation in AP203 since 1998)
        // should be taken into account
        bool readSRR = aStepModel.InternalParameters.ReadRelationship;

        bool readConstructiveGeomRR = aStepModel.InternalParameters.ReadConstrRelation;
        // Flag indicating whether SDRs associated with the product`s main SDR
        // by SAs (which correspond to hybrid model representation in AP203 before 1998)
        // should be taken into account
        int readSA = aStepModel.InternalParameters.ReadShapeAspect ? 1 : 0;
        if (readSA == 0)
        {
            listSDRAspect.Clear();
        }

        // remember number of normal SDRs (not those found via ShapeAspect)
        // and merge both lists in one
        int nbNotAspect = listSDR.Length();
        listSDR.Append(listSDRAspect);

        // Flag indicating level of reading assemblies: only structure (3),
        // structure with shapes of final parts (2), or everything, including shapes
        // possibly attached directly to intermediate assemblies (1)
        // Special mode (4) is used to translate shape attached to this product only,
        // ignoring sub-assemblies if any
        int readAssembly = (int)aStepModel.InternalParameters.ReadAssemblyLevel;
        if (readAssembly == 3 || (readAssembly == 2 && listNAUO.Length() > 0))
        {
            listSDR.Clear();
        }
        else if (readAssembly == 4)
        {
            listNAUO.Clear();
        }

        int nbEnt = listSDR.Length() + listNAUO.Length();
        if (nbEnt <= 0)
        {
            return shbinder;
        }

        // common progress indicator for translation of own shapes and sub-assemblies
        using ProgressScope PS = new ProgressScope(theProgress, "Part", nbEnt);
        int nbComponents = 0;

        // translate sub-assemblies
        for (int nbNauo = 1; nbNauo <= listNAUO.Length() && PS.More(); nbNauo++)
        {
            StepRepr_NextAssemblyUsageOccurrence NAUO = listNAUO.Value(nbNauo) as StepRepr_NextAssemblyUsageOccurrence;

            // TRANSLOG
            if (TP.TraceLevel() > 1)
            {
                using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
                sout.Append(" -- Actor : Ent.n0 ").Append(TP.Model().Number(PD)).Append(" -> Shared Ent.no")
                    .Append(TP.Model().Number(NAUO)).Append('\n');
            }
            Transfer_Binder binder;
            ProgressRange aRange = PS.Next();
            if (!TP.IsBound(NAUO))
            {
                binder = TransferEntity(NAUO, TP, theLocalFactors, aRange);
            }
            else
            {
                binder = TP.Find(NAUO);
            }

            TopoDS_Shape theResult = TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull())
            {
                Result1 = theResult;
                // [BEGIN] ssv: OCCT#22436: extra compound in NMSSR case
                if (NM_DETECTED && Result1.ShapeType() == TopAbs_COMPOUND)
                {
                    TopoDS_Iterator it = new TopoDS_Iterator(Result1);
                    for (; it.More(); it.Next())
                    {
                        TopoDS_Shape aSubShape = it.Value();
                        B.Add(Cund, aSubShape);
                    }
                }
                else
                {
                    B.Add(Cund, theResult);
                }
                // [END] ssv: OCCT#22436: extra compound in NMSSR case
                nbComponents++;
            }
        }

        // translate shapes assigned directly
        for (int i = 1; i <= listSDR.Length() && PS.More(); i++)
        {
            StepShape_ShapeDefinitionRepresentation sdr = listSDR.Value(i) as StepShape_ShapeDefinitionRepresentation;
            StepShape_ShapeRepresentation rep = sdr.UsedRepresentation() as StepShape_ShapeRepresentation;
            if (rep is null)
            {
                continue;
            }

            using ProgressScope aPS1 = new ProgressScope(PS.Next(), null, 2);

            // translate SDR representation
            bool isBound = true;
            // SKL for bug 29068: transformation need to applied only for "main"
            // ShapeDefinitionRepresentation. Part of listSDR given by ShapeAspect must be ignored because
            // all needed transformations will be applied during its transfer. Therefore flag for using Trsf
            // must be updated.
            bool useTrsf = theUseTrsf && (i <= nbNotAspect);
            Transfer_Binder binder = TP.Find(rep);
            if (binder is null)
            {
                binder = TransferEntity(rep, TP, theLocalFactors, ref isBound, useTrsf, aPS1.Next());
            }

            // if SDR is obtained from ShapeAspect and representation items have already been tramnslated,
            // this means that that ShapeAspect is used to refer to sub-shape of the main shape
            // (e.g. for validation properties), and we do not need to translate it actually;
            // otherwise it is considered as part of hybrid representation in AP203 before 1998
            if (i > nbNotAspect && isBound)
            {
                continue;
            }

            // record shape resulting from translation
            TopoDS_Shape theResult = new TopoDS_Shape();
            if (binder is not null)
            {
                theResult = TransferBRep.ShapeResult(binder);
                if (!theResult.IsNull())
                {
                    Result1 = theResult;
                    // [BEGIN] ssv: OCCT#22436: extra compound in NMSSR case
                    if (NM_DETECTED && Result1.ShapeType() == TopAbs_COMPOUND)
                    {
                        TopoDS_Iterator it = new TopoDS_Iterator(Result1);
                        for (; it.More(); it.Next())
                        {
                            TopoDS_Shape aSubShape = it.Value();
                            B.Add(Cund, aSubShape);
                        }
                    }
                    else
                    {
                        B.Add(Cund, theResult);
                    }
                    // [END] ssv: OCCT#22436: extra compound in NMSSR case
                }
            }

            // translate possible shapes related by SRRs, which corresponds to
            // way of writing hybrid models in AP203 since 1998, and AP209
            // Note that both AP203 and AP209 allow main representation to be non-empty
            if (readSRR && /*theResult.IsNull() &&*/ i <= nbNotAspect)
            {
                TopoDS_Shape aNewResult = TransferRelatedSRR(TP, rep, useTrsf, readConstructiveGeomRR, theLocalFactors, Cund, aPS1);
                if (!aNewResult.IsNull())
                {
                    Result1 = aNewResult;
                }
            }
        }

        // make a warning if both own shape and sub-assemblies are present
        if ((Cund.NbChildren() - nbComponents) > 0 && nbComponents > 0)
        {
            TP.AddWarning(PD, "Product has both sub-assemblies and directly assigned shape");
        }

        // if only single shape is read, add it as it is; otherwise add compound
        if ((Cund.NbChildren() - nbComponents) == 1 && nbComponents == 0)
        {
            shbinder = new TransferBRep_ShapeBinder(Result1);
        }
        else
        {
            shbinder = new TransferBRep_ShapeBinder(Cund);
        }

        // TP->Unbind( PD ); //:j3: unbind start (let it be bound by TransferProcess)
        TP.Bind(PD, shbinder);
        return shbinder;
    }

    /// <summary>Transfers next assembly usage occurrence entity</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepRepr_NextAssemblyUsageOccurrence NAUO, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, ProgressRange theProgress = default)
    {
        TransferBRep_ShapeBinder shbinder = null;
        StepBasic_ProductDefinition PD = null;
        Interface_Graph graph = TP.Graph();
        Transform3D Trsf = new Transform3D();
        bool iatrsf = false, SRRReversed = false, IsDepend = false;
        StepRepr_ShapeRepresentationRelationship SRR = null;
        Interface_EntityIterator subs1 = graph.Sharings(NAUO);
        for (subs1.Start(); subs1.More(); subs1.Next())
        {
            StepRepr_ProductDefinitionShape PDS = subs1.Value() as StepRepr_ProductDefinitionShape;
            if (PDS is null)
            {
                continue;
            }
            Interface_EntityIterator subs2 = graph.Sharings(PDS);
            for (subs2.Start(); subs2.More(); subs2.Next())
            {
                StepShape_ContextDependentShapeRepresentation CDSR = subs2.Value() as StepShape_ContextDependentShapeRepresentation;
                if (CDSR is null)
                {
                    continue;
                }
                IsDepend = true;
                StepRepr_RepresentationRelationship RR = CDSR.RepresentationRelation();
                if (RR is null)
                {
                    continue;
                }
                SRRReversed = STEPConstruct_Assembly.CheckSRRReversesNAUO(graph, CDSR);
                StepRepr_Representation rep = (SRRReversed ? RR.Rep2() : RR.Rep1());
                if (rep is null)
                {
                    continue;
                }
                iatrsf = ComputeSRRWT(RR, TP, ref Trsf, theLocalFactors);
                // find real ProductDefinition used rep
                Interface_EntityIterator subs3 = TP.Graph().Sharings(rep);
                for (subs3.Start(); subs3.More(); subs3.Next())
                {
                    Transient aSubs3Val = subs3.Value();
                    if (aSubs3Val is StepShape_ShapeDefinitionRepresentation SDR)
                    {
                        StepRepr_ProductDefinitionShape PDS1 = SDR.Definition().PropertyDefinition() as StepRepr_ProductDefinitionShape;
                        if (PDS1 is null)
                        {
                            continue;
                        }
                        Interface_EntityIterator subs4 = graph.Shareds(PDS1);
                        for (subs4.Start(); PD is null && subs4.More(); subs4.Next())
                        {
                            PD = subs4.Value() as StepBasic_ProductDefinition;
                        }
                    }
                    else if (aSubs3Val.IsKind(typeof(StepRepr_ShapeRepresentationRelationship)))
                    {
                        // NB: C cast is used instead of DownCast() to improve performance on some cases.
                        SRR = (StepRepr_ShapeRepresentationRelationship)aSubs3Val;
                    }
                }
            }
        }

        Transfer_Binder binder;
        TopoDS_Shape theResult = new TopoDS_Shape();
        shbinder = null;

        if (IsDepend)
        {
            using ProgressScope aPS = new ProgressScope(theProgress, null, 2);

            if (PD is not null)
            {
                binder = TP.Find(PD);
                if (binder is null)
                {
                    binder = TransferEntity(PD, TP, theLocalFactors, false, aPS.Next());
                }
                theResult = TransferBRep.ShapeResult(binder);
                if (!theResult.IsNull())
                {
                    if (iatrsf)
                    {
                        if (SRRReversed)
                        {
                            ApplyTransformation(ref theResult, Trsf.Inverted());
                        }
                        else
                        {
                            ApplyTransformation(ref theResult, Trsf);
                        }
                    }
                    shbinder = new TransferBRep_ShapeBinder(theResult);
                }
            }

            if (theResult.IsNull() && SRR is not null)
            {
                binder = TP.Find(SRR);
                if (binder is null)
                {
                    binder = TransferEntity(SRR, TP, theLocalFactors, 0, false, aPS.Next());
                    theResult = TransferBRep.ShapeResult(binder);
                    if (!theResult.IsNull())
                    {
                        shbinder = new TransferBRep_ShapeBinder(theResult);
                    }
                }
            }
        }
        TP.Bind(NAUO, shbinder);
        return shbinder;
    }

    /// <summary>Transfers shape representation entity</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepShape_ShapeRepresentation sr, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, ref bool isBound, bool theUseTrsf = false,
                                                      ProgressRange theProgress = default)
    {
        NM_DETECTED = false;
        TransferBRep_ShapeBinder shbinder = null;
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        if (!Recognize(sr))
        {
            return shbinder;
        }
        isBound = false;
        int nb = sr.NbItems();
        int nbTPitems = TP.NbMapped();
        // TRANSLOG
        if (TP.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
            sout.Append(" -- Actor : case  ShapeRepr. NbItems=").Append(nb).Append('\n');
        }

        // Compute unit conversion factors and geometric Accuracy
        StepRepr_Representation oldSRContext = mySRContext; //: S4136
        StepData_Factors aLocalFactors = theLocalFactors;
        PrepareUnits(sr, TP, ref aLocalFactors);

        BRep_Builder B = new BRep_Builder();
        TopoDS_Compound comp = new TopoDS_Compound();
        B.MakeCompound(ref comp);
        TopoDS_Shape OneResult = new TopoDS_Shape();
        int nsh = 0;

        // [BEGIN] Proceed with non-manifold topology (ssv; 12.11.2010)
        bool isNMMode = aStepModel.InternalParameters.ReadNonmanifold;
        bool isManifold = true;
        if (isNMMode && sr.IsKind(typeof(StepShape_NonManifoldSurfaceShapeRepresentation)))
        {
            isManifold = false;
            NM_DETECTED = true;
        }
        // Special processing for I-DEAS STP case (ssv; 15.11.2010)
        else
        {
            bool isIDeasMode = aStepModel.InternalParameters.ReadIdeas;
            if (isNMMode && myNMTool.IsIDEASCase() && isIDeasMode)
            {
                isManifold = false;
                NM_DETECTED = true;
            }
        }
        myNMTool.CleanUp();
        // Don't use NM tool in manifold cases (ssv; 24.11.2010)
        myNMTool.SetActive(!isManifold && isNMMode);
        // [END] Proceed with non-manifold topology (ssv; 12.11.2010)

        Transform3D aTrsf = new Transform3D();
        using ProgressScope aPSRoot = new ProgressScope(theProgress, "Sub-assembly", isManifold ? 1 : 2);
        using ProgressScope aPS = new ProgressScope(aPSRoot.Next(), "Transfer", nb);
        NCollection_IndexedMap<TopoDS_Shape> aCompoundedShapes = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (int i = 1; i <= nb && aPS.More(); i++)
        {
            ProgressRange aRange = aPS.Next();
            // TRANSLOG
            if (TP.TraceLevel() > 2)
            {
                using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
                sout.Append(" -- Actor, shape_representation.item n0. ").Append(i).Append('\n');
            }
            StepRepr_RepresentationItem anitem = sr.ItemsValue(i);
            if (anitem is null)
            {
                continue;
            }
            if (theUseTrsf)
            {
                if (anitem.IsKind(typeof(StepGeom_Axis2Placement3d)))
                {
                    Interface_Graph graph = TP.Graph();
                    Interface_EntityIterator subs3 = graph.Sharings(anitem);
                    for (subs3.Start(); subs3.More(); subs3.Next())
                    {
                        StepRepr_ItemDefinedTransformation IDT = subs3.Value() as StepRepr_ItemDefinedTransformation;
                        if (IDT is not null)
                        {
                            // current Axis2Placement is used for related ShapeRepresentation => ignore it
                            break;
                        }
                    }
                    if (!subs3.More())
                    {
                        StepGeom_Axis2Placement3d aCS = anitem as StepGeom_Axis2Placement3d;
                        Geom_Axis2Placement aTargAP = StepToGeom.MakeAxis2Placement(aCS, aLocalFactors);
                        if (aTargAP is not null)
                        {
                            CoordinateSystem ax3Orig = new CoordinateSystem(new Point3D(0.0, 0.0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
                            CoordinateSystem ax3Targ = new CoordinateSystem(aTargAP.Ax2());
                            if (ax3Targ.Location().SquareDistance(ax3Orig.Location()) < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                                && ax3Targ.Direction().IsEqual(ax3Orig.Direction(), global::Kerf.Engine.Foundation.Precision.Angular())
                                && ax3Targ.XDirection().IsEqual(ax3Orig.XDirection(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                continue;
                            }
                            aTrsf.SetTransformation(ax3Targ, ax3Orig);
                        }
                        continue;
                    }
                }
            }
            Transfer_Binder binder;
            if (!TP.IsBound(anitem))
            {
                binder = TransferShape(anitem, TP, aLocalFactors, isManifold, false, aRange);
            }
            else
            {
                isBound = true;
                binder = TP.Find(anitem);
            }
            TopoDS_Shape theResult = TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull())
            {
                OneResult = theResult;
                if (!aCompoundedShapes.Contains(theResult))
                {
                    aCompoundedShapes.Add(theResult);
                    TopExp.MapShapes(theResult, aCompoundedShapes, false, false);
                    B.Add(comp, theResult);
                    nsh++;
                }
            }
        }

        // [BEGIN] Proceed with non-manifold topology (ssv; 12.11.2010)
        if (!isManifold)
        {
            using ProgressScope aPS1 = new ProgressScope(aPSRoot.Next(), "Process", 1);

            // Set tolerances for shape processing.
            // These parameters are calculated inside STEPControl_ActorRead::Transfer() and cannot be set
            // from outside.
            NCollection_DataMap<string, string> aParameters = GetShapeFixParameters();
            XSAlgo_ShapeProcessor.SetParameter("FixShape.Tolerance3d", myPrecision, true, aParameters);
            XSAlgo_ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", myMaxTol, true, aParameters);
            XSAlgo_ShapeProcessor.SetParameter("FixShape.NonManifold", "1", true, aParameters);
            XSAlgo_ShapeProcessor aShapeProcessor = new XSAlgo_ShapeProcessor(aParameters);
            TopoDS_Shape fixedResult = aShapeProcessor.InternalProcessShape(comp, GetProcessingFlags().Item1, aPS1.Next());
            aShapeProcessor.InternalMergeTransferInfo(TP, nbTPitems);

            if (fixedResult.ShapeType() == TopAbs_COMPOUND)
            {
                comp = TopoDS.Compound(fixedResult);
            }
            else
            {
                comp.Nullify();
                B.MakeCompound(ref comp);
                B.Add(comp, fixedResult);
            }

            BRep_Builder brepBuilder = new BRep_Builder();

            // [BEGIN] Try to close OPEN Shells in I-DEAS case (ssv; 17.11.2010)
            if (myNMTool.IsIDEASCase())
            {
                // ==========================================================
                //  For each Shell (always OPEN in I-DEAS case) find all
                //  the possibilities to close it with adjacent non-manifold
                //  Shells which are stored separately in I-DEAS-like STP
                // ==========================================================

                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> shellClosingsMap =
                    new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
                // Find possible closings for each shell
                computeIDEASClosings(comp, shellClosingsMap);

                // Make compound to store closed Shells
                TopoDS_Compound compWithClosings = new TopoDS_Compound();
                brepBuilder.MakeCompound(ref compWithClosings);

                // Attempt to close Shells one-by-one
                for (int i = 1; i <= shellClosingsMap.Extent(); i++)
                {
                    TopoDS_Shell adjustedShell = closeIDEASShell(TopoDS.Shell(shellClosingsMap.FindKey(i)),
                                                                 shellClosingsMap.FindFromIndex(i));
                    brepBuilder.Add(compWithClosings, adjustedShell);
                }
                // Put not suspected open Shells as they are (updated 23.11.2010)
                TopExp_Explorer allShellsExp = new TopExp_Explorer(comp, TopAbs_SHELL);
                for (; allShellsExp.More(); allShellsExp.Next())
                {
                    TopoDS_Shape aCurrentShell = allShellsExp.Current();
                    if (!myNMTool.IsPureNMShell(aCurrentShell) && !shellClosingsMap.Contains(aCurrentShell))
                    {
                        brepBuilder.Add(compWithClosings, aCurrentShell);
                    }
                }
                comp = compWithClosings;
            }
            // [END] Try to close OPEN Shells in I-DEAS case (ssv; 17.11.2010)

            // [BEGIN] Reconstruct Solids from Closed Shells (ssv; 15.11.2010)
            TopoDS_Compound reconstComp = new TopoDS_Compound();
            brepBuilder.MakeCompound(ref reconstComp);
            TopExp_Explorer exp = new TopExp_Explorer(comp, TopAbs_SHELL);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Shape aSubShape = exp.Current();
                if (aSubShape.ShapeType() == TopAbs_SHELL && aSubShape.Closed())
                {
                    TopoDS_Solid nextSolid = new TopoDS_Solid();
                    brepBuilder.MakeSolid(ref nextSolid);
                    brepBuilder.Add(nextSolid, aSubShape);
                    brepBuilder.Add(reconstComp, nextSolid);
                }
                else if (aSubShape.ShapeType() == TopAbs_SHELL)
                {
                    brepBuilder.Add(reconstComp, aSubShape);
                }
            }
            comp = reconstComp;
            // [END] Reconstruct Solids from Closed Shells (ssv; 15.11.2010)
        }

        // Bind the resulting shape
        if (nsh == 0)
        {
            shbinder = null;
        }
        else if (nsh == 1)
        {
            if (aTrsf.Form() != Identity)
            {
                Placement aLoc = new Placement(aTrsf);
                OneResult.Move(aLoc);
            }
            shbinder = new TransferBRep_ShapeBinder(OneResult);
        }
        else
        {
            if (aTrsf.Form() != Identity)
            {
                Placement aLoc = new Placement(aTrsf);
                comp.Move(aLoc);
            }
            shbinder = new TransferBRep_ShapeBinder(comp);
        }

        PrepareUnits(oldSRContext, TP, ref aLocalFactors); //: S4136
        TP.Bind(sr, shbinder);

        return shbinder;
    }

    /// <summary>Transfers context dependent shape representation entity</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepShape_ContextDependentShapeRepresentation CDSR, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, ProgressRange theProgress = default)
    {
        TransferBRep_ShapeBinder shbinder = null;
        //: j2: treat SRRs here in order to compare them with NAUO
        StepRepr_ShapeRepresentationRelationship SRR = CDSR.RepresentationRelation();
        if (SRR is null)
        {
            return shbinder;
        }

        bool SRRReversed = STEPConstruct_Assembly.CheckSRRReversesNAUO(TP.Graph(), CDSR);
        StepRepr_Representation rep1 = (SRRReversed ? SRR.Rep2() : SRR.Rep1());
        StepShape_ShapeRepresentation rep = rep1 as StepShape_ShapeRepresentation;

        if (SRRReversed)
        {
            TP.AddWarning(SRR, "SRR reverses relation defined by NAUO; NAUO definition is taken");
        }

        TopoDS_Shape theResult;

        Transform3D Trsf = new Transform3D();
        bool iatrsf = ComputeSRRWT(SRR, TP, ref Trsf, theLocalFactors);

        Transfer_Binder binder;
        bool isBound = false;
        if (!TP.IsBound(rep))
        {
            binder = TransferEntity(rep, TP, theLocalFactors, ref isBound, false, theProgress);
        }
        else
        {
            binder = TP.Find(rep);
        }
        theResult = TransferBRep.ShapeResult(binder);

        if (!theResult.IsNull())
        {
            if (iatrsf)
            {
                if (SRRReversed)
                {
                    ApplyTransformation(ref theResult, Trsf.Inverted());
                }
                else
                {
                    ApplyTransformation(ref theResult, Trsf);
                }
            }
            shbinder = new TransferBRep_ShapeBinder(theResult);
        }
        else
        {
            shbinder = null;
        }
        TP.Bind(CDSR, shbinder);
        return shbinder;
    }

    /// <summary>Transfers shape representation relationship entity</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepRepr_ShapeRepresentationRelationship und, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, int nbrep = 0, bool theUseTrsf = false,
                                                      ProgressRange theProgress = default)
    {
        //  REPRESENTATION_RELATIONSHIP et la famille
        TransferBRep_ShapeBinder shbinder = null;

        if (und is null)
        {
            return shbinder;
        }

        //  On prend son contenu : Rep1 ou Rep2 , that is the question
        //    on prend les 2. Mais quoi faire des axes

        TopoDS_Compound Cund = new TopoDS_Compound();
        TopoDS_Shape OneResult = new TopoDS_Shape();
        BRep_Builder B = new BRep_Builder();
        B.MakeCompound(ref Cund);
        int nsh = 0;

        Transform3D Trsf = new Transform3D();
        bool iatrsf = ComputeSRRWT(und, TP, ref Trsf, theLocalFactors);

        //    Transfert : que faut-il prendre au juste ?
        using ProgressScope aPS = new ProgressScope(theProgress, null, 2);
        for (int i = 1; i <= 2 && aPS.More(); i++)
        {
            ProgressRange aRange = aPS.Next();
            if (nbrep != 0 && nbrep != i)
            {
                continue;
            }
            StepRepr_Representation anitemt = null;
            if (i == 1)
            {
                anitemt = und.Rep1();
            }
            if (i == 2)
            {
                anitemt = und.Rep2();
            }
            StepShape_ShapeRepresentation anitem = anitemt as StepShape_ShapeRepresentation;
            Transfer_Binder binder;
            bool isBound = false;
            if (!TP.IsBound(anitem))
            {
                binder = TransferEntity(anitem, TP, theLocalFactors, ref isBound, theUseTrsf, aRange);
            }
            else
            {
                binder = TP.Find(anitem);
            }
            TopoDS_Shape theResult = TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull())
            {
                OneResult = theResult;
                B.Add(Cund, theResult);
                nsh++;
            }
        }

        //   La Transformation : selon les cas

        //    Appliquer la transformation

        if (iatrsf)
        {
            if (nsh == 1)
            {
                ApplyTransformation(ref OneResult, Trsf);
            }
            else if (nsh > 1)
            {
                ApplyTransformation(ref Cund, Trsf);
            }
        }
        if (nsh == 0)
        {
            shbinder = null;
        }
        else if (nsh == 1)
        {
            shbinder = new TransferBRep_ShapeBinder(OneResult);
        }
        else
        {
            shbinder = new TransferBRep_ShapeBinder(Cund);
        }
        TP.Bind(und, shbinder);
        return shbinder;
    }

    private TransferBRep_ShapeBinder TransferEntity(StepRepr_ConstructiveGeometryRepresentationRelationship theCGRR,
                                                    Transfer_TransientProcess theTP, in StepData_Factors theLocalFactors)
    {
        TransferBRep_ShapeBinder shbinder = null;
        if (theCGRR is null)
        {
            return shbinder;
        }
        bool resetUnits = false;
        StepRepr_Representation oldSRContext = mySRContext;
        StepData_Factors aLocalFactors = theLocalFactors;
        TopoDS_Compound aComp = new TopoDS_Compound();
        BRep_Builder aB = new BRep_Builder();
        aB.MakeCompound(ref aComp);
        for (int i = 1; i <= 2; i++)
        {
            StepRepr_ConstructiveGeometryRepresentation aCRepr =
                (i == 1 ? theCGRR.Rep1() : theCGRR.Rep2()) as StepRepr_ConstructiveGeometryRepresentation;
            if (aCRepr is null)
            {
                continue;
            }
            if (mySRContext is null || aCRepr.ContextOfItems() != mySRContext.ContextOfItems())
            {
                PrepareUnits(aCRepr, theTP, ref aLocalFactors);
                resetUnits = true;
            }
            int j = 1;
            for (; j <= aCRepr.NbItems(); j++)
            {
                StepRepr_RepresentationItem anItem = aCRepr.ItemsValue(j);
                StepGeom_Axis2Placement3d aStepAxis = anItem as StepGeom_Axis2Placement3d;
                if (aStepAxis is not null)
                {
                    Geom_Axis2Placement anAxis = StepToGeom.MakeAxis2Placement(aStepAxis, aLocalFactors);
                    if (anAxis is null)
                    {
                        continue;
                    }
                    Geom_Plane aPlane = new Geom_Plane(new CoordinateSystem(anAxis.Ax2()));
                    TopoDS_Face aPlaneFace = new TopoDS_Face();
                    aB.MakeFace(ref aPlaneFace, aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());
                    TransferBRep_ShapeBinder axisbinder = new TransferBRep_ShapeBinder(aPlaneFace);
                    theTP.Bind(aStepAxis, axisbinder);
                    aB.Add(aComp, aPlaneFace);
                }
            }
        }
        shbinder = new TransferBRep_ShapeBinder(aComp);
        mySRContext = oldSRContext;
        if (oldSRContext is null || resetUnits)
        {
            PrepareUnits(oldSRContext, theTP, ref aLocalFactors);
        }

        theTP.Bind(theCGRR, shbinder);

        return shbinder;
    }

    /// <summary>Transfers MechanicalDesignAndDraughtingRelationship entity</summary>
    private TransferBRep_ShapeBinder TransferEntity(StepRepr_MechanicalDesignAndDraughtingRelationship theMDADR,
                                                    Transfer_TransientProcess theTP, in StepData_Factors theLocalFactors,
                                                    ProgressRange theProgress)
    {
        TransferBRep_ShapeBinder aShBinder = null;
        if (theMDADR is null)
        {
            return aShBinder;
        }

        bool aResetUnits = false;
        StepRepr_Representation anOldSRContext = mySRContext;
        StepData_Factors aLocalFactors = theLocalFactors;
        TopoDS_Compound aComp = new TopoDS_Compound();
        BRep_Builder aBuilder = new BRep_Builder();
        aBuilder.MakeCompound(ref aComp);

        using ProgressScope aPS = new ProgressScope(theProgress, null, 2);
        for (int anIndex = 1; anIndex <= 2; anIndex++)
        {
            ProgressRange aRange = aPS.Next();
            StepRepr_Representation aRepr = (anIndex == 1) ? theMDADR.Rep1() : theMDADR.Rep2();
            if (aRepr is null)
            {
                continue;
            }
            if (mySRContext is null || aRepr.ContextOfItems() != mySRContext.ContextOfItems())
            {
                PrepareUnits(aRepr, theTP, ref aLocalFactors);
                aResetUnits = true;
            }
            Transfer_Binder aBinder = null;
            if (aRepr.IsKind(typeof(StepShape_ShapeRepresentation)))
            {
                StepShape_ShapeRepresentation aShapeRepr = aRepr as StepShape_ShapeRepresentation;
                bool isBound = false;
                if (!theTP.IsBound(aShapeRepr))
                {
                    aBinder = TransferEntity(aShapeRepr, theTP, theLocalFactors, ref isBound, false, aRange);
                }
                else
                {
                    aBinder = theTP.Find(aShapeRepr);
                }
            }
            TopoDS_Shape aResult = TransferBRep.ShapeResult(aBinder);
            if (!aResult.IsNull())
            {
                aBuilder.Add(aComp, aResult);
            }
        }

        aShBinder = new TransferBRep_ShapeBinder(aComp);
        mySRContext = anOldSRContext;
        if (anOldSRContext is null || aResetUnits)
        {
            PrepareUnits(anOldSRContext, theTP, ref aLocalFactors);
        }

        theTP.Bind(theMDADR, aShBinder);

        return aShBinder;
    }

    private static bool IsNeedRepresentation(StepRepr_ShapeAspect sa, StepRepr_Representation repA, Transfer_TransientProcess TP)
    {
        bool IsSDRaspect = true;
        StepRepr_ProductDefinitionShape PDSA = sa.OfShape();
        Interface_Graph graph = TP.Graph();
        Interface_EntityIterator subs7 = graph.Sharings(PDSA);
        for (subs7.Start(); PDSA is not null && subs7.More(); subs7.Next())
        {
            StepShape_ShapeDefinitionRepresentation sdrA = subs7.Value() as StepShape_ShapeDefinitionRepresentation;
            if (sdrA is null)
            {
                continue;
            }
            // abv 23 Feb 00: use iterator to take into account AP203 hybrid models
            Interface_EntityIterator subs8 = graph.Shareds(sdrA);
            for (subs8.Start(); subs8.More(); subs8.Next())
            {
                StepRepr_Representation rep2 = subs8.Value() as StepRepr_Representation;
                if (rep2 is null)
                {
                    StepRepr_ShapeRepresentationRelationship SRR = subs8.Value() as StepRepr_ShapeRepresentationRelationship;
                    if (SRR is null)
                    {
                        continue;
                    }
                    rep2 = (sdrA.UsedRepresentation() == SRR.Rep1() ? SRR.Rep2() : SRR.Rep1());
                }
                int i, j; // svv Jan 11 2000 : porting on DEC
                for (i = 1; i <= repA.NbItems(); i++)
                {
                    StepRepr_RepresentationItem it = repA.ItemsValue(i);
                    for (j = 1; j <= rep2.NbItems(); j++)
                    {
                        if (it == rep2.ItemsValue(j))
                        {
                            break;
                        }
                    }
                    if (j > rep2.NbItems())
                    {
                        break;
                    }
                }
                if (i > repA.NbItems())
                {
                    IsSDRaspect = false;
                }
            }
        }
        return IsSDRaspect;
    }

    /// <summary>
    /// Translates file by old way when CDSR are roots. Acts only if "read.step.product_mode" is equal Off.
    /// </summary>
    protected TransferBRep_ShapeBinder OldWay(Transient start, Transfer_TransientProcess TP, ProgressRange theProgress)
    {
        Interface_Graph graph = TP.Graph();
        TransferBRep_ShapeBinder shbinder = null;
        StepShape_ShapeDefinitionRepresentation sdr = start as StepShape_ShapeDefinitionRepresentation;
        StepRepr_Representation rep = sdr.UsedRepresentation();

        // abv 7 Oct 99: TRJ2: skip SDRs used only for defining SHAPE_ASPECT (validation properties)
        // BUT ONLY if they have representation duplicated with other SDR,
        // (SHAPE_ASPECT also used by some systems to define geometry)
        StepRepr_PropertyDefinition PD = sdr.Definition().PropertyDefinition();
        if (PD is not null)
        {
            StepRepr_ShapeAspect SA = PD.Definition().ShapeAspect();
            if (SA is not null)
            {
                if (!IsNeedRepresentation(SA, rep, TP))
                {
                    return shbinder;
                }
            }
        }

        using ProgressScope aPSRoot = new ProgressScope(theProgress, null, 2);

        // TRANSLOG
        if (TP.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
            sout.Append(" -- Actor : case  shape_definition_representation.").Append('\n');
        }
        Transfer_Binder binder = TP.Find(rep);
        {
            ProgressRange aRange = aPSRoot.Next();
            if (binder is null)
            {
                binder = TP.Transferring(rep, aRange);
            }
        }
        if (aPSRoot.UserBreak())
        {
            return shbinder;
        }
        //: j2    if (!binder.IsNull()) return binder;

        //    SDR designant des CDSR (lien implicite, via la UsedRepr)

        TopoDS_Compound Cund = new TopoDS_Compound();
        TopoDS_Shape OneResult = new TopoDS_Shape();
        BRep_Builder B = new BRep_Builder();
        B.MakeCompound(ref Cund);
        int nsh = 0;

        //: j2
        shbinder = new TransferBRep_ShapeBinder();
        TP.Bind(start, shbinder); //:j3 abv 23 Oct 98: rp1sd.stp: bind something to protect against loops
        if (binder is not null)
        {
            TopoDS_Shape theResult = TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull())
            {
                OneResult = theResult;
                B.Add(Cund, theResult);
                nsh++;
                shbinder.SetResult(theResult);
            }
        }

        // process subcomponents of assembly (CDSR) and hybrid models (SRR)
        Interface_EntityIterator subs = graph.Shareds(start);
        Type tCDSR = typeof(StepShape_ContextDependentShapeRepresentation);
        Type tSRR = typeof(StepRepr_ShapeRepresentationRelationship);
        int nbitem = 0;
        for (subs.Start(); subs.More(); subs.Next())
        {
            nbitem++;
        }
        using ProgressScope PS = new ProgressScope(aPSRoot.Next(), "Sub", nbitem);
        for (subs.Start(); subs.More() && PS.More(); subs.Next())
        {
            ProgressRange aRange = PS.Next();
            Transient anitem = subs.Value();
            if (anitem.DynamicType() != tCDSR && anitem.DynamicType() != tSRR)
            {
                continue;
            }
            // TRANSLOG
            if (TP.TraceLevel() > 1)
            {
                using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
                sout.Append(" -- Actor : Ent.n0 ").Append(TP.Model().Number(start)).Append(" -> Shared Ent.no")
                    .Append(TP.Model().Number(anitem)).Append('\n');
            }

            if (!TP.IsBound(anitem))
            {
                binder = TP.Transferring(anitem, aRange);
            }
            else
            {
                binder = TP.Find(anitem);
            }
            TopoDS_Shape theResult = TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull())
            {
                OneResult = theResult;
                B.Add(Cund, theResult);
                nsh += 2; // abv 11.10.00: 2 instead of 1 in order to keep assembly structure
            }
        }

        if (nsh == 0)
        {
            shbinder = new TransferBRep_ShapeBinder(Cund); // shbinder.Nullify();
        }
        else if (nsh == 1)
        {
            shbinder = new TransferBRep_ShapeBinder(OneResult);
        }
        else
        {
            shbinder = new TransferBRep_ShapeBinder(Cund);
        }
        TP.Unbind(start); //: j3: unbind start (let it be bound by TransferProcess)
        return shbinder;
    }

    /// <summary>Transfers geometric representation item entity such as ManifoldSolidBRep ,...etc</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepGeom_GeometricRepresentationItem start, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, bool isManifold,
                                                      ProgressRange theProgress)
    {
        TransferBRep_ShapeBinder shbinder = null;
        bool found = false;
        StepToTopoDS_Builder myShapeBuilder = new StepToTopoDS_Builder();
        TopoDS_Shape mappedShape = new TopoDS_Shape();
        int nbTPitems = TP.NbMapped();
        // TRANSLOG (OCCT's chronometer, shown at trace level 3, left out)
        if (TP.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
            sout.Append("Begin transfer STEP -> CASCADE, Type ").Append(start.DynamicTypeName()).Append('\n');
        }
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        //: S4136
        StepData_Factors aLocalFactors = theLocalFactors;
        StepRepr_Representation oldSRContext = mySRContext;
        if (mySRContext is null)
        { // if no context, try to find it (ex: r0701_ug.stp #4790)
            StepRepr_Representation context = FindContext(start, TP);
            if (context is null)
            {
                TP.AddWarning(start, "Entity with no unit context; default units taken");
                ResetUnits(aStepModel, ref aLocalFactors);
            }
            else
            {
                PrepareUnits(context, TP, ref aLocalFactors);
            }
        }
        myShapeBuilder.SetPrecision(myPrecision);
        myShapeBuilder.SetMaxTol(myMaxTol);

        // Start progress scope (no need to check if progress exists -- it is safe)
        using ProgressScope aPS = new ProgressScope(theProgress, "Transfer stage", isManifold ? 2 : 1);
        bool aReadTessellatedWhenNoBRepOnly = (aStepModel.InternalParameters.ReadTessellated == DESTEP_Parameters.RWMode_Tessellated.RWMode_Tessellated_OnNoBRep);
        bool aHasGeom = true;
        try
        {
            ProgressRange aRange = aPS.Next();
            if (start.IsKind(typeof(StepShape_FacetedBrep)))
            {
                myShapeBuilder.Init(start as StepShape_FacetedBrep, TP, aLocalFactors, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepShape_BrepWithVoids)))
            {
                myShapeBuilder.Init(start as StepShape_BrepWithVoids, TP, aLocalFactors, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepShape_ManifoldSolidBrep)))
            {
                myShapeBuilder.Init(start as StepShape_ManifoldSolidBrep, TP, aLocalFactors, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepShape_ShellBasedSurfaceModel)))
            {
                myShapeBuilder.Init(start as StepShape_ShellBasedSurfaceModel, TP, myNMTool, aLocalFactors, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepShape_FacetedBrepAndBrepWithVoids)))
            {
                myShapeBuilder.Init(start as StepShape_FacetedBrepAndBrepWithVoids, TP, aLocalFactors, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepShape_GeometricSet)))
            {
                myShapeBuilder.Init(start as StepShape_GeometricSet, TP, aLocalFactors, this, isManifold, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepShape_EdgeBasedWireframeModel)))
            {
                myShapeBuilder.Init(start as StepShape_EdgeBasedWireframeModel, TP, aLocalFactors);
                found = true;
            }
            else if (start.IsKind(typeof(StepShape_FaceBasedSurfaceModel)))
            {
                myShapeBuilder.Init(start as StepShape_FaceBasedSurfaceModel, TP, aLocalFactors);
                found = true;
            }
            // TODO: Normally, StepVisual_Tessellated* entities should be processed after
            //       StepShape_* entities in order to resolve links to BRep topological objects.
            //       Currently it is not guaranteed and might require changes in the processing order.
            else if (start.IsKind(typeof(StepVisual_TessellatedSolid)))
            {
                myShapeBuilder.Init(start as StepVisual_TessellatedSolid, TP, aReadTessellatedWhenNoBRepOnly, ref aHasGeom, aLocalFactors, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepVisual_TessellatedShell)))
            {
                myShapeBuilder.Init(start as StepVisual_TessellatedShell, TP, aReadTessellatedWhenNoBRepOnly, ref aHasGeom, aLocalFactors, aRange);
                found = true;
            }
            else if (start.IsKind(typeof(StepVisual_TessellatedFace)))
            {
                myShapeBuilder.Init(start as StepVisual_TessellatedFace, TP, aReadTessellatedWhenNoBRepOnly, ref aHasGeom, aLocalFactors);
                found = true;
            }
            else if (start.IsKind(typeof(StepVisual_TessellatedSurfaceSet)))
            {
                myShapeBuilder.Init(start as StepVisual_TessellatedSurfaceSet, TP, ref aHasGeom, aLocalFactors);
                found = true;
            }
        }
        catch (Exception anException) when (anException is FailureException || StepToTopoDS_Root.CaughtBySignals(anException))
        {
            TP.AddFail(start, "Exception is raised. Entity was not translated.");
            TP.Bind(start, shbinder);
            return shbinder;
        }

        if (aPS.UserBreak())
        {
            return shbinder;
        }

        if (found && myShapeBuilder.IsDone())
        {
            mappedShape = myShapeBuilder.InternalValue();
            // Apply ShapeFix (on manifold shapes only. Non-manifold topology is processed separately:
            // ssv; 13.11.2010)
            if (isManifold && aHasGeom)
            {
                // Set tolerances for shape processing.
                // These parameters are calculated inside STEPControl_ActorRead::Transfer() and cannot be set
                // from outside.
                NCollection_DataMap<string, string> aParameters = GetShapeFixParameters();
                XSAlgo_ShapeProcessor.SetParameter("FixShape.Tolerance3d", myPrecision, true, aParameters);
                XSAlgo_ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", myMaxTol, true, aParameters);
                XSAlgo_ShapeProcessor aShapeProcessor = new XSAlgo_ShapeProcessor(aParameters);
                mappedShape = aShapeProcessor.InternalProcessShape(mappedShape, GetProcessingFlags().Item1, aPS.Next());
                aShapeProcessor.InternalMergeTransferInfo(TP, nbTPitems);
            }
        }
        found = !mappedShape.IsNull();
        if (found && shbinder is null)
        {
            shbinder = new TransferBRep_ShapeBinder(mappedShape);
        }
        // TRANSLOG
        if (TP.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
            sout.Append("End transfer STEP -> CASCADE :").Append(found ? "OK" : " : no result").Append('\n');
        }
        if (oldSRContext is null && mySRContext is not null)
        { //: S4136
            PrepareUnits(oldSRContext, TP, ref aLocalFactors);
        }
        TP.Bind(start, shbinder);
        return shbinder;
    }

    /// <summary>Transfers mapped item</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepRepr_MappedItem mapit, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, ProgressRange theProgress)
    {
        TransferBRep_ShapeBinder shbinder = null;

        // --------------------------------------------------------------
        // On se trouve ici dans un contexte " d'assemblage geometrique "
        //    - MappedItem
        // --------------------------------------------------------------

        //: S4136: abv 20 Apr 99: as1ug.stp: MAPPED_ITEM transformation computed
        //        taking into account units of origin and target SHAPE_REPRESENTATIONs

        //  La Shape, et la mise en position
        StepShape_ShapeRepresentation maprep = mapit.MappingSource().MappedRepresentation() as StepShape_ShapeRepresentation;
        bool isBound = false;
        using ProgressScope aPSRoot = new ProgressScope(theProgress, null, 2);
        Transfer_Binder binder = TP.Find(maprep);
        if (binder is null)
        {
            binder = TransferEntity(maprep, TP, theLocalFactors, ref isBound, false, theProgress);
        }
        shbinder = binder as TransferBRep_ShapeBinder;
        if (shbinder is null)
        {
            TP.AddWarning(mapit, "No Shape Produced");
        }
        else
        {
            TopoDS_Shape mappedShape = shbinder.Result();
            if (!mappedShape.IsNull())
            {
                //  Positionnement : 2 formules
                //  1/ Ax2 dans Source et comme Target  : passage de Source a Target
                //  2/ CartesianOperator3d comme Target : on applique

                Transform3D Trsf = new Transform3D();
                bool ok = false;

                StepGeom_CartesianTransformationOperator3d CartOp = mapit.MappingTarget() as StepGeom_CartesianTransformationOperator3d;
                if (CartOp is not null)
                {
                    ok = StepToGeom.MakeTransformation3d(CartOp, ref Trsf, theLocalFactors);
                }
                else
                {
                    StepGeom_Axis2Placement3d Origin = mapit.MappingSource().MappingOrigin() as StepGeom_Axis2Placement3d;
                    StepGeom_Axis2Placement3d Target = mapit.MappingTarget() as StepGeom_Axis2Placement3d;
                    if (Origin is not null && Target is not null)
                    {
                        ok = true;
                        StepRepr_Representation rep = mySRContext; // NOTE: copy of handle !
                        ComputeTransformation(Origin, Target, maprep, rep, TP, ref Trsf, theLocalFactors);
                        ok = true;
                    }
                }

                if (ok)
                {
                    ApplyTransformation(ref mappedShape, Trsf);
                }
                else
                {
                    TP.AddWarning(mapit, "Mapped Item, case not recognized, location ignored");
                }

                shbinder = new TransferBRep_ShapeBinder(mappedShape);
            }
        }
        TopoDS_Compound aCund = new TopoDS_Compound();
        TopoDS_Shape aResult = new TopoDS_Shape();
        BRep_Builder aBuilder = new BRep_Builder();
        aBuilder.MakeCompound(ref aCund);
        if (shbinder is not null)
        {
            aResult = TransferBRep.ShapeResult(shbinder);
            aBuilder.Add(aCund, aResult);
        }

        // translate possible shapes related by SRRs, which corresponds to
        // way of writing hybrid models in AP203 since 1998, and AP209
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        if (aStepModel.InternalParameters.ReadRelationship)
        {
            Interface_Graph aGraph = TP.Graph();
            int aSRRnum = 0;
            for (Interface_EntityIterator aSubsIt = aGraph.Sharings(maprep); aSubsIt.More(); aSubsIt.Next())
            {
                ++aSRRnum;
            }
            using ProgressScope aPS = new ProgressScope(aPSRoot.Next(), "Part", aSRRnum);
            TopoDS_Shape aNewResult = TransferRelatedSRR(TP, maprep, false, aStepModel.InternalParameters.ReadConstrRelation,
                                                         theLocalFactors, aCund, aPS);
            if (!aNewResult.IsNull())
            {
                aResult = aNewResult;
            }
            // if only single shape is read, add it as it is; otherwise add compound
            if (aCund.NbChildren() == 1)
            {
                shbinder = new TransferBRep_ShapeBinder(aResult);
            }
            else if (aCund.NbChildren() > 1)
            {
                shbinder = new TransferBRep_ShapeBinder(aCund);
            }
        }
        TP.Bind(mapit, shbinder);
        return shbinder;
    }

    /// <summary>Transfers FaceSurface entity</summary>
    protected TransferBRep_ShapeBinder TransferEntity(StepShape_FaceSurface fs, Transfer_TransientProcess TP,
                                                      in StepData_Factors theLocalFactors, ProgressRange theProgress)
    {
        //    Cas bien utile meme si non reconnu explicitement
        TransferBRep_ShapeBinder sb = null;
        int nbTPitems = TP.NbMapped();
        try
        {
            StepToTopoDS_Tool myTool = new StepToTopoDS_Tool();
            NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape> aMap =
                new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>();

            myTool.Init(aMap, TP);
            StepToTopoDS_TranslateFace myTF = new StepToTopoDS_TranslateFace();
            myTF.SetPrecision(myPrecision);
            myTF.SetMaxTol(myMaxTol);

            // Non-manifold topology is not processed here (ssv; 15.11.2010)
            StepToTopoDS_NMTool dummyNMTool = new StepToTopoDS_NMTool();
            myTF.Init(fs, myTool, dummyNMTool, theLocalFactors);
            StepRepr_Representation oldSRContext = mySRContext;
            StepData_Factors aLocalFactors = theLocalFactors;
            StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
            if (mySRContext is null)
            { // if no context, try to find it (ex: r0701_ug.stp #4790)
                StepRepr_Representation context = FindContext(fs, TP);
                if (context is null)
                {
                    TP.AddWarning(fs, "Entity with no unit context; default units taken");
                    ResetUnits(aStepModel, ref aLocalFactors);
                }
                else
                {
                    PrepareUnits(context, TP, ref aLocalFactors);
                }
            }

            // Apply ShapeFix
            Transfer_Binder binder = TP.Find(fs);
            sb = binder as TransferBRep_ShapeBinder;
            if (sb is not null && !sb.Result().IsNull())
            {
                TopoDS_Shape S = sb.Result();

                NCollection_DataMap<string, string> aParameters = GetShapeFixParameters();
                XSAlgo_ShapeProcessor.SetParameter("FixShape.Tolerance3d", myPrecision, true, aParameters);
                XSAlgo_ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", myMaxTol, true, aParameters);
                XSAlgo_ShapeProcessor aShapeProcessor = new XSAlgo_ShapeProcessor(aParameters);
                TopoDS_Shape shape = aShapeProcessor.InternalProcessShape(S, GetProcessingFlags().Item1, theProgress);
                aShapeProcessor.InternalMergeTransferInfo(TP, nbTPitems);

                if (shape != S)
                {
                    sb.SetResult(shape);
                }

                aShapeProcessor.InternalMergeTransferInfo(TP, nbTPitems);
            }

            if (oldSRContext is null && mySRContext is not null)
            { //: S4136
                PrepareUnits(oldSRContext, TP, ref aLocalFactors);
            }
            TP.Bind(fs, sb);
            return sb; // TP->Find (start);
        }
        catch (Exception anException) when (anException is FailureException || StepToTopoDS_Root.CaughtBySignals(anException))
        {
            TP.AddFail(fs, "Exception is raised. Entity was not translated.");
            sb = null;
            TP.Bind(fs, sb);
            return sb;
        }
    }

    /// <summary>
    /// theUseTrsf - special flag for using Axis2Placement from ShapeRepresentation for transform root shape
    /// </summary>
    public Transfer_Binder TransferShape(Transient start, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors,
                                         bool isManifold = true, bool theUseTrsf = false, ProgressRange theProgress = default)
    {
        if (start is null)
        {
            return NullResult();
        }
        // OCCT.NET: a cycle of assembly links overflows OCCT's stack; here an exception (decision 1268)
        RuntimeHelpers.EnsureSufficientExecutionStack();

        // TRANSLOG
        //  POUR MISE AU POINT, a supprimer ensuite
        if (TP.TraceLevel() > 1)
        {
            using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
            sout.Append(" -- Actor : Transfer Ent.n0 ").Append(TP.Model().Number(start)).Append("  Type ")
                .Append(start.DynamicTypeName()).Append('\n');
        }

        TransferBRep_ShapeBinder shbinder = null;
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;

        // Product Definition Entities
        // They should be treated with Design Manager
        // case ShapeDefinitionRepresentation if ProductMode != ON
        bool aProdMode = aStepModel.InternalParameters.ReadProductMode;
        if (!aProdMode && start.IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
        {
            shbinder = OldWay(start, TP, theProgress);
            // skl
        }
        else if (start.IsKind(typeof(StepBasic_ProductDefinition)))
        {
            StepBasic_ProductDefinition PD = start as StepBasic_ProductDefinition;
            shbinder = TransferEntity(PD, TP, theLocalFactors, theUseTrsf, theProgress);
        }

        // NextAssemblyUsageOccurrence
        else if (start.IsKind(typeof(StepRepr_NextAssemblyUsageOccurrence)))
        {
            StepRepr_NextAssemblyUsageOccurrence NAUO = start as StepRepr_NextAssemblyUsageOccurrence;
            shbinder = TransferEntity(NAUO, TP, theLocalFactors, theProgress);
        }
        // end skl

        // Shape Representation
        else if (start.IsKind(typeof(StepShape_ShapeRepresentation)))
        {
            StepShape_ShapeRepresentation sr = start as StepShape_ShapeRepresentation;
            bool isBound = false;
            shbinder = TransferEntity(sr, TP, theLocalFactors, ref isBound, false, theProgress);
        }

        // --------------------------------------------------------------
        // On se trouve ici aussi dans un contexte " d'assemblage geometrique "
        //    - ShapeRepresentationRelationship + Transformation ou non
        // --------------------------------------------------------------

        else if (start.IsKind(typeof(StepShape_ContextDependentShapeRepresentation)))
        {
            StepShape_ContextDependentShapeRepresentation CDSR = start as StepShape_ContextDependentShapeRepresentation;
            shbinder = TransferEntity(CDSR, TP, theLocalFactors, theProgress);
        }

        else if (start.IsKind(typeof(StepRepr_ShapeRepresentationRelationship)))
        {
            //  REPRESENTATION_RELATIONSHIP et la famille

            StepRepr_ShapeRepresentationRelationship und = start as StepRepr_ShapeRepresentationRelationship;
            shbinder = TransferEntity(und, TP, theLocalFactors, 0, false, theProgress);
        }

        else if (start.IsKind(typeof(StepGeom_GeometricRepresentationItem)))
        {
            // Here starts the entity to be treated : Shape Representation Subtype
            // It can be also other Root entities
            StepGeom_GeometricRepresentationItem git = start as StepGeom_GeometricRepresentationItem;
            shbinder = TransferEntity(git, TP, theLocalFactors, isManifold, theProgress);
        }
        else if (start.IsKind(typeof(StepRepr_MappedItem)))
        {
            StepRepr_MappedItem mapit = start as StepRepr_MappedItem;
            shbinder = TransferEntity(mapit, TP, theLocalFactors, theProgress);
        }
        else if (start.IsKind(typeof(StepShape_FaceSurface)))
        {
            StepShape_FaceSurface fs = start as StepShape_FaceSurface;
            shbinder = TransferEntity(fs, TP, theLocalFactors, theProgress);
        }

        //  if (!shbinder.IsNull()) TP->Bind(start,binder);

        return shbinder;
    }

    public Transfer_Binder TransferShape(Transient start, Transfer_TransientProcess TP)
        => TransferShape(start, TP, new StepData_Factors());

    Transfer_Binder StepToTopoDS_IActorRead.TransferShape(Transient start, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors,
                                                          bool isManifold, bool theUseTrsf, ProgressRange theProgress)
        => TransferShape(start, TP, theLocalFactors, isManifold, theUseTrsf, theProgress);

    /// <summary>set units and tolerances context by given ShapeRepresentation</summary>
    public void PrepareUnits(StepRepr_Representation rep, Transfer_TransientProcess TP, ref StepData_Factors theLocalFactors)
    {
        mySRContext = rep;
        StepData_StepModel aModel = TP.Model() as StepData_StepModel;
        int stat1, stat2 = 0; // sera alimente par STEPControl_Unit
        if (rep is null)
        {
            ResetUnits(aModel, ref theLocalFactors);
            return;
        }

        // Get Units Applied to this model
        StepRepr_RepresentationContext theRepCont = rep.ContextOfItems();
        if (theRepCont is null)
        {
            TP.AddWarning(rep, "Bad RepresentationContext, default unit taken");
            ResetUnits(aModel, ref theLocalFactors);
            return;
        }

        // --------------------------------------------------
        // Complex ENTITY : GeometricRepresentationContext &&
        //                  GlobalUnitAssignedContext
        // --------------------------------------------------

        STEPConstruct_UnitContext myUnit = new STEPConstruct_UnitContext();
        StepRepr_GlobalUnitAssignedContext theGUAC = null;
        StepRepr_GlobalUncertaintyAssignedContext aTol = null;

        if (theRepCont.IsKind(typeof(StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext)))
        {
            StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext theGRCAGAUC =
                theRepCont as StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext;
            theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
        }

        // ----------------------------------------------------
        // Complex ENTITY : GeometricRepresentationContext   &&
        //                  GlobalUnitAssignedContext        &&
        //                  GlobalUncertaintyAssignedContext
        // ----------------------------------------------------

        if (theRepCont.IsKind(typeof(StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx)))
        {
            StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx theGRCAGAUC =
                theRepCont as StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx;
            theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
            aTol = theGRCAGAUC.GlobalUncertaintyAssignedContext();
        }

        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        // ----------------------------------------------------
        // Decoding and Setting the Values
        // ----------------------------------------------------
        if (theGUAC is not null)
        {
            stat1 = myUnit.ComputeFactors(theGUAC, theLocalFactors);
            int anglemode = (int)aStepModel.InternalParameters.AngleUnit;
            double angleFactor = (anglemode == 0 ? myUnit.PlaneAngleFactor()
                                  : anglemode == 1 ? 1.0
                                                   : Math.PI / 180.0);
            theLocalFactors.InitializeFactors(myUnit.LengthFactor(), angleFactor, myUnit.SolidAngleFactor());
            if (stat1 != 0)
            {
                TP.AddWarning(theRepCont, myUnit.StatusMessage(stat1));
            }
        }

        if (aTol is not null)
        {
            stat2 = myUnit.ComputeTolerance(aTol);
            if (stat2 != 0)
            {
                TP.AddWarning(theRepCont, myUnit.StatusMessage(stat2));
            }
        }

        //  myPrecision = Precision::Confusion();
        if (aStepModel.InternalParameters.ReadPrecisionMode == DESTEP_Parameters.ReadMode_Precision.ReadMode_Precision_User)
        { //: i1 gka S4136 05.04.99
            myPrecision = aStepModel.InternalParameters.ReadPrecisionVal;
        }
        else if (myUnit.HasUncertainty())
        {
            myPrecision = myUnit.Uncertainty() * myUnit.LengthFactor();
        }
        else
        {
            TP.AddWarning(theRepCont, "No Length Uncertainty, value of read.precision.val is taken");
            myPrecision = aStepModel.InternalParameters.ReadPrecisionVal;
        }
        myMaxTol = Math.Max(myPrecision, aStepModel.InternalParameters.ReadMaxPrecisionVal);
        // Assign uncertainty
        // TRANSLOG
        if (TP.TraceLevel() > 1)
        {
            using Messenger.StreamBuffer sout = TP.Messenger().SendInfo();
            sout.Append("  Cc1ToTopoDS : Length Unit = ").Append(myUnit.LengthFactor()).Append("  Tolerance CASCADE = ")
                .Append(myPrecision).Append('\n');
        }
    }

    /// <summary>reset units and tolerances context to default (mm, radians, read.precision.val, etc.)</summary>
    public void ResetUnits(StepData_StepModel theModel, ref StepData_Factors theLocalFactors)
    {
        theLocalFactors.InitializeFactors(1, 1, 1);
        myPrecision = theModel.InternalParameters.ReadPrecisionVal;
        myMaxTol = Math.Max(myPrecision, theModel.InternalParameters.ReadMaxPrecisionVal);
    }

    /// <summary>
    /// Computes transformation defined by two axis placements (in MAPPED_ITEM or ITEM_DEFINED_TRANSFORMATION) taking
    /// into account their representation contexts (i.e. units, which may be different) Returns True if
    /// transformation is computed and is not an identity.
    /// </summary>
    public bool ComputeTransformation(StepGeom_Axis2Placement3d Origin, StepGeom_Axis2Placement3d Target, StepRepr_Representation OrigContext,
                                      StepRepr_Representation TargContext, Transfer_TransientProcess TP, ref Transform3D Trsf,
                                      in StepData_Factors theLocalFactors)
    {
        Trsf = new Transform3D(); // reinit
        if (Origin is null || Target is null)
        {
            return false;
        }

        //: abv 31.10.01: TEST_MCI_2.step: check that Ax1 and Ax2 belong to
        // corresponding reps and fix case of inversion error
        StepGeom_Axis2Placement3d org = Origin;
        StepGeom_Axis2Placement3d trg = Target;
        bool isOKOrigin = false, isSwapOrigin = false;
        bool isOKTarget = false, isSwapTarget = false;
        for (int i = 1; i <= OrigContext.NbItems(); i++)
        {
            if (OrigContext.ItemsValue(i) == org)
            {
                isOKOrigin = true;
            }
            else if (OrigContext.ItemsValue(i) == trg)
            {
                isSwapTarget = true;
            }
        }
        for (int i = 1; i <= TargContext.NbItems(); i++)
        {
            if (TargContext.ItemsValue(i) == trg)
            {
                isOKTarget = true;
            }
            else if (TargContext.ItemsValue(i) == org)
            {
                isSwapOrigin = true;
            }
        }
        if (!isOKOrigin || !isOKTarget)
        {
            if (isSwapOrigin && isSwapTarget)
            {
                (org, trg) = (trg, org);
                TP.AddWarning(org, "Axis placements are swapped in SRRWT; corrected");
            }
            else
            {
                TP.AddWarning((isOKOrigin ? trg : org),
                              "Axis placement used by SRRWT does not belong to corresponding representation");
            }
        }

        // translate axis_placements taking units into account
        StepRepr_Representation oldSRContext = mySRContext;
        StepData_Factors aLocalFactors = theLocalFactors;
        if (OrigContext != oldSRContext)
        {
            PrepareUnits(OrigContext, TP, ref aLocalFactors);
        }
        Geom_Axis2Placement theOrig = StepToGeom.MakeAxis2Placement(org, aLocalFactors);
        if (TargContext != OrigContext)
        {
            PrepareUnits(TargContext, TP, ref aLocalFactors);
        }
        Geom_Axis2Placement theTarg = StepToGeom.MakeAxis2Placement(trg, aLocalFactors);
        if (oldSRContext != TargContext)
        {
            PrepareUnits(oldSRContext, TP, ref aLocalFactors);
        }

        CoordinateSystem ax3Orig = new CoordinateSystem(theOrig.Ax2());
        CoordinateSystem ax3Targ = new CoordinateSystem(theTarg.Ax2());

        //  ne pas se tromper de sens !
        Trsf.SetTransformation(ax3Targ, ax3Orig);
        return Trsf.Form() != Identity;
    }

    public bool ComputeTransformation(StepGeom_Axis2Placement3d Origin, StepGeom_Axis2Placement3d Target, StepRepr_Representation OrigContext,
                                      StepRepr_Representation TargContext, Transfer_TransientProcess TP, ref Transform3D Trsf)
        => ComputeTransformation(Origin, Target, OrigContext, TargContext, TP, ref Trsf, new StepData_Factors());

    /// <summary>Computes transformation defined by given REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION</summary>
    public bool ComputeSRRWT(StepRepr_RepresentationRelationship SRR, Transfer_TransientProcess TP, ref Transform3D Trsf,
                             in StepData_Factors theLocalFactors)
    {
        Trsf = new Transform3D(); // init

        StepRepr_ShapeRepresentationRelationshipWithTransformation srwt = SRR as StepRepr_ShapeRepresentationRelationshipWithTransformation;
        if (srwt is null)
        {
            return false;
        }

        StepRepr_Transformation SelectTrans = srwt.TransformationOperator();
        StepData_Factors aLocalFactors = theLocalFactors;
        // cartesian transformation
        StepGeom_CartesianTransformationOperator3d CartOp = SelectTrans.Value() as StepGeom_CartesianTransformationOperator3d;
        if (CartOp is not null)
        {
            // reset units (by Rep2 - ?)
            StepRepr_Representation oldSRContext = mySRContext;
            if (SRR.Rep2() != oldSRContext)
            {
                PrepareUnits(SRR.Rep2(), TP, ref aLocalFactors);
            }
            StepToGeom.MakeTransformation3d(CartOp, ref Trsf, aLocalFactors);
            if (SRR.Rep2() != oldSRContext)
            {
                PrepareUnits(oldSRContext, TP, ref aLocalFactors);
            }
            return Trsf.Form() != Identity;
        }

        // item-defined transformation
        StepRepr_ItemDefinedTransformation ItemDef = SelectTrans.ItemDefinedTransformation();
        if (ItemDef is null)
        {
            return false;
        }

        StepGeom_Axis2Placement3d Ax1 = ItemDef.TransformItem1() as StepGeom_Axis2Placement3d;
        StepGeom_Axis2Placement3d Ax2 = ItemDef.TransformItem2() as StepGeom_Axis2Placement3d;
        if (Ax1 is null || Ax2 is null)
        {
            return false;
        }
        return ComputeTransformation(Ax1, Ax2, SRR.Rep1(), SRR.Rep2(), TP, ref Trsf, aLocalFactors);
    }

    public bool ComputeSRRWT(StepRepr_RepresentationRelationship SRR, Transfer_TransientProcess TP, ref Transform3D Trsf)
        => ComputeSRRWT(SRR, TP, ref Trsf, new StepData_Factors());

    //=======================================================================
    // Method  : closeIDEASShell
    // Purpose : Attempts to close the passed Shell with the passed closing
    //           Shells. Cuts redundant Faces from the closing Shells if any
    //=======================================================================
    private TopoDS_Shell closeIDEASShell(TopoDS_Shell shell, NCollection_List<TopoDS_Shape> closingShells)
    {
        // Make Shell to provide closeness adjustments
        TopoDS_Shell result = new TopoDS_Shell();
        BRep_Builder brepBuilder = new BRep_Builder();
        brepBuilder.MakeShell(ref result);

        // Firstly, add all existing faces to the new Shell
        TopExp_Explorer currentFExp = new TopExp_Explorer(shell, TopAbs_FACE);
        for (; currentFExp.More(); currentFExp.Next())
        {
            TopoDS_Face currentFace = TopoDS.Face(currentFExp.Current());
            brepBuilder.Add(result, currentFace);
        }

        NCollection_List<TopoDS_Shape> closingFaces = new NCollection_List<TopoDS_Shape>();

        // Then add the closing faces
        foreach (TopoDS_Shape currentClosing in closingShells)
        {
            TopExp_Explorer faceExp = new TopExp_Explorer(currentClosing, TopAbs_FACE);

            for (; faceExp.More(); faceExp.Next())
            {
                TopoDS_Face currentFace = TopoDS.Face(faceExp.Current());
                brepBuilder.Add(result, currentFace);
                // Store each added closing face for subsequent processing
                closingFaces.Append(currentFace);
            }
        }

        // Check if the result is closed
        BRepCheck_Shell checker = new BRepCheck_Shell(TopoDS.Shell(result));
        BRepCheck_Status checkStatus = checker.Closed();
        if (checkStatus == BRepCheck_NoError)
        {
            result.Closed(true);
        }
        else
        {
            return shell; // Cannot close this shell, skip it so...
        }

        // Try to remove redundant Faces
        foreach (TopoDS_Shape aClosingFace in closingFaces)
        {
            TopoDS_Face currentFace = TopoDS.Face(aClosingFace);
            // Remove face to see if Shell is still closed
            brepBuilder.Remove(result, currentFace);
            BRepCheck_Shell subChecker = new BRepCheck_Shell(TopoDS.Shell(result));
            BRepCheck_Status subCheckStatus = subChecker.Closed();
            // If Shell becomes open, just put the deleted face back
            if (subCheckStatus != BRepCheck_NoError)
            {
                brepBuilder.Add(result, currentFace);
            }
        }

        return result;
    }

    //=======================================================================
    // Method  : computeIDEASClosings
    // Purpose : For each Shell from the compound passed (comp), find all
    //           non-manifold adjacent Shells and put the results into
    //           the passed map (shellClosingMap)
    //=======================================================================
    private void computeIDEASClosings(TopoDS_Compound comp,
                                      NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> shellClosingsMap)
    {
        TopExp_Explorer shellExpA = new TopExp_Explorer(comp, TopAbs_SHELL);

        for (; shellExpA.More(); shellExpA.Next())
        {
            TopoDS_Shape shellA = shellExpA.Current();
            TopExp_Explorer shellExpB = new TopExp_Explorer(comp, TopAbs_SHELL);
            NCollection_List<TopoDS_Shape> closingShells = new NCollection_List<TopoDS_Shape>();

            for (; shellExpB.More(); shellExpB.Next())
            {
                TopoDS_Shape shellB = shellExpB.Current();
                if (shellA.IsSame(shellB))
                {
                    continue;
                }
                // Check whether ShellB is non-manifold and adjacent to ShellA.
                // If so, ShellA has a chance to be closed with ShellB
                if (myNMTool.IsSuspectedAsClosing(shellA, shellB))
                {
                    closingShells.Append(shellB);
                }
            }

            if (!closingShells.IsEmpty())
            {
                shellClosingsMap.Add(shellA, closingShells);
            }
        }
    }

    /// <summary>Set model</summary>
    public void SetModel(Interface_InterfaceModel theModel)
    {
        myModel = theModel;
    }

    //=======================================================================
    // Method  : TransferRelatedSRR
    // Purpose : Helper method to transfer SRR related to the representation
    //=======================================================================
    private TopoDS_Shape TransferRelatedSRR(Transfer_TransientProcess theTP, StepShape_ShapeRepresentation theRep, bool theUseTrsf,
                                            bool theReadConstructiveGeomRR, in StepData_Factors theLocalFactors, TopoDS_Compound theCund,
                                            ProgressScope thePS)
    {
        BRep_Builder aBuilder = new BRep_Builder();
        TopoDS_Shape aResult = new TopoDS_Shape();
        Interface_Graph aGraph = theTP.Graph();
        for (Interface_EntityIterator aSubsIt = aGraph.Sharings(theRep); aSubsIt.More() && thePS.More(); aSubsIt.Next())
        {
            Transient anItem = aSubsIt.Value();
            if (!anItem.IsKind(typeof(StepRepr_RepresentationRelationship)))
            {
                continue;
            }
            Transfer_Binder aBinder = null;
            if (anItem.DynamicType() == typeof(StepRepr_ShapeRepresentationRelationship))
            {
                StepRepr_ShapeRepresentationRelationship aSRR = anItem as StepRepr_ShapeRepresentationRelationship;
                int aNbRep = (theRep == aSRR.Rep1() ? 2 : 1);
                aBinder = TransferEntity(aSRR, theTP, theLocalFactors, aNbRep, theUseTrsf, thePS.Next());
            }
            else if (anItem.DynamicType() == typeof(StepRepr_MechanicalDesignAndDraughtingRelationship))
            {
                StepRepr_MechanicalDesignAndDraughtingRelationship aMDADR = anItem as StepRepr_MechanicalDesignAndDraughtingRelationship;
                aBinder = TransferEntity(aMDADR, theTP, theLocalFactors, thePS.Next());
            }
            else if (theReadConstructiveGeomRR
                     && anItem.DynamicType() == typeof(StepRepr_ConstructiveGeometryRepresentationRelationship))
            {
                StepRepr_ConstructiveGeometryRepresentationRelationship aCGRR = anItem as StepRepr_ConstructiveGeometryRepresentationRelationship;
                aBinder = TransferEntity(aCGRR, theTP, theLocalFactors);
            }
            if (aBinder is not null)
            {
                aResult = TransferBRep.ShapeResult(aBinder);
                aBuilder.Add(theCund, aResult);
            }
        }
        return aResult;
    }

    private StepToTopoDS_NMTool myNMTool = new StepToTopoDS_NMTool();
    private double myPrecision;
    private double myMaxTol;
    private StepRepr_Representation mySRContext;
    private Interface_InterfaceModel myModel;
}
