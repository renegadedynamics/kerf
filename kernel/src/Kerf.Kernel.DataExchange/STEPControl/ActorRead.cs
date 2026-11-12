// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorRead.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_ActorRead.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Import;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Tolerances;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Exchange.Step;

/// <summary>
/// This class performs the transfer of an Entity from AP214 and AP203, either Geometric or Topologic. I.E. for each
/// type of Entity, it invokes the appropriate Tool then returns the Binder which contains the Result
/// </summary>
/// <remarks>
/// <para>
/// Kerf (decisions 1247, 1249, 1265, 1268):
/// </para>
/// <list type="bullet">
/// <item>the shapes are made unfrozen (StepToTopoDS's <c>InternalValue</c>), bound and reused through the transient
/// process, and frozen when the reader publishes them (XSControl_Reader, decision 1224);</item>
/// <item>OCCT's file-level <c>_nmDetected</c> (a process-wide flag telling the callers that the representation just
/// translated was non-manifold) is the actor's own;</item>
/// <item>a model whose length unit isn't initialised is read in millimetres (OCCT: the process-wide unit, from the
/// static <c>xstep.cascade.unit</c>); STEPControl_Reader initialises it from its session's statics;</item>
/// <item>OCCT recurses along a file's assembly links (NAUO to product definition and back) without a guard, so a cycle
/// overflows the stack; the port checks the stack (<c>EnsureSufficientExecutionStack</c>) at each shape transfer,
/// which raises an exception the transient process records as a fail (occt-issues.md, 8F).</item>
/// </list>
/// </remarks>
internal class ActorRead : ActorOfTransientProcess, IActorRead
{
    // Set to inform outer methods that current representation item is non-manifold (OCCT: a file-level global).
    private bool _nmDetected;

    public ActorRead(InterfaceModel model)
    {
        _precision = 0.0;
        _maxTol = 0.0;
        _model = model;
    }

    public override bool Recognize(Transient start)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(ProductDefinition)))
        {
            return true;
        }

        if (start.IsKind(typeof(NextAssemblyUsageOccurrence)))
        {
            return true;
        }

        StepModel stepModel = _model as StepModel;
        if (stepModel is null)
        {
            return false;
        }
        bool prodMode = stepModel.InternalParameters.ReadProductMode;
        if (!prodMode)
        {
            if (start.IsKind(typeof(StepShapeDefinitionRepresentation)))
            {
                return true;
            }
        }

        StepShapeRepresentation sr = start as StepShapeRepresentation;
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

        bool canReadTessGeom = stepModel.InternalParameters.ReadTessellated != 0;

        if (start.IsKind(typeof(StepFacetedBrep)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepBrepWithVoids)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepManifoldSolidBrep)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepShellBasedSurfaceModel)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepFacetedBrepAndBrepWithVoids)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepGeometricSet)))
        {
            return true;
        }
        if (start.IsKind(typeof(MappedItem)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepFaceSurface)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepEdgeBasedWireframeModel)))
        {
            return true;
        }
        if (start.IsKind(typeof(StepFaceBasedSurfaceModel)))
        {
            return true;
        }
        if (canReadTessGeom && start.IsKind(typeof(TessellatedFace)))
        {
            return true;
        }
        if (canReadTessGeom && start.IsKind(typeof(TessellatedSurfaceSet)))
        {
            return true;
        }
        if (canReadTessGeom && start.IsKind(typeof(TessellatedShell)))
        {
            return true;
        }
        if (canReadTessGeom && start.IsKind(typeof(TessellatedSolid)))
        {
            return true;
        }
        if (canReadTessGeom && start.IsKind(typeof(TessellatedShapeRepresentation)))
        {
            return true;
        }

        //  REPRESENTATION_RELATIONSHIP et consorts : on regarde le contenu ...
        //  On prend WithTransformation ou non ...

        if (start.IsKind(typeof(ShapeRepresentationRelationship)))
        {
            ShapeRepresentationRelationship und = start as ShapeRepresentationRelationship;

            //  On prend son contenu

            return Recognize(und.Rep1()) || Recognize(und.Rep2());
        }

        if (start.IsKind(typeof(StepContextDependentShapeRepresentation)))
        {
            return true;
            //  on fait le pari que, si ce n est pas transferable tel quel,
            //  des CDSR implicitement references le sont ...
            //  Sinon cette entite n aurait pas grand sens ...
        }

        return false;
    }

    /// <summary>recursive method that accesses the root entities and starts the mapping</summary>
    public override Binder Transfer(Transient start, TransientProcess tp,
                                             ProgressRange progress = default)
    {
        // [BEGIN] Get version of preprocessor (to detect I-Deas case) (ssv; 23.11.2010)
        Factors localFactors = new Factors();
        StepModel stepModel = tp.Model() as StepModel;
        if (!stepModel.IsInitializedUnit)
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; the port has no
            // process-wide unit: millimetres (decision 1265)
            double cascadeUnit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(null);
            stepModel.SetLocalLengthUnit(cascadeUnit);
            localFactors.SetCascadeUnit(cascadeUnit);
        }
        localFactors.SetCascadeUnit(stepModel.LocalLengthUnit());
        EntityIterator entIt = stepModel.Header();
        for (entIt.Start(); entIt.More(); entIt.Next())
        {
            FileName fileNameEntity = entIt.Value() as FileName;
            if (fileNameEntity is not null)
            {
                TCollection_HAsciiString ppVersion = fileNameEntity.PreprocessorVersion();
                if (ppVersion is null)
                {
                    continue;
                }
                int iDeasResult = ppVersion.Search("I-DEAS");
                if (iDeasResult != -1)
                {
                    _nmTool.SetIDEASCase(true);
                }
            }
        }
        // [END] Get version of preprocessor (to detect I-Deas case) (ssv; 23.11.2010)
        bool trsfUse = stepModel.InternalParameters.ReadRootTransformation;
        return TransferShape(start, tp, localFactors, true, trsfUse, progress);
    }

    // ============================================================================
    // auxiliary function : ApplyTransformation
    // ============================================================================
    private static void ApplyTransformation(ref Shape shape, in Transform3D trsf)
    {
        if (trsf.Form == Identity)
        {
            return;
        }
        Placement theLoc = new Placement(trsf);
        shape.Move(theLoc);
    }

    private static void ApplyTransformation(ref Compound shape, in Transform3D trsf)
    {
        if (trsf.Form == Identity)
        {
            return;
        }
        Placement theLoc = new Placement(trsf);
        shape.Move(theLoc);
    }

    // ============================================================================
    // auxiliary function : FindContext
    // ============================================================================
    private static StepRepresentation FindContext(Transient start, TransientProcess tp, int level = 10)
    {
        StepRepresentation rep = null;
        EntityGraph graph = tp.Graph();
        EntityIterator subs = graph.Sharings(start);
        for (subs.Start(); subs.More() && rep is null; subs.Next())
        {
            rep = subs.Value() as StepRepresentation;
            if (rep is null && level > 0)
            {
                rep = FindContext(subs.Value(), tp, level - 1);
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
        if (start.IsKind(typeof(StepAdvancedBrepShapeRepresentation)))
        {
            return 2;
        }
        if (start.IsKind(typeof(StepManifoldSurfaceShapeRepresentation)))
        {
            return 3;
        }
        if (start.IsKind(typeof(StepGeometricallyBoundedSurfaceShapeRepresentation)))
        {
            return 4;
        }
        if (start.IsKind(typeof(StepFacetedBrepShapeRepresentation)))
        {
            return 5;
        }
        if (start.IsKind(typeof(StepEdgeBasedWireframeShapeRepresentation)))
        {
            return 6;
        }
        if (start.IsKind(typeof(StepGeometricallyBoundedWireframeShapeRepresentation)))
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
    private static void GetListSDR(ShapeAspect sa, NCollection_HSequence<Transient> listSDR, TransientProcess tp)
    {
        EntityGraph graph = tp.Graph();

        // check whether this ShapeAspect is used in G&DT, and if yes, ignore it
        if (sa.IsKind(typeof(DatumFeature)))
        {
            return;
        }
        EntityIterator subs5 = graph.Sharings(sa);
        for (subs5.Start(); subs5.More(); subs5.Next())
        {
            if (subs5.Value().IsKind(typeof(GeometricTolerance)))
            {
                return;
            }
        }

        // reiterate by referred entities and take all SDRs; note that SDRs that
        // refer to sub-shapes of main SDR will be filtered out further during translation
        subs5 = graph.Sharings(sa);
        for (subs5.Start(); subs5.More(); subs5.Next())
        {
            PropertyDefinition propd = subs5.Value() as PropertyDefinition;
            if (propd is null)
            {
                continue;
            }
            EntityIterator subs6 = graph.Sharings(propd);
            for (subs6.Start(); subs6.More(); subs6.Next())
            {
                StepShapeDefinitionRepresentation sdr = subs6.Value() as StepShapeDefinitionRepresentation;
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
    private static void GetSDR(ProductDefinitionShape pds, NCollection_HSequence<Transient> listSDR,
                               NCollection_HSequence<Transient> listNAUO, NCollection_HSequence<Transient> listSDRAspect,
                               TransientProcess tp)
    {
        StepModel stepModel = tp.Model() as StepModel;

        // Flag indicating preferred shape representation type, to be chosen if
        // several different representations are attached to the same shape
        int delta = 100;
        int ics = (int)stepModel.InternalParameters.ReadShapeRepr;
        int nbSDR0 = listSDR.Length();

        // Iterate by entities referring PDS
        EntityGraph graph = tp.Graph();
        EntityIterator subs4 = graph.Sharings(pds);
        for (subs4.Start(); subs4.More(); subs4.Next())
        {
            // Directly assigned SDR
            StepShapeDefinitionRepresentation sdr = subs4.Value() as StepShapeDefinitionRepresentation;
            if (sdr is not null)
            {
                StepRepresentation rep = sdr.UsedRepresentation();

                if (rep is not null)
                {
                    // if specific representation type is preferred, choose the
                    // representations of the closest type
                    if (ics == 1)
                    { // translate all SDRs
                        listSDR.Append(sdr);
                    }
                    else
                    {
                        int iDiff = Math.Abs(FindShapeReprType(rep) - ics);
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
            ShapeAspect sa = subs4.Value() as ShapeAspect;
            if (sa is not null)
            {
                GetListSDR(sa, listSDRAspect, tp);
                continue;
            }

            // NAUO is used to find sub-assemblies
            NextAssemblyUsageOccurrence nauo = subs4.Value() as NextAssemblyUsageOccurrence;
            if (nauo is not null)
            {
                if (pds.Definition().ProductDefinition() == nauo.RelatingProductDefinition())
                {
                    listNAUO.Append(nauo);
                }
                continue;
            }
        }
    }

    /// <summary>Transfers product definition entity</summary>
    protected ShapeBinder TransferEntity(ProductDefinition pd, TransientProcess tp,
                                                      in Factors localFactors, bool theUseTrsf = false,
                                                      ProgressRange progress = default)
    {
        ShapeBinder shbinder = null;

        Compound cund = new Compound();
        Shape result1 = new Shape();
        BrepBuilder b = new BrepBuilder();
        b.MakeCompound(ref cund);

        // Find subcomponents of assembly (NAUO)
        // and definitions of shape of the current product (SDR)
        NCollection_HSequence<Transient> listSDR = new NCollection_HSequence<Transient>();
        NCollection_HSequence<Transient> listNAUO = new NCollection_HSequence<Transient>();
        NCollection_HSequence<Transient> listSDRAspect = new NCollection_HSequence<Transient>();
        EntityGraph graph = tp.Graph();
        EntityIterator subs3 = graph.Sharings(pd);
        for (subs3.Start(); subs3.More(); subs3.Next())
        {
            // PDS is used to find shape definitions attached to this product
            ProductDefinitionShape pds = subs3.Value() as ProductDefinitionShape;
            if (pds is not null)
            {
                GetSDR(pds, listSDR, listNAUO, listSDRAspect, tp);
                continue;
            }
            // NAUO is used to find sub-assemblies
            NextAssemblyUsageOccurrence nauo1 = subs3.Value() as NextAssemblyUsageOccurrence;
            if (nauo1 is not null)
            {
                if (pd == nauo1.RelatingProductDefinition())
                {
                    listNAUO.Append(nauo1);
                }
                continue;
            }
        }

        StepModel stepModel = tp.Model() as StepModel;

        // Flag indicating whether SDRs associated with the product`s main SDR
        // by SRRs (which correspond to hybrid model representation in AP203 since 1998)
        // should be taken into account
        bool readSRR = stepModel.InternalParameters.ReadRelationship;

        bool readConstructiveGeomRR = stepModel.InternalParameters.ReadConstrRelation;
        // Flag indicating whether SDRs associated with the product`s main SDR
        // by SAs (which correspond to hybrid model representation in AP203 before 1998)
        // should be taken into account
        int readSA = stepModel.InternalParameters.ReadShapeAspect ? 1 : 0;
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
        int readAssembly = (int)stepModel.InternalParameters.ReadAssemblyLevel;
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
        using ProgressScope ps = new ProgressScope(progress, "Part", nbEnt);
        int nbComponents = 0;

        // translate sub-assemblies
        for (int nbNauo = 1; nbNauo <= listNAUO.Length() && ps.More(); nbNauo++)
        {
            NextAssemblyUsageOccurrence nauo = listNAUO.Value(nbNauo) as NextAssemblyUsageOccurrence;

            // TRANSLOG
            if (tp.TraceLevel() > 1)
            {
                using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
                sout.Append(" -- Actor : Ent.n0 ").Append(tp.Model().Number(pd)).Append(" -> Shared Ent.no")
                    .Append(tp.Model().Number(nauo)).Append('\n');
            }
            Binder binder;
            ProgressRange range = ps.Next();
            if (!tp.IsBound(nauo))
            {
                binder = TransferEntity(nauo, tp, localFactors, range);
            }
            else
            {
                binder = tp.Find(nauo);
            }

            Shape theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull)
            {
                result1 = theResult;
                // [BEGIN] ssv: OCCT#22436: extra compound in NMSSR case
                if (_nmDetected && result1.Type == ShapeType.Compound)
                {
                    Iterator it = new Iterator(result1);
                    for (; it.More(); it.Next())
                    {
                        Shape subShape = it.Value();
                        b.Add(cund, subShape);
                    }
                }
                else
                {
                    b.Add(cund, theResult);
                }
                // [END] ssv: OCCT#22436: extra compound in NMSSR case
                nbComponents++;
            }
        }

        // translate shapes assigned directly
        for (int i = 1; i <= listSDR.Length() && ps.More(); i++)
        {
            StepShapeDefinitionRepresentation sdr = listSDR.Value(i) as StepShapeDefinitionRepresentation;
            StepShapeRepresentation rep = sdr.UsedRepresentation() as StepShapeRepresentation;
            if (rep is null)
            {
                continue;
            }

            using ProgressScope ps1 = new ProgressScope(ps.Next(), null, 2);

            // translate SDR representation
            bool isBound = true;
            // SKL for bug 29068: transformation need to applied only for "main"
            // ShapeDefinitionRepresentation. Part of listSDR given by ShapeAspect must be ignored because
            // all needed transformations will be applied during its transfer. Therefore flag for using Trsf
            // must be updated.
            bool useTrsf = theUseTrsf && (i <= nbNotAspect);
            Binder binder = tp.Find(rep);
            if (binder is null)
            {
                binder = TransferEntity(rep, tp, localFactors, ref isBound, useTrsf, ps1.Next());
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
            Shape theResult = new Shape();
            if (binder is not null)
            {
                theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
                if (!theResult.IsNull)
                {
                    result1 = theResult;
                    // [BEGIN] ssv: OCCT#22436: extra compound in NMSSR case
                    if (_nmDetected && result1.Type == ShapeType.Compound)
                    {
                        Iterator it = new Iterator(result1);
                        for (; it.More(); it.Next())
                        {
                            Shape subShape = it.Value();
                            b.Add(cund, subShape);
                        }
                    }
                    else
                    {
                        b.Add(cund, theResult);
                    }
                    // [END] ssv: OCCT#22436: extra compound in NMSSR case
                }
            }

            // translate possible shapes related by SRRs, which corresponds to
            // way of writing hybrid models in AP203 since 1998, and AP209
            // Note that both AP203 and AP209 allow main representation to be non-empty
            if (readSRR && /*theResult.IsNull() &&*/ i <= nbNotAspect)
            {
                Shape newResult = TransferRelatedSRR(tp, rep, useTrsf, readConstructiveGeomRR, localFactors, cund, ps1);
                if (!newResult.IsNull)
                {
                    result1 = newResult;
                }
            }
        }

        // make a warning if both own shape and sub-assemblies are present
        if ((cund.ChildCount - nbComponents) > 0 && nbComponents > 0)
        {
            tp.AddWarning(pd, "Product has both sub-assemblies and directly assigned shape");
        }

        // if only single shape is read, add it as it is; otherwise add compound
        if ((cund.ChildCount - nbComponents) == 1 && nbComponents == 0)
        {
            shbinder = new ShapeBinder(result1);
        }
        else
        {
            shbinder = new ShapeBinder(cund);
        }

        // TP->Unbind( PD ); //:j3: unbind start (let it be bound by TransferProcess)
        tp.Bind(pd, shbinder);
        return shbinder;
    }

    /// <summary>Transfers next assembly usage occurrence entity</summary>
    /// <exception cref="InvalidDataException">Kerf: the assemblies are nested too deep (decision 1854): each level takes
    /// about 1 kB of stack here, and a stack overflow ends the process.</exception>
    protected ShapeBinder TransferEntity(NextAssemblyUsageOccurrence nauo, TransientProcess tp,
                                                      in Factors localFactors, ProgressRange progress = default)
    {
        if (_assemblyDepth >= global::Kerf.Exchange.ModelNode.MaxNesting)
        {
            throw new InvalidDataException("Its assemblies are nested more than " + global::Kerf.Exchange.ModelNode.MaxNesting + " deep.");
        }

        _assemblyDepth++;
        try
        {
            return TransferAssemblyUsage(nauo, tp, localFactors, progress);
        }
        finally
        {
            _assemblyDepth--;
        }
    }

    // Kerf: how many assembly levels the transfer is inside
    private int _assemblyDepth;

    private ShapeBinder TransferAssemblyUsage(NextAssemblyUsageOccurrence nauo, TransientProcess tp,
                                              in Factors localFactors, ProgressRange progress)
    {
        ShapeBinder shbinder = null;
        ProductDefinition pd = null;
        EntityGraph graph = tp.Graph();
        Transform3D trsf = new Transform3D();
        bool iatrsf = false, srrReversed = false, isDepend = false;
        ShapeRepresentationRelationship srr = null;
        EntityIterator subs1 = graph.Sharings(nauo);
        for (subs1.Start(); subs1.More(); subs1.Next())
        {
            ProductDefinitionShape pds = subs1.Value() as ProductDefinitionShape;
            if (pds is null)
            {
                continue;
            }
            EntityIterator subs2 = graph.Sharings(pds);
            for (subs2.Start(); subs2.More(); subs2.Next())
            {
                StepContextDependentShapeRepresentation cdsr = subs2.Value() as StepContextDependentShapeRepresentation;
                if (cdsr is null)
                {
                    continue;
                }
                isDepend = true;
                RepresentationRelationship rr = cdsr.RepresentationRelation();
                if (rr is null)
                {
                    continue;
                }
                srrReversed = ConstructAssembly.CheckSRRReversesNAUO(graph, cdsr);
                StepRepresentation rep = (srrReversed ? rr.Rep2() : rr.Rep1());
                if (rep is null)
                {
                    continue;
                }
                iatrsf = ComputeSRRWT(rr, tp, ref trsf, localFactors);
                // find real ProductDefinition used rep
                EntityIterator subs3 = tp.Graph().Sharings(rep);
                for (subs3.Start(); subs3.More(); subs3.Next())
                {
                    Transient subs3Val = subs3.Value();
                    if (subs3Val is StepShapeDefinitionRepresentation sdr)
                    {
                        ProductDefinitionShape pds1 = sdr.Definition().PropertyDefinition() as ProductDefinitionShape;
                        if (pds1 is null)
                        {
                            continue;
                        }
                        EntityIterator subs4 = graph.Shareds(pds1);
                        for (subs4.Start(); pd is null && subs4.More(); subs4.Next())
                        {
                            pd = subs4.Value() as ProductDefinition;
                        }
                    }
                    else if (subs3Val.IsKind(typeof(ShapeRepresentationRelationship)))
                    {
                        // NB: C cast is used instead of DownCast() to improve performance on some cases.
                        srr = (ShapeRepresentationRelationship)subs3Val;
                    }
                }
            }
        }

        Binder binder;
        Shape theResult = new Shape();
        shbinder = null;

        if (isDepend)
        {
            using ProgressScope ps = new ProgressScope(progress, null, 2);

            if (pd is not null)
            {
                binder = tp.Find(pd);
                if (binder is null)
                {
                    binder = TransferEntity(pd, tp, localFactors, false, ps.Next());
                }
                theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
                if (!theResult.IsNull)
                {
                    if (iatrsf)
                    {
                        if (srrReversed)
                        {
                            ApplyTransformation(ref theResult, trsf.Inverted());
                        }
                        else
                        {
                            ApplyTransformation(ref theResult, trsf);
                        }
                    }
                    shbinder = new ShapeBinder(theResult);
                }
            }

            if (theResult.IsNull && srr is not null)
            {
                binder = tp.Find(srr);
                if (binder is null)
                {
                    binder = TransferEntity(srr, tp, localFactors, 0, false, ps.Next());
                    theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
                    if (!theResult.IsNull)
                    {
                        shbinder = new ShapeBinder(theResult);
                    }
                }
            }
        }
        tp.Bind(nauo, shbinder);
        return shbinder;
    }

    /// <summary>Transfers shape representation entity</summary>
    protected ShapeBinder TransferEntity(StepShapeRepresentation sr, TransientProcess tp,
                                                      in Factors localFactors, ref bool isBound, bool useTrsf = false,
                                                      ProgressRange progress = default)
    {
        _nmDetected = false;
        ShapeBinder shbinder = null;
        StepModel stepModel = tp.Model() as StepModel;
        if (!Recognize(sr))
        {
            return shbinder;
        }
        isBound = false;
        int nb = sr.NbItems();
        int nbTPitems = tp.MappedCount;
        // TRANSLOG
        if (tp.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
            sout.Append(" -- Actor : case  ShapeRepr. NbItems=").Append(nb).Append('\n');
        }

        // Compute unit conversion factors and geometric Accuracy
        StepRepresentation oldSRContext = _srContext; //: S4136
        Factors aLocalFactors = localFactors;
        PrepareUnits(sr, tp, ref aLocalFactors);

        BrepBuilder b = new BrepBuilder();
        Compound comp = new Compound();
        b.MakeCompound(ref comp);
        Shape oneResult = new Shape();
        int nsh = 0;

        // [BEGIN] Proceed with non-manifold topology (ssv; 12.11.2010)
        bool isNMMode = stepModel.InternalParameters.ReadNonmanifold;
        bool isManifold = true;
        if (isNMMode && sr.IsKind(typeof(StepNonManifoldSurfaceShapeRepresentation)))
        {
            isManifold = false;
            _nmDetected = true;
        }
        // Special processing for I-DEAS STP case (ssv; 15.11.2010)
        else
        {
            bool isIDeasMode = stepModel.InternalParameters.ReadIdeas;
            if (isNMMode && _nmTool.IsIDEASCase && isIDeasMode)
            {
                isManifold = false;
                _nmDetected = true;
            }
        }
        _nmTool.CleanUp();
        // Don't use NM tool in manifold cases (ssv; 24.11.2010)
        _nmTool.SetActive(!isManifold && isNMMode);
        // [END] Proceed with non-manifold topology (ssv; 12.11.2010)

        Transform3D trsf = new Transform3D();
        using ProgressScope psRoot = new ProgressScope(progress, "Sub-assembly", isManifold ? 1 : 2);
        using ProgressScope ps = new ProgressScope(psRoot.Next(), "Transfer", nb);
        IndexedSet<Shape> compoundedShapes = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (int i = 1; i <= nb && ps.More(); i++)
        {
            ProgressRange range = ps.Next();
            // TRANSLOG
            if (tp.TraceLevel() > 2)
            {
                using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
                sout.Append(" -- Actor, shape_representation.item n0. ").Append(i).Append('\n');
            }
            RepresentationItem anitem = sr.ItemsValue(i);
            if (anitem is null)
            {
                continue;
            }
            if (useTrsf)
            {
                if (anitem.IsKind(typeof(StepAxis2Placement3d)))
                {
                    EntityGraph graph = tp.Graph();
                    EntityIterator subs3 = graph.Sharings(anitem);
                    for (subs3.Start(); subs3.More(); subs3.Next())
                    {
                        ItemDefinedTransformation idt = subs3.Value() as ItemDefinedTransformation;
                        if (idt is not null)
                        {
                            // current Axis2Placement is used for related ShapeRepresentation => ignore it
                            break;
                        }
                    }
                    if (!subs3.More())
                    {
                        StepAxis2Placement3d cs = anitem as StepAxis2Placement3d;
                        Axis2Placement targAP = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(cs, aLocalFactors);
                        if (targAP is not null)
                        {
                            CoordinateSystem ax3Orig = new CoordinateSystem(new Point3D(0.0, 0.0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
                            CoordinateSystem ax3Targ = new CoordinateSystem(targAP.Ax2());
                            if (ax3Targ.Location.SquareDistanceTo(ax3Orig.Location) < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                                && ax3Targ.Direction.IsEqual(ax3Orig.Direction, global::Kerf.Engine.Foundation.Precision.Angular())
                                && ax3Targ.XDirection.IsEqual(ax3Orig.XDirection, global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                continue;
                            }
                            trsf = trsf.WithTransformation(ax3Targ, ax3Orig);
                        }
                        continue;
                    }
                }
            }
            Binder binder;
            if (!tp.IsBound(anitem))
            {
                binder = TransferShape(anitem, tp, aLocalFactors, isManifold, false, range);
            }
            else
            {
                isBound = true;
                binder = tp.Find(anitem);
            }
            Shape theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull)
            {
                oneResult = theResult;
                if (!compoundedShapes.Contains(theResult))
                {
                    compoundedShapes.Add(theResult);
                    Exploration.MapShapes(theResult, compoundedShapes, false, false);
                    b.Add(comp, theResult);
                    nsh++;
                }
            }
        }

        // [BEGIN] Proceed with non-manifold topology (ssv; 12.11.2010)
        if (!isManifold)
        {
            using ProgressScope ps1 = new ProgressScope(psRoot.Next(), "Process", 1);

            // Set tolerances for shape processing.
            // These parameters are calculated inside STEPControl_ActorRead::Transfer() and cannot be set
            // from outside.
            Dictionary<string, string> parameters = GetShapeFixParameters();
            ShapeProcessor.SetParameter("FixShape.Tolerance3d", _precision, true, parameters);
            ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", _maxTol, true, parameters);
            ShapeProcessor.SetParameter("FixShape.NonManifold", "1", true, parameters);
            ShapeProcessor shapeProcessor = new ShapeProcessor(parameters);
            Shape fixedResult = shapeProcessor.InternalProcessShape(comp, GetProcessingFlags().Item1, ps1.Next());
            HealedFaceCheck.Check(comp, fixedResult, shapeProcessor.GetContext(), tp); // Kerf: decision 33502
            shapeProcessor.InternalMergeTransferInfo(tp, nbTPitems);

            if (fixedResult.Type == ShapeType.Compound)
            {
                comp = ShapeCasts.Compound(fixedResult);
            }
            else
            {
                comp.Nullify();
                b.MakeCompound(ref comp);
                b.Add(comp, fixedResult);
            }

            BrepBuilder brepBuilder = new BrepBuilder();

            // [BEGIN] Try to close OPEN Shells in I-DEAS case (ssv; 17.11.2010)
            if (_nmTool.IsIDEASCase)
            {
                // ==========================================================
                //  For each Shell (always OPEN in I-DEAS case) find all
                //  the possibilities to close it with adjacent non-manifold
                //  Shells which are stored separately in I-DEAS-like STP
                // ==========================================================

                IndexedMap<Shape, NCollection_List<Shape>> shellClosingsMap =
                    new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                // Find possible closings for each shell
                ComputeIDEASClosings(comp, shellClosingsMap);

                // Make compound to store closed Shells
                Compound compWithClosings = new Compound();
                brepBuilder.MakeCompound(ref compWithClosings);

                // Attempt to close Shells one-by-one
                for (int i = 0; i < shellClosingsMap.Count; i++)
                {
                    Shell adjustedShell = CloseIDEASShell(ShapeCasts.Shell(shellClosingsMap.KeyAt(i)),
                                                                 shellClosingsMap.ValueAt(i));
                    brepBuilder.Add(compWithClosings, adjustedShell);
                }
                // Put not suspected open Shells as they are (updated 23.11.2010)
                Explorer allShellsExp = new Explorer(comp, ShapeType.Shell);
                for (; allShellsExp.More(); allShellsExp.Next())
                {
                    Shape currentShell = allShellsExp.Current();
                    if (!_nmTool.IsPureNMShell(currentShell) && !shellClosingsMap.ContainsKey(currentShell))
                    {
                        brepBuilder.Add(compWithClosings, currentShell);
                    }
                }
                comp = compWithClosings;
            }
            // [END] Try to close OPEN Shells in I-DEAS case (ssv; 17.11.2010)

            // [BEGIN] Reconstruct Solids from Closed Shells (ssv; 15.11.2010)
            Compound reconstComp = new Compound();
            brepBuilder.MakeCompound(ref reconstComp);
            Explorer exp = new Explorer(comp, ShapeType.Shell);
            for (; exp.More(); exp.Next())
            {
                Shape subShape = exp.Current();
                if (subShape.Type == ShapeType.Shell && subShape.ClosedFlag)
                {
                    Solid nextSolid = new Solid();
                    brepBuilder.MakeSolid(ref nextSolid);
                    brepBuilder.Add(nextSolid, subShape);
                    brepBuilder.Add(reconstComp, nextSolid);
                }
                else if (subShape.Type == ShapeType.Shell)
                {
                    brepBuilder.Add(reconstComp, subShape);
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
            if (trsf.Form != Identity)
            {
                Placement loc = new Placement(trsf);
                oneResult.Move(loc);
            }
            shbinder = new ShapeBinder(oneResult);
        }
        else
        {
            if (trsf.Form != Identity)
            {
                Placement loc = new Placement(trsf);
                comp.Move(loc);
            }
            shbinder = new ShapeBinder(comp);
        }

        PrepareUnits(oldSRContext, tp, ref aLocalFactors); //: S4136
        tp.Bind(sr, shbinder);

        return shbinder;
    }

    /// <summary>Transfers context dependent shape representation entity</summary>
    protected ShapeBinder TransferEntity(StepContextDependentShapeRepresentation cdsr, TransientProcess tp,
                                                      in Factors localFactors, ProgressRange progress = default)
    {
        ShapeBinder shbinder = null;
        //: j2: treat SRRs here in order to compare them with NAUO
        ShapeRepresentationRelationship srr = cdsr.RepresentationRelation();
        if (srr is null)
        {
            return shbinder;
        }

        bool srrReversed = ConstructAssembly.CheckSRRReversesNAUO(tp.Graph(), cdsr);
        StepRepresentation rep1 = (srrReversed ? srr.Rep2() : srr.Rep1());
        StepShapeRepresentation rep = rep1 as StepShapeRepresentation;

        if (srrReversed)
        {
            tp.AddWarning(srr, "SRR reverses relation defined by NAUO; NAUO definition is taken");
        }

        Shape theResult;

        Transform3D trsf = new Transform3D();
        bool iatrsf = ComputeSRRWT(srr, tp, ref trsf, localFactors);

        Binder binder;
        bool isBound = false;
        if (!tp.IsBound(rep))
        {
            binder = TransferEntity(rep, tp, localFactors, ref isBound, false, progress);
        }
        else
        {
            binder = tp.Find(rep);
        }
        theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);

        if (!theResult.IsNull)
        {
            if (iatrsf)
            {
                if (srrReversed)
                {
                    ApplyTransformation(ref theResult, trsf.Inverted());
                }
                else
                {
                    ApplyTransformation(ref theResult, trsf);
                }
            }
            shbinder = new ShapeBinder(theResult);
        }
        else
        {
            shbinder = null;
        }
        tp.Bind(cdsr, shbinder);
        return shbinder;
    }

    /// <summary>Transfers shape representation relationship entity</summary>
    protected ShapeBinder TransferEntity(ShapeRepresentationRelationship und, TransientProcess tp,
                                                      in Factors localFactors, int nbrep = 0, bool useTrsf = false,
                                                      ProgressRange progress = default)
    {
        //  REPRESENTATION_RELATIONSHIP et la famille
        ShapeBinder shbinder = null;

        if (und is null)
        {
            return shbinder;
        }

        //  On prend son contenu : Rep1 ou Rep2 , that is the question
        //    on prend les 2. Mais quoi faire des axes

        Compound cund = new Compound();
        Shape oneResult = new Shape();
        BrepBuilder b = new BrepBuilder();
        b.MakeCompound(ref cund);
        int nsh = 0;

        Transform3D trsf = new Transform3D();
        bool iatrsf = ComputeSRRWT(und, tp, ref trsf, localFactors);

        //    Transfert : que faut-il prendre au juste ?
        using ProgressScope ps = new ProgressScope(progress, null, 2);
        for (int i = 1; i <= 2 && ps.More(); i++)
        {
            ProgressRange range = ps.Next();
            if (nbrep != 0 && nbrep != i)
            {
                continue;
            }
            StepRepresentation anitemt = null;
            if (i == 1)
            {
                anitemt = und.Rep1();
            }
            if (i == 2)
            {
                anitemt = und.Rep2();
            }
            StepShapeRepresentation anitem = anitemt as StepShapeRepresentation;
            Binder binder;
            bool isBound = false;
            if (!tp.IsBound(anitem))
            {
                binder = TransferEntity(anitem, tp, localFactors, ref isBound, useTrsf, range);
            }
            else
            {
                binder = tp.Find(anitem);
            }
            Shape theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull)
            {
                oneResult = theResult;
                b.Add(cund, theResult);
                nsh++;
            }
        }

        //   La Transformation : selon les cas

        //    Appliquer la transformation

        if (iatrsf)
        {
            if (nsh == 1)
            {
                ApplyTransformation(ref oneResult, trsf);
            }
            else if (nsh > 1)
            {
                ApplyTransformation(ref cund, trsf);
            }
        }
        if (nsh == 0)
        {
            shbinder = null;
        }
        else if (nsh == 1)
        {
            shbinder = new ShapeBinder(oneResult);
        }
        else
        {
            shbinder = new ShapeBinder(cund);
        }
        tp.Bind(und, shbinder);
        return shbinder;
    }

    private ShapeBinder TransferEntity(ConstructiveGeometryRepresentationRelationship cgrr,
                                                    TransientProcess tp, in Factors localFactors)
    {
        ShapeBinder shbinder = null;
        if (cgrr is null)
        {
            return shbinder;
        }
        bool resetUnits = false;
        StepRepresentation oldSRContext = _srContext;
        Factors aLocalFactors = localFactors;
        Compound comp = new Compound();
        BrepBuilder b = new BrepBuilder();
        b.MakeCompound(ref comp);
        for (int i = 1; i <= 2; i++)
        {
            ConstructiveGeometryRepresentation cRepr =
                (i == 1 ? cgrr.Rep1() : cgrr.Rep2()) as ConstructiveGeometryRepresentation;
            if (cRepr is null)
            {
                continue;
            }
            if (_srContext is null || cRepr.ContextOfItems() != _srContext.ContextOfItems())
            {
                PrepareUnits(cRepr, tp, ref aLocalFactors);
                resetUnits = true;
            }
            int j = 1;
            for (; j <= cRepr.NbItems(); j++)
            {
                RepresentationItem item = cRepr.ItemsValue(j);
                StepAxis2Placement3d stepAxis = item as StepAxis2Placement3d;
                if (stepAxis is not null)
                {
                    Axis2Placement axis = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(stepAxis, aLocalFactors);
                    if (axis is null)
                    {
                        continue;
                    }
                    PlaneSurface plane = new PlaneSurface(new CoordinateSystem(axis.Ax2()));
                    Face planeFace = new Face();
                    b.MakeFace(ref planeFace, plane, global::Kerf.Engine.Foundation.Precision.Confusion());
                    ShapeBinder axisbinder = new ShapeBinder(planeFace);
                    tp.Bind(stepAxis, axisbinder);
                    b.Add(comp, planeFace);
                }
            }
        }
        shbinder = new ShapeBinder(comp);
        _srContext = oldSRContext;
        if (oldSRContext is null || resetUnits)
        {
            PrepareUnits(oldSRContext, tp, ref aLocalFactors);
        }

        tp.Bind(cgrr, shbinder);

        return shbinder;
    }

    /// <summary>Transfers MechanicalDesignAndDraughtingRelationship entity</summary>
    private ShapeBinder TransferEntity(MechanicalDesignAndDraughtingRelationship mdadr,
                                                    TransientProcess tp, in Factors localFactors,
                                                    ProgressRange progress)
    {
        ShapeBinder shBinder = null;
        if (mdadr is null)
        {
            return shBinder;
        }

        bool resetUnits = false;
        StepRepresentation oldSRContext = _srContext;
        Factors aLocalFactors = localFactors;
        Compound comp = new Compound();
        BrepBuilder builder = new BrepBuilder();
        builder.MakeCompound(ref comp);

        using ProgressScope ps = new ProgressScope(progress, null, 2);
        for (int index = 1; index <= 2; index++)
        {
            ProgressRange range = ps.Next();
            StepRepresentation repr = (index == 1) ? mdadr.Rep1() : mdadr.Rep2();
            if (repr is null)
            {
                continue;
            }
            if (_srContext is null || repr.ContextOfItems() != _srContext.ContextOfItems())
            {
                PrepareUnits(repr, tp, ref aLocalFactors);
                resetUnits = true;
            }
            Binder binder = null;
            if (repr.IsKind(typeof(StepShapeRepresentation)))
            {
                StepShapeRepresentation shapeRepr = repr as StepShapeRepresentation;
                bool isBound = false;
                if (!tp.IsBound(shapeRepr))
                {
                    binder = TransferEntity(shapeRepr, tp, localFactors, ref isBound, false, range);
                }
                else
                {
                    binder = tp.Find(shapeRepr);
                }
            }
            Shape result = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
            if (!result.IsNull)
            {
                builder.Add(comp, result);
            }
        }

        shBinder = new ShapeBinder(comp);
        _srContext = oldSRContext;
        if (oldSRContext is null || resetUnits)
        {
            PrepareUnits(oldSRContext, tp, ref aLocalFactors);
        }

        tp.Bind(mdadr, shBinder);

        return shBinder;
    }

    private static bool IsNeedRepresentation(ShapeAspect sa, StepRepresentation repA, TransientProcess tp)
    {
        bool isSDRaspect = true;
        ProductDefinitionShape pdsa = sa.OfShape();
        EntityGraph graph = tp.Graph();
        EntityIterator subs7 = graph.Sharings(pdsa);
        for (subs7.Start(); pdsa is not null && subs7.More(); subs7.Next())
        {
            StepShapeDefinitionRepresentation sdrA = subs7.Value() as StepShapeDefinitionRepresentation;
            if (sdrA is null)
            {
                continue;
            }
            // abv 23 Feb 00: use iterator to take into account AP203 hybrid models
            EntityIterator subs8 = graph.Shareds(sdrA);
            for (subs8.Start(); subs8.More(); subs8.Next())
            {
                StepRepresentation rep2 = subs8.Value() as StepRepresentation;
                if (rep2 is null)
                {
                    ShapeRepresentationRelationship srr = subs8.Value() as ShapeRepresentationRelationship;
                    if (srr is null)
                    {
                        continue;
                    }
                    rep2 = (sdrA.UsedRepresentation() == srr.Rep1() ? srr.Rep2() : srr.Rep1());
                }
                int i, j; // svv Jan 11 2000 : porting on DEC
                for (i = 1; i <= repA.NbItems(); i++)
                {
                    RepresentationItem it = repA.ItemsValue(i);
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
                    isSDRaspect = false;
                }
            }
        }
        return isSDRaspect;
    }

    /// <summary>
    /// Translates file by old way when CDSR are roots. Acts only if "read.step.product_mode" is equal Off.
    /// </summary>
    protected ShapeBinder OldWay(Transient start, TransientProcess tp, ProgressRange progress)
    {
        EntityGraph graph = tp.Graph();
        ShapeBinder shbinder = null;
        StepShapeDefinitionRepresentation sdr = start as StepShapeDefinitionRepresentation;
        StepRepresentation rep = sdr.UsedRepresentation();

        // abv 7 Oct 99: TRJ2: skip SDRs used only for defining SHAPE_ASPECT (validation properties)
        // BUT ONLY if they have representation duplicated with other SDR,
        // (SHAPE_ASPECT also used by some systems to define geometry)
        PropertyDefinition pd = sdr.Definition().PropertyDefinition();
        if (pd is not null)
        {
            ShapeAspect sa = pd.Definition().ShapeAspect();
            if (sa is not null)
            {
                if (!IsNeedRepresentation(sa, rep, tp))
                {
                    return shbinder;
                }
            }
        }

        using ProgressScope psRoot = new ProgressScope(progress, null, 2);

        // TRANSLOG
        if (tp.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
            sout.Append(" -- Actor : case  shape_definition_representation.").Append('\n');
        }
        Binder binder = tp.Find(rep);
        {
            ProgressRange range = psRoot.Next();
            if (binder is null)
            {
                binder = tp.Transferring(rep, range);
            }
        }
        if (psRoot.UserBreak())
        {
            return shbinder;
        }
        //: j2    if (!binder.IsNull()) return binder;

        //    SDR designant des CDSR (lien implicite, via la UsedRepr)

        Compound cund = new Compound();
        Shape oneResult = new Shape();
        BrepBuilder b = new BrepBuilder();
        b.MakeCompound(ref cund);
        int nsh = 0;

        //: j2
        shbinder = new ShapeBinder();
        tp.Bind(start, shbinder); //:j3 abv 23 Oct 98: rp1sd.stp: bind something to protect against loops
        if (binder is not null)
        {
            Shape theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull)
            {
                oneResult = theResult;
                b.Add(cund, theResult);
                nsh++;
                shbinder.SetResult(theResult);
            }
        }

        // process subcomponents of assembly (CDSR) and hybrid models (SRR)
        EntityIterator subs = graph.Shareds(start);
        Type tCDSR = typeof(StepContextDependentShapeRepresentation);
        Type tSRR = typeof(ShapeRepresentationRelationship);
        int nbitem = 0;
        for (subs.Start(); subs.More(); subs.Next())
        {
            nbitem++;
        }
        using ProgressScope ps = new ProgressScope(psRoot.Next(), "Sub", nbitem);
        for (subs.Start(); subs.More() && ps.More(); subs.Next())
        {
            ProgressRange range = ps.Next();
            Transient anitem = subs.Value();
            if (anitem.DynamicType() != tCDSR && anitem.DynamicType() != tSRR)
            {
                continue;
            }
            // TRANSLOG
            if (tp.TraceLevel() > 1)
            {
                using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
                sout.Append(" -- Actor : Ent.n0 ").Append(tp.Model().Number(start)).Append(" -> Shared Ent.no")
                    .Append(tp.Model().Number(anitem)).Append('\n');
            }

            if (!tp.IsBound(anitem))
            {
                binder = tp.Transferring(anitem, range);
            }
            else
            {
                binder = tp.Find(anitem);
            }
            Shape theResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
            if (!theResult.IsNull)
            {
                oneResult = theResult;
                b.Add(cund, theResult);
                nsh += 2; // abv 11.10.00: 2 instead of 1 in order to keep assembly structure
            }
        }

        if (nsh == 0)
        {
            shbinder = new ShapeBinder(cund); // shbinder.Nullify();
        }
        else if (nsh == 1)
        {
            shbinder = new ShapeBinder(oneResult);
        }
        else
        {
            shbinder = new ShapeBinder(cund);
        }
        tp.Unbind(start); //: j3: unbind start (let it be bound by TransferProcess)
        return shbinder;
    }

    /// <summary>Transfers geometric representation item entity such as ManifoldSolidBRep ,...etc</summary>
    protected ShapeBinder TransferEntity(StepGeometricRepresentationItem start, TransientProcess tp,
                                                      in Factors localFactors, bool isManifold,
                                                      ProgressRange progress)
    {
        ShapeBinder shbinder = null;
        bool found = false;
        ImportBuilder myShapeBuilder = new ImportBuilder();
        Shape mappedShape = new Shape();
        int nbTPitems = tp.MappedCount;
        // TRANSLOG (OCCT's chronometer, shown at trace level 3, left out)
        if (tp.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
            sout.Append("Begin transfer STEP -> CASCADE, Type ").Append(start.DynamicTypeName()).Append('\n');
        }
        StepModel stepModel = tp.Model() as StepModel;
        //: S4136
        Factors aLocalFactors = localFactors;
        StepRepresentation oldSRContext = _srContext;
        if (_srContext is null)
        { // if no context, try to find it (ex: r0701_ug.stp #4790)
            StepRepresentation context = FindContext(start, tp);
            if (context is null)
            {
                tp.AddWarning(start, "Entity with no unit context; default units taken");
                ResetUnits(stepModel, ref aLocalFactors);
            }
            else
            {
                PrepareUnits(context, tp, ref aLocalFactors);
            }
        }
        myShapeBuilder.SetPrecision(_precision);
        myShapeBuilder.SetMaxTol(_maxTol);

        // Start progress scope (no need to check if progress exists -- it is safe)
        using ProgressScope ps = new ProgressScope(progress, "Transfer stage", isManifold ? 2 : 1);
        bool readTessellatedWhenNoBRepOnly = (stepModel.InternalParameters.ReadTessellated == Parameters.RWMode_Tessellated.RWMode_Tessellated_OnNoBRep);
        bool hasGeom = true;
        try
        {
            ProgressRange range = ps.Next();
            if (start.IsKind(typeof(StepFacetedBrep)))
            {
                myShapeBuilder.Init(start as StepFacetedBrep, tp, aLocalFactors, range);
                found = true;
            }
            else if (start.IsKind(typeof(StepBrepWithVoids)))
            {
                myShapeBuilder.Init(start as StepBrepWithVoids, tp, aLocalFactors, range);
                found = true;
            }
            // Kerf: before the manifold solid B-rep it is one of: OCCT tests it after, so it read the outer shell alone
            // and lost every void (decision 1963)
            else if (start.IsKind(typeof(StepFacetedBrepAndBrepWithVoids)))
            {
                myShapeBuilder.Init(start as StepFacetedBrepAndBrepWithVoids, tp, aLocalFactors, range);
                found = true;
            }
            else if (start.IsKind(typeof(StepManifoldSolidBrep)))
            {
                myShapeBuilder.Init(start as StepManifoldSolidBrep, tp, aLocalFactors, range);
                found = true;
            }
            else if (start.IsKind(typeof(StepShellBasedSurfaceModel)))
            {
                myShapeBuilder.Init(start as StepShellBasedSurfaceModel, tp, _nmTool, aLocalFactors, range);
                found = true;
            }
            else if (start.IsKind(typeof(StepGeometricSet)))
            {
                myShapeBuilder.Init(start as StepGeometricSet, tp, aLocalFactors, this, isManifold, range);
                found = true;
            }
            else if (start.IsKind(typeof(StepEdgeBasedWireframeModel)))
            {
                myShapeBuilder.Init(start as StepEdgeBasedWireframeModel, tp, aLocalFactors);
                found = true;
            }
            else if (start.IsKind(typeof(StepFaceBasedSurfaceModel)))
            {
                myShapeBuilder.Init(start as StepFaceBasedSurfaceModel, tp, aLocalFactors);
                found = true;
            }
            // TODO: Normally, StepVisual_Tessellated* entities should be processed after
            //       StepShape_* entities in order to resolve links to BRep topological objects.
            //       Currently it is not guaranteed and might require changes in the processing order.
            else if (start.IsKind(typeof(TessellatedSolid)))
            {
                myShapeBuilder.Init(start as TessellatedSolid, tp, readTessellatedWhenNoBRepOnly, ref hasGeom, aLocalFactors, range);
                found = true;
            }
            else if (start.IsKind(typeof(TessellatedShell)))
            {
                myShapeBuilder.Init(start as TessellatedShell, tp, readTessellatedWhenNoBRepOnly, ref hasGeom, aLocalFactors, range);
                found = true;
            }
            else if (start.IsKind(typeof(TessellatedFace)))
            {
                myShapeBuilder.Init(start as TessellatedFace, tp, readTessellatedWhenNoBRepOnly, ref hasGeom, aLocalFactors);
                found = true;
            }
            else if (start.IsKind(typeof(TessellatedSurfaceSet)))
            {
                myShapeBuilder.Init(start as TessellatedSurfaceSet, tp, ref hasGeom, aLocalFactors);
                found = true;
            }
        }
        catch (Exception exception) when (Refusal.IsFailure(exception) || ImportRoot.CaughtBySignals(exception))
        {
            tp.AddFail(start, "Exception is raised. Entity was not translated.");
            tp.Bind(start, shbinder);
            return shbinder;
        }

        if (ps.UserBreak())
        {
            return shbinder;
        }

        if (found && myShapeBuilder.IsDone)
        {
            mappedShape = myShapeBuilder.InternalValue();
            // Apply ShapeFix (on manifold shapes only. Non-manifold topology is processed separately:
            // ssv; 13.11.2010)
            if (isManifold && hasGeom)
            {
                // Set tolerances for shape processing.
                // These parameters are calculated inside STEPControl_ActorRead::Transfer() and cannot be set
                // from outside.
                Dictionary<string, string> parameters = GetShapeFixParameters();
                ShapeProcessor.SetParameter("FixShape.Tolerance3d", _precision, true, parameters);
                ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", _maxTol, true, parameters);
                ShapeProcessor shapeProcessor = new ShapeProcessor(parameters);
                Shape unhealed = mappedShape;
                mappedShape = shapeProcessor.InternalProcessShape(mappedShape, GetProcessingFlags().Item1, ps.Next());
                HealedFaceCheck.Check(unhealed, mappedShape, shapeProcessor.GetContext(), tp); // Kerf: decision 33502
                shapeProcessor.InternalMergeTransferInfo(tp, nbTPitems);
            }
        }
        found = !mappedShape.IsNull;
        if (found && shbinder is null)
        {
            shbinder = new ShapeBinder(mappedShape);
        }
        // TRANSLOG
        if (tp.TraceLevel() > 2)
        {
            using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
            sout.Append("End transfer STEP -> CASCADE :").Append(found ? "OK" : " : no result").Append('\n');
        }
        if (oldSRContext is null && _srContext is not null)
        { //: S4136
            PrepareUnits(oldSRContext, tp, ref aLocalFactors);
        }
        tp.Bind(start, shbinder);
        return shbinder;
    }

    /// <summary>Transfers mapped item</summary>
    protected ShapeBinder TransferEntity(MappedItem mapit, TransientProcess tp,
                                                      in Factors localFactors, ProgressRange progress)
    {
        ShapeBinder shbinder = null;

        // --------------------------------------------------------------
        // On se trouve ici dans un contexte " d'assemblage geometrique "
        //    - MappedItem
        // --------------------------------------------------------------

        //: S4136: abv 20 Apr 99: as1ug.stp: MAPPED_ITEM transformation computed
        //        taking into account units of origin and target SHAPE_REPRESENTATIONs

        //  La Shape, et la mise en position
        StepShapeRepresentation maprep = mapit.MappingSource().MappedRepresentation() as StepShapeRepresentation;
        bool isBound = false;
        using ProgressScope psRoot = new ProgressScope(progress, null, 2);
        Binder binder = tp.Find(maprep);
        if (binder is null)
        {
            binder = TransferEntity(maprep, tp, localFactors, ref isBound, false, progress);
        }
        shbinder = binder as ShapeBinder;
        if (shbinder is null)
        {
            tp.AddWarning(mapit, "No Shape Produced");
        }
        else
        {
            Shape mappedShape = shbinder.Result();
            if (!mappedShape.IsNull)
            {
                //  Positionnement : 2 formules
                //  1/ Ax2 dans Source et comme Target  : passage de Source a Target
                //  2/ CartesianOperator3d comme Target : on applique

                Transform3D trsf = new Transform3D();
                bool ok = false;

                StepCartesianTransformationOperator3d cartOp = mapit.MappingTarget() as StepCartesianTransformationOperator3d;
                if (cartOp is not null)
                {
                    ok = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation3d(cartOp, ref trsf, localFactors);
                }
                else
                {
                    StepAxis2Placement3d origin = mapit.MappingSource().MappingOrigin() as StepAxis2Placement3d;
                    StepAxis2Placement3d target = mapit.MappingTarget() as StepAxis2Placement3d;
                    if (origin is not null && target is not null)
                    {
                        ok = true;
                        StepRepresentation rep = _srContext; // NOTE: copy of handle !
                        ComputeTransformation(origin, target, maprep, rep, tp, ref trsf, localFactors);
                        ok = true;
                    }
                }

                if (ok)
                {
                    ApplyTransformation(ref mappedShape, trsf);
                }
                else
                {
                    tp.AddWarning(mapit, "Mapped Item, case not recognized, location ignored");
                }

                shbinder = new ShapeBinder(mappedShape);
            }
        }
        Compound cund = new Compound();
        Shape result = new Shape();
        BrepBuilder builder = new BrepBuilder();
        builder.MakeCompound(ref cund);
        if (shbinder is not null)
        {
            result = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(shbinder);
            builder.Add(cund, result);
        }

        // translate possible shapes related by SRRs, which corresponds to
        // way of writing hybrid models in AP203 since 1998, and AP209
        StepModel stepModel = tp.Model() as StepModel;
        if (stepModel.InternalParameters.ReadRelationship)
        {
            EntityGraph graph = tp.Graph();
            int srRnum = 0;
            for (EntityIterator subsIt = graph.Sharings(maprep); subsIt.More(); subsIt.Next())
            {
                ++srRnum;
            }
            using ProgressScope ps = new ProgressScope(psRoot.Next(), "Part", srRnum);
            Shape newResult = TransferRelatedSRR(tp, maprep, false, stepModel.InternalParameters.ReadConstrRelation,
                                                         localFactors, cund, ps);
            if (!newResult.IsNull)
            {
                result = newResult;
            }
            // if only single shape is read, add it as it is; otherwise add compound
            if (cund.ChildCount == 1)
            {
                shbinder = new ShapeBinder(result);
            }
            else if (cund.ChildCount > 1)
            {
                shbinder = new ShapeBinder(cund);
            }
        }
        tp.Bind(mapit, shbinder);
        return shbinder;
    }

    /// <summary>Transfers FaceSurface entity</summary>
    protected ShapeBinder TransferEntity(StepFaceSurface fs, TransientProcess tp,
                                                      in Factors localFactors, ProgressRange progress)
    {
        //    Cas bien utile meme si non reconnu explicitement
        ShapeBinder sb = null;
        int nbTPitems = tp.MappedCount;
        try
        {
            ImportTool myTool = new ImportTool();
            Dictionary<StepTopologicalRepresentationItem, Shape> map =
                new Dictionary<StepTopologicalRepresentationItem, Shape>();

            myTool.Init(map, tp);
            TranslateFace myTF = new TranslateFace();
            myTF.SetPrecision(_precision);
            myTF.SetMaxTol(_maxTol);

            // Non-manifold topology is not processed here (ssv; 15.11.2010)
            NMTool dummyNMTool = new NMTool();
            myTF.Init(fs, myTool, dummyNMTool, localFactors);
            StepRepresentation oldSRContext = _srContext;
            Factors aLocalFactors = localFactors;
            StepModel stepModel = tp.Model() as StepModel;
            if (_srContext is null)
            { // if no context, try to find it (ex: r0701_ug.stp #4790)
                StepRepresentation context = FindContext(fs, tp);
                if (context is null)
                {
                    tp.AddWarning(fs, "Entity with no unit context; default units taken");
                    ResetUnits(stepModel, ref aLocalFactors);
                }
                else
                {
                    PrepareUnits(context, tp, ref aLocalFactors);
                }
            }

            // Apply ShapeFix
            Binder binder = tp.Find(fs);
            sb = binder as ShapeBinder;
            if (sb is not null && !sb.Result().IsNull)
            {
                Shape s = sb.Result();

                Dictionary<string, string> parameters = GetShapeFixParameters();
                ShapeProcessor.SetParameter("FixShape.Tolerance3d", _precision, true, parameters);
                ShapeProcessor.SetParameter("FixShape.MaxTolerance3d", _maxTol, true, parameters);
                ShapeProcessor shapeProcessor = new ShapeProcessor(parameters);
                Shape shape = shapeProcessor.InternalProcessShape(s, GetProcessingFlags().Item1, progress);
                HealedFaceCheck.Check(s, shape, shapeProcessor.GetContext(), tp); // Kerf: decision 33502
                shapeProcessor.InternalMergeTransferInfo(tp, nbTPitems);

                if (shape != s)
                {
                    sb.SetResult(shape);
                }

                shapeProcessor.InternalMergeTransferInfo(tp, nbTPitems);
            }

            if (oldSRContext is null && _srContext is not null)
            { //: S4136
                PrepareUnits(oldSRContext, tp, ref aLocalFactors);
            }
            tp.Bind(fs, sb);
            return sb; // TP->Find (start);
        }
        catch (Exception exception) when (Refusal.IsFailure(exception) || ImportRoot.CaughtBySignals(exception))
        {
            tp.AddFail(fs, "Exception is raised. Entity was not translated.");
            sb = null;
            tp.Bind(fs, sb);
            return sb;
        }
    }

    /// <summary>
    /// theUseTrsf - special flag for using Axis2Placement from ShapeRepresentation for transform root shape
    /// </summary>
    public Binder TransferShape(Transient start, TransientProcess tp, in Factors localFactors,
                                         bool isManifold = true, bool useTrsf = false, ProgressRange progress = default)
    {
        if (start is null)
        {
            return NullResult();
        }
        // Kerf: a cycle of assembly links overflows OCCT's stack; here an exception (decision 1268)
        RuntimeHelpers.EnsureSufficientExecutionStack();

        // TRANSLOG
        //  POUR MISE AU POINT, a supprimer ensuite
        if (tp.TraceLevel() > 1)
        {
            using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
            sout.Append(" -- Actor : Transfer Ent.n0 ").Append(tp.Model().Number(start)).Append("  Type ")
                .Append(start.DynamicTypeName()).Append('\n');
        }

        ShapeBinder shbinder = null;
        StepModel stepModel = tp.Model() as StepModel;

        // Product Definition Entities
        // They should be treated with Design Manager
        // case ShapeDefinitionRepresentation if ProductMode != ON
        bool prodMode = stepModel.InternalParameters.ReadProductMode;
        if (!prodMode && start.IsKind(typeof(StepShapeDefinitionRepresentation)))
        {
            shbinder = OldWay(start, tp, progress);
            // skl
        }
        else if (start.IsKind(typeof(ProductDefinition)))
        {
            ProductDefinition pd = start as ProductDefinition;
            shbinder = TransferEntity(pd, tp, localFactors, useTrsf, progress);
        }

        // NextAssemblyUsageOccurrence
        else if (start.IsKind(typeof(NextAssemblyUsageOccurrence)))
        {
            NextAssemblyUsageOccurrence nauo = start as NextAssemblyUsageOccurrence;
            shbinder = TransferEntity(nauo, tp, localFactors, progress);
        }
        // end skl

        // Shape Representation
        else if (start.IsKind(typeof(StepShapeRepresentation)))
        {
            StepShapeRepresentation sr = start as StepShapeRepresentation;
            bool isBound = false;
            shbinder = TransferEntity(sr, tp, localFactors, ref isBound, false, progress);
        }

        // --------------------------------------------------------------
        // On se trouve ici aussi dans un contexte " d'assemblage geometrique "
        //    - ShapeRepresentationRelationship + Transformation ou non
        // --------------------------------------------------------------

        else if (start.IsKind(typeof(StepContextDependentShapeRepresentation)))
        {
            StepContextDependentShapeRepresentation cdsr = start as StepContextDependentShapeRepresentation;
            shbinder = TransferEntity(cdsr, tp, localFactors, progress);
        }

        else if (start.IsKind(typeof(ShapeRepresentationRelationship)))
        {
            //  REPRESENTATION_RELATIONSHIP et la famille

            ShapeRepresentationRelationship und = start as ShapeRepresentationRelationship;
            shbinder = TransferEntity(und, tp, localFactors, 0, false, progress);
        }

        else if (start.IsKind(typeof(StepGeometricRepresentationItem)))
        {
            // Here starts the entity to be treated : Shape Representation Subtype
            // It can be also other Root entities
            StepGeometricRepresentationItem git = start as StepGeometricRepresentationItem;
            shbinder = TransferEntity(git, tp, localFactors, isManifold, progress);
        }
        else if (start.IsKind(typeof(MappedItem)))
        {
            MappedItem mapit = start as MappedItem;
            shbinder = TransferEntity(mapit, tp, localFactors, progress);
        }
        else if (start.IsKind(typeof(StepFaceSurface)))
        {
            StepFaceSurface fs = start as StepFaceSurface;
            shbinder = TransferEntity(fs, tp, localFactors, progress);
        }

        //  if (!shbinder.IsNull()) TP->Bind(start,binder);

        return shbinder;
    }

    public Binder TransferShape(Transient start, TransientProcess tp)
        => TransferShape(start, tp, new Factors());

    Binder IActorRead.TransferShape(Transient start, TransientProcess tp, in Factors localFactors,
                                                          bool isManifold, bool useTrsf, ProgressRange progress)
        => TransferShape(start, tp, localFactors, isManifold, useTrsf, progress);

    /// <summary>set units and tolerances context by given ShapeRepresentation</summary>
    public void PrepareUnits(StepRepresentation rep, TransientProcess tp, ref Factors localFactors)
    {
        _srContext = rep;
        StepModel model = tp.Model() as StepModel;
        int stat1, stat2 = 0; // sera alimente par STEPControl_Unit
        if (rep is null)
        {
            ResetUnits(model, ref localFactors);
            return;
        }

        // Get Units Applied to this model
        RepresentationContext theRepCont = rep.ContextOfItems();
        if (theRepCont is null)
        {
            tp.AddWarning(rep, "Bad RepresentationContext, default unit taken");
            ResetUnits(model, ref localFactors);
            return;
        }

        // --------------------------------------------------
        // Complex ENTITY : GeometricRepresentationContext &&
        //                  GlobalUnitAssignedContext
        // --------------------------------------------------

        UnitContext myUnit = new UnitContext();
        GlobalUnitAssignedContext theGUAC = null;
        GlobalUncertaintyAssignedContext tol = null;

        if (theRepCont.IsKind(typeof(StepGeometricRepresentationContextAndGlobalUnitAssignedContext)))
        {
            StepGeometricRepresentationContextAndGlobalUnitAssignedContext theGRCAGAUC =
                theRepCont as StepGeometricRepresentationContextAndGlobalUnitAssignedContext;
            theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
        }

        // ----------------------------------------------------
        // Complex ENTITY : GeometricRepresentationContext   &&
        //                  GlobalUnitAssignedContext        &&
        //                  GlobalUncertaintyAssignedContext
        // ----------------------------------------------------

        if (theRepCont.IsKind(typeof(StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx)))
        {
            StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx theGRCAGAUC =
                theRepCont as StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx;
            theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
            tol = theGRCAGAUC.GlobalUncertaintyAssignedContext();
        }

        StepModel stepModel = tp.Model() as StepModel;
        // ----------------------------------------------------
        // Decoding and Setting the Values
        // ----------------------------------------------------
        if (theGUAC is not null)
        {
            stat1 = myUnit.ComputeFactors(theGUAC, localFactors);
            int anglemode = (int)stepModel.InternalParameters.AngleUnit;
            double angleFactor = (anglemode == 0 ? myUnit.PlaneAngleFactor()
                                  : anglemode == 1 ? 1.0
                                                   : Math.PI / 180.0);
            localFactors.InitializeFactors(myUnit.LengthFactor(), angleFactor, myUnit.SolidAngleFactor());
            if (stat1 != 0)
            {
                tp.AddWarning(theRepCont, myUnit.StatusMessage(stat1));
            }
        }

        if (tol is not null)
        {
            stat2 = myUnit.ComputeTolerance(tol);
            if (stat2 != 0)
            {
                tp.AddWarning(theRepCont, myUnit.StatusMessage(stat2));
            }
        }

        //  myPrecision = Precision::Confusion();
        if (stepModel.InternalParameters.ReadPrecisionMode == Parameters.ReadMode_Precision.ReadMode_Precision_User)
        { //: i1 gka S4136 05.04.99
            _precision = stepModel.InternalParameters.ReadPrecisionVal;
        }
        else if (myUnit.HasUncertainty)
        {
            _precision = myUnit.Uncertainty() * myUnit.LengthFactor();
        }
        else
        {
            tp.AddWarning(theRepCont, "No Length Uncertainty, value of read.precision.val is taken");
            _precision = stepModel.InternalParameters.ReadPrecisionVal;
        }
        _maxTol = Math.Max(_precision, stepModel.InternalParameters.ReadMaxPrecisionVal);
        // Assign uncertainty
        // TRANSLOG
        if (tp.TraceLevel() > 1)
        {
            using Messenger.StreamBuffer sout = tp.Messenger().SendInfo();
            sout.Append("  Cc1ToTopoDS : Length Unit = ").Append(myUnit.LengthFactor()).Append("  Tolerance CASCADE = ")
                .Append(_precision).Append('\n');
        }
    }

    /// <summary>reset units and tolerances context to default (mm, radians, read.precision.val, etc.)</summary>
    public void ResetUnits(StepModel model, ref Factors localFactors)
    {
        localFactors.InitializeFactors(1, 1, 1);
        _precision = model.InternalParameters.ReadPrecisionVal;
        _maxTol = Math.Max(_precision, model.InternalParameters.ReadMaxPrecisionVal);
    }

    /// <summary>
    /// Computes transformation defined by two axis placements (in MAPPED_ITEM or ITEM_DEFINED_TRANSFORMATION) taking
    /// into account their representation contexts (i.e. units, which may be different) Returns True if
    /// transformation is computed and is not an identity.
    /// </summary>
    public bool ComputeTransformation(StepAxis2Placement3d origin, StepAxis2Placement3d target, StepRepresentation origContext,
                                      StepRepresentation targContext, TransientProcess tp, ref Transform3D trsf,
                                      in Factors localFactors)
    {
        trsf = new Transform3D(); // reinit
        if (origin is null || target is null)
        {
            return false;
        }

        //: abv 31.10.01: TEST_MCI_2.step: check that Ax1 and Ax2 belong to
        // corresponding reps and fix case of inversion error
        StepAxis2Placement3d org = origin;
        StepAxis2Placement3d trg = target;
        bool isOKOrigin = false, isSwapOrigin = false;
        bool isOKTarget = false, isSwapTarget = false;
        for (int i = 1; i <= origContext.NbItems(); i++)
        {
            if (origContext.ItemsValue(i) == org)
            {
                isOKOrigin = true;
            }
            else if (origContext.ItemsValue(i) == trg)
            {
                isSwapTarget = true;
            }
        }
        for (int i = 1; i <= targContext.NbItems(); i++)
        {
            if (targContext.ItemsValue(i) == trg)
            {
                isOKTarget = true;
            }
            else if (targContext.ItemsValue(i) == org)
            {
                isSwapOrigin = true;
            }
        }
        if (!isOKOrigin || !isOKTarget)
        {
            if (isSwapOrigin && isSwapTarget)
            {
                (org, trg) = (trg, org);
                tp.AddWarning(org, "Axis placements are swapped in SRRWT; corrected");
            }
            else
            {
                tp.AddWarning((isOKOrigin ? trg : org),
                              "Axis placement used by SRRWT does not belong to corresponding representation");
            }
        }

        // translate axis_placements taking units into account
        StepRepresentation oldSRContext = _srContext;
        Factors aLocalFactors = localFactors;
        if (origContext != oldSRContext)
        {
            PrepareUnits(origContext, tp, ref aLocalFactors);
        }
        Axis2Placement theOrig = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(org, aLocalFactors);
        if (targContext != origContext)
        {
            PrepareUnits(targContext, tp, ref aLocalFactors);
        }
        Axis2Placement theTarg = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(trg, aLocalFactors);
        if (oldSRContext != targContext)
        {
            PrepareUnits(oldSRContext, tp, ref aLocalFactors);
        }

        CoordinateSystem ax3Orig = new CoordinateSystem(theOrig.Ax2());
        CoordinateSystem ax3Targ = new CoordinateSystem(theTarg.Ax2());

        //  ne pas se tromper de sens !
        trsf = trsf.WithTransformation(ax3Targ, ax3Orig);
        return trsf.Form != Identity;
    }

    public bool ComputeTransformation(StepAxis2Placement3d origin, StepAxis2Placement3d target, StepRepresentation origContext,
                                      StepRepresentation targContext, TransientProcess tp, ref Transform3D trsf)
        => ComputeTransformation(origin, target, origContext, targContext, tp, ref trsf, new Factors());

    /// <summary>Computes transformation defined by given REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION</summary>
    public bool ComputeSRRWT(RepresentationRelationship srr, TransientProcess tp, ref Transform3D trsf,
                             in Factors localFactors)
    {
        trsf = new Transform3D(); // init

        ShapeRepresentationRelationshipWithTransformation srwt = srr as ShapeRepresentationRelationshipWithTransformation;
        if (srwt is null)
        {
            return false;
        }

        Transformation selectTrans = srwt.TransformationOperator();
        Factors aLocalFactors = localFactors;
        // cartesian transformation
        StepCartesianTransformationOperator3d cartOp = selectTrans.Value() as StepCartesianTransformationOperator3d;
        if (cartOp is not null)
        {
            // reset units (by Rep2 - ?)
            StepRepresentation oldSRContext = _srContext;
            if (srr.Rep2() != oldSRContext)
            {
                PrepareUnits(srr.Rep2(), tp, ref aLocalFactors);
            }
            global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation3d(cartOp, ref trsf, aLocalFactors);
            if (srr.Rep2() != oldSRContext)
            {
                PrepareUnits(oldSRContext, tp, ref aLocalFactors);
            }
            return trsf.Form != Identity;
        }

        // item-defined transformation
        ItemDefinedTransformation itemDef = selectTrans.ItemDefinedTransformation();
        if (itemDef is null)
        {
            return false;
        }

        StepAxis2Placement3d ax1 = itemDef.TransformItem1() as StepAxis2Placement3d;
        StepAxis2Placement3d ax2 = itemDef.TransformItem2() as StepAxis2Placement3d;
        if (ax1 is null || ax2 is null)
        {
            return false;
        }
        return ComputeTransformation(ax1, ax2, srr.Rep1(), srr.Rep2(), tp, ref trsf, aLocalFactors);
    }

    public bool ComputeSRRWT(RepresentationRelationship srr, TransientProcess tp, ref Transform3D trsf)
        => ComputeSRRWT(srr, tp, ref trsf, new Factors());

    //=======================================================================
    // Method  : closeIDEASShell
    // Purpose : Attempts to close the passed Shell with the passed closing
    //           Shells. Cuts redundant Faces from the closing Shells if any
    //=======================================================================
    private Shell CloseIDEASShell(Shell shell, NCollection_List<Shape> closingShells)
    {
        // Make Shell to provide closeness adjustments
        Shell result = new Shell();
        BrepBuilder brepBuilder = new BrepBuilder();
        brepBuilder.MakeShell(ref result);

        // Firstly, add all existing faces to the new Shell
        Explorer currentFExp = new Explorer(shell, ShapeType.Face);
        for (; currentFExp.More(); currentFExp.Next())
        {
            Face currentFace = ShapeCasts.Face(currentFExp.Current());
            brepBuilder.Add(result, currentFace);
        }

        NCollection_List<Shape> closingFaces = new NCollection_List<Shape>();

        // Then add the closing faces
        foreach (Shape currentClosing in closingShells)
        {
            Explorer faceExp = new Explorer(currentClosing, ShapeType.Face);

            for (; faceExp.More(); faceExp.Next())
            {
                Face currentFace = ShapeCasts.Face(faceExp.Current());
                brepBuilder.Add(result, currentFace);
                // Store each added closing face for subsequent processing
                closingFaces.Append(currentFace);
            }
        }

        // Check if the result is closed
        ShellCheck checker = new ShellCheck(ShapeCasts.Shell(result));
        StatusCheck checkStatus = checker.Closed();
        if (checkStatus == StatusCheck.NoError)
        {
            result.Closed(true);
        }
        else
        {
            return shell; // Cannot close this shell, skip it so...
        }

        // Try to remove redundant Faces
        foreach (Shape closingFace in closingFaces)
        {
            Face currentFace = ShapeCasts.Face(closingFace);
            // Remove face to see if Shell is still closed
            brepBuilder.Remove(result, currentFace);
            ShellCheck subChecker = new ShellCheck(ShapeCasts.Shell(result));
            StatusCheck subCheckStatus = subChecker.Closed();
            // If Shell becomes open, just put the deleted face back
            if (subCheckStatus != StatusCheck.NoError)
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
    private void ComputeIDEASClosings(Compound comp,
                                      IndexedMap<Shape, NCollection_List<Shape>> shellClosingsMap)
    {
        Explorer shellExpA = new Explorer(comp, ShapeType.Shell);

        for (; shellExpA.More(); shellExpA.Next())
        {
            Shape shellA = shellExpA.Current();
            Explorer shellExpB = new Explorer(comp, ShapeType.Shell);
            NCollection_List<Shape> closingShells = new NCollection_List<Shape>();

            for (; shellExpB.More(); shellExpB.Next())
            {
                Shape shellB = shellExpB.Current();
                if (shellA.IsSame(shellB))
                {
                    continue;
                }
                // Check whether ShellB is non-manifold and adjacent to ShellA.
                // If so, ShellA has a chance to be closed with ShellB
                if (_nmTool.IsSuspectedAsClosing(shellA, shellB))
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
    public void SetModel(InterfaceModel model)
    {
        _model = model;
    }

    //=======================================================================
    // Method  : TransferRelatedSRR
    // Purpose : Helper method to transfer SRR related to the representation
    //=======================================================================
    private Shape TransferRelatedSRR(TransientProcess tp, StepShapeRepresentation rep, bool useTrsf,
                                            bool readConstructiveGeomRR, in Factors localFactors, Compound cund,
                                            ProgressScope ps)
    {
        BrepBuilder builder = new BrepBuilder();
        Shape result = new Shape();
        EntityGraph graph = tp.Graph();
        for (EntityIterator subsIt = graph.Sharings(rep); subsIt.More() && ps.More(); subsIt.Next())
        {
            Transient item = subsIt.Value();
            if (!item.IsKind(typeof(RepresentationRelationship)))
            {
                continue;
            }
            Binder binder = null;
            if (item.DynamicType() == typeof(ShapeRepresentationRelationship))
            {
                ShapeRepresentationRelationship srr = item as ShapeRepresentationRelationship;
                int nbRep = (rep == srr.Rep1() ? 2 : 1);
                binder = TransferEntity(srr, tp, localFactors, nbRep, useTrsf, ps.Next());
            }
            else if (item.DynamicType() == typeof(MechanicalDesignAndDraughtingRelationship))
            {
                MechanicalDesignAndDraughtingRelationship mdadr = item as MechanicalDesignAndDraughtingRelationship;
                binder = TransferEntity(mdadr, tp, localFactors, ps.Next());
            }
            else if (readConstructiveGeomRR
                     && item.DynamicType() == typeof(ConstructiveGeometryRepresentationRelationship))
            {
                ConstructiveGeometryRepresentationRelationship cgrr = item as ConstructiveGeometryRepresentationRelationship;
                binder = TransferEntity(cgrr, tp, localFactors);
            }
            if (binder is not null)
            {
                result = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
                builder.Add(cund, result);
            }
        }
        return result;
    }

    private NMTool _nmTool = new NMTool();
    private double _precision;
    private double _maxTol;
    private StepRepresentation _srContext;
    private InterfaceModel _model;
}
