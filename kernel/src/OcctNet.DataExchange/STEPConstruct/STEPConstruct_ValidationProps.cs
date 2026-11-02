// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ValidationProps.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ValidationProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.StepBasic_SiPrefix;
using static Occt.StepBasic_SiUnitName;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// This class provides tools for access (write and read) the validation properties on shapes in the STEP file. These
/// are surface area, solid volume and centroid.
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class STEPConstruct_ValidationProps : STEPConstruct_Tool
{
    /// <summary>Creates an empty tool</summary>
    public STEPConstruct_ValidationProps()
    {
    }

    /// <summary>Creates a tool and loads it with worksession</summary>
    public STEPConstruct_ValidationProps(XSControl_WorkSession WS) : base(WS)
    {
    }

    /// <summary>Load worksession; returns True if succeeded</summary>
    public bool Init(XSControl_WorkSession WS)
    {
        return SetWS(WS);
    }

    private static Transfer_SimpleBinderOfTransient TransientResult(Standard_Transient res)
    {
        Transfer_SimpleBinderOfTransient binder = null;
        if (res is null)
        {
            return binder;
        }
        binder = new Transfer_SimpleBinderOfTransient();
        binder.SetResult(res);
        return binder;
    }

    /// <summary>
    /// Finds target STEP entity to which validation props should be assigned, and corresponding context, starting
    /// from shape Returns True if success, False in case of fail
    /// </summary>
    public bool FindTarget(TopoDS_Shape Shape, StepRepr_CharacterizedDefinition target,
                           ref StepRepr_RepresentationContext Context, bool instance = false)
    {
        // find the target STEP entity corresponding to a shape
        TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(FinderProcess(), Shape);
        Transfer_Binder binder = FinderProcess().Find(mapper);

        // if requested, try to find instance of assembly
        if (instance)
        {
            // OCCT: the search of an instance is commented out
        }

        // for Compounds, search for SDR
        else if (Shape.ShapeType() == TopAbs_COMPOUND)
        {
            StepBasic_ProductDefinition ProdDef = null;
            if (FinderProcess().FindTypedTransient(mapper, typeof(StepBasic_ProductDefinition), ref ProdDef))
            {
                StepRepr_ProductDefinitionShape PDS = null;
                Interface_EntityIterator subs1 = Graph().Sharings(ProdDef);
                for (subs1.Start(); PDS is null && subs1.More(); subs1.Next())
                {
                    PDS = subs1.Value() as StepRepr_ProductDefinitionShape;
                }
                target.SetValue(PDS);
            }
            else
            {
                StepShape_ShapeRepresentation SR = null;
                if (FinderProcess().FindTypedTransient(mapper, typeof(StepShape_ShapeRepresentation), ref SR))
                {
                    StepRepr_ProductDefinitionShape PDS = null;
                    Interface_EntityIterator subs1 = Graph().Sharings(SR);
                    for (subs1.Start(); PDS is null && subs1.More(); subs1.Next())
                    {
                        StepShape_ShapeDefinitionRepresentation SDR = subs1.Value() as StepShape_ShapeDefinitionRepresentation;
                        if (SDR is null)
                        {
                            continue;
                        }
                        PDS = SDR.Definition().PropertyDefinition() as StepRepr_ProductDefinitionShape;
                    }
                    if (PDS is null)
                    {
                        subs1 = Graph().Sharings(SR);
                        for (subs1.Start(); PDS is null && subs1.More(); subs1.Next())
                        {
                            StepRepr_RepresentationRelationship RR = subs1.Value() as StepRepr_RepresentationRelationship;
                            if (RR is null)
                            {
                                continue;
                            }
                            StepShape_ShapeRepresentation SR1;
                            if (RR.Rep1() == SR)
                            {
                                SR1 = RR.Rep2() as StepShape_ShapeRepresentation;
                            }
                            else
                            {
                                SR1 = RR.Rep1() as StepShape_ShapeRepresentation;
                            }
                            if (SR1 is not null)
                            {
                                Interface_EntityIterator subs2 = Graph().Sharings(SR1);
                                for (subs2.Start(); PDS is null && subs2.More(); subs2.Next())
                                {
                                    StepShape_ShapeDefinitionRepresentation SDR = subs2.Value() as StepShape_ShapeDefinitionRepresentation;
                                    if (SDR is null)
                                    {
                                        continue;
                                    }
                                    PDS = SDR.Definition().PropertyDefinition() as StepRepr_ProductDefinitionShape;
                                }
                            }
                        }
                    }
                    if (PDS is not null)
                    {
                        target.SetValue(PDS);
                        Context = SR.ContextOfItems();
                    }
                }
            }
        }

        // for others, search for GEOMETRIC_REPRESENTATION_ITEM
        else
        {
            StepGeom_GeometricRepresentationItem item = null;
            if (FinderProcess().FindTypedTransient(mapper, typeof(StepGeom_GeometricRepresentationItem), ref item))
            {
                // find PDS (GRI <- SR <- SDR -> PDS)
                StepRepr_ProductDefinitionShape PDS = null;
                Interface_EntityIterator subs = Graph().Sharings(item);
                for (subs.Start(); PDS is null && subs.More(); subs.Next())
                {
                    if (!subs.Value().IsKind(typeof(StepShape_ShapeRepresentation)))
                    {
                        continue;
                    }
                    StepShape_ShapeRepresentation sr = subs.Value() as StepShape_ShapeRepresentation;
                    Context = sr.ContextOfItems();
                    Interface_EntityIterator sub2 = Graph().Sharings(subs.Value());
                    for (sub2.Start(); sub2.More(); sub2.Next())
                    {
                        if (!sub2.Value().IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
                        {
                            continue;
                        }
                        StepShape_ShapeDefinitionRepresentation sdr = sub2.Value() as StepShape_ShapeDefinitionRepresentation;
                        PDS = sdr.Definition().PropertyDefinition() as StepRepr_ProductDefinitionShape;
                    }
                }
                if (PDS is not null)
                {
                    // find SHAPE_ASPECT or create it with all associated info if not yet exists
                    StepRepr_ShapeAspect aspect = null;
                    Transfer_Binder bbb = binder;
                    while (bbb is not null)
                    {
                        Transfer_SimpleBinderOfTransient bx = bbb as Transfer_SimpleBinderOfTransient;
                        if (bx is not null)
                        {
                            StepRepr_ShapeAspect asp = bx.Result() as StepRepr_ShapeAspect;
                            if (asp is not null && asp.OfShape() == PDS)
                            {
                                aspect = asp;
                                break;
                            }
                        }
                        bbb = bbb.NextResult();
                    }
                    if (aspect is null)
                    {
                        // create aspect and all related data
                        TCollection_HAsciiString AspectName = new TCollection_HAsciiString("");
                        TCollection_HAsciiString AspectDescr = new TCollection_HAsciiString("");
                        aspect = new StepRepr_ShapeAspect();
                        aspect.Init(AspectName, AspectDescr, PDS, StepData_Logical.StepData_LFalse);

                        StepRepr_CharacterizedDefinition SA = new StepRepr_CharacterizedDefinition();
                        SA.SetValue(aspect);

                        TCollection_HAsciiString PropDefName = new TCollection_HAsciiString("shape with specific properties");
                        TCollection_HAsciiString PropDefDescr = new TCollection_HAsciiString("properties for subshape");
                        StepRepr_PropertyDefinition propdef = new StepRepr_PropertyDefinition();
                        propdef.Init(PropDefName, true, PropDefDescr, SA);

                        TCollection_HAsciiString SRName = new TCollection_HAsciiString("");
                        StepShape_ShapeRepresentation SR = new StepShape_ShapeRepresentation();
                        NCollection_HArray1<StepRepr_RepresentationItem> SRItems = new NCollection_HArray1<StepRepr_RepresentationItem>(1, 1);
                        SRItems.SetValue(1, item);
                        SR.Init(SRName, SRItems, Context);

                        StepShape_ShapeDefinitionRepresentation SDR = new StepShape_ShapeDefinitionRepresentation();
                        StepRepr_RepresentedDefinition RD = new StepRepr_RepresentedDefinition();
                        RD.SetValue(propdef);
                        SDR.Init(RD, SR);

                        // record SHAPE_ASPECT in the map
                        binder.AddResult(TransientResult(aspect));

                        // add SDR and all the data into model
                        Model().AddWithRefs(SDR);
                    }
                    // SHAPE_ASPECT found, but we also need context: FIND IT !!!!
                    else
                    {
                        StepRepr_ProductDefinitionShape aPDS = aspect.OfShape();
                        Interface_EntityIterator asubs = Graph().Sharings(aPDS);
                        for (asubs.Start(); Context is null && asubs.More(); asubs.Next())
                        {
                            if (!asubs.Value().IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
                            {
                                continue;
                            }
                            StepShape_ShapeDefinitionRepresentation sdr = asubs.Value() as StepShape_ShapeDefinitionRepresentation;
                            Context = sdr.UsedRepresentation().ContextOfItems();
                        }
                    }

                    if (aspect is not null)
                    {
                        target.SetValue(aspect);
                    }
                }
                // OCCT_DEBUG output left out
            }
            // OCCT_DEBUG output left out
        }

        // if target not found and shape has location, try the same shape without location
        return !target.IsNull();
    }

    /// <summary>
    /// General method for adding (writing) a validation property for shape which should be already mapped on writing
    /// itself. It takes target and Context entities which correspond to shape Returns True if success, False in case
    /// of fail
    /// </summary>
    public bool AddProp(StepRepr_CharacterizedDefinition target, StepRepr_RepresentationContext Context,
                        StepRepr_RepresentationItem Prop, string Descr)
    {
        // FINALLY, create a structure of 5 entities describing a link between a shape and its property
        TCollection_HAsciiString PropDefName = new TCollection_HAsciiString("geometric validation property");
        TCollection_HAsciiString PropDefDescr = new TCollection_HAsciiString(Descr);
        StepRepr_PropertyDefinition propdef = new StepRepr_PropertyDefinition();
        propdef.Init(PropDefName, true, PropDefDescr, target);

        TCollection_HAsciiString SRName = new TCollection_HAsciiString(Descr);
        StepRepr_Representation rep = new StepRepr_Representation();
        NCollection_HArray1<StepRepr_RepresentationItem> SRItems = new NCollection_HArray1<StepRepr_RepresentationItem>(1, 1);
        SRItems.SetValue(1, Prop);
        rep.Init(SRName, SRItems, Context);

        StepRepr_PropertyDefinitionRepresentation PrDR = new StepRepr_PropertyDefinitionRepresentation();
        StepRepr_RepresentedDefinition RD = new StepRepr_RepresentedDefinition();
        RD.SetValue(propdef);
        PrDR.Init(RD, rep);

        // record SDR in order to have it written to the file
        Model().AddWithRefs(PrDR);

        StepData_StepModel aStepModel = Model() as StepData_StepModel;
        // for AP203, add subschema name
        if (aStepModel.InternalParameters.WriteSchema == DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203)
        {
            APIHeaderSection_MakeHeader mkHdr = new APIHeaderSection_MakeHeader(aStepModel);
            TCollection_HAsciiString subSchema = new TCollection_HAsciiString("GEOMETRIC_VALIDATION_PROPERTIES_MIM");
            mkHdr.AddSchemaIdentifier(subSchema);
        }

        return true;
    }

    /// <summary>
    /// General method for adding (writing) a validation property for shape which should be already mapped on writing
    /// itself. It uses FindTarget() to find target STEP entity resulting from given shape, and associated context
    /// Returns True if success, False in case of fail
    /// </summary>
    public bool AddProp(TopoDS_Shape Shape, StepRepr_RepresentationItem Prop, string Descr, bool instance = false)
    {
        StepRepr_CharacterizedDefinition target = new StepRepr_CharacterizedDefinition();
        StepRepr_RepresentationContext Context = null;
        if (!FindTarget(Shape, target, ref Context, instance))
        {
            return false;
        }
        return AddProp(target, Context, Prop, Descr);
    }

    /// <summary>Adds volume property for given shape (already mapped). Returns True if success, False in case of fail</summary>
    public bool AddVolume(TopoDS_Shape Shape, double Vol)
    {
        StepBasic_MeasureValueMember Val = new StepBasic_MeasureValueMember();
        Val.SetReal(Vol);
        // Val->SetName ( "solid volume" );
        Val.SetName("VOLUME_MEASURE");

        // for volume unit, either take existing or create a new
        if (volUnit.DerivedUnit() is null)
        {
            StepBasic_SiUnitAndLengthUnit MM = new StepBasic_SiUnitAndLengthUnit();
            MM.Init(true, StepBasic_spMilli, StepBasic_sunMetre);

            StepBasic_DerivedUnitElement DUE = new StepBasic_DerivedUnitElement();
            DUE.Init(MM, 3.0);

            NCollection_HArray1<StepBasic_DerivedUnitElement> DUElems = new NCollection_HArray1<StepBasic_DerivedUnitElement>(1, 1);
            DUElems.SetValue(1, DUE);

            StepBasic_DerivedUnit DU = new StepBasic_DerivedUnit();
            DU.Init(DUElems);

            volUnit.SetValue(DU);
        }

        TCollection_HAsciiString MRIName = new TCollection_HAsciiString("volume measure");
        StepRepr_MeasureRepresentationItem MRI = new StepRepr_MeasureRepresentationItem();
        MRI.Init(MRIName, Val, new StepBasic_Unit(volUnit));

        return AddProp(Shape, MRI, "volume");
    }

    /// <summary>Adds surface area property for given shape (already mapped). Returns True if success, False in case of fail</summary>
    public bool AddArea(TopoDS_Shape Shape, double Area)
    {
        StepBasic_MeasureValueMember Val = new StepBasic_MeasureValueMember();
        Val.SetReal(Area);
        // Val->SetName ( "surface area" );
        Val.SetName("AREA_MEASURE");

        // for area unit, either take existing or create a new
        if (areaUnit.DerivedUnit() is null)
        {
            StepBasic_SiUnitAndLengthUnit MM = new StepBasic_SiUnitAndLengthUnit();
            MM.Init(true, StepBasic_spMilli, StepBasic_sunMetre);

            StepBasic_DerivedUnitElement DUE = new StepBasic_DerivedUnitElement();
            DUE.Init(MM, 2.0);

            NCollection_HArray1<StepBasic_DerivedUnitElement> DUElems = new NCollection_HArray1<StepBasic_DerivedUnitElement>(1, 1);
            DUElems.SetValue(1, DUE);

            StepBasic_DerivedUnit DU = new StepBasic_DerivedUnit();
            DU.Init(DUElems);

            areaUnit.SetValue(DU);
        }

        TCollection_HAsciiString MRIName = new TCollection_HAsciiString("surface area measure");
        StepRepr_MeasureRepresentationItem MRI = new StepRepr_MeasureRepresentationItem();
        MRI.Init(MRIName, Val, new StepBasic_Unit(areaUnit));

        return AddProp(Shape, MRI, "surface area");
    }

    /// <summary>
    /// Adds centroid property for given shape (already mapped). Returns True if success, False in case of fail If
    /// instance is True, then centroid is assigned to an instance of component in assembly
    /// </summary>
    public bool AddCentroid(TopoDS_Shape Shape, in gp_Pnt Pnt, bool instance = false)
    {
        TCollection_HAsciiString CPName = new TCollection_HAsciiString("centre point");
        StepGeom_CartesianPoint CP = new StepGeom_CartesianPoint();
        CP.Init3D(CPName, Pnt.X(), Pnt.Y(), Pnt.Z());

        return AddProp(Shape, CP, "centroid", instance);
    }

    /// <summary>
    /// Searches for entities of the type PropertyDefinitionRepresentation in the model and fills the sequence by them
    /// </summary>
    public bool LoadProps(NCollection_Sequence<Standard_Transient> seq)
    {
        // parse on PropertyDefinitionRepresentations
        int nb = Model().NbEntities();
        Type tPDR = typeof(StepRepr_PropertyDefinitionRepresentation);
        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient enti = Model().Value(i);
            if (!enti.IsKind(tPDR))
            {
                continue;
            }

            StepRepr_PropertyDefinitionRepresentation PDR = enti as StepRepr_PropertyDefinitionRepresentation;

            // Check that PDR is for validation props.
            StepRepr_PropertyDefinition PD = PDR.Definition().PropertyDefinition();
            if (PD is not null && PD.Name() is not null)
            {
                // Note: according to "Recommended Practices for Geometric and Assembly Validation Properties"
                // Release 4.4 as of Augist 17, 2016, item 4.6, the name of PropertyDefinition should be
                // "geometric validation property" with words separated by spaces; however older versions of
                // the same RP document used underscores. To be able to read files written using older
                // convention, we convert all underscores to spaces for this check.
                string aName = PD.Name().ToCString().Replace('_', ' ').ToLowerInvariant();
                if (aName != "geometric validation property")
                {
                    continue;
                }
            }

            seq.Append(PDR);
        }
        return seq.Length() > 0;
    }

    /// <summary>Returns CDSR associated with given PpD or NULL if not found (when, try GetPropSDR)</summary>
    public StepRepr_NextAssemblyUsageOccurrence GetPropNAUO(StepRepr_PropertyDefinition PD)
    {
        StepRepr_CharacterizedDefinition CD = PD.Definition();

        // detect target entity of valprop
        StepRepr_NextAssemblyUsageOccurrence NAUO = null;
        StepRepr_PropertyDefinition PDS = CD.ProductDefinitionShape();
        if (PDS is null)
        {
            return NAUO; // not found
        }
        Interface_EntityIterator subs = Graph().Shareds(PDS);
        for (subs.Start(); NAUO is null && subs.More(); subs.Next())
        {
            NAUO = subs.Value() as StepRepr_NextAssemblyUsageOccurrence;
        }
        return NAUO;
    }

    /// <summary>Returns SDR associated with given PpD or NULL if not found (when, try GetPropCDSR)</summary>
    public StepBasic_ProductDefinition GetPropPD(StepRepr_PropertyDefinition PD)
    {
        StepRepr_CharacterizedDefinition CD = PD.Definition();

        // detect target entity of valprop
        StepBasic_ProductDefinition ProdDef = null;
        StepRepr_PropertyDefinition PDS = CD.ProductDefinitionShape();
        if (PDS is null)
        {
            StepRepr_ShapeAspect SA = CD.ShapeAspect();
            if (SA is null)
            {
                // OCCT_DEBUG output left out
                return ProdDef;
            }
            Interface_EntityIterator subs = Graph().Sharings(SA);
            for (subs.Start(); subs.More(); subs.Next())
            {
                PDS = subs.Value() as StepRepr_PropertyDefinition;
                if (PDS is null)
                {
                    return ProdDef;
                }
                Interface_EntityIterator subs1 = Graph().Shareds(PDS);
                for (subs1.Start(); ProdDef is null && subs1.More(); subs1.Next())
                {
                    ProdDef = subs1.Value() as StepBasic_ProductDefinition;
                }
            }
        }
        else
        {
            Interface_EntityIterator subs = Graph().Shareds(PDS);
            for (subs.Start(); ProdDef is null && subs.More(); subs.Next())
            {
                ProdDef = subs.Value() as StepBasic_ProductDefinition;
            }
        }
        // OCCT_DEBUG output left out
        return ProdDef;
    }

    /// <summary>Returns Shape associated with given SDR or Null Shape if not found</summary>
    public TopoDS_Shape GetPropShape(StepBasic_ProductDefinition ProdDef)
    {
        // find target shape
        TopoDS_Shape S = new TopoDS_Shape();
        Transfer_Binder binder = TransientProcess().Find(ProdDef);
        if (binder is not null && binder.HasResult())
        {
            S = TransferBRep.ShapeResult(TransientProcess(), binder);
        }
        // OCCT_DEBUG output left out
        return S;
    }

    /// <summary>Returns Shape associated with given PpD or Null Shape if not found</summary>
    public TopoDS_Shape GetPropShape(StepRepr_PropertyDefinition PD)
    {
        StepBasic_ProductDefinition ProdDef = GetPropPD(PD);
        TopoDS_Shape S = new TopoDS_Shape();
        if (ProdDef is not null)
        {
            S = GetPropShape(ProdDef);
        }
        return S;
    }

    /// <summary>
    /// Returns value of Real-Valued property (Area or Volume) If Property is neither Area nor Volume, returns False
    /// Else returns True and isArea indicates whether property is area or volume
    /// </summary>
    public bool GetPropReal(StepRepr_RepresentationItem item, out double Val, out bool isArea, in StepData_Factors theLocalFactors)
    {
        Val = 0.0;
        isArea = false;
        // decode volume & area
        if (!item.IsKind(typeof(StepRepr_MeasureRepresentationItem)))
        {
            return false;
        }

        StepRepr_MeasureRepresentationItem mri = item as StepRepr_MeasureRepresentationItem;

        StepBasic_MeasureWithUnit M = mri.Measure();
        string Name = M.ValueComponentMember().Name();
        StepBasic_Unit Unit = M.UnitComponent();

        double scale = 1.0;
        StepBasic_DerivedUnit DU = Unit.DerivedUnit();
        if (DU is not null)
        {
            for (int ind = 1; ind <= DU.NbElements(); ind++)
            {
                StepBasic_DerivedUnitElement DUE = DU.ElementsValue(ind);
                double exp = DUE.Exponent();
                StepBasic_NamedUnit NU = DUE.Unit();
                STEPConstruct_UnitContext unit = new STEPConstruct_UnitContext();
                unit.ComputeFactors(NU, theLocalFactors);
                if (unit.LengthDone())
                {
                    double lengthFactor = unit.LengthFactor();
                    scale *= Math.Pow(lengthFactor, exp);
                }
            }
        }
        else
        {
            StepBasic_NamedUnit NU = Unit.NamedUnit();
            if (NU is not null)
            {
                STEPConstruct_UnitContext unit = new STEPConstruct_UnitContext();
                unit.ComputeFactors(NU, theLocalFactors);
                if (unit.AreaDone())
                {
                    scale = unit.AreaFactor();
                }
                if (unit.VolumeDone())
                {
                    scale = unit.VolumeFactor();
                }
            }
        }

        Val = M.ValueComponent() * scale;

        if (Name == "AREA_MEASURE")
        {
            isArea = true;
        }
        else if (Name == "VOLUME_MEASURE")
        {
            isArea = false;
        }
        else
        {
            // OCCT_DEBUG output left out
            return false;
        }
        return true;
    }

    /// <summary>Returns value of Centroid property (or False if it is not)</summary>
    public bool GetPropPnt(StepRepr_RepresentationItem item, StepRepr_RepresentationContext Context, out gp_Pnt Pnt,
                           in StepData_Factors theLocalFactors)
    {
        Pnt = new gp_Pnt();
        // centroid
        if (!item.IsKind(typeof(StepGeom_CartesianPoint)))
        {
            return false;
        }

        StepGeom_CartesianPoint P = item as StepGeom_CartesianPoint;
        if (P is null || P.NbCoordinates() != 3)
        {
            // OCCT_DEBUG output left out
            return false;
        }

        gp_Pnt pos = new gp_Pnt(P.CoordinatesValue(1), P.CoordinatesValue(2), P.CoordinatesValue(3));

        // scale according to units
        if (Context is not null)
        {
            StepRepr_GlobalUnitAssignedContext theGUAC = null;
            if (Context.IsKind(typeof(StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext)))
            {
                StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext theGRCAGAUC =
                    Context as StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext;
                theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
            }
            else if (Context.IsKind(typeof(StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx)))
            {
                StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx theGRCAGAUC =
                    Context as StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx;
                theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
            }
            if (theGUAC is not null)
            {
                STEPConstruct_UnitContext UnitTool = new STEPConstruct_UnitContext();
                UnitTool.ComputeFactors(theGUAC, theLocalFactors);
                gp_Pnt zero = new gp_Pnt(0, 0, 0);
                pos.Scale(zero, UnitTool.LengthFactor());
            }
        }
        Pnt = pos;
        return true;
    }

    /// <summary>Sets current assembly shape SDR (for FindCDSR calls)</summary>
    public void SetAssemblyShape(TopoDS_Shape shape)
    {
        TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(FinderProcess(), shape);
        FinderProcess().FindTypedTransient(mapper, typeof(StepBasic_ProductDefinition), ref myAssemblyPD);
    }

    private readonly StepBasic_Unit areaUnit = new StepBasic_Unit();
    private readonly StepBasic_Unit volUnit = new StepBasic_Unit();
    private StepBasic_ProductDefinition myAssemblyPD;
}
