// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ValidationProps.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ValidationProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Basic.SiPrefix;
using static Kerf.Engine.Exchange.Step.Basic.SiUnitName;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// This class provides tools for access (write and read) the validation properties on shapes in the STEP file. These
/// are surface area, solid volume and centroid.
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
internal class ValidationProps : ConstructTool
{
    /// <summary>Creates an empty tool</summary>
    public ValidationProps()
    {
    }

    /// <summary>Creates a tool and loads it with worksession</summary>
    public ValidationProps(ExchangeWorkSession ws) : base(ws)
    {
    }

    /// <summary>Load worksession; returns True if succeeded</summary>
    public bool Init(ExchangeWorkSession ws)
    {
        return SetWS(ws);
    }

    private static SimpleBinderOfTransient TransientResult(Transient res)
    {
        SimpleBinderOfTransient binder = null;
        if (res is null)
        {
            return binder;
        }
        binder = new SimpleBinderOfTransient();
        binder.SetResult(res);
        return binder;
    }

    /// <summary>
    /// Finds target STEP entity to which validation props should be assigned, and corresponding context, starting
    /// from shape Returns True if success, False in case of fail
    /// </summary>
    public bool FindTarget(Shape shape, CharacterizedDefinition target,
                           ref RepresentationContext context, bool instance = false)
    {
        // find the target STEP entity corresponding to a shape
        ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(FinderProcess(), shape);
        Binder binder = FinderProcess().Find(mapper);

        // if requested, try to find instance of assembly
        if (instance)
        {
            // OCCT: the search of an instance is commented out
        }

        // for Compounds, search for SDR
        else if (shape.Type == ShapeType.Compound)
        {
            ProductDefinition prodDef = null;
            if (FinderProcess().FindTypedTransient(mapper, typeof(ProductDefinition), ref prodDef))
            {
                ProductDefinitionShape PDS = null;
                EntityIterator subs1 = Graph().Sharings(prodDef);
                for (subs1.Start(); PDS is null && subs1.More(); subs1.Next())
                {
                    PDS = subs1.Value() as ProductDefinitionShape;
                }
                target.SetValue(PDS);
            }
            else
            {
                StepShapeRepresentation SR = null;
                if (FinderProcess().FindTypedTransient(mapper, typeof(StepShapeRepresentation), ref SR))
                {
                    ProductDefinitionShape PDS = null;
                    EntityIterator subs1 = Graph().Sharings(SR);
                    for (subs1.Start(); PDS is null && subs1.More(); subs1.Next())
                    {
                        StepShapeDefinitionRepresentation SDR = subs1.Value() as StepShapeDefinitionRepresentation;
                        if (SDR is null)
                        {
                            continue;
                        }
                        PDS = SDR.Definition().PropertyDefinition() as ProductDefinitionShape;
                    }
                    if (PDS is null)
                    {
                        subs1 = Graph().Sharings(SR);
                        for (subs1.Start(); PDS is null && subs1.More(); subs1.Next())
                        {
                            RepresentationRelationship rr = subs1.Value() as RepresentationRelationship;
                            if (rr is null)
                            {
                                continue;
                            }
                            StepShapeRepresentation sr1;
                            if (rr.Rep1() == SR)
                            {
                                sr1 = rr.Rep2() as StepShapeRepresentation;
                            }
                            else
                            {
                                sr1 = rr.Rep1() as StepShapeRepresentation;
                            }
                            if (sr1 is not null)
                            {
                                EntityIterator subs2 = Graph().Sharings(sr1);
                                for (subs2.Start(); PDS is null && subs2.More(); subs2.Next())
                                {
                                    StepShapeDefinitionRepresentation SDR = subs2.Value() as StepShapeDefinitionRepresentation;
                                    if (SDR is null)
                                    {
                                        continue;
                                    }
                                    PDS = SDR.Definition().PropertyDefinition() as ProductDefinitionShape;
                                }
                            }
                        }
                    }
                    if (PDS is not null)
                    {
                        target.SetValue(PDS);
                        context = SR.ContextOfItems();
                    }
                }
            }
        }

        // for others, search for GEOMETRIC_REPRESENTATION_ITEM
        else
        {
            StepGeometricRepresentationItem item = null;
            if (FinderProcess().FindTypedTransient(mapper, typeof(StepGeometricRepresentationItem), ref item))
            {
                // find PDS (GRI <- SR <- SDR -> PDS)
                ProductDefinitionShape PDS = null;
                EntityIterator subs = Graph().Sharings(item);
                for (subs.Start(); PDS is null && subs.More(); subs.Next())
                {
                    if (!subs.Value().IsKind(typeof(StepShapeRepresentation)))
                    {
                        continue;
                    }
                    StepShapeRepresentation sr = subs.Value() as StepShapeRepresentation;
                    context = sr.ContextOfItems();
                    EntityIterator sub2 = Graph().Sharings(subs.Value());
                    for (sub2.Start(); sub2.More(); sub2.Next())
                    {
                        if (!sub2.Value().IsKind(typeof(StepShapeDefinitionRepresentation)))
                        {
                            continue;
                        }
                        StepShapeDefinitionRepresentation sdr = sub2.Value() as StepShapeDefinitionRepresentation;
                        PDS = sdr.Definition().PropertyDefinition() as ProductDefinitionShape;
                    }
                }
                if (PDS is not null)
                {
                    // find SHAPE_ASPECT or create it with all associated info if not yet exists
                    ShapeAspect aspect = null;
                    Binder bbb = binder;
                    while (bbb is not null)
                    {
                        SimpleBinderOfTransient bx = bbb as SimpleBinderOfTransient;
                        if (bx is not null)
                        {
                            ShapeAspect asp = bx.Result() as ShapeAspect;
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
                        TCollection_HAsciiString aspectName = new TCollection_HAsciiString("");
                        TCollection_HAsciiString aspectDescr = new TCollection_HAsciiString("");
                        aspect = new ShapeAspect();
                        aspect.Init(aspectName, aspectDescr, PDS, Logical.LFalse);

                        CharacterizedDefinition sa = new CharacterizedDefinition();
                        sa.SetValue(aspect);

                        TCollection_HAsciiString propDefName = new TCollection_HAsciiString("shape with specific properties");
                        TCollection_HAsciiString propDefDescr = new TCollection_HAsciiString("properties for subshape");
                        PropertyDefinition propdef = new PropertyDefinition();
                        propdef.Init(propDefName, true, propDefDescr, sa);

                        TCollection_HAsciiString srName = new TCollection_HAsciiString("");
                        StepShapeRepresentation SR = new StepShapeRepresentation();
                        NCollection_HArray1<RepresentationItem> srItems = new NCollection_HArray1<RepresentationItem>(1, 1);
                        srItems.SetValue(1, item);
                        SR.Init(srName, srItems, context);

                        StepShapeDefinitionRepresentation SDR = new StepShapeDefinitionRepresentation();
                        RepresentedDefinition rd = new RepresentedDefinition();
                        rd.SetValue(propdef);
                        SDR.Init(rd, SR);

                        // record SHAPE_ASPECT in the map
                        binder.AddResult(TransientResult(aspect));

                        // add SDR and all the data into model
                        Model().AddWithRefs(SDR);
                    }
                    // SHAPE_ASPECT found, but we also need context: FIND IT !!!!
                    else
                    {
                        ProductDefinitionShape pds = aspect.OfShape();
                        EntityIterator asubs = Graph().Sharings(pds);
                        for (asubs.Start(); context is null && asubs.More(); asubs.Next())
                        {
                            if (!asubs.Value().IsKind(typeof(StepShapeDefinitionRepresentation)))
                            {
                                continue;
                            }
                            StepShapeDefinitionRepresentation sdr = asubs.Value() as StepShapeDefinitionRepresentation;
                            context = sdr.UsedRepresentation().ContextOfItems();
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
        return !target.IsNull;
    }

    /// <summary>
    /// General method for adding (writing) a validation property for shape which should be already mapped on writing
    /// itself. It takes target and Context entities which correspond to shape Returns True if success, False in case
    /// of fail
    /// </summary>
    public bool AddProp(CharacterizedDefinition target, RepresentationContext context,
                        RepresentationItem prop, string descr)
    {
        // FINALLY, create a structure of 5 entities describing a link between a shape and its property
        TCollection_HAsciiString propDefName = new TCollection_HAsciiString("geometric validation property");
        TCollection_HAsciiString propDefDescr = new TCollection_HAsciiString(descr);
        PropertyDefinition propdef = new PropertyDefinition();
        propdef.Init(propDefName, true, propDefDescr, target);

        TCollection_HAsciiString srName = new TCollection_HAsciiString(descr);
        StepRepresentation rep = new StepRepresentation();
        NCollection_HArray1<RepresentationItem> srItems = new NCollection_HArray1<RepresentationItem>(1, 1);
        srItems.SetValue(1, prop);
        rep.Init(srName, srItems, context);

        PropertyDefinitionRepresentation prDR = new PropertyDefinitionRepresentation();
        RepresentedDefinition rd = new RepresentedDefinition();
        rd.SetValue(propdef);
        prDR.Init(rd, rep);

        // record SDR in order to have it written to the file
        Model().AddWithRefs(prDR);

        StepModel stepModel = Model() as StepModel;
        // for AP203, add subschema name
        if (stepModel.InternalParameters.WriteSchema == Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203)
        {
            MakeHeader mkHdr = new MakeHeader(stepModel);
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
    public bool AddProp(Shape shape, RepresentationItem prop, string descr, bool instance = false)
    {
        CharacterizedDefinition target = new CharacterizedDefinition();
        RepresentationContext context = null;
        if (!FindTarget(shape, target, ref context, instance))
        {
            return false;
        }
        return AddProp(target, context, prop, descr);
    }

    /// <summary>Adds volume property for given shape (already mapped). Returns True if success, False in case of fail</summary>
    public bool AddVolume(Shape shape, double vol)
    {
        MeasureValueMember val = new MeasureValueMember();
        val.SetReal(vol);
        // Val->SetName ( "solid volume" );
        val.SetName("VOLUME_MEASURE");

        // for volume unit, either take existing or create a new
        if (_volUnit.DerivedUnit() is null)
        {
            SiUnitAndLengthUnit mm = new SiUnitAndLengthUnit();
            mm.Init(true, SiPrefix.SpMilli, SiUnitName.SunMetre);

            DerivedUnitElement due = new DerivedUnitElement();
            due.Init(mm, 3.0);

            NCollection_HArray1<DerivedUnitElement> duElems = new NCollection_HArray1<DerivedUnitElement>(1, 1);
            duElems.SetValue(1, due);

            DerivedUnit du = new DerivedUnit();
            du.Init(duElems);

            _volUnit.SetValue(du);
        }

        TCollection_HAsciiString mriName = new TCollection_HAsciiString("volume measure");
        MeasureRepresentationItem mri = new MeasureRepresentationItem();
        mri.Init(mriName, val, new Unit(_volUnit));

        return AddProp(shape, mri, "volume");
    }

    /// <summary>Adds surface area property for given shape (already mapped). Returns True if success, False in case of fail</summary>
    public bool AddArea(Shape shape, double area)
    {
        MeasureValueMember val = new MeasureValueMember();
        val.SetReal(area);
        // Val->SetName ( "surface area" );
        val.SetName("AREA_MEASURE");

        // for area unit, either take existing or create a new
        if (_areaUnit.DerivedUnit() is null)
        {
            SiUnitAndLengthUnit mm = new SiUnitAndLengthUnit();
            mm.Init(true, SiPrefix.SpMilli, SiUnitName.SunMetre);

            DerivedUnitElement due = new DerivedUnitElement();
            due.Init(mm, 2.0);

            NCollection_HArray1<DerivedUnitElement> duElems = new NCollection_HArray1<DerivedUnitElement>(1, 1);
            duElems.SetValue(1, due);

            DerivedUnit du = new DerivedUnit();
            du.Init(duElems);

            _areaUnit.SetValue(du);
        }

        TCollection_HAsciiString mriName = new TCollection_HAsciiString("surface area measure");
        MeasureRepresentationItem mri = new MeasureRepresentationItem();
        mri.Init(mriName, val, new Unit(_areaUnit));

        return AddProp(shape, mri, "surface area");
    }

    /// <summary>
    /// Adds centroid property for given shape (already mapped). Returns True if success, False in case of fail If
    /// instance is True, then centroid is assigned to an instance of component in assembly
    /// </summary>
    public bool AddCentroid(Shape shape, in Point3D pnt, bool instance = false)
    {
        TCollection_HAsciiString cpName = new TCollection_HAsciiString("centre point");
        StepCartesianPoint cp = new StepCartesianPoint();
        cp.Init3D(cpName, pnt.X, pnt.Y, pnt.Z);

        return AddProp(shape, cp, "centroid", instance);
    }

    /// <summary>
    /// Searches for entities of the type PropertyDefinitionRepresentation in the model and fills the sequence by them
    /// </summary>
    public bool LoadProps(NCollection_Sequence<Transient> seq)
    {
        // parse on PropertyDefinitionRepresentations
        int nb = Model().EntityCount;
        Type tPDR = typeof(PropertyDefinitionRepresentation);
        for (int i = 1; i <= nb; i++)
        {
            Transient enti = Model().Value(i);
            if (!enti.IsKind(tPDR))
            {
                continue;
            }

            PropertyDefinitionRepresentation pdr = enti as PropertyDefinitionRepresentation;

            // Check that PDR is for validation props.
            PropertyDefinition pd = pdr.Definition().PropertyDefinition();
            if (pd is not null && pd.Name() is not null)
            {
                // Note: according to "Recommended Practices for Geometric and Assembly Validation Properties"
                // Release 4.4 as of Augist 17, 2016, item 4.6, the name of PropertyDefinition should be
                // "geometric validation property" with words separated by spaces; however older versions of
                // the same RP document used underscores. To be able to read files written using older
                // convention, we convert all underscores to spaces for this check.
                string name = pd.Name().ToCString().Replace('_', ' ').ToLowerInvariant();
                if (name != "geometric validation property")
                {
                    continue;
                }
            }

            seq.Append(pdr);
        }
        return seq.Length() > 0;
    }

    /// <summary>Returns CDSR associated with given PpD or NULL if not found (when, try GetPropSDR)</summary>
    public NextAssemblyUsageOccurrence GetPropNAUO(PropertyDefinition pd)
    {
        CharacterizedDefinition cd = pd.Definition();

        // detect target entity of valprop
        NextAssemblyUsageOccurrence nauo = null;
        PropertyDefinition pds = cd.ProductDefinitionShape();
        if (pds is null)
        {
            return nauo; // not found
        }
        EntityIterator subs = Graph().Shareds(pds);
        for (subs.Start(); nauo is null && subs.More(); subs.Next())
        {
            nauo = subs.Value() as NextAssemblyUsageOccurrence;
        }
        return nauo;
    }

    /// <summary>Returns SDR associated with given PpD or NULL if not found (when, try GetPropCDSR)</summary>
    public ProductDefinition GetPropPD(PropertyDefinition pd)
    {
        CharacterizedDefinition cd = pd.Definition();

        // detect target entity of valprop
        ProductDefinition prodDef = null;
        PropertyDefinition pds = cd.ProductDefinitionShape();
        if (pds is null)
        {
            ShapeAspect sa = cd.ShapeAspect();
            if (sa is null)
            {
                // OCCT_DEBUG output left out
                return prodDef;
            }
            EntityIterator subs = Graph().Sharings(sa);
            for (subs.Start(); subs.More(); subs.Next())
            {
                pds = subs.Value() as PropertyDefinition;
                if (pds is null)
                {
                    return prodDef;
                }
                EntityIterator subs1 = Graph().Shareds(pds);
                for (subs1.Start(); prodDef is null && subs1.More(); subs1.Next())
                {
                    prodDef = subs1.Value() as ProductDefinition;
                }
            }
        }
        else
        {
            EntityIterator subs = Graph().Shareds(pds);
            for (subs.Start(); prodDef is null && subs.More(); subs.Next())
            {
                prodDef = subs.Value() as ProductDefinition;
            }
        }
        // OCCT_DEBUG output left out
        return prodDef;
    }

    /// <summary>Returns Shape associated with given SDR or Null Shape if not found</summary>
    public Shape GetPropShape(ProductDefinition prodDef)
    {
        // find target shape
        Shape s = new Shape();
        Binder binder = TransientProcess().Find(prodDef);
        if (binder is not null && binder.HasResult)
        {
            s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TransientProcess(), binder);
        }
        // OCCT_DEBUG output left out
        return s;
    }

    /// <summary>Returns Shape associated with given PpD or Null Shape if not found</summary>
    public Shape GetPropShape(PropertyDefinition pd)
    {
        ProductDefinition prodDef = GetPropPD(pd);
        Shape s = new Shape();
        if (prodDef is not null)
        {
            s = GetPropShape(prodDef);
        }
        return s;
    }

    /// <summary>
    /// Returns value of Real-Valued property (Area or Volume) If Property is neither Area nor Volume, returns False
    /// Else returns True and isArea indicates whether property is area or volume
    /// </summary>
    public bool GetPropReal(RepresentationItem item, out double val, out bool isArea, in Factors localFactors)
    {
        val = 0.0;
        isArea = false;
        // decode volume & area
        if (!item.IsKind(typeof(MeasureRepresentationItem)))
        {
            return false;
        }

        MeasureRepresentationItem mri = item as MeasureRepresentationItem;

        MeasureWithUnit m = mri.Measure();
        string name = m.ValueComponentMember().Name();
        Unit Unit = m.UnitComponent();

        double scale = 1.0;
        DerivedUnit du = Unit.DerivedUnit();
        if (du is not null)
        {
            for (int ind = 1; ind <= du.ElementCount; ind++)
            {
                DerivedUnitElement due = du.ElementsValue(ind);
                double exp = due.Exponent();
                NamedUnit nu = due.Unit();
                UnitContext unit = new UnitContext();
                unit.ComputeFactors(nu, localFactors);
                if (unit.LengthDone())
                {
                    double lengthFactor = unit.LengthFactor();
                    scale *= Math.Pow(lengthFactor, exp);
                }
            }
        }
        else
        {
            NamedUnit nu = Unit.NamedUnit();
            if (nu is not null)
            {
                UnitContext unit = new UnitContext();
                unit.ComputeFactors(nu, localFactors);
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

        val = m.ValueComponent() * scale;

        if (name == "AREA_MEASURE")
        {
            isArea = true;
        }
        else if (name == "VOLUME_MEASURE")
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
    public bool GetPropPnt(RepresentationItem item, RepresentationContext context, out Point3D pnt,
                           in Factors localFactors)
    {
        pnt = new Point3D();
        // centroid
        if (!item.IsKind(typeof(StepCartesianPoint)))
        {
            return false;
        }

        StepCartesianPoint p = item as StepCartesianPoint;
        if (p is null || p.CoordinateCount != 3)
        {
            // OCCT_DEBUG output left out
            return false;
        }

        Point3D pos = new Point3D(p.CoordinatesValue(1), p.CoordinatesValue(2), p.CoordinatesValue(3));

        // scale according to units
        if (context is not null)
        {
            GlobalUnitAssignedContext theGUAC = null;
            if (context.IsKind(typeof(StepGeometricRepresentationContextAndGlobalUnitAssignedContext)))
            {
                StepGeometricRepresentationContextAndGlobalUnitAssignedContext theGRCAGAUC =
                    context as StepGeometricRepresentationContextAndGlobalUnitAssignedContext;
                theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
            }
            else if (context.IsKind(typeof(StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx)))
            {
                StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx theGRCAGAUC =
                    context as StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx;
                theGUAC = theGRCAGAUC.GlobalUnitAssignedContext();
            }
            if (theGUAC is not null)
            {
                UnitContext unitTool = new UnitContext();
                unitTool.ComputeFactors(theGUAC, localFactors);
                Point3D zero = new Point3D(0, 0, 0);
                pos = pos.Scaled(zero, unitTool.LengthFactor());
            }
        }
        pnt = pos;
        return true;
    }

    /// <summary>Sets current assembly shape SDR (for FindCDSR calls)</summary>
    public void SetAssemblyShape(Shape shape)
    {
        ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(FinderProcess(), shape);
        FinderProcess().FindTypedTransient(mapper, typeof(ProductDefinition), ref _assemblyPD);
    }

    private readonly Unit _areaUnit = new Unit();
    private readonly Unit _volUnit = new Unit();
    private ProductDefinition _assemblyPD;
}
