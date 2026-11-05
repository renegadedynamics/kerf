// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Reader.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Import;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Selections;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;
using static Kerf.Engine.Exchange.Step.Basic.SiPrefix;
using static Kerf.Engine.Exchange.Step.Basic.SiUnitName;

namespace Kerf.Engine.Exchange.Step;

/// <summary>
/// Reads STEP files, checks them and translates their contents into Open CASCADE models. The STEP data can be that
/// of a whole model or that of a specific list of entities in the model. As in XSControl_Reader, you specify the
/// list using a selection. For the translation of iges files it is possible to use next sequence: To change
/// translation parameters class Interface_Static should be used before beginning of translation. Creation of
/// reader - STEPControl_Reader reader; To load s file in a model use method reader.ReadFile("filename.stp") To print
/// load results reader.PrintCheckLoad(failsonly,mode) where mode is equal to the value of enumeration
/// IFSelect_PrintCount To transfer entities from a model the following methods can be used: for the whole model -
/// reader.TransferRoots(); to transfer a list of entities: reader.TransferList(list); to transfer one entity
/// reader.TransferEntity(ent) or reader.Transfer(num); To obtain a result the following method can be used:
/// reader.Shape() or reader.OneShape(); To print the results of transfer use method
/// reader.PrintCheckTransfer(failwarn,mode); where failwarn is equal to the value of enumeration
/// IFSelect_PrintFail, mode see above; or reader.PrintStatsTransfer(); Gets correspondence between a STEP entity
/// and a result shape obtained from it. occ::handle&lt;XSControl_WorkSession&gt; WS = reader.WS();
/// if ( WS-&gt;TransferReader()-&gt;HasResult(ent) ) TopoDS_Shape shape =
/// WS-&gt;TransferReader()-&gt;ShapeResult(ent);
/// </summary>
/// <remarks>
/// <para>Kerf (plan 2.7, decisions 1225, 1265):</para>
/// <list type="bullet">
/// <item>the parameters come from the session's parameter set (<see cref="WorkSession.Statics"/>), where
/// OCCT reads the process-wide statics; or from a <see cref="Parameters"/> given to the read;</item>
/// <item>files and streams are read with a progress range (cancellation) and the parser's limits for hostile input
/// (<see cref="ParserReadLimits"/>, the defaults when none is given);</item>
/// <item>after a successful read the model's length unit is set from the session's <c>xstep.cascade.unit</c> (OCCT
/// takes the process-wide unit at the first transfer);</item>
/// <item>the shapes the reader publishes (<c>Shape</c>, <c>OneShape</c>) are frozen (decision 1224).</item>
/// </list>
/// </remarks>
internal class StepReader : FrameworkReader
{
    /// <summary>Creates a reader object with an empty STEP model.</summary>
    public StepReader()
    {
        StepController.Init();
        SelectStep();
    }

    /// <summary>Creates a Reader for STEP from an already existing Session Clears the session if it was not yet set for STEP</summary>
    public StepReader(ExchangeWorkSession ws, bool scratch = true) : base(InitFirst(ws), scratch)
    {
        SelectStep();
    }

    // OCCT: SetNorm("STEP"), which selects the controller recorded process-wide; here one of the session's own
    // (STEPControl_Controller.SelectFor, decision 1264), then the transfer reader initialised as SetNorm does
    private void SelectStep()
    {
        if (WS() is null)
        {
            SetWS(new ExchangeWorkSession());
        }
        if (StepController.SelectFor(WS()))
        {
            WS().InitTransferReader(0);
            WS().InitTransferReader(4);
        }
    }

    // OCCT calls STEPControl_Controller::Init() before SetWS: the base constructor calls SetWS
    private static ExchangeWorkSession InitFirst(ExchangeWorkSession ws)
    {
        StepController.Init();
        return ws;
    }

    /// <summary>Returns the model as a StepModel. It can then be consulted (header, product)</summary>
    public StepModel StepModel()
    {
        return Model() as StepModel;
    }

    /// <summary>Loads a file and returns the read status Zero for a Model which complies with the Controller</summary>
    public override ReturnStatus ReadFile(string filename, ProgressRange progress = default)
    {
        return ReadFile(filename, progress, null);
    }

    /// <summary>As <see cref="ReadFile(string, ProgressRange)"/>, with the parser's limits (Kerf).</summary>
    public ReturnStatus ReadFile(string filename, ProgressRange progress, ParserReadLimits limits)
    {
        return ReadInto(filename, null, ModelParameters(), progress, limits);
    }

    /// <summary>Loads a file from stream and returns the read status</summary>
    public override ReturnStatus ReadStream(string name, Stream iStream, ProgressRange progress = default)
    {
        return ReadStream(name, iStream, progress, null);
    }

    /// <summary>As <see cref="ReadStream(string, Stream, ProgressRange)"/>, with the parser's limits (Kerf).</summary>
    public ReturnStatus ReadStream(string name, Stream iStream, ProgressRange progress,
                                            ParserReadLimits limits)
    {
        return ReadInto(name, iStream, ModelParameters(), progress, limits);
    }

    /// <summary>Loads a file and returns the read status Zero for a Model which complies with the Controller</summary>
    public ReturnStatus ReadFile(string filename, in Parameters theParams, ProgressRange progress = default,
                                          ParserReadLimits limits = null)
    {
        return ReadInto(filename, null, theParams, progress, limits);
    }

    /// <summary>Loads a file from stream and returns the read status</summary>
    public ReturnStatus ReadStream(string name, in Parameters theParams, Stream iStream,
                                            ProgressRange progress = default, ParserReadLimits limits = null)
    {
        return ReadInto(name, iStream, theParams, progress, limits);
    }

    // OCCT: InternalParameters.InitFromStatic(), from the process-wide statics; here the session's
    private Parameters ModelParameters()
    {
        Parameters aParams = new Parameters();
        aParams.InitFromStatic(WS().Statics());
        return aParams;
    }

    // The four ReadFile/ReadStream bodies of OCCT, which differ only in the parameters and the stream
    private ReturnStatus ReadInto(string name, Stream iStream, in Parameters theParams,
                                           ProgressRange progress, ParserReadLimits limits)
    {
        WorkLibrary library = WS().WorkLibrary();
        Protocol protocol = WS().Protocol();
        if (library is null)
        {
            return ReturnStatus.RetVoid;
        }
        if (protocol is null)
        {
            return ReturnStatus.RetVoid;
        }
        StepModel stepModel = new StepModel();
        stepModel.InternalParameters = theParams;
        stepModel.SetSourceCodePage(stepModel.InternalParameters.ReadCodePage);
        MakeHeader headerMaker = new MakeHeader();
        headerMaker.Apply(stepModel);
        ReturnStatus status = ReturnStatus.RetVoid;
        try
        {
            int stat;
            if (library is StepWorkLibrary stepLibrary)
            {
                stat = iStream is null
                    ? stepLibrary.ReadFile(name, stepModel, protocol, progress, limits)
                    : stepLibrary.ReadStream(name, iStream, stepModel, protocol, progress, limits);
            }
            else
            {
                InterfaceModel model = stepModel;
                stat = iStream is null
                    ? library.ReadFile(name, ref model, protocol, progress)
                    : library.ReadStream(name, iStream, ref model, protocol, progress);
            }
            if (stat == 0)
            {
                status = ReturnStatus.RetDone;
            }
            else if (stat < 0)
            {
                status = ReturnStatus.RetError;
            }
            else
            {
                status = ReturnStatus.RetFail;
            }
        }
        catch (Exception exception) when (Refusal.IsFailure(exception) || ImportRoot.CaughtBySignals(exception))
        {
            using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
            sout.Append("    ****    Interruption ReadFile par Exception :   ****\n");
            sout.Append(Refusal.Unwrapped(exception).Message);
            sout.Append("\n    Abandon").Append('\n');
            status = ReturnStatus.RetFail;
        }
        if (status != ReturnStatus.RetDone)
        {
            return status;
        }
        // Kerf: the length unit, from the session's statics (decision 1265)
        if (!stepModel.IsInitializedUnit)
        {
            stepModel.SetLocalLengthUnit(global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(WS().Statics()));
        }
        WS().SetModel(stepModel);
        WS().SetLoadedFile(name);
        WS().InitTransferReader(4);
        return status;
    }

    /// <summary>Transfers a root given its rank in the list of candidate roots Default is the first one Returns True if a shape has resulted, false else Same as inherited TransferOneRoot, kept for compatibility</summary>
    public bool TransferRoot(int num = 1, ProgressRange progress = default)
    {
        return TransferOneRoot(num, progress);
    }

    /// <summary>Determines the list of root entities from Model which are candidate for a transfer to a Shape (type of entities is PRODUCT)</summary>
    public override int NbRootsForTransfer()
    {
        if (_therootsta)
        {
            return _theroots.Length();
        }
        _therootsta = true;

        StepModel stepModel = WS().Model() as StepModel;
        // theroots.Clear();
        int nb = Model().EntityCount;
        for (int i = 1; i <= nb; i++)
        {
            Transient ent = Model().Value(i);
            if (stepModel.InternalParameters.ReadAllShapes)
            {
                // Special case to read invalid shape_representation without links to shapes.
                if (ent.IsKind(typeof(StepManifoldSolidBrep)))
                {
                    EntityIterator shareds = WS().Graph().Sharings(ent);
                    if (!shareds.More())
                    {
                        _theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
                if (ent.IsKind(typeof(StepShellBasedSurfaceModel)))
                {
                    EntityIterator shareds = WS().Graph().Sharings(ent);
                    if (!shareds.More())
                    {
                        _theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
            }
            if (ent.IsKind(typeof(ProductDefinition)))
            {
                // PTV 31.01.2003 TRJ11 exclude Product Definition With Associated Document from roots
                if (ent.IsKind(typeof(ProductDefinitionWithAssociatedDocuments)))
                {
                    // check if PDWAD-> PDF <-Document_Product_Equivalence.
                    bool iSexclude = false;
                    ProductDefinitionWithAssociatedDocuments pdwad = ent as ProductDefinitionWithAssociatedDocuments;
                    EntityIterator pdwaDsubs = WS().Graph().Shareds(pdwad);
                    for (pdwaDsubs.Start(); pdwaDsubs.More(); pdwaDsubs.Next())
                    {
                        if (!pdwaDsubs.Value().IsKind(typeof(ProductDefinitionFormation)))
                        {
                            continue;
                        }
                        ProductDefinitionFormation localPDF = pdwaDsubs.Value() as ProductDefinitionFormation;
                        EntityIterator pdFsubs = WS().Graph().Sharings(localPDF);
                        for (pdFsubs.Start(); pdFsubs.More(); pdFsubs.Next())
                        {
                            if (pdFsubs.Value().IsKind(typeof(DocumentProductEquivalence)))
                            {
                                iSexclude = true;
                                break;
                            }
                        }
                        if (iSexclude)
                        {
                            break;
                        }
                    }
                    if (iSexclude)
                    {
                        continue;
                    }
                }
                ProductDefinition pd = ent as ProductDefinition;
                bool isRoot = true;
                EntityGraph graph = WS().Graph();
                // determinate roots used NextAssemblyUsageOccurrence
                EntityIterator subs = graph.Sharings(pd);
                for (subs.Start(); subs.More(); subs.Next())
                {
                    NextAssemblyUsageOccurrence nauo = subs.Value() as NextAssemblyUsageOccurrence;
                    if (nauo is null)
                    {
                        continue;
                    }
                    if (pd == nauo.RelatedProductDefinition())
                    {
                        isRoot = false;
                    }
                }
                // determinate roots used ProductDefinitionContext
                if (isRoot)
                {
                    Parameters.ReadMode_ProductContext prodContMode = stepModel.InternalParameters.ReadProductContext;
                    string str1 = stepModel.InternalParameters.GetString(prodContMode);
                    int ics = (int)stepModel.InternalParameters.ReadProductContext;
                    if (ics > 1)
                    {
                        subs = graph.Shareds(pd);
                        for (subs.Start(); subs.More(); subs.Next())
                        {
                            ProductDefinitionContext pdc = subs.Value() as ProductDefinitionContext;
                            if (pdc is null)
                            {
                                continue;
                            }
                            string str2 = pdc.LifeCycleStage().String();
                            string str3 = pdc.Name().String();
                            if (!string.Equals(str1, str2, StringComparison.OrdinalIgnoreCase)
                                && !string.Equals(str1, str3, StringComparison.OrdinalIgnoreCase))
                            {
                                isRoot = false;
                            }
                        }
                    }
                }
                if (isRoot)
                {
                    _theroots.Append(ent);
                    WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                }
            }
            bool prodMode = stepModel.InternalParameters.ReadProductMode;
            if (!prodMode)
            {
                if (ent.IsKind(typeof(StepShapeDefinitionRepresentation)))
                {
                    bool isRoot = true;
                    StepShapeDefinitionRepresentation sdr = ent as StepShapeDefinitionRepresentation;
                    PropertyDefinition propDef = sdr.Definition().PropertyDefinition();
                    if (propDef is not null)
                    {
                        ProductDefinition pd = propDef.Definition().ProductDefinition();
                        if (pd is not null)
                        {
                            isRoot = false;
                        }
                        if (isRoot)
                        {
                            ShapeAspect sa = propDef.Definition().ShapeAspect();
                            if (sa is not null)
                            {
                                ProductDefinitionShape pds = sa.OfShape();
                                pd = pds.Definition().ProductDefinition();
                                if (pd is not null)
                                {
                                    isRoot = false;
                                }
                            }
                        }
                        if (isRoot)
                        {
                            NextAssemblyUsageOccurrence nauo =
                                propDef.Definition().ProductDefinitionRelationship() as NextAssemblyUsageOccurrence;
                            if (nauo is not null)
                            {
                                isRoot = false;
                            }
                        }
                        if (isRoot)
                        {
                            StepShapeRepresentation sr = sdr.UsedRepresentation() as StepShapeRepresentation;
                            if (sr is null)
                            {
                                isRoot = false;
                            }
                        }
                    }
                    if (isRoot)
                    {
                        _theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
                if (ent.IsKind(typeof(StepShapeRepresentation)))
                {
                    bool isRoot = true;
                    StepShapeRepresentation sr = ent as StepShapeRepresentation;
                    EntityGraph graph = WS().Graph();
                    EntityIterator subs = graph.Sharings(sr);
                    for (subs.Start(); subs.More() && isRoot; subs.Next())
                    {
                        StepShapeDefinitionRepresentation sdr = subs.Value() as StepShapeDefinitionRepresentation;
                        if (sdr is not null)
                        {
                            isRoot = false;
                        }
                        if (isRoot)
                        {
                            RepresentationRelationship rr = subs.Value() as RepresentationRelationship;
                            if (rr is not null)
                            {
                                StepShapeRepresentation sr2 = rr.Rep1() as StepShapeRepresentation;
                                if (sr == sr2)
                                {
                                    sr2 = rr.Rep2() as StepShapeRepresentation;
                                }
                                if (sr2 is not null)
                                {
                                    EntityIterator subs2 = graph.Sharings(sr2);
                                    for (subs2.Start(); subs2.More(); subs2.Next())
                                    {
                                        StepShapeDefinitionRepresentation sdr2 = subs2.Value() as StepShapeDefinitionRepresentation;
                                        if (sdr2 is not null)
                                        {
                                            isRoot = false;
                                        }
                                    }
                                }
                            }
                        }
                        if (isRoot)
                        {
                            RepresentationMap rm = subs.Value() as RepresentationMap;
                            if (rm is not null)
                            {
                                EntityIterator subs2 = graph.Sharings(rm);
                                for (subs2.Start(); subs2.More(); subs2.Next())
                                {
                                    MappedItem mi = subs2.Value() as MappedItem;
                                    if (mi is not null)
                                    {
                                        EntityIterator subs3 = graph.Sharings(mi);
                                        for (subs3.Start(); subs3.More(); subs3.Next())
                                        {
                                            StepShapeRepresentation sr2 = subs3.Value() as StepShapeRepresentation;
                                            if (sr2 is not null)
                                            {
                                                isRoot = false;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if (isRoot)
                    {
                        _theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
            }
        }

        return _theroots.Length();
    }

    /// <summary>
    /// Returns sequence of all unit names for shape representations found in file
    /// </summary>
    public void FileUnits(NCollection_Sequence<string> unitLengthNames, NCollection_Sequence<string> unitAngleNames,
                          NCollection_Sequence<string> unitSolidAngleNames)
    {
        int nbroots = NbRootsForTransfer();
        if (nbroots == 0)
        {
            return;
        }

        const int length = 0;
        const int anlge = 1;
        const int SOLID_ANGLE = 2;

        EntityGraph graph = WS().Graph();
        HashSet<string>[] mapUnits = { new HashSet<string>(), new HashSet<string>(), new HashSet<string>() };

        for (int i = 1; i <= nbroots; i++)
        {
            Transient ent = _theroots.Value(i);
            int num = graph.EntityNumber(ent);
            if (num == 0)
            {
                continue;
            }
            ProductDefinition prodDef = ent as ProductDefinition;
            StepShapeDefinitionRepresentation shapeDefRepr = null;
            if (prodDef is not null)
            {
                EntityIterator subsPD = graph.Sharings(prodDef);
                for (subsPD.Start(); subsPD.More() && shapeDefRepr is null; subsPD.Next())
                {
                    ProductDefinitionShape prodDefShape = subsPD.Value() as ProductDefinitionShape;
                    if (prodDefShape is null)
                    {
                        continue;
                    }
                    EntityIterator subsSR = graph.Sharings(prodDefShape);
                    for (subsSR.Start(); subsSR.More() && shapeDefRepr is null; subsSR.Next())
                    {
                        StepShapeDefinitionRepresentation curShapeDefRepr = subsSR.Value() as StepShapeDefinitionRepresentation;
                        if (curShapeDefRepr is null)
                        {
                            continue;
                        }
                        StepRepresentation useRepr = curShapeDefRepr.UsedRepresentation();
                        if (useRepr is null)
                        {
                            continue;
                        }
                        StepShapeRepresentation shapeRepr = useRepr as StepShapeRepresentation;
                        if (shapeRepr is null)
                        {
                            continue;
                        }
                        shapeDefRepr = curShapeDefRepr;
                    }
                }
            }
            else
            {
                shapeDefRepr = ent as StepShapeDefinitionRepresentation;
            }
            if (shapeDefRepr is not null)
            {
                StepShapeRepresentation shapeRepr = shapeDefRepr.UsedRepresentation() as StepShapeRepresentation;
                RepresentationContext repCont = shapeRepr.ContextOfItems();
                if (repCont is null)
                {
                    continue;
                }
                string[] nameUnits = { "", "", "" };
                double[] factorUnits = new double[3];
                if (FindUnits(repCont, nameUnits, factorUnits))
                {
                    for (int k = length; k <= SOLID_ANGLE; k++)
                    {
                        if (mapUnits[k].Add(nameUnits[k]))
                        {
                            NCollection_Sequence<string> unitSeq =
                                (k == length ? unitLengthNames : (k == anlge ? unitAngleNames : unitSolidAngleNames));
                            unitSeq.Append(nameUnits[k]);
                        }
                    }
                }
            }
        }
        // for case when units was not found through PDF or SDR
        if (unitLengthNames.IsEmpty())
        {
            InterfaceModel model = WS().Model();
            if (model is null)
            {
                return;
            }
            int nb = model.EntityCount;
            for (int i = 1; i <= nb; i++)
            {
                Transient ent = model.Value(i);
                RepresentationContext repCont = ent as RepresentationContext;
                if (repCont is null)
                {
                    continue;
                }
                string[] nameUnits = { "", "", "" };
                double[] factorUnits = new double[3];
                if (FindUnits(repCont, nameUnits, factorUnits))
                {
                    for (int k = length; k <= SOLID_ANGLE; k++)
                    {
                        if (mapUnits[k].Add(nameUnits[k]))
                        {
                            NCollection_Sequence<string> unitSeq =
                                (k == length ? unitLengthNames : (k == anlge ? unitAngleNames : unitSolidAngleNames));
                            unitSeq.Append(nameUnits[k]);
                        }
                    }
                }
            }
        }
    }

    /// <summary>Sets system length unit used by transfer process. Performs only if a model is not NULL</summary>
    public void SetSystemLengthUnit(double lengthUnit)
    {
        if (StepModel() is null)
        {
            return;
        }
        StepModel().SetLocalLengthUnit(lengthUnit);
    }

    /// <summary>Returns system length unit used by transfer process. Performs only if a model is not NULL</summary>
    public double SystemLengthUnit()
    {
        if (StepModel() is null)
        {
            return 1.0;
        }
        return StepModel().LocalLengthUnit();
    }

    private static string GetSiName(SiUnit unit)
    {
        string name = "";
        if (unit.HasPrefix)
        {
            switch (unit.Prefix())
            {
                case SiPrefix.SpExa:
                    name += "exa";
                    break;
                case SiPrefix.SpPeta:
                    name += "peta";
                    break;
                case SiPrefix.SpTera:
                    name += "tera";
                    break;
                case SiPrefix.SpGiga:
                    name += "giga";
                    break;
                case SiPrefix.SpMega:
                    name += "mega";
                    break;
                case SiPrefix.SpHecto:
                    name += "hecto";
                    break;
                case SiPrefix.SpDeca:
                    name += "deca";
                    break;
                case SiPrefix.SpDeci:
                    name += "deci";
                    break;

                case SiPrefix.SpPico:
                    name += "pico";
                    break;
                case SiPrefix.SpFemto:
                    name += "femto";
                    break;
                case SiPrefix.SpAtto:
                    name += "atto";
                    break;

                case SiPrefix.SpKilo:
                    name += "kilo";
                    break;
                case SiPrefix.SpCenti:
                    name += "centi";
                    break;
                case SiPrefix.SpMilli:
                    name += "milli";
                    break;
                case SiPrefix.SpMicro:
                    name += "micro";
                    break;
                case SiPrefix.SpNano:
                    name += "nano";
                    break;
                default:
                    break;
            }
        }

        switch (unit.Name())
        {
            case SiUnitName.SunMetre:
                name += "metre";
                break;
            case SiUnitName.SunRadian:
                name += "radian";
                break;
            case SiUnitName.SunSteradian:
                name += "steradian";
                break;
            default:
                break;
        }
        return name;
    }

    protected override ShapeFixParameters GetDefaultShapeFixParameters()
    {
        return Parameters.GetDefaultShapeFixParameters();
    }

    protected override global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags GetDefaultShapeProcessFlags()
    {
        return new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.FixShape);
    }

    // OCCT's 1-based arrays of 3 units (length, plane angle, solid angle): 0-based here
    private bool FindUnits(RepresentationContext repCont, string[] nameUnits, double[] factorUnits)
    {
        GlobalUnitAssignedContext context = null;

        if (repCont.IsKind(typeof(StepGeometricRepresentationContextAndGlobalUnitAssignedContext)))
        {
            context = (repCont as StepGeometricRepresentationContextAndGlobalUnitAssignedContext).GlobalUnitAssignedContext();
        }

        if (repCont.IsKind(typeof(StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx)))
        {
            context = (repCont as StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx).GlobalUnitAssignedContext();
        }
        if (context is null)
        {
            return false;
        }
        // Start Computation
        int nbU = context.NbUnits();
        int nbFind = 0;
        for (int i = 1; i <= nbU; i++)
        {
            NamedUnit namedUnit = context.UnitsValue(i);
            ConversionBasedUnit convUnit = namedUnit as ConversionBasedUnit;
            int ind = 0;
            string name = "";
            double unitFact = 0;
            if (convUnit is not null)
            {
                MeasureWithUnit measWithUnit = null;
                Transient convFactor = convUnit.ConversionFactor();
                if (convFactor.IsKind(typeof(MeasureWithUnit)))
                {
                    measWithUnit = convFactor as MeasureWithUnit;
                }
                else if (convFactor.IsKind(typeof(ReprItemAndMeasureWithUnit)))
                {
                    ReprItemAndMeasureWithUnit reprMeasureItem = convFactor as ReprItemAndMeasureWithUnit;
                    measWithUnit = reprMeasureItem.GetMeasureWithUnit();
                }

                if (measWithUnit is null)
                {
                    continue;
                }

                if (measWithUnit.IsKind(typeof(LengthMeasureWithUnit)))
                {
                    ind = 1;
                }
                else if (measWithUnit.IsKind(typeof(PlaneAngleMeasureWithUnit)))
                {
                    ind = 2;
                }
                else if (measWithUnit.IsKind(typeof(SolidAngleMeasureWithUnit)))
                {
                    ind = 3;
                }
                if (ind == 0)
                {
                    continue;
                }
                name = convUnit.Name().String();
                unitFact = measWithUnit.ValueComponent();
            }
            else
            {
                SiUnit siUnit = namedUnit as SiUnit;
                if (siUnit is null)
                {
                    continue;
                }
                if (siUnit.IsKind(typeof(SiUnitAndLengthUnit)))
                {
                    ind = 1;
                }
                else if (siUnit.IsKind(typeof(SiUnitAndPlaneAngleUnit)))
                {
                    ind = 2;
                }
                else if (siUnit.IsKind(typeof(SiUnitAndSolidAngleUnit)))
                {
                    ind = 3;
                }
                if (ind == 0)
                {
                    continue;
                }
                unitFact = (!siUnit.HasPrefix ? 1.0 : UnitContext.ConvertSiPrefix(siUnit.Prefix()));
                name = GetSiName(siUnit);
            }
            if (ind == 0)
            {
                continue;
            }

            nameUnits[ind - 1] = name;
            factorUnits[ind - 1] = unitFact;
            nbFind++;
        }

        return nbFind != 0;
    }
}
