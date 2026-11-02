// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Reader.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_ReturnStatus;
using static Occt.StepBasic_SiPrefix;
using static Occt.StepBasic_SiUnitName;

namespace Occt;

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
/// <para>OCCT.NET (plan 2.7, decisions 1225, 13202):</para>
/// <list type="bullet">
/// <item>the parameters come from the session's parameter set (<see cref="IFSelect_WorkSession.Statics"/>), where
/// OCCT reads the process-wide statics; or from a <see cref="DESTEP_Parameters"/> given to the read;</item>
/// <item>files and streams are read with a progress range (cancellation) and the parser's limits for hostile input
/// (<see cref="StepFile_ReadLimits"/>, the defaults when none is given);</item>
/// <item>after a successful read the model's length unit is set from the session's <c>xstep.cascade.unit</c> (OCCT
/// takes the process-wide unit at the first transfer);</item>
/// <item>the shapes the reader publishes (<c>Shape</c>, <c>OneShape</c>) are frozen (decision 1224).</item>
/// </list>
/// </remarks>
public class STEPControl_Reader : XSControl_Reader
{
    /// <summary>Creates a reader object with an empty STEP model.</summary>
    public STEPControl_Reader()
    {
        STEPControl_Controller.Init();
        SelectStep();
    }

    /// <summary>Creates a Reader for STEP from an already existing Session Clears the session if it was not yet set for STEP</summary>
    public STEPControl_Reader(XSControl_WorkSession WS, bool scratch = true) : base(InitFirst(WS), scratch)
    {
        SelectStep();
    }

    // OCCT: SetNorm("STEP"), which selects the controller recorded process-wide; here one of the session's own
    // (STEPControl_Controller.SelectFor, decision 13201), then the transfer reader initialised as SetNorm does
    private void SelectStep()
    {
        if (WS() is null)
        {
            SetWS(new XSControl_WorkSession());
        }
        if (STEPControl_Controller.SelectFor(WS()))
        {
            WS().InitTransferReader(0);
            WS().InitTransferReader(4);
        }
    }

    // OCCT calls STEPControl_Controller::Init() before SetWS: the base constructor calls SetWS
    private static XSControl_WorkSession InitFirst(XSControl_WorkSession WS)
    {
        STEPControl_Controller.Init();
        return WS;
    }

    /// <summary>Returns the model as a StepModel. It can then be consulted (header, product)</summary>
    public StepData_StepModel StepModel()
    {
        return Model() as StepData_StepModel;
    }

    /// <summary>Loads a file and returns the read status Zero for a Model which complies with the Controller</summary>
    public override IFSelect_ReturnStatus ReadFile(string filename, Message_ProgressRange theProgress = default)
    {
        return ReadFile(filename, theProgress, null);
    }

    /// <summary>As <see cref="ReadFile(string, Message_ProgressRange)"/>, with the parser's limits (OCCT.NET).</summary>
    public IFSelect_ReturnStatus ReadFile(string filename, Message_ProgressRange theProgress, StepFile_ReadLimits theLimits)
    {
        return ReadInto(filename, null, ModelParameters(), theProgress, theLimits);
    }

    /// <summary>Loads a file from stream and returns the read status</summary>
    public override IFSelect_ReturnStatus ReadStream(string theName, Stream theIStream, Message_ProgressRange theProgress = default)
    {
        return ReadStream(theName, theIStream, theProgress, null);
    }

    /// <summary>As <see cref="ReadStream(string, Stream, Message_ProgressRange)"/>, with the parser's limits (OCCT.NET).</summary>
    public IFSelect_ReturnStatus ReadStream(string theName, Stream theIStream, Message_ProgressRange theProgress,
                                            StepFile_ReadLimits theLimits)
    {
        return ReadInto(theName, theIStream, ModelParameters(), theProgress, theLimits);
    }

    /// <summary>Loads a file and returns the read status Zero for a Model which complies with the Controller</summary>
    public IFSelect_ReturnStatus ReadFile(string filename, in DESTEP_Parameters theParams, Message_ProgressRange theProgress = default,
                                          StepFile_ReadLimits theLimits = null)
    {
        return ReadInto(filename, null, theParams, theProgress, theLimits);
    }

    /// <summary>Loads a file from stream and returns the read status</summary>
    public IFSelect_ReturnStatus ReadStream(string theName, in DESTEP_Parameters theParams, Stream theIStream,
                                            Message_ProgressRange theProgress = default, StepFile_ReadLimits theLimits = null)
    {
        return ReadInto(theName, theIStream, theParams, theProgress, theLimits);
    }

    // OCCT: InternalParameters.InitFromStatic(), from the process-wide statics; here the session's
    private DESTEP_Parameters ModelParameters()
    {
        DESTEP_Parameters aParams = new DESTEP_Parameters();
        aParams.InitFromStatic(WS().Statics());
        return aParams;
    }

    // The four ReadFile/ReadStream bodies of OCCT, which differ only in the parameters and the stream
    private IFSelect_ReturnStatus ReadInto(string theName, Stream theIStream, in DESTEP_Parameters theParams,
                                           Message_ProgressRange theProgress, StepFile_ReadLimits theLimits)
    {
        IFSelect_WorkLibrary aLibrary = WS().WorkLibrary();
        Interface_Protocol aProtocol = WS().Protocol();
        if (aLibrary is null)
        {
            return IFSelect_RetVoid;
        }
        if (aProtocol is null)
        {
            return IFSelect_RetVoid;
        }
        StepData_StepModel aStepModel = new StepData_StepModel();
        aStepModel.InternalParameters = theParams;
        aStepModel.SetSourceCodePage(aStepModel.InternalParameters.ReadCodePage);
        APIHeaderSection_MakeHeader aHeaderMaker = new APIHeaderSection_MakeHeader();
        aHeaderMaker.Apply(aStepModel);
        IFSelect_ReturnStatus status = IFSelect_RetVoid;
        try
        {
            int stat;
            if (aLibrary is StepSelect_WorkLibrary aStepLibrary)
            {
                stat = theIStream is null
                    ? aStepLibrary.ReadFile(theName, aStepModel, aProtocol, theProgress, theLimits)
                    : aStepLibrary.ReadStream(theName, theIStream, aStepModel, aProtocol, theProgress, theLimits);
            }
            else
            {
                Interface_InterfaceModel aModel = aStepModel;
                stat = theIStream is null
                    ? aLibrary.ReadFile(theName, ref aModel, aProtocol, theProgress)
                    : aLibrary.ReadStream(theName, theIStream, ref aModel, aProtocol, theProgress);
            }
            if (stat == 0)
            {
                status = IFSelect_RetDone;
            }
            else if (stat < 0)
            {
                status = IFSelect_RetError;
            }
            else
            {
                status = IFSelect_RetFail;
            }
        }
        catch (Exception anException) when (anException is Standard_Failure || StepToTopoDS_Root.CaughtBySignals(anException))
        {
            using Message_Messenger.StreamBuffer sout = Message.SendInfo();
            sout.Append("    ****    Interruption ReadFile par Exception :   ****\n");
            sout.Append(anException.Message);
            sout.Append("\n    Abandon").Append('\n');
            status = IFSelect_RetFail;
        }
        if (status != IFSelect_RetDone)
        {
            return status;
        }
        // OCCT.NET: the length unit, from the session's statics (decision 13202)
        if (!aStepModel.IsInitializedUnit())
        {
            aStepModel.SetLocalLengthUnit(UnitsMethods.GetCasCadeLengthUnit(WS().Statics()));
        }
        WS().SetModel(aStepModel);
        WS().SetLoadedFile(theName);
        WS().InitTransferReader(4);
        return status;
    }

    /// <summary>Transfers a root given its rank in the list of candidate roots Default is the first one Returns True if a shape has resulted, false else Same as inherited TransferOneRoot, kept for compatibility</summary>
    public bool TransferRoot(int num = 1, Message_ProgressRange theProgress = default)
    {
        return TransferOneRoot(num, theProgress);
    }

    /// <summary>Determines the list of root entities from Model which are candidate for a transfer to a Shape (type of entities is PRODUCT)</summary>
    public override int NbRootsForTransfer()
    {
        if (therootsta)
        {
            return theroots.Length();
        }
        therootsta = true;

        StepData_StepModel aStepModel = WS().Model() as StepData_StepModel;
        // theroots.Clear();
        int nb = Model().NbEntities();
        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient ent = Model().Value(i);
            if (aStepModel.InternalParameters.ReadAllShapes)
            {
                // Special case to read invalid shape_representation without links to shapes.
                if (ent.IsKind(typeof(StepShape_ManifoldSolidBrep)))
                {
                    Interface_EntityIterator aShareds = WS().Graph().Sharings(ent);
                    if (!aShareds.More())
                    {
                        theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
                if (ent.IsKind(typeof(StepShape_ShellBasedSurfaceModel)))
                {
                    Interface_EntityIterator aShareds = WS().Graph().Sharings(ent);
                    if (!aShareds.More())
                    {
                        theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
            }
            if (ent.IsKind(typeof(StepBasic_ProductDefinition)))
            {
                // PTV 31.01.2003 TRJ11 exclude Product Definition With Associated Document from roots
                if (ent.IsKind(typeof(StepBasic_ProductDefinitionWithAssociatedDocuments)))
                {
                    // check if PDWAD-> PDF <-Document_Product_Equivalence.
                    bool iSexclude = false;
                    StepBasic_ProductDefinitionWithAssociatedDocuments PDWAD = ent as StepBasic_ProductDefinitionWithAssociatedDocuments;
                    Interface_EntityIterator PDWADsubs = WS().Graph().Shareds(PDWAD);
                    for (PDWADsubs.Start(); PDWADsubs.More(); PDWADsubs.Next())
                    {
                        if (!PDWADsubs.Value().IsKind(typeof(StepBasic_ProductDefinitionFormation)))
                        {
                            continue;
                        }
                        StepBasic_ProductDefinitionFormation localPDF = PDWADsubs.Value() as StepBasic_ProductDefinitionFormation;
                        Interface_EntityIterator PDFsubs = WS().Graph().Sharings(localPDF);
                        for (PDFsubs.Start(); PDFsubs.More(); PDFsubs.Next())
                        {
                            if (PDFsubs.Value().IsKind(typeof(StepBasic_DocumentProductEquivalence)))
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
                StepBasic_ProductDefinition PD = ent as StepBasic_ProductDefinition;
                bool IsRoot = true;
                Interface_Graph graph = WS().Graph();
                // determinate roots used NextAssemblyUsageOccurrence
                Interface_EntityIterator subs = graph.Sharings(PD);
                for (subs.Start(); subs.More(); subs.Next())
                {
                    StepRepr_NextAssemblyUsageOccurrence NAUO = subs.Value() as StepRepr_NextAssemblyUsageOccurrence;
                    if (NAUO is null)
                    {
                        continue;
                    }
                    if (PD == NAUO.RelatedProductDefinition())
                    {
                        IsRoot = false;
                    }
                }
                // determinate roots used ProductDefinitionContext
                if (IsRoot)
                {
                    DESTEP_Parameters.ReadMode_ProductContext aProdContMode = aStepModel.InternalParameters.ReadProductContext;
                    string str1 = aStepModel.InternalParameters.GetString(aProdContMode);
                    int ICS = (int)aStepModel.InternalParameters.ReadProductContext;
                    if (ICS > 1)
                    {
                        subs = graph.Shareds(PD);
                        for (subs.Start(); subs.More(); subs.Next())
                        {
                            StepBasic_ProductDefinitionContext PDC = subs.Value() as StepBasic_ProductDefinitionContext;
                            if (PDC is null)
                            {
                                continue;
                            }
                            string str2 = PDC.LifeCycleStage().String();
                            string str3 = PDC.Name().String();
                            if (!string.Equals(str1, str2, StringComparison.OrdinalIgnoreCase)
                                && !string.Equals(str1, str3, StringComparison.OrdinalIgnoreCase))
                            {
                                IsRoot = false;
                            }
                        }
                    }
                }
                if (IsRoot)
                {
                    theroots.Append(ent);
                    WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                }
            }
            bool aProdMode = aStepModel.InternalParameters.ReadProductMode;
            if (!aProdMode)
            {
                if (ent.IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
                {
                    bool IsRoot = true;
                    StepShape_ShapeDefinitionRepresentation SDR = ent as StepShape_ShapeDefinitionRepresentation;
                    StepRepr_PropertyDefinition PropDef = SDR.Definition().PropertyDefinition();
                    if (PropDef is not null)
                    {
                        StepBasic_ProductDefinition PD = PropDef.Definition().ProductDefinition();
                        if (PD is not null)
                        {
                            IsRoot = false;
                        }
                        if (IsRoot)
                        {
                            StepRepr_ShapeAspect SA = PropDef.Definition().ShapeAspect();
                            if (SA is not null)
                            {
                                StepRepr_ProductDefinitionShape PDS = SA.OfShape();
                                PD = PDS.Definition().ProductDefinition();
                                if (PD is not null)
                                {
                                    IsRoot = false;
                                }
                            }
                        }
                        if (IsRoot)
                        {
                            StepRepr_NextAssemblyUsageOccurrence NAUO =
                                PropDef.Definition().ProductDefinitionRelationship() as StepRepr_NextAssemblyUsageOccurrence;
                            if (NAUO is not null)
                            {
                                IsRoot = false;
                            }
                        }
                        if (IsRoot)
                        {
                            StepShape_ShapeRepresentation SR = SDR.UsedRepresentation() as StepShape_ShapeRepresentation;
                            if (SR is null)
                            {
                                IsRoot = false;
                            }
                        }
                    }
                    if (IsRoot)
                    {
                        theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
                if (ent.IsKind(typeof(StepShape_ShapeRepresentation)))
                {
                    bool IsRoot = true;
                    StepShape_ShapeRepresentation SR = ent as StepShape_ShapeRepresentation;
                    Interface_Graph graph = WS().Graph();
                    Interface_EntityIterator subs = graph.Sharings(SR);
                    for (subs.Start(); subs.More() && IsRoot; subs.Next())
                    {
                        StepShape_ShapeDefinitionRepresentation SDR = subs.Value() as StepShape_ShapeDefinitionRepresentation;
                        if (SDR is not null)
                        {
                            IsRoot = false;
                        }
                        if (IsRoot)
                        {
                            StepRepr_RepresentationRelationship RR = subs.Value() as StepRepr_RepresentationRelationship;
                            if (RR is not null)
                            {
                                StepShape_ShapeRepresentation SR2 = RR.Rep1() as StepShape_ShapeRepresentation;
                                if (SR == SR2)
                                {
                                    SR2 = RR.Rep2() as StepShape_ShapeRepresentation;
                                }
                                if (SR2 is not null)
                                {
                                    Interface_EntityIterator subs2 = graph.Sharings(SR2);
                                    for (subs2.Start(); subs2.More(); subs2.Next())
                                    {
                                        StepShape_ShapeDefinitionRepresentation SDR2 = subs2.Value() as StepShape_ShapeDefinitionRepresentation;
                                        if (SDR2 is not null)
                                        {
                                            IsRoot = false;
                                        }
                                    }
                                }
                            }
                        }
                        if (IsRoot)
                        {
                            StepRepr_RepresentationMap RM = subs.Value() as StepRepr_RepresentationMap;
                            if (RM is not null)
                            {
                                Interface_EntityIterator subs2 = graph.Sharings(RM);
                                for (subs2.Start(); subs2.More(); subs2.Next())
                                {
                                    StepRepr_MappedItem MI = subs2.Value() as StepRepr_MappedItem;
                                    if (MI is not null)
                                    {
                                        Interface_EntityIterator subs3 = graph.Sharings(MI);
                                        for (subs3.Start(); subs3.More(); subs3.Next())
                                        {
                                            StepShape_ShapeRepresentation SR2 = subs3.Value() as StepShape_ShapeRepresentation;
                                            if (SR2 is not null)
                                            {
                                                IsRoot = false;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if (IsRoot)
                    {
                        theroots.Append(ent);
                        WS().TransferReader().TransientProcess().RootsForTransfer().Append(ent);
                    }
                }
            }
        }

        return theroots.Length();
    }

    /// <summary>
    /// Returns sequence of all unit names for shape representations found in file
    /// </summary>
    public void FileUnits(NCollection_Sequence<string> theUnitLengthNames, NCollection_Sequence<string> theUnitAngleNames,
                          NCollection_Sequence<string> theUnitSolidAngleNames)
    {
        int nbroots = NbRootsForTransfer();
        if (nbroots == 0)
        {
            return;
        }

        const int LENGTH = 0;
        const int ANLGE = 1;
        const int SOLID_ANGLE = 2;

        Interface_Graph graph = WS().Graph();
        HashSet<string>[] aMapUnits = { new HashSet<string>(), new HashSet<string>(), new HashSet<string>() };

        for (int i = 1; i <= nbroots; i++)
        {
            Standard_Transient anEnt = theroots.Value(i);
            int num = graph.EntityNumber(anEnt);
            if (num == 0)
            {
                continue;
            }
            StepBasic_ProductDefinition aProdDef = anEnt as StepBasic_ProductDefinition;
            StepShape_ShapeDefinitionRepresentation aShapeDefRepr = null;
            if (aProdDef is not null)
            {
                Interface_EntityIterator subsPD = graph.Sharings(aProdDef);
                for (subsPD.Start(); subsPD.More() && aShapeDefRepr is null; subsPD.Next())
                {
                    StepRepr_ProductDefinitionShape aProdDefShape = subsPD.Value() as StepRepr_ProductDefinitionShape;
                    if (aProdDefShape is null)
                    {
                        continue;
                    }
                    Interface_EntityIterator subsSR = graph.Sharings(aProdDefShape);
                    for (subsSR.Start(); subsSR.More() && aShapeDefRepr is null; subsSR.Next())
                    {
                        StepShape_ShapeDefinitionRepresentation aCurShapeDefRepr = subsSR.Value() as StepShape_ShapeDefinitionRepresentation;
                        if (aCurShapeDefRepr is null)
                        {
                            continue;
                        }
                        StepRepr_Representation aUseRepr = aCurShapeDefRepr.UsedRepresentation();
                        if (aUseRepr is null)
                        {
                            continue;
                        }
                        StepShape_ShapeRepresentation aShapeRepr = aUseRepr as StepShape_ShapeRepresentation;
                        if (aShapeRepr is null)
                        {
                            continue;
                        }
                        aShapeDefRepr = aCurShapeDefRepr;
                    }
                }
            }
            else
            {
                aShapeDefRepr = anEnt as StepShape_ShapeDefinitionRepresentation;
            }
            if (aShapeDefRepr is not null)
            {
                StepShape_ShapeRepresentation aShapeRepr = aShapeDefRepr.UsedRepresentation() as StepShape_ShapeRepresentation;
                StepRepr_RepresentationContext aRepCont = aShapeRepr.ContextOfItems();
                if (aRepCont is null)
                {
                    continue;
                }
                string[] aNameUnits = { "", "", "" };
                double[] aFactorUnits = new double[3];
                if (findUnits(aRepCont, aNameUnits, aFactorUnits))
                {
                    for (int k = LENGTH; k <= SOLID_ANGLE; k++)
                    {
                        if (aMapUnits[k].Add(aNameUnits[k]))
                        {
                            NCollection_Sequence<string> anUnitSeq =
                                (k == LENGTH ? theUnitLengthNames : (k == ANLGE ? theUnitAngleNames : theUnitSolidAngleNames));
                            anUnitSeq.Append(aNameUnits[k]);
                        }
                    }
                }
            }
        }
        // for case when units was not found through PDF or SDR
        if (theUnitLengthNames.IsEmpty())
        {
            Interface_InterfaceModel aModel = WS().Model();
            if (aModel is null)
            {
                return;
            }
            int nb = aModel.NbEntities();
            for (int i = 1; i <= nb; i++)
            {
                Standard_Transient anEnt = aModel.Value(i);
                StepRepr_RepresentationContext aRepCont = anEnt as StepRepr_RepresentationContext;
                if (aRepCont is null)
                {
                    continue;
                }
                string[] aNameUnits = { "", "", "" };
                double[] aFactorUnits = new double[3];
                if (findUnits(aRepCont, aNameUnits, aFactorUnits))
                {
                    for (int k = LENGTH; k <= SOLID_ANGLE; k++)
                    {
                        if (aMapUnits[k].Add(aNameUnits[k]))
                        {
                            NCollection_Sequence<string> anUnitSeq =
                                (k == LENGTH ? theUnitLengthNames : (k == ANLGE ? theUnitAngleNames : theUnitSolidAngleNames));
                            anUnitSeq.Append(aNameUnits[k]);
                        }
                    }
                }
            }
        }
    }

    /// <summary>Sets system length unit used by transfer process. Performs only if a model is not NULL</summary>
    public void SetSystemLengthUnit(double theLengthUnit)
    {
        if (StepModel() is null)
        {
            return;
        }
        StepModel().SetLocalLengthUnit(theLengthUnit);
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

    private static string getSiName(StepBasic_SiUnit theUnit)
    {
        string aName = "";
        if (theUnit.HasPrefix())
        {
            switch (theUnit.Prefix())
            {
                case StepBasic_spExa:
                    aName += "exa";
                    break;
                case StepBasic_spPeta:
                    aName += "peta";
                    break;
                case StepBasic_spTera:
                    aName += "tera";
                    break;
                case StepBasic_spGiga:
                    aName += "giga";
                    break;
                case StepBasic_spMega:
                    aName += "mega";
                    break;
                case StepBasic_spHecto:
                    aName += "hecto";
                    break;
                case StepBasic_spDeca:
                    aName += "deca";
                    break;
                case StepBasic_spDeci:
                    aName += "deci";
                    break;

                case StepBasic_spPico:
                    aName += "pico";
                    break;
                case StepBasic_spFemto:
                    aName += "femto";
                    break;
                case StepBasic_spAtto:
                    aName += "atto";
                    break;

                case StepBasic_spKilo:
                    aName += "kilo";
                    break;
                case StepBasic_spCenti:
                    aName += "centi";
                    break;
                case StepBasic_spMilli:
                    aName += "milli";
                    break;
                case StepBasic_spMicro:
                    aName += "micro";
                    break;
                case StepBasic_spNano:
                    aName += "nano";
                    break;
                default:
                    break;
            }
        }

        switch (theUnit.Name())
        {
            case StepBasic_sunMetre:
                aName += "metre";
                break;
            case StepBasic_sunRadian:
                aName += "radian";
                break;
            case StepBasic_sunSteradian:
                aName += "steradian";
                break;
            default:
                break;
        }
        return aName;
    }

    protected override DE_ShapeFixParameters GetDefaultShapeFixParameters()
    {
        return DESTEP_Parameters.GetDefaultShapeFixParameters();
    }

    protected override ShapeProcess.OperationsFlags GetDefaultShapeProcessFlags()
    {
        return new ShapeProcess.OperationsFlags(ShapeProcess.Operation.FixShape);
    }

    // OCCT's 1-based arrays of 3 units (length, plane angle, solid angle): 0-based here
    private bool findUnits(StepRepr_RepresentationContext theRepCont, string[] theNameUnits, double[] theFactorUnits)
    {
        StepRepr_GlobalUnitAssignedContext aContext = null;

        if (theRepCont.IsKind(typeof(StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext)))
        {
            aContext = (theRepCont as StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext).GlobalUnitAssignedContext();
        }

        if (theRepCont.IsKind(typeof(StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx)))
        {
            aContext = (theRepCont as StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx).GlobalUnitAssignedContext();
        }
        if (aContext is null)
        {
            return false;
        }
        // Start Computation
        int nbU = aContext.NbUnits();
        int nbFind = 0;
        for (int i = 1; i <= nbU; i++)
        {
            StepBasic_NamedUnit aNamedUnit = aContext.UnitsValue(i);
            StepBasic_ConversionBasedUnit aConvUnit = aNamedUnit as StepBasic_ConversionBasedUnit;
            int anInd = 0;
            string aName = "";
            double anUnitFact = 0;
            if (aConvUnit is not null)
            {
                StepBasic_MeasureWithUnit aMeasWithUnit = null;
                Standard_Transient aConvFactor = aConvUnit.ConversionFactor();
                if (aConvFactor.IsKind(typeof(StepBasic_MeasureWithUnit)))
                {
                    aMeasWithUnit = aConvFactor as StepBasic_MeasureWithUnit;
                }
                else if (aConvFactor.IsKind(typeof(StepRepr_ReprItemAndMeasureWithUnit)))
                {
                    StepRepr_ReprItemAndMeasureWithUnit aReprMeasureItem = aConvFactor as StepRepr_ReprItemAndMeasureWithUnit;
                    aMeasWithUnit = aReprMeasureItem.GetMeasureWithUnit();
                }

                if (aMeasWithUnit is null)
                {
                    continue;
                }

                if (aMeasWithUnit.IsKind(typeof(StepBasic_LengthMeasureWithUnit)))
                {
                    anInd = 1;
                }
                else if (aMeasWithUnit.IsKind(typeof(StepBasic_PlaneAngleMeasureWithUnit)))
                {
                    anInd = 2;
                }
                else if (aMeasWithUnit.IsKind(typeof(StepBasic_SolidAngleMeasureWithUnit)))
                {
                    anInd = 3;
                }
                if (anInd == 0)
                {
                    continue;
                }
                aName = aConvUnit.Name().String();
                anUnitFact = aMeasWithUnit.ValueComponent();
            }
            else
            {
                StepBasic_SiUnit aSiUnit = aNamedUnit as StepBasic_SiUnit;
                if (aSiUnit is null)
                {
                    continue;
                }
                if (aSiUnit.IsKind(typeof(StepBasic_SiUnitAndLengthUnit)))
                {
                    anInd = 1;
                }
                else if (aSiUnit.IsKind(typeof(StepBasic_SiUnitAndPlaneAngleUnit)))
                {
                    anInd = 2;
                }
                else if (aSiUnit.IsKind(typeof(StepBasic_SiUnitAndSolidAngleUnit)))
                {
                    anInd = 3;
                }
                if (anInd == 0)
                {
                    continue;
                }
                anUnitFact = (!aSiUnit.HasPrefix() ? 1.0 : STEPConstruct_UnitContext.ConvertSiPrefix(aSiUnit.Prefix()));
                aName = getSiName(aSiUnit);
            }
            if (anInd == 0)
            {
                continue;
            }

            theNameUnits[anInd - 1] = aName;
            theFactorUnits[anInd - 1] = anUnitFact;
            nbFind++;
        }

        return nbFind != 0;
    }
}
