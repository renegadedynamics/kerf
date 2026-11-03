// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Controller.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Controller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Occt.IFSelect_ReturnStatus;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>defines basic controller for STEP processor</summary>
/// <remarks>
/// <para>
/// OCCT.NET (plan 2.7, decisions 1181, 1222, 1264): OCCT's constructor registers the STEP modules and declares STEP's
/// statics in the process-wide <c>Interface_Static</c> the first time; the port registers the modules once per process
/// (<see cref="Init"/> and the constructor, idempotent) and declares the statics in each session's parameter set
/// (<see cref="InitStatics"/>, once per set). <see cref="Init"/> records one controller under "STEP" and "step", as
/// OCCT's; STEPControl_Reader and STEPControl_Writer give their session a controller of its own
/// (<see cref="SelectFor"/>), so that the write actor's settings (tolerance, group mode, shape fix parameters) and
/// counters (product names, assembly occurrences) are the session's.
/// </para>
/// </remarks>
public class STEPControl_Controller : XSControl_Controller
{
    private static readonly object theInitLock = new object();
    private static STEPControl_Controller theRecorded;

    //  Pour NewModel et Write : definition de produit (temporaire ...)
    /// <summary>Initializes the use of STEP Norm (the first time) and returns a Controller</summary>
    public STEPControl_Controller() : base("STEP", "step")
    {
        RegisterModules();

        STEPControl_ActorWrite ActWrite = new STEPControl_ActorWrite();
        myAdaptorWrite = ActWrite;

        ActWrite.SetShapeFixParameters(DESTEP_Parameters.GetDefaultShapeFixParameters(), new NCollection_DataMap<string, string>());
        global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags aDefaultProcFlags = new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.SplitCommonVertex,
                                                                                         global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.DirectFaces);
        ActWrite.SetShapeProcessFlags(aDefaultProcFlags);

        StepSelect_WorkLibrary swl = new StepSelect_WorkLibrary();
        swl.SetDumpLabel(1);
        myAdaptorLibrary = swl;
        myAdaptorProtocol = STEPEdit.Protocol();

        SetModeWrite(0, 4);
        SetModeWriteHelp(0, "As Is");
        SetModeWriteHelp(1, "Faceted Brep");
        SetModeWriteHelp(2, "Shell Based");
        SetModeWriteHelp(3, "Manifold Solid");
        SetModeWriteHelp(4, "Wireframe");
        TraceStatic("read.surfacecurve.mode", 5);

        //   ---  SELECTIONS, SIGNATURES, COMPTEURS, EDITEURS

        IFSelect_Selection xmr = SessionItem("xst-model-roots") as IFSelect_Selection;
        if (xmr is not null)
        {
            IFSelect_Signature sty = STEPEdit.SignType();
            AddSessionItem(sty, "step-type");
            IFSelect_SignCounter tys = new IFSelect_SignCounter(sty, false, true);
            AddSessionItem(tys, "step-types");

            // szv:mySignType = sty;

            // pdn S4133 18.02.99
            AddSessionItem(new IFSelect_SignAncestor(), "xst-derived");

            STEPSelections_SelectDerived stdvar = new STEPSelections_SelectDerived();
            stdvar.SetProtocol(STEPEdit.Protocol());
            AddSessionItem(stdvar, "step-derived");

            IFSelect_SelectSignature selsdr = STEPEdit.NewSelectSDR();
            selsdr.SetInput(xmr);
            AddSessionItem(selsdr, "step-shape-def-repr");

            AddSessionItem(STEPEdit.NewSelectPlacedItem(), "step-placed-items");
            // input deja pret avec ModelAll
            AddSessionItem(STEPEdit.NewSelectShapeRepr(), "step-shape-repr");
        }

        // pdn
        STEPSelections_SelectFaces stfaces = new STEPSelections_SelectFaces();
        stfaces.SetInput(xmr);
        AddSessionItem(stfaces, "step-faces");

        STEPSelections_SelectInstances stinst = new STEPSelections_SelectInstances();
        AddSessionItem(stinst, "step-instances");

        STEPSelections_SelectGSCurves stcurves = new STEPSelections_SelectGSCurves();
        stcurves.SetInput(xmr);
        AddSessionItem(stcurves, "step-GS-curves");

        STEPSelections_SelectAssembly assembly = new STEPSelections_SelectAssembly();
        assembly.SetInput(xmr);
        AddSessionItem(assembly, "step-assembly");

        APIHeaderSection_EditHeader edhead = new APIHeaderSection_EditHeader();
        IFSelect_EditForm edheadf = new IFSelect_EditForm(edhead, false, true, "Step Header");
        AddSessionItem(edhead, "step-header-edit");
        AddSessionItem(edheadf, "step-header");

        STEPEdit_EditContext edctx = new STEPEdit_EditContext();
        IFSelect_EditForm edctxf = new IFSelect_EditForm(edctx, false, true, "STEP Product Definition Context");
        AddSessionItem(edctx, "step-context-edit");
        AddSessionItem(edctxf, "step-context");

        STEPEdit_EditSDR edsdr = new STEPEdit_EditSDR();
        IFSelect_EditForm edsdrf = new IFSelect_EditForm(edsdr, false, true, "STEP Product Data (SDR)");
        AddSessionItem(edsdr, "step-SDR-edit");
        AddSessionItem(edsdrf, "step-SDR-data");
    }

    // OCCT's constructor's first part (behind its static flag and mutex): the modules, once per process; the statics
    // it declares there are declared per parameter set (InitStatics)
    private static void RegisterModules()
    {
        RWHeaderSection.Init();
        RWStepAP214.Init();
    }

    /// <summary>
    /// Declares STEP's statics (OCCT's constructor's <c>Interface_Static::Init</c> calls) in a parameter set, once per
    /// set, after the standard ones and XSAlgo's (OCCT.NET, decision 1181).
    /// </summary>
    public override void InitStatics(Interface_StaticSet theStatics)
    {
        base.InitStatics(theStatics);
        XSAlgo.Init(theStatics);
        if (!theStatics.StartInit("STEPControl_Controller"))
        {
            return;
        }

        theStatics.Init("step", "write.step.product.name", 't', "Open CASCADE STEP translator " + Interface_Version.OCC_VERSION_STRING);
        theStatics.Init("step", "write.step.assembly", 'e', "");
        theStatics.Init("step", "write.step.assembly", '&', "enum 0");
        theStatics.Init("step", "write.step.assembly", '&', "eval Off");
        theStatics.Init("step", "write.step.assembly", '&', "eval On");
        theStatics.Init("step", "write.step.assembly", '&', "eval Auto");
        theStatics.SetCVal("write.step.assembly", "Auto");

        theStatics.Init("step", "step.angleunit.mode", 'e', "");
        theStatics.Init("step", "step.angleunit.mode", '&', "enum 0");
        theStatics.Init("step", "step.angleunit.mode", '&', "eval File");
        theStatics.Init("step", "step.angleunit.mode", '&', "eval Rad");
        theStatics.Init("step", "step.angleunit.mode", '&', "eval Deg");
        theStatics.SetCVal("step.angleunit.mode", "File");

        theStatics.Init("step", "write.step.schema", 'e', "");
        theStatics.Init("step", "write.step.schema", '&', "enum 1");
        theStatics.Init("step", "write.step.schema", '&', "eval AP214CD");
        theStatics.Init("step", "write.step.schema", '&', "eval AP214DIS");
        theStatics.Init("step", "write.step.schema", '&', "eval AP203");
        theStatics.Init("step", "write.step.schema", '&', "eval AP214IS");
        theStatics.Init("step", "write.step.schema", '&', "eval AP242DIS");
        theStatics.SetCVal("write.step.schema", "AP214IS");

        // Type of Product Definition for reading
        // Note: the numbers should be consistent with function FindShapeReprType()
        // in STEPControl_ActorRead.cxx
        theStatics.Init("step", "read.step.shape.repr", 'e', "");
        theStatics.Init("step", "read.step.shape.repr", '&', "enum 1");
        theStatics.Init("step", "read.step.shape.repr", '&', "eval All");   // 1
        theStatics.Init("step", "read.step.shape.repr", '&', "eval ABSR");  // 2
        theStatics.Init("step", "read.step.shape.repr", '&', "eval MSSR");  // 3
        theStatics.Init("step", "read.step.shape.repr", '&', "eval GBSSR"); // 4
        theStatics.Init("step", "read.step.shape.repr", '&', "eval FBSR");  // 5
        theStatics.Init("step", "read.step.shape.repr", '&', "eval EBWSR"); // 6
        theStatics.Init("step", "read.step.shape.repr", '&', "eval GBWSR"); // 7
        theStatics.SetCVal("read.step.shape.repr", "All");

        // Mode for reading shapes attached to main SDR by SRR
        // (hybrid model representation in AP203 since 1998)
        theStatics.Init("step", "read.step.shape.relationship", 'e', "");
        theStatics.Init("step", "read.step.shape.relationship", '&', "enum 0");
        theStatics.Init("step", "read.step.shape.relationship", '&', "eval OFF");
        theStatics.Init("step", "read.step.shape.relationship", '&', "eval ON");
        theStatics.SetCVal("read.step.shape.relationship", "ON");

        // Mode for reading shapes attached to Product by ShapeAspect
        // (hybrid model representation in AP203 before 1998)
        theStatics.Init("step", "read.step.shape.aspect", 'e', "");
        theStatics.Init("step", "read.step.shape.aspect", '&', "enum 0");
        theStatics.Init("step", "read.step.shape.aspect", '&', "eval OFF");
        theStatics.Init("step", "read.step.shape.aspect", '&', "eval ON");
        theStatics.SetCVal("read.step.shape.aspect", "ON");

        // Mode for reading SDR and ShapeRepr if it is necessary
        theStatics.Init("step", "read.step.product.mode", 'e', "");
        theStatics.Init("step", "read.step.product.mode", '&', "enum 0");
        theStatics.Init("step", "read.step.product.mode", '&', "eval OFF");
        theStatics.Init("step", "read.step.product.mode", '&', "eval ON");
        theStatics.SetCVal("read.step.product.mode", "ON");

        // Order of reading ShapeDefinitionRepresentation in ProductDefinition
        theStatics.Init("step", "read.step.product.context", 'e', "");
        theStatics.Init("step", "read.step.product.context", '&', "enum 1");
        theStatics.Init("step", "read.step.product.context", '&', "eval all");      // 1
        theStatics.Init("step", "read.step.product.context", '&', "eval design");   // 2
        theStatics.Init("step", "read.step.product.context", '&', "eval analysis"); // 3
        theStatics.SetCVal("read.step.product.context", "all");

        // What we try to read in ProductDefinition
        theStatics.Init("step", "read.step.assembly.level", 'e', "");
        theStatics.Init("step", "read.step.assembly.level", '&', "enum 1");
        theStatics.Init("step", "read.step.assembly.level", '&', "eval all");       // 1
        theStatics.Init("step", "read.step.assembly.level", '&', "eval assembly");  // 2
        theStatics.Init("step", "read.step.assembly.level", '&', "eval structure"); // 3
        theStatics.Init("step", "read.step.assembly.level", '&', "eval shape");     // 4
        theStatics.SetCVal("read.step.assembly.level", "all");

        // unit: supposed to be cascade unit (target unit for reading)
        theStatics.Init("step", "write.step.unit", 'e', "");
        theStatics.Init("step", "write.step.unit", '&', "enum 1");
        theStatics.Init("step", "write.step.unit", '&', "eval INCH"); // 1
        theStatics.Init("step", "write.step.unit", '&', "eval MM");   // 2
        theStatics.Init("step", "write.step.unit", '&', "eval ??");   // 3
        theStatics.Init("step", "write.step.unit", '&', "eval FT");   // 4
        theStatics.Init("step", "write.step.unit", '&', "eval MI");   // 5
        theStatics.Init("step", "write.step.unit", '&', "eval M");    // 6
        theStatics.Init("step", "write.step.unit", '&', "eval KM");   // 7
        theStatics.Init("step", "write.step.unit", '&', "eval MIL");  // 8
        theStatics.Init("step", "write.step.unit", '&', "eval UM");   // 9
        theStatics.Init("step", "write.step.unit", '&', "eval CM");   // 10
        theStatics.Init("step", "write.step.unit", '&', "eval UIN");  // 11
        theStatics.SetCVal("write.step.unit", "MM");

        // Non-manifold topology reading: OFF by default (ssv; 26.11.2010)
        theStatics.Init("step", "read.step.nonmanifold", 'e', "");
        theStatics.Init("step", "read.step.nonmanifold", '&', "enum 0");
        theStatics.Init("step", "read.step.nonmanifold", '&', "eval Off");
        theStatics.Init("step", "read.step.nonmanifold", '&', "eval On");
        theStatics.SetIVal("read.step.nonmanifold", 0);

        // Non-manifold topology writing: OFF by default (ssv; 26.11.2010)
        theStatics.Init("step", "write.step.nonmanifold", 'e', "");
        theStatics.Init("step", "write.step.nonmanifold", '&', "enum 0");
        theStatics.Init("step", "write.step.nonmanifold", '&', "eval Off");
        theStatics.Init("step", "write.step.nonmanifold", '&', "eval On");
        theStatics.SetIVal("write.step.nonmanifold", 0);

        // I-Deas-like STEP processing: OFF by default (ssv; 22.11.2010)
        theStatics.Init("step", "read.step.ideas", 'e', "");
        theStatics.Init("step", "read.step.ideas", '&', "enum 0");
        theStatics.Init("step", "read.step.ideas", '&', "eval Off");
        theStatics.Init("step", "read.step.ideas", '&', "eval On");
        theStatics.SetIVal("read.step.ideas", 0);

        // Parameter to write all free vertices in one SDR (name and style of vertex are lost) (default)
        // or each vertex in its own SDR (name and style of vertex are exported). (ika; 21.07.2014)
        theStatics.Init("step", "write.step.vertex.mode", 'e', "");
        theStatics.Init("step", "write.step.vertex.mode", '&', "enum 0");
        theStatics.Init("step", "write.step.vertex.mode", '&', "eval One Compound");
        theStatics.Init("step", "write.step.vertex.mode", '&', "eval Single Vertex");
        theStatics.SetIVal("write.step.vertex.mode", 0);

        // abv 15.11.00: ShapeProcessing
        theStatics.Init("XSTEP", "write.step.resource.name", 't', "STEP");
        theStatics.Init("XSTEP", "read.step.resource.name", 't', "STEP");
        theStatics.Init("XSTEP", "write.step.sequence", 't', "ToSTEP");
        theStatics.Init("XSTEP", "read.step.sequence", 't', "FromSTEP");
        theStatics.Init("XSTEP", "ToSTEP.exec.op", 't', "SplitCommonVertex,DirectFaces");
        theStatics.Init("XSTEP", "FromSTEP.exec.op", 't', "FixShape");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.Tolerance3d", 't', "&Runtime.Tolerance");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.MaxTolerance3d", 't', "&Runtime.MaxTolerance");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.MinTolerance3d", 't', "1.e-7");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixFreeShellMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixFreeFaceMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixFreeWireMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSameParameterMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSolidMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixShellOrientationMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.CreateOpenSolidMode", 't', "0");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixShellMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixFaceOrientationMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixFaceMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixWireMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixOrientationMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixAddNaturalBoundMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixMissingSeamMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSmallAreaWireMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.RemoveSmallAreaFaceMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixIntersectingWiresMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixLoopWiresMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSplitFaceMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.AutoCorrectPrecisionMode", 't', "1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.ModifyTopologyMode", 't', "0");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.ModifyGeometryMode", 't', "1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.ClosedWireMode", 't', "1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.PreferencePCurveMode", 't', "1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixReorderMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSmallMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixConnectedMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixEdgeCurvesMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixDegeneratedMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixLackingMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSelfIntersectionMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.RemoveLoopMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixReversed2dMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixRemovePCurveMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixRemoveCurve3dMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixAddPCurveMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixAddCurve3dMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSeamMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixShiftedMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixEdgeSameParameterMode", 't', "0");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixNotchedEdgesMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixTailMode", 't', "0");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.MaxTailAngle", 't', "0.0");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.MaxTailWidth", 't', "-1.0");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixSelfIntersectingEdgeMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixIntersectingEdgesMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixNonAdjacentIntersectingEdgesMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixVertexPositionMode", 't', "0");
        theStatics.Init("XSTEP", "FromSTEP.FixShape.FixVertexToleranceMode", 't', "-1");
        theStatics.Init("XSTEP", "FromSTEP.FixFaceSize.Tolerance", 't', "1.e-7");

        // ika 28.07.16: Parameter to read all top level solids and shells,
        // should be used only in case of invalid shape_representation without links to shapes.
        theStatics.Init("step", "read.step.all.shapes", 'e', "");
        theStatics.Init("step", "read.step.all.shapes", '&', "enum 0");
        theStatics.Init("step", "read.step.all.shapes", '&', "eval Off");
        theStatics.Init("step", "read.step.all.shapes", '&', "eval On");
        theStatics.SetIVal("read.step.all.shapes", 0);

        // Mode for reading constructive geometry representation relationship to read
        // StepRepr_ConstructiveGeometryRepresentation method implemented only for
        // StepGeom_MakeAxis2Placement3d for axis placements representing axis for supplemented
        // geometry. Axis placements are translated to planar faces with CS equal to translated axis
        // placements
        theStatics.Init("step", "read.step.constructivegeom.relationship", 'e', "");
        theStatics.Init("step", "read.step.constructivegeom.relationship", '&', "enum 0");
        theStatics.Init("step", "read.step.constructivegeom.relationship", '&', "eval OFF");
        theStatics.Init("step", "read.step.constructivegeom.relationship", '&', "eval ON");
        theStatics.SetCVal("read.step.constructivegeom.relationship", "OFF");

        // Mode to variate apply or not transformation placed in the root shape representation.
        // Issues #29068 and #31491.
        theStatics.Init("step", "read.step.root.transformation", 'e', "");
        theStatics.Init("step", "read.step.root.transformation", '&', "enum 0");
        theStatics.Init("step", "read.step.root.transformation", '&', "eval OFF");
        theStatics.Init("step", "read.step.root.transformation", '&', "eval ON");
        theStatics.SetCVal("read.step.root.transformation", "ON");

        // STEP file encoding for names translation
        // Note: the numbers should be consistent with Resource_FormatType enumeration
        theStatics.Init("step", "read.step.codepage", 'e', "");
        theStatics.Init("step", "read.step.codepage", '&', "enum 0");
        theStatics.Init("step", "read.step.codepage", '&', "eval SJIS");         // Resource_FormatType_SJIS 0
        theStatics.Init("step", "read.step.codepage", '&', "eval EUC");          // Resource_FormatType_EUC 1
        theStatics.Init("step", "read.step.codepage", '&', "eval NoConversion"); // Resource_FormatType_NoConversion 2
        theStatics.Init("step", "read.step.codepage", '&', "eval GB");           // Resource_FormatType_GB 3
        theStatics.Init("step", "read.step.codepage", '&', "eval UTF8");         // Resource_FormatType_UTF8 4
        theStatics.Init("step", "read.step.codepage", '&', "eval SystemLocale"); // Resource_FormatType_SystemLocale 5
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1250");       // Resource_FormatType_CP1250 6
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1251");       // Resource_FormatType_CP1251 7
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1252");       // Resource_FormatType_CP1252 8
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1253");       // Resource_FormatType_CP1253 9
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1254");       // Resource_FormatType_CP1254 10
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1255");       // Resource_FormatType_CP1255 11
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1256");       // Resource_FormatType_CP1256 12
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1257");       // Resource_FormatType_CP1257 13
        theStatics.Init("step", "read.step.codepage", '&', "eval CP1258");       // Resource_FormatType_CP1258 14
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-1");    // Resource_FormatType_iso8859_1 15
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-2");    // Resource_FormatType_iso8859_2 16
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-3");    // Resource_FormatType_iso8859_3 17
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-4");    // Resource_FormatType_iso8859_4 18
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-5");    // Resource_FormatType_iso8859_5 19
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-6");    // Resource_FormatType_iso8859_6 20
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-7");    // Resource_FormatType_iso8859_7 21
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-8");    // Resource_FormatType_iso8859_8 22
        theStatics.Init("step", "read.step.codepage", '&', "eval iso8859-9");    // Resource_FormatType_iso8859_9 23
        theStatics.Init("step", "read.step.codepage", '&', "eval CP850");        // Resource_FormatType_CP850 24
        theStatics.SetCVal("read.step.codepage", "UTF8");

        // Tessellated geometry reading: Off by default
        theStatics.Init("step", "read.step.tessellated", 'e', "");
        theStatics.Init("step", "read.step.tessellated", '&', "enum 0");
        theStatics.Init("step", "read.step.tessellated", '&', "eval Off");      // 0
        theStatics.Init("step", "read.step.tessellated", '&', "eval On");       // 1
        theStatics.Init("step", "read.step.tessellated", '&', "eval OnNoBRep"); // 2
        theStatics.SetCVal("read.step.tessellated", "On");

        // Tessellated geometry writing: Off by default
        theStatics.Init("step", "write.step.tessellated", 'e', "");
        theStatics.Init("step", "write.step.tessellated", '&', "enum 0");
        theStatics.Init("step", "write.step.tessellated", '&', "eval Off");      // 0
        theStatics.Init("step", "write.step.tessellated", '&', "eval On");       // 1
        theStatics.Init("step", "write.step.tessellated", '&', "eval OnNoBRep"); // 2
        theStatics.SetCVal("write.step.tessellated", "OnNoBRep");
    }

    /// <summary>Creates a new empty Model ready to receive data of the Norm. It is taken from STEP Template Model</summary>
    public override Interface_InterfaceModel NewModel()
    {
        return STEPEdit.NewModel();
    }

    /// <summary>Returns the Actor for Read attached to the pair (norm,appli)</summary>
    public override Transfer_ActorOfTransientProcess ActorRead(Interface_InterfaceModel theModel)
    {
        STEPControl_ActorRead anAdap = myAdaptorRead as STEPControl_ActorRead;
        if (anAdap is null)
        {
            anAdap = new STEPControl_ActorRead(theModel);
            anAdap.SetModel(theModel);
        }
        return anAdap;
    }

    //  ####    PROVISOIRE ???   ####

    /// <summary>
    /// Takes one Shape and transfers it to the InterfaceModel (already created by NewModel for instance)
    /// &lt;modeshape&gt; is to be interpreted by each kind of XstepAdaptor Returns a status : 0 OK 1 No result 2
    /// Fail -1 bad modeshape -2 bad model (requires a StepModel) modeshape : 1 Facetted BRep, 2 Shell, 3 Manifold
    /// Solid
    /// </summary>
    public override IFSelect_ReturnStatus TransferWriteShape(Shape shape, Transfer_FinderProcess FP, Interface_InterfaceModel model,
                                                             int modeshape = 0, ProgressRange theProgress = default)
    {
        if (modeshape < 0 || modeshape > 4)
        {
            return IFSelect_ReturnStatus.IFSelect_RetError;
        }
        STEPControl_ActorWrite ActWrite = myAdaptorWrite as STEPControl_ActorWrite;
        //    A PRESENT ON PASSE PAR LE PROFILE
        StepData_StepModel aModel = model as StepData_StepModel;
        if (ActWrite is not null)
        {
            ActWrite.SetGroupMode((int)aModel.InternalParameters.WriteAssembly);
        }
        Shape aShape = shape;
        NCollection_DataMap<Shape, Shape> aModifedMap = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        if (aModel.InternalParameters.WriteNonmanifold)
        {
            RemoveLocations aRemLoc = new RemoveLocations();
            aRemLoc.SetRemoveLevel(ShapeType.Compound);
            aRemLoc.Remove(aShape);
            aShape = aRemLoc.GetResult();
            aModifedMap = aRemLoc.GetModifiedShapesMap();
        }
        IFSelect_ReturnStatus aStatus = base.TransferWriteShape(aShape, FP, model, modeshape, theProgress);
        XSAlgo_ShapeProcessor.MergeShapeTransferInfo(FP, aModifedMap, null);
        return aStatus;
    }

    /// <summary>
    /// Standard Initialisation. It creates a Controller for STEP and records it to various names, available to
    /// select it later Returns True when done, False if could not be done
    /// </summary>
    /// <remarks>OCCT.NET: once per process, thread-safe; also initialises ShapeAlgo (decision 1241).</remarks>
    public static bool Init()
    {
        lock (theInitLock)
        {
            if (theRecorded is null)
            {
                global::Kerf.Engine.Healing.ShapeAlgo.Init();
                STEPControl_Controller STEPCTL = new STEPControl_Controller();
                STEPCTL.AutoRecord(); // avec les noms donnes a la construction
                // OCCT's XSAlgo::Init() declares XSAlgo's statics process-wide: here per parameter set (InitStatics)
                theRecorded = STEPCTL;
            }
        }
        return true;
    }

    /// <summary>
    /// Selects STEP in a session as STEPControl_Reader and STEPControl_Writer do (OCCT: <c>SelectNorm("STEP")</c>, the
    /// controller recorded process-wide): a session already working with a STEP controller keeps it; another gets a
    /// controller of its own, so that the write actor's settings and counters belong to the session (OCCT.NET, decision
    /// 13201). As SelectNorm, it clears the session's transfer reader first.
    /// </summary>
    public static bool SelectFor(XSControl_WorkSession theSession)
    {
        Init();
        theSession.TransferReader()?.Clear(-1);
        if (theSession.NormAdaptor() is STEPControl_Controller)
        {
            return true;
        }
        theSession.SetController(new STEPControl_Controller());
        return true;
    }

    public override void Customise(XSControl_WorkSession WS)
    {
        base.Customise(WS);

        IFSelect_SelectModelRoots slr;
        Transient slr1 = WS.NamedItem("xst-model-roots");
        if (slr1 is not null)
        {
            slr = slr1 as IFSelect_SelectModelRoots;
        }
        else
        {
            slr = new IFSelect_SelectModelRoots();
            WS.AddNamedItem("xst-model-roots", slr);
        }

        STEPSelections_SelectForTransfer st1 = new STEPSelections_SelectForTransfer();
        st1.SetReader(WS.TransferReader());
        WS.AddNamedItem("xst-transferrable-roots", st1);

        if (slr is not null)
        {
            IFSelect_Signature sty = STEPEdit.SignType();
            WS.AddNamedItem("step-type", sty);

            IFSelect_SignCounter tys = new IFSelect_SignCounter(sty, false, true);
            WS.AddNamedItem("step-types", tys);

            // szv:mySignType = sty;
            WS.SetSignType(sty);

            // pdn S4133 18.02.99
            WS.AddNamedItem("xst-derived", new IFSelect_SignAncestor());
            STEPSelections_SelectDerived stdvar = new STEPSelections_SelectDerived();
            stdvar.SetProtocol(STEPEdit.Protocol());
            WS.AddNamedItem("step-derived", stdvar);

            IFSelect_SelectSignature selsdr = STEPEdit.NewSelectSDR();
            selsdr.SetInput(slr);
            WS.AddNamedItem("step-shape-def-repr", selsdr);
            IFSelect_SelectSignature selrrs = STEPEdit.NewSelectPlacedItem();
            WS.AddNamedItem("step-placed-items", selrrs);
            IFSelect_SelectSignature selsr = STEPEdit.NewSelectShapeRepr();
            // input deja pret avec ModelAll
            WS.AddNamedItem("step-shape-repr", selsr);
        }

        // pdn
        STEPSelections_SelectFaces stfaces = new STEPSelections_SelectFaces();
        stfaces.SetInput(slr);
        WS.AddNamedItem("step-faces", stfaces);

        STEPSelections_SelectInstances stinst = new STEPSelections_SelectInstances();
        WS.AddNamedItem("step-instances", stinst);

        STEPSelections_SelectGSCurves stcurves = new STEPSelections_SelectGSCurves();
        stcurves.SetInput(slr);
        WS.AddNamedItem("step-GS-curves", stcurves);

        STEPSelections_SelectAssembly assembly = new STEPSelections_SelectAssembly();
        assembly.SetInput(slr);
        WS.AddNamedItem("step-assembly", assembly);

        APIHeaderSection_EditHeader edhead = new APIHeaderSection_EditHeader();
        IFSelect_EditForm edheadf = new IFSelect_EditForm(edhead, false, true, "Step Header");
        WS.AddNamedItem("step-header-edit", edhead);
        WS.AddNamedItem("step-header", edheadf);

        STEPEdit_EditContext edctx = new STEPEdit_EditContext();
        IFSelect_EditForm edctxf = new IFSelect_EditForm(edctx, false, true, "STEP Product Definition Context");
        WS.AddNamedItem("step-context-edit", edctx);
        WS.AddNamedItem("step-context", edctxf);

        STEPEdit_EditSDR edsdr = new STEPEdit_EditSDR();
        IFSelect_EditForm edsdrf = new IFSelect_EditForm(edsdr, false, true, "STEP Product Data (SDR)");
        WS.AddNamedItem("step-SDR-edit", edsdr);
        WS.AddNamedItem("step-SDR-data", edsdrf);
    }
}
