// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Controller.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Controller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Ap214.ReadWrite;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Edit;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Selections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Exchange.Step;

/// <summary>defines basic controller for STEP processor</summary>
/// <remarks>
/// <para>
/// Kerf (plan 2.7, decisions 1181, 1222, 1264): OCCT's constructor registers the STEP modules and declares STEP's
/// statics in the process-wide <c>Interface_Static</c> the first time; the port registers the modules once per process
/// (<see cref="Init"/> and the constructor, idempotent) and declares the statics in each session's parameter set
/// (<see cref="InitStatics"/>, once per set). <see cref="Init"/> records one controller under "STEP" and "step", as
/// OCCT's; STEPControl_Reader and STEPControl_Writer give their session a controller of its own
/// (<see cref="SelectFor"/>), so that the write actor's settings (tolerance, group mode, shape fix parameters) and
/// counters (product names, assembly occurrences) are the session's.
/// </para>
/// </remarks>
internal class StepController : Controller
{
    // The controller recorded under "STEP" and "step", made once (a one-time initialisation, unit 10I)
    private static readonly Lazy<StepController> TheRecorded = new(() =>
    {
        global::Kerf.Engine.Healing.ShapeAlgo.Init();
        StepController stepctl = new StepController();
        stepctl.AutoRecord(); // avec les noms donnes a la construction
        // OCCT's XSAlgo::Init() declares XSAlgo's statics process-wide: here per parameter set (InitStatics)
        return stepctl;
    });

    //  Pour NewModel et Write : definition de produit (temporaire ...)
    /// <summary>Initializes the use of STEP Norm (the first time) and returns a Controller</summary>
    public StepController() : base("STEP", "step")
    {
        RegisterModules();

        ActorWrite actWrite = new ActorWrite();
        _adaptorWrite = actWrite;

        actWrite.SetShapeFixParameters(Parameters.GetDefaultShapeFixParameters(), new Dictionary<string, string>());
        global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags defaultProcFlags = new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.SplitCommonVertex,
                                                                                         global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.DirectFaces);
        actWrite.SetShapeProcessFlags(defaultProcFlags);

        StepWorkLibrary swl = new StepWorkLibrary();
        swl.SetDumpLabel(1);
        _adaptorLibrary = swl;
        _adaptorProtocol = StepEdit.Protocol();

        SetModeWrite(0, 4);
        SetModeWriteHelp(0, "As Is");
        SetModeWriteHelp(1, "Faceted Brep");
        SetModeWriteHelp(2, "Shell Based");
        SetModeWriteHelp(3, "Manifold Solid");
        SetModeWriteHelp(4, "Wireframe");
        TraceStatic("read.surfacecurve.mode", 5);

        //   ---  SELECTIONS, SIGNATURES, COMPTEURS, EDITEURS

        AbstractSelection xmr = SessionItem("xst-model-roots") as AbstractSelection;
        if (xmr is not null)
        {
            Signature sty = StepEdit.SignType();
            AddSessionItem(sty, "step-type");
            SignCounter tys = new SignCounter(sty, false, true);
            AddSessionItem(tys, "step-types");

            // szv:mySignType = sty;

            // pdn S4133 18.02.99
            AddSessionItem(new SignAncestor(), "xst-derived");

            SelectDerived stdvar = new SelectDerived();
            stdvar.SetProtocol(StepEdit.Protocol());
            AddSessionItem(stdvar, "step-derived");

            SelectSignature selsdr = StepEdit.NewSelectSDR();
            selsdr.SetInput(xmr);
            AddSessionItem(selsdr, "step-shape-def-repr");

            AddSessionItem(StepEdit.NewSelectPlacedItem(), "step-placed-items");
            // input deja pret avec ModelAll
            AddSessionItem(StepEdit.NewSelectShapeRepr(), "step-shape-repr");
        }

        // pdn
        SelectFaces stfaces = new SelectFaces();
        stfaces.SetInput(xmr);
        AddSessionItem(stfaces, "step-faces");

        SelectInstances stinst = new SelectInstances();
        AddSessionItem(stinst, "step-instances");

        SelectGSCurves stcurves = new SelectGSCurves();
        stcurves.SetInput(xmr);
        AddSessionItem(stcurves, "step-GS-curves");

        SelectAssembly assembly = new SelectAssembly();
        assembly.SetInput(xmr);
        AddSessionItem(assembly, "step-assembly");

        EditHeader edhead = new EditHeader();
        EditForm edheadf = new EditForm(edhead, false, true, "Step Header");
        AddSessionItem(edhead, "step-header-edit");
        AddSessionItem(edheadf, "step-header");

        EditContext edctx = new EditContext();
        EditForm edctxf = new EditForm(edctx, false, true, "STEP Product Definition Context");
        AddSessionItem(edctx, "step-context-edit");
        AddSessionItem(edctxf, "step-context");

        EditSdr edsdr = new EditSdr();
        EditForm edsdrf = new EditForm(edsdr, false, true, "STEP Product Data (SDR)");
        AddSessionItem(edsdr, "step-SDR-edit");
        AddSessionItem(edsdrf, "step-SDR-data");
    }

    // OCCT's constructor's first part (behind its static flag and mutex): the modules, once per process; the statics
    // it declares there are declared per parameter set (InitStatics)
    private static void RegisterModules()
    {
        global::Kerf.Engine.Exchange.Step.Header.RWHeaderSection.Init();
        Ap214ReadWrite.Init();
    }

    /// <summary>
    /// Declares STEP's statics (OCCT's constructor's <c>Interface_Static::Init</c> calls) in a parameter set, once per
    /// set, after the standard ones and XSAlgo's (Kerf, decision 1181).
    /// </summary>
    public override void InitStatics(StaticSet statics)
    {
        base.InitStatics(statics);
        global::Kerf.Engine.Exchange.Framework.XSAlgo.Init(statics);
        if (!statics.StartInit("STEPControl_Controller"))
        {
            return;
        }

        statics.Init("step", "write.step.product.name", 't', "Kerf STEP translator");
        statics.Init("step", "write.step.assembly", 'e', "");
        statics.Init("step", "write.step.assembly", '&', "enum 0");
        statics.Init("step", "write.step.assembly", '&', "eval Off");
        statics.Init("step", "write.step.assembly", '&', "eval On");
        statics.Init("step", "write.step.assembly", '&', "eval Auto");
        statics.SetCVal("write.step.assembly", "Auto");

        statics.Init("step", "step.angleunit.mode", 'e', "");
        statics.Init("step", "step.angleunit.mode", '&', "enum 0");
        statics.Init("step", "step.angleunit.mode", '&', "eval File");
        statics.Init("step", "step.angleunit.mode", '&', "eval Rad");
        statics.Init("step", "step.angleunit.mode", '&', "eval Deg");
        statics.SetCVal("step.angleunit.mode", "File");

        statics.Init("step", "write.step.schema", 'e', "");
        statics.Init("step", "write.step.schema", '&', "enum 1");
        statics.Init("step", "write.step.schema", '&', "eval AP214CD");
        statics.Init("step", "write.step.schema", '&', "eval AP214DIS");
        statics.Init("step", "write.step.schema", '&', "eval AP203");
        statics.Init("step", "write.step.schema", '&', "eval AP214IS");
        statics.Init("step", "write.step.schema", '&', "eval AP242DIS");
        statics.SetCVal("write.step.schema", "AP214IS");

        // Type of Product Definition for reading
        // Note: the numbers should be consistent with function FindShapeReprType()
        // in STEPControl_ActorRead.cxx
        statics.Init("step", "read.step.shape.repr", 'e', "");
        statics.Init("step", "read.step.shape.repr", '&', "enum 1");
        statics.Init("step", "read.step.shape.repr", '&', "eval All");   // 1
        statics.Init("step", "read.step.shape.repr", '&', "eval ABSR");  // 2
        statics.Init("step", "read.step.shape.repr", '&', "eval MSSR");  // 3
        statics.Init("step", "read.step.shape.repr", '&', "eval GBSSR"); // 4
        statics.Init("step", "read.step.shape.repr", '&', "eval FBSR");  // 5
        statics.Init("step", "read.step.shape.repr", '&', "eval EBWSR"); // 6
        statics.Init("step", "read.step.shape.repr", '&', "eval GBWSR"); // 7
        statics.SetCVal("read.step.shape.repr", "All");

        // Mode for reading shapes attached to main SDR by SRR
        // (hybrid model representation in AP203 since 1998)
        statics.Init("step", "read.step.shape.relationship", 'e', "");
        statics.Init("step", "read.step.shape.relationship", '&', "enum 0");
        statics.Init("step", "read.step.shape.relationship", '&', "eval OFF");
        statics.Init("step", "read.step.shape.relationship", '&', "eval ON");
        statics.SetCVal("read.step.shape.relationship", "ON");

        // Mode for reading shapes attached to Product by ShapeAspect
        // (hybrid model representation in AP203 before 1998)
        statics.Init("step", "read.step.shape.aspect", 'e', "");
        statics.Init("step", "read.step.shape.aspect", '&', "enum 0");
        statics.Init("step", "read.step.shape.aspect", '&', "eval OFF");
        statics.Init("step", "read.step.shape.aspect", '&', "eval ON");
        statics.SetCVal("read.step.shape.aspect", "ON");

        // Mode for reading SDR and ShapeRepr if it is necessary
        statics.Init("step", "read.step.product.mode", 'e', "");
        statics.Init("step", "read.step.product.mode", '&', "enum 0");
        statics.Init("step", "read.step.product.mode", '&', "eval OFF");
        statics.Init("step", "read.step.product.mode", '&', "eval ON");
        statics.SetCVal("read.step.product.mode", "ON");

        // Order of reading ShapeDefinitionRepresentation in ProductDefinition
        statics.Init("step", "read.step.product.context", 'e', "");
        statics.Init("step", "read.step.product.context", '&', "enum 1");
        statics.Init("step", "read.step.product.context", '&', "eval all");      // 1
        statics.Init("step", "read.step.product.context", '&', "eval design");   // 2
        statics.Init("step", "read.step.product.context", '&', "eval analysis"); // 3
        statics.SetCVal("read.step.product.context", "all");

        // What we try to read in ProductDefinition
        statics.Init("step", "read.step.assembly.level", 'e', "");
        statics.Init("step", "read.step.assembly.level", '&', "enum 1");
        statics.Init("step", "read.step.assembly.level", '&', "eval all");       // 1
        statics.Init("step", "read.step.assembly.level", '&', "eval assembly");  // 2
        statics.Init("step", "read.step.assembly.level", '&', "eval structure"); // 3
        statics.Init("step", "read.step.assembly.level", '&', "eval shape");     // 4
        statics.SetCVal("read.step.assembly.level", "all");

        // unit: supposed to be cascade unit (target unit for reading)
        statics.Init("step", "write.step.unit", 'e', "");
        statics.Init("step", "write.step.unit", '&', "enum 1");
        statics.Init("step", "write.step.unit", '&', "eval INCH"); // 1
        statics.Init("step", "write.step.unit", '&', "eval MM");   // 2
        statics.Init("step", "write.step.unit", '&', "eval ??");   // 3
        statics.Init("step", "write.step.unit", '&', "eval FT");   // 4
        statics.Init("step", "write.step.unit", '&', "eval MI");   // 5
        statics.Init("step", "write.step.unit", '&', "eval M");    // 6
        statics.Init("step", "write.step.unit", '&', "eval KM");   // 7
        statics.Init("step", "write.step.unit", '&', "eval MIL");  // 8
        statics.Init("step", "write.step.unit", '&', "eval UM");   // 9
        statics.Init("step", "write.step.unit", '&', "eval CM");   // 10
        statics.Init("step", "write.step.unit", '&', "eval UIN");  // 11
        statics.SetCVal("write.step.unit", "MM");

        // Non-manifold topology reading: OFF by default (ssv; 26.11.2010)
        statics.Init("step", "read.step.nonmanifold", 'e', "");
        statics.Init("step", "read.step.nonmanifold", '&', "enum 0");
        statics.Init("step", "read.step.nonmanifold", '&', "eval Off");
        statics.Init("step", "read.step.nonmanifold", '&', "eval On");
        statics.SetIVal("read.step.nonmanifold", 0);

        // Non-manifold topology writing: OFF by default (ssv; 26.11.2010)
        statics.Init("step", "write.step.nonmanifold", 'e', "");
        statics.Init("step", "write.step.nonmanifold", '&', "enum 0");
        statics.Init("step", "write.step.nonmanifold", '&', "eval Off");
        statics.Init("step", "write.step.nonmanifold", '&', "eval On");
        statics.SetIVal("write.step.nonmanifold", 0);

        // I-Deas-like STEP processing: OFF by default (ssv; 22.11.2010)
        statics.Init("step", "read.step.ideas", 'e', "");
        statics.Init("step", "read.step.ideas", '&', "enum 0");
        statics.Init("step", "read.step.ideas", '&', "eval Off");
        statics.Init("step", "read.step.ideas", '&', "eval On");
        statics.SetIVal("read.step.ideas", 0);

        // Parameter to write all free vertices in one SDR (name and style of vertex are lost) (default)
        // or each vertex in its own SDR (name and style of vertex are exported). (ika; 21.07.2014)
        statics.Init("step", "write.step.vertex.mode", 'e', "");
        statics.Init("step", "write.step.vertex.mode", '&', "enum 0");
        statics.Init("step", "write.step.vertex.mode", '&', "eval One Compound");
        statics.Init("step", "write.step.vertex.mode", '&', "eval Single Vertex");
        statics.SetIVal("write.step.vertex.mode", 0);

        // abv 15.11.00: ShapeProcessing
        statics.Init("XSTEP", "write.step.resource.name", 't', "STEP");
        statics.Init("XSTEP", "read.step.resource.name", 't', "STEP");
        statics.Init("XSTEP", "write.step.sequence", 't', "ToSTEP");
        statics.Init("XSTEP", "read.step.sequence", 't', "FromSTEP");
        statics.Init("XSTEP", "ToSTEP.exec.op", 't', "SplitCommonVertex,DirectFaces");
        statics.Init("XSTEP", "FromSTEP.exec.op", 't', "FixShape");
        statics.Init("XSTEP", "FromSTEP.FixShape.Tolerance3d", 't', "&Runtime.Tolerance");
        statics.Init("XSTEP", "FromSTEP.FixShape.MaxTolerance3d", 't', "&Runtime.MaxTolerance");
        statics.Init("XSTEP", "FromSTEP.FixShape.MinTolerance3d", 't', "1.e-7");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixFreeShellMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixFreeFaceMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixFreeWireMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSameParameterMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSolidMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixShellOrientationMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.CreateOpenSolidMode", 't', "0");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixShellMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixFaceOrientationMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixFaceMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixWireMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixOrientationMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixAddNaturalBoundMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixMissingSeamMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSmallAreaWireMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.RemoveSmallAreaFaceMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixIntersectingWiresMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixLoopWiresMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSplitFaceMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.AutoCorrectPrecisionMode", 't', "1");
        statics.Init("XSTEP", "FromSTEP.FixShape.ModifyTopologyMode", 't', "0");
        statics.Init("XSTEP", "FromSTEP.FixShape.ModifyGeometryMode", 't', "1");
        statics.Init("XSTEP", "FromSTEP.FixShape.ClosedWireMode", 't', "1");
        statics.Init("XSTEP", "FromSTEP.FixShape.PreferencePCurveMode", 't', "1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixReorderMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSmallMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixConnectedMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixEdgeCurvesMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixDegeneratedMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixLackingMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSelfIntersectionMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.RemoveLoopMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixReversed2dMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixRemovePCurveMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixRemoveCurve3dMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixAddPCurveMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixAddCurve3dMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSeamMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixShiftedMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixEdgeSameParameterMode", 't', "0");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixNotchedEdgesMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixTailMode", 't', "0");
        statics.Init("XSTEP", "FromSTEP.FixShape.MaxTailAngle", 't', "0.0");
        statics.Init("XSTEP", "FromSTEP.FixShape.MaxTailWidth", 't', "-1.0");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixSelfIntersectingEdgeMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixIntersectingEdgesMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixNonAdjacentIntersectingEdgesMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixVertexPositionMode", 't', "0");
        statics.Init("XSTEP", "FromSTEP.FixShape.FixVertexToleranceMode", 't', "-1");
        statics.Init("XSTEP", "FromSTEP.FixFaceSize.Tolerance", 't', "1.e-7");

        // ika 28.07.16: Parameter to read all top level solids and shells,
        // should be used only in case of invalid shape_representation without links to shapes.
        statics.Init("step", "read.step.all.shapes", 'e', "");
        statics.Init("step", "read.step.all.shapes", '&', "enum 0");
        statics.Init("step", "read.step.all.shapes", '&', "eval Off");
        statics.Init("step", "read.step.all.shapes", '&', "eval On");
        statics.SetIVal("read.step.all.shapes", 0);

        // Mode for reading constructive geometry representation relationship to read
        // StepRepr_ConstructiveGeometryRepresentation method implemented only for
        // StepGeom_MakeAxis2Placement3d for axis placements representing axis for supplemented
        // geometry. Axis placements are translated to planar faces with CS equal to translated axis
        // placements
        statics.Init("step", "read.step.constructivegeom.relationship", 'e', "");
        statics.Init("step", "read.step.constructivegeom.relationship", '&', "enum 0");
        statics.Init("step", "read.step.constructivegeom.relationship", '&', "eval OFF");
        statics.Init("step", "read.step.constructivegeom.relationship", '&', "eval ON");
        statics.SetCVal("read.step.constructivegeom.relationship", "OFF");

        // Mode to variate apply or not transformation placed in the root shape representation.
        // Issues #29068 and #31491.
        statics.Init("step", "read.step.root.transformation", 'e', "");
        statics.Init("step", "read.step.root.transformation", '&', "enum 0");
        statics.Init("step", "read.step.root.transformation", '&', "eval OFF");
        statics.Init("step", "read.step.root.transformation", '&', "eval ON");
        statics.SetCVal("read.step.root.transformation", "ON");

        // STEP file encoding for names translation
        // Note: the numbers should be consistent with Resource_FormatType enumeration
        statics.Init("step", "read.step.codepage", 'e', "");
        statics.Init("step", "read.step.codepage", '&', "enum 0");
        statics.Init("step", "read.step.codepage", '&', "eval SJIS");         // Resource_FormatType_SJIS 0
        statics.Init("step", "read.step.codepage", '&', "eval EUC");          // Resource_FormatType_EUC 1
        statics.Init("step", "read.step.codepage", '&', "eval NoConversion"); // Resource_FormatType_NoConversion 2
        statics.Init("step", "read.step.codepage", '&', "eval GB");           // Resource_FormatType_GB 3
        statics.Init("step", "read.step.codepage", '&', "eval UTF8");         // Resource_FormatType_UTF8 4
        statics.Init("step", "read.step.codepage", '&', "eval SystemLocale"); // Resource_FormatType_SystemLocale 5
        statics.Init("step", "read.step.codepage", '&', "eval CP1250");       // Resource_FormatType_CP1250 6
        statics.Init("step", "read.step.codepage", '&', "eval CP1251");       // Resource_FormatType_CP1251 7
        statics.Init("step", "read.step.codepage", '&', "eval CP1252");       // Resource_FormatType_CP1252 8
        statics.Init("step", "read.step.codepage", '&', "eval CP1253");       // Resource_FormatType_CP1253 9
        statics.Init("step", "read.step.codepage", '&', "eval CP1254");       // Resource_FormatType_CP1254 10
        statics.Init("step", "read.step.codepage", '&', "eval CP1255");       // Resource_FormatType_CP1255 11
        statics.Init("step", "read.step.codepage", '&', "eval CP1256");       // Resource_FormatType_CP1256 12
        statics.Init("step", "read.step.codepage", '&', "eval CP1257");       // Resource_FormatType_CP1257 13
        statics.Init("step", "read.step.codepage", '&', "eval CP1258");       // Resource_FormatType_CP1258 14
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-1");    // Resource_FormatType_iso8859_1 15
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-2");    // Resource_FormatType_iso8859_2 16
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-3");    // Resource_FormatType_iso8859_3 17
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-4");    // Resource_FormatType_iso8859_4 18
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-5");    // Resource_FormatType_iso8859_5 19
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-6");    // Resource_FormatType_iso8859_6 20
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-7");    // Resource_FormatType_iso8859_7 21
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-8");    // Resource_FormatType_iso8859_8 22
        statics.Init("step", "read.step.codepage", '&', "eval iso8859-9");    // Resource_FormatType_iso8859_9 23
        statics.Init("step", "read.step.codepage", '&', "eval CP850");        // Resource_FormatType_CP850 24
        statics.SetCVal("read.step.codepage", "UTF8");

        // Tessellated geometry reading: Off by default
        statics.Init("step", "read.step.tessellated", 'e', "");
        statics.Init("step", "read.step.tessellated", '&', "enum 0");
        statics.Init("step", "read.step.tessellated", '&', "eval Off");      // 0
        statics.Init("step", "read.step.tessellated", '&', "eval On");       // 1
        statics.Init("step", "read.step.tessellated", '&', "eval OnNoBRep"); // 2
        statics.SetCVal("read.step.tessellated", "On");

        // Tessellated geometry writing: Off by default
        statics.Init("step", "write.step.tessellated", 'e', "");
        statics.Init("step", "write.step.tessellated", '&', "enum 0");
        statics.Init("step", "write.step.tessellated", '&', "eval Off");      // 0
        statics.Init("step", "write.step.tessellated", '&', "eval On");       // 1
        statics.Init("step", "write.step.tessellated", '&', "eval OnNoBRep"); // 2
        statics.SetCVal("write.step.tessellated", "OnNoBRep");
    }

    /// <summary>Creates a new empty Model ready to receive data of the Norm. It is taken from STEP Template Model</summary>
    public override InterfaceModel NewModel()
    {
        return StepEdit.NewModel();
    }

    /// <summary>Returns the Actor for Read attached to the pair (norm,appli)</summary>
    public override ActorOfTransientProcess ActorRead(InterfaceModel model)
    {
        ActorRead adap = _adaptorRead as ActorRead;
        if (adap is null)
        {
            adap = new ActorRead(model);
            adap.SetModel(model);
        }
        return adap;
    }

    //  ####    PROVISOIRE ???   ####

    /// <summary>
    /// Takes one Shape and transfers it to the InterfaceModel (already created by NewModel for instance)
    /// &lt;modeshape&gt; is to be interpreted by each kind of XstepAdaptor Returns a status : 0 OK 1 No result 2
    /// Fail -1 bad modeshape -2 bad model (requires a StepModel) modeshape : 1 Facetted BRep, 2 Shell, 3 Manifold
    /// Solid
    /// </summary>
    public override ReturnStatus TransferWriteShape(Shape shape, FinderProcess fp, InterfaceModel model,
                                                             int modeshape = 0, ProgressRange progress = default)
    {
        if (modeshape < 0 || modeshape > 4)
        {
            return ReturnStatus.RetError;
        }
        ActorWrite actWrite = _adaptorWrite as ActorWrite;
        //    A PRESENT ON PASSE PAR LE PROFILE
        StepModel aModel = model as StepModel;
        if (actWrite is not null)
        {
            actWrite.SetGroupMode((int)aModel.InternalParameters.WriteAssembly);
        }
        Shape aShape = shape;
        Dictionary<Shape, Shape> modifedMap = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        if (aModel.InternalParameters.WriteNonmanifold)
        {
            RemoveLocations remLoc = new RemoveLocations();
            remLoc.SetRemoveLevel(ShapeType.Compound);
            remLoc.Remove(aShape);
            aShape = remLoc.GetResult();
            modifedMap = remLoc.GetModifiedShapesMap();
        }
        ReturnStatus status = base.TransferWriteShape(aShape, fp, model, modeshape, progress);
        ShapeProcessor.MergeShapeTransferInfo(fp, modifedMap, null);
        return status;
    }

    /// <summary>
    /// Standard Initialisation. It creates a Controller for STEP and records it to various names, available to
    /// select it later Returns True when done, False if could not be done
    /// </summary>
    /// <remarks>Kerf: once per process, thread-safe; also initialises ShapeAlgo (decision 1241).</remarks>
    public static bool Init()
    {
        _ = TheRecorded.Value;
        return true;
    }

    /// <summary>
    /// Selects STEP in a session as STEPControl_Reader and STEPControl_Writer do (OCCT: <c>SelectNorm("STEP")</c>, the
    /// controller recorded process-wide): a session already working with a STEP controller keeps it; another gets a
    /// controller of its own, so that the write actor's settings and counters belong to the session (Kerf, decision
    /// 13201). As SelectNorm, it clears the session's transfer reader first.
    /// </summary>
    public static bool SelectFor(ExchangeWorkSession session)
    {
        Init();
        session.TransferReader()?.Clear(-1);
        if (session.NormAdaptor() is StepController)
        {
            return true;
        }
        session.SetController(new StepController());
        return true;
    }

    public override void Customise(ExchangeWorkSession ws)
    {
        base.Customise(ws);

        SelectModelRoots slr;
        Transient slr1 = ws.NamedItem("xst-model-roots");
        if (slr1 is not null)
        {
            slr = slr1 as SelectModelRoots;
        }
        else
        {
            slr = new SelectModelRoots();
            ws.AddNamedItem("xst-model-roots", slr);
        }

        StepSelectForTransfer st1 = new StepSelectForTransfer();
        st1.SetReader(ws.TransferReader());
        ws.AddNamedItem("xst-transferrable-roots", st1);

        if (slr is not null)
        {
            Signature sty = StepEdit.SignType();
            ws.AddNamedItem("step-type", sty);

            SignCounter tys = new SignCounter(sty, false, true);
            ws.AddNamedItem("step-types", tys);

            // szv:mySignType = sty;
            ws.SetSignType(sty);

            // pdn S4133 18.02.99
            ws.AddNamedItem("xst-derived", new SignAncestor());
            SelectDerived stdvar = new SelectDerived();
            stdvar.SetProtocol(StepEdit.Protocol());
            ws.AddNamedItem("step-derived", stdvar);

            SelectSignature selsdr = StepEdit.NewSelectSDR();
            selsdr.SetInput(slr);
            ws.AddNamedItem("step-shape-def-repr", selsdr);
            SelectSignature selrrs = StepEdit.NewSelectPlacedItem();
            ws.AddNamedItem("step-placed-items", selrrs);
            SelectSignature selsr = StepEdit.NewSelectShapeRepr();
            // input deja pret avec ModelAll
            ws.AddNamedItem("step-shape-repr", selsr);
        }

        // pdn
        SelectFaces stfaces = new SelectFaces();
        stfaces.SetInput(slr);
        ws.AddNamedItem("step-faces", stfaces);

        SelectInstances stinst = new SelectInstances();
        ws.AddNamedItem("step-instances", stinst);

        SelectGSCurves stcurves = new SelectGSCurves();
        stcurves.SetInput(slr);
        ws.AddNamedItem("step-GS-curves", stcurves);

        SelectAssembly assembly = new SelectAssembly();
        assembly.SetInput(slr);
        ws.AddNamedItem("step-assembly", assembly);

        EditHeader edhead = new EditHeader();
        EditForm edheadf = new EditForm(edhead, false, true, "Step Header");
        ws.AddNamedItem("step-header-edit", edhead);
        ws.AddNamedItem("step-header", edheadf);

        EditContext edctx = new EditContext();
        EditForm edctxf = new EditForm(edctx, false, true, "STEP Product Definition Context");
        ws.AddNamedItem("step-context-edit", edctx);
        ws.AddNamedItem("step-context", edctxf);

        EditSdr edsdr = new EditSdr();
        EditForm edsdrf = new EditForm(edsdr, false, true, "STEP Product Data (SDR)");
        ws.AddNamedItem("step-SDR-edit", edsdr);
        ws.AddNamedItem("step-SDR-data", edsdrf);
    }
}
