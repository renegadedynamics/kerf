// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Writer.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Writer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Kerf.Engine.Exchange.Step;

/// <summary>
/// This class creates and writes STEP files from Open CASCADE models. A STEP file can be written to an existing
/// STEP file or to a new one. Translation can be performed in one or several operations. Each translation operation
/// outputs a distinct root entity in the STEP file.
/// </summary>
/// <remarks>
/// <para>A C++ value class, a C# class here (decision 1185). Kerf (plan 2.7, decisions 1264, 1265, 1267):</para>
/// <list type="bullet">
/// <item>the writer never changes the shape it is given;</item>
/// <item>the parameters come from the session's parameter set (<see cref="WorkSession.Statics"/>), or from
/// a <see cref="Parameters"/> given to the transfer;</item>
/// <item>the session gets a STEP controller of its own (not the one recorded process-wide), so that the write
/// actor's settings (<see cref="SetTolerance"/>, the shape fix parameters, the group mode) and its counters belong to
/// this writer's session;</item>
/// <item>the header's FILE_NAME (time stamp, author...) is set at the first transfer from the system (the local
/// time); for repeatable output, change it before writing through <c>new APIHeaderSection_MakeHeader(Model())</c>
/// (<c>SetTimeStamp</c>, <c>SetAuthorValue</c>...), which <see cref="Write"/> and <see cref="WriteStream"/> keep.
/// AP203's dates and persons come from the write actor's context (<see cref="ActorWrite.ContextTool"/>).</item>
/// </list>
/// </remarks>
internal class Writer
{
    /// <summary>Creates a Writer from scratch</summary>
    public Writer()
    {
        StepController.Init();
        SetWS(new ExchangeWorkSession());
    }

    /// <summary>Creates a Writer from an already existing Session If &lt;scratch&gt; is True (D), clears already recorded data</summary>
    public Writer(ExchangeWorkSession ws, bool scratch = true)
    {
        StepController.Init();
        SetWS(ws, scratch);
    }

    /// <summary>
    /// Sets a length-measure value that will be written to uncertainty-measure-with-unit when the next shape is
    /// translated.
    /// </summary>
    public void SetTolerance(double tol)
    {
        ActorWrite act = WS().NormAdaptor().ActorWrite() as ActorWrite;
        if (act is not null)
        {
            act.SetTolerance(tol);
        }
    }

    /// <summary>Unsets the tolerance formerly forced by SetTolerance</summary>
    public void UnsetTolerance()
    {
        SetTolerance(-1.0);
    }

    /// <summary>Sets a specific session to &lt;me&gt;</summary>
    public void SetWS(ExchangeWorkSession ws, bool scratch = true)
    {
        _thesession = ws;
        // OCCT: SelectNorm("STEP"), the controller recorded process-wide; here one of the session's own (decision 1264)
        StepController.SelectFor(_thesession);
        _thesession.InitTransferReader(0);
        Model(scratch);
    }

    /// <summary>Returns the session used in &lt;me&gt;</summary>
    public ExchangeWorkSession WS()
    {
        return _thesession;
    }

    /// <summary>
    /// Returns the produced model. Produces a new one if not yet done or if &lt;newone&gt; is True This method allows
    /// for instance to edit product or header data before writing.
    /// </summary>
    public StepModel Model(bool newone = false)
    {
        StepModel model = _thesession.Model() as StepModel;
        if (newone || model is null)
        {
            model = _thesession.NewModel() as StepModel;
        }
        return model;
    }

    /// <summary>
    /// Translates shape sh to a STEP entity. mode defines the STEP entity type to be output: -
    /// STEPControlStd_AsIs translates a shape to its highest possible STEP representation. -
    /// STEPControlStd_ManifoldSolidBrep translates a shape to a STEP manifold_solid_brep or brep_with_voids entity.
    /// - STEPControlStd_FacetedBrep translates a shape into a STEP faceted_brep entity. -
    /// STEPControlStd_ShellBasedSurfaceModel translates a shape into a STEP shell_based_surface_model entity. -
    /// STEPControlStd_GeometricCurveSet translates a shape into a STEP geometric_curve_set entity.
    /// </summary>
    public ReturnStatus Transfer(Shape sh, ModelType mode, bool compgraph = true,
                                          ProgressRange progress = default)
    {
        StepModel stepModel = _thesession.Model() as StepModel;
        if (stepModel is not null)
        {
            // OCCT: from the process-wide statics; here the session's (decision 1181)
            stepModel.InternalParameters.InitFromStatic(_thesession.Statics());
        }
        return Transfer(sh, mode, stepModel.InternalParameters, compgraph, progress);
    }

    /// <summary>
    /// Translates shape sh to a STEP entity, with the given parameters (which the model takes).
    /// </summary>
    public ReturnStatus Transfer(Shape sh, ModelType mode, in Parameters theParams,
                                          bool compgraph = true, ProgressRange progress = default)
    {
        int mws = -1;
        switch (mode)
        {
            case ModelType.AsIs:
                mws = 0;
                break;
            case ModelType.FacetedBrep:
                mws = 1;
                break;
            case ModelType.ShellBasedSurfaceModel:
                mws = 2;
                break;
            case ModelType.ManifoldSolidBrep:
                mws = 3;
                break;
            case ModelType.GeometricCurveSet:
                mws = 4;
                break;
            // Kerf: the actor writes these two (its modes 5 and 6), but OCCT's writer refuses them as unknown (decision 1963)
            case ModelType.BrepWithVoids:
                mws = 5;
                break;
            case ModelType.FacetedBrepAndBrepWithVoids:
                mws = 6;
                break;
            default:
                break;
        }
        if (mws < 0)
        {
            return ReturnStatus.RetError; // cas non reconnu
        }
        _thesession.TransferWriter().SetTransferMode(mws);
        if (!Model().IsInitializedUnit)
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; here the session's
            // xstep.cascade.unit (decision 1265)
            Model().SetLocalLengthUnit(global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(_thesession.Statics()));
        }
        Model().InternalParameters = theParams;
        MakeHeader headerMaker = new MakeHeader();
        headerMaker.Apply(Model());
        ActorWrite actWrite = WS().NormAdaptor().ActorWrite() as ActorWrite;
        actWrite.SetGroupMode((int)(_thesession.Model() as StepModel).InternalParameters.WriteAssembly);
        InitializeMissingParameters();
        return _thesession.TransferWriteShape(sh, compgraph, progress);
    }

    /// <summary>Writes a STEP model in the file identified by filename.</summary>
    public ReturnStatus Write(string fileName)
    {
        StepModel model = Model();
        if (model is null)
        {
            return ReturnStatus.RetFail;
        }
        MakeHeader headerMaker = new MakeHeader();
        headerMaker.Apply(model);
        return _thesession.SendAll(fileName);
    }

    /// <summary>Writes a STEP model in the std::ostream.</summary>
    /// <remarks>Kerf: a .NET stream; the model's text is written as bytes (decision 1192).</remarks>
    public ReturnStatus WriteStream(Stream oStream)
    {
        StepModel model = Model();
        if (model is null)
        {
            return ReturnStatus.RetFail;
        }

        StepProtocol protocol = model.Protocol() as StepProtocol;
        if (protocol is null)
        {
            return ReturnStatus.RetFail;
        }

        StepWriter writer = new StepWriter(model);
        writer.SendModel(protocol);
        MakeHeader headerMaker = new MakeHeader();
        headerMaker.Apply(model);
        return writer.Print(oStream) ? ReturnStatus.RetDone : ReturnStatus.RetFail;
    }

    /// <summary>
    /// Displays the statistics for the last translation. what defines the kind of statistics that are displayed: -
    /// 0 gives general statistics (number of translated roots, number of warnings, number of fail messages), - 1
    /// gives root results, - 2 gives statistics for all checked entities, - 3 gives the list of translated
    /// entities, - 4 gives warning and fail messages, - 5 gives fail messages only. mode is used according to the
    /// use of what. If what is 0, mode is ignored. If what is 1, 2 or 3, mode defines the following: - 0 lists the
    /// numbers of STEP entities in a STEP model, - 1 gives the number, identifier, type and result type for each STEP
    /// entity and/or its status (fail, warning, etc.), - 2 gives maximum information for each STEP entity (i.e.
    /// checks), - 3 gives the number of entities by the type of a STEP entity, - 4 gives the number of of STEP
    /// entities per result type and/or status, - 5 gives the number of pairs (STEP or result type and status), -
    /// 6 gives the number of pairs (STEP or result type and status) AND the list of entity numbers in the STEP
    /// model.
    /// </summary>
    public void PrintStatsTransfer(int what, int mode = 0)
    {
        _thesession.TransferWriter().PrintStats(what, mode);
    }

    /// <summary>Merges equal entities in the model and removes the duplicates, to shorten the file written.</summary>
    public void CleanDuplicateEntities()
    {
        new Tidy.DuplicateCleaner(_thesession).Perform();
    }

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(Dictionary<string, string> parameters)
    {
        ActorOfFinderProcess actor = GetActor();
        if (actor is not null)
        {
            actor.SetShapeFixParameters(parameters);
        }
    }

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(in ShapeFixParameters parameters)
    {
        Dictionary<string, string> additionalParameters = new Dictionary<string, string>();
        ActorOfFinderProcess actor = GetActor();
        if (actor is not null)
        {
            actor.SetShapeFixParameters(parameters, additionalParameters);
        }
    }

    /// <summary>
    /// Sets parameters for shape processing. Parameters from theParameters are copied to the internal map. Parameters
    /// from theAdditionalParameters are copied to the internal map if they are not present in theParameters.
    /// </summary>
    public void SetShapeFixParameters(in ShapeFixParameters parameters, Dictionary<string, string> additionalParameters)
    {
        ActorOfFinderProcess actor = GetActor();
        if (actor is not null)
        {
            actor.SetShapeFixParameters(parameters, additionalParameters);
        }
    }

    /// <summary>Returns parameters for shape processing that was set by SetParameters() method.</summary>
    public Dictionary<string, string> GetShapeFixParameters()
    {
        ActorOfFinderProcess actor = GetActor();
        return actor is null ? new Dictionary<string, string>() : actor.GetShapeFixParameters();
    }

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetShapeProcessFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags flags)
    {
        ActorOfFinderProcess actor = GetActor();
        if (actor is not null)
        {
            actor.SetShapeProcessFlags(flags);
        }
    }

    /// <summary>Returns flags defining operations to be performed on shapes.</summary>
    public (global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags()
    {
        ActorOfFinderProcess actor = GetActor();
        return actor is null ? (new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(), false) : actor.GetShapeProcessFlags();
    }

    /// <summary>Returns the Actor for the Transfer of an Entity. @return the Actor for the Transfer of an Entity. May be nullptr.</summary>
    private ActorOfFinderProcess GetActor()
    {
        ExchangeWorkSession session = WS();
        if (session is null)
        {
            return null;
        }

        Controller controller = session.NormAdaptor();
        if (controller is null)
        {
            return null;
        }

        return controller.ActorWrite();
    }

    /// <summary>
    /// If parameters haven't yet been provided, initializes them with default values provided by
    /// GetDefaultShapeFixParameters() method.
    /// </summary>
    private void InitializeMissingParameters()
    {
        if (GetShapeFixParameters().Count == 0)
        {
            SetShapeFixParameters(Parameters.GetDefaultShapeFixParameters());
        }
        if (!GetShapeProcessFlags().Item2)
        {
            global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags flags = new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.SplitCommonVertex,
                                                                                  global::Kerf.Engine.Healing.Process.ShapeProcess.Operation.DirectFaces);
            SetShapeProcessFlags(flags);
        }
    }

    private ExchangeWorkSession _thesession;
}
