// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Writer.hxx
// Port of: src/DataExchange/TKDESTEP/STEPControl/STEPControl_Writer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_ReturnStatus;
using static Occt.STEPControl_StepModelType;

namespace Occt;

/// <summary>
/// This class creates and writes STEP files from Open CASCADE models. A STEP file can be written to an existing
/// STEP file or to a new one. Translation can be performed in one or several operations. Each translation operation
/// outputs a distinct root entity in the STEP file.
/// </summary>
/// <remarks>
/// <para>A C++ value class, a C# class here (decision 1185). OCCT.NET (plan 2.7, decisions 13201, 13202, 13204):</para>
/// <list type="bullet">
/// <item>the writer never changes the shape it is given;</item>
/// <item>the parameters come from the session's parameter set (<see cref="IFSelect_WorkSession.Statics"/>), or from
/// a <see cref="DESTEP_Parameters"/> given to the transfer;</item>
/// <item>the session gets a STEP controller of its own (not the one recorded process-wide), so that the write
/// actor's settings (<see cref="SetTolerance"/>, the shape fix parameters, the group mode) and its counters belong to
/// this writer's session;</item>
/// <item>the header's FILE_NAME (time stamp, author...) is set at the first transfer from the system (the local
/// time); for repeatable output, change it before writing through <c>new APIHeaderSection_MakeHeader(Model())</c>
/// (<c>SetTimeStamp</c>, <c>SetAuthorValue</c>...), which <see cref="Write"/> and <see cref="WriteStream"/> keep.
/// AP203's dates and persons come from the write actor's context (<see cref="STEPControl_ActorWrite.ContextTool"/>).</item>
/// </list>
/// </remarks>
public class STEPControl_Writer
{
    /// <summary>Creates a Writer from scratch</summary>
    public STEPControl_Writer()
    {
        STEPControl_Controller.Init();
        SetWS(new XSControl_WorkSession());
    }

    /// <summary>Creates a Writer from an already existing Session If &lt;scratch&gt; is True (D), clears already recorded data</summary>
    public STEPControl_Writer(XSControl_WorkSession WS, bool scratch = true)
    {
        STEPControl_Controller.Init();
        SetWS(WS, scratch);
    }

    /// <summary>
    /// Sets a length-measure value that will be written to uncertainty-measure-with-unit when the next shape is
    /// translated.
    /// </summary>
    public void SetTolerance(double Tol)
    {
        STEPControl_ActorWrite act = WS().NormAdaptor().ActorWrite() as STEPControl_ActorWrite;
        if (act is not null)
        {
            act.SetTolerance(Tol);
        }
    }

    /// <summary>Unsets the tolerance formerly forced by SetTolerance</summary>
    public void UnsetTolerance()
    {
        SetTolerance(-1.0);
    }

    /// <summary>Sets a specific session to &lt;me&gt;</summary>
    public void SetWS(XSControl_WorkSession WS, bool scratch = true)
    {
        thesession = WS;
        // OCCT: SelectNorm("STEP"), the controller recorded process-wide; here one of the session's own (decision 13201)
        STEPControl_Controller.SelectFor(thesession);
        thesession.InitTransferReader(0);
        Model(scratch);
    }

    /// <summary>Returns the session used in &lt;me&gt;</summary>
    public XSControl_WorkSession WS()
    {
        return thesession;
    }

    /// <summary>
    /// Returns the produced model. Produces a new one if not yet done or if &lt;newone&gt; is True This method allows
    /// for instance to edit product or header data before writing.
    /// </summary>
    public StepData_StepModel Model(bool newone = false)
    {
        StepData_StepModel model = thesession.Model() as StepData_StepModel;
        if (newone || model is null)
        {
            model = thesession.NewModel() as StepData_StepModel;
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
    public IFSelect_ReturnStatus Transfer(TopoDS_Shape sh, STEPControl_StepModelType mode, bool compgraph = true,
                                          Message_ProgressRange theProgress = default)
    {
        StepData_StepModel aStepModel = thesession.Model() as StepData_StepModel;
        if (aStepModel is not null)
        {
            // OCCT: from the process-wide statics; here the session's (decision 1181)
            aStepModel.InternalParameters.InitFromStatic(thesession.Statics());
        }
        return Transfer(sh, mode, aStepModel.InternalParameters, compgraph, theProgress);
    }

    /// <summary>
    /// Translates shape sh to a STEP entity, with the given parameters (which the model takes).
    /// </summary>
    public IFSelect_ReturnStatus Transfer(TopoDS_Shape sh, STEPControl_StepModelType mode, in DESTEP_Parameters theParams,
                                          bool compgraph = true, Message_ProgressRange theProgress = default)
    {
        int mws = -1;
        switch (mode)
        {
            case STEPControl_AsIs:
                mws = 0;
                break;
            case STEPControl_FacetedBrep:
                mws = 1;
                break;
            case STEPControl_ShellBasedSurfaceModel:
                mws = 2;
                break;
            case STEPControl_ManifoldSolidBrep:
                mws = 3;
                break;
            case STEPControl_GeometricCurveSet:
                mws = 4;
                break;
            default:
                break;
        }
        if (mws < 0)
        {
            return IFSelect_RetError; // cas non reconnu
        }
        thesession.TransferWriter().SetTransferMode(mws);
        if (!Model().IsInitializedUnit())
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; here the session's
            // xstep.cascade.unit (decision 13202)
            Model().SetLocalLengthUnit(UnitsMethods.GetCasCadeLengthUnit(thesession.Statics()));
        }
        Model().InternalParameters = theParams;
        APIHeaderSection_MakeHeader aHeaderMaker = new APIHeaderSection_MakeHeader();
        aHeaderMaker.Apply(Model());
        STEPControl_ActorWrite ActWrite = WS().NormAdaptor().ActorWrite() as STEPControl_ActorWrite;
        ActWrite.SetGroupMode((int)(thesession.Model() as StepData_StepModel).InternalParameters.WriteAssembly);
        InitializeMissingParameters();
        return thesession.TransferWriteShape(sh, compgraph, theProgress);
    }

    /// <summary>Writes a STEP model in the file identified by filename.</summary>
    public IFSelect_ReturnStatus Write(string theFileName)
    {
        StepData_StepModel aModel = Model();
        if (aModel is null)
        {
            return IFSelect_RetFail;
        }
        APIHeaderSection_MakeHeader aHeaderMaker = new APIHeaderSection_MakeHeader();
        aHeaderMaker.Apply(aModel);
        return thesession.SendAll(theFileName);
    }

    /// <summary>Writes a STEP model in the std::ostream.</summary>
    /// <remarks>OCCT.NET: a .NET stream; the model's text is written as bytes (decision 1192).</remarks>
    public IFSelect_ReturnStatus WriteStream(Stream theOStream)
    {
        StepData_StepModel aModel = Model();
        if (aModel is null)
        {
            return IFSelect_RetFail;
        }

        StepData_Protocol aProtocol = aModel.Protocol() as StepData_Protocol;
        if (aProtocol is null)
        {
            return IFSelect_RetFail;
        }

        StepData_StepWriter aWriter = new StepData_StepWriter(aModel);
        aWriter.SendModel(aProtocol);
        APIHeaderSection_MakeHeader aHeaderMaker = new APIHeaderSection_MakeHeader();
        aHeaderMaker.Apply(aModel);
        return aWriter.Print(theOStream) ? IFSelect_RetDone : IFSelect_RetFail;
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
        thesession.TransferWriter().PrintStats(what, mode);
    }

    // CleanDuplicateEntities (StepTidy_DuplicateCleaner, write.step.cleanduplicates) is not ported (decision 13208)

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(NCollection_DataMap<string, string> theParameters)
    {
        Transfer_ActorOfFinderProcess anActor = GetActor();
        if (anActor is not null)
        {
            anActor.SetShapeFixParameters(theParameters);
        }
    }

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(in DE_ShapeFixParameters theParameters)
    {
        NCollection_DataMap<string, string> anAdditionalParameters = new NCollection_DataMap<string, string>();
        Transfer_ActorOfFinderProcess anActor = GetActor();
        if (anActor is not null)
        {
            anActor.SetShapeFixParameters(theParameters, anAdditionalParameters);
        }
    }

    /// <summary>
    /// Sets parameters for shape processing. Parameters from theParameters are copied to the internal map. Parameters
    /// from theAdditionalParameters are copied to the internal map if they are not present in theParameters.
    /// </summary>
    public void SetShapeFixParameters(in DE_ShapeFixParameters theParameters, NCollection_DataMap<string, string> theAdditionalParameters)
    {
        Transfer_ActorOfFinderProcess anActor = GetActor();
        if (anActor is not null)
        {
            anActor.SetShapeFixParameters(theParameters, theAdditionalParameters);
        }
    }

    /// <summary>Returns parameters for shape processing that was set by SetParameters() method.</summary>
    public NCollection_DataMap<string, string> GetShapeFixParameters()
    {
        Transfer_ActorOfFinderProcess anActor = GetActor();
        return anActor is null ? new NCollection_DataMap<string, string>() : anActor.GetShapeFixParameters();
    }

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetShapeProcessFlags(ShapeProcess.OperationsFlags theFlags)
    {
        Transfer_ActorOfFinderProcess anActor = GetActor();
        if (anActor is not null)
        {
            anActor.SetShapeProcessFlags(theFlags);
        }
    }

    /// <summary>Returns flags defining operations to be performed on shapes.</summary>
    public (ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags()
    {
        Transfer_ActorOfFinderProcess anActor = GetActor();
        return anActor is null ? (new ShapeProcess.OperationsFlags(), false) : anActor.GetShapeProcessFlags();
    }

    /// <summary>Returns the Actor for the Transfer of an Entity. @return the Actor for the Transfer of an Entity. May be nullptr.</summary>
    private Transfer_ActorOfFinderProcess GetActor()
    {
        XSControl_WorkSession aSession = WS();
        if (aSession is null)
        {
            return null;
        }

        XSControl_Controller aController = aSession.NormAdaptor();
        if (aController is null)
        {
            return null;
        }

        return aController.ActorWrite();
    }

    /// <summary>
    /// If parameters haven't yet been provided, initializes them with default values provided by
    /// GetDefaultShapeFixParameters() method.
    /// </summary>
    private void InitializeMissingParameters()
    {
        if (GetShapeFixParameters().IsEmpty())
        {
            SetShapeFixParameters(DESTEP_Parameters.GetDefaultShapeFixParameters());
        }
        if (!GetShapeProcessFlags().Item2)
        {
            ShapeProcess.OperationsFlags aFlags = new ShapeProcess.OperationsFlags(ShapeProcess.Operation.SplitCommonVertex,
                                                                                  ShapeProcess.Operation.DirectFaces);
            SetShapeProcessFlags(aFlags);
        }
    }

    private XSControl_WorkSession thesession;
}
