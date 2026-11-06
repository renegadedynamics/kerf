// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Controller.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Controller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;

namespace Kerf.Engine.Exchange.Step.Documents;

/// <summary>
/// Extends Controller from STEPControl in order to provide ActorWrite adapted for writing assemblies from DECAF Note
/// that ActorRead from STEPControl is used for reading (inherited automatically)
/// </summary>
/// <remarks>
/// <para>
/// Kerf: as STEPControl_Controller (decision 1264), <see cref="Init"/> records a controller under "STEP" and
/// "step" once per process (it replaces STEPControl's there, as in OCCT), and STEPCAFControl_Reader and
/// STEPCAFControl_Writer give their sessions a controller of their own (<see cref="SelectFor"/>). The statics
/// <c>write.stepcaf.subshapes.name</c> and <c>read.stepcaf.subshapes.name</c>, which OCCT's <c>Init</c> declares
/// process-wide, are declared per parameter set by <see cref="InitStatics"/>.
/// </para>
/// <para>
/// As in OCCT, the write actor is a fresh <see cref="CafActorWrite"/>: the shape fix parameters and the
/// processing flags STEPControl_Controller's constructor gives its own actor are not set on it.
/// </para>
/// </remarks>
internal class CafController : StepController
{
    // The controller recorded for STEP-XCAF, made once (a one-time initialisation, unit 10I)
    private static readonly Lazy<CafController> TheRecorded = new(() =>
    {
        // self-registering
        CafController stepctl = new CafController();
        // OCCT's XSAlgo::Init() and the statics: per parameter set (InitStatics)
        stepctl.AutoRecord();
        return stepctl;
    });

    /// <summary>Initializes the use of STEP Norm (the first time)</summary>
    public CafController()
    {
        CafActorWrite actWrite = new CafActorWrite();
        _adaptorWrite = actWrite;
    }

    /// <summary>
    /// Declares STEP's statics and STEPCAFControl's (OCCT's <c>Init</c>'s <c>Interface_Static::Init</c> calls) in a
    /// parameter set, once per set (Kerf, decision 1181).
    /// </summary>
    public override void InitStatics(StaticSet statics)
    {
        base.InitStatics(statics);
        if (!statics.StartInit("STEPCAFControl_Controller"))
        {
            return;
        }

        //-----------------------------------------------------------
        // Few variables for advanced control of translation process
        //-----------------------------------------------------------

        // Indicates whether to write sub-shape names to 'Name' attributes of
        // STEP Representation Items
        statics.Init("stepcaf", "write.stepcaf.subshapes.name", 'e', "");
        statics.Init("stepcaf", "write.stepcaf.subshapes.name", '&', "enum 0");
        statics.Init("stepcaf", "write.stepcaf.subshapes.name", '&', "eval Off"); // 0
        statics.Init("stepcaf", "write.stepcaf.subshapes.name", '&', "eval On");  // 1
        statics.SetIVal("write.stepcaf.subshapes.name", 0);                      // Disabled by default

        // Indicates whether to read sub-shape names from 'Name' attributes of
        // STEP Representation Items
        statics.Init("stepcaf", "read.stepcaf.subshapes.name", 'e', "");
        statics.Init("stepcaf", "read.stepcaf.subshapes.name", '&', "enum 0");
        statics.Init("stepcaf", "read.stepcaf.subshapes.name", '&', "eval Off"); // 0
        statics.Init("stepcaf", "read.stepcaf.subshapes.name", '&', "eval On");  // 1
        statics.SetIVal("read.stepcaf.subshapes.name", 0);                      // Disabled by default
    }

    /// <summary>
    /// Standard Initialisation. It creates a Controller for STEP-XCAF and records it to various names, available to
    /// select it later Returns True when done, False if could not be done
    /// </summary>
    /// <remarks>Kerf: once per process, thread-safe (decision 1264).</remarks>
    public new static bool Init()
    {
        StepController.Init();
        _ = TheRecorded.Value;
        return true;
    }

    /// <summary>
    /// Selects STEP in a session as STEPCAFControl_Reader and STEPCAFControl_Writer do (OCCT: <c>SelectNorm("STEP")</c>,
    /// the controller recorded process-wide, which <see cref="Init"/> made a STEPCAFControl_Controller): a session
    /// already working with a STEPCAFControl_Controller keeps it; another gets a controller of its own (Kerf,
    /// decision 1264). As SelectNorm, it clears the session's transfer reader first.
    /// </summary>
    public new static bool SelectFor(ExchangeWorkSession session)
    {
        Init();
        session.TransferReader()?.Clear(-1);
        if (session.NormAdaptor() is not CafController)
        {
            session.SetController(new CafController());
        }
        return true;
    }
}
