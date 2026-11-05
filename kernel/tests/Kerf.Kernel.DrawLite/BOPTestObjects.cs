// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Topology;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BOPTest_Objects.cxx: the Boolean session of an interpreter (the objects and tools, the pave filler of
/// <c>bfillds</c>, the current builder, the options) and BOPTest::ReportAlerts.
/// </summary>
/// <remarks>
/// OCCT's session is process-wide; Draw-lite keeps one per interpreter, so corpus cases can run in parallel. The class
/// is partial: the units that port a builder Draw keeps in the session add it in their own file (the cells builder,
/// unit 6G; BOP, Section and Splitter, unit 6L). The allocators go.
/// </remarks>
internal sealed partial class BOPTestObjects
{
    private static readonly ConditionalWeakTable<DrawInterpreter, BOPTestObjects> theSessions = new();

    /// <summary>The session of the interpreter (GetSession()).</summary>
    public static BOPTestObjects Of(DrawInterpreter theDI) => theSessions.GetValue(theDI, static _ => new BOPTestObjects());

    private BOPTestObjects()
    {
        Init();
    }

    private PaveFiller myPaveFiller = null!;
    private BooleanBuilder myBuilder = null!;
    private BooleanBuilder myBuilderDefault = null!;

    /// <summary>The objects (baddobjects).</summary>
    public NCollection_List<Shape> Shapes { get; } = new();

    /// <summary>The tools (baddtools).</summary>
    public NCollection_List<Shape> Tools { get; } = new();

    public bool RunParallel { get; set; }

    public bool NonDestructive { get; set; }

    public double FuzzyValue { get; set; }

    public global::Kerf.Booleans.GlueMode Glue { get; set; }

    public bool DrawWarnShapes { get; set; }

    public bool CheckInverted { get; set; }

    public bool UseOBB { get; set; }

    public bool UnifyEdges { get; set; }

    public bool UnifyFaces { get; set; }

    public double Angular { get; set; }

    /// <summary>A new pave filler and default builder (Init).</summary>
    public void Init()
    {
        myPaveFiller = new PaveFiller();
        myBuilderDefault = new BooleanBuilder();
        myBuilder = myBuilderDefault;
        SetDefaultOptions();
    }

    /// <summary><c>bclear</c>: a new session's filler and builder, no objects and tools.</summary>
    public void Clear()
    {
        // OCCT deletes the filler and the default builder, and GetSession() makes new ones
        Init();
        Shapes.Clear();
        Tools.Clear();
    }

    /// <summary>The pave filler of <c>bfillds</c>.</summary>
    public PaveFiller PaveFiller() => myPaveFiller;

    /// <summary>The data structure of <c>bfillds</c>.</summary>
    public DS? PDS() => myPaveFiller.PDS();

    /// <summary>The current builder (the default General Fuse, or the one a command set).</summary>
    public BooleanBuilder Builder() => myBuilder;

    public void SetBuilder(BooleanBuilder theBuilder) => myBuilder = theBuilder;

    public void SetBuilderDefault() => myBuilder = myBuilderDefault;

    /// <summary>Resets all options to their default values.</summary>
    public void SetDefaultOptions()
    {
        RunParallel = false;
        NonDestructive = false;
        FuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        Glue = global::Kerf.Booleans.GlueMode.Off;
        DrawWarnShapes = false;
        CheckInverted = true;
        UseOBB = false;
        UnifyEdges = false;
        UnifyFaces = false;
        Angular = global::Kerf.Engine.Foundation.Precision.Angular();
    }

    /// <summary>
    /// BOPTest::ReportAlerts: the warnings, then the errors, one line per alert type ("Warning: " or "Error: " and the
    /// message text), with the shapes of the alerts of that type saved as <c>ws_i_j</c>/<c>es_i_j</c> when
    /// <c>bdrawwarnshapes</c> is on.
    /// </summary>
    public static void ReportAlerts(DrawInterpreter theDI, DrawResult di, global::Kerf.Engine.Foundation.Report theReport)
    {
        // first report warnings, then errors
        global::Kerf.Engine.Foundation.Gravity[] anAlertTypes = [global::Kerf.Engine.Foundation.Gravity.Warning, global::Kerf.Engine.Foundation.Gravity.Fail];
        string[] aMsgType = ["Warning: ", "Error: "];
        for (int iGravity = 0; iGravity < 2; iGravity++)
        {
            // report shapes for the same type of alert together
            HashSet<Type> aPassedTypes = [];
            List<global::Kerf.Engine.Foundation.Alert> aList = [.. theReport.GetAlerts(anAlertTypes[iGravity])];
            for (int aIt = 0; aIt < aList.Count; ++aIt)
            {
                // check that this type of warnings has not yet been processed
                Type aType = aList[aIt].GetType();
                if (!aPassedTypes.Add(aType))
                {
                    continue;
                }

                // get alert message
                global::Kerf.Engine.Foundation.Msg aMsg = new(aList[aIt].GetMessageKey());
                string aText = aMsgType[iGravity] + aMsg.Get();

                // collect all shapes if any attached to this alert
                if (Of(theDI).DrawWarnShapes)
                {
                    string aShapeList = "";
                    int aNbShapes = 0;
                    for (int aIt2 = aIt; aIt2 < aList.Count; ++aIt2)
                    {
                        if (aList[aIt2] is AlertWithShape aShapeAlert && aType == aShapeAlert.GetType()
                            && !aShapeAlert.GetShape().IsNull)
                        {
                            string aName = $"{(iGravity != 0 ? "e" : "w")}s_{aPassedTypes.Count}_{++aNbShapes}";
                            theDI.SetShape(aName, aShapeAlert.GetShape());
                            aShapeList += " " + aName;
                        }
                    }
                    aText += aNbShapes != 0 ? ": " : "(no shapes attached)";
                    aText += aShapeList;
                }

                // output message with list of shapes
                di.Append(aText).Append("\n");
            }
        }
    }
}
