// Port of: src/Draw/TKTopTest/SWDRAW/SWDRAW_ShapeProcess.cxx
// Port of: src/Draw/TKTopTest/SWDRAW/SWDRAW_ShapeProcessAPI.cxx
// Port of: src/Draw/TKDraw/Draw/Draw_Printer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The commands of SWDRAW_ShapeProcess.cxx and SWDRAW_ShapeProcessAPI.cxx (unit 7C): <c>SPApply</c> (a
/// ShapeProcess_ShapeContext and ShapeProcess::Perform) and <c>DT_ApplySeq</c> (ShapeProcessAPI_ApplySequence).
/// </summary>
/// <remarks>
/// <para>
/// The resource file: OCCT's Resource_Manager finds the file <c>name</c> in the directories the environment variables
/// <c>CSF_nameDefaults</c> and <c>CSF_nameUserDefaults</c> name, which the corpus's scripts set with
/// <c>set env(...)</c>. The port's library reads no environment variable (decision 1023), so the commands read them
/// from the interpreter's <c>env</c> array and pass the directories; without them the library loads OCCT's own file of
/// that name, compiled in (<c>IGES</c>, <c>STEP</c>), or nothing.
/// </para>
/// <para>
/// The messages ShapeProcess sends (the sequence's errors, PrintStatistics and PrintPreparationResult) go to
/// DRAWEXE's standard output, which the harness logs: Draw-lite gives the context a messenger writing to the
/// interpreter's output (OCCT's Draw_Printer, at Message_PrinterOStream's default trace level, Info), rather than
/// changing the process-wide default messenger.
/// </para>
/// <para>
/// OCCT's operators write into the shape they process (tolerances, pcurves, the modifiers' vertices), and in DRAWEXE
/// every variable holding the input's sub-shapes sees it. The library never writes into a caller's shape (decision
/// 8702), so Draw-lite runs the sequence as decision 960's in-place commands: on an unfrozen copy of the input
/// (<see cref="SWDRAWShapeFixCommands.InPlace"/>, reading the unfrozen results), where ShapeProcess works in place as
/// OCCT's does, then rebinds every variable sharing a changed sub-shape; the copy's unchanged sub-shapes are the
/// input's own again, so OCCT's test <c>result == Shape</c> holds as in DRAWEXE.
/// </para>
/// </remarks>
internal static class SWDRAWShapeProcessCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        global::Kerf.Engine.Healing.Process.OperLibrary.Init();

        DrawCxx.Add(theInterpreter, "SPApply", ApplySequence);
        DrawCxx.Add(theInterpreter, "DT_ApplySeq", ApplySequenceAPI);
    }

    // OCCT's Draw_Printer: messages written to the interpreter (here its output, as DRAWEXE's standard output)
    private sealed class DrawPrinter : Printer
    {
        private readonly TextWriter myOutput;

        public DrawPrinter(TextWriter theOutput)
        {
            myOutput = theOutput;
            _traceLevel = Gravity.Info;
        }

        protected override void SendCore(string theString, Gravity theGravity)
        {
            if (theGravity < _traceLevel)
            {
                return;
            }
            myOutput.Write(theString + "\n");
        }
    }

    private static Messenger Messenger(DrawInterpreter theDI) => new(new DrawPrinter(theDI.Output));

    // The directories of the resource file theName: the interpreter's env(CSF_<name>Defaults) and
    // env(CSF_<name>UserDefaults), null where unset
    private static (string?, string?) ResourceDirectories(DrawInterpreter theDI, string theName)
    {
        string? aDefaults = theDI.TryGetVariable("::env", "CSF_" + theName + "Defaults", out string aValue) && aValue.Length != 0
            ? aValue : null;
        string? aUser = theDI.TryGetVariable("::env", "CSF_" + theName + "UserDefaults", out string aUserValue) && aUserValue.Length != 0
            ? aUserValue : null;
        return (aDefaults, aUser);
    }

    // SPApply result shape rscfilename [sequence]
    private static int ApplySequence(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            di.Append("Specify result, shape, resource name and prefix (optional)\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        string arg2 = argv[2];
        string arg3 = argv[3];
        string arg4 = "";
        if (argc > 4)
        {
            arg4 = argv[4];
        }
        Shape Shape = theDI.GetShape(arg2);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg2).Append("\n");
            return 1 /* Error */;
        }

        (string? aDefaults, string? aUser) = ResourceDirectories(theDI, arg3);
        global::Kerf.Engine.Healing.Process.ShapeContext? context = null;
        // OCCT works on the input itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape result = SWDRAWShapeFixCommands.InPlace(theDI, Shape, aWork =>
        {
            context = new global::Kerf.Engine.Healing.Process.ShapeContext(aWork, arg3, "", aDefaults, aUser);
            context.SetMessenger(Messenger(theDI));
            global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(context, arg4);
            return context.InternalResult();
        });
        context!.PrintStatistics();

        //  seq.PrintPreparationResult ();
        if (result.IsNull)
        {
            di.Append("No result\n");
        }
        else if (result == Shape)
        {
            di.Append("Shape not modified\n");
        }
        theDI.SetShape(arg1, result);
        return 0;
    }

    // DT_ApplySeq result shape rscfilename [prefix]
    private static int ApplySequenceAPI(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            di.Append("Specify result, shape, resource name and prefix (optional)\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        string arg2 = argv[2];
        string arg3 = argv[3];
        string arg4 = "";
        if (argc > 4)
        {
            arg4 = argv[4];
        }
        Shape Shape = theDI.GetShape(arg2);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg2).Append("\n");
            return 1 /* Error */;
        }

        (string? aDefaults, string? aUser) = ResourceDirectories(theDI, arg3);
        var seq = new global::Kerf.Engine.Healing.Process.ApplySequence(arg3, arg4, aDefaults, aUser);
        seq.Context().SetMessenger(Messenger(theDI));
        // OCCT works on the input itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        //   seq.PrepareShape(Shape, true, TopAbs_FACE); //fill history map for faces and above
        Shape result = SWDRAWShapeFixCommands.InPlace(theDI, Shape, aWork => seq.InternalPrepareShape(aWork));
        seq.PrintPreparationResult();
        theDI.SetShape(arg1, result);
        return 0;
    }
}
