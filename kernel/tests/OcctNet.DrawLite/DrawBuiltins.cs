// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;

namespace OcctNet.DrawLite;

/// <summary>The Tcl commands, and the Draw commands that need no geometry.</summary>
internal static class DrawBuiltins
{
    /// <summary>
    /// Draw commands that only drive OCCT's viewers, plugins, logging or resource limits. The port has no viewer,
    /// so these do nothing and a script that uses them still runs. OCCT's CheckCommands.tcl defines checkview as
    /// a procedure that drives the viewers; Draw-lite keeps it a no-op. vglinfo only reports the OpenGL driver. The
    /// commands that mesh the shapes they display (vdisplay, vsetdispmode, ...) are ViewerTestCommands.
    /// </summary>
    internal static readonly string[] Ignored =
    [
        "pload", "cpulimit", "checkview", "vfit", "vdump", "vglinfo",
        "vmoveto", "vselect", "vtop", "vaxo", "vzoom", "smallview", "fit", "erase",
        "donly", "display", "clear", "xwd", "decho", "vrenderparams", "vcamera",
        "dsetsignal", "dtracelevel", "dbreak", "wait", "pu", "pd", "pl", "pr",
    ];

    public static void Register(DrawInterpreter theInterpreter)
    {
        TclControlCommands.Register(theInterpreter);
        TclStringCommands.Register(theInterpreter);
        TclListCommands.Register(theInterpreter);
        TclDataCommands.Register(theInterpreter);
        TclSystemCommands.Register(theInterpreter);
        RegisterIgnored(theInterpreter);
        DrawVariableCommands.Register(theInterpreter);
        DBRepCommands.Register(theInterpreter);
        BRepTestCommands.Register(theInterpreter);
        BRepTestCheckCommands.Register(theInterpreter);
        BRepTestBoundingCommands.Register(theInterpreter);
        BOPTestClassifyCommands.Register(theInterpreter);
        BRepTestPrimitiveCommands.Register(theInterpreter);
        BRepTestSweepCommands.Register(theInterpreter);
        DrawChronoCommands.Register(theInterpreter);
        GeomliteTestCommands.Register(theInterpreter);
        BRepTestExtremaCommands.Register(theInterpreter);
        BRepTestCurveCommands.Register(theInterpreter);
        BRepTestSurfaceCommands.Register(theInterpreter);
        BRepTestModifyCommands.Register(theInterpreter);
        GeometryTestCommands.Register(theInterpreter);
        GeometryTestIntersectCommands.Register(theInterpreter);
        GeometryTestBisecCommands.Register(theInterpreter);
        GeometryTestProjCommands.Register(theInterpreter);
        SWDRAWShapeFixCommands.Register(theInterpreter);
        DrawTrSurfCommands.Register(theInterpreter);
        MeshTestCommands.Register(theInterpreter);
        SWDrawShapeUpgradeCommands.Register(theInterpreter);
        ViewerTestCommands.Register(theInterpreter);
        DrawSourceFiles.Register(theInterpreter);

        // dputs ?-color? ?-intense? ?-nonewline? text: puts with colors, which Draw-lite drops
        theInterpreter.Register("dputs", static (i, a) =>
        {
            bool aNoNewline = a.Contains("-nonewline");
            string aText = a.Count > 1 ? a[^1] : "";
            if (aNoNewline)
            {
                i.Output.Write(aText);
            }
            else
            {
                i.Output.WriteLine(aText);
            }

            return "";
        });
        // dgetenv name: the variable's value, empty when it isn't set (Draw_VariableCommands.cxx: getenv)
        theInterpreter.Register("dgetenv", static (i, a) =>
        {
            if (a.Count < 2)
            {
                i.Output.WriteLine($"Use: {a[0]} {{varname}}");
                throw new DrawException("");
            }

            return i.TryGetVariable("::env", a[1], out string aValue) ? aValue : "";
        });
        theInterpreter.Register("dsetenv", static (i, a) =>
        {
            if (a.Count == 3)
            {
                i.SetVariable("::env", a[1], a[2]);
            }
            else if (a.Count == 2)
            {
                i.UnsetVariable($"::env({a[1]})");
            }

            return "";
        });

        SetVariables(theInterpreter);
    }

    internal static void RegisterIgnored(DrawInterpreter theInterpreter)
    {
        foreach (string aName in Ignored)
        {
            theInterpreter.Register(aName, static (_, _) => "");
        }
    }

    private static void SetVariables(DrawInterpreter theInterpreter)
    {
        foreach (DictionaryEntry anEntry in Environment.GetEnvironmentVariables())
        {
            theInterpreter.SetVariable("env", (string)anEntry.Key, (string?)anEntry.Value ?? "");
        }

        foreach ((string aKey, string aValue) in TclSystemCommands.Platform())
        {
            theInterpreter.SetVariable("tcl_platform", aKey, aValue);
        }

        theInterpreter.SetVariable("tcl_version", null, "8.6");
        theInterpreter.SetVariable("tcl_patchLevel", null, "8.6.13");
        theInterpreter.SetVariable("tcl_interactive", null, "0");
        theInterpreter.SetVariable("argv0", null, "DRAWEXE");
        theInterpreter.SetVariable("argv", null, "");
        theInterpreter.SetVariable("argc", null, "0");
        theInterpreter.SetVariable("auto_path", null, "");
    }
}
