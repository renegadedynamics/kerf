// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DrawLite;

/// <summary>The Tcl and Draw commands that need no geometry.</summary>
internal static class DrawBuiltins
{
    /// <summary>
    /// Draw commands that only drive OCCT's viewers, plugins or resource limits. The port has no viewer, so these
    /// do nothing and a script that uses them still runs.
    /// </summary>
    internal static readonly string[] Ignored =
    [
        "pload", "cpulimit", "dset", "protect", "checkview", "vinit", "vdisplay", "vfit", "vdump", "vclear",
        "vsetdispmode", "vmoveto", "vselect", "vclose", "vtop", "vaxo", "vzoom", "smallview", "fit", "erase",
        "donly", "display", "clear", "xwd", "decho", "dlog", "dchrono", "chrono", "vrenderparams", "vcamera",
    ];

    public static void Register(DrawInterpreter theInterpreter)
    {
        foreach (string aName in Ignored)
        {
            theInterpreter.Register(aName, static (_, _) => "");
        }

        theInterpreter.Register("set", Set);
        theInterpreter.Register("unset", Unset);
        theInterpreter.Register("puts", Puts);
        theInterpreter.Register("info", Info);
    }

    private static string Set(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        switch (theArgs.Count)
        {
            case 2:
                return theInterpreter.TryGetVariable(theArgs[1], out string aValue)
                           ? aValue
                           : throw new DrawException($"can't read \"{theArgs[1]}\": no such variable");
            case 3:
                theInterpreter.SetVariable(theArgs[1], theArgs[2]);
                return theArgs[2];
            default:
                throw new DrawException("wrong # args: should be \"set varName ?newValue?\"");
        }
    }

    private static string Unset(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        foreach (string aName in theArgs.Skip(1).Where(a => a != "-nocomplain"))
        {
            theInterpreter.UnsetVariable(aName);
        }

        return "";
    }

    private static string Puts(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        // puts ?-nonewline? ?channel? string
        bool aNoNewline = theArgs.Count > 1 && theArgs[1] == "-nonewline";
        string aText = theArgs[^1];
        if (aNoNewline)
        {
            theInterpreter.Output.Write(aText);
        }
        else
        {
            theInterpreter.Output.WriteLine(aText);
        }

        return "";
    }

    private static string Info(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count == 3 && theArgs[1] == "exists")
        {
            return theInterpreter.TryGetVariable(theArgs[2], out _) ? "1" : "0";
        }

        if (theArgs.Count == 3 && theArgs[1] == "commands")
        {
            return theInterpreter.HasCommand(theArgs[2]) ? theArgs[2] : "";
        }

        throw new DrawException($"info {string.Join(' ', theArgs.Skip(1))}: not supported");
    }
}
