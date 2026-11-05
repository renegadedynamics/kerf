// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>Helpers: run a script in a fresh interpreter and look at its result, output or error.</summary>
internal static class Tcl
{
    public static string Eval(string theScript) => new DrawInterpreter().Eval(theScript);

    public static string Output(string theScript)
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        new DrawInterpreter(anOutput).Eval(theScript);
        return anOutput.ToString();
    }

    public static string Error(string theScript) => Assert.ThrowsAny<DrawException>(() => new DrawInterpreter().Eval(theScript)).Reason;
}
