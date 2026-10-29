// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// A Draw command written as OCCT writes it: <c>int f(Draw_Interpretor&amp; di, int n, const char** a)</c>, with
/// <c>di</c> the result being built and <c>a</c> the words (<c>a[0]</c> the command's name).
/// </summary>
public delegate int DrawCxxCommand(DrawInterpreter theInterpreter, DrawResult di, IReadOnlyList<string> a);

/// <summary>Registers Draw's C++ commands, run as Draw_Interpretor's CommandCmd runs them.</summary>
internal static class DrawCxx
{
    /// <summary>
    /// Registers a command: a non-zero return is a Tcl error whose message is the result built so far; an OCCT
    /// exception becomes "An exception was caught Type: message", as CommandCmd reports it.
    /// </summary>
    public static void Add(DrawInterpreter theInterpreter, string theName, DrawCxxCommand theCommand)
        => theInterpreter.Register(theName, (i, a) =>
        {
            // CommandCmd logs the command's words and then its result, except for dlog and decho (decision 3551)
            bool aDoLog = i.DoLog && theName is not ("dlog" or "decho");
            if (aDoLog)
            {
                i.Output.Write(string.Concat(a.Select(w => w + " ")) + "\n");
            }

            var aResult = new DrawResult();
            int aStatus;
            string aText;
            try
            {
                aStatus = theCommand(i, aResult, a);
                aText = aResult.ToString();
            }
            catch (Standard_Failure aFailure)
            {
                aStatus = 1;
                aText = "An exception was caught " + Print(aFailure);
            }

            if (aDoLog && aText.Length > 0)
            {
                i.Output.Write(aText + "\n");
            }

            return aStatus == 0 ? aText : throw new DrawException(aText);
        });

    /// <summary>Standard_Failure::Print: the type, then ": " and the message when there is one.</summary>
    public static string Print(Standard_Failure theFailure)
    {
        string aMessage = theFailure.GetMessageString();
        bool aHasMessage = aMessage.Length > 0 && !aMessage.StartsWith("Exception of type '", StringComparison.Ordinal);
        return aHasMessage ? $"{theFailure.OcctType}: {aMessage}" : theFailure.OcctType;
    }
}
