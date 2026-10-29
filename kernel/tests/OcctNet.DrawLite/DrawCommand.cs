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
            var aResult = new DrawResult();
            int aStatus;
            try
            {
                aStatus = theCommand(i, aResult, a);
            }
            catch (Standard_Failure aFailure)
            {
                throw new DrawException("An exception was caught " + Print(aFailure));
            }

            return aStatus == 0 ? aResult.ToString() : throw new DrawException(aResult.ToString());
        });

    /// <summary>Standard_Failure::Print: the type, then ": " and the message when there is one.</summary>
    public static string Print(Standard_Failure theFailure)
    {
        string aMessage = theFailure.GetMessageString();
        bool aHasMessage = aMessage.Length > 0 && !aMessage.StartsWith("Exception of type '", StringComparison.Ordinal);
        return aHasMessage ? $"{theFailure.OcctType}: {aMessage}" : theFailure.OcctType;
    }
}
