// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DrawLite;

/// <summary>Draw's command log (Draw_Interpretor: SetDoLog, ResetLog, GetLog, AddLog).</summary>
public sealed partial class DrawInterpreter
{
    private int myLogStart;

    /// <summary>
    /// Draw_Interpretor's logging, on while OCCT's harness runs a case (<c>dlog on</c> in _run_test): CommandCmd then
    /// writes each C++ command's words and, after it ran, its result to the log (<see cref="Output"/>), so the log has
    /// what DRAWEXE's has. Off by default, as in Draw.
    /// </summary>
    public bool DoLog { get; set; }

    /// <summary>Draw_Interpretor::ResetLog: the log starts here (when the output keeps its text).</summary>
    internal void ResetLog() => myLogStart = Output is StringWriter aWriter ? aWriter.GetStringBuilder().Length : 0;

    /// <summary>Draw_Interpretor::GetLog: the output since the last reset, when the output keeps its text.</summary>
    internal string GetLog()
    {
        if (Output is not StringWriter aWriter)
        {
            return "";
        }

        System.Text.StringBuilder aText = aWriter.GetStringBuilder();
        int aStart = Math.Min(myLogStart, aText.Length);
        return aText.ToString(aStart, aText.Length - aStart);
    }
}
