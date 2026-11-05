// Port of: src/FoundationClasses/TKernel/Message/Message_ConsoleColor.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Foundation;

/// <summary>The colours of console text, for <see cref="PrinterOStream"/>.</summary>
internal enum MessageConsoleColor
{
    /// <summary>The default (white) colour.</summary>
    ConsoleColor_Default,
    ConsoleColor_Black,
    ConsoleColor_White,
    ConsoleColor_Red,
    ConsoleColor_Blue,
    ConsoleColor_Green,
    ConsoleColor_Yellow,
    ConsoleColor_Cyan,
    ConsoleColor_Magenta,
}
