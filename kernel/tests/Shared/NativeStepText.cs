// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text.RegularExpressions;

namespace Kerf.Tests.Shared;

/// <summary>
/// Written STEP as native OCCT wrote it, for comparing with its frozen recordings: Kerf's files name Kerf (unit 11L), and
/// the recordings stay as they are. FILE_DESCRIPTION and PRODUCT's names go back to OCCT's bytes, its line breaks
/// included, and FILE_NAME (the time, the names) is masked. A file another system wrote keeps its description, as the
/// tape hashes the corpus's files through this. Everything else is compared as written.
/// </summary>
internal static partial class NativeStepText
{
    /// <summary>The text with what names the writer as native wrote it, and with FILE_NAME masked.</summary>
    public static string Of(string text)
    {
        string newLine = text.Contains("\r\n", StringComparison.Ordinal) ? "\r\n" : "\n";
        text = FileDescription().Replace(text, "FILE_DESCRIPTION(('Open CASCADE Model'),'2;1');");
        text = FileName().Replace(text, "FILE_NAME(...);");
        return KerfProduct().Replace(text, m =>
            $"PRODUCT('Open CASCADE STEP translator 8.0{m.Groups[1].Value}',{newLine}  'Open CASCADE STEP translator 8.0{m.Groups[1].Value}','',({m.Groups[2].Value}));");
    }

    [GeneratedRegex(@"FILE_DESCRIPTION\(\('Kerf Model'.*?\),\s*'2;1'\);", RegexOptions.Singleline)]
    private static partial Regex FileDescription();

    [GeneratedRegex(@"FILE_NAME\(.*?\);", RegexOptions.Singleline)]
    private static partial Regex FileName();

    [GeneratedRegex(@"PRODUCT\(\s*'Kerf STEP translator( [0-9.]+)?'\s*,\s*'Kerf STEP translator\1'\s*,\s*''\s*,\s*\(\s*(#\d+)\s*\)\s*\)\s*;")]
    private static partial Regex KerfProduct();
}
