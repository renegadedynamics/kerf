// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DrawLite;

/// <summary>One test case of OCCT's corpus: tests/&lt;group&gt;/&lt;grid&gt;/&lt;case&gt;.</summary>
public sealed record DrawTestCase(string Group, string Grid, string Name, string Path)
{
    public string Id => $"{Group}/{Grid}/{Name}";
}

/// <summary>What a test case needs: the commands it calls that the interpreter doesn't have yet.</summary>
public sealed record DrawCaseScan(DrawTestCase Case, IReadOnlySet<string> Commands, IReadOnlySet<string> Missing)
{
    public bool IsRunnable => Missing.Count == 0;
}

/// <summary>
/// OCCT's Draw test corpus (tests/ in the OCCT source). A case runs as group/begin, grid/begin, the case,
/// grid/end, group/end, which is how OCCT's own test harness assembles it.
/// </summary>
public sealed class DrawCorpus(string theTestsDirectory)
{
    public string TestsDirectory { get; } = theTestsDirectory;

    public IEnumerable<DrawTestCase> Cases()
    {
        foreach (string aGroup in Directory.EnumerateDirectories(TestsDirectory).Order(StringComparer.Ordinal))
        {
            foreach (string aGrid in Directory.EnumerateDirectories(aGroup).Order(StringComparer.Ordinal))
            {
                foreach (string aCase in Directory.EnumerateFiles(aGrid).Order(StringComparer.Ordinal))
                {
                    string aName = Path.GetFileName(aCase);
                    if (aName is "begin" or "end" or "parse.rules" or "cases.list" || aName.Contains('.'))
                    {
                        continue;
                    }

                    yield return new DrawTestCase(Path.GetFileName(aGroup), Path.GetFileName(aGrid), aName, aCase);
                }
            }
        }
    }

    /// <summary>The whole script of a case, with its group's and grid's begin and end scripts around it.</summary>
    public string Script(DrawTestCase theCase)
    {
        string aGroupDir = Path.Combine(TestsDirectory, theCase.Group);
        string aGridDir = Path.Combine(aGroupDir, theCase.Grid);
        return string.Join('\n',
                           ReadIfExists(Path.Combine(aGroupDir, "begin")),
                           ReadIfExists(Path.Combine(aGridDir, "begin")),
                           File.ReadAllText(theCase.Path),
                           ReadIfExists(Path.Combine(aGridDir, "end")),
                           ReadIfExists(Path.Combine(aGroupDir, "end")));
    }

    /// <summary>
    /// The commands a case calls, found without running it: the first word of each command, including those in
    /// braced bodies (if, foreach, proc) and brackets. Words that are themselves substituted are skipped.
    /// </summary>
    public DrawCaseScan Scan(DrawTestCase theCase, DrawInterpreter theInterpreter)
    {
        var aCommands = new HashSet<string>(StringComparer.Ordinal);
        var aDefined = new HashSet<string>(StringComparer.Ordinal);
        try
        {
            Collect(Script(theCase), aCommands, aDefined, theDepth: 0);
        }
        catch (DrawException)
        {
            aCommands.Add("<unparsable>");
        }

        var aMissing = aCommands.Where(c => !theInterpreter.HasCommand(c) && !aDefined.Contains(c)).ToHashSet(StringComparer.Ordinal);
        return new DrawCaseScan(theCase, aCommands, aMissing);
    }

    private static void Collect(string theScript, HashSet<string> theCommands, HashSet<string> theDefined, int theDepth)
    {
        if (theDepth > 20)
        {
            return;
        }

        foreach (Command aCommand in TclParser.Parse(theScript))
        {
            if (aCommand.Words[0].Literal is { } aName)
            {
                theCommands.Add(aName);
                if (aName == "proc" && aCommand.Words.Count > 1 && aCommand.Words[1].Literal is { } aProc)
                {
                    theDefined.Add(aProc);
                }
            }

            foreach (Word aWord in aCommand.Words.Skip(1))
            {
                foreach (WordPart aPart in aWord.Parts)
                {
                    // Braced words are often bodies (if, foreach, proc); bracketed parts always are scripts.
                    // A braced word that is plain data parses as harmless "commands"; expr operands are the
                    // main source of noise, which the report tolerates.
                    string? aNested = aPart switch
                    {
                        WordPart.Command aBracketed => aBracketed.Script,
                        WordPart.Text aText when aWord.IsBraced && aText.Value.Contains('\n') => aText.Value,
                        _ => null,
                    };
                    if (aNested is not null)
                    {
                        Collect(aNested, theCommands, theDefined, theDepth + 1);
                    }
                }
            }
        }
    }

    private static string ReadIfExists(string thePath) => File.Exists(thePath) ? File.ReadAllText(thePath) : "";
}
