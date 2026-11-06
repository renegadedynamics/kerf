// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;

namespace Kerf.Kernel.DataExchange.Tests.StepGen;

/// <summary>
/// tools/stepgen (units 8C1 and 8C2): the generator is deterministic. Its output was the committed STEP entity classes
/// until unit 10E renamed them.
/// </summary>
public sealed class StepGenTests
{
    private static string Metadata(string theKey) =>
        typeof(StepGenTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string? OcctSource()
    {
        string aDir = Metadata("OcctSourceDir");
        return Directory.Exists(Path.Combine(aDir, "src", "DataExchange", "TKDESTEP", "StepBasic")) ? aDir : null;
    }

    private static string RepoRoot()
    {
        for (var d = new DirectoryInfo(AppContext.BaseDirectory); d != null; d = d.Parent)
        {
            if (File.Exists(Path.Combine(d.FullName, "occt-version.json")))
            {
                return d.FullName;
            }
        }

        throw new InvalidOperationException("not inside the repository");
    }

    private static readonly string[] Packages = global::StepGen.Generator.AllPackages;

    // Unit 10E renamed the committed classes and the clean-up their locals, while the generator still writes OCCT's names, so
    // its output can't equal the committed source. It should still have the same shape: the same tokens, with every name
    // taken as one, and the file's public or internal, a call with no arguments and a property left aside. The files that
    // differ even so were changed by hand on purpose, such as fixes of OCCT's bugs, and are listed (decision 20213): a file
    // joining the list means the generator or the source has changed, and should be looked at.
    [Fact]
    public void CommittedSourcesHaveTheGeneratorsShape()
    {
        string? anOcct = OcctSource();
        if (anOcct is null)
        {
            return; // OCCT's source isn't fetched (tools/get-occt.ps1)
        }

        string anOut = Directory.CreateTempSubdirectory("stepgen-shape-").FullName;
        try
        {
            var aLog = new StringWriter();
            Assert.Equal(0, global::StepGen.Generator.Run(["--occt", anOcct, "--out", anOut], aLog, aLog));

            string aRoot = RepoRoot();
            var aNames = new Dictionary<string, string>();
            foreach (string aLine in File.ReadLines(Path.Combine(aRoot, "src", "Kerf.Kernel.Foundation", "Standard", "OcctTypeNames.g.cs")))
            {
                string[] aParts = aLine.Trim().Split(' ');
                if (aParts.Length == 2 && aParts[0].Contains('.'))
                {
                    aNames[aParts[1]] = aParts[0][(aParts[0].LastIndexOf('.') + 1)..];
                }
            }

            var aDiffering = new List<string>();
            foreach (string aFile in Directory.EnumerateFiles(anOut, "*.cs", SearchOption.AllDirectories))
            {
                string aPackage = Path.GetFileName(Path.GetDirectoryName(aFile))!;
                string aBase = Path.GetFileNameWithoutExtension(aFile);
                string aCommitted = Path.Combine(aRoot, "src", "Kerf.Kernel.DataExchange", aPackage, aNames.GetValueOrDefault(aBase, aBase) + ".cs");
                if (!File.Exists(aCommitted) || Shape(File.ReadAllText(aFile)) != Shape(File.ReadAllText(aCommitted)))
                {
                    aDiffering.Add(aBase);
                }
            }

            aDiffering.Sort(StringComparer.Ordinal);
            string aListFile = Path.Combine(aRoot, "tests", "Kerf.Kernel.DataExchange.Tests", "StepGen", "hand-edited.txt");
            if (Environment.GetEnvironmentVariable("KERF_STEPGEN_LIST") == "1")
            {
                File.WriteAllLines(aListFile, aDiffering);
            }

            string[] anExpected = File.ReadAllLines(aListFile);
            Assert.Equal(anExpected, aDiffering);
        }
        finally
        {
            Directory.Delete(anOut, true);
        }
    }

    private static readonly System.Text.RegularExpressions.Regex s_token =
        new("\"(?:[^\"\\\\]|\\\\.)*\"|\\d[\\d.eE+-]*[fdL]?|[A-Za-z_][A-Za-z0-9_]*|[^\\sA-Za-z0-9_]", System.Text.RegularExpressions.RegexOptions.Compiled);

    private static readonly HashSet<string> s_keywords =
    [
        "if", "else", "for", "foreach", "while", "do", "return", "new", "null", "true", "false", "this", "base", "private", "protected",
        "static", "readonly", "class", "struct", "enum", "void", "int", "double", "bool", "string", "out", "ref", "in", "is", "as", "var",
        "get", "set", "const", "abstract", "virtual", "using",
    ];

    private static string Shape(string theSource)
    {
        string aText = System.Text.RegularExpressions.Regex.Replace(theSource, @"/\*.*?\*/", "", System.Text.RegularExpressions.RegexOptions.Singleline);
        aText = System.Text.RegularExpressions.Regex.Replace(aText, "//[^\n]*", "");
        aText = System.Text.RegularExpressions.Regex.Replace(aText, "^(using|namespace) [^\n]*$", "", System.Text.RegularExpressions.RegexOptions.Multiline);
        aText = System.Text.RegularExpressions.Regex.Replace(aText, @"\[[A-Z]\w*(\([^\]]*\))?\]", "");
        var aTokens = new List<string>();
        foreach (System.Text.RegularExpressions.Match aMatch in s_token.Matches(aText))
        {
            string aToken = aMatch.Value;
            aTokens.Add((char.IsLetter(aToken[0]) || aToken[0] == '_') && !s_keywords.Contains(aToken) ? "I" : aToken);
        }

        string aShape = string.Join(' ', aTokens);
        aShape = System.Text.RegularExpressions.Regex.Replace(aShape, @"I( \. I)+", "I");
        aShape = aShape.Replace("I : : I", "I").Replace("I ( )", "I").Replace("{ get { return I ; } }", "{ return I ; }");
        return System.Text.RegularExpressions.Regex.Replace(aShape, @"I( . I)+", "I");
    }

    [Fact]
    public void GeneratorIsDeterministic()
    {
        string? anOcct = OcctSource();
        if (anOcct is null)
        {
            return;
        }

        string aTemp = Path.Combine(Path.GetTempPath(), "stepgen-" + Guid.NewGuid().ToString("N"));
        try
        {
            string a = Path.Combine(aTemp, "a"), b = Path.Combine(aTemp, "b");
            foreach (string aDir in new[] { a, b })
            {
                int aStatus = global::StepGen.Generator.Run(["--occt", anOcct, "--out", aDir, "--packages", string.Join(",", Packages)],
                                                            TextWriter.Null, TextWriter.Null);
                Assert.Equal(0, aStatus);
            }

            var aFiles = Directory.GetFiles(a, "*.cs", SearchOption.AllDirectories).Select(f => Path.GetRelativePath(a, f)).Order(StringComparer.Ordinal).ToList();
            var bFiles = Directory.GetFiles(b, "*.cs", SearchOption.AllDirectories).Select(f => Path.GetRelativePath(b, f)).Order(StringComparer.Ordinal).ToList();
            Assert.Equal(aFiles, bFiles);
            Assert.True(aFiles.Count > 1600);
            foreach (string f in aFiles)
            {
                Assert.True(File.ReadAllBytes(Path.Combine(a, f)).AsSpan().SequenceEqual(File.ReadAllBytes(Path.Combine(b, f))), f);
            }
        }
        finally
        {
            if (Directory.Exists(aTemp))
            {
                Directory.Delete(aTemp, true);
            }
        }
    }
}
