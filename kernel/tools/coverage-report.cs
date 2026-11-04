// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Merges the Cobertura files of a coverage run (tools/coverage.ps1) and writes line coverage per OCCT package (the
// folder under each kernel assembly) and per folder of the modeling layer, with the packages under the floor listed
// (phase10-plan.md 3.2, question 49: 80% per engine package). A line counts as covered if any test run covered it.
//
// The reason for each package under the floor comes from docs/coverage-gaps.tsv (assembly, package, reason), kept by hand;
// a package without one shows as "no reason recorded".
//
//   dotnet run tools/coverage-report.cs -- <results folder> <report.md> [floor percent, 80 by default]

#:property PublishAot=false

using System.Globalization;
using System.Text;
using System.Xml;

if (args.Length < 2)
{
    Console.Error.WriteLine("usage: dotnet run tools/coverage-report.cs -- <results folder> <report.md> [floor]");
    return 1;
}
string aResults = Path.GetFullPath(args[0]);
string aReport = Path.GetFullPath(args[1]);
double aFloor = args.Length > 2 ? double.Parse(args[2], CultureInfo.InvariantCulture) : 80;
var aReasons = new Dictionary<(string Area, string Package), string>();
string aReasonsFile = Path.Combine(Path.GetDirectoryName(aReport)!, "coverage-gaps.tsv");
if (File.Exists(aReasonsFile))
{
    foreach (string aLine in File.ReadLines(aReasonsFile).Where(l => l.Length > 0 && l[0] != '#'))
    {
        string[] aParts = aLine.Split('\t');
        aReasons[(aParts[0], aParts[1])] = aParts[2];
    }
}

// (file, line) -> covered, for the source files of the kernel's and the modeling layer's assemblies
var aLines = new Dictionary<string, Dictionary<int, bool>>(StringComparer.OrdinalIgnoreCase);
string[] aFiles = Directory.GetFiles(aResults, "*.cobertura.xml", SearchOption.AllDirectories);
var aRuns = new List<string>();
foreach (string aFile in aFiles)
{
    aRuns.Add(Path.GetRelativePath(aResults, aFile).Split(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)[0]);
    using var aReader = XmlReader.Create(aFile, new XmlReaderSettings { DtdProcessing = DtdProcessing.Ignore });
    string? aClassFile = null;
    while (aReader.Read())
    {
        if (aReader.NodeType != XmlNodeType.Element)
        {
            continue;
        }
        if (aReader.Name == "class")
        {
            aClassFile = Normalise(aReader.GetAttribute("filename"));
        }
        else if (aReader.Name == "line" && aClassFile is not null && Area(aClassFile) is not null)
        {
            int aNumber = int.Parse(aReader.GetAttribute("number")!, CultureInfo.InvariantCulture);
            bool isHit = long.Parse(aReader.GetAttribute("hits") ?? "0", CultureInfo.InvariantCulture) > 0;
            if (!aLines.TryGetValue(aClassFile, out var aFileLines))
            {
                aLines[aClassFile] = aFileLines = [];
            }
            aFileLines[aNumber] = aFileLines.GetValueOrDefault(aNumber) || isHit;
        }
    }
}

// per area (assembly) and package (folder)
var aPackages = new SortedDictionary<(string Area, string Package), (int Lines, int Covered, int Files)>();
foreach ((string aFile, var aFileLines) in aLines)
{
    (string anArea, string aPackage) = Area(aFile)!.Value;
    var e = aPackages.GetValueOrDefault((anArea, aPackage));
    aPackages[(anArea, aPackage)] = (e.Lines + aFileLines.Count, e.Covered + aFileLines.Values.Count(v => v), e.Files + 1);
}

var aText = new StringBuilder();
aText.Append("""
    # Line coverage

    The floor of question 49 (phase10-plan.md 3.2): every public member tested, at least 80% line coverage per engine
    package, lower where the Draw-lite corpus covers a package and the gap is listed. Written by `tools/coverage.ps1`
    (`tools/coverage-report.cs`): line coverage of the kernel's and the modeling layer's assemblies, merged (a line is
    covered if any run covered it): the kernel's unit tests, the public API's tests, the Draw-lite tests as they run by
    default (the corpus classes on their samples, `OCCTNET_CORPUS_FULL` unset), the modeling layer's suite, and the
    oracle tests' replays with `-WithOracle`. A package is the folder under the assembly (OCCT's package for the kernel).
    Lines are the coverable lines Microsoft's code coverage counts. The public members no test calls are in
    [public-members.md](public-members.md).


    """);
aText.Append(CultureInfo.InvariantCulture, $"Measured on {DateTime.Now:d MMMM yyyy}.\n");
aText.Append("Test projects merged: ").Append(string.Join(", ", aRuns.Distinct().Order(StringComparer.Ordinal).Select(r => $"`{r}`"))).Append(".\n");
aText.Append("\n## By assembly\n\n| Assembly | Packages | Lines | Covered | % | Packages under ").Append(aFloor.ToString(CultureInfo.InvariantCulture)).Append("% |\n|---|---:|---:|---:|---:|---:|\n");
foreach (var aGroup in aPackages.GroupBy(p => p.Key.Area))
{
    int aL = aGroup.Sum(p => p.Value.Lines), aC = aGroup.Sum(p => p.Value.Covered);
    int anUnder = aGroup.Count(p => Percent(p.Value.Covered, p.Value.Lines) < aFloor);
    aText.Append(CultureInfo.InvariantCulture, $"| {aGroup.Key} | {aGroup.Count()} | {aL:N0} | {aC:N0} | {Percent(aC, aL):F1} | {anUnder} |\n");
}
int aTotalL = aPackages.Values.Sum(v => v.Lines), aTotalC = aPackages.Values.Sum(v => v.Covered);
aText.Append(CultureInfo.InvariantCulture, $"| **Total** | {aPackages.Count} | {aTotalL:N0} | {aTotalC:N0} | {Percent(aTotalC, aTotalL):F1} | {aPackages.Count(p => Percent(p.Value.Covered, p.Value.Lines) < aFloor)} |\n");

aText.Append("\n## The gaps\n\nEvery package under the floor, with the reason (`docs/coverage-gaps.tsv`).\n\n");
aText.Append("| Assembly | Package | Lines | Uncovered | % | Reason |\n|---|---|---:|---:|---:|---|\n");
foreach (var p in aPackages.Where(p => Percent(p.Value.Covered, p.Value.Lines) < aFloor)
                           .OrderBy(p => p.Key.Area, StringComparer.Ordinal).ThenByDescending(p => p.Value.Lines - p.Value.Covered))
{
    string aReason = aReasons.GetValueOrDefault(p.Key) ?? "no reason recorded";
    aText.Append(CultureInfo.InvariantCulture,
        $"| {p.Key.Area} | {p.Key.Package} | {p.Value.Lines:N0} | {p.Value.Lines - p.Value.Covered:N0} | {Percent(p.Value.Covered, p.Value.Lines):F1} | {aReason} |\n");
}

foreach (var aGroup in aPackages.GroupBy(p => p.Key.Area))
{
    aText.Append(CultureInfo.InvariantCulture, $"\n## {aGroup.Key}\n\nPackages under {aFloor}%, least covered first (uncovered lines in brackets); the rest after.\n\n");
    aText.Append("| Package | Files | Lines | Covered | % |\n|---|---:|---:|---:|---:|\n");
    foreach (var p in aGroup.OrderBy(p => Percent(p.Value.Covered, p.Value.Lines) >= aFloor).ThenBy(p => Percent(p.Value.Covered, p.Value.Lines))
                            .ThenBy(p => p.Key.Package, StringComparer.Ordinal))
    {
        double aPercent = Percent(p.Value.Covered, p.Value.Lines);
        string aName = aPercent < aFloor ? $"**{p.Key.Package}**" : p.Key.Package;
        aText.Append(CultureInfo.InvariantCulture,
            $"| {aName} | {p.Value.Files} | {p.Value.Lines:N0} | {p.Value.Covered:N0} ({p.Value.Lines - p.Value.Covered:N0}) | {aPercent:F1} |\n");
    }
}
Directory.CreateDirectory(Path.GetDirectoryName(aReport)!);
File.WriteAllText(aReport, aText.ToString().Replace("\r\n", "\n"));
Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{aFiles.Length} runs, {aLines.Count} files, {aTotalC:N0} of {aTotalL:N0} lines ({Percent(aTotalC, aTotalL):F1}%) -> {aReport}"));
return 0;

static double Percent(int theCovered, int theLines) => theLines == 0 ? 100 : 100.0 * theCovered / theLines;

static string? Normalise(string? thePath) => thePath?.Replace('\\', '/');

// The kernel's assemblies by OCCT package (the folder under the assembly), the modeling layer's by folder
static (string Area, string Package)? Area(string theFile)
{
    foreach (string aRoot in new[] { "/kernel/src/", "/modeling/src/" })
    {
        int i = theFile.IndexOf(aRoot, StringComparison.OrdinalIgnoreCase);
        if (i < 0)
        {
            continue;
        }
        string[] aParts = theFile[(i + aRoot.Length)..].Split('/');
        if (aParts[0] is "Kerf.Analyzers" || aParts.Length < 2 || aParts[1] is "obj" or "bin")
        {
            return null;
        }
        return (aParts[0], aParts.Length > 2 ? aParts[1] : "(root)");
    }
    return null;
}
