// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// tools/stepgen: generates OCCT.NET's STEP entity classes and their readers and writers from OCCT 8's own C++
// (phase8-plan.md 2.1, decision 1166), by translating each class's header and source to C# by
// docs/translation-rules.md (decisions 12601 to 12610).
//
//   dotnet run --project tools/stepgen -- [--packages StepBasic,RWStepBasic,...] [--check] [--report file]
//
// Without --packages it generates unit 8C1's packages (StepBasic, StepGeom, StepShape, StepRepr and their RW
// packages). --check writes nothing and fails if a file differs from what would be written.

using System.Text;
using StepGen;

string? aRoot = FindRoot(Directory.GetCurrentDirectory()) ?? FindRoot(AppContext.BaseDirectory);
string? anOcct = null, anOut = null, aReport = null, aStubs = null;
var aPackages = new List<string> { "StepBasic", "StepGeom", "StepShape", "StepRepr", "RWStepBasic", "RWStepGeom", "RWStepShape", "RWStepRepr" };
bool isCheck = false;
for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--occt": anOcct = args[++i]; break;
        case "--out": anOut = args[++i]; break;
        case "--packages": aPackages = args[++i].Split(',', StringSplitOptions.RemoveEmptyEntries).ToList(); break;
        case "--check": isCheck = true; break;
        case "--report": aReport = args[++i]; break;
        case "--stubs": aStubs = args[++i]; break;
        default:
            Console.Error.WriteLine($"unknown argument {args[i]}");
            return 2;
    }
}

anOcct ??= aRoot != null ? Path.Combine(aRoot, ".deps", "occt-src") : null;
anOut ??= aRoot != null ? Path.Combine(aRoot, "src", "OcctNet.DataExchange") : null;
if (anOcct == null || !Directory.Exists(anOcct) || anOut == null)
{
    Console.Error.WriteLine("No OCCT source: run from inside the repository after tools/get-occt.ps1, or pass --occt and --out.");
    return 2;
}

string aStepDir = Path.Combine(anOcct, "src", "DataExchange", "TKDESTEP");
string anOverridesDir = Path.Combine(aRoot ?? ".", "tools", "stepgen", "overrides");

// ---------------------------------------------------------------- the database
var aDb = new Db();
var aPackageDirs = Directory.GetDirectories(aStepDir).Select(Path.GetFileName).OfType<string>().OrderBy(x => x, StringComparer.Ordinal).ToList();
foreach (string aPkg in aPackageDirs)
{
    foreach (string aFile in Directory.EnumerateFiles(Path.Combine(aStepDir, aPkg)).Where(f => f.EndsWith(".hxx") || f.EndsWith(".pxx")).OrderBy(x => x, StringComparer.Ordinal))
    {
        aDb.AddHeader(aFile, Rel(anOcct, aFile), aPkg);
    }
}

foreach (string aDir in new[] { "src/DataExchange/TKXSBase/Interface" })
{
    foreach (string aFile in Directory.EnumerateFiles(Path.Combine(anOcct, aDir), "*.hxx").OrderBy(x => x, StringComparer.Ordinal))
    {
        aDb.AddHeader(aFile, Rel(anOcct, aFile), "Interface");
    }
}

foreach (string aFile in new[] { "src/FoundationClasses/TKernel/TCollection/TCollection_HAsciiString.hxx", "src/FoundationClasses/TKernel/Standard/Standard_Transient.hxx" })
{
    aDb.AddHeader(Path.Combine(anOcct, aFile), aFile, "Foundation");
}

foreach (string aPkg in aPackageDirs)
{
    foreach (string aFile in Directory.EnumerateFiles(Path.Combine(aStepDir, aPkg), "*.cxx").OrderBy(x => x, StringComparer.Ordinal))
    {
        foreach (System.Text.RegularExpressions.Match m in System.Text.RegularExpressions.Regex.Matches(File.ReadAllText(aFile), @"(?:->|\.)\s*(Step\w+)::(\w+)\s*\("))
        {
            aDb.QualifiedCalls.Add(m.Groups[1].Value + "::" + m.Groups[2].Value);
        }
    }
}

// value classes: the entity packages' classes C++ holds by value (SELECT types and the plain ones)
foreach (var c in aDb.Classes.Values)
{
    if (c.Package.StartsWith("Step", StringComparison.Ordinal) && c.Package != "StepData" && c.Package != "StepFile" && !c.IsNamespace
        && !aDb.IsTransient(c.Name) && c.Name != "Standard_Transient")
    {
        aDb.ValueClasses.Add(c.Name);
    }
}

if (aStubs != null)
{
    return Stubs.Write(aDb, anOcct, aStubs);
}

// ---------------------------------------------------------------- generation
var anOverrides = new Overrides(anOverridesDir);
var anEmitter = new Emitter(aDb, anOcct, anOverrides);
var aResults = new List<ClassResult>();
foreach (string aPkg in aPackages)
{
    string aDir = Path.Combine(aStepDir, aPkg);
    if (!Directory.Exists(aDir))
    {
        Console.Error.WriteLine($"no package {aPkg}");
        return 2;
    }

    foreach (string aHeader in Directory.EnumerateFiles(aDir).Where(f => f.EndsWith(".hxx") || f.EndsWith(".pxx")).OrderBy(x => x, StringComparer.Ordinal))
    {
        string aRel = Rel(anOcct, aHeader);
        string aStem = Path.GetFileNameWithoutExtension(aHeader);
        var aClasses = aDb.Classes.Values.Where(c => c.Header == aRel).OrderBy(c => c.Name, StringComparer.Ordinal).ToList();
        var anEnums = aDb.Enums.Values.Where(e => e.Name == aStem && aClasses.Count == 0).ToList();
        foreach (var e in anEnums)
        {
            aResults.Add(anEmitter.EmitEnum(e, aRel, aPkg));
        }

        foreach (var cls in aClasses)
        {
            aDb.Generated.Add(cls.Name);
            string aSource = Path.Combine(aDir, cls.Name + ".cxx");
            var aDefs = new List<Parser.SourceFunction>();
            var aConsts = new List<CppField>();
            if (File.Exists(aSource))
            {
                cls.Sources.Add(Rel(anOcct, aSource));
                var (aFns, aCs, aProblems) = Parser.ParseSource(Lexer.Lex(File.ReadAllText(aSource)));
                aDefs = aFns;
                aConsts = aCs;
                cls.Problems.AddRange(aProblems);
            }

            ClassResult r;
            try
            {
                r = anEmitter.EmitClass(cls, aDefs, aConsts);
            }
            catch (Exception ex) when (ex is FormatException or InvalidOperationException or ArgumentOutOfRangeException or NullReferenceException)
            {
                r = new ClassResult { Name = cls.Name, Package = aPkg, Problems = { "the generator failed: " + ex.Message } };
            }

            aResults.Add(r);
        }

        if (aClasses.Count == 0 && anEnums.Count == 0)
        {
            aResults.Add(new ClassResult { Name = aStem, Package = aPkg, Problems = { "no class or enum found in " + aRel } });
        }
    }
}

// ---------------------------------------------------------------- output
int aDiffs = 0, aWritten = 0;
var anExpected = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
foreach (var r in aResults.Where(x => x.Text.Length > 0))
{
    r.Lines = r.Text.Count(ch => ch == '\n');
    string aPath = Path.Combine(anOut, r.Package, r.Name + ".cs");
    anExpected.Add(Path.GetFullPath(aPath));
    string? anOld = File.Exists(aPath) ? File.ReadAllText(aPath).Replace("\r\n", "\n") : null;
    if (anOld == r.Text)
    {
        continue;
    }

    aDiffs++;
    if (!isCheck)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(aPath)!);
        File.WriteAllText(aPath, r.Text, new UTF8Encoding(false));
        aWritten++;
    }
    else
    {
        Console.Error.WriteLine($"differs: {aPath}");
    }
}

// stale generated files
foreach (string aPkg in aPackages)
{
    string aDir = Path.Combine(anOut, aPkg);
    if (!Directory.Exists(aDir))
    {
        continue;
    }

    foreach (string aFile in Directory.EnumerateFiles(aDir, "*.cs"))
    {
        if (!anExpected.Contains(Path.GetFullPath(aFile)) && File.ReadLines(aFile).Take(8).Any(l => l.StartsWith(Emitter.GeneratedLine[..30], StringComparison.Ordinal)))
        {
            aDiffs++;
            if (isCheck)
            {
                Console.Error.WriteLine($"stale: {aFile}");
            }
            else
            {
                File.Delete(aFile);
            }
        }
    }
}

// report
var aSb = new StringBuilder();
aSb.Append("# stepgen report\n\n");
aSb.Append("| Package | Files | Lines | With problems |\n|---|---:|---:|---:|\n");
foreach (var g in aResults.GroupBy(x => x.Package))
{
    aSb.Append($"| {g.Key} | {g.Count(x => x.Text.Length > 0)} | {g.Sum(x => x.Lines)} | {g.Count(x => x.Problems.Count > 0)} |\n");
}

aSb.Append($"| **All** | {aResults.Count(x => x.Text.Length > 0)} | {aResults.Sum(x => x.Lines)} | {aResults.Count(x => x.Problems.Count > 0)} |\n\n");
aSb.Append("## Written by hand (tools/stepgen/overrides)\n\n");
foreach (string o in anOverrides.Used.OrderBy(x => x, StringComparer.Ordinal))
{
    aSb.Append("- ").Append(o).Append('\n');
}

aSb.Append("\n## Problems\n\n");
foreach (var r in aResults.Where(x => x.Problems.Count > 0).OrderBy(x => x.Name, StringComparer.Ordinal))
{
    foreach (string p in r.Problems)
    {
        aSb.Append("- ").Append(r.Name).Append(": ").Append(p.Replace("\n", " ")).Append('\n');
    }
}

foreach (string l in aDb.Log)
{
    aSb.Append("- (db) ").Append(l).Append('\n');
}

if (aReport != null)
{
    File.WriteAllText(aReport, aSb.ToString(), new UTF8Encoding(false));
}

Console.WriteLine($"stepgen: {aResults.Count} files, {aResults.Sum(x => x.Lines)} lines, {aResults.Count(x => x.Problems.Count > 0)} with problems; "
                  + (isCheck ? $"{aDiffs} differ" : $"{aWritten} written"));
return isCheck && aDiffs > 0 ? 1 : 0;

static string Rel(string theRoot, string thePath) => Path.GetRelativePath(theRoot, thePath).Replace('\\', '/');

static string? FindRoot(string theStart)
{
    for (var d = new DirectoryInfo(theStart); d != null; d = d.Parent)
    {
        if (File.Exists(Path.Combine(d.FullName, "occt-version.json")))
        {
            return d.FullName;
        }
    }

    return null;
}
