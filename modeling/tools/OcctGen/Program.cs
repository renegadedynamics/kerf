using ClangSharp;
using ClangSharp.Interop;
using OcctGen;

// OcctGen <occt-include-dir> <whitelist> <cpp-out-dir> <cs-out-dir> [skipped-report]
if (args.Length < 4)
{
    Console.Error.WriteLine("usage: OcctGen <occt-include-dir> <whitelist> <cpp-out-dir> <cs-out-dir> [skipped-report]");
    return 2;
}
var (include, whitelist, cppOut, csOut) = (args[0], args[1], args[2], args[3]);
var report = args.Length > 4 ? args[4] : null;

var config = Config.Load(whitelist);

// Parse every whitelisted header as one translation unit
string[] clangArgs = ["-x", "c++", "-std=c++17", $"-I{include}", "-Wno-everything",
                      .. OperatingSystem.IsWindows() ? new[] { "-fms-compatibility", "-fms-extensions" } : []];
using var index = CXIndex.Create();

CXTranslationUnit Parse(IEnumerable<string> extra)
{
    var umbrella = Path.Combine(Path.GetTempPath(), $"occtgen_{Environment.ProcessId}.cpp");
    File.WriteAllText(umbrella, string.Join("\n", config.Classes.Select(c => $"#include <{c}.hxx>").Concat(extra)) + "\n");
    var status = CXTranslationUnit.TryParse(index, umbrella, clangArgs, [],
        CXTranslationUnit_Flags.CXTranslationUnit_SkipFunctionBodies, out var h);
    File.Delete(umbrella);
    if (status != CXErrorCode.CXError_Success)
        throw new InvalidOperationException($"clang failed: {status}");
    return h;
}

var handle = Parse([]);
// A template alias is only instantiated where it's used; make clang instantiate the whitelisted ones
{
    // (the probe isn't disposed: that would dispose the handle it wraps)
    var probe = TranslationUnit.GetOrCreate(handle);
    var pending = Model.TemplateAliases(probe, config).Where(a => a.Value.Def is null).Select(a => a.Key).ToList();
    if (pending.Count > 0)
    {
        handle = Parse(pending.Select(a => $"static_assert(sizeof({a}) > 0, \"\");"));
    }
}

int errors = 0;
for (uint i = 0; i < handle.NumDiagnostics; i++)
{
    using var diag = handle.GetDiagnostic(i);
    if (diag.Severity >= CXDiagnosticSeverity.CXDiagnostic_Error)
    {
        Console.Error.WriteLine(diag.Format(CXDiagnostic.DefaultDisplayOptions));
        errors++;
    }
}
if (errors > 0)
    return 1;

using var tu = TranslationUnit.GetOrCreate(handle);
var model = Model.Build(tu, config);

CppEmitter.Emit(model, cppOut);
var bound = model.Classes.Keys.Concat(model.Collections.Values.Select(c => c.Name)).ToHashSet();
Writer.Prune(cppOut, "*.cpp", bound.Select(c => Path.Combine(cppOut, c + ".cpp")));
var csFiles = CsEmitter.Emit(model, csOut).ToList();
foreach (var c in model.Collections.Values.Distinct())
{
    var path = Path.Combine(csOut, c.Name + ".g.cs");
    Writer.WriteIfChanged(path, CollectionEmitter.EmitCs(c.Collection!));
    csFiles.Add(path);
}
Writer.Prune(csOut, "*.g.cs", csFiles);

if (report != null)
    Writer.WriteIfChanged(report,
        "# Members skipped by OcctGen, and why. Generated; do not edit.\n"
        + string.Concat(model.Skipped.Distinct().Order().Select(s => s + "\n")));

var members = model.Classes.Values.Sum(c => c.Members.Count);
Console.WriteLine($"{model.Classes.Count} classes, {model.Enums.Count} enums, {members} functions, {model.Skipped.Count} skipped");
return 0;
