// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Per-class and per-method C# written by hand, for what the generator can't translate (decision 12602):
//   overrides/<Class>.cs               the whole file
//   overrides/<Class>.<Method>.<n>.cs  one method (or file-level helper) with n parameters (indented as a class member)
//   overrides/<Class>.statics.cs       the source file's statics that aren't constants (process-wide state), as members

namespace StepGen;

internal sealed class Overrides
{
    private readonly string myDir;
    public readonly HashSet<string> Used = new(StringComparer.Ordinal);

    public Overrides(string theDir) => myDir = theDir;

    private string? Read(string theName)
    {
        string aPath = Path.Combine(myDir, theName);
        if (!File.Exists(aPath))
        {
            return null;
        }

        Used.Add(theName);
        return File.ReadAllText(aPath).Replace("\r\n", "\n");
    }

    public string? WholeClass(string theClass) => Read(theClass + ".cs");

    public string? Method(string theClass, string theMethod, int theArity) => Read($"{theClass}.{theMethod}.{theArity}.cs");

    public string? Statics(string theClass) => Read(theClass + ".statics.cs");

    public IEnumerable<string> All() =>
        Directory.Exists(myDir) ? Directory.EnumerateFiles(myDir, "*.cs").Select(Path.GetFileName).OfType<string>().OrderBy(x => x, StringComparer.Ordinal) : [];
}
