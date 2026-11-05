// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Classifies every public type of the kernel's four assemblies as public API, engine or removed (phase10-plan.md 2.1,
// unit 10D), from the rules in docs/api/classification-rules.txt, and writes:
//   docs/api/classification.tsv   one row per public type (nested types included), with its new name
//   kerf-seed.map                 kerf-rename's seed: a package line per package, a type line per top-level type
//                                 (to the temporary folder, or where --seed-out says; it isn't committed)
// and prints the counts by assembly and area. Build the kernel in Release first.
//
//   dotnet run tools/api-classify.cs [--check]
//   dotnet run tools/api-classify.cs --finish-map <draft.map> <out.map>
//
// The second form turns kerf-rename's draft (made with --seed docs/api/kerf-seed.map over the four assemblies) into
// docs/api/kerf.map: it drops every entry of a removed type, which keeps its OCCT name until 10J deletes it, and spells
// OCCT's capitalised enum members in PascalCase (TopAbs_FORWARD -> Forward, not FORWARD).
//
// --check writes nothing and fails if a type has no rule, two types get one name, or a public name is a common .NET
// type's.

using System.Reflection;
using System.Reflection.Metadata;
using System.Reflection.PortableExecutable;
using System.Text;

string root = FindRoot(Directory.GetCurrentDirectory()) ?? throw new InvalidOperationException("Run from inside the repository.");
string kernel = Path.Combine(root, "kernel");
string docs = Path.Combine(kernel, "docs", "api");
bool checkOnly = args.Contains("--check");
string[] assemblies = ["Kerf.Kernel.Foundation", "Kerf.Kernel.ModelingData", "Kerf.Kernel.ModelingAlgorithms", "Kerf.Kernel.DataExchange"];

var rules = Rules.Load(Path.Combine(docs, "classification-rules.txt"));
var modeling = File.Exists(Path.Combine(docs, "modeling-uses.txt"))
    ? File.ReadLines(Path.Combine(docs, "modeling-uses.txt")).Where(l => l.Length > 0 && l[0] != '#')
        .Select(l => l.Split('\t')).ToDictionary(p => p[0], p => p[1])
    : [];

// Names a public type mustn't take (design-rules.md section 1): two usings would make them ambiguous.
string[] dotnetNames = ["Math", "Vector", "Vector2", "Vector3", "Vector4", "Matrix", "Matrix3x2", "Matrix4x4", "Plane", "Quaternion",
    "Point", "PointF", "Size", "Rectangle", "Color", "Timer", "Path", "File", "Directory", "Buffer", "Array", "List", "Queue", "Stack",
    "Tuple", "Range", "Index", "Exception", "Attribute", "Type", "Enum", "Object", "String", "Version", "Environment", "Random",
    "Monitor", "Convert", "Task", "Thread", "Action", "Func", "Delegate", "Encoding", "Stream", "Half", "Complex", "Interval", "Lock"];
// System.Numerics has Plane and Quaternion; the pilot named gp_Pln Plane and gp_Quaternion Quaternion. Allowed only
// where the rules say so explicitly ('allow').

var rows = new List<Row>();
var crossRefs = new HashSet<(string From, string To, string Type)>();
foreach (string name in assemblies)
{
    string dll = Path.Combine(kernel, "src", name, "bin", "Release", "net10.0", name + ".dll");
    if (!File.Exists(dll))
    {
        Console.Error.WriteLine($"No {dll}: build the kernel in Release first.");
        return 1;
    }

    using var stream = File.OpenRead(dll);
    using var pe = new PEReader(stream);
    MetadataReader md = pe.GetMetadataReader();
    foreach (TypeDefinitionHandle handle in md.TypeDefinitions)
    {
        TypeDefinition type = md.GetTypeDefinition(handle);
        if (!IsVisible(md, type))
        {
            continue;
        }

        string metaName = md.GetString(type.Name);
        string ns = md.GetString(type.Namespace);
        var chain = new List<string> { metaName };
        TypeDefinition outer = type;
        while (outer.GetDeclaringType() is { IsNil: false } parent)
        {
            outer = md.GetTypeDefinition(parent);
            chain.Insert(0, md.GetString(outer.Name));
            ns = md.GetString(outer.Namespace);
        }

        rows.Add(new Row(name, ns, chain, Kind(md, type)));
    }

    // The kernel's types this assembly uses from another kernel assembly: what an engine without InternalsVisibleTo
    // between the assemblies would have to keep public.
    foreach (TypeReferenceHandle handle in md.TypeReferences)
    {
        TypeReference reference = md.GetTypeReference(handle);
        if (reference.ResolutionScope.Kind == HandleKind.AssemblyReference)
        {
            string target = md.GetString(md.GetAssemblyReference((AssemblyReferenceHandle)reference.ResolutionScope).Name);
            if (assemblies.Contains(target))
            {
                crossRefs.Add((name, target, md.GetString(reference.Name)));
            }
        }
    }
}

// Classify the top-level types, then the nested ones after their outer type.
var problems = new List<string>();
var byName = new Dictionary<string, Row>();
foreach (Row row in rows.Where(r => r.Chain.Count == 1))
{
    string plain = Plain(row.Chain[0]);
    string package = Package(plain);
    if (rules.Classify(plain, package) is not { } rule)
    {
        problems.Add($"no rule: {row.Assembly} {plain} (package {package})");
        continue;
    }

    row.Package = package;
    row.Rule = rule;
    row.NewName = rule.Class == "removed" ? rule.Namespace(rules) + "." + plain
        : rule.Namespace(rules) + "." + (rule.Name ?? rule.Prefix + Stem(plain, package) + rule.Suffix);
    row.Modeling = modeling.GetValueOrDefault(plain, "");
    string key = row.NewName + Arity(row.Chain[0]);
    if (byName.TryGetValue(key, out Row? other))
    {
        problems.Add($"one name for two types: {key} <- {plain}, {Plain(other.Chain[0])}");
    }

    byName[key] = row;
    string simple = row.NewName[(row.NewName.LastIndexOf('.') + 1)..];
    if (rule.Class == "public" && dotnetNames.Contains(simple) && !rules.Allowed.Contains(simple))
    {
        problems.Add($"a common .NET name: {row.NewName} <- {plain}");
    }
}

foreach (Row row in rows.Where(r => r.Chain.Count > 1))
{
    Row? outer = rows.FirstOrDefault(r => r.Chain.Count == 1 && r.Assembly == row.Assembly && r.Chain[0] == row.Chain[0]);
    row.Package = outer?.Package ?? "";
    row.Rule = outer?.Rule;
    row.NewName = outer?.NewName is { } o ? o + "." + string.Join(".", row.Chain.Skip(1).Select(Plain)) : "";
    row.Modeling = "";
}

// An engine type named like a public one would capture that name for code in its namespace: reported, not an error.
var publicNames = rows.Where(r => r.Chain.Count == 1 && r.Rule?.Class == "public").Select(r => Simple(r.NewName)).ToHashSet();
var shadows = rows.Where(r => r.Chain.Count == 1 && r.Rule?.Class == "engine" && publicNames.Contains(Simple(r.NewName)))
    .Select(r => $"{r.NewName} <- {Plain(r.Chain[0])}").ToList();
if (shadows.Count > 0)
{
    Console.WriteLine($"Engine types named like a public type ({shadows.Count}): " + string.Join("; ", shadows));
}

foreach (string unused in rules.Unused())
{
    problems.Add($"rule matches nothing: {unused}");
}

foreach (string problem in problems)
{
    Console.Error.WriteLine(problem);
}

// Counts
string[] classes = ["public", "engine", "removed"];
var report = new StringBuilder();
report.AppendLine("| Assembly | Public API | Engine | Removed | Total | Nested (included) |");
report.AppendLine("|---|---:|---:|---:|---:|---:|");
foreach (string name in assemblies.Append("Total"))
{
    var mine = rows.Where(r => r.Rule is not null && (name == "Total" || r.Assembly == name)).ToList();
    report.AppendLine($"| {name.Replace("Kerf.Kernel.", "")} | " + string.Join(" | ", classes.Select(c => mine.Count(r => r.Rule!.Class == c).ToString("N0")))
        + $" | {mine.Count:N0} | {mine.Count(r => r.Chain.Count > 1):N0} |");
}

report.AppendLine();
report.AppendLine("| Area | Public API | Engine | Removed |");
report.AppendLine("|---|---:|---:|---:|");
foreach (var area in rows.Where(r => r.Rule is not null && r.Chain.Count == 1).GroupBy(r => r.Rule!.Area.Split('/')[0]).OrderBy(g => g.Key))
{
    report.AppendLine($"| {area.Key} | " + string.Join(" | ", classes.Select(c => area.Count(r => r.Rule!.Class == c).ToString("N0"))) + " |");
}

var engineByAssembly = rows.Where(r => r.Chain.Count == 1 && r.Rule is { Class: not "public" })
    .Select(r => (r.Assembly, r.Chain[0])).ToHashSet();
var crossEngine = crossRefs.Where(c => engineByAssembly.Contains((c.To, c.Type))).ToList();
report.AppendLine();
report.AppendLine($"Engine and removed types used from another kernel assembly: {crossEngine.Select(c => (c.To, c.Type)).Distinct().Count():N0} "
    + "(" + string.Join(", ", crossEngine.GroupBy(c => c.To).OrderBy(g => Array.IndexOf(assemblies, g.Key))
        .Select(g => $"{g.Select(c => c.Type).Distinct().Count():N0} of {g.Key.Replace("Kerf.Kernel.", "")}'s")) + "), "
    + $"and {crossRefs.Select(c => (c.To, c.Type)).Distinct().Count(t => !rows.Any(r => r.Assembly == t.To && r.Chain.Count == 1 && r.Chain[0] == t.Type)):N0} "
    + "internal types reached through InternalsVisibleTo.");

var used = rows.Where(r => r.Modeling.Length > 0).ToList();
report.AppendLine();
report.AppendLine($"Modelling layer's types: {used.Count} ({used.Count(r => r.Modeling == "named")} named): "
    + string.Join(", ", classes.Select(c => $"{used.Count(r => r.Rule!.Class == c)} {c}")) + "; "
    + $"{used.Count(r => r.Rule!.Class != "public" && r.Rule!.Replacement is null)} engine or removed without a stated replacement.");
Console.WriteLine(report);

if (args is ["--finish-map", string draftPath, string outPath])
{
    var removed = rows.Where(r => r.Chain.Count == 1 && r.Rule?.Class == "removed").Select(r => r.Namespace + "." + r.Chain[0]).ToHashSet();
    var output = new List<string>
    {
        "# DRAFT for the owner's review (unit 10D, 3 November 2026): kerf-rename's map for the kernel's four assemblies.",
        "# Made by `kerf-rename draft` with the seed docs/api/kerf-seed.map (from tools/api-classify.cs and",
        "# docs/api/classification-rules.txt), then `api-classify.cs --finish-map`. Types marked 'public API (10D)' take the",
        "# names of the API reviews in docs/api; 'engine (10D)' the engine's namespaces; 'draft' the drafter's systematic",
        "# names. Removed types (NCollection, TCollection…) have no entries: they keep their OCCT names until 10J.",
        "#",
    };
    int dropped = 0, recased = 0;
    foreach (string line in File.ReadLines(draftPath))
    {
        string[] f = line.Split(' ', 3);
        if (f.Length >= 3 && f[0] is "type" or "member" or "property" or "param" or "keep" && TypeOf(f[1]) is { } owner && removed.Contains(owner))
        {
            dropped++;
            continue;
        }

        if (f.Length >= 3 && f[0] == "member" && f[1].StartsWith("F:", StringComparison.Ordinal) && EnumCase(f[2].Split(' ')[0]) is { } cased)
        {
            output.Add($"member {f[1]} {cased}" + f[2][f[2].Split(' ')[0].Length..]);
            recased++;
            continue;
        }

        output.Add(line);
    }

    File.WriteAllLines(outPath, output);
    Console.WriteLine($"{outPath}: {dropped} entries of removed types dropped, {recased} enum members recased.");
    return 0;
}

if (checkOnly)
{
    return problems.Count == 0 ? 0 : 1;
}

var tsv = new StringBuilder();
tsv.AppendLine("assembly\ttype\tkind\tpackage\tclass\tarea\tnew_name\tmodeling\treplacement\tnote");
foreach (Row row in rows.Where(r => r.Rule is not null).OrderBy(r => r.Assembly, StringComparer.Ordinal)
             .ThenBy(r => string.Join("+", r.Chain), StringComparer.Ordinal))
{
    bool nested = row.Chain.Count > 1;
    tsv.AppendLine(string.Join('\t', row.Assembly.Replace("Kerf.Kernel.", ""), string.Join("+", row.Chain), row.Kind, row.Package,
        row.Rule!.Class, row.Rule.Area, row.NewName, row.Modeling, nested ? "" : row.Rule.Replacement ?? "",
        nested ? "" : row.Rule.Note ?? ""));
}

File.WriteAllText(Path.Combine(docs, "classification.tsv"), tsv.ToString());

// The seed for kerf-rename draft: a package line per package that has public or engine types (so that enum members
// lose their prefix), and a type line per top-level public or engine type. Removed types keep their OCCT names in the
// namespace Occt until 10J deletes them, so they get no line.
var seed = new StringBuilder();
seed.AppendLine("# kerf-rename seed, written by tools/api-classify.cs from docs/api/classification-rules.txt (unit 10D).");
seed.AppendLine("rule parameters on");
seed.AppendLine("rule fields on");
seed.AppendLine("rule statics on");
seed.AppendLine("rule locals on");
seed.AppendLine("rule methods on");
foreach (var package in rows.Where(r => r.Chain.Count == 1 && r.Rule is { Class: not "removed" })
             .GroupBy(r => r.Package).OrderBy(g => g.Key, StringComparer.Ordinal))
{
    // A package split between namespaces takes its engine's (or its only) namespace.
    var target = package.GroupBy(r => r.Rule!.Namespace(rules)).OrderByDescending(g => g.Count()).First().Key;
    seed.AppendLine($"package {package.Key} {target}");
}

foreach (Row row in rows.Where(r => r.Chain.Count == 1 && r.Rule is { Class: not "removed" })
             .OrderBy(r => "T:Occt." + r.Chain[0], StringComparer.Ordinal))
{
    string note = row.Rule!.Class == "public" ? "public API (10D)" : "engine (10D)";
    seed.AppendLine($"type T:{row.Namespace}.{row.Chain[0]} {row.NewName}  # {note}");
}

int seedAt = Array.IndexOf(args, "--seed-out");
string seedPath = seedAt >= 0 && seedAt + 1 < args.Length ? args[seedAt + 1] : Path.Combine(Path.GetTempPath(), "kerf-seed.map");
File.WriteAllText(seedPath, seed.ToString());
Console.WriteLine($"Wrote {rows.Count(r => r.Rule is not null):N0} rows to docs/api/classification.tsv and the seed to {seedPath}.");
return problems.Count == 0 ? 0 : 1;

// The type a map id belongs to: T:Occt.X -> Occt.X; M:Occt.X.Method(…) and F:Occt.X.Field -> Occt.X (top level only).
static string? TypeOf(string id)
{
    if (id.Length < 3 || id[1] != ':')
    {
        return null;
    }

    string body = id[2..];
    int paren = body.IndexOf('(');
    if (paren >= 0)
    {
        body = body[..paren];
    }

    int hash = body.IndexOf('#');
    if (hash >= 0)
    {
        body = body[..hash];
    }

    int tilde = body.IndexOf('~');
    if (tilde >= 0)
    {
        body = body[..tilde];
    }

    string[] parts = body.Split('.');
    return id[0] == 'T' ? string.Join('.', parts.Take(2)) : parts.Length >= 3 ? parts[0] + "." + parts[1] : null;
}

// FORWARD -> Forward, SAME_DOMAIN -> SameDomain; two-letter acronyms (CN, UV) and names with digits (C0) stay.
static string? EnumCase(string name)
{
    if (!name.All(c => char.IsUpper(c) || char.IsDigit(c) || c == '_') || name.Count(char.IsLetter) < 2)
    {
        return null;
    }

    if (name.Length == 2 && name is not ("IN" or "ON" or "NO" or "UP" or "TO" or "OR"))
    {
        return null;
    }

    if (name.Any(char.IsDigit) && !name.Contains('_'))
    {
        return null;
    }

    if (name == "COMPSOLID")
    {
        return "CompSolid";
    }

    string cased = string.Concat(name.Split('_', StringSplitOptions.RemoveEmptyEntries)
        .Select(w => w.Length == 0 ? w : w[0] + w[1..].ToLowerInvariant()));
    return cased == name || cased.Length == 0 || char.IsDigit(cased[0]) ? null : cased;
}

static bool IsVisible(MetadataReader md, TypeDefinition type)
{
    TypeAttributes visibility = type.Attributes & TypeAttributes.VisibilityMask;
    if (visibility == TypeAttributes.Public)
    {
        return true;
    }

    if (visibility is not (TypeAttributes.NestedPublic or TypeAttributes.NestedFamily or TypeAttributes.NestedFamORAssem))
    {
        return false;
    }

    return IsVisible(md, md.GetTypeDefinition(type.GetDeclaringType()));
}

static string Kind(MetadataReader md, TypeDefinition type)
{
    if ((type.Attributes & TypeAttributes.Interface) != 0)
    {
        return "interface";
    }

    string baseName = type.BaseType.Kind == HandleKind.TypeReference
        ? md.GetString(md.GetTypeReference((TypeReferenceHandle)type.BaseType).Name) : "";
    return baseName switch
    {
        "Enum" => "enum",
        "ValueType" => "struct",
        "MulticastDelegate" => "delegate",
        _ => (type.Attributes & (TypeAttributes.Abstract | TypeAttributes.Sealed)) == (TypeAttributes.Abstract | TypeAttributes.Sealed)
            ? "static class" : "class",
    };
}

static string Simple(string fullName) => fullName[(fullName.LastIndexOf('.') + 1)..];

static string Plain(string metaName) => metaName.IndexOf('`') is var tick and > 0 ? metaName[..tick] : metaName;

static string Arity(string metaName) => metaName.IndexOf('`') is var tick and > 0 ? metaName[tick..] : "";

static string Package(string name) => name.IndexOf('_') is var underscore and > 0 ? name[..underscore] : name;

// The type's name without its package prefix, in .NET's style: underscores go, and acronyms of three letters or more
// become words (BFGS -> Bfgs, XYZ -> Xyz); the package's own class takes the package's name.
static string Stem(string name, string package)
{
    string rest = name == package ? package : name[(package.Length + 1)..];
    var result = new StringBuilder();
    foreach (string part in rest.Split('_', StringSplitOptions.RemoveEmptyEntries))
    {
        int i = 0;
        while (i < part.Length)
        {
            int j = i;
            while (j < part.Length && char.IsUpper(part[j]))
            {
                j++;
            }

            // A run of capitals followed by a lower-case letter leaves its last capital to the next word.
            int end = j < part.Length && char.IsLower(part[j]) && j - i > 1 ? j - 1 : j;
            if (end - i >= 3)
            {
                result.Append(part[i]).Append(part[(i + 1)..end].ToLowerInvariant());
                i = end;
            }
            else if (end > i)
            {
                result.Append(part[i..end]);
                i = end;
            }

            int k = i;
            while (k < part.Length && !char.IsUpper(part[k]))
            {
                k++;
            }

            if (k > i)
            {
                string word = part[i..k];
                result.Append(result.Length == 0 ? char.ToUpperInvariant(word[0]) + word[1..] : word);
                i = k;
            }
        }
    }

    return result.ToString();
}

static string? FindRoot(string start)
{
    for (var dir = new DirectoryInfo(start); dir is not null; dir = dir.Parent)
    {
        if (File.Exists(Path.Combine(dir.FullName, "Kerf.slnx")))
        {
            return dir.FullName;
        }
    }

    return null;
}

sealed class Row(string assembly, string ns, List<string> chain, string kind)
{
    public string Assembly { get; } = assembly;
    public string Namespace { get; } = ns;
    public List<string> Chain { get; } = chain;
    public string Kind { get; } = kind;
    public string Package { get; set; } = "";
    public Rule? Rule { get; set; }
    public string NewName { get; set; } = "";
    public string Modeling { get; set; } = "";
}

/// <summary>A line of the rules: a package's or a type's class, area and, for a type, its name and replacement.</summary>
sealed record Rule(string Kind, string Subject, string Class, string Area, string? Name, string? Replacement, string? Note)
{
    public bool Used { get; set; }

    /// <summary>What a package's types take before and after their stem, where two packages share a namespace.</summary>
    public string Prefix { get; init; } = "";
    public string Suffix { get; init; } = "";

    /// <summary>The namespace: an area's public one for public types, its engine one (plus any sub-namespace) otherwise.</summary>
    public string Namespace(Rules rules)
    {
        string[] parts = Area.Split('/');
        (string publicNs, string engineNs) = rules.Areas[parts[0]];
        if (Class == "removed")
        {
            return "Occt";
        }

        string ns = Class == "public" ? publicNs : engineNs;
        return parts.Length > 1 ? ns + "." + string.Join('.', parts[1..]) : ns;
    }
}

sealed class Rules
{
    public Dictionary<string, (string Public, string Engine)> Areas { get; } = [];
    public Dictionary<string, Rule> Packages { get; } = [];
    public Dictionary<string, Rule> Types { get; } = [];
    public HashSet<string> Allowed { get; } = [];

    // area <Area> <public namespace or -> <engine namespace>
    // package <Package> public|engine|removed <Area>[/<Sub>/<Namespace>] [pre:<Prefix>] [suf:<Suffix>]  # note
    // type <Type> public|engine|removed <Area>[/<Sub>] [<NewName>] [=> <replacement>]  # note
    // allow <Name>   (a public name that is also a .NET type's, accepted on purpose)
    public static Rules Load(string path)
    {
        var rules = new Rules();
        foreach (string raw in File.ReadLines(path))
        {
            string line = raw;
            string? note = null;
            int hash = line.IndexOf('#');
            if (hash >= 0)
            {
                note = line[(hash + 1)..].Trim();
                line = line[..hash];
            }

            string? replacement = null;
            int arrow = line.IndexOf("=>", StringComparison.Ordinal);
            if (arrow >= 0)
            {
                replacement = line[(arrow + 2)..].Trim();
                line = line[..arrow];
            }

            string[] f = line.Split((char[])[' ', '\t'], StringSplitOptions.RemoveEmptyEntries);
            if (f.Length == 0)
            {
                continue;
            }

            switch (f[0])
            {
                case "area":
                    rules.Areas[f[1]] = (f[2], f[3]);
                    break;
                case "allow":
                    rules.Allowed.Add(f[1]);
                    break;
                case "package":
                    rules.Packages[f[1]] = new Rule("package", f[1], f[2], f[3], null, replacement, note)
                    {
                        Prefix = f.Skip(4).FirstOrDefault(t => t.StartsWith("pre:", StringComparison.Ordinal))?[4..] ?? "",
                        Suffix = f.Skip(4).FirstOrDefault(t => t.StartsWith("suf:", StringComparison.Ordinal))?[4..] ?? "",
                    };
                    break;
                case "type":
                    rules.Types[f[1]] = new Rule("type", f[1], f[2], f[3], f.Length > 4 ? f[4] : null, replacement, note);
                    break;
                default:
                    throw new InvalidDataException($"{path}: unknown line '{raw}'");
            }
        }

        foreach (Rule rule in rules.Packages.Values.Concat(rules.Types.Values))
        {
            if (!rules.Areas.ContainsKey(rule.Area.Split('/')[0]) || rule.Class is not ("public" or "engine" or "removed"))
            {
                throw new InvalidDataException($"{path}: bad rule for {rule.Subject}: {rule.Class} {rule.Area}");
            }
        }

        return rules;
    }

    public Rule? Classify(string type, string package)
    {
        if (Types.TryGetValue(type, out Rule? rule) || Packages.TryGetValue(package, out rule))
        {
            rule.Used = true;
            if (rule.Kind == "type" && Packages.TryGetValue(package, out Rule? p))
            {
                p.Used = true;
            }

            return rule;
        }

        return null;
    }

    public IEnumerable<string> Unused() => Packages.Values.Concat(Types.Values).Where(r => !r.Used).Select(r => $"{r.Kind} {r.Subject}");
}
