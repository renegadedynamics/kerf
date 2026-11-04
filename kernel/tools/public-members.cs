// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Which public members no test calls (phase10-plan.md 3.2, question 49: every public member tested): the public API of
// the kernel's assemblies (every exported type outside Kerf.Engine) and of Kerf.Modeling, read by reflection, against
// the Cobertura files of a coverage run (tools/coverage.ps1). A member counts as called if any line of it ran in any
// test run, directly or through another member. Writes the list to the report and prints the counts.
//
//   dotnet run tools/public-members.cs -- <results folder> <assemblies folder> <report.md>
//
// The assemblies folder is a test project's output that holds all five (Kerf.Modeling.Tests' does). Not measured, and
// counted apart: abstract members and interfaces' (no body; their implementations are measured), fields, and what the
// compiler wrote where coverage has no line for it (an auto-property's accessors, a record's members).

#:property PublishAot=false

using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.Loader;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml;

if (args.Length < 3)
{
    Console.Error.WriteLine("usage: dotnet run tools/public-members.cs -- <results folder> <assemblies folder> <report.md>");
    return 1;
}
string results = Path.GetFullPath(args[0]);
string folder = Path.GetFullPath(args[1]);
string report = Path.GetFullPath(args[2]);
string[] assemblies = ["Kerf.Kernel.Foundation", "Kerf.Kernel.ModelingData", "Kerf.Kernel.ModelingAlgorithms", "Kerf.Kernel.DataExchange", "Kerf.Modeling"];

// class -> method -> (signature -> ran), and which classes hold the state machines of iterators and async methods
var coverage = new Dictionary<string, Dictionary<string, Dictionary<string, bool>>>(StringComparer.Ordinal);
foreach (string file in Directory.GetFiles(results, "*.cobertura.xml", SearchOption.AllDirectories))
{
    using var reader = XmlReader.Create(file, new XmlReaderSettings { DtdProcessing = DtdProcessing.Ignore });
    string? package = null;
    string? className = null;
    while (reader.Read())
    {
        if (reader.NodeType != XmlNodeType.Element)
        {
            continue;
        }
        if (reader.Name == "package")
        {
            package = reader.GetAttribute("name");
        }
        else if (reader.Name == "class" && package is not null && assemblies.Contains(package))
        {
            className = StripGenerics(reader.GetAttribute("name")!);
        }
        else if (reader.Name == "class")
        {
            className = null;
        }
        else if (reader.Name == "method" && className is not null)
        {
            string name = reader.GetAttribute("name")!;
            string signature = NormaliseSignature(reader.GetAttribute("signature") ?? "()");
            bool ran = double.Parse(reader.GetAttribute("line-rate") ?? "0", System.Globalization.CultureInfo.InvariantCulture) > 0;
            if (!coverage.TryGetValue(className, out var methods))
            {
                coverage[className] = methods = new(StringComparer.Ordinal);
            }
            if (!methods.TryGetValue(name, out var overloads))
            {
                methods[name] = overloads = new(StringComparer.Ordinal);
            }
            overloads[signature] = overloads.GetValueOrDefault(signature) || ran;
        }
    }
}

AssemblyLoadContext.Default.Resolving += (context, name) =>
{
    string path = Path.Combine(folder, name.Name + ".dll");
    return File.Exists(path) ? context.LoadFromAssemblyPath(path) : null;
};

const BindingFlags Declared = BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
var rows = new List<Row>();
int abstractMembers = 0, fields = 0, generated = 0;
foreach (string name in assemblies)
{
    Assembly assembly = AssemblyLoadContext.Default.LoadFromAssemblyPath(Path.Combine(folder, name + ".dll"));
    foreach (Type type in assembly.GetExportedTypes())
    {
        if (!IsPublicApi(type) || type.IsEnum || typeof(Delegate).IsAssignableFrom(type))
        {
            continue;
        }
        fields += type.GetFields(Declared).Length;
        string className = StripGenerics(type.FullName!.Replace('+', '.'));
        coverage.TryGetValue(className, out var methods);
        var members = type.GetMethods(Declared).Cast<MethodBase>().Concat(type.GetConstructors(BindingFlags.Public | BindingFlags.Instance));
        foreach (MethodBase member in members)
        {
            if (member.IsAbstract)
            {
                abstractMembers++;
                continue;
            }
            bool isGenerated = member.IsDefined(typeof(CompilerGeneratedAttribute), false);
            bool? ran = Ran(member, methods, className, coverage);
            if (ran is null && (isGenerated || member.IsConstructor && member.GetParameters().Length == 0 || IsRecordMember(type, member)))
            {
                generated++;
                continue;
            }
            rows.Add(new Row(name, type.Namespace ?? "", TypeName(type), Describe(member), ran));
        }
    }
}

var text = new StringBuilder();
text.Append("""
    # Public members no test calls

    Written by `tools/public-members.cs` from the coverage run of `tools/coverage.ps1` (phase10-plan.md 3.2, question 49:
    every public member tested). The public API is every exported type outside `Kerf.Engine` in the kernel's assemblies,
    and `Kerf.Modeling`. A member counts as called if a line of it ran in any test run, directly or through another
    member; "no data" means coverage has no entry for it. Abstract members, fields and compiler-written members without
    lines are not measured, and are counted below.


    """);
int total = rows.Count, called = rows.Count(r => r.Ran == true), notCalled = rows.Count(r => r.Ran == false), noData = rows.Count(r => r.Ran is null);
text.Append($"Measured: {total:N0} members; called {called:N0}; not called {notCalled:N0}; no data {noData:N0}. ");
text.Append($"Not measured: {abstractMembers:N0} abstract, {fields:N0} fields, {generated:N0} compiler-written.\n\n");
text.Append("| Assembly | Members | Called | Not called | No data |\n|---|---:|---:|---:|---:|\n");
foreach (var group in rows.GroupBy(r => r.Assembly))
{
    text.Append($"| {group.Key} | {group.Count():N0} | {group.Count(r => r.Ran == true):N0} | {group.Count(r => r.Ran == false):N0} | {group.Count(r => r.Ran is null):N0} |\n");
}
foreach (var byType in rows.Where(r => r.Ran != true).GroupBy(r => $"{r.Namespace}.{r.Type}").OrderBy(g => g.Key, StringComparer.Ordinal))
{
    text.Append($"\n## `{byType.Key}`\n\n");
    foreach (Row row in byType.OrderBy(r => r.Member, StringComparer.Ordinal))
    {
        text.Append($"- `{row.Member}`{(row.Ran is null ? " (no data)" : "")}\n");
    }
}
Directory.CreateDirectory(Path.GetDirectoryName(report)!);
File.WriteAllText(report, text.ToString().Replace("\r\n", "\n"));
Console.WriteLine($"{total:N0} public members: {called:N0} called, {notCalled:N0} not called, {noData:N0} no data. Written to {report}");
return 0;

// Everything exported but the engine and the kernel's assembly markers
static bool IsPublicApi(Type type)
{
    string ns = type.Namespace ?? "";
    return !ns.StartsWith("Kerf.Engine", StringComparison.Ordinal) && !ns.StartsWith("Kerf.Kernel", StringComparison.Ordinal) && ns != "Occt";
}

static bool IsRecordMember(Type type, MethodBase member) =>
    type.GetMethod("<Clone>$") is not null
    && member.Name is "ToString" or "GetHashCode" or "Equals" or "op_Equality" or "op_Inequality" or "PrintMembers" or "Deconstruct" or "get_EqualityContract" or "<Clone>$";

// null: no entry for it in the coverage data. An iterator's or async method's body is in its state machine's class.
static bool? Ran(MethodBase member, Dictionary<string, Dictionary<string, bool>>? methods, string className,
                 Dictionary<string, Dictionary<string, Dictionary<string, bool>>> coverage)
{
    if (methods is not null && methods.TryGetValue(member.Name, out var overloads))
    {
        string signature = NormaliseSignature(Signature(member));
        if (overloads.TryGetValue(signature, out bool exact))
        {
            return exact;
        }
        // types spelt differently (tuples, nullables): by the number of parameters, or by name alone
        int count = member.GetParameters().Length;
        var sameCount = overloads.Where(o => ParameterCount(o.Key) == count).ToList();
        if (sameCount.Count > 0)
        {
            return sameCount.Any(o => o.Value);
        }
    }
    string prefix = $"{className}.<{member.Name}>";
    var machines = coverage.Where(c => c.Key.StartsWith(prefix, StringComparison.Ordinal)).ToList();
    return machines.Count > 0 ? machines.Any(c => c.Value.Values.Any(o => o.Values.Any(v => v))) : null;
}

static string TypeName(Type type) => StripGenerics(type.FullName!.Replace('+', '.'))[((type.Namespace?.Length + 1) ?? 0)..];

static string Describe(MethodBase member) => $"{member.Name}{Signature(member)}";

static string Signature(MethodBase member) =>
    "(" + string.Join(", ", member.GetParameters().Select(p => Spell(p.ParameterType))) + ")";

static string Spell(Type type)
{
    if (type.IsByRef)
    {
        return Spell(type.GetElementType()!);
    }
    if (type.IsArray)
    {
        return Spell(type.GetElementType()!) + "[]";
    }
    if (type.IsPointer)
    {
        return Spell(type.GetElementType()!) + "*";
    }
    if (Nullable.GetUnderlyingType(type) is { } underlying)
    {
        return Spell(underlying) + "?";
    }
    if (type == typeof(double)) return "double";
    if (type == typeof(int)) return "int";
    if (type == typeof(bool)) return "bool";
    if (type == typeof(string)) return "string";
    if (type == typeof(object)) return "object";
    if (type == typeof(long)) return "long";
    if (type == typeof(float)) return "float";
    if (type == typeof(byte)) return "byte";
    if (type == typeof(char)) return "char";
    if (type == typeof(short)) return "short";
    if (type == typeof(uint)) return "uint";
    if (type == typeof(ulong)) return "ulong";
    if (type == typeof(ushort)) return "ushort";
    if (type == typeof(sbyte)) return "sbyte";
    string name = type.Name;
    int tick = name.IndexOf('`');
    if (tick >= 0)
    {
        name = name[..tick];
    }
    if (type.IsGenericType)
    {
        return $"{name}<{string.Join(", ", type.GetGenericArguments().Select(Spell))}>";
    }
    return name;
}

// A signature's parameter types without namespaces, by-ref keywords or a method's generic parameters
static string NormaliseSignature(string signature)
{
    int open = signature.IndexOf('(');
    string text = open >= 0 ? signature[open..] : signature;
    text = Regex.Replace(text, @"\b(ref|out|in|params)\s+", "");
    text = Regex.Replace(text, @"(?:[A-Za-z_][\w]*\.)+([A-Za-z_]\w*)", "$1");
    return text.Replace(" ", "");
}

static int ParameterCount(string signature)
{
    if (signature == "()")
    {
        return 0;
    }
    int depth = 0, count = 1;
    foreach (char c in signature)
    {
        if (c is '<' or '(' or '[')
        {
            depth++;
        }
        else if (c is '>' or ')' or ']')
        {
            depth--;
        }
        else if (c == ',' && depth == 1)
        {
            count++;
        }
    }
    return count;
}

// A class's name without generic parameters, keeping a compiler's <Member>d__1 parts
static string StripGenerics(string name)
{
    var builder = new StringBuilder();
    int depth = 0;
    bool keep = false;
    foreach (char c in name)
    {
        if (c == '<')
        {
            if (depth++ == 0)
            {
                keep = builder.Length > 0 && builder[^1] == '.';
            }
        }
        if (depth == 0 || keep)
        {
            builder.Append(c);
        }
        if (c == '>')
        {
            depth--;
        }
    }
    return Regex.Replace(builder.ToString(), @"`\d+", "");
}

internal sealed record Row(string Assembly, string Namespace, string Type, string Member, bool? Ran);
