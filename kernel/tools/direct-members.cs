// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Which public members no test calls directly (unit 11J; tools/public-members.cs asks the looser question, whether any
// line of a member ran). Reads the IL of the built test assemblies: a member is called directly when a test assembly's
// own code names it. It is checked as well when the test that names it, or a test that reaches the helper that names
// it, also reaches an assertion: a filter against tests that only call, not a proof that this member's result was the
// thing asserted. Needs no test run, only a Release build.
//
//   dotnet run tools/direct-members.cs -- <repository root> <report.md>
//
// The public API is as tools/public-members.cs reads it. As there, abstract members, fields and what the compiler wrote
// for a record are not measured, nor is an implicit parameterless constructor. An auto-property's accessors are measured
// here, as a call names them whether or not coverage has a line for them. C# names the least derived declaration of a
// virtual member, so an override counts when its base's declaration is named; the IL can't say which override a test
// meant. For the same reason a class's overrides of object's ToString, Equals and GetHashCode can't be seen, and are
// counted apart. A value type's are seen.

#:property PublishAot=false

using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using System.Runtime.Loader;
using System.Text;

if (args.Length < 2)
{
    Console.Error.WriteLine("usage: dotnet run tools/direct-members.cs -- <repository root> <report.md>");
    return 1;
}
string root = Path.GetFullPath(args[0]);
string report = Path.GetFullPath(args[1]);
string[] products = ["Kerf.Kernel.Foundation", "Kerf.Kernel.ModelingData", "Kerf.Kernel.ModelingAlgorithms", "Kerf.Kernel.DataExchange", "Kerf.Modeling"];
string[] testAssemblies =
[
    "kernel/tests/Kerf.Kernel.DataExchange.Tests/bin/Release/net10.0/Kerf.Kernel.DataExchange.Tests.dll",
    "kernel/tests/Kerf.Kernel.DrawLite.Tests/bin/Release/net10.0/Kerf.Kernel.DrawLite.Tests.dll",
    "kernel/tests/Kerf.Kernel.Foundation.Tests/bin/Release/net10.0/Kerf.Kernel.Foundation.Tests.dll",
    "kernel/tests/Kerf.Kernel.ModelingAlgorithms.Tests/bin/Release/net10.0/Kerf.Kernel.ModelingAlgorithms.Tests.dll",
    "kernel/tests/Kerf.Kernel.ModelingData.Tests/bin/Release/net10.0/Kerf.Kernel.ModelingData.Tests.dll",
    "kernel/tests/Kerf.Kernel.Oracle.Tests/bin/Release/net10.0/Kerf.Kernel.Oracle.Tests.dll",
    "kernel/tests/Kerf.Kernel.PublicApi.Tests/bin/Release/net10.0/Kerf.Kernel.PublicApi.Tests.dll",
    "modeling/tests/Kerf.Modeling.Tests/bin/Release/net10.0/Kerf.Modeling.Tests.dll",
];

var opCodes = typeof(OpCodes).GetFields(BindingFlags.Public | BindingFlags.Static)
    .Select(f => (OpCode)f.GetValue(null)!)
    .ToDictionary(o => (ushort)o.Value);

const BindingFlags Declared = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;

// (assembly, metadata token) of what test code names, and of what it names from a test that reaches an assertion
var named = new HashSet<(string, int)>();
var checkedNames = new HashSet<(string, int)>();
int testMethods = 0;
foreach (string relative in testAssemblies)
{
    string path = Path.Combine(root, relative);
    if (!File.Exists(path))
    {
        Console.Error.WriteLine($"Not built: {path}");
        return 1;
    }
    string folder = Path.GetDirectoryName(path)!;
    var context = new AssemblyLoadContext(relative);
    context.Resolving += (c, name) =>
    {
        string file = Path.Combine(folder, name.Name + ".dll");
        return File.Exists(file) ? c.LoadFromAssemblyPath(file) : null;
    };
    Assembly tests = context.LoadFromAssemblyPath(path);

    // Each piece of code is filed under the method it was written in: a closure or an iterator's body under its method
    var units = new Dictionary<string, Unit>(StringComparer.Ordinal);
    foreach (Type type in tests.GetTypes())
    {
        string owner = OwnerType(type);
        foreach (MethodBase method in type.GetMethods(Declared).Cast<MethodBase>().Concat(type.GetConstructors(Declared)))
        {
            MethodBody? body = method.GetMethodBody();
            if (body is null)
            {
                continue;
            }
            string key = owner + "::" + OwnerMethod(type, method);
            if (!units.TryGetValue(key, out Unit? unit))
            {
                units[key] = unit = new Unit();
            }
            unit.IsTest |= method.GetCustomAttributesData().Any(a => a.AttributeType.Name is "FactAttribute" or "TheoryAttribute"
                || a.AttributeType.BaseType?.Name is "FactAttribute" or "TheoryAttribute");
            Scan(body.GetILAsByteArray()!, type, method, unit, tests, opCodes);
        }
    }
    testMethods += units.Values.Count(u => u.IsTest);

    // A helper is reached from the tests that call it; a test checks if anything it reaches asserts
    foreach (Unit unit in units.Values)
    {
        unit.Calls = unit.CallKeys.Select(k => units.GetValueOrDefault(k)).OfType<Unit>().ToList();
    }
    bool changed = true;
    while (changed)
    {
        changed = false;
        foreach (Unit unit in units.Values)
        {
            if (!unit.Asserts && unit.Calls.Any(c => c.Asserts))
            {
                unit.Asserts = true;
                changed = true;
            }
        }
    }
    // Everything an asserting unit reaches is checked: its helpers build what it asserts on
    var reached = new HashSet<Unit>();
    var stack = new Stack<Unit>(units.Values.Where(u => u.Asserts));
    while (stack.Count > 0)
    {
        Unit unit = stack.Pop();
        if (reached.Add(unit))
        {
            foreach (Unit call in unit.Calls)
            {
                stack.Push(call);
            }
        }
    }
    foreach (Unit unit in units.Values)
    {
        named.UnionWith(unit.Named);
        if (reached.Contains(unit))
        {
            checkedNames.UnionWith(unit.Named);
        }
    }
}

// The public API, from one test project's output, which holds all five
string productFolder = Path.GetDirectoryName(Path.Combine(root, testAssemblies[^1]))!;
var productContext = new AssemblyLoadContext("products");
productContext.Resolving += (c, name) =>
{
    string file = Path.Combine(productFolder, name.Name + ".dll");
    return File.Exists(file) ? c.LoadFromAssemblyPath(file) : null;
};

const BindingFlags Public = BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
var rows = new List<Row>();
int abstractMembers = 0, generated = 0, objectOverrides = 0;
foreach (string name in products)
{
    Assembly assembly = productContext.LoadFromAssemblyPath(Path.Combine(productFolder, name + ".dll"));
    foreach (Type type in assembly.GetExportedTypes())
    {
        if (!IsPublicApi(type) || type.IsEnum || typeof(Delegate).IsAssignableFrom(type) || type.FullName!.Contains('<'))
        {
            continue;
        }
        bool isRecord = type.GetMethod("<Clone>$") is not null || type.GetMethod("PrintMembers", BindingFlags.NonPublic | BindingFlags.Instance) is not null
            || type.GetProperty("EqualityContract", BindingFlags.NonPublic | BindingFlags.Instance) is not null
            || (type.IsValueType && type.GetMethod("PrintMembers", BindingFlags.NonPublic | BindingFlags.Instance) is not null);
        var members = type.GetMethods(Public).Cast<MethodBase>().Concat(type.GetConstructors(BindingFlags.Public | BindingFlags.Instance));
        foreach (MethodBase member in members)
        {
            if (member.IsAbstract)
            {
                abstractMembers++;
                continue;
            }
            if (isRecord && member.Name is "ToString" or "GetHashCode" or "Equals" or "op_Equality" or "op_Inequality" or "Deconstruct" or "get_EqualityContract" or "<Clone>$"
                && member.IsDefined(typeof(CompilerGeneratedAttribute), false))
            {
                generated++;
                continue;
            }
            if (member.IsDefined(typeof(CompilerGeneratedAttribute), false) && member.Name.StartsWith("set_", StringComparison.Ordinal)
                && type.GetConstructors().Any(c => c.GetParameters().Any(p => p.Name == member.Name[4..])))
            {
                generated++;
                continue;
            }
            if (member.IsConstructor && member.GetParameters().Length == 0 && IsImplicit(type))
            {
                generated++;
                continue;
            }
            if (member is MethodInfo info && !type.IsValueType && info.IsVirtual && info.GetBaseDefinition().DeclaringType == typeof(object))
            {
                objectOverrides++;
                continue;
            }
            bool direct = false, isChecked = false;
            foreach (MethodBase declaration in Declarations(member))
            {
                var key = (declaration.Module.Assembly.GetName().Name!, declaration.MetadataToken);
                direct |= named.Contains(key);
                isChecked |= checkedNames.Contains(key);
            }
            rows.Add(new Row(name, type.Namespace ?? "", TypeName(type), Describe(member), direct, isChecked));
        }
    }
}

var text = new StringBuilder();
text.Append("""
    # Public members no test calls directly

    Written by `tools/direct-members.cs` from the IL of the Release test assemblies (unit 11J). `public-members.md`
    counts a member as called if any line of it ran, through any other member; this counts it only if a test assembly's
    own code names it. "Checked" means that the test naming it, or a test reaching the helper naming it, also reaches
    an assertion: a filter against tests that only call, not a proof that this member's result is what was asserted.
    An override counts when its base's declaration is named, since C# names the least derived one. The Draw-lite
    harness isn't a test assembly, so what only its commands name doesn't count.


    """);
int total = rows.Count, direct2 = rows.Count(r => r.Direct), checked2 = rows.Count(r => r.Checked);
text.Append($"Measured: {total:N0} members; called directly {direct2:N0}; of them checked {checked2:N0}; no direct test {total - direct2:N0}. ");
text.Append($"From {testMethods:N0} test methods. Not measured: {abstractMembers:N0} abstract, {generated:N0} compiler-written, {objectOverrides:N0} overrides of `object`'s members on classes (the IL can't show them).\n\n");
text.Append("| Assembly | Members | Direct | Checked | No direct test |\n|---|---:|---:|---:|---:|\n");
foreach (var group in rows.GroupBy(r => r.Assembly))
{
    text.Append($"| {group.Key} | {group.Count():N0} | {group.Count(r => r.Direct):N0} | {group.Count(r => r.Checked):N0} | {group.Count(r => !r.Direct):N0} |\n");
}
text.Append("\n| Namespace | Members | Direct | Checked | No direct test |\n|---|---:|---:|---:|---:|\n");
foreach (var group in rows.GroupBy(r => r.Namespace).OrderBy(g => g.Key, StringComparer.Ordinal))
{
    text.Append($"| `{group.Key}` | {group.Count():N0} | {group.Count(r => r.Direct):N0} | {group.Count(r => r.Checked):N0} | {group.Count(r => !r.Direct):N0} |\n");
}
foreach (var byType in rows.Where(r => !r.Checked).GroupBy(r => $"{r.Namespace}.{r.Type}").OrderBy(g => g.Key, StringComparer.Ordinal))
{
    text.Append($"\n## `{byType.Key}`\n\n");
    foreach (Row row in byType.OrderBy(r => r.Member, StringComparer.Ordinal))
    {
        text.Append($"- `{row.Member}`{(row.Direct ? " (called, no assertion reached)" : "")}\n");
    }
}
Directory.CreateDirectory(Path.GetDirectoryName(report)!);
File.WriteAllText(report, text.ToString().Replace("\r\n", "\n"));
Console.WriteLine($"{total:N0} public members: {direct2:N0} called directly, {checked2:N0} checked, {total - direct2:N0} with no direct test. Written to {report}");
return 0;

// Walks a method's IL for what it names
static void Scan(byte[] il, Type type, MethodBase method, Unit unit, Assembly tests, Dictionary<ushort, OpCode> opCodes)
{
    Type[]? typeArguments = type.IsGenericType ? type.GetGenericArguments() : null;
    Type[]? methodArguments = method.IsGenericMethod ? method.GetGenericArguments() : null;
    Module module = type.Module;
    int i = 0;
    Type? constrained = null;
    while (i < il.Length)
    {
        ushort value = il[i++];
        if (value == 0xFE)
        {
            value = (ushort)(0xFE00 | il[i++]);
        }
        OpCode op = opCodes[value];
        switch (op.OperandType)
        {
            case OperandType.InlineNone:
                break;
            case OperandType.ShortInlineBrTarget:
            case OperandType.ShortInlineI:
            case OperandType.ShortInlineVar:
                i += 1;
                break;
            case OperandType.InlineVar:
                i += 2;
                break;
            case OperandType.InlineI8:
            case OperandType.InlineR:
                i += 8;
                break;
            case OperandType.InlineSwitch:
                int count = BitConverter.ToInt32(il, i);
                i += 4 + 4 * count;
                break;
            case OperandType.InlineMethod:
            case OperandType.InlineTok:
                int token = BitConverter.ToInt32(il, i);
                i += 4;
                MemberInfo? member = null;
                try
                {
                    member = module.ResolveMember(token, typeArguments, methodArguments);
                }
                catch (Exception e) when (e is ArgumentException or TypeLoadException or BadImageFormatException or MissingMemberException or FileNotFoundException)
                {
                }
                if (member is MethodBase called)
                {
                    Note(called, constrained, unit, tests);
                }
                constrained = null;
                break;
            case OperandType.InlineType:
                int typeToken = BitConverter.ToInt32(il, i);
                i += 4;
                if (op == OpCodes.Constrained)
                {
                    try
                    {
                        constrained = module.ResolveType(typeToken, typeArguments, methodArguments);
                    }
                    catch (Exception e) when (e is ArgumentException or TypeLoadException or BadImageFormatException)
                    {
                        constrained = null;
                    }
                }
                break;
            default:
                i += 4;
                break;
        }
    }
}

static void Note(MethodBase called, Type? constrained, Unit unit, Assembly tests)
{
    Type? declaring = called.DeclaringType;
    if (declaring is null)
    {
        return;
    }
    string assembly = called.Module.Assembly.GetName().Name!;
    if (declaring.Assembly == tests)
    {
        unit.CallKeys.Add(OwnerType(declaring) + "::" + OwnerMethod(declaring, called));
        return;
    }
    if (declaring.Name is "Assert" or "XunitException" or "AssertEx" || declaring.FullName?.StartsWith("Xunit.Sdk.", StringComparison.Ordinal) == true)
    {
        unit.Asserts = true;
        return;
    }
    // a value type's own override, named through object's or an interface's declaration
    if (constrained is not null && constrained.IsValueType && !called.IsConstructor)
    {
        const BindingFlags All = BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly;
        Type[] parameters = called.GetParameters().Select(p => p.ParameterType).ToArray();
        MethodBase? own = constrained.GetMethod(called.Name, All, parameters);
        if (own is not null)
        {
            unit.Named.Add((own.Module.Assembly.GetName().Name!, own.MetadataToken));
        }
    }
    unit.Named.Add((assembly, called.MetadataToken));
}

// The declarations a call could have named for this member: its own, the virtual it overrides, and the interface
// members it implements
static IEnumerable<MethodBase> Declarations(MethodBase member)
{
    yield return member;
    if (member is not MethodInfo info || !info.IsVirtual)
    {
        yield break;
    }
    MethodInfo current = info;
    for (MethodInfo basis = current.GetBaseDefinition(); basis != current; basis = current.GetBaseDefinition())
    {
        yield return basis;
        current = basis;
    }
    Type declaring = info.DeclaringType!;
    if (declaring.IsInterface || declaring.ContainsGenericParameters)
    {
        yield break;
    }
    foreach (Type face in declaring.GetInterfaces())
    {
        InterfaceMapping map;
        try
        {
            map = declaring.GetInterfaceMap(face);
        }
        catch (ArgumentException)
        {
            continue;
        }
        for (int i = 0; i < map.TargetMethods.Length; i++)
        {
            if (map.TargetMethods[i] == info)
            {
                yield return map.InterfaceMethods[i];
            }
        }
    }
}

// The type code was written in: a closure's or state machine's class counts as the class around it
static string OwnerType(Type type)
{
    while (type.IsNested && type.Name.StartsWith('<'))
    {
        type = type.DeclaringType!;
    }
    return type.IsGenericType ? type.GetGenericTypeDefinition().FullName! : type.FullName!;
}

// The method code was written in: <Name>b__1_0, <Name>g__Local|1_0 and <Name>d__3 all belong to Name; others to themselves
static string OwnerMethod(Type type, MethodBase method)
{
    for (Type? t = type; t is not null && t.IsNested && t.Name.StartsWith('<'); t = t.DeclaringType)
    {
        if (t.Name.StartsWith("<>", StringComparison.Ordinal))
        {
            continue;
        }
        return t.Name[1..t.Name.IndexOf('>')];
    }
    if (method.Name.StartsWith('<'))
    {
        int close = method.Name.IndexOf('>');
        if (close > 1)
        {
            return method.Name[1..close];
        }
    }
    return method.Name;
}

static bool IsPublicApi(Type type)
{
    string ns = type.Namespace ?? "";
    return !ns.StartsWith("Kerf.Engine", StringComparison.Ordinal) && !ns.StartsWith("Kerf.Kernel", StringComparison.Ordinal) && ns != "Occt";
}

// A parameterless constructor the source didn't write has no line of its own: read here by there being no other
static bool IsImplicit(Type type) => type.GetConstructors(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance).Length == 1;

static string TypeName(Type type)
{
    string full = type.FullName!.Replace('+', '.');
    return StripGenerics(full)[((type.Namespace?.Length + 1) ?? 0)..];
}

static string StripGenerics(string name)
{
    var result = new StringBuilder();
    for (int i = 0; i < name.Length; i++)
    {
        if (name[i] == '`')
        {
            while (i + 1 < name.Length && char.IsDigit(name[i + 1]))
            {
                i++;
            }
            continue;
        }
        result.Append(name[i]);
    }
    return result.ToString();
}

static string Describe(MethodBase member) =>
    $"{member.Name}({string.Join(", ", member.GetParameters().Select(p => Spell(p.ParameterType)))})";

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
    if (Nullable.GetUnderlyingType(type) is { } underlying)
    {
        return Spell(underlying) + "?";
    }
    string name = type.Name;
    int tick = name.IndexOf('`');
    if (tick >= 0)
    {
        name = name[..tick];
    }
    return type.IsGenericType ? $"{name}<{string.Join(", ", type.GetGenericArguments().Select(Spell))}>" : name;
}

sealed class Unit
{
    public bool IsTest;
    public bool Asserts;
    public readonly HashSet<(string, int)> Named = [];
    public readonly HashSet<string> CallKeys = new(StringComparer.Ordinal);
    public List<Unit> Calls = [];
}

sealed record Row(string Assembly, string Namespace, string Type, string Member, bool Direct, bool Checked);
