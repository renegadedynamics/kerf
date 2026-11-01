using System.Reflection;
using System.Text;

namespace OcctNetGen;

/// <summary>C# spellings of types and names.</summary>
internal static class Cs
{
    private static readonly Dictionary<Type, string> Keywords = new()
    {
        [typeof(void)] = "void", [typeof(bool)] = "bool", [typeof(byte)] = "byte", [typeof(short)] = "short",
        [typeof(int)] = "int", [typeof(long)] = "long", [typeof(uint)] = "uint", [typeof(ulong)] = "ulong",
        [typeof(float)] = "float", [typeof(double)] = "double", [typeof(string)] = "string", [typeof(object)] = "object",
        [typeof(char)] = "char", [typeof(IntPtr)] = "IntPtr",
    };

    private static readonly HashSet<string> Reserved =
    [
        "abstract", "as", "base", "bool", "break", "byte", "case", "catch", "char", "checked", "class", "const", "continue",
        "decimal", "default", "delegate", "do", "double", "else", "enum", "event", "explicit", "extern", "false", "finally",
        "fixed", "float", "for", "foreach", "goto", "if", "implicit", "in", "int", "interface", "internal", "is", "lock",
        "long", "namespace", "new", "null", "object", "operator", "out", "override", "params", "private", "protected",
        "public", "readonly", "ref", "return", "sbyte", "sealed", "short", "sizeof", "stackalloc", "static", "string",
        "struct", "switch", "this", "throw", "true", "try", "typeof", "uint", "ulong", "unchecked", "unsafe", "ushort",
        "using", "virtual", "void", "volatile", "while",
    ];

    public static string Id(string name) => Reserved.Contains(name) ? "@" + name : name;

    private static string Name(Type t, Func<Type, string> plain)
    {
        if (t.IsByRef)
            return Name(t.GetElementType()!, plain);
        if (t.IsArray)
            return Name(t.GetElementType()!, plain) + "[]";
        if (Keywords.TryGetValue(t, out var k))
            return k;
        if (t.IsGenericType)
        {
            var def = t.GetGenericTypeDefinition();
            var stem = plain(def);
            stem = stem[..stem.IndexOf('`')];
            return $"{stem}<{string.Join(", ", t.GetGenericArguments().Select(a => Name(a, plain)))}>";
        }
        return plain(t);
    }

    /// <summary>A Build123d.Occt type as Build123d.Occt's own code spells it.</summary>
    public static string Native(Type t) => Name(t, x => x.Namespace == "Build123d.Occt" ? x.Name : "global::" + x.FullName!.Replace('+', '.'));

    /// <summary>An OCCT.NET type, fully qualified (the static class Build123d.Occt.Occt hides the namespace).</summary>
    public static string Occt(Type t) => Name(t, x => "global::" + x.FullName!.Replace('+', '.'));

    public static string Nullable(Type t, NullabilityInfo info) =>
        Native(t) + (!t.IsValueType && info.ReadState == NullabilityState.Nullable ? "?" : "");

    /// <summary>A member's key in the hand-written list and the report: <c>Name(T1, ref T2)</c>.</summary>
    public static string Key(MethodBase m) =>
        $"{(m is ConstructorInfo ? ".ctor" : m.Name)}({string.Join(", ", m.GetParameters().Select(p => (p.ParameterType.IsByRef ? "ref " : "") + Native(p.ParameterType)))})";
}

/// <summary>Members written by hand in src/Build123d.OcctNet/Runtime: bindings/occtnet.handwritten.txt.</summary>
internal sealed class Handwritten
{
    private readonly HashSet<string> _types = [];
    private readonly HashSet<string> _members = [];

    public static Handwritten Load(string path)
    {
        var h = new Handwritten();
        foreach (var raw in File.ReadAllLines(path))
        {
            var line = raw.Split('#')[0].Trim();
            if (line.Length == 0)
                continue;
            if (!line.Contains("::"))
                h._types.Add(line);
            else
                h._members.Add(line.Replace(" ", ""));
        }
        return h;
    }

    /// <summary>The whole type is hand-written.</summary>
    public bool Type(string name) => _types.Contains(name);

    /// <summary>The member is hand-written: listed by name for every overload, or with its parameters.</summary>
    public bool Member(string type, MethodBase m)
    {
        var name = m is ConstructorInfo ? ".ctor" : m.Name;
        return _members.Contains($"{type}::{name}") || _members.Contains($"{type}::{Cs.Key(m)}".Replace(" ", ""));
    }
}

internal static class Writer
{
    public static void WriteIfChanged(string path, string text)
    {
        text = text.Replace("\r\n", "\n");
        if (File.Exists(path) && File.ReadAllText(path).Replace("\r\n", "\n") == text)
            return;
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        File.WriteAllText(path, text, new UTF8Encoding(false));
    }

    public static void Prune(string dir, string pattern, IEnumerable<string> keep)
    {
        var set = keep.Select(Path.GetFullPath).ToHashSet(StringComparer.OrdinalIgnoreCase);
        foreach (var f in Directory.GetFiles(dir, pattern))
            if (!set.Contains(Path.GetFullPath(f)))
                File.Delete(f);
    }
}
