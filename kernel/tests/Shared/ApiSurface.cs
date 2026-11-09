// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using System.Text;

namespace Kerf.Tests.Shared;

/// <summary>
/// The public surface of Kerf's shipped assemblies, read by reflection (unit 13P, decision 2043): as text, one line per
/// type and member, for the committed baselines; and the ways it lets a caller change a value, for the immutability
/// guard (design-rules.md section 3). It reads metadata only (<see cref="CustomAttributeData"/>, no attribute
/// instances), so the same code describes assemblies loaded into a <c>MetadataLoadContext</c>, such as a release's.
/// </summary>
public static class ApiSurface
{
    private const BindingFlags Declared = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;

    /// <summary>The environment variable that makes a baseline test write the surface it found rather than fail.</summary>
    public const string UpdateVariable = "KERF_API_UPDATE";

    /// <summary>The surface as text: per assembly, each visible type's line and its visible members' lines, sorted.</summary>
    public static string Describe(IEnumerable<Assembly> assemblies)
    {
        var text = new StringBuilder();
        text.Append("# Kerf's public API, by reflection (unit 13P; design-rules.md, section 3). Generated: run PublicApiBaselineTests\r\n")
            .Append("# with ").Append(UpdateVariable).Append("=1 to rewrite it. Each type's line, then its public and protected members', sorted.\r\n");
        foreach (Assembly assembly in assemblies.OrderBy(a => a.GetName().Name, StringComparer.Ordinal))
        {
            text.Append("\r\nassembly ").Append(assembly.GetName().Name).Append("\r\n");
            foreach (Type type in VisibleTypes(assembly).OrderBy(TypeName, StringComparer.Ordinal))
            {
                text.Append(TypeLine(type)).Append("\r\n");
                foreach (string member in MemberLines(type))
                {
                    text.Append("  ").Append(member).Append("\r\n");
                }
            }
        }

        return text.ToString();
    }

    /// <summary>Compares <paramref name="assemblies"/>' surface with the baseline file, or rewrites it when asked to.</summary>
    /// <returns>Null when they agree (or the file was rewritten); otherwise a message with the lines that differ.</returns>
    public static string? CompareWithBaseline(IEnumerable<Assembly> assemblies, string baselinePath)
    {
        string actual = Describe(assemblies);
        if (Environment.GetEnvironmentVariable(UpdateVariable) == "1")
        {
            File.WriteAllText(baselinePath, actual);
            return null;
        }

        string expected = File.Exists(baselinePath) ? File.ReadAllText(baselinePath).ReplaceLineEndings("\r\n") : "";
        if (expected == actual)
        {
            return null;
        }

        string[] before = expected.Split("\r\n"), after = actual.Split("\r\n");
        var removed = before.Except(after).ToList();
        var added = after.Except(before).ToList();
        var message = new StringBuilder();
        message.Append("The public API differs from ").Append(baselinePath)
               .Append(". If the change is meant, regenerate the file (").Append(UpdateVariable)
               .Append("=1, then run this test) and commit it with the change, so it's seen at review.\n");
        foreach (string line in removed.Take(40))
        {
            message.Append("- ").Append(line).Append('\n');
        }

        foreach (string line in added.Take(40))
        {
            message.Append("+ ").Append(line).Append('\n');
        }

        if (removed.Count + added.Count == 0)
        {
            message.Append("(the same lines in another order)\n");
        }

        return message.ToString();
    }

    /// <summary>The repository's root, found from the tests' folder (it holds Kerf.slnx).</summary>
    public static string RepositoryRoot()
    {
        for (string? dir = AppContext.BaseDirectory; dir is not null; dir = Path.GetDirectoryName(dir))
        {
            if (File.Exists(Path.Combine(dir, "Kerf.slnx")))
            {
                return dir;
            }
        }

        throw new InvalidOperationException("Kerf.slnx wasn't found above " + AppContext.BaseDirectory);
    }

    // ---- The guard: the ways the public surface lets a value change ----

    /// <summary>
    /// What could change a value of the public types of Kerf's public namespaces (<c>Kerf</c>, <c>Kerf.X</c>; not
    /// <c>Kerf.Engine.*</c> or <c>Kerf.Kernel.*</c>), each as a key (<c>Type.Member</c>, or the type's name) and why:
    /// a settable property (not <c>init</c>); a field that isn't read-only; a mutable collection or span in a
    /// property's or field's type, or returned by a method; an <c>init</c> property of a read-only collection
    /// interface, which would keep the caller's own collection; an event; a static field that isn't read-only, whatever
    /// its access; a struct member that can change the struct; a type that is a mutable collection; or a
    /// <c>void</c> method shaped like an in-place mutator.
    /// </summary>
    public static IEnumerable<(string Key, string Why)> Mutability(IEnumerable<Assembly> assemblies)
    {
        foreach (Type type in assemblies.SelectMany(VisibleTypes).Where(t => IsPublicNamespace(t.Namespace) && !t.IsEnum && !IsDelegate(t))
                                        .OrderBy(TypeName, StringComparer.Ordinal))
        {
            string name = TypeName(type);
            if (!type.IsInterface && MutableCollectionInterface(type) is { } collection)
            {
                yield return (name, "a mutable collection (" + collection + ")");
            }

            foreach (FieldInfo field in type.GetFields(Declared))
            {
                if (IsCompilerGenerated(field) && !field.Name.Contains("k__BackingField", StringComparison.Ordinal))
                {
                    continue;
                }

                if (field.IsStatic && !field.IsLiteral && !field.IsInitOnly)
                {
                    yield return (name + "." + BackedName(field.Name), "mutable static state");
                }
                else if (IsVisible(field) && !field.IsLiteral && !field.IsInitOnly)
                {
                    yield return (name + "." + field.Name, "a field that isn't readonly");
                }
                else if (IsVisible(field) && MutableCollectionName(field.FieldType) is { } held)
                {
                    yield return (name + "." + field.Name, "a field holding " + held);
                }
            }

            foreach (PropertyInfo property in type.GetProperties(Declared))
            {
                if (!IsVisible(property))
                {
                    continue;
                }

                if (property.SetMethod is { } set && IsVisible(set))
                {
                    if (!IsInitOnly(set))
                    {
                        yield return (name + "." + property.Name, "a settable property");
                    }
                    else if (CollectionInterfaceName(property.PropertyType) is { } kept)
                    {
                        yield return (name + "." + property.Name, "an init property of " + kept + ", which keeps the caller's collection");
                    }
                }

                if (MutableCollectionName(property.PropertyType) is { } held)
                {
                    yield return (name + "." + property.Name, "a property holding " + held);
                }
            }

            foreach (EventInfo e in type.GetEvents(Declared))
            {
                if (e.AddMethod is { } add && IsVisible(add))
                {
                    yield return (name + "." + e.Name, "an event");
                }
            }

            foreach (MethodInfo method in type.GetMethods(Declared).Where(m => IsVisible(m) && !IsCompilerGenerated(m)))
            {
                if (!method.IsSpecialName && MutableCollectionName(method.ReturnType) is { } returned)
                {
                    yield return (name + "." + method.Name, "a method returning " + returned);
                }

                if (method.IsStatic)
                {
                    continue;
                }

                if (type.IsValueType && !IsReadOnlyStruct(type) && !HasAttribute(method, "System.Runtime.CompilerServices.IsReadOnlyAttribute")
                    && !(method.IsSpecialName && IsInitOnly(method)))
                {
                    yield return (name + "." + method.Name, "a member of a struct that isn't readonly, which can change it");
                }
                else if (!type.IsValueType && !method.IsSpecialName && method.ReturnType.FullName == "System.Void"
                         && method.GetParameters().All(p => !p.IsOut) && IsMutatorName(method.Name))
                {
                    yield return (name + "." + method.Name, "a void method that changes its object in place");
                }
            }
        }
    }

    /// <summary>Whether a namespace is one of Kerf's public ones (as KERF0003 has it).</summary>
    public static bool IsPublicNamespace(string? ns)
        => ns == "Kerf" || ns is not null && ns.StartsWith("Kerf.", StringComparison.Ordinal)
           && !ns.StartsWith("Kerf.Engine", StringComparison.Ordinal) && !ns.StartsWith("Kerf.Kernel", StringComparison.Ordinal);

    // OCCT's in-place operations, as KERF0003 lists them
    private static readonly HashSet<string> InPlaceNames =
    [
        "Transform", "Translate", "Rotate", "Scale", "Mirror", "Reverse", "UReverse", "VReverse", "Normalize",
        "Invert", "Multiply", "PreMultiply", "Add", "Subtract", "Divide", "Cross", "CrossCross", "Power", "Transpose",
        "Nullify", "Move", "Complement", "Compose", "Segment", "IncreaseDegree", "InsertKnot", "InsertKnots",
        "RemoveKnot", "RemovePole", "MovePoint", "ExchangeUV", "Clear", "Remove", "Insert", "Append", "Prepend",
    ];

    private static bool IsMutatorName(string name)
        => InPlaceNames.Contains(name) || name.Length > 3 && name.StartsWith("Set", StringComparison.Ordinal) && char.IsUpper(name[3]);

    private static readonly HashSet<string> MutableCollections =
    [
        "System.Collections.Generic.List`1", "System.Collections.Generic.Dictionary`2", "System.Collections.Generic.HashSet`1",
        "System.Collections.Generic.SortedDictionary`2", "System.Collections.Generic.SortedList`2", "System.Collections.Generic.SortedSet`1",
        "System.Collections.Generic.Queue`1", "System.Collections.Generic.Stack`1", "System.Collections.Generic.LinkedList`1",
        "System.Collections.Generic.PriorityQueue`2", "System.Collections.Generic.ICollection`1", "System.Collections.Generic.IList`1",
        "System.Collections.Generic.IDictionary`2", "System.Collections.Generic.ISet`1", "System.Collections.ObjectModel.Collection`1",
        "System.Collections.ObjectModel.ObservableCollection`1", "System.Collections.Concurrent.ConcurrentDictionary`2",
        "System.Collections.Concurrent.ConcurrentBag`1", "System.Collections.Concurrent.ConcurrentQueue`1", "System.Span`1",
        "System.Memory`1", "System.ArraySegment`1", "System.Collections.ArrayList", "System.Collections.Hashtable",
        "System.Collections.IList", "System.Collections.IDictionary", "System.Text.StringBuilder",
    ];

    private static string? MutableCollectionName(Type type)
    {
        if (type.IsByRef)
        {
            type = type.GetElementType()!;
        }

        if (type.IsArray)
        {
            return "an array";
        }

        string? definition = type.IsGenericType ? type.GetGenericTypeDefinition().FullName : type.FullName;
        return definition is not null && MutableCollections.Contains(definition) ? ShortName(type) : null;
    }

    private static string? MutableCollectionInterface(Type type)
    {
        foreach (Type implemented in type.GetInterfaces())
        {
            string? definition = implemented.IsGenericType ? implemented.GetGenericTypeDefinition().FullName : implemented.FullName;
            if (definition is "System.Collections.Generic.ICollection`1" or "System.Collections.Generic.IList`1" or "System.Collections.Generic.IDictionary`2"
                or "System.Collections.Generic.ISet`1" or "System.Collections.IList" or "System.Collections.IDictionary")
            {
                return ShortName(implemented);
            }
        }

        return null;
    }

    private static string? CollectionInterfaceName(Type type)
    {
        string? definition = type.IsGenericType ? type.GetGenericTypeDefinition().FullName : type.FullName;
        return definition is "System.Collections.Generic.IEnumerable`1" or "System.Collections.Generic.IReadOnlyList`1"
            or "System.Collections.Generic.IReadOnlyCollection`1" or "System.Collections.Generic.IReadOnlyDictionary`2"
            or "System.Collections.Generic.IReadOnlySet`1" or "System.ReadOnlyMemory`1"
            ? ShortName(type)
            : null;
    }

    private static string BackedName(string fieldName)
        => fieldName.StartsWith('<') && fieldName.IndexOf('>') is > 1 and var end ? fieldName[1..end] : fieldName;

    // ---- Reading the metadata ----

    private static IEnumerable<Type> VisibleTypes(Assembly assembly)
    {
        Type[] types;
        try
        {
            types = assembly.GetTypes();
        }
        catch (ReflectionTypeLoadException e)
        {
            types = e.Types.Where(t => t is not null).ToArray()!;
        }

        return types.Where(IsVisible);
    }

    private static bool IsVisible(Type type)
    {
        if (type.Name.Contains('<'))
        {
            return false;
        }

        for (Type? t = type; t is not null; t = t.DeclaringType)
        {
            if (!(t.IsPublic || t.IsNestedPublic || t.IsNestedFamily || t.IsNestedFamORAssem))
            {
                return false;
            }
        }

        return true;
    }

    // A protected member is reachable only from a type another assembly can derive: one that isn't sealed and has a
    // public or protected constructor. The geometry's protected internal mutators, on a hierarchy closed to other
    // assemblies, aren't.
    private static bool IsVisible(MethodBase method)
        => method.IsPublic || (method.IsFamily || method.IsFamilyOrAssembly) && IsDerivable(method.DeclaringType!);

    private static bool IsVisible(FieldInfo field)
        => field.IsPublic || (field.IsFamily || field.IsFamilyOrAssembly) && IsDerivable(field.DeclaringType!);

    private static bool IsDerivable(Type type)
        => !type.IsSealed && !type.IsValueType && !type.IsInterface
           && type.GetConstructors(Declared).Any(c => !c.IsStatic && (c.IsPublic || c.IsFamily || c.IsFamilyOrAssembly));

    private static bool IsVisible(PropertyInfo property)
        => property.GetMethod is { } get && IsVisible(get) || property.SetMethod is { } set && IsVisible(set);

    private static bool HasAttribute(MemberInfo member, string fullName)
        => member.GetCustomAttributesData().Any(a => a.AttributeType.FullName == fullName);

    private static bool HasAttribute(ParameterInfo parameter, string fullName)
        => parameter.GetCustomAttributesData().Any(a => a.AttributeType.FullName == fullName);

    private static bool IsCompilerGenerated(MemberInfo member) => HasAttribute(member, "System.Runtime.CompilerServices.CompilerGeneratedAttribute");

    private static bool IsReadOnlyStruct(Type type) => HasAttribute(type, "System.Runtime.CompilerServices.IsReadOnlyAttribute");

    private static bool IsInitOnly(MethodInfo setter)
        => setter.ReturnParameter.GetRequiredCustomModifiers().Any(m => m.FullName == "System.Runtime.CompilerServices.IsExternalInit");

    private static bool IsRecord(Type type)
        => type.GetMethods(Declared).Any(m => m.Name == "PrintMembers" && IsCompilerGenerated(m));

    private static bool IsDelegate(Type type) => type.BaseType?.FullName == "System.MulticastDelegate";

    // ---- Writing it as text ----

    private static string TypeLine(Type type)
    {
        var line = new StringBuilder(Access(type)).Append(' ');
        if (type.IsEnum)
        {
            line.Append("enum ").Append(TypeName(type));
            Type underlying = type.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance)[0].FieldType;
            if (underlying.FullName != "System.Int32")
            {
                line.Append(" : ").Append(ShortName(underlying));
            }

            if (HasAttribute(type, "System.FlagsAttribute"))
            {
                line.Insert(0, "[Flags] ");
            }

            return line.ToString();
        }

        if (IsDelegate(type))
        {
            MethodInfo invoke = type.GetMethod("Invoke")!;
            return line.Append("delegate ").Append(ShortName(invoke.ReturnType)).Append(' ').Append(TypeName(type))
                       .Append(GenericParameters(type)).Append(Parameters(invoke.GetParameters())).ToString();
        }

        if (type.IsInterface)
        {
            line.Append("interface ");
        }
        else if (type.IsValueType)
        {
            line.Append(IsReadOnlyStruct(type) ? "readonly " : "")
                .Append(HasAttribute(type, "System.Runtime.CompilerServices.IsByRefLikeAttribute") ? "ref " : "")
                .Append(IsRecord(type) ? "record struct " : "struct ");
        }
        else
        {
            line.Append(type.IsAbstract && type.IsSealed ? "static " : type.IsAbstract ? "abstract " : type.IsSealed ? "sealed " : "")
                .Append(IsRecord(type) ? "record " : "class ");
        }

        line.Append(TypeName(type)).Append(GenericParameters(type));
        var bases = new List<string>();
        if (type.BaseType is { } baseType && baseType.FullName is not ("System.Object" or "System.ValueType"))
        {
            bases.Add(ShortName(baseType));
        }

        bases.AddRange(type.GetInterfaces().Where(IsVisible).Select(ShortName).Order(StringComparer.Ordinal));
        if (bases.Count > 0)
        {
            line.Append(" : ").AppendJoin(", ", bases);
        }

        return line.ToString();
    }

    private static IEnumerable<string> MemberLines(Type type)
    {
        if (type.IsEnum)
        {
            return type.GetFields(BindingFlags.Public | BindingFlags.Static)
                       .OrderBy(f => Convert.ToInt64(f.GetRawConstantValue(), System.Globalization.CultureInfo.InvariantCulture))
                       .ThenBy(f => f.Name, StringComparer.Ordinal)
                       .Select(f => f.Name + " = " + Convert.ToString(f.GetRawConstantValue(), System.Globalization.CultureInfo.InvariantCulture));
        }

        if (IsDelegate(type))
        {
            return [];
        }

        var lines = new List<(int Kind, string Text)>();
        foreach (ConstructorInfo constructor in type.GetConstructors(Declared))
        {
            if (!constructor.IsStatic && IsVisible(constructor) && !IsCompilerGenerated(constructor))
            {
                lines.Add((0, Access(constructor) + " " + type.Name.Split('`')[0] + Parameters(constructor.GetParameters())));
            }
        }

        foreach (FieldInfo field in type.GetFields(Declared))
        {
            if (IsVisible(field) && !IsCompilerGenerated(field))
            {
                string value = field.IsLiteral ? " = " + DefaultValue(field.FieldType, field.GetRawConstantValue()) : "";
                lines.Add((1, Access(field) + (field.IsLiteral ? " const " : field.IsStatic ? " static " : " ")
                              + (field.IsInitOnly ? "readonly " : "") + Annotated(field.FieldType, Nullability(() => Context.Create(field)))
                              + " " + field.Name + value));
            }
        }

        foreach (PropertyInfo property in type.GetProperties(Declared))
        {
            if (!IsVisible(property) || IsCompilerGenerated(property))
            {
                continue;
            }

            MethodInfo accessor = (property.GetMethod is { } g && IsVisible(g) ? g : property.SetMethod)!;
            var line = new StringBuilder(Access(accessor)).Append(Modifiers(type, accessor));
            if (HasAttribute(property, "System.Runtime.CompilerServices.RequiredMemberAttribute"))
            {
                line.Append(" required");
            }

            line.Append(' ').Append(Annotated(property.PropertyType, Nullability(() => Context.Create(property)))).Append(' ');
            ParameterInfo[] index = property.GetIndexParameters();
            line.Append(index.Length > 0 ? "this[" + Parameters(index)[1..^1] + "]" : property.Name).Append(" {");
            if (property.GetMethod is { } get && IsVisible(get))
            {
                line.Append(" get;");
            }

            if (property.SetMethod is { } set && IsVisible(set))
            {
                line.Append(Access(set) == Access(accessor) ? "" : " " + Access(set)).Append(IsInitOnly(set) ? " init;" : " set;");
            }

            lines.Add((2, line.Append(" }").ToString()));
        }

        foreach (EventInfo e in type.GetEvents(Declared))
        {
            if (e.AddMethod is { } add && IsVisible(add))
            {
                lines.Add((3, Access(add) + Modifiers(type, add) + " event " + ShortName(e.EventHandlerType!) + " " + e.Name));
            }
        }

        foreach (MethodInfo method in type.GetMethods(Declared))
        {
            if (!IsVisible(method) || IsCompilerGenerated(method) || method.IsSpecialName && !method.Name.StartsWith("op_", StringComparison.Ordinal))
            {
                continue;
            }

            string returned = Annotated(method.ReturnType, Nullability(() => Context.Create(method.ReturnParameter)));
            if (method.ReturnType.IsByRef)
            {
                returned = (HasAttribute(method.ReturnParameter, "System.Runtime.CompilerServices.IsReadOnlyAttribute") ? "ref readonly " : "ref ") + returned;
            }

            lines.Add((4, Access(method) + Modifiers(type, method) + " " + returned + " " + method.Name + GenericParameters(method)
                          + Parameters(method.GetParameters())));
        }

        return lines.OrderBy(l => l.Kind).ThenBy(l => l.Text, StringComparer.Ordinal).Select(l => l.Text);
    }

    private static string Access(Type type) => type.IsPublic || type.IsNestedPublic ? "public" : "protected";

    private static string Access(MethodBase method) => method.IsPublic ? "public" : "protected";

    private static string Access(FieldInfo field) => field.IsPublic ? "public" : "protected";

    private static string Modifiers(Type type, MethodInfo method)
    {
        if (method.IsStatic)
        {
            return method.IsAbstract ? " static abstract" : method.IsVirtual ? " static virtual" : " static";
        }

        if (type.IsInterface)
        {
            return method.IsAbstract ? "" : " virtual";
        }

        if (method.IsAbstract)
        {
            return " abstract";
        }

        if (!method.IsVirtual)
        {
            return "";
        }

        bool overrides = !method.Attributes.HasFlag(MethodAttributes.NewSlot);
        return overrides ? method.IsFinal ? " sealed override" : " override" : method.IsFinal ? "" : " virtual";
    }

    private static string Parameters(ParameterInfo[] parameters)
    {
        var text = new StringBuilder("(");
        for (int i = 0; i < parameters.Length; i++)
        {
            ParameterInfo p = parameters[i];
            if (i > 0)
            {
                text.Append(", ");
            }

            if (HasAttribute(p, "System.ParamArrayAttribute") || HasAttribute(p, "System.Runtime.CompilerServices.ParamCollectionAttribute"))
            {
                text.Append("params ");
            }

            if (i == 0 && p.Member is MethodInfo { IsStatic: true } m && HasAttribute(m, "System.Runtime.CompilerServices.ExtensionAttribute"))
            {
                text.Append("this ");
            }

            if (p.ParameterType.IsByRef)
            {
                text.Append(p.IsOut ? "out " : HasAttribute(p, "System.Runtime.CompilerServices.RequiresLocationAttribute") ? "ref readonly "
                                      : p.IsIn ? "in " : "ref ");
            }

            ParameterInfo parameter = p;
            text.Append(Annotated(p.ParameterType, Nullability(() => Context.Create(parameter)))).Append(' ').Append(p.Name);
            if (p.HasDefaultValue)
            {
                text.Append(" = ").Append(DefaultValue(p.ParameterType, p.RawDefaultValue));
            }
        }

        return text.Append(')').ToString();
    }

    // An enum's default by its member's name, as it's written
    private static string DefaultValue(Type type, object? value)
    {
        if (type.IsGenericType && type.GetGenericTypeDefinition().FullName == "System.Nullable`1")
        {
            type = type.GetGenericArguments()[0];
        }

        if (type.IsEnum && value is not null)
        {
            long raw = Convert.ToInt64(value, System.Globalization.CultureInfo.InvariantCulture);
            FieldInfo? member = type.GetFields(BindingFlags.Public | BindingFlags.Static)
                                    .FirstOrDefault(f => Convert.ToInt64(f.GetRawConstantValue(), System.Globalization.CultureInfo.InvariantCulture) == raw);
            return member is not null ? ShortName(type) + "." + member.Name : "(" + ShortName(type) + ")" + Literal(value);
        }

        return Literal(value);
    }

    private static string Literal(object? value) => value switch
    {
        null => "default",
        string s => "\"" + s + "\"",
        bool b => b ? "true" : "false",
        char c => "'" + c + "'",
        double d => d.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
        float f => f.ToString("R", System.Globalization.CultureInfo.InvariantCulture),
        IFormattable f => f.ToString(null, System.Globalization.CultureInfo.InvariantCulture),
        _ => value.ToString() ?? "",
    };

    private static string GenericParameters(Type type)
    {
        Type[] all = type.GetGenericArguments();
        int inherited = type.DeclaringType?.GetGenericArguments().Length ?? 0;
        return all.Length > inherited ? "<" + string.Join(", ", all.Skip(inherited).Select(t => t.Name)) + ">" : "";
    }

    private static string GenericParameters(MethodInfo method)
        => method.IsGenericMethodDefinition ? "<" + string.Join(", ", method.GetGenericArguments().Select(t => t.Name)) + ">" : "";

    /// <summary>A type's name with its namespace and enclosing types (<c>Kerf.Geometry.Point3D</c>), without generic arguments.</summary>
    private static string TypeName(Type type)
    {
        string name = type.Name.Split('`')[0];
        return type.DeclaringType is { } outer ? TypeName(outer) + "." + name : (type.Namespace is { } ns ? ns + "." : "") + name;
    }

    private static readonly NullabilityInfoContext Context = new();

    private static NullabilityInfo? Nullability(Func<NullabilityInfo> create)
    {
        try
        {
            return create();
        }
        catch (Exception e) when (e is NotSupportedException or InvalidOperationException or ArgumentException)
        {
            return null;
        }
    }

    /// <summary>A type as C# names it in a signature, without namespaces, with <c>?</c> where its annotation says null.</summary>
    private static string Annotated(Type type, NullabilityInfo? info)
    {
        if (type.IsByRef)
        {
            return Annotated(type.GetElementType()!, info);
        }

        string text;
        if (type.IsArray)
        {
            text = Annotated(type.GetElementType()!, info?.ElementType) + "[" + new string(',', type.GetArrayRank() - 1) + "]";
        }
        else if (type.IsPointer)
        {
            text = Annotated(type.GetElementType()!, null) + "*";
        }
        else if (type.IsGenericParameter)
        {
            text = type.Name;
        }
        else if (type.IsGenericType && type.GetGenericTypeDefinition().FullName == "System.Nullable`1")
        {
            return Annotated(type.GetGenericArguments()[0], null) + "?";
        }
        else if (type.IsGenericType)
        {
            Type[] arguments = type.GetGenericArguments();
            NullabilityInfo[]? infos = info?.GenericTypeArguments.Length == arguments.Length ? info.GenericTypeArguments : null;
            string outer = type.DeclaringType is { } d ? ShortName(d) + "." : "";
            int inherited = type.DeclaringType?.GetGenericArguments().Length ?? 0;
            text = outer + type.Name.Split('`')[0];
            if (arguments.Length > inherited)
            {
                text += "<" + string.Join(", ", arguments.Select((a, i) => (a, i)).Skip(inherited).Select(x => Annotated(x.a, infos?[x.i]))) + ">";
            }

            if (type.FullName?.StartsWith("System.ValueTuple`", StringComparison.Ordinal) == true)
            {
                text = "(" + string.Join(", ", arguments.Select((a, i) => Annotated(a, infos?[i]))) + ")";
            }
        }
        else
        {
            text = ShortName(type);
        }

        return !type.IsValueType && info?.ReadState == NullabilityState.Nullable ? text + "?" : text;
    }

    private static string ShortName(Type type)
    {
        if (type.IsGenericType || type.IsArray || type.IsByRef || type.IsPointer)
        {
            return Annotated(type, null);
        }

        return type.FullName switch
        {
            "System.Void" => "void",
            "System.Object" => "object",
            "System.String" => "string",
            "System.Boolean" => "bool",
            "System.Char" => "char",
            "System.Byte" => "byte",
            "System.SByte" => "sbyte",
            "System.Int16" => "short",
            "System.UInt16" => "ushort",
            "System.Int32" => "int",
            "System.UInt32" => "uint",
            "System.Int64" => "long",
            "System.UInt64" => "ulong",
            "System.Single" => "float",
            "System.Double" => "double",
            "System.Decimal" => "decimal",
            "System.IntPtr" => "nint",
            "System.UIntPtr" => "nuint",
            _ => type.DeclaringType is { } outer && !type.IsGenericParameter ? ShortName(outer) + "." + type.Name : type.Name,
        };
    }
}
