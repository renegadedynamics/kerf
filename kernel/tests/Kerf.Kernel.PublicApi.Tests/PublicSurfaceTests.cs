// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The public surface shows no engine (docs/api/README.md, section 1.1; decision 1519): no public or protected member of
/// a public type outside <c>Kerf.Engine.*</c> names a type of those namespaces, except the transient base
/// (question 55); and nothing is public in those namespaces but what <see cref="PublicEngineTypes"/> lists, with its reason.
/// </summary>
public class PublicSurfaceTests
{
    /// <summary>The public namespaces, so that a new one is added here on purpose. The checks read every public type.</summary>
    private static readonly string[] PublicNamespaces =
    [
        "Kerf", "Kerf.Booleans", "Kerf.Construction", "Kerf.Drawing", "Kerf.Exchange", "Kerf.Features", "Kerf.Fillets",
        "Kerf.Geometry", "Kerf.Healing", "Kerf.Measurement", "Kerf.Meshing", "Kerf.Offsets", "Kerf.Topology",
    ];

    /// <summary>The engine's types left public, each with the reason it can't be internal.</summary>
    private static readonly Dictionary<string, string> PublicEngineTypes = new()
    {
        ["Kerf.Engine.Foundation.Transient"] =
            "The base of the public Geometry3D and Geometry2D: C# needs a public class's base to be public. It has no public "
            + "or protected members and only the kernel can derive from it (question 55).",
    };

    private static IEnumerable<Type> ExportedTypes()
        => new[] { typeof(Point3D).Assembly, typeof(Shape).Assembly, typeof(GeometryOperations).Assembly, typeof(Kerf.Exchange.Step).Assembly }
            .Distinct()
            .SelectMany(a => a.GetExportedTypes());

    private static bool IsEngineNamespace(string? ns)
        => ns is not null && ns.StartsWith("Kerf.Engine", StringComparison.Ordinal);

    private static IEnumerable<Type> PublicTypes() => ExportedTypes().Where(t => !IsEngineNamespace(t.Namespace));

    [Fact]
    public void The_public_namespaces_are_the_ones_listed()
        => Assert.Equal(PublicNamespaces.Order(StringComparer.Ordinal), PublicTypes().Select(t => t.Namespace!).Distinct().Order(StringComparer.Ordinal));

    [Fact]
    public void Nothing_is_public_in_the_engine_but_what_is_listed()
        => Assert.Equal(PublicEngineTypes.Keys.Order(StringComparer.Ordinal),
                        ExportedTypes().Where(t => IsEngineNamespace(t.Namespace)).Select(t => t.FullName!).Order(StringComparer.Ordinal));

    [Fact]
    public void The_transient_base_shows_no_members()
    {
        Type transient = typeof(Geometry3D).BaseType!;
        Assert.Equal("Kerf.Engine.Foundation.Transient", transient.FullName);
        const BindingFlags all = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
        Assert.Empty(transient.GetMembers(all).Where(IsVisible).Select(m => m.Name));
    }

    private static bool IsVisible(MemberInfo member) => member switch
    {
        MethodBase m => m.IsPublic || m.IsFamily || m.IsFamilyOrAssembly,
        FieldInfo f => f.IsPublic || f.IsFamily || f.IsFamilyOrAssembly,
        PropertyInfo p => (p.GetMethod is { } g && (g.IsPublic || g.IsFamily || g.IsFamilyOrAssembly))
                          || (p.SetMethod is { } s && (s.IsPublic || s.IsFamily || s.IsFamilyOrAssembly)),
        EventInfo e => e.AddMethod is { } add && (add.IsPublic || add.IsFamily || add.IsFamilyOrAssembly),
        Type t => t.IsNestedPublic || t.IsNestedFamily || t.IsNestedFamORAssem,
        _ => false,
    };

    private static bool IsEngine(Type type)
    {
        if (type.HasElementType)
        {
            return IsEngine(type.GetElementType()!);
        }

        if (type.IsGenericParameter)
        {
            return false;
        }

        if (type.IsGenericType && type.GetGenericArguments().Any(IsEngine))
        {
            return true;
        }

        return IsEngineNamespace(type.Namespace) && type.Name != "Transient";
    }

    [Fact]
    public void No_public_signature_names_an_engine_type()
    {
        var found = new List<string>();
        const BindingFlags all = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
        foreach (Type type in PublicTypes())
        {
            if (type.BaseType is { } baseType && IsEngine(baseType))
            {
                found.Add($"{type}: base {baseType}");
            }

            foreach (Type implemented in type.GetInterfaces().Where(IsEngine))
            {
                found.Add($"{type}: interface {implemented}");
            }

            foreach (MemberInfo member in type.GetMembers(all))
            {
                if (!IsVisible(member))
                {
                    continue;
                }

                IEnumerable<Type> named = member switch
                {
                    MethodInfo m => [m.ReturnType, .. m.GetParameters().Select(p => p.ParameterType)],
                    ConstructorInfo c => c.GetParameters().Select(p => p.ParameterType),
                    FieldInfo f => [f.FieldType],
                    PropertyInfo p => [p.PropertyType, .. p.GetIndexParameters().Select(i => i.ParameterType)],
                    EventInfo e => [e.EventHandlerType!],
                    _ => [],
                };
                foreach (Type t in named.Where(IsEngine))
                {
                    found.Add($"{type}.{member.Name}: {t}");
                }
            }
        }

        Assert.Empty(found);
    }

    [Fact]
    public void The_geometry_hierarchy_is_closed_to_other_assemblies()
    {
        foreach (Type type in PublicTypes().Where(t => t.IsClass && (t.IsSubclassOf(typeof(Geometry3D)) || t.IsSubclassOf(typeof(Geometry2D)))))
        {
            if (type.IsAbstract)
            {
                Assert.DoesNotContain(type.GetConstructors(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance),
                                      c => c.IsPublic || c.IsFamily || c.IsFamilyOrAssembly);
            }
            else
            {
                Assert.True(type.IsSealed, $"{type} is not sealed.");
            }
        }
    }
}
