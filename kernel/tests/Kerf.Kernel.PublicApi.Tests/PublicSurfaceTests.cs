// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The public surface shows no engine (docs/api/README.md, section 1.1; decision 16606): no public or protected member of
/// a public type in <c>Kerf</c>, <c>Kerf.Geometry</c> or <c>Kerf.Topology</c> names a type of <c>Kerf.Engine.*</c> or
/// <c>Occt</c>, except the transient base (question 55).
/// </summary>
public class PublicSurfaceTests
{
    private static readonly string[] PublicNamespaces = ["Kerf", "Kerf.Geometry", "Kerf.Topology"];

    private static IEnumerable<Type> PublicTypes()
        => new[] { typeof(Point3D).Assembly, typeof(Shape).Assembly }
            .SelectMany(a => a.GetExportedTypes())
            .Where(t => PublicNamespaces.Contains(t.Namespace));

    private static bool IsEngine(Type type)
    {
        if (type.HasElementType)
        {
            return IsEngine(type.GetElementType()!);
        }

        if (type.IsGenericType && type.GetGenericArguments().Any(IsEngine))
        {
            return true;
        }

        string? ns = type.Namespace;
        return ns is not null && (ns == "Occt" || ns.StartsWith("Occt.", StringComparison.Ordinal) || ns.StartsWith("Kerf.Engine", StringComparison.Ordinal))
               && type.Name != "Transient";
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

            foreach (MemberInfo member in type.GetMembers(all))
            {
                bool visible = member switch
                {
                    MethodBase m => m.IsPublic || m.IsFamily || m.IsFamilyOrAssembly,
                    FieldInfo f => f.IsPublic || f.IsFamily || f.IsFamilyOrAssembly,
                    PropertyInfo p => (p.GetMethod is { } g && (g.IsPublic || g.IsFamily || g.IsFamilyOrAssembly))
                                      || (p.SetMethod is { } s && (s.IsPublic || s.IsFamily || s.IsFamilyOrAssembly)),
                    Type t => t.IsNestedPublic || t.IsNestedFamily || t.IsNestedFamORAssem,
                    _ => false,
                };
                if (!visible)
                {
                    continue;
                }

                IEnumerable<Type> named = member switch
                {
                    MethodInfo m => [m.ReturnType, .. m.GetParameters().Select(p => p.ParameterType)],
                    ConstructorInfo c => c.GetParameters().Select(p => p.ParameterType),
                    FieldInfo f => [f.FieldType],
                    PropertyInfo p => [p.PropertyType, .. p.GetIndexParameters().Select(i => i.ParameterType)],
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
