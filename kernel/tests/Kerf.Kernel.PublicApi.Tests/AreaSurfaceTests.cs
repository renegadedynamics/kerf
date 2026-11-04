// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Kerf.Exchange;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Meshing;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Unit 10L3's areas show no engine (decision 1519's rule, as <see cref="PublicSurfaceTests"/> checks it for 10L1's):
/// no public or protected member of a public type in <c>Kerf.Measurement</c>, <c>Kerf.Healing</c>, <c>Kerf.Meshing</c>
/// or <c>Kerf.Exchange</c>, nor <c>Kerf.Color</c> and <c>Kerf.Geometry.OrientedBox</c>, names a type of
/// <c>Kerf.Engine.*</c> or <c>Occt</c>.
/// </summary>
public class AreaSurfaceTests
{
    private static readonly string[] Namespaces = ["Kerf.Measurement", "Kerf.Healing", "Kerf.Meshing", "Kerf.Exchange"];

    private static IEnumerable<Type> AreaTypes()
        => new[] { typeof(MeasureOperations).Assembly, typeof(Step).Assembly }
            .SelectMany(a => a.GetExportedTypes())
            .Where(t => Namespaces.Contains(t.Namespace))
            .Concat([typeof(Color), typeof(Kerf.Geometry.OrientedBox)]);

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
        return ns is not null && (ns == "Occt" || ns.StartsWith("Occt.", StringComparison.Ordinal) || ns.StartsWith("Kerf.Engine", StringComparison.Ordinal));
    }

    [Fact]
    public void No_public_signature_names_an_engine_type()
    {
        var found = new List<string>();
        const BindingFlags all = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
        foreach (Type type in AreaTypes())
        {
            foreach (MemberInfo member in type.GetMembers(all))
            {
                bool visible = member switch
                {
                    MethodBase m => m.IsPublic || m.IsFamily || m.IsFamilyOrAssembly,
                    FieldInfo f => f.IsPublic || f.IsFamily || f.IsFamilyOrAssembly,
                    PropertyInfo p => (p.GetMethod is { } g && (g.IsPublic || g.IsFamily || g.IsFamilyOrAssembly))
                                      || (p.SetMethod is { } s && (s.IsPublic || s.IsFamily || s.IsFamilyOrAssembly)),
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
                    PropertyInfo p => [p.PropertyType],
                    _ => [],
                };
                found.AddRange(named.Where(IsEngine).Select(t => $"{type}.{member.Name}: {t}"));
            }
        }

        Assert.Empty(found);
    }

    [Fact]
    public void Each_area_has_its_operations_and_options()
    {
        Assert.All(new[] { typeof(MeasureOperations), typeof(HealingOperations), typeof(Mesher), typeof(Step), typeof(Stl), typeof(Gltf),
                           typeof(ThreeMf), typeof(Obj), typeof(Brep) },
                   t => Assert.True(t.IsAbstract && t.IsSealed, $"{t} is not static."));
        Assert.All(new[] { typeof(MassOptions), typeof(DistanceOptions), typeof(FixOptions), typeof(SewOptions), typeof(UnifyOptions),
                           typeof(MeshOptions), typeof(StepWriteOptions), typeof(GltfWriteOptions), typeof(ReadLimits) },
                   t => Assert.NotNull(t.GetProperty("Default", BindingFlags.Public | BindingFlags.Static)));
    }
}
