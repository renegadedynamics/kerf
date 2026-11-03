using System.Collections.Concurrent;
using System.Diagnostics.CodeAnalysis;
using System.Reflection;

namespace Kerf.Modeling;

/// <summary>
/// A shape's property, or argument-free method, named by a string: what build123d takes as a <c>property</c>
/// (<c>Edge.length</c>, <c>Face.radius</c>) in <c>filter_by</c>, <c>filter_by_value</c>, <c>sort_by</c> and
/// <c>group_by</c>, for callers that can pass a value but not a delegate, such as a spreadsheet formula.
/// </summary>
/// <remarks>
/// A name matches a public member of the shape's own class, ignoring case and underscores, so "IsClosed",
/// "isClosed" and build123d's "is_closed" are the same. Properties come before methods, and a method is taken only
/// if all its parameters are optional (<c>Center()</c>).
/// </remarks>
internal static class ShapeProperty
{
    private const DynamicallyAccessedMemberTypes Members =
        DynamicallyAccessedMemberTypes.PublicProperties | DynamicallyAccessedMemberTypes.PublicMethods;

    private static readonly ConcurrentDictionary<(Type, string), Func<Shape, object?>?> Cache = new();

    /// <summary>The named measure of <paramref name="shape"/>.</summary>
    /// <exception cref="ArgumentException">The shape's class has no such property or method.</exception>
    internal static object? Get(Shape shape, string name)
    {
        var type = shape.GetType();
        var getter = Cache.GetOrAdd((type, Normalize(name)), _ => Resolve(type, Normalize(name)))
                     ?? throw new ArgumentException(
                         $"A {type.Name} has no property '{name}'. It has: {string.Join(", ", Names(type))}",
                         nameof(name));
        return getter(shape);
    }

    /// <summary>The named measure as a number, for comparison, or the value itself.</summary>
    internal static object? Key(Shape shape, string name, int? roundTo = null)
    {
        var value = Get(shape, name);
        return roundTo is { } digits && AsDouble(value) is { } d ? Math.Round(d, digits) : value;
    }

    internal static double? AsDouble(object? x) => x switch
    {
        double d => d,
        float f => f,
        int i => i,
        long l => l,
        _ => null,
    };

    /// <summary>
    /// Whether a measured value is <paramref name="value"/>: numbers within <paramref name="tolerance"/>, an enum
    /// equal to one or to its name ("Plane", or build123d's "PLANE"), anything else equal.
    /// </summary>
    internal static bool Matches(object? measured, object? value, double tolerance)
    {
        if (measured is null)
            return false;
        if (AsDouble(measured) is { } m && AsDouble(value) is { } v)
            return Math.Abs(m - v) <= tolerance;
        if (measured is Enum && value is string text)
            return Enum.TryParse(measured.GetType(), text.Replace("_", ""), ignoreCase: true, out var parsed) && measured.Equals(parsed);
        return measured.Equals(value);
    }

    private static string Normalize(string name)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(name);
        return name.Replace("_", "").Trim().ToLowerInvariant();
    }

    private static Func<Shape, object?>? Resolve([DynamicallyAccessedMembers(Members)] Type type, string name)
    {
        if (Properties(type).FirstOrDefault(p => Normalize(p.Name) == name) is { } property)
            return shape => property.GetValue(shape, BindingFlags.DoNotWrapExceptions, null, null, null);
        if (Methods(type).Where(m => Normalize(m.Name) == name).OrderBy(m => m.GetParameters().Length).FirstOrDefault() is { } method)
        {
            var defaults = method.GetParameters().Select(p => p.DefaultValue).ToArray();
            return shape => method.Invoke(shape, BindingFlags.DoNotWrapExceptions, null, defaults.ToArray(), null);
        }
        return null;
    }

    private static IEnumerable<PropertyInfo> Properties([DynamicallyAccessedMembers(Members)] Type type) =>
        type.GetProperties(BindingFlags.Public | BindingFlags.Instance).Where(p => p.GetIndexParameters().Length == 0 && p.CanRead);

    private static IEnumerable<MethodInfo> Methods([DynamicallyAccessedMembers(Members)] Type type) =>
        type.GetMethods(BindingFlags.Public | BindingFlags.Instance)
            .Where(m => !m.IsSpecialName && !m.IsGenericMethodDefinition && m.ReturnType != typeof(void)
                        && m.DeclaringType != typeof(object) && m.GetParameters().All(p => p.HasDefaultValue));

    /// <summary>The names a shape class answers to, for an error message.</summary>
    private static IEnumerable<string> Names([DynamicallyAccessedMembers(Members)] Type type) =>
        Properties(type).Select(p => p.Name).Concat(Methods(type).Select(m => m.Name + "()")).Distinct().Order();
}
