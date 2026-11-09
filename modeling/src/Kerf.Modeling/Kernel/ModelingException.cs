using System.Collections.Immutable;

namespace Kerf.Modeling;

/// <summary>
/// A failure a caller should expect, thrown where a <see cref="Result{T}"/> can't be returned: by an object's constructor
/// (<c>new Spline(points)</c>, <c>new ConvexPolyhedron(points)</c>), an operator (<c>a - b</c>) or a topological shape's
/// own method. It carries the error the <see cref="Result{T}"/> would: the same <see cref="Error"/>, with its code and
/// message, and the same <see cref="Diagnostics"/>.
/// </summary>
/// <remarks>
/// <para>
/// Each constructor that can throw it has a twin in <see cref="Operations"/> returning a <see cref="Result{T}"/>
/// (<c>MakeSpline</c>, <c>MakeConvexPolyhedron</c>, …), for values that may not make a shape (decision 2008).
/// </para>
/// <para>
/// It is an <see cref="InvalidOperationException"/>, as the layer's failures always were, so code catching that still
/// catches it. Misuse that a caller can always avoid by checking an argument on its own (a null, a count below the
/// least, a negative size the object checks, a missing optional value) still throws .NET's
/// <see cref="ArgumentException"/>s, and cancellation <see cref="OperationCanceledException"/>.
/// </para>
/// </remarks>
public sealed class ModelingException : InvalidOperationException
{
    internal ModelingException(string message, ImmutableArray<Diagnostic> diagnostics, Exception? inner)
        : base(message, inner)
    {
        Diagnostics = diagnostics;
    }

    /// <summary>The first error, as a failed result's <see cref="Result{T}.Error"/>.</summary>
    public Diagnostic Error => Diagnostics.First(d => d.IsError);

    /// <summary>The error (or errors), and any warnings met before it, as a failed result's <see cref="Result{T}.Diagnostics"/>.</summary>
    public ImmutableArray<Diagnostic> Diagnostics { get; }

    /// <summary>
    /// What the operation made that isn't valid (<c>result.invalid</c>), if it kept it, as a failed result's
    /// <see cref="Result{T}.TryGetInvalid"/>: to look at or to draw, not to build on (decision 1742).
    /// </summary>
    public bool TryGetInvalid(out Shape invalid)
    {
        invalid = Invalid ?? default;
        return Invalid is not null;
    }

    // The shape it carries, if any; what was being done, and the diagnostics before that was said, so an outer operation
    // can say both ("extrude / fuse failed"): set by Failures
    internal Shape? Invalid { get; set; }

    internal string? Operation { get; set; }

    internal ImmutableArray<Diagnostic>? Cause { get; set; }
}
