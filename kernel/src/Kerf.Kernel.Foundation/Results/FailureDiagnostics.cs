// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Text;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The bridge from the engine's exceptions to diagnostics (phase10-plan.md 2.5): an exception that escapes an engine
/// algorithm becomes an error with a stable code, for the operations that report failures as results.
/// </summary>
/// <remarks>
/// <para>
/// An OCCT exception (a <see cref="FailureException"/>) has the code <c>engine.</c> followed by its OCCT class name,
/// without the package, in lower case with hyphens between words: <c>StdFail_NotDone</c> is <c>engine.not-done</c>,
/// <c>Standard_ConstructionError</c> <c>engine.construction-error</c>. Any other exception from inside the engine
/// (an index out of range, a null reference) is a fault in the port: <c>engine.fault</c>. The Booleans' own failure
/// (<c>BRepAlgoAPI_Failure</c>) is <c>boolean.failed</c>. docs/api/diagnostics.md
/// lists the codes.
/// </para>
/// <para>
/// The message says in plain words what kind of failure it was, then OCCT's own message if it has one.
/// </para>
/// </remarks>
internal static class FailureDiagnostics
{
    /// <summary>The code of a fault: an exception from the engine that isn't one of OCCT's.</summary>
    public static DiagnosticCode Fault { get; } = new("engine.fault");

    private static readonly ConcurrentDictionary<Type, DiagnosticCode> Codes = new();

    // The exceptions whose code the rule would make misleading: the Booleans' failure isn't Standard_Failure
    private static readonly Dictionary<string, string> Overrides = new(StringComparer.Ordinal)
    {
        ["BRepAlgoAPI_Failure"] = "boolean.failed",
    };

    /// <summary>The code for an exception of type <paramref name="exceptionType"/>.</summary>
    public static DiagnosticCode CodeOf(Type exceptionType)
    {
        ArgumentNullException.ThrowIfNull(exceptionType);
        if (!typeof(FailureException).IsAssignableFrom(exceptionType))
            return Fault;
        return Codes.GetOrAdd(exceptionType, static type =>
            Overrides.TryGetValue(OcctTypeNames.Of(type), out var code) ? new(code) : new("engine." + Kebab(OcctName(type))));
    }

    /// <summary>The error for an exception that escaped the engine.</summary>
    public static Diagnostic ToDiagnostic(Exception exception)
    {
        ArgumentNullException.ThrowIfNull(exception);
        exception = Refusal.Unwrapped(exception);
        var own = Refusal.Of(exception) is { } failure ? failure.GetMessageString() : exception.Message;
        // A failure raised without a message has .NET's default one, where OCCT's would be empty
        if (own.StartsWith("Exception of type '", StringComparison.Ordinal))
            own = "";
        var kind = Describe(exception);
        return Diagnostic.Error(CodeOf(exception.GetType()), own.Length == 0 ? kind : $"{kind}: {own}");
    }

    /// <summary>The plain description of a kind of failure, by the nearest OCCT family it belongs to.</summary>
    internal static string Describe(Exception exception) => exception switch
    {
        NotDoneException => "the algorithm couldn't complete",
        ConstructionErrorException => "the geometry can't be constructed",
        NullVectorException => "a vector or direction has zero length",
        TypeMismatchException => "a shape or geometry is of the wrong type",
        NullObjectException => "something needed is missing",
        NoSuchObjectException or NoMoreObjectException => "something looked for isn't there",
        RangeErrorException => "a value is out of range",
        DomainErrorException => "a value is outside what the operation accepts",
        NumericErrorException => "a numeric error",
        InfiniteSolutionsException => "there are infinitely many solutions",
        MethodNotImplementedException => "not implemented",
        StreamFormatErrorException => "the data is malformed",
        FailureException => "the operation failed",
        _ => "an internal fault",
    };

    // The OCCT class name without its package: StdFail_NotDone → NotDone
    private static string OcctName(Type type)
    {
        var name = OcctTypeNames.Of(type);
        var underscore = name.IndexOf('_');
        return underscore >= 0 ? name[(underscore + 1)..] : name;
    }

    // NotDone → not-done, BRepAlgoAPI → brepalgoapi; runs of capitals stay one word
    internal static string Kebab(string name)
    {
        var text = new StringBuilder(name.Length + 8);
        for (int i = 0; i < name.Length; i++)
        {
            char c = name[i];
            if (!char.IsLetterOrDigit(c))
            {
                if (text.Length > 0 && text[^1] != '-')
                    text.Append('-');
                continue;
            }
            bool wordStart = char.IsUpper(c) && i > 0 && text.Length > 0 && text[^1] != '-'
                && (char.IsLower(name[i - 1]) || (i + 1 < name.Length && char.IsLower(name[i + 1])));
            if (wordStart)
                text.Append('-');
            text.Append(char.ToLowerInvariant(c));
        }
        return text.ToString().Trim('-');
    }
}
