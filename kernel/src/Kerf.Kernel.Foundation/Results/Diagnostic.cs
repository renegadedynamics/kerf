// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Runtime.CompilerServices;
using System.Text.RegularExpressions;

namespace Kerf;

/// <summary>How serious a <see cref="Diagnostic"/> is.</summary>
public enum Severity
{
    /// <summary>The operation succeeded, but something deserves the caller's attention.</summary>
    Warning,

    /// <summary>The operation failed.</summary>
    Error,
}

/// <summary>
/// A diagnostic's code: a stable string for programs to test (<c>"boolean.self-interference"</c>), lower case, in
/// dot-separated words of letters, digits and hyphens. Codes are strings rather than an enum so that each area adds its
/// own (decision 1349, question 61); docs/api/diagnostics.md lists them.
/// </summary>
public readonly partial record struct DiagnosticCode
{
    /// <summary>A code; <paramref name="value"/> must have the form described on the type.</summary>
    /// <exception cref="ArgumentException">The value isn't a well-formed code.</exception>
    public DiagnosticCode(string value)
    {
        ArgumentNullException.ThrowIfNull(value);
        if (!IsPlainlyWellFormed(value) && !Format().IsMatch(value))
            throw new ArgumentException($"'{value}' isn't a diagnostic code: lower-case words of letters, digits and hyphens, separated by dots", nameof(value));
        Value = value;
    }

    /// <summary>The code's text, or null for the default code.</summary>
    public string Value { get; }

    /// <summary>The input is invalid for the operation: <c>input.invalid</c>.</summary>
    public static DiagnosticCode InvalidInput { get; } = new("input.invalid");

    /// <summary>An iterative computation didn't converge: <c>numerics.not-converged</c>.</summary>
    public static DiagnosticCode NotConverged { get; } = new("numerics.not-converged");

    /// <summary>The operation was cancelled: <c>operation.cancelled</c>.</summary>
    public static DiagnosticCode Cancelled { get; } = new("operation.cancelled");

    /// <summary>A result that was never set (a default <see cref="Result{T}"/>): <c>result.default</c>.</summary>
    public static DiagnosticCode DefaultResult { get; } = new("result.default");

    /// <summary>The code's text.</summary>
    public override string ToString() => Value ?? "";

    [GeneratedRegex("^[a-z0-9]+(-[a-z0-9]+)*(\\.[a-z0-9]+(-[a-z0-9]+)*)*$")]
    private static partial Regex Format();

    // What that pattern accepts, short of its '$' allowing a final newline. The well-known codes are made when this type is
    // first touched, so this keeps regular expressions out of opening a workbook (unit 11C).
    private static bool IsPlainlyWellFormed(string value)
    {
        char previous = '.';
        foreach (char c in value)
        {
            bool isWordCharacter = c is (>= 'a' and <= 'z') or (>= '0' and <= '9');
            if (!isWordCharacter && (c is not ('.' or '-') || previous is '.' or '-'))
            {
                return false;
            }
            previous = c;
        }
        return previous is not ('.' or '-');
    }
}

/// <summary>
/// Something an operation reports: a warning, or the error that made it fail. It has a stable <see cref="Code"/> for
/// programs, a plain <see cref="Message"/> for people and, through Kerf.Kernel.ModelingData's <c>Shapes</c>, the
/// sub-shapes it concerns.
/// </summary>
public sealed record Diagnostic
{
    /// <summary>A warning or an error.</summary>
    public required Severity Severity
    {
        get;
        init => field = Enum.IsDefined(value) ? value : throw new ArgumentOutOfRangeException(nameof(value));
    }

    /// <summary>The code, stable for programs.</summary>
    public required DiagnosticCode Code
    {
        get;
        init => field = value.Value is null ? throw new ArgumentException("A diagnostic needs a code", nameof(value)) : value;
    }

    /// <summary>What happened, in plain English.</summary>
    public required string Message
    {
        get;
        init => field = value ?? throw new ArgumentNullException(nameof(value));
    }

    /// <summary>
    /// The sub-shapes concerned (an <c>ImmutableArray&lt;Shape&gt;</c>, read and set through Kerf.Kernel.ModelingData's
    /// <c>Shapes</c> and <c>WithShapes</c>: Foundation can't name shapes), or null.
    /// </summary>
    internal object? ShapeData { get; init; }

    /// <summary>An error.</summary>
    public static Diagnostic Error(DiagnosticCode code, string message) =>
        new() { Severity = Severity.Error, Code = code, Message = message };

    /// <summary>A warning.</summary>
    public static Diagnostic Warning(DiagnosticCode code, string message) =>
        new() { Severity = Severity.Warning, Code = code, Message = message };

    /// <summary>
    /// The error for an exception the kernel threw: its kind of failure, in words, and its message, with the code the
    /// operations report it under (<c>engine.construction-error</c>, say), as a failed result carries it.
    /// </summary>
    /// <remarks>The operations turn the engine's failures into results; the values and evaluators refuse misuse (a
    /// direction of zero length) with .NET's exceptions, and a caller that catches one can report it the same way.</remarks>
    /// <exception cref="ArgumentNullException"><paramref name="exception"/> is null.</exception>
    public static Diagnostic FromException(Exception exception) => Kerf.Engine.Foundation.FailureDiagnostics.ToDiagnostic(exception);

    /// <summary>
    /// Whether an exception is a refusal of the kernel's: the .NET exception a value or evaluator throws on misuse, as
    /// opposed to one that didn't come from the kernel's checks.
    /// </summary>
    public static bool IsKernelFailure(Exception exception) => Kerf.Engine.Foundation.Refusal.IsFailure(exception);

    /// <summary>Whether this is an error.</summary>
    public bool IsError => Severity == Severity.Error;

    /// <summary>The diagnostic as a compiler would print it: <c>error boolean.self-interference: …</c>.</summary>
    public override string ToString() => $"{(IsError ? "error" : "warning")} {Code}: {Message}";

    /// <summary>The diagnostic of a default <see cref="Result{T}"/>, which was never given a value or an error.</summary>
    internal static Diagnostic DefaultResult { get; } =
        Error(DiagnosticCode.DefaultResult, "The result was never set: it is a default value, neither a success nor a failure.");

    /// <summary>The first error among <paramref name="diagnostics"/>, or null.</summary>
    internal static Diagnostic? FirstError(ImmutableArray<Diagnostic> diagnostics)
    {
        foreach (var diagnostic in diagnostics)
            if (diagnostic.IsError)
                return diagnostic;
        return null;
    }

    /// <summary>
    /// The exception for reading a failed result's value, a programmer's error: its message is the first error's code
    /// and message, saying how many more errors there are.
    /// </summary>
    internal static InvalidOperationException NoValue(ImmutableArray<Diagnostic> diagnostics)
    {
        var first = FirstError(diagnostics)!;
        var errors = diagnostics.Count(d => d.IsError);
        var more = errors switch
        {
            1 => "",
            2 => " (and 1 more error)",
            _ => $" (and {errors - 1} more errors)",
        };
        return new InvalidOperationException(
            $"The result is a failure, with no value: {first.Code}: {first.Message}{more}. Check IsSuccess, or use TryGetValue, Match or the combinators.");
    }

    /// <summary>Checks that every diagnostic is a warning (for a success) or that there is an error (for a failure).</summary>
    internal static ImmutableArray<Diagnostic> Check(ImmutableArray<Diagnostic> diagnostics, bool success,
        [CallerArgumentExpression(nameof(diagnostics))] string? name = null)
    {
        if (diagnostics.IsDefault)
            diagnostics = [];
        foreach (var diagnostic in diagnostics)
            if (diagnostic is null)
                throw new ArgumentException("A diagnostic is null", name);
        var error = FirstError(diagnostics);
        if (success && error is not null)
            throw new ArgumentException($"A success can't carry an error ({error})", name);
        if (!success && error is null)
            throw new ArgumentException("A failure needs an error", name);
        return diagnostics;
    }
}
