using System.Collections.Immutable;
using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>
/// What the modelling layer was doing when the kernel failed. The native backend's messages started with it ("box 0.0 ×
/// 1.0 × 1.0 failed: …"), which Bento shows in a cell; unit 10B lost it with the forwarding layer (decision 1342), and
/// unit 10F restores it through the kernel's diagnostics.
/// </summary>
/// <remarks>
/// <para>
/// An operation that can fail in ways a caller should expect (a primitive with a zero size, a Boolean, a loft, a sweep)
/// runs its kernel calls inside <see cref="Doing{T}"/>. A kernel failure escaping them becomes a failure: its
/// diagnostics are the kernel's (the engine exception's, or a Boolean's alerts, with their codes and sub-shapes), each
/// error's message prefixed with what was being done. Operations nest: a failure inside a fuse inside an extrusion says
/// <c>extrude / fuse of … failed: …</c>.
/// </para>
/// <para>
/// Inside the layer a failure travels as an <see cref="InvalidOperationException"/> whose <see cref="Exception.Data"/>
/// holds its diagnostics (<see cref="Failed"/>), its message the first error's, its inner exception the kernel's own.
/// It is .NET's exception, not one of Kerf's: the public operations that can fail as a caller should expect
/// (<see cref="Operations"/>, the algebra's <c>Add</c>, <c>Subtract</c> and <c>Common</c>, the importers) catch it with
/// <see cref="Try{T}"/> and return a failed <see cref="Result{T}"/>; the shorthands that can't return a result (the
/// operators, the objects' constructors, the topological shapes' methods) let it escape, as reading a failed result's
/// value would (unit 10M3). The modelling layer's own misuse errors (build123d's ArgumentException and
/// InvalidOperationException) keep their types and messages.
/// </para>
/// </remarks>
internal static class Failures
{
    // A failure's diagnostics; what it was doing, and its diagnostics before the prefix, so an outer operation can say
    // both: "extrude / fuse failed"
    private const string DiagnosticsKey = "Kerf.Modeling.Diagnostics";
    private const string OperationKey = "Kerf.Modeling.Operation";
    private const string CauseKey = "Kerf.Modeling.Cause";

    /// <summary>
    /// Runs <paramref name="operation"/>, turning a kernel failure into a failure (<see cref="Failed"/>) that says what
    /// was being done (<paramref name="what"/>, only formatted on failure).
    /// </summary>
    internal static T Doing<T>(Func<string> what, Func<T> operation)
    {
        try
        {
            return operation();
        }
        catch (Exception e) when (IsKernelFailure(e))
        {
            throw In(what(), e);
        }
    }

    /// <inheritdoc cref="Doing{T}(Func{string}, Func{T})"/>
    internal static void Doing(Func<string> what, Action operation)
    {
        try
        {
            operation();
        }
        catch (Exception e) when (IsKernelFailure(e))
        {
            throw In(what(), e);
        }
    }

    /// <summary>
    /// Whether an exception is a failure of the kernel's (its own exceptions, or one this class made from them), as the
    /// modelling layer's best-effort paths catch.
    /// </summary>
    internal static bool IsKernelFailure(Exception e) => IsFailure(e) || Diagnostic.IsKernelFailure(e) || IsKernelArgumentError(e);

    /// <summary>Whether an exception is a failure this class made (<see cref="Failed"/>).</summary>
    internal static bool IsFailure(Exception e) => e.Data[DiagnosticsKey] is ImmutableArray<Diagnostic>;

    /// <summary>A failure's diagnostics (<see cref="Failed"/>), or none if the exception isn't one.</summary>
    internal static ImmutableArray<Diagnostic> DiagnosticsOf(Exception e) =>
        e.Data[DiagnosticsKey] is ImmutableArray<Diagnostic> diagnostics ? diagnostics : [];

    /// <summary>
    /// A failure with <paramref name="diagnostics"/> (at least one an error), as the layer carries one to the public
    /// operation that returns it as a result: an <see cref="InvalidOperationException"/> whose message is the first
    /// error's, saying how many more there are.
    /// </summary>
    internal static InvalidOperationException Failed(ImmutableArray<Diagnostic> diagnostics, Exception? inner = null)
    {
        diagnostics = Result<int>.Failure(diagnostics).Diagnostics;
        var errors = diagnostics.Count(d => d.IsError);
        var first = diagnostics.First(d => d.IsError).Message;
        var message = errors switch
        {
            1 => first,
            2 => $"{first} (and 1 more error)",
            _ => $"{first} (and {errors - 1} more errors)",
        };
        var exception = new InvalidOperationException(message, inner);
        exception.Data[DiagnosticsKey] = diagnostics;
        return exception;
    }

    // The kernel's public API checks its arguments first (ArgumentException) where OCCT's own checks came later, as its
    // failures (Standard_DomainError): an argument the kernel refuses is the kernel's failure, said in context as one
    private static bool IsKernelArgumentError(Exception e)
    {
        if (e is not ArgumentException)
            return false;
        foreach (var frame in new System.Diagnostics.StackTrace(e).GetFrames())
        {
            var assembly = System.Diagnostics.DiagnosticMethodInfo.Create(frame)?.DeclaringAssemblyName is { } full
                ? new System.Reflection.AssemblyName(full).Name
                : null;
            if (assembly is null || assembly.StartsWith("System.", StringComparison.Ordinal))
                continue;
            return assembly.StartsWith("Kerf.Kernel.", StringComparison.Ordinal);
        }
        return false;
    }

    /// <summary>A refused argument's diagnostic: <c>kernel.invalid-argument</c>, with the kernel's message.</summary>
    private static Diagnostic ArgumentDiagnostic(ArgumentException e)
    {
        var message = e.ParamName is { } name ? e.Message.Replace($" (Parameter '{name}')", "", StringComparison.Ordinal) : e.Message;
        var newline = message.IndexOf('\n');
        return Diagnostic.Error(InvalidArgument, newline < 0 ? message : message[..newline].TrimEnd());
    }

    private static DiagnosticCode InvalidArgument => ModelingDiagnostics.KernelInvalidArgument;

    /// <summary>
    /// A kernel operation's value, or, if it failed, a failure with its diagnostics that says what was being done
    /// (<paramref name="what"/>, only formatted on failure). Its warnings aren't read.
    /// </summary>
    internal static T Get<T>(Result<T> result, Func<string> what) =>
        result.TryGetValue(out var value) ? value : throw In(what(), Failed(result.Diagnostics));

    /// <summary>A kernel operation that returns no value: done, or a failure as <see cref="Get{T}"/>.</summary>
    internal static void Check(Result result, Func<string> what)
    {
        if (result.IsFailure)
            throw In(what(), Failed(result.Diagnostics));
    }

    /// <summary>
    /// A failure the modelling layer detects that a caller should expect (a fillet that can't be built): an error with
    /// <paramref name="code"/> and <paramref name="message"/>, then the kernel's diagnostics that led to it, if any.
    /// </summary>
    internal static InvalidOperationException Expected(DiagnosticCode code, string message, ImmutableArray<Diagnostic> kernel = default,
                                                       Exception? inner = null, params ReadOnlySpan<global::Kerf.Topology.Shape> shapes)
    {
        var error = Diagnostic.Error(code, message);
        if (!shapes.IsEmpty)
            error = error.WithShapes(shapes);
        return Failed([error, .. kernel.IsDefault ? [] : kernel], inner);
    }

    /// <summary>
    /// Runs an operation, giving its value, or its failure as a result: a failure's diagnostics (<see cref="Failed"/>),
    /// or a kernel failure that escaped as another exception. Misuse (the modelling layer's own argument checks) and
    /// cancellation still throw.
    /// </summary>
    internal static Result<T> Try<T>(Func<T> operation)
    {
        try
        {
            return Result<T>.Success(operation());
        }
        catch (Exception e) when (IsFailure(e))
        {
            return Result<T>.Failure(DiagnosticsOf(e));
        }
        catch (Exception e) when (e is not OperationCanceledException && IsKernelFailure(e))
        {
            return Result<T>.Failure(DiagnosticsOf(In("operation", e)));
        }
    }

    /// <summary>The failure for a kernel failure met while doing <paramref name="what"/>.</summary>
    internal static InvalidOperationException In(string what, Exception failure)
    {
        string operation = what;
        ImmutableArray<Diagnostic> cause;
        Exception kernel;
        if (IsFailure(failure))
        {
            // An operation that reported its own failure in its own name isn't named twice ("loft / loft")
            if (failure.Data[OperationKey] is string inner && inner != what)
                operation = $"{what} / {inner}";
            cause = failure.Data[CauseKey] is ImmutableArray<Diagnostic> unprefixed ? unprefixed : DiagnosticsOf(failure);
            kernel = failure.InnerException ?? failure;
        }
        else
        {
            cause = [failure is ArgumentException argument && !Diagnostic.IsKernelFailure(failure) ? ArgumentDiagnostic(argument) : Diagnostic.FromException(failure)];
            kernel = failure;
        }
        var diagnostics = cause.Select(d => d.IsError ? d with { Message = $"{operation} failed: {d.Message}" } : d).ToImmutableArray();
        var exception = Failed(diagnostics, kernel);
        exception.Data[OperationKey] = operation;
        exception.Data[CauseKey] = cause;
        return exception;
    }
}
