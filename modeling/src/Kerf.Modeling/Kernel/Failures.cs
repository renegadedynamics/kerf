using System.Collections.Immutable;

namespace Kerf.Modeling;

/// <summary>
/// What the modelling layer was doing when the kernel failed. The native backend's messages started with it ("box 0.0 ×
/// 1.0 × 1.0 failed: …"), which Bento shows in a cell; unit 10B lost it with the forwarding layer (decision 1342), and
/// unit 10F restores it through the kernel's diagnostics.
/// </summary>
/// <remarks>
/// <para>
/// An operation that can fail in ways a caller should expect (a primitive with a zero size, a Boolean, a loft, a sweep)
/// runs its kernel calls inside <see cref="Doing{T}"/>. A kernel failure escaping them becomes a
/// <see cref="KerfException"/>: its diagnostics are the kernel's (the engine exception's, or a Boolean's alerts, with
/// their codes and sub-shapes), each error's message prefixed with what was being done, and its inner exception is the
/// kernel's own. Operations nest: a failure inside a fuse inside an extrusion says <c>extrude / fuse of … failed: …</c>.
/// </para>
/// <para>
/// Until unit 10M gives the modelling layer results, this is <c>Result&lt;T&gt;.Value</c>'s behaviour: the exception a
/// caller who takes a failed result's value gets. The modelling layer's own errors (build123d's ArgumentException and
/// InvalidOperationException, as a fillet's) keep their types and messages.
/// </para>
/// </remarks>
internal static class Failures
{
    // What a KerfException this class made was doing, and its diagnostics before the prefix, so an outer operation can
    // say both: "extrude / fuse failed"
    private const string OperationKey = "Kerf.Modeling.Operation";
    private const string CauseKey = "Kerf.Modeling.Cause";

    /// <summary>
    /// Runs <paramref name="operation"/>, turning a kernel failure into a <see cref="KerfException"/> that says what was
    /// being done (<paramref name="what"/>, only formatted on failure).
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
    internal static bool IsKernelFailure(Exception e) => e is KerfException || Diagnostic.IsKernelFailure(e) || IsKernelArgumentError(e);

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

    private static readonly DiagnosticCode InvalidArgument = new("kernel.invalid-argument");

    /// <summary>
    /// A kernel operation's value, or, if it failed, a <see cref="KerfException"/> with its diagnostics that says what was
    /// being done (<paramref name="what"/>, only formatted on failure). Its warnings aren't read.
    /// </summary>
    internal static T Get<T>(Result<T> result, Func<string> what) =>
        result.TryGetValue(out var value) ? value : throw In(what(), new KerfException(result.Diagnostics));

    /// <summary>A kernel operation that returns no value: done, or a <see cref="KerfException"/> as <see cref="Get{T}"/>.</summary>
    internal static void Check(Result result, Func<string> what)
    {
        if (result.IsFailure)
            throw In(what(), new KerfException(result.Diagnostics));
    }

    /// <summary>The exception for a kernel failure met while doing <paramref name="what"/>.</summary>
    internal static KerfException In(string what, Exception failure)
    {
        string operation = what;
        ImmutableArray<Diagnostic> cause;
        Exception kernel;
        if (failure is KerfException known)
        {
            // An operation that reported its own failure in its own name isn't named twice ("loft / loft")
            if (known.Data[OperationKey] is string inner && inner != what)
                operation = $"{what} / {inner}";
            cause = known.Data[CauseKey] is ImmutableArray<Diagnostic> unprefixed ? unprefixed : known.Diagnostics;
            kernel = known.InnerException ?? known;
        }
        else
        {
            cause = [failure is ArgumentException argument ? ArgumentDiagnostic(argument) : Diagnostic.FromException(failure)];
            kernel = failure;
        }
        var diagnostics = cause.Select(d => d.IsError ? d with { Message = $"{operation} failed: {d.Message}" } : d).ToImmutableArray();
        var exception = new KerfException(diagnostics, kernel);
        exception.Data[OperationKey] = operation;
        exception.Data[CauseKey] = cause;
        return exception;
    }
}
