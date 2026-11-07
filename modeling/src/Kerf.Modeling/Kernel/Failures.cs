using System.Collections.Immutable;
using Kerf.Healing;

namespace Kerf.Modeling;

/// <summary>
/// What the modelling layer was doing when the kernel failed, as the start of the message ("box 0.0 × 1.0 × 1.0 failed: …"),
/// which Bento shows in a cell. It goes through the kernel's diagnostics.
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
/// value would. The modelling layer's own misuse errors (build123d's ArgumentException and
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
    private const string InvalidKey = "Kerf.Modeling.Invalid";

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
        result.TryGetValue(out var value) ? value : throw In(what(), Failed(result.Diagnostics).Carrying(result));

    /// <summary>As <see cref="Get{T}"/>, raising the operation's warnings (<see cref="Warnings"/>).</summary>
    internal static T GetWarned<T>(Result<T> result, Func<string> what)
    {
        var value = Get(result, what);
        foreach (var diagnostic in result.Diagnostics)
            if (!diagnostic.IsError)
                Warnings.Warn(diagnostic.Message);
        return value;
    }

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
                                                       Exception? inner = null, params ReadOnlySpan<Shape> shapes)
    {
        var error = Diagnostic.Error(code, message);
        if (!shapes.IsEmpty)
            error = error.WithShapes(shapes);
        return Failed([error, .. kernel.IsDefault ? [] : kernel], inner);
    }

    /// <summary>
    /// A failure the modelling layer detects from a kernel result: as <see cref="Expected(DiagnosticCode, string, ImmutableArray{Diagnostic}, Exception?, ReadOnlySpan{Shape})"/>,
    /// or, if the kernel made something invalid, the kernel's <c>result.invalid</c> with this advice after it, carrying the shape
    /// (decision 1742).
    /// </summary>
    internal static InvalidOperationException Expected<T>(DiagnosticCode code, string message, Result<T> kernel)
    {
        if (!kernel.TryGetInvalid(out _))
            return Expected(code, message, kernel.Diagnostics);
        var diagnostics = kernel.Diagnostics;
        var error = diagnostics.First(d => d.IsError);
        return Failed(diagnostics.Replace(error, error with { Message = $"{error.Message} {message}" })).Carrying(kernel);
    }

    /// <summary>
    /// The failure for <paramref name="shape"/>, which isn't valid: <c>result.invalid</c>, saying what's wrong with what
    /// the operation made (<paramref name="what"/>), carrying it (decision 1742).
    /// </summary>
    internal static InvalidOperationException Invalid(Shape shape, ValidationReport report, string what, string? advice = null)
    {
        var error = report.ToDiagnostic(what);
        if (advice is not null)
            error = error with { Message = $"{error.Message} {advice}" };
        var failure = Failed([error]);
        failure.Data[InvalidKey] = shape;
        return failure;
    }

    /// <summary>The failure for <paramref name="shape"/>, which is wrong for the reason <paramref name="error"/> gives, carrying it (decision 1860).</summary>
    internal static InvalidOperationException Wrong(Shape shape, Diagnostic error)
    {
        var failure = Failed([error]);
        failure.Data[InvalidKey] = shape;
        return failure;
    }

    /// <summary><paramref name="shape"/> if it's valid, or the failure of <see cref="Invalid"/>.</summary>
    internal static T Valid<T>(T shape, string what, string? advice = null) where T : struct, IShape<T>
    {
        var report = shape.Validate();
        return report.IsValid ? shape : throw Invalid(shape.AsShape(), report, what, advice);
    }

    /// <summary>The shape a failure carries, if it does.</summary>
    internal static Shape? InvalidOf(Exception e) => e.Data[InvalidKey] is Shape shape ? shape : null;

    // This failure with the invalid shape the kernel's result carries, if it does
    private static InvalidOperationException Carrying<T>(this InvalidOperationException failure, Result<T> kernel)
    {
        if (kernel.TryGetInvalid(out var invalid) && ShapeOf(invalid) is { } shape)
            failure.Data[InvalidKey] = shape;
        return failure;
    }

    private static Shape? ShapeOf<T>(T value) => value switch
    {
        Built<Shape> built => built.Shape,
        Built<Solid> built => built.Shape,
        Built<Shell> built => built.Shape,
        Built<Face> built => built.Shape,
        Kerf.Features.SplitFacesResult split => split.Shape,
        Shape shape => shape,
        Solid solid => solid,
        Shell shell => shell,
        Face face => face,
        _ => null,
    };

    /// <summary>A failure's value as the type a public operation returns, for <see cref="Result{T}.TryGetInvalid"/>.</summary>
    private static bool TryWrap<T>(Shape shape, out T value)
    {
        object? wrapped = default(T) switch
        {
            Shape => shape,
            Solid when shape.TryAs(out Solid solid) => solid,
            Shell when shape.TryAs(out Shell shell) => shell,
            Face when shape.TryAs(out Face face) => face,
            _ => null,
        };
        if (wrapped is null && !typeof(T).IsValueType)
        {
            if (typeof(T) == typeof(Part) || (typeof(T) == typeof(ModelShape) && TopoUtil.Dim(shape) == 3))
                wrapped = new Part(shape);
            else if (typeof(T) == typeof(Sketch) || (typeof(T) == typeof(ModelShape) && TopoUtil.Dim(shape) == 2))
                wrapped = new Sketch(shape);
            else if (typeof(T) == typeof(Curve) || typeof(T) == typeof(ModelShape))
                wrapped = new Curve(shape);
        }
        value = wrapped is T typed ? typed : default!;
        return wrapped is T;
    }

    /// <summary>
    /// A result of an object narrowed to its class, keeping the shape a failure carries: <c>Map</c> doesn't pass that on.
    /// </summary>
    internal static Result<T> Narrowed<T>(this Result<ModelShape> result) where T : ModelShape =>
        result.TryGetInvalid(out var invalid) && TryWrap(invalid.Shape, out T narrowed)
            ? Result<T>.Invalid(narrowed, result.Diagnostics)
            : result.Map(c => c.As<T>());

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
            return InvalidOf(e) is { } invalid && TryWrap(invalid, out T carried)
                ? Result<T>.Invalid(carried, DiagnosticsOf(e))
                : Result<T>.Failure(DiagnosticsOf(e));
        }
        catch (Exception e) when (e is not OperationCanceledException && IsKernelFailure(e))
        {
            return Result<T>.Failure(DiagnosticsOf(In("operation", e)));
        }
    }

    /// <summary>As <see cref="Try{T}"/>, with the warnings raised meanwhile (<see cref="Warnings"/>) on a successful result.</summary>
    internal static Result<T> TryWarned<T>(DiagnosticCode code, Func<T> operation)
    {
        using var warnings = Warnings.Capture();
        var result = Try(operation);
        return result.TryGetValue(out var value) && warnings.Messages.Count > 0
            ? Result<T>.Success(value, [.. warnings.Messages.Select(m => Diagnostic.Warning(code, m))])
            : result;
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
        if (InvalidOf(failure) is { } invalid)
            exception.Data[InvalidKey] = invalid;
        exception.Data[OperationKey] = operation;
        exception.Data[CauseKey] = cause;
        return exception;
    }
}
