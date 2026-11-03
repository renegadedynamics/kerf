// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Diagnostics.CodeAnalysis;

namespace Kerf;

/// <summary>
/// The result of an operation that can fail in a way the caller should expect (a fillet that can't be built, a file
/// that can't be read): a value, or the error that stopped it, with any warnings either way (phase10-plan.md 2.5).
/// Misuse, such as a null argument, throws .NET's exceptions instead, and cancellation throws
/// <see cref="OperationCanceledException"/>.
/// </summary>
/// <remarks>
/// <c>default(Result&lt;T&gt;)</c> is a failure whose error has the code <see cref="DiagnosticCode.DefaultResult"/>.
/// For callers who prefer exceptions, <see cref="Value"/> and <see cref="GetValueOrThrow"/> throw a
/// <see cref="KerfException"/> carrying the diagnostics.
/// </remarks>
public readonly struct Result<T>
{
    private readonly T _value;
    private readonly ImmutableArray<Diagnostic> _diagnostics;

    private Result(bool isSuccess, T value, ImmutableArray<Diagnostic> diagnostics)
    {
        IsSuccess = isSuccess;
        _value = value;
        _diagnostics = diagnostics;
    }

    /// <summary>Whether the operation succeeded.</summary>
    public bool IsSuccess { get; }

    /// <summary>Whether the operation failed.</summary>
    public bool IsFailure => !IsSuccess;

    /// <summary>The value.</summary>
    /// <exception cref="KerfException">The operation failed; the exception carries the diagnostics.</exception>
    public T Value => IsSuccess ? _value : throw new KerfException(Diagnostics);

    /// <summary>The warnings, and on failure the error (or errors).</summary>
    public ImmutableArray<Diagnostic> Diagnostics => _diagnostics.IsDefault ? [Diagnostic.DefaultResult] : _diagnostics;

    /// <summary>The first error, or null on success.</summary>
    public Diagnostic? Error => IsSuccess ? null : Diagnostic.FirstError(Diagnostics);

    /// <summary>The value, if the operation succeeded.</summary>
    public bool TryGetValue([MaybeNullWhen(false)] out T value)
    {
        value = _value;
        return IsSuccess;
    }

    /// <summary>The value; the same as <see cref="Value"/>, for readers who want the throw spelt out.</summary>
    /// <exception cref="KerfException">The operation failed; the exception carries the diagnostics.</exception>
    public T GetValueOrThrow() => Value;

    /// <summary>The value, or <paramref name="fallback"/> if the operation failed.</summary>
    public T GetValueOrDefault(T fallback) => IsSuccess ? _value : fallback;

    /// <summary>The value passed through <paramref name="map"/>, keeping the warnings; a failure stays one.</summary>
    public Result<TOut> Map<TOut>(Func<T, TOut> map)
    {
        ArgumentNullException.ThrowIfNull(map);
        return IsSuccess ? new(true, map(_value), _diagnostics) : Result<TOut>.Carry(Diagnostics);
    }

    /// <summary>
    /// The result of <paramref name="next"/> on the value, with this result's warnings before its own; a failure stays
    /// one, and <paramref name="next"/> isn't called.
    /// </summary>
    public Result<TOut> Bind<TOut>(Func<T, Result<TOut>> next)
    {
        ArgumentNullException.ThrowIfNull(next);
        if (!IsSuccess)
            return Result<TOut>.Carry(Diagnostics);
        var then = next(_value);
        return _diagnostics.IsEmpty ? then : then.After(_diagnostics);
    }

    /// <summary>A success, with any warnings.</summary>
    /// <exception cref="ArgumentException">A diagnostic in <paramref name="warnings"/> is an error.</exception>
    public static Result<T> Success(T value, ImmutableArray<Diagnostic> warnings = default) =>
        new(true, value, Diagnostic.Check(warnings, success: true));

    /// <summary>A failure, with any warnings met before it.</summary>
    /// <exception cref="ArgumentException"><paramref name="error"/> isn't an error.</exception>
    public static Result<T> Failure(Diagnostic error, ImmutableArray<Diagnostic> warnings = default)
    {
        ArgumentNullException.ThrowIfNull(error);
        if (!error.IsError)
            throw new ArgumentException($"A failure needs an error, not a {error.Severity}", nameof(error));
        return new(false, default!, [.. Diagnostic.Check(warnings, success: true), error]);
    }

    /// <summary>A failure with several diagnostics, at least one of them an error.</summary>
    /// <exception cref="ArgumentException">None of <paramref name="diagnostics"/> is an error.</exception>
    public static Result<T> Failure(ImmutableArray<Diagnostic> diagnostics) =>
        new(false, default!, Diagnostic.Check(diagnostics, success: false));

    /// <summary>A success.</summary>
    public static implicit operator Result<T>(T value) => new(true, value, []);

    /// <summary>The value or the error, for debugging.</summary>
    public override string ToString() => IsSuccess ? $"Success: {_value}" : $"Failure: {Error}";

    // A failure's diagnostics carried into a result of another type (already checked)
    internal static Result<T> Carry(ImmutableArray<Diagnostic> diagnostics) => new(false, default!, diagnostics);

    // This result with earlier warnings before its own diagnostics
    private Result<T> After(ImmutableArray<Diagnostic> warnings) => new(IsSuccess, _value, [.. warnings, .. Diagnostics]);
}

/// <summary>
/// The result of an operation that returns nothing but can fail as callers should expect (a writer): success, or the
/// error that stopped it, with any warnings either way. <c>default(Result)</c> is a failure, as for
/// <see cref="Result{T}"/>.
/// </summary>
public readonly struct Result
{
    private readonly ImmutableArray<Diagnostic> _diagnostics;

    private Result(bool isSuccess, ImmutableArray<Diagnostic> diagnostics)
    {
        IsSuccess = isSuccess;
        _diagnostics = diagnostics;
    }

    /// <summary>Whether the operation succeeded.</summary>
    public bool IsSuccess { get; }

    /// <summary>Whether the operation failed.</summary>
    public bool IsFailure => !IsSuccess;

    /// <summary>The warnings, and on failure the error (or errors).</summary>
    public ImmutableArray<Diagnostic> Diagnostics => _diagnostics.IsDefault ? [Diagnostic.DefaultResult] : _diagnostics;

    /// <summary>The first error, or null on success.</summary>
    public Diagnostic? Error => IsSuccess ? null : Diagnostic.FirstError(Diagnostics);

    /// <summary>Throws if the operation failed.</summary>
    /// <exception cref="KerfException">The operation failed; the exception carries the diagnostics.</exception>
    public void ThrowIfFailed()
    {
        if (!IsSuccess)
            throw new KerfException(Diagnostics);
    }

    /// <summary>The value <paramref name="next"/> gives after a success, keeping the warnings; a failure stays one.</summary>
    public Result<T> Map<T>(Func<T> next)
    {
        ArgumentNullException.ThrowIfNull(next);
        return IsSuccess ? Result<T>.Success(next(), _diagnostics) : Result<T>.Carry(Diagnostics);
    }

    /// <summary>The result of <paramref name="next"/> after a success, with this result's warnings before its own.</summary>
    public Result Bind(Func<Result> next)
    {
        ArgumentNullException.ThrowIfNull(next);
        if (!IsSuccess)
            return this;
        var then = next();
        return _diagnostics.IsEmpty ? then : new(then.IsSuccess, [.. _diagnostics, .. then.Diagnostics]);
    }

    /// <summary>A success, with any warnings.</summary>
    /// <exception cref="ArgumentException">A diagnostic in <paramref name="warnings"/> is an error.</exception>
    public static Result Success(ImmutableArray<Diagnostic> warnings = default) =>
        new(true, Diagnostic.Check(warnings, success: true));

    /// <summary>A failure, with any warnings met before it.</summary>
    /// <exception cref="ArgumentException"><paramref name="error"/> isn't an error.</exception>
    public static Result Failure(Diagnostic error, ImmutableArray<Diagnostic> warnings = default)
    {
        ArgumentNullException.ThrowIfNull(error);
        if (!error.IsError)
            throw new ArgumentException($"A failure needs an error, not a {error.Severity}", nameof(error));
        return new(false, [.. Diagnostic.Check(warnings, success: true), error]);
    }

    /// <summary>A failure with several diagnostics, at least one of them an error.</summary>
    /// <exception cref="ArgumentException">None of <paramref name="diagnostics"/> is an error.</exception>
    public static Result Failure(ImmutableArray<Diagnostic> diagnostics) =>
        new(false, Diagnostic.Check(diagnostics, success: false));

    /// <summary>Success or the error, for debugging.</summary>
    public override string ToString() => IsSuccess ? "Success" : $"Failure: {Error}";
}

/// <summary>
/// The exception for callers who take a failed <see cref="Result{T}"/>'s value: it carries the result's diagnostics,
/// and its message is the first error's (saying how many more there are).
/// </summary>
public sealed class KerfException : Exception
{
    /// <summary>The exception for a failure's diagnostics (at least one of them an error).</summary>
    public KerfException(ImmutableArray<Diagnostic> diagnostics, Exception? innerException = null)
        : base(Compose(diagnostics = Diagnostic.Check(diagnostics, success: false)), innerException)
    {
        Diagnostics = diagnostics;
    }

    /// <summary>The exception for one error.</summary>
    public KerfException(Diagnostic error, Exception? innerException = null)
        : this([error ?? throw new ArgumentNullException(nameof(error))], innerException)
    {
    }

    /// <summary>The warnings and the error (or errors).</summary>
    public ImmutableArray<Diagnostic> Diagnostics { get; }

    /// <summary>The first error.</summary>
    public Diagnostic Error => Diagnostic.FirstError(Diagnostics)!;

    // The first error's message, and how many more there are
    private static string Compose(ImmutableArray<Diagnostic> diagnostics)
    {
        var errors = diagnostics.Count(d => d.IsError);
        var first = Diagnostic.FirstError(diagnostics)!.Message;
        return errors switch
        {
            1 => first,
            2 => $"{first} (and 1 more error)",
            _ => $"{first} (and {errors - 1} more errors)",
        };
    }
}
