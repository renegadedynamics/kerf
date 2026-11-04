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
/// <para>
/// <c>default(Result&lt;T&gt;)</c> is a failure whose error has the code <see cref="DiagnosticCode.DefaultResult"/>.
/// </para>
/// <para>
/// A failure is read, not caught: check <see cref="IsSuccess"/>, or use <see cref="TryGetValue"/>, <see cref="Match{TOut}"/>,
/// <see cref="GetValueOrDefault"/> or the combinators (<see cref="Map{TOut}"/>, <see cref="Bind{TOut}"/>, and
/// <see cref="Select{TOut}"/> and <see cref="SelectMany{TNext, TOut}"/> for LINQ's query syntax), which carry a failure
/// through untouched. Reading <see cref="Value"/> of a failure is a programmer's error, as reading a null
/// <see cref="Nullable{T}"/>'s is: it throws <see cref="InvalidOperationException"/> (unit 10M3).
/// </para>
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

    /// <summary>
    /// The value of a success. Check <see cref="IsSuccess"/> first, or use <see cref="TryGetValue"/>,
    /// <see cref="Match{TOut}"/> or the combinators: a failure has no value.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The operation failed (a programmer's error); the message gives the first error's code and message.
    /// </exception>
    public T Value => IsSuccess ? _value : throw Diagnostic.NoValue(Diagnostics);

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

    /// <summary>
    /// <paramref name="success"/> of the value, or <paramref name="failure"/> of the first error: one value either way,
    /// as a cell shows a shape or an error.
    /// </summary>
    public TOut Match<TOut>(Func<T, TOut> success, Func<Diagnostic, TOut> failure)
    {
        ArgumentNullException.ThrowIfNull(success);
        ArgumentNullException.ThrowIfNull(failure);
        return IsSuccess ? success(_value) : failure(Error!);
    }

    /// <summary><see cref="Map{TOut}"/> under LINQ's name, for query syntax (<c>from part in … select …</c>).</summary>
    public Result<TOut> Select<TOut>(Func<T, TOut> selector) => Map(selector);

    /// <summary>
    /// <see cref="Bind{TOut}"/> then <see cref="Map{TOut}"/>, for query syntax with several <c>from</c> clauses:
    /// <c>from box in Fillet(…) from part in box.Subtract(hole) select part</c>. The first failure stops the query.
    /// </summary>
    public Result<TOut> SelectMany<TNext, TOut>(Func<T, Result<TNext>> next, Func<T, TNext, TOut> selector)
    {
        ArgumentNullException.ThrowIfNull(next);
        ArgumentNullException.ThrowIfNull(selector);
        return Bind(value => next(value).Map(nextValue => selector(value, nextValue)));
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

    /// <summary><paramref name="success"/>, or <paramref name="failure"/> of the first error.</summary>
    public TOut Match<TOut>(Func<TOut> success, Func<Diagnostic, TOut> failure)
    {
        ArgumentNullException.ThrowIfNull(success);
        ArgumentNullException.ThrowIfNull(failure);
        return IsSuccess ? success() : failure(Error!);
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
