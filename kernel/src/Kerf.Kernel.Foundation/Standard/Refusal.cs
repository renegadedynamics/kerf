// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Foundation;

/// <summary>The kinds of OCCT failure a public member's guard refuses with, each the engine's exception of that name.</summary>
internal enum FailureKind
{
    ConstructionError,
    DimensionError,
    DimensionMismatch,
    DivideByZero,
    DomainError,
    NegativeValue,
    NoSuchObject,
    NullObject,
    NullValue,
    NullVector,
    OutOfRange,
    ProgramError,
    RangeError,
    TypeMismatch,
}

/// <summary>
/// How a public value, geometry or shape refuses misuse (unit 10P2): with .NET's exception, which a caller can name,
/// holding as its inner exception the OCCT failure the engine knows it by.
/// </summary>
/// <remarks>
/// <para>
/// The engine calls the same members, and OCCT's algorithms use a refusal as a status: a degenerate normal caught, and
/// the next case tried. So an engine catch takes a failure in either form (<see cref="IsFailure"/>), and the
/// diagnostics, the codes and the printed type names read the failure inside (<see cref="Of"/>), exactly as they did
/// when the failure itself was thrown.
/// </para>
/// <para>
/// A guard is free on the path that passes: the <c>…If</c> forms inline to a test, and the throw is in a helper that
/// doesn't inline, so the message and the two exceptions are made only on the way out.
/// </para>
/// </remarks>
internal static class Refusal
{
    /// <summary>Refuses an argument: <see cref="ArgumentException"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ArgumentIf(bool condition, FailureKind kind, string? message = null)
    {
        if (condition)
            ThrowArgument(kind, message);
    }

    /// <summary>Refuses an index or a value out of range: <see cref="ArgumentOutOfRangeException"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void RangeIf(bool condition, FailureKind kind, string? message = null)
    {
        if (condition)
            ThrowRange(kind, message);
    }

    /// <summary>Refuses a call the object's state doesn't allow: <see cref="InvalidOperationException"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void StateIf(bool condition, FailureKind kind, string? message = null)
    {
        if (condition)
            ThrowState(kind, message);
    }

    /// <summary>The <see cref="ArgumentException"/> for a failure.</summary>
    public static ArgumentException Argument(FailureException failure) => new KernelArgumentException(PublicMessage(failure), failure);

    /// <summary>The <see cref="ArgumentOutOfRangeException"/> for a failure.</summary>
    public static ArgumentOutOfRangeException Range(FailureException failure) => new KernelArgumentOutOfRangeException(PublicMessage(failure), failure);

    /// <summary>The <see cref="InvalidOperationException"/> for a failure.</summary>
    public static InvalidOperationException State(FailureException failure) => new(PublicMessage(failure), failure);

    /// <summary>The <see cref="NotSupportedException"/> for a failure.</summary>
    public static NotSupportedException NotSupported(FailureException failure) => new(PublicMessage(failure), failure);

    /// <summary>The engine's failure in an exception, itself or the one a refusal holds; null for any other exception.</summary>
    public static FailureException? Of(Exception exception) => exception switch
    {
        FailureException failure => failure,
        ArgumentException or InvalidOperationException or NotSupportedException => exception.InnerException as FailureException,
        _ => null,
    };

    /// <summary>Whether an exception is one of the engine's failures, or a refusal holding one.</summary>
    public static bool IsFailure(Exception exception) => Of(exception) is not null;

    /// <summary>The exception as the engine knows it: a refusal's failure, or the exception itself.</summary>
    public static Exception Unwrapped(Exception exception) => Of(exception) ?? exception;

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void ThrowArgument(FailureKind kind, string? message) => throw Argument(Failure(kind, message));

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void ThrowRange(FailureKind kind, string? message) => throw Range(Failure(kind, message));

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void ThrowState(FailureKind kind, string? message) => throw State(Failure(kind, message));

    private static FailureException Failure(FailureKind kind, string? message) => kind switch
    {
        FailureKind.ConstructionError => new ConstructionErrorException(message),
        FailureKind.DimensionError => new DimensionErrorException(message),
        FailureKind.DimensionMismatch => new DimensionMismatchException(message),
        FailureKind.DivideByZero => new ZeroDivisorException(message),
        FailureKind.DomainError => new DomainErrorException(message),
        FailureKind.NegativeValue => new NegativeValueException(message),
        FailureKind.NoSuchObject => new NoSuchObjectException(message),
        FailureKind.NullObject => new NullObjectException(message),
        FailureKind.NullValue => new NullValueException(message),
        FailureKind.NullVector => new NullVectorException(message),
        FailureKind.OutOfRange => new OutOfRangeException(message),
        FailureKind.ProgramError => new ProgramErrorException(message),
        FailureKind.RangeError => new RangeErrorException(message),
        FailureKind.TypeMismatch => new TypeMismatchException(message),
        _ => new FailureException(message),
    };

    // The failure's description, then what its message says in Kerf's terms, for callers who have never heard of OCCT
    private static string PublicMessage(FailureException failure)
    {
        string kind = FailureDiagnostics.Describe(failure);
        kind = char.ToUpperInvariant(kind[0]) + kind[1..];
        string own = FailureDiagnostics.PublicText(failure);
        return own.Length == 0 ? kind + "." : $"{kind}: {own}.";
    }
}
