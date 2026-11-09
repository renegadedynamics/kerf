// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Foundation;

/// <summary>
/// An argument the kernel refused: the mark its three argument exceptions carry, so a caller in another assembly (the
/// modelling layer) can tell the kernel's refusal of a value from its own misuse by the exception's type, wherever the
/// JIT compiled the check (decision 25801).
/// </summary>
/// <remarks>
/// Every argument exception the kernel's sources make is one of these (KERF0004 reports .NET's own): each is .NET's type
/// of that name underneath, with the same message, parameter name and value, so a caller catching
/// <see cref="ArgumentException"/>, <see cref="ArgumentOutOfRangeException"/> or <see cref="ArgumentNullException"/>
/// catches it as before. An argument exception .NET's own code throws from inside the kernel (a list indexed out of
/// range) isn't the kernel's refusal but a fault, and doesn't carry it.
/// </remarks>
internal interface IKernelArgumentError
{
}

/// <summary>The kernel's <see cref="ArgumentException"/> (<see cref="IKernelArgumentError"/>).</summary>
internal sealed class KernelArgumentException : ArgumentException, IKernelArgumentError
{
    // .NET's message, when it was made by .NET's own check (ThrowIfNullOrEmpty)
    private readonly string? _message;

    public KernelArgumentException()
    {
    }

    public KernelArgumentException(string? message) : base(message)
    {
    }

    public KernelArgumentException(string? message, Exception? innerException) : base(message, innerException)
    {
    }

    public KernelArgumentException(string? message, string? paramName) : base(message, paramName)
    {
    }

    public KernelArgumentException(string? message, string? paramName, Exception? innerException) : base(message, paramName, innerException)
    {
    }

    private KernelArgumentException(ArgumentException copied) : base(null, copied.ParamName, copied.InnerException) => _message = copied.Message;

    /// <inheritdoc/>
    public override string Message => _message ?? base.Message;

    /// <summary>As <see cref="ArgumentException.ThrowIfNullOrEmpty"/>, throwing the kernel's exceptions.</summary>
    public static new void ThrowIfNullOrEmpty([NotNull] string? argument, [CallerArgumentExpression(nameof(argument))] string? paramName = null)
    {
        if (string.IsNullOrEmpty(argument))
            NullOrEmpty(argument, paramName);
    }

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void NullOrEmpty(string? argument, string? paramName)
    {
        try
        {
            ArgumentException.ThrowIfNullOrEmpty(argument, paramName);
        }
        catch (ArgumentNullException e)
        {
            throw KernelArgumentNullException.Copy(e);
        }
        catch (ArgumentException e)
        {
            throw new KernelArgumentException(e);
        }
        throw new UnreachableException();
    }
}

/// <summary>The kernel's <see cref="ArgumentOutOfRangeException"/> (<see cref="IKernelArgumentError"/>).</summary>
internal sealed class KernelArgumentOutOfRangeException : ArgumentOutOfRangeException, IKernelArgumentError
{
    private readonly string? _message;

    public KernelArgumentOutOfRangeException()
    {
    }

    public KernelArgumentOutOfRangeException(string? paramName) : base(paramName)
    {
    }

    public KernelArgumentOutOfRangeException(string? paramName, string? message) : base(paramName, message)
    {
    }

    public KernelArgumentOutOfRangeException(string? message, Exception? innerException) : base(message, innerException)
    {
    }

    public KernelArgumentOutOfRangeException(string? paramName, object? actualValue, string? message) : base(paramName, actualValue, message)
    {
    }

    private KernelArgumentOutOfRangeException(ArgumentOutOfRangeException copied) : base(copied.ParamName, copied.ActualValue, null) => _message = copied.Message;

    /// <inheritdoc/>
    public override string Message => _message ?? base.Message;

    /// <summary>As <see cref="ArgumentOutOfRangeException.ThrowIfNegative{T}"/>, throwing the kernel's exception.</summary>
    public static new void ThrowIfNegative<T>(T value, [CallerArgumentExpression(nameof(value))] string? paramName = null)
        where T : INumberBase<T>
    {
        if (T.IsNegative(value))
            Negative(value, paramName);
    }

    /// <summary>As <see cref="ArgumentOutOfRangeException.ThrowIfNegativeOrZero{T}"/>, throwing the kernel's exception.</summary>
    public static new void ThrowIfNegativeOrZero<T>(T value, [CallerArgumentExpression(nameof(value))] string? paramName = null)
        where T : INumberBase<T>
    {
        if (T.IsNegative(value) || T.IsZero(value))
            NegativeOrZero(value, paramName);
    }

    /// <summary>As <see cref="ArgumentOutOfRangeException.ThrowIfGreaterThan{T}"/>, throwing the kernel's exception.</summary>
    public static new void ThrowIfGreaterThan<T>(T value, T other, [CallerArgumentExpression(nameof(value))] string? paramName = null)
        where T : IComparable<T>
    {
        if (value.CompareTo(other) > 0)
            GreaterThan(value, other, paramName);
    }

    /// <summary>As <see cref="ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual{T}"/>, throwing the kernel's exception.</summary>
    public static new void ThrowIfGreaterThanOrEqual<T>(T value, T other, [CallerArgumentExpression(nameof(value))] string? paramName = null)
        where T : IComparable<T>
    {
        if (value.CompareTo(other) >= 0)
            GreaterThanOrEqual(value, other, paramName);
    }

    /// <summary>As <see cref="ArgumentOutOfRangeException.ThrowIfLessThan{T}"/>, throwing the kernel's exception.</summary>
    public static new void ThrowIfLessThan<T>(T value, T other, [CallerArgumentExpression(nameof(value))] string? paramName = null)
        where T : IComparable<T>
    {
        if (value.CompareTo(other) < 0)
            LessThan(value, other, paramName);
    }

    // Each throw path runs .NET's own check, which fails, for its message, and throws the kernel's copy of what it threw
    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void Negative<T>(T value, string? paramName) where T : INumberBase<T>
    {
        try
        {
            ArgumentOutOfRangeException.ThrowIfNegative(value, paramName);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new KernelArgumentOutOfRangeException(e);
        }
        throw new UnreachableException();
    }

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void NegativeOrZero<T>(T value, string? paramName) where T : INumberBase<T>
    {
        try
        {
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, paramName);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new KernelArgumentOutOfRangeException(e);
        }
        throw new UnreachableException();
    }

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void GreaterThan<T>(T value, T other, string? paramName) where T : IComparable<T>
    {
        try
        {
            ArgumentOutOfRangeException.ThrowIfGreaterThan(value, other, paramName);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new KernelArgumentOutOfRangeException(e);
        }
        throw new UnreachableException();
    }

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void GreaterThanOrEqual<T>(T value, T other, string? paramName) where T : IComparable<T>
    {
        try
        {
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(value, other, paramName);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new KernelArgumentOutOfRangeException(e);
        }
        throw new UnreachableException();
    }

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void LessThan<T>(T value, T other, string? paramName) where T : IComparable<T>
    {
        try
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, other, paramName);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new KernelArgumentOutOfRangeException(e);
        }
        throw new UnreachableException();
    }
}

/// <summary>The kernel's <see cref="ArgumentNullException"/> (<see cref="IKernelArgumentError"/>).</summary>
internal sealed class KernelArgumentNullException : ArgumentNullException, IKernelArgumentError
{
    private readonly string? _message;

    public KernelArgumentNullException()
    {
    }

    public KernelArgumentNullException(string? paramName) : base(paramName)
    {
    }

    public KernelArgumentNullException(string? paramName, string? message) : base(paramName, message)
    {
    }

    public KernelArgumentNullException(string? message, Exception? innerException) : base(message, innerException)
    {
    }

    private KernelArgumentNullException(ArgumentNullException copied) : base(copied.ParamName, (string?)null) => _message = copied.Message;

    /// <inheritdoc/>
    public override string Message => _message ?? base.Message;

    /// <summary>As <see cref="ArgumentNullException.ThrowIfNull(object?, string?)"/>, throwing the kernel's exception.</summary>
    public static new void ThrowIfNull([NotNull] object? argument, [CallerArgumentExpression(nameof(argument))] string? paramName = null)
    {
        if (argument is null)
            Null(paramName);
    }

    internal static KernelArgumentNullException Copy(ArgumentNullException e) => new(e);

    [DoesNotReturn, MethodImpl(MethodImplOptions.NoInlining)]
    private static void Null(string? paramName) => throw new KernelArgumentNullException(paramName);
}
