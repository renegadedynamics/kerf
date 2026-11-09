// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf;
using Xunit;

/// <summary>
/// Asserts the kernel's refusal of an argument: an exception that is .NET's <typeparamref name="T"/> to a caller (the
/// kernel's own type derived from exactly it) and the kernel's, as the modelling layer tells it
/// (<see cref="Diagnostic.IsKernelArgumentError"/>, decision 25801).
/// </summary>
internal static class KernelAssert
{
    /// <summary>The kernel's refusal, as .NET's <typeparamref name="T"/>, that <paramref name="call"/> throws.</summary>
    public static T Throws<T>(Action call) where T : ArgumentException => Refusal<T>(Record.Exception(call));

    /// <inheritdoc cref="Throws{T}(Action)"/>
    public static T Throws<T>(Func<object?> call) where T : ArgumentException => Refusal<T>(Record.Exception(call));

    private static T Refusal<T>(Exception? thrown) where T : ArgumentException
    {
        Assert.NotNull(thrown);
        Assert.True(Diagnostic.IsKernelArgumentError(thrown), $"{thrown.GetType()} isn't the kernel's refusal: {thrown.Message}");
        Assert.Equal(typeof(T), thrown.GetType().BaseType);
        return (T)thrown;
    }
}
