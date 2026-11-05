// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Xunit;

/// <summary>
/// Asserts the engine's failure a call ends in, as the engine reads it: thrown itself, or held by the .NET exception a
/// public member refuses with (<see cref="Refusal"/>, unit 10P2). Kerf.Kernel.PublicApi.Tests pins the .NET exceptions.
/// </summary>
internal static class EngineAssert
{
    /// <summary>The failure of exactly the type <typeparamref name="T"/> that <paramref name="call"/> ends in.</summary>
    public static T Throws<T>(Action call) where T : Exception
    {
        Exception? thrown = Record.Exception(call);
        return Assert.IsType<T>(thrown is null ? null : Refusal.Unwrapped(thrown));
    }

    /// <inheritdoc cref="Throws{T}(Action)"/>
    public static T Throws<T>(Func<object?> call) where T : Exception
    {
        Exception? thrown = Record.Exception(call);
        return Assert.IsType<T>(thrown is null ? null : Refusal.Unwrapped(thrown));
    }

    /// <summary>The failure of type <typeparamref name="T"/>, or one derived from it, that <paramref name="call"/> ends in.</summary>
    public static T ThrowsAny<T>(Action call) where T : Exception
    {
        Exception? thrown = Record.Exception(call);
        return Assert.IsAssignableFrom<T>(thrown is null ? null : Refusal.Unwrapped(thrown));
    }

    /// <inheritdoc cref="ThrowsAny{T}(Action)"/>
    public static T ThrowsAny<T>(Func<object?> call) where T : Exception
    {
        Exception? thrown = Record.Exception(call);
        return Assert.IsAssignableFrom<T>(thrown is null ? null : Refusal.Unwrapped(thrown));
    }
}
