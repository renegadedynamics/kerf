// Kerf: the per-operation context (phase10-plan.md, section 2.7; unit 10I): the settings OCCT kept in process-wide
// statics, as immutable defaults with overrides that flow with an operation. Not a port of an OCCT file.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Foundation;

/// <summary>
/// A setting of the <see cref="OperationContext"/>: its name and its default value, which holds wherever no context
/// overrides it.
/// </summary>
/// <remarks>
/// Each setting is declared once, as a static readonly field of the class it configures (so a key is a constant of
/// its owner, and the assemblies above Foundation declare their own). Keys compare by identity.
/// </remarks>
/// <typeparam name="T">The type of the setting's value: an immutable value, or an object no one changes once set.</typeparam>
internal sealed class ContextKey<T>
{
    /// <summary>A setting named <paramref name="name"/> (for messages and debugging), with its default value.</summary>
    public ContextKey(string name, T defaultValue)
    {
        KernelArgumentException.ThrowIfNullOrEmpty(name);
        Name = name;
        Default = defaultValue;
    }

    /// <summary>The setting's name, such as <c>BOPAlgo.RunParallel</c>.</summary>
    public string Name { get; }

    /// <summary>The value where no context overrides it.</summary>
    public T Default { get; }

    /// <summary>The value in the current flow of execution (<see cref="OperationContext.Current"/>).</summary>
    public T Value => OperationContext.ValueOf(this);

    /// <inheritdoc/>
    public override string ToString() => Name;
}

/// <summary>
/// The context of the operations running in a flow of execution: the settings OCCT kept in process-wide statics
/// (debug flags, parallel defaults, the meshing algorithm's name, the message table's additions, ShapeProcess's
/// operators and resource overrides…), each a <see cref="ContextKey{T}"/> with an immutable default.
/// </summary>
/// <remarks>
/// <para>
/// A context is immutable: <see cref="With{T}"/> gives a new one. The current context is held in an
/// <see cref="AsyncLocal{T}"/>, so it flows into the tasks and <c>Parallel.For</c> bodies an operation starts and into
/// every operation nested in it, and independent operations on other threads (or other tests, or other Bento cells)
/// each see their own. Where nothing was set, every setting has its default, and reading one costs an
/// <see cref="AsyncLocal{T}"/> read.
/// </para>
/// <para>
/// Two ways to set: <see cref="Begin(OperationContext)"/> (or <see cref="Begin{T}(ContextKey{T}, T)"/>) for a scope,
/// restored when disposed; and <see cref="Set{T}"/>, which changes the setting for the rest of the current flow, as
/// OCCT's process-wide setters (<c>BOPAlgo_Options::SetParallelMode</c>, <c>Quantity_Color::SetEpsilon</c>) did for
/// the process. A flow's setting ends with the flow: an <c>async</c> method's caller, a finished task, or another
/// test doesn't see it.
/// </para>
/// <para>
/// The working overlay of 10H1 (<c>WorkingOverlay</c>, ModelingData) is the topology part of an operation's state and
/// flows the same way, through an <see cref="AsyncLocal{T}"/> of its own, which its accessors mirror in a thread
/// static for speed (decision 1359).
/// </para>
/// </remarks>
internal sealed class OperationContext
{
    private static readonly AsyncLocal<OperationContext?> CurrentContext = new();

    // The settings that differ from their keys' defaults, as (key, value) pairs: a context overrides few, so a scan is
    // faster than a dictionary
    private readonly KeyValuePair<object, object?>[] _values;

    private OperationContext(KeyValuePair<object, object?>[] values) => _values = values;

    /// <summary>The context that overrides nothing: every setting has its default.</summary>
    public static OperationContext Default { get; } = new([]);

    /// <summary>The context of the current flow of execution (<see cref="Default"/> unless one was set).</summary>
    public static OperationContext Current => CurrentContext.Value ?? Default;

    /// <summary>The number of settings this context overrides.</summary>
    public int Count => _values.Length;

    /// <summary>The value of the setting in this context.</summary>
    public T Get<T>(ContextKey<T> key)
    {
        foreach (KeyValuePair<object, object?> pair in _values)
        {
            if (ReferenceEquals(pair.Key, key))
            {
                return (T)pair.Value!;
            }
        }
        return key.Default;
    }

    /// <summary>True, with the value, if this context overrides the setting.</summary>
    public bool TryGet<T>(ContextKey<T> key, out T value)
    {
        foreach (KeyValuePair<object, object?> pair in _values)
        {
            if (ReferenceEquals(pair.Key, key))
            {
                value = (T)pair.Value!;
                return true;
            }
        }
        value = key.Default;
        return false;
    }

    /// <summary>This context with the setting <paramref name="key"/> set to <paramref name="value"/>.</summary>
    public OperationContext With<T>(ContextKey<T> key, T value)
    {
        KernelArgumentNullException.ThrowIfNull(key);
        for (int i = 0; i < _values.Length; i++)
        {
            if (ReferenceEquals(_values[i].Key, key))
            {
                var replaced = (KeyValuePair<object, object?>[])_values.Clone();
                replaced[i] = new(key, value);
                return new OperationContext(replaced);
            }
        }
        var added = new KeyValuePair<object, object?>[_values.Length + 1];
        _values.CopyTo(added, 0);
        added[^1] = new(key, value);
        return new OperationContext(added);
    }

    /// <summary>This context with the setting <paramref name="key"/> back to its default.</summary>
    public OperationContext Without<T>(ContextKey<T> key)
    {
        int index = Array.FindIndex(_values, pair => ReferenceEquals(pair.Key, key));
        if (index < 0)
        {
            return this;
        }
        if (_values.Length == 1)
        {
            return Default;
        }
        var removed = new KeyValuePair<object, object?>[_values.Length - 1];
        Array.Copy(_values, 0, removed, 0, index);
        Array.Copy(_values, index + 1, removed, index, _values.Length - index - 1);
        return new OperationContext(removed);
    }

    /// <summary>The value of the setting in the current flow of execution.</summary>
    public static T ValueOf<T>(ContextKey<T> key) => CurrentContext.Value is { } context ? context.Get(key) : key.Default;

    /// <summary>
    /// Makes <paramref name="context"/> current in this flow of execution (and the tasks it starts) until the scope
    /// is disposed, which restores the previous one.
    /// </summary>
    public static Scope Begin(OperationContext context)
    {
        KernelArgumentNullException.ThrowIfNull(context);
        OperationContext? previous = CurrentContext.Value;
        CurrentContext.Value = ReferenceEquals(context, Default) ? null : context;
        return new Scope(previous);
    }

    /// <summary>
    /// Sets one setting for a scope: the current context with <paramref name="key"/> set to <paramref name="value"/>,
    /// until the scope is disposed.
    /// </summary>
    public static Scope Begin<T>(ContextKey<T> key, T value) => Begin(Current.With(key, value));

    /// <summary>
    /// Sets the setting for the rest of the current flow of execution and returns its previous value: the form of
    /// OCCT's process-wide setters. Prefer <see cref="Begin{T}(ContextKey{T}, T)"/>, which restores it.
    /// </summary>
    public static T Set<T>(ContextKey<T> key, T value)
    {
        OperationContext current = Current;
        T previous = current.Get(key);
        OperationContext next = current.With(key, value);
        CurrentContext.Value = ReferenceEquals(next, Default) ? null : next;
        return previous;
    }

    /// <summary>The scope of a context made current by <see cref="Begin(OperationContext)"/>.</summary>
    public readonly struct Scope : IDisposable
    {
        private readonly OperationContext? _previous;

        internal Scope(OperationContext? previous) => _previous = previous;

        /// <summary>Restores the context current before the scope began.</summary>
        public void Dispose() => CurrentContext.Value = _previous;
    }
}
