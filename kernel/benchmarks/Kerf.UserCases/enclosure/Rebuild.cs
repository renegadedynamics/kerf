// Not part of the enclosure model: Kerf's addition, so a benchmark can build the model more than once in a process.
//
// The model keeps each part in a static System.Lazy<T>, so a process builds it once. This type has the same name in
// the model's own namespace, which C# looks in before the namespaces the files import, so the model's sources compile
// unchanged against it, and Rebuild.Forget makes the next read of every part build it again.

namespace Renegade.BladeBumper;

/// <summary>What the enclosure model's <c>Lazy&lt;T&gt;</c> fields are here: built once, until <see cref="Rebuild.Forget"/>.</summary>
internal sealed class Lazy<T>
{
    private readonly Func<T> _factory;
    private readonly object _gate = new();
    private T? _value;
    private bool _isBuilt;

    public Lazy(Func<T> factory)
    {
        _factory = factory;
        Rebuild.Register(Forget);
    }

    /// <summary>The value, built on first read since the last <see cref="Rebuild.Forget"/>.</summary>
    public T Value
    {
        get
        {
            lock (_gate)
            {
                if (!_isBuilt)
                {
                    _value = _factory();
                    _isBuilt = true;
                }
                return _value!;
            }
        }
    }

    private void Forget()
    {
        lock (_gate)
        {
            _value = default;
            _isBuilt = false;
        }
    }
}

/// <summary>Forgets every part the enclosure model has built, so the next read builds it again.</summary>
public static class Rebuild
{
    private static readonly List<Action> Forgets = [];

    internal static void Register(Action forget)
    {
        lock (Forgets)
        {
            Forgets.Add(forget);
        }
    }

    /// <summary>Forgets every built part. The model's static fields are all created on its classes' first use.</summary>
    public static void Forget()
    {
        Action[] all;
        lock (Forgets)
        {
            all = [.. Forgets];
        }
        foreach (var forget in all)
        {
            forget();
        }
    }
}
