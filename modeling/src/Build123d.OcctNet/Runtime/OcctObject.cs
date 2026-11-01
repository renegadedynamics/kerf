namespace Build123d.Occt;

/// <summary>
/// Base of every wrapper. Under the OCCT.NET backend a wrapper holds a managed OCCT.NET object: a reference to a class,
/// or a struct of its own (gp_Pnt, TopoDS_Shape…) as the native binding holds a heap copy. Disposing drops it.
/// </summary>
public abstract class OcctObject : IDisposable
{
    private object? _raw;
    private volatile bool _disposed;

    private protected OcctObject() { }

    private protected OcctObject(object raw, bool _) => _raw = raw;

    private protected OcctObject(Unported _) { }

    /// <summary>The OCCT.NET object of a class-backed wrapper.</summary>
    internal object RawImpl => _raw ?? throw new ObjectDisposedException(GetType().Name);

    public bool IsDisposed => _disposed;

    /// <summary>OCCT.NET's memory is managed, so the GC already counts it: nothing to add.</summary>
    internal void AddMemoryPressure(long bytes) { }

    internal long MemoryPressure => 0;

    public void Dispose()
    {
        _disposed = true;
        _raw = null;
        GC.SuppressFinalize(this);
    }
}

/// <summary>Base of wrappers around OCCT's reference-counted (Standard_Transient) classes.</summary>
public abstract class OcctTransient : OcctObject
{
    private protected OcctTransient() { }

    private protected OcctTransient(object raw, bool _) : base(raw, _) { }

    private protected OcctTransient(Unported u) : base(u) { }
}

/// <summary>What OCCT.NET doesn't have, or doesn't make public: these throw <see cref="NotSupportedException"/>.</summary>
internal sealed class Unported
{
    private Unported() { }

    /// <summary>A class not yet ported to OCCT.NET, such as phase 7's fillets and offsets.</summary>
    internal static NotSupportedException Class(string name) => new($"not yet ported to OCCT.NET: {name}");

    /// <summary>A member whose OCCT.NET counterpart isn't public, or differs.</summary>
    internal static NotSupportedException Member(string member) => new($"not available through OCCT.NET's public API: {member}");

    /// <summary>Throws, for a constructor that can't be forwarded.</summary>
    internal static Unported Throw(Exception e) => throw e;
}
