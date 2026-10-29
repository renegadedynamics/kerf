namespace Build123d.Occt;

/// <summary>
/// Base of every wrapper around a native OCCT object. The wrapper owns its native object (a heap copy,
/// or one reference on a transient) and frees it when disposed or finalised.
/// </summary>
public abstract class OcctObject : IDisposable
{
    private IntPtr _ptr;
    // Native memory the GC has been told this object keeps alive, given back when it's released
    private long _pressure;

    private protected OcctObject(IntPtr ptr)
    {
        if (ptr == IntPtr.Zero)
            throw new ArgumentNullException(nameof(ptr), "native object is null");
        _ptr = ptr;
    }

    /// <summary>The native pointer, typed as this wrapper's exact class.</summary>
    private protected IntPtr Ptr => _ptr != IntPtr.Zero ? _ptr : throw new ObjectDisposedException(GetType().Name);

    public bool IsDisposed => _ptr == IntPtr.Zero;

    /// <summary>
    /// Tells the GC this object keeps <paramref name="bytes"/> of native memory alive, until it's released, so the
    /// GC collects before native memory balloons. Once per object: later calls are ignored.
    /// </summary>
    internal void AddMemoryPressure(long bytes)
    {
        if (bytes > 0 && Interlocked.CompareExchange(ref _pressure, bytes, 0) == 0)
            GC.AddMemoryPressure(bytes);
    }

    /// <summary>The native memory this object has told the GC about.</summary>
    internal long MemoryPressure => Interlocked.Read(ref _pressure);

    private void Release()
    {
        var p = Interlocked.Exchange(ref _ptr, IntPtr.Zero);
        if (p != IntPtr.Zero)
            ReleaseNative(p);
        var bytes = Interlocked.Exchange(ref _pressure, 0);
        if (bytes > 0)
            GC.RemoveMemoryPressure(bytes);
    }

    /// <summary>Frees the native object. Overridden by every class that can be deleted.</summary>
    protected virtual void ReleaseNative(IntPtr ptr) { }

    public void Dispose()
    {
        Release();
        GC.SuppressFinalize(this);
    }

    ~OcctObject() => Release();
}
