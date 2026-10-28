namespace Build123d.Occt;

/// <summary>
/// Base of every wrapper around a native OCCT object. The wrapper owns its native object (a heap copy,
/// or one reference on a transient) and frees it when disposed or finalised.
/// </summary>
public abstract class OcctObject : IDisposable
{
    private IntPtr _ptr;

    private protected OcctObject(IntPtr ptr)
    {
        if (ptr == IntPtr.Zero)
            throw new ArgumentNullException(nameof(ptr), "native object is null");
        _ptr = ptr;
    }

    /// <summary>The native pointer, typed as this wrapper's exact class.</summary>
    private protected IntPtr Ptr => _ptr != IntPtr.Zero ? _ptr : throw new ObjectDisposedException(GetType().Name);

    public bool IsDisposed => _ptr == IntPtr.Zero;

    /// <summary>Frees the native object. Overridden by every class that can be deleted.</summary>
    protected virtual void ReleaseNative(IntPtr ptr) { }

    public void Dispose()
    {
        var p = Interlocked.Exchange(ref _ptr, IntPtr.Zero);
        if (p != IntPtr.Zero)
            ReleaseNative(p);
        GC.SuppressFinalize(this);
    }

    ~OcctObject()
    {
        var p = Interlocked.Exchange(ref _ptr, IntPtr.Zero);
        if (p != IntPtr.Zero)
            ReleaseNative(p);
    }
}
