namespace Build123d.Occt;

/// <summary>
/// Base of wrappers around reference-counted OCCT objects (those deriving from Standard_Transient,
/// held in C++ by occ::handle). Each wrapper holds one reference.
/// </summary>
public abstract class OcctTransient : OcctObject
{
    private protected OcctTransient(IntPtr ptr) : base(ptr) { }

    /// <summary>The native pointer as a Standard_Transient*, for down-casting.</summary>
    internal abstract IntPtr PtrAs_Standard_Transient();
}
