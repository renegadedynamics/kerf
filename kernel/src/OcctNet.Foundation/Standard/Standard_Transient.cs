// Port of: src/FoundationClasses/TKernel/Standard/Standard_Transient.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The base of OCCT's reference-counted classes, those held by <c>occ::handle&lt;T&gt;</c>. In .NET the garbage
/// collector replaces the reference count, so a handle is a plain (nullable) reference: <c>IsNull()</c> is
/// <c>is null</c> and <c>DownCast</c> is <c>as</c>.
/// </summary>
public abstract class Standard_Transient
{
    /// <summary>The object's type, as OCCT's <c>DynamicType()</c>.</summary>
    public Type DynamicType() => GetType();

    /// <summary>True if this object is of the type or derives from it, as OCCT's <c>IsKind</c>.</summary>
    public bool IsKind(Type theType) => theType.IsInstanceOfType(this);

    public bool IsKind<T>() => this is T;

    /// <summary>True if this object is exactly of the type, as OCCT's <c>IsInstance</c>.</summary>
    public bool IsInstance(Type theType) => GetType() == theType;

    public Standard_Transient This() => this;

    // Freezing (OCCT.NET, not OCCT): objects that callers or topology share must not change underneath them.
    // Ported mutators of geometry and topology classes start with CheckMutable(); the port freezes an object
    // when it is attached to topology or otherwise shared. Freezing is one-way. See docs/translation-rules.md.

    private volatile bool myIsFrozen;

    /// <summary>True once the object can no longer change.</summary>
    public bool IsFrozen() => myIsFrozen;

    /// <summary>Makes the object unchangeable from now on. Idempotent and thread-safe.</summary>
    internal void Freeze() => myIsFrozen = true;

    /// <summary>Throws <see cref="Standard_ImmutableObject"/> if the object is frozen: the first line of every mutator.</summary>
    protected internal void CheckMutable()
    {
        if (myIsFrozen)
        {
            throw new Standard_ImmutableObject($"{GetType().Name} is frozen: it is shared, so it cannot change. Change a copy.");
        }
    }
}
