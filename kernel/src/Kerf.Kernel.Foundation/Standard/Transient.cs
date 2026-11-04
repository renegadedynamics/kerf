// Port of: src/FoundationClasses/TKernel/Standard/Standard_Transient.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The base of OCCT's reference-counted classes, those held by <c>occ::handle&lt;T&gt;</c>. In .NET the garbage
/// collector replaces the reference count, so a handle is a plain (nullable) reference: <c>IsNull()</c> is
/// <c>is null</c> and <c>DownCast</c> is <c>as</c>.
/// </summary>
public abstract class Transient
{
    // Nothing of it is public (question 55): the public geometry only needs it as a base
    private protected Transient()
    {
    }

    /// <summary>The object's type, as OCCT's <c>DynamicType()</c>.</summary>
    internal Type DynamicType() => GetType();

    /// <summary>The name of the object's type, as OCCT's <c>DynamicType()->Name()</c>: its OCCT name (<see cref="OcctTypeNames"/>).</summary>
    internal string DynamicTypeName() => OcctTypeNames.Of(GetType());

    /// <summary>True if this object is of the type or derives from it, as OCCT's <c>IsKind</c>.</summary>
    internal bool IsKind(Type type) => type.IsInstanceOfType(this);

    internal bool IsKind<T>() => this is T;

    /// <summary>True if this object is exactly of the type, as OCCT's <c>IsInstance</c>.</summary>
    internal bool IsInstance(Type type) => GetType() == type;

    internal Transient This() => this;

    // Freezing (OCCT.NET, not OCCT): objects that callers or topology share must not change underneath them.
    // Ported mutators of geometry and topology classes start with CheckMutable(); the port freezes an object
    // when it is attached to topology or otherwise shared. Freezing is one-way. See docs/translation-rules.md.

    private volatile bool _isFrozen;

    /// <summary>True once the object can no longer change.</summary>
    internal bool IsFrozen => _isFrozen;

    /// <summary>Makes the object unchangeable from now on. Idempotent and thread-safe.</summary>
    internal void Freeze() => _isFrozen = true;

    /// <summary>Throws <see cref="ImmutableObjectException"/> if the object is frozen: the first line of every mutator.</summary>
    internal void CheckMutable()
    {
        if (_isFrozen)
        {
            throw new ImmutableObjectException($"{OcctTypeNames.Of(GetType())} is frozen: it is shared, so it cannot change. Change a copy.");
        }
    }
}
