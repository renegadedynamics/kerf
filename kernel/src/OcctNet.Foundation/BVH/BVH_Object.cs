// Port of: src/FoundationClasses/TKMath/BVH/BVH_Object.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A non-generic base for <see cref="BVH_Object{T, TVec}"/> (just to have a named base class).</summary>
public abstract class BVH_ObjectTransient : Standard_Transient
{
    protected bool myIsDirty;                  //!< Marks internal object state as outdated
    protected BVH_Properties? myProperties;    //!< Generic properties assigned to the object

    /// <summary>Creates a new abstract geometric object.</summary>
    protected BVH_ObjectTransient()
    {
        myIsDirty = false;
    }

    /// <summary>The properties of the geometric object.</summary>
    public virtual BVH_Properties? Properties() => myProperties;

    /// <summary>Sets the properties of the geometric object.</summary>
    public virtual void SetProperties(BVH_Properties? theProperties) => myProperties = theProperties;

    /// <summary>True if the object state should be updated.</summary>
    public virtual bool IsDirty() => myIsDirty;

    /// <summary>Marks the object state as outdated (needs BVH rebuilding).</summary>
    public virtual void MarkDirty() => myIsDirty = true;
}

/// <summary>An abstract geometric object bounded by a BVH box.</summary>
public abstract class BVH_Object<T, TVec> : BVH_ObjectTransient
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>The AABB of the geometric object.</summary>
    public abstract BVH_Box<T, TVec> Box();
}
