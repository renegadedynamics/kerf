// Port of: src/FoundationClasses/TKMath/BVH/BVH_Object.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A non-generic base for <see cref="Object{T, TVec}"/> (just to have a named base class).</summary>
internal abstract class ObjectTransient : Transient
{
    protected bool _isDirty;                  //!< Marks internal object state as outdated
    protected Properties? _properties;    //!< Generic properties assigned to the object

    /// <summary>Creates a new abstract geometric object.</summary>
    protected ObjectTransient()
    {
        _isDirty = false;
    }

    /// <summary>The properties of the geometric object.</summary>
    public virtual Properties? Properties() => _properties;

    /// <summary>Sets the properties of the geometric object.</summary>
    public virtual void SetProperties(Properties? properties) => _properties = properties;

    /// <summary>True if the object state should be updated.</summary>
    public virtual bool IsDirty => _isDirty;

    /// <summary>Marks the object state as outdated (needs BVH rebuilding).</summary>
    public virtual void MarkDirty() => _isDirty = true;
}

/// <summary>An abstract geometric object bounded by a BVH box.</summary>
internal abstract class Object<T, TVec> : ObjectTransient
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>The AABB of the geometric object.</summary>
    public abstract Box<T, TVec> Box();
}
