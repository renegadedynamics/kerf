// Port of: src/FoundationClasses/TKMath/BVH/BVH_Types.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// OCCT's <c>BVH</c> namespace: tools shared by the BVH classes.
/// </summary>
/// <remarks>
/// OCCT templates BVH over the element type and the dimension (<c>BVH_Box&lt;T, N&gt;</c>); the port is generic over
/// the element type <c>T</c> and the vector type <c>TVec</c> (<see cref="NCollection_Vec2{T}"/>,
/// <see cref="NCollection_Vec3{T}"/> or <see cref="NCollection_Vec4{T}"/>), which fixes the dimension. So OCCT's
/// <c>BVH_Box&lt;double, 3&gt;</c> is <c>BVH_Box&lt;double, NCollection_Vec3&lt;double&gt;&gt;</c>. OCCT's typedefs
/// (<c>BVH_Vec3d</c>, <c>BVH_Array4i</c>, ...) are written out. <c>BVH::VectorType</c>, <c>BVH::ArrayType</c>
/// and <c>BVH::Array</c> (thin wrappers over the array type) are not needed: ported code calls
/// <see cref="NCollection_LinearVector{T}"/> directly. The one-dimensional case (<c>N = 1</c>), which OCCT
/// declares but never instantiates, is not ported.
/// </remarks>
public static partial class BvhTools
{
    // ToBndBox (the N = 1 overload is not ported)

    public static Box ToBndBox<T>(in NCollection_Vec2<T> min, in NCollection_Vec2<T> max)
        where T : unmanaged, INumber<T>
        => new(new Point3D(double.CreateTruncating(min.X()), double.CreateTruncating(min.Y()), 0.0),
               new Point3D(double.CreateTruncating(max.X()), double.CreateTruncating(max.Y()), 0.0));

    public static Box ToBndBox<T>(in NCollection_Vec3<T> min, in NCollection_Vec3<T> max)
        where T : unmanaged, INumber<T>
        => new(new Point3D(double.CreateTruncating(min.X()), double.CreateTruncating(min.Y()), double.CreateTruncating(min.Z())),
               new Point3D(double.CreateTruncating(max.X()), double.CreateTruncating(max.Y()), double.CreateTruncating(max.Z())));

    public static Box ToBndBox<T>(in NCollection_Vec4<T> min, in NCollection_Vec4<T> max)
        where T : unmanaged, INumber<T>
        => new(new Point3D(double.CreateTruncating(min.X()), double.CreateTruncating(min.Y()), double.CreateTruncating(min.Z())),
               new Point3D(double.CreateTruncating(max.X()), double.CreateTruncating(max.Y()), double.CreateTruncating(max.Z())));

    /// <summary>
    /// The component of the given axis (<c>BVH::VecComp&lt;T, N&gt;::Get</c>). As in OCCT, an axis beyond the last
    /// gives the last component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T VecComp<T, TVec>(in TVec vec, int axis)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        int length = TVec.Length();
        if (length == 2)
        {
            return axis == 0 ? vec[0] : vec[1];
        }

        if (length == 3)
        {
            return axis == 0 ? vec[0] : (axis == 1 ? vec[1] : vec[2]);
        }

        return axis == 0 ? vec[0]
                            : (axis == 1 ? vec[1] : (axis == 2 ? vec[2] : vec[3]));
    }

    /// <summary>The floor of a value as an integer, computed as OCCT does (truncate, then correct).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int IntFloor<T>(T value) where T : unmanaged, INumber<T>
    {
        int res = int.CreateTruncating(value);

        return res - (T.CreateTruncating(res) > value ? 1 : 0);
    }

    /// <summary><c>std::numeric_limits&lt;T&gt;::epsilon()</c>: the machine epsilon of float or double.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Epsilon<T>() where T : unmanaged, INumber<T>
        => typeof(T) == typeof(float) ? T.CreateTruncating(1.1920928955078125E-07)
           : typeof(T) == typeof(double) ? T.CreateTruncating(2.220446049250313E-16)
           : T.Zero;

    /// <summary><c>std::min</c>: the first argument unless the second is smaller.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Min<T>(T value1, T value2) where T : INumber<T> => value2 < value1 ? value2 : value1;

    /// <summary><c>std::max</c>: the first argument unless it is smaller than the second.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Max<T>(T value1, T value2) where T : INumber<T> => value1 < value2 ? value2 : value1;
}
