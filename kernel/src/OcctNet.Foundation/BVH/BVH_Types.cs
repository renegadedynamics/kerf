// Port of: src/FoundationClasses/TKMath/BVH/BVH_Types.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using System.Runtime.CompilerServices;

namespace Occt;

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
public static partial class BVH
{
    // ToBndBox (the N = 1 overload is not ported)

    public static Bnd_Box ToBndBox<T>(in NCollection_Vec2<T> theMin, in NCollection_Vec2<T> theMax)
        where T : unmanaged, INumber<T>
        => new(new gp_Pnt(double.CreateTruncating(theMin.x()), double.CreateTruncating(theMin.y()), 0.0),
               new gp_Pnt(double.CreateTruncating(theMax.x()), double.CreateTruncating(theMax.y()), 0.0));

    public static Bnd_Box ToBndBox<T>(in NCollection_Vec3<T> theMin, in NCollection_Vec3<T> theMax)
        where T : unmanaged, INumber<T>
        => new(new gp_Pnt(double.CreateTruncating(theMin.x()), double.CreateTruncating(theMin.y()), double.CreateTruncating(theMin.z())),
               new gp_Pnt(double.CreateTruncating(theMax.x()), double.CreateTruncating(theMax.y()), double.CreateTruncating(theMax.z())));

    public static Bnd_Box ToBndBox<T>(in NCollection_Vec4<T> theMin, in NCollection_Vec4<T> theMax)
        where T : unmanaged, INumber<T>
        => new(new gp_Pnt(double.CreateTruncating(theMin.x()), double.CreateTruncating(theMin.y()), double.CreateTruncating(theMin.z())),
               new gp_Pnt(double.CreateTruncating(theMax.x()), double.CreateTruncating(theMax.y()), double.CreateTruncating(theMax.z())));

    /// <summary>
    /// The component of the given axis (<c>BVH::VecComp&lt;T, N&gt;::Get</c>). As in OCCT, an axis beyond the last
    /// gives the last component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T VecComp<T, TVec>(in TVec theVec, int theAxis)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        int aLength = TVec.Length();
        if (aLength == 2)
        {
            return theAxis == 0 ? theVec[0] : theVec[1];
        }

        if (aLength == 3)
        {
            return theAxis == 0 ? theVec[0] : (theAxis == 1 ? theVec[1] : theVec[2]);
        }

        return theAxis == 0 ? theVec[0]
                            : (theAxis == 1 ? theVec[1] : (theAxis == 2 ? theVec[2] : theVec[3]));
    }

    /// <summary>The floor of a value as an integer, computed as OCCT does (truncate, then correct).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int IntFloor<T>(T theValue) where T : unmanaged, INumber<T>
    {
        int aRes = int.CreateTruncating(theValue);

        return aRes - (T.CreateTruncating(aRes) > theValue ? 1 : 0);
    }

    /// <summary><c>std::numeric_limits&lt;T&gt;::epsilon()</c>: the machine epsilon of float or double.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Epsilon<T>() where T : unmanaged, INumber<T>
        => typeof(T) == typeof(float) ? T.CreateTruncating(1.1920928955078125E-07)
           : typeof(T) == typeof(double) ? T.CreateTruncating(2.220446049250313E-16)
           : T.Zero;

    /// <summary><c>std::min</c>: the first argument unless the second is smaller.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Min<T>(T theValue1, T theValue2) where T : INumber<T> => theValue2 < theValue1 ? theValue2 : theValue1;

    /// <summary><c>std::max</c>: the first argument unless it is smaller than the second.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Max<T>(T theValue1, T theValue2) where T : INumber<T> => theValue1 < theValue2 ? theValue2 : theValue1;
}
