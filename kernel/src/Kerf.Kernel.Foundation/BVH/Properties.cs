// Port of: src/FoundationClasses/TKMath/BVH/BVH_Properties.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_Properties.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>Abstract properties of a geometric object.</summary>
internal abstract class Properties : Transient
{
}

/// <summary>
/// The transformation properties of a geometric object: a 4x4 matrix and its inverse. OCCT's
/// <c>BVH_Transform&lt;T, N&gt;</c> exists only for N = 4 (the only dimension with a matrix type), so the port
/// has the element type alone.
/// </summary>
internal class BvhTransform<T> : Properties
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
{
    protected NCollection_Mat4<T> _transform = new();         //!< Transformation matrix
    protected NCollection_Mat4<T> _transformInversed = new(); //!< Inversed transformation matrix

    /// <summary>Creates a new identity transformation.</summary>
    public BvhTransform()
    {
    }

    /// <summary>Creates a new transformation with the given matrix (the inverse stays the identity, as in OCCT).</summary>
    public BvhTransform(in NCollection_Mat4<T> transform) => _transform = transform;

    /// <summary>The transformation matrix.</summary>
    public NCollection_Mat4<T> Transform() => _transform;

    /// <summary>Sets a new transformation matrix, and computes its inverse.</summary>
    public void SetTransform(in NCollection_Mat4<T> transform)
    {
        _transform = transform;
        // BVH::MatrixOp<T, 4>::Inverse; the inverse is left as computed when the matrix is singular, as in OCCT
        _transform.Inverted(out _transformInversed);
    }

    /// <summary>The inverse transformation matrix.</summary>
    public NCollection_Mat4<T> Inversed() => _transformInversed;

    /// <summary>Applies the transformation to a bounding box: the box of its eight transformed corners.</summary>
    public Box<T, NCollection_Vec4<T>> Apply(in Box<T, NCollection_Vec4<T>> box)
    {
        NCollection_Vec4<T> size = box.Size();

        Box<T, NCollection_Vec4<T>> aBox = new();
        for (int x = 0; x <= 1; ++x)
        {
            for (int y = 0; y <= 1; ++y)
            {
                for (int z = 0; z <= 1; ++z)
                {
                    NCollection_Vec4<T> corner =
                        box.CornerMin() + DX() * size * T.CreateTruncating(x)
                        + DY() * size * T.CreateTruncating(y)
                        + DZ() * size * T.CreateTruncating(z);

                    aBox.Add(Multiply(_transform, corner));
                }
            }
        }

        return aBox;
    }

    // BVH::MatrixOp<T, 4>::Multiply: the product, divided by its w (1.0 / w computed in double, as OCCT does)
    private static NCollection_Vec4<T> Multiply(in NCollection_Mat4<T> mat, in NCollection_Vec4<T> vec)
    {
        NCollection_Vec4<T> aOut = mat * vec;
        return aOut * T.CreateTruncating(1.0 / double.CreateTruncating(aOut.W()));
    }

    // BVH::UnitVector<T, 4>
    private static NCollection_Vec4<T> DX() => new(T.One, T.Zero, T.Zero, T.Zero);

    private static NCollection_Vec4<T> DY() => new(T.Zero, T.One, T.Zero, T.Zero);

    private static NCollection_Vec4<T> DZ() => new(T.Zero, T.Zero, T.One, T.Zero);
}
