// Port of: src/FoundationClasses/TKMath/BVH/BVH_Properties.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_Properties.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>Abstract properties of a geometric object.</summary>
public abstract class BVH_Properties : Standard_Transient
{
}

/// <summary>
/// The transformation properties of a geometric object: a 4x4 matrix and its inverse. OCCT's
/// <c>BVH_Transform&lt;T, N&gt;</c> exists only for N = 4 (the only dimension with a matrix type), so the port
/// has the element type alone.
/// </summary>
public class BVH_Transform<T> : BVH_Properties
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
{
    protected NCollection_Mat4<T> myTransform = new();         //!< Transformation matrix
    protected NCollection_Mat4<T> myTransformInversed = new(); //!< Inversed transformation matrix

    /// <summary>Creates a new identity transformation.</summary>
    public BVH_Transform()
    {
    }

    /// <summary>Creates a new transformation with the given matrix (the inverse stays the identity, as in OCCT).</summary>
    public BVH_Transform(in NCollection_Mat4<T> theTransform) => myTransform = theTransform;

    /// <summary>The transformation matrix.</summary>
    public NCollection_Mat4<T> Transform() => myTransform;

    /// <summary>Sets a new transformation matrix, and computes its inverse.</summary>
    public void SetTransform(in NCollection_Mat4<T> theTransform)
    {
        myTransform = theTransform;
        // BVH::MatrixOp<T, 4>::Inverse; the inverse is left as computed when the matrix is singular, as in OCCT
        myTransform.Inverted(out myTransformInversed);
    }

    /// <summary>The inverse transformation matrix.</summary>
    public NCollection_Mat4<T> Inversed() => myTransformInversed;

    /// <summary>Applies the transformation to a bounding box: the box of its eight transformed corners.</summary>
    public BVH_Box<T, NCollection_Vec4<T>> Apply(in BVH_Box<T, NCollection_Vec4<T>> theBox)
    {
        NCollection_Vec4<T> aSize = theBox.Size();

        BVH_Box<T, NCollection_Vec4<T>> aBox = new();
        for (int aX = 0; aX <= 1; ++aX)
        {
            for (int aY = 0; aY <= 1; ++aY)
            {
                for (int aZ = 0; aZ <= 1; ++aZ)
                {
                    NCollection_Vec4<T> aCorner =
                        theBox.CornerMin() + DX() * aSize * T.CreateTruncating(aX)
                        + DY() * aSize * T.CreateTruncating(aY)
                        + DZ() * aSize * T.CreateTruncating(aZ);

                    aBox.Add(Multiply(myTransform, aCorner));
                }
            }
        }

        return aBox;
    }

    // BVH::MatrixOp<T, 4>::Multiply: the product, divided by its w (1.0 / w computed in double, as OCCT does)
    private static NCollection_Vec4<T> Multiply(in NCollection_Mat4<T> theMat, in NCollection_Vec4<T> theVec)
    {
        NCollection_Vec4<T> aOut = theMat * theVec;
        return aOut * T.CreateTruncating(1.0 / double.CreateTruncating(aOut.w()));
    }

    // BVH::UnitVector<T, 4>
    private static NCollection_Vec4<T> DX() => new(T.One, T.Zero, T.Zero, T.Zero);

    private static NCollection_Vec4<T> DY() => new(T.Zero, T.One, T.Zero, T.Zero);

    private static NCollection_Vec4<T> DZ() => new(T.Zero, T.Zero, T.One, T.Zero);
}
