// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10L1: the public surface the axes, frames and transforms add to OCCT's (docs/api/geometry.md). The constants are
// gp's (gp::OX(), gp::XOY()…) as static properties of their types.

using System.Numerics;

namespace Kerf.Geometry;

public readonly partial struct Axis3D
{
    /// <summary>The X axis: through the origin, along (1, 0, 0).</summary>
    public static Axis3D XAxis => new(Direction3D.D.X);

    /// <summary>The Y axis: through the origin, along (0, 1, 0).</summary>
    public static Axis3D YAxis => new(Direction3D.D.Y);

    /// <summary>The Z axis: through the origin, along (0, 0, 1).</summary>
    public static Axis3D ZAxis => new(Direction3D.D.Z);
}

public readonly partial struct Axis2D
{
    /// <summary>The X axis: through the origin, along (1, 0).</summary>
    public static Axis2D XAxis => new(Direction2D.D.X);

    /// <summary>The Y axis: through the origin, along (0, 1).</summary>
    public static Axis2D YAxis => new(Direction2D.D.Y);
}

public readonly partial struct Frame3D
{
    /// <summary>The world frame: at the origin, normal Z, X direction X (gp::XOY()).</summary>
    public static Frame3D World => new(Direction3D.D.Z);

    /// <summary>The frame of the YZ plane: at the origin, normal X, X direction Y (gp::YOZ()).</summary>
    public static Frame3D YZ => new(Direction3D.D.X);

    /// <summary>The frame of the ZX plane: at the origin, normal Y, X direction Z (gp::ZOX()).</summary>
    public static Frame3D ZX => new(Direction3D.D.Y);
}

public readonly partial struct Frame2D
{
    /// <summary>The world frame: at the origin, X direction (1, 0), Y direction (0, 1).</summary>
    public static Frame2D World => new();
}

public readonly partial struct CoordinateSystem
{
    /// <summary>The world coordinate system: at the origin, main direction Z, X direction X.</summary>
    public static CoordinateSystem World => new();
}

public readonly partial struct Transform3D
{
    /// <summary>The identity transformation. <c>default(Transform3D)</c> is not valid: start from this.</summary>
    public static Transform3D Identity => new();

    /// <summary>The translation part: where the transformation takes the origin.</summary>
    public Vector3D TranslationPart => new(_loc);

    /// <summary>A rotation given by Euler angles, in radians, in the order <paramref name="sequence"/> names.</summary>
    public static Transform3D Rotation(EulerSequence sequence, double alpha, double beta, double gamma)
        => Rotation(Kerf.Engine.Geometry.Quaternion3.FromEulerAngles(sequence, alpha, beta, gamma));

    /// <summary>The rotation part's Euler angles, in radians, in the order <paramref name="sequence"/> names.</summary>
    public (double Alpha, double Beta, double Gamma) GetEulerAngles(EulerSequence sequence)
    {
        GetRotation().GetEulerAngles(sequence, out double alpha, out double beta, out double gamma);
        return (alpha, beta, gamma);
    }

    /// <summary>The rotation part as a unit quaternion (OCCT's <c>gp_Quaternion</c> of the matrix).</summary>
    public (double X, double Y, double Z, double W) GetQuaternion()
    {
        Kerf.Engine.Geometry.Quaternion3 q = GetRotation();
        return (q.X(), q.Y(), q.Z(), q.W());
    }

    /// <summary>The rotation part alone, about the origin: this transformation's quaternion as a rotation, without
    /// its translation or scale.</summary>
    public Transform3D ToRotation() => Rotation(GetRotation());

    /// <summary>The transformation as a single-precision 4×4 matrix, for viewers (row vectors, as System.Numerics).</summary>
    public Matrix4x4 ToMatrix4x4()
    {
        Span<double> m = stackalloc double[16];
        GetMat4(m);
        // GetMat4 is OCCT's: column vectors, row by row. System.Numerics multiplies row vectors: the transpose
        return new Matrix4x4((float)m[0], (float)m[4], (float)m[8], (float)m[12],
                             (float)m[1], (float)m[5], (float)m[9], (float)m[13],
                             (float)m[2], (float)m[6], (float)m[10], (float)m[14],
                             (float)m[3], (float)m[7], (float)m[11], (float)m[15]);
    }

    /// <summary>The point transformed.</summary>
    public static Point3D operator *(in Transform3D transform, in Point3D point) => point.Transformed(transform);

    /// <summary>The vector transformed (the translation doesn't apply).</summary>
    public static Vector3D operator *(in Transform3D transform, in Vector3D vector) => vector.Transformed(transform);

    /// <summary>The direction transformed (the translation and the scale don't apply).</summary>
    public static Direction3D operator *(in Transform3D transform, in Direction3D direction) => direction.Transformed(transform);
}

public readonly partial struct Transform2D
{
    /// <summary>The identity transformation. <c>default(Transform2D)</c> is not valid: start from this.</summary>
    public static Transform2D Identity => new();

    /// <summary>The translation part: where the transformation takes the origin.</summary>
    public Vector2D TranslationPart => new(TranslationXy());

    /// <summary>The point transformed.</summary>
    public static Point2D operator *(in Transform2D transform, in Point2D point) => point.Transformed(transform);

    /// <summary>The vector transformed (the translation doesn't apply).</summary>
    public static Vector2D operator *(in Transform2D transform, in Vector2D vector) => vector.Transformed(transform);

    /// <summary>The direction transformed (the translation and the scale don't apply).</summary>
    public static Direction2D operator *(in Transform2D transform, in Direction2D direction) => direction.Transformed(transform);
}

public readonly partial struct GeneralTransform3D
{
    /// <summary>The translation part: where the transformation takes the origin.</summary>
    public Vector3D TranslationPart => new(TranslationXyz());

    /// <summary>The point transformed: the matrix, then the translation, applied to its coordinates (OCCT's
    /// <c>gp_GTrsf::Transforms</c>), so any affine transformation applies, not only a similarity.</summary>
    public static Point3D operator *(in GeneralTransform3D transform, in Point3D point) => new(transform.Transformed(point.XYZ()));
}

public readonly partial struct GeneralTransform2D
{
    /// <summary>The translation part: where the transformation takes the origin.</summary>
    public Vector2D TranslationPart => new(TranslationXy());
}
