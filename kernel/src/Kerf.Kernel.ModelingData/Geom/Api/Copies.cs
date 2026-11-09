// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10L2: the splines' copies with one pole, weight or knot changed, indexed from 0 (docs/api/geometry.md). OCCT's
// 1-based ones are internal (WithPoleOneBased…); a bad index or value is misuse, an ArgumentException.

using Kerf.Engine.Foundation;

namespace Kerf.Geometry;

public sealed partial class BSplineCurve3D
{
    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/>, its weight kept.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's.</exception>
    public BSplineCurve3D WithPole(int index, Point3D pole)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        return WithPoleOneBased(index + 1, pole);
    }

    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/> and given
    /// <paramref name="weight"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BSplineCurve3D WithPole(int index, Point3D pole, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithPoleOneBased(index + 1, pole, weight);
    }

    /// <summary>A copy with the weight of the pole <paramref name="index"/> (from 0) changed; the copy is rational
    /// unless every weight is then equal.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BSplineCurve3D WithWeight(int index, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithWeightOneBased(index + 1, weight);
    }

    /// <summary>A copy with the knot <paramref name="index"/> (from 0) moved to <paramref name="knot"/>, between its
    /// neighbours.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a knot's.</exception>
    /// <exception cref="ArgumentException">The knot wouldn't lie strictly between its neighbours.</exception>
    public BSplineCurve3D WithKnot(int index, double knot)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, KnotCount);
        return CopyChecks.Knot(() => WithKnotOneBased(index + 1, knot));
    }

    /// <summary>A copy with the knot <paramref name="index"/> (from 0) moved to <paramref name="knot"/> and its
    /// multiplicity raised to <paramref name="multiplicity"/> (never lowered).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a knot's.</exception>
    /// <exception cref="ArgumentException">The knot wouldn't lie strictly between its neighbours, or the multiplicity
    /// is more than the degree.</exception>
    public BSplineCurve3D WithKnot(int index, double knot, int multiplicity)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, KnotCount);
        return CopyChecks.Knot(() => WithKnotOneBased(index + 1, knot, multiplicity));
    }
}

public sealed partial class BSplineCurve2D
{
    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/>, its weight kept.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's.</exception>
    public BSplineCurve2D WithPole(int index, Point2D pole)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        return WithPoleOneBased(index + 1, pole);
    }

    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/> and given
    /// <paramref name="weight"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BSplineCurve2D WithPole(int index, Point2D pole, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithPoleOneBased(index + 1, pole, weight);
    }

    /// <summary>A copy with the weight of the pole <paramref name="index"/> (from 0) changed; the copy is rational
    /// unless every weight is then equal.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BSplineCurve2D WithWeight(int index, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithWeightOneBased(index + 1, weight);
    }

    /// <summary>A copy with the knot <paramref name="index"/> (from 0) moved to <paramref name="knot"/>, between its
    /// neighbours.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a knot's.</exception>
    /// <exception cref="ArgumentException">The knot wouldn't lie strictly between its neighbours.</exception>
    public BSplineCurve2D WithKnot(int index, double knot)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, KnotCount);
        return CopyChecks.Knot(() => WithKnotOneBased(index + 1, knot));
    }

    /// <summary>A copy with the knot <paramref name="index"/> (from 0) moved to <paramref name="knot"/> and its
    /// multiplicity raised to <paramref name="multiplicity"/> (never lowered).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a knot's.</exception>
    /// <exception cref="ArgumentException">The knot wouldn't lie strictly between its neighbours, or the multiplicity
    /// is more than the degree.</exception>
    public BSplineCurve2D WithKnot(int index, double knot, int multiplicity)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, KnotCount);
        return CopyChecks.Knot(() => WithKnotOneBased(index + 1, knot, multiplicity));
    }
}

public sealed partial class BezierCurve3D
{
    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/>, its weight kept.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's.</exception>
    public BezierCurve3D WithPole(int index, Point3D pole)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        return WithPoleOneBased(index + 1, pole);
    }

    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/> and given
    /// <paramref name="weight"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BezierCurve3D WithPole(int index, Point3D pole, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithPoleOneBased(index + 1, pole, weight);
    }

    /// <summary>A copy with the weight of the pole <paramref name="index"/> (from 0) changed; the copy is rational
    /// unless every weight is then equal.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BezierCurve3D WithWeight(int index, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithWeightOneBased(index + 1, weight);
    }
}

public sealed partial class BezierCurve2D
{
    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/>, its weight kept.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's.</exception>
    public BezierCurve2D WithPole(int index, Point2D pole)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        return WithPoleOneBased(index + 1, pole);
    }

    /// <summary>A copy with the pole <paramref name="index"/> (from 0) moved to <paramref name="pole"/> and given
    /// <paramref name="weight"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BezierCurve2D WithPole(int index, Point2D pole, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithPoleOneBased(index + 1, pole, weight);
    }

    /// <summary>A copy with the weight of the pole <paramref name="index"/> (from 0) changed; the copy is rational
    /// unless every weight is then equal.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> isn't a pole's, or the weight isn't
    /// positive.</exception>
    public BezierCurve2D WithWeight(int index, double weight)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(index);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, PoleCount);
        CopyChecks.Weight(weight);
        return WithWeightOneBased(index + 1, weight);
    }
}

public sealed partial class BSplineSurface
{
    /// <summary>A copy with the pole at (<paramref name="uIndex"/>, <paramref name="vIndex"/>), from 0, moved to
    /// <paramref name="pole"/>, its weight kept.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An index isn't a pole's.</exception>
    public BSplineSurface WithPole(int uIndex, int vIndex, Point3D pole)
    {
        CheckPoleIndex(uIndex, vIndex);
        return WithPoleOneBased(uIndex + 1, vIndex + 1, pole);
    }

    /// <summary>A copy with the pole at (<paramref name="uIndex"/>, <paramref name="vIndex"/>), from 0, moved to
    /// <paramref name="pole"/> and given <paramref name="weight"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An index isn't a pole's, or the weight isn't positive.</exception>
    public BSplineSurface WithPole(int uIndex, int vIndex, Point3D pole, double weight)
    {
        CheckPoleIndex(uIndex, vIndex);
        CopyChecks.Weight(weight);
        return WithPoleOneBased(uIndex + 1, vIndex + 1, pole, weight);
    }

    /// <summary>A copy with the weight of the pole at (<paramref name="uIndex"/>, <paramref name="vIndex"/>), from 0,
    /// changed.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An index isn't a pole's, or the weight isn't positive.</exception>
    public BSplineSurface WithWeight(int uIndex, int vIndex, double weight)
    {
        CheckPoleIndex(uIndex, vIndex);
        CopyChecks.Weight(weight);
        return WithWeightOneBased(uIndex + 1, vIndex + 1, weight);
    }

    private void CheckPoleIndex(int uIndex, int vIndex)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(uIndex);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(uIndex, UPoleCount);
        KernelArgumentOutOfRangeException.ThrowIfNegative(vIndex);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(vIndex, VPoleCount);
    }
}

public sealed partial class BezierSurface
{
    /// <summary>A copy with the pole at (<paramref name="uIndex"/>, <paramref name="vIndex"/>), from 0, moved to
    /// <paramref name="pole"/>, its weight kept.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An index isn't a pole's.</exception>
    public BezierSurface WithPole(int uIndex, int vIndex, Point3D pole)
    {
        CheckPoleIndex(uIndex, vIndex);
        return WithPoleOneBased(uIndex + 1, vIndex + 1, pole);
    }

    /// <summary>A copy with the pole at (<paramref name="uIndex"/>, <paramref name="vIndex"/>), from 0, moved to
    /// <paramref name="pole"/> and given <paramref name="weight"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An index isn't a pole's, or the weight isn't positive.</exception>
    public BezierSurface WithPole(int uIndex, int vIndex, Point3D pole, double weight)
    {
        CheckPoleIndex(uIndex, vIndex);
        CopyChecks.Weight(weight);
        return WithPoleOneBased(uIndex + 1, vIndex + 1, pole, weight);
    }

    /// <summary>A copy with the weight of the pole at (<paramref name="uIndex"/>, <paramref name="vIndex"/>), from 0,
    /// changed.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An index isn't a pole's, or the weight isn't positive.</exception>
    public BezierSurface WithWeight(int uIndex, int vIndex, double weight)
    {
        CheckPoleIndex(uIndex, vIndex);
        CopyChecks.Weight(weight);
        return WithWeightOneBased(uIndex + 1, vIndex + 1, weight);
    }

    private void CheckPoleIndex(int uIndex, int vIndex)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(uIndex);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(uIndex, UPoleCount);
        KernelArgumentOutOfRangeException.ThrowIfNegative(vIndex);
        KernelArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(vIndex, VPoleCount);
    }
}

internal static class CopyChecks
{
    // OCCT refuses a weight at most gp::Resolution()
    public static void Weight(double weight, [System.Runtime.CompilerServices.CallerArgumentExpression(nameof(weight))] string? name = null)
    {
        if (!(weight > Kerf.Engine.Geometry.Constants.Resolution()))
        {
            throw new KernelArgumentOutOfRangeException(name, weight, "A weight must be positive.");
        }
    }

    public static T Knot<T>(Func<T> copy)
    {
        try
        {
            return copy();
        }
        catch (Exception e) when (Refusal.Of(e) is global::Kerf.Engine.Foundation.ConstructionErrorException)
        {
            throw new KernelArgumentException("The knot must lie strictly between its neighbours, with a multiplicity no more than the degree.", e);
        }
    }
}
