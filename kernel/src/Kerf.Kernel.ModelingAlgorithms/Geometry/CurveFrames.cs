// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10M1: the moving frames along an edge's or a wire's curve, which the modelling layer's locations along curves use
// (OCCT's GeomFill trihedron laws).

using System.Collections.Immutable;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Topology;

namespace Kerf.Geometry;

/// <summary>How a moving frame along a curve turns (OCCT's GeomFill trihedron laws).</summary>
public enum FrameLaw
{
    /// <summary>The Frenet frame: the normal points to the centre of curvature, so it can flip where the curve straightens.</summary>
    Frenet,

    /// <summary>The corrected Frenet frame: it turns as little as it can, as a profile swept along the curve does.</summary>
    CorrectedFrenet,
}

/// <summary>A frame on a curve: its point, and the unit tangent, normal and binormal there.</summary>
/// <param name="Point">The point of the curve.</param>
/// <param name="Tangent">The unit tangent.</param>
/// <param name="Normal">The unit normal.</param>
/// <param name="Binormal">The unit binormal: the tangent crossed with the normal.</param>
public readonly record struct CurveFrame(Point3D Point, Vector3D Tangent, Vector3D Normal, Vector3D Binormal);

public static partial class GeometryOperations
{
    extension(EdgeCurve curve)
    {
        /// <summary>
        /// The frames of a law at <paramref name="parameters"/> of the edge's curve (OCCT's <c>GeomFill_Frenet</c> or
        /// <c>GeomFill_CorrectedFrenet</c>): the law is set up on the curve once, and evaluated at each parameter.
        /// </summary>
        /// <exception cref="ArgumentNullException">The curve is null.</exception>
        /// <exception cref="InvalidOperationException">The law can't be set up on the curve.</exception>
        public ImmutableArray<CurveFrame> Frames(ReadOnlySpan<double> parameters, FrameLaw law = FrameLaw.Frenet)
        {
            ArgumentNullException.ThrowIfNull(curve);
            return FramesOf(curve.Adaptor, parameters, law);
        }
    }

    extension(WireCurve curve)
    {
        /// <summary>The frames of a law at <paramref name="parameters"/> of the wire's curve, as an edge's.</summary>
        /// <exception cref="ArgumentNullException">The curve is null.</exception>
        /// <exception cref="InvalidOperationException">The law can't be set up on the curve.</exception>
        public ImmutableArray<CurveFrame> Frames(ReadOnlySpan<double> parameters, FrameLaw law = FrameLaw.Frenet)
        {
            ArgumentNullException.ThrowIfNull(curve);
            return FramesOf(curve.Adaptor, parameters, law);
        }
    }

    private static ImmutableArray<CurveFrame> FramesOf(CurveAdaptorBase adaptor, ReadOnlySpan<double> parameters, FrameLaw law)
    {
        TrihedronLaw trihedron = law switch
        {
            FrameLaw.Frenet => new Frenet(),
            FrameLaw.CorrectedFrenet => new CorrectedFrenet(),
            _ => throw new ArgumentOutOfRangeException(nameof(law), law, "An unknown frame law."),
        };
        trihedron.SetCurve(adaptor);
        var frames = ImmutableArray.CreateBuilder<CurveFrame>(parameters.Length);
        foreach (double u in parameters)
        {
            Vector3D tangent = default, normal = default, binormal = default;
            trihedron.D0(u, ref tangent, ref normal, ref binormal);
            frames.Add(new CurveFrame(adaptor.EvalD0(u), tangent, normal, binormal));
        }

        return frames.MoveToImmutable();
    }
}
