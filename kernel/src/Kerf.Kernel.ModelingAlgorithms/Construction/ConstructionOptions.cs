// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Construction;

/// <summary>A sphere's angles, for a part of one (OCCT's <c>BRepPrimAPI_MakeSphere</c>), in radians.</summary>
public sealed record SphereOptions
{
    /// <summary>The whole sphere.</summary>
    public static SphereOptions Default { get; } = new();

    /// <summary>The lowest latitude, from -π/2.</summary>
    public double MinLatitude { get; init; } = -Math.PI / 2;

    /// <summary>The highest latitude, up to π/2.</summary>
    public double MaxLatitude { get; init; } = Math.PI / 2;

    /// <summary>The sweep about the axis, up to 2π.</summary>
    public double Angle { get; init; } = 2 * Math.PI;
}

/// <summary>A torus's angles, for a part of one, in radians.</summary>
public sealed record TorusOptions
{
    /// <summary>The whole torus.</summary>
    public static TorusOptions Default { get; } = new();

    /// <summary>
    /// Where the minor circle starts, about its centre from the side away from the axis. Part of a minor circle makes a
    /// slice of the tube: the section is that arc closed by a straight line across its ends. (OCCT's
    /// <c>BRepPrimAPI_MakeTorus</c> makes the solid under the arc instead, closed by flat discs to the axis.)
    /// </summary>
    public double MinorStart { get; init; }

    /// <summary>Where the minor circle ends, up to <see cref="MinorStart"/> + 2π.</summary>
    public double MinorEnd { get; init; } = 2 * Math.PI;

    /// <summary>The sweep about the axis, up to 2π.</summary>
    public double Angle { get; init; } = 2 * Math.PI;
}

/// <summary>A wedge's top face (OCCT's <c>BRepPrimAPI_MakeWedge</c>): either its X length (<see cref="TopLength"/>), or its
/// extent in X and Z (<see cref="TopMin"/>, <see cref="TopMax"/>), at Y = dy.</summary>
public sealed record WedgeOptions
{
    /// <summary>The top face's length in X, from 0 (a prism with a triangular end) to dx (a box).</summary>
    public double? TopLength { get; init; }

    /// <summary>The top face's (X, Z) minimum corner, with <see cref="TopMax"/>.</summary>
    public (double X, double Z)? TopMin { get; init; }

    /// <summary>The top face's (X, Z) maximum corner, with <see cref="TopMin"/>.</summary>
    public (double X, double Z)? TopMax { get; init; }
}

/// <summary>How <see cref="Sweeps.Extrude"/> extrudes (OCCT's <c>BRepPrimAPI_MakePrism</c>).</summary>
public sealed record ExtrudeOptions
{
    /// <summary>Check the profile, the only way an extrusion comes out invalid, and if it isn't valid, the result; fail with <c>result.invalid</c>, carrying it, if that isn't valid either: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults.</summary>
    public static ExtrudeOptions Default { get; } = new();

    /// <summary>Simplify the swept surfaces to planes, cylinders… where they are (OCCT's <c>Canonize</c>).</summary>
    public bool Canonize { get; init; } = true;
}

/// <summary>How a sweep keeps its profile's orientation along the spine (OCCT's <c>BRepOffsetAPI_MakePipeShell.SetMode</c>).</summary>
public enum SweepFrame
{
    /// <summary>The corrected Frenet frame, which doesn't twist where the curvature vanishes (OCCT's default).</summary>
    CorrectedFrenet,

    /// <summary>The Frenet frame.</summary>
    Frenet,

    /// <summary>The discrete frame, for spines with sharp turns.</summary>
    Discrete,

    /// <summary>A fixed frame (<see cref="SweepOptions.FixedFrame"/>): the profile is translated, not turned.</summary>
    Fixed,

    /// <summary>A constant binormal (<see cref="SweepOptions.Binormal"/>), as for a helix.</summary>
    Binormal,

    /// <summary>The normal follows an auxiliary spine (<see cref="SweepOptions.AuxiliarySpine"/>).</summary>
    Auxiliary,
}

/// <summary>How <see cref="Sweeps.Sweep(Shape, Wire, SweepOptions?, IProgress{double}?, CancellationToken)"/> sweeps.</summary>
public sealed record SweepOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid (a sweep that folds over itself): true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults: a solid where the profiles are closed, corrected Frenet, transformed corners.</summary>
    public static SweepOptions Default { get; } = new();

    /// <summary>Close the result into a solid when the profiles are closed (ignored for open profiles).</summary>
    public bool MakeSolid { get; init; } = true;

    /// <summary>How the corners of a spine that isn't tangent-continuous are treated.</summary>
    public TransitionMode Transition { get; init; } = TransitionMode.Transformed;

    /// <summary>How the profile's orientation follows the spine.</summary>
    public SweepFrame Frame { get; init; } = SweepFrame.CorrectedFrenet;

    /// <summary>
    /// If a sweep with the corrected Frenet frame fails, try the Frenet frame and then the discrete one, and take the first
    /// valid result, with a warning saying which (<see cref="Sweeps.SweptWithFrenet"/>, <see cref="Sweeps.SweptWithDiscrete"/>).
    /// Otherwise, or if none works, the first failure is the result. The other frames are left as they are.
    /// </summary>
    public bool TryOtherFrames { get; init; } = true;

    /// <summary>The binormal, for <see cref="SweepFrame.Binormal"/>.</summary>
    public Direction3D? Binormal { get; init; }

    /// <summary>The frame, for <see cref="SweepFrame.Fixed"/>.</summary>
    public Frame3D? FixedFrame { get; init; }

    /// <summary>The auxiliary spine, for <see cref="SweepFrame.Auxiliary"/>.</summary>
    public Wire? AuxiliarySpine { get; init; }

    /// <summary>For <see cref="SweepFrame.Auxiliary"/>: the auxiliary spine's points matched to the spine's by the fraction
    /// of their lengths (OCCT's <c>CurvilinearEquivalence</c>) rather than by the planes normal to the spine.</summary>
    public bool CurvilinearEquivalence { get; init; }

    /// <summary>Translate each profile to touch the spine.</summary>
    public bool WithContact { get; init; }

    /// <summary>Rotate each profile to be orthogonal to the spine's tangent.</summary>
    public bool WithCorrection { get; init; }

    /// <summary>The 3D tolerance of the approximation.</summary>
    public double Tolerance { get; init; } = 1e-4;
}

/// <summary>How <see cref="Sweeps.Loft"/> passes surfaces through its sections (OCCT's <c>BRepOffsetAPI_ThruSections</c>).</summary>
public sealed record LoftOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults: a smooth solid.</summary>
    public static LoftOptions Default { get; } = new();

    /// <summary>Close the result into a solid (the first and last sections closed wires, or vertices).</summary>
    public bool MakeSolid { get; init; } = true;

    /// <summary>Ruled surfaces between consecutive sections, rather than one smooth surface through them all.</summary>
    public bool Ruled { get; init; }

    /// <summary>The tolerance of the approximation.</summary>
    public double Tolerance { get; init; } = 1e-6;

    /// <summary>Check the sections' orientation and starting points, and match them.</summary>
    public bool CheckCompatibility { get; init; } = true;

    /// <summary>The continuity of the smooth surface.</summary>
    public Continuity Continuity { get; init; } = Continuity.C2;

    /// <summary>The highest degree of the smooth surface.</summary>
    public int MaxDegree { get; init; } = 8;

    /// <summary>How the sections are given parameters.</summary>
    public ParametrizationType Parametrization { get; init; } = ParametrizationType.ChordLength;

    /// <summary>Smooth the approximation (OCCT's <c>SetSmoothing</c>).</summary>
    public bool Smoothing { get; init; }
}

/// <summary>How <see cref="Sweeps.Helix(in Frame3D, double, double, double, HelixOptions?)"/> builds a helix (OCCT's
/// <c>HelixBRep_BuilderHelix</c>; its defaults).</summary>
public sealed record HelixOptions
{
    /// <summary>The defaults: a cylindrical helix.</summary>
    public static HelixOptions Default { get; } = new();

    /// <summary>The radius at the top, for a conical helix; null for the start radius (a cylindrical one).</summary>
    public double? EndRadius { get; init; }

    /// <summary>The 3D tolerance of the approximation.</summary>
    public double Tolerance { get; init; } = 0.0001;

    /// <summary>The highest degree of the approximating B-splines.</summary>
    public int MaxDegree { get; init; } = 8;

    /// <summary>The continuity of the approximation, and between its edges.</summary>
    public Continuity Continuity { get; init; } = Continuity.C1;
}
