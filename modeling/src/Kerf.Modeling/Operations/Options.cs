using System.Collections.Immutable;

namespace Kerf.Modeling;

// The options of the operations with more than a few settings, in place of long lists of optional parameters, as the
// kernel's operations take them: XxxOptions? options = null, null meaning XxxOptions.Default. They are nested in
// Operations because the kernel has records of the same names (Kerf.Construction.ExtrudeOptions and others): with
// `using static Kerf.Modeling.Operations;` they are in scope by their short names.
public static partial class Operations
{
    /// <summary>How <see cref="Chamfer(IEnumerable{Edge}, double, double?, double?, Face?, ChamferOptions?)"/> chamfers.</summary>
    public sealed record ChamferOptions
    {
        /// <summary>The defaults: a chamfer longer than a convex arc's radius on its face is refused.</summary>
        public static ChamferOptions Default { get; } = new();

        /// <summary>
        /// What to do at a convex arc whose radius is no more than the length on its face: refuse, or let the corner
        /// collapse to a point (a Kerf addition). Only a chamfer with such an arc is affected.
        /// </summary>
        public TightCorners TightCorners { get; init; } = TightCorners.Refuse;
    }

    /// <summary>How <see cref="Extrude(Shape, double, ExtrudeOptions?)"/> extrudes.</summary>
    public sealed record ExtrudeOptions
    {
        /// <summary>The defaults: along each face's normal, one way, no taper, cleaned.</summary>
        public static ExtrudeOptions Default { get; } = new();

        /// <summary>The direction to extrude in, instead of each face's normal; non-planar faces need it.</summary>
        public Vector3D? Direction { get; init; }

        /// <summary>Extrude both ways from the faces.</summary>
        public bool Both { get; init; }

        /// <summary>
        /// A taper angle in degrees: positive narrows the extrusion away from the face, negative widens it. The sides are
        /// drafted where they are planes, cylinders or cones, which keeps sharp corners sharp, and lofted otherwise. Only
        /// applied when extruding by an amount.
        /// </summary>
        public double Taper { get; init; }

        /// <summary>Remove extraneous internal structure.</summary>
        public bool Clean { get; init; } = true;
    }

    /// <summary>How <see cref="Sweep(IEnumerable{Shape}, Shape, SweepOptions?)"/> sweeps.</summary>
    public sealed record SweepOptions
    {
        /// <summary>The defaults: each section swept on its own, corrected Frenet frames, transformed corners, cleaned.</summary>
        public static SweepOptions Default { get; } = new();

        /// <summary>Sweep one solid through all the sections (their outer wires).</summary>
        public bool Multisection { get; init; }

        /// <summary>Use the path's Frenet frame.</summary>
        public bool IsFrenet { get; init; }

        /// <summary>How a section turns a corner of the path.</summary>
        public Transition Transition { get; init; } = Transition.Transformed;

        /// <summary>A fixed normal for the sections.</summary>
        public Vector3D? Normal { get; init; }

        /// <summary>An edge or wire guiding the rotation of the sections along the path.</summary>
        public IShape1D? Binormal { get; init; }

        /// <summary>Remove extraneous internal structure.</summary>
        public bool Clean { get; init; } = true;
    }

    /// <summary>How <see cref="Offset(IEnumerable{Shape}, double, OffsetOptions?)"/> offsets.</summary>
    public sealed record OffsetOptions
    {
        /// <summary>The defaults: arcs at the corners, both sides of an edge, closed, nothing left open.</summary>
        public static OffsetOptions Default { get; } = new();

        /// <summary>Faces of the solids to leave open (a hollow box with no lid).</summary>
        public ImmutableArray<Face> Openings { get; init; } = [];

        /// <summary>The transition at corners.</summary>
        public Kind Kind { get; init; } = Kind.Arc;

        /// <summary>For edges, which side to offset to.</summary>
        public Side Side { get; init; } = Side.Both;

        /// <summary>For edges offset to one side, close the offset back to the edges.</summary>
        public bool Closed { get; init; } = true;

        /// <summary>Remove edges of the offset wires shorter than this, which an offset can degenerate into.</summary>
        public double? MinEdgeLength { get; init; }
    }

    /// <summary>How <see cref="Thicken(Shape, double, ThickenOptions?)"/> thickens.</summary>
    public sealed record ThickenOptions
    {
        /// <summary>The defaults: along each face's normal, one way, cleaned.</summary>
        public static ThickenOptions Default { get; } = new();

        /// <summary>
        /// Which way is up: faces whose normals point the other way (by more than 90°) are flipped, so that faces with
        /// different normals all go the same way.
        /// </summary>
        public Vector3D? NormalOverride { get; init; }

        /// <summary>Thicken both ways from the faces.</summary>
        public bool Both { get; init; }

        /// <summary>Remove extraneous internal structure.</summary>
        public bool Clean { get; init; } = true;
    }
}
