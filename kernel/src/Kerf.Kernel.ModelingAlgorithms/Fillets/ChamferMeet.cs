// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// A chamfer whose corner ChFi3d couldn't build, built with its chamfer faces carried on until they meet (unit 15U; decisions
/// 2137, 2144): each stripe ChFi3d computed before its corner failed, its surface extended past the corner and across its
/// contacts, a general fuse of those faces with the input's, and the pieces kept that bound the region the chamfered shape is.
/// </summary>
/// <remarks>
/// <para>
/// <b>The region.</b> Beside an edge a chamfer is the input with the side of its face away from the material taken off (a
/// convex edge) or added (a concave one). At a corner where chamfers meet, the edges there that aren't chamfered say how they
/// combine: where those are convex, the material is on the material side of all the chamfers (convex chamfers cut in turn;
/// concave ones wrapping a convex corner fill only where they all do); where they're concave, of any. Near a corner every
/// chamfer that ends there counts, its extended surface whole; elsewhere a chamfer counts only beside its own edge. The input's
/// material is the side of its nearest face behind its normal, as the faces are oriented in a shell.
/// </para>
/// <para>
/// <b>The pieces.</b> A piece of the fuse is kept where the region is on one side of it and not the other, turned to face
/// out of it. The fuse leaves the cuts that end inside a face (where an extended surface stops) as internal edges; they're
/// taken out.
/// </para>
/// <para>
/// <b>What's built:</b> open shells and faces, with every corner of one kind (its other edges all convex or all concave),
/// the chamfers there all convex or all concave, a corner with one chamfer only where it and the corner are convex, and a
/// corner whose edges are all chamfered where its chamfers are all convex; every chamfer face one of ChFi3d's B-spline
/// stripes, its contacts its u-isos, one of ChFiKPart's planes between two planes, taken as a patch over its contacts (unit
/// 15U3), or one of Kerf's own (unit 15V: <c>ChamferMeet.Chains.cs</c>) where stripes run on through a vertex or ChFi3d's
/// stripe is off the chamfer's length. A corner of convex and concave edges is built from its faces' formula (unit 15U2:
/// <c>ChamferMeet.Mixed.cs</c>). A solid is proved by its volume against an integral apart from the fuse
/// (<c>ChamferMeet.Volume.cs</c>). What doesn't fit gives null, and ChFi3d's failure stands.
/// </para>
/// <para>
/// <b>The proof</b> (<see cref="Proved"/>), of the result alone and the chamfer's specs, not of how it was made: every free
/// edge runs along one of the input's, or is a chamfer face's end between two; and where a chamfer face meets one of its edge's
/// faces beside the edge, the contact is at the chamfer's distance on that face, and on the reference face at the chamfer's
/// angle (ChFi3d's: in the plane square to the edge). The operation then checks validity, tolerances and the crossings
/// (decision 2125) as for any result. Everything is local to the call, on frozen shapes.
/// </para>
/// </remarks>
internal static partial class ChamferMeet
{
    // How far past a corner a chamfer's surface is carried, and how far across its contacts, in the widest chamfer's widths
    private const double Past = 2;

    private const double Across = 1;

    // Within this many widths of a corner, the corner's rule holds; a chamfer counts beside its edge out to this many widths
    private const double Near = 2;

    // How much farther than that where Kerf's own bands are among them (unit 15V)
    private const double OwnReach = 3;

    // The fewest contact points checked on each chamfer
    private const int FewestChecked = 5;

    // Points along each contact of a plane's stripe, of which the other chamfers have to leave one
    private const int ContactSamples = 33;

    private sealed class Band
    {
        public required BSplineSurface Original { get; init; }

        public required double UMax { get; init; }      // the contacts: u = 0 on S1, u = UMax on S2

        public required Point2D A1 { get; init; }       // the contacts' ends on the surface: A at the first end, B at the last

        public required Point2D A2 { get; init; }

        public required Point2D B1 { get; init; }

        public required Point2D B2 { get; init; }

        public required double Width { get; init; }

        public required bool Convex { get; init; }

        public required Vertex FirstCorner { get; init; }   // the corner at each end, or null where the end is the edge's

        public required Vertex LastCorner { get; init; }

        public required Point3D EdgePoint { get; init; }    // a point of the chamfered edge, beside the band's middle

        public required Edge[] Edges { get; init; }

        public Face Face1 { get; init; }   // the faces its contacts lie on, u = 0 and u = UMax, where it's ChFi3d's

        public Face Face2 { get; init; }

        public BSplineSurface Surface { get; set; } = null!; // extended

        public double EU0 { get; set; }

        public double EU1 { get; set; }

        public double EV0 { get; set; }

        public double EV1 { get; set; }

        public double EdgeSign { get; set; }

        public Face Face { get; set; }

        public double VLo => Math.Min(A1.Y, A2.Y);

        public double VHi => Math.Max(B1.Y, B2.Y);

        // The face's v range at u: past a corner the extended surface's; at the edge's end the line through the contacts' ends
        public double Lo(double u) => !FirstCorner.IsNull ? EV0 : A1.Y + ((A2.Y - A1.Y) * u / UMax);

        public double Hi(double u) => !LastCorner.IsNull ? EV1 : B1.Y + ((B2.Y - B1.Y) * u / UMax);

        public bool EndsAt(Vertex corner) => FirstCorner.IsSame(corner) || LastCorner.IsSame(corner);

        // A parameter along a closed band taken on to its period from lo, where the band's half runs past the surface's end
        public double Within(double v, double lo)
        {
            if (!Original.IsVPeriodic)
            {
                return v;
            }
            double period = Original.VPeriod;
            return v < lo ? v + (period * Math.Ceiling((lo - v) / period)) : v;
        }

        public bool Own { get; init; }   // Kerf's own band through a chain (unit 15V), not one of ChFi3d's stripes

        public bool Flat { get; init; }  // a patch of ChFiKPart's plane (unit 15U3), carried on as the plane it is

        // An own band runs on through the vertex: one of its edges ends there, and the vertex's section is inside its span
        public bool Passes(Vertex corner)
        {
            if (!Edges.Any(e => e.AllVertices().Any(v => v.IsSame(corner))))
            {
                return false;
            }
            var projection = new ProjectPointOnSurface(corner.Point, Original);
            if (projection.NbPoints() == 0)
            {
                return false;
            }
            projection.LowerDistanceParameters(out _, out double v);
            v = Within(v, VLo);
            return v > VLo && v < VHi;
        }
    }

    // How a corner's chamfers combine: all or any (a corner of one kind), or by the formula of its faces (unit 15U2)
    private readonly record struct Corner(Vertex Vertex, Point3D Point, bool All, Band[] Bands, Term? Mixed = null);

    /// <summary>
    /// The chamfer built with its faces carried on until they meet, where ChFi3d's <paramref name="chamfer"/> has computed its
    /// stripes and failed at a corner, and proved; null where this doesn't apply, what it builds isn't proved, or it took longer
    /// than the options' <see cref="ChamferOptions.TimeLimit"/> (<paramref name="late"/>; the owner, decisions 2179 and 2185).
    /// A solid is proved by its volume too (unit 15V).
    /// </summary>
    /// <exception cref="OperationCanceledException">The caller's own token, or the one in force, was cancelled.</exception>
    public static EngineOutcome? Build(Shape shape, MakeChamfer chamfer, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options,
                                       out bool late)
    {
        late = false;
        if (options.Mode != ChamferMode.Classic || shape.AllSolids().Length > 1 || shape.AllFaces().Length == 0)
        {
            return null;
        }

        // The limit is a deadline joined to the token in force, so a caller's own cancellation still stops it, and as itself
        CancellationToken caller = Cancellation.Current;
        using var deadline = CancellationTokenSource.CreateLinkedTokenSource(caller);
        if (options.TimeLimit != Timeout.InfiniteTimeSpan)
        {
            deadline.CancelAfter(options.TimeLimit);
        }
        try
        {
            using (Cancellation.Use(deadline.Token))
            {
                return Attempts(shape, chamfer, specs, referenceFaces);
            }
        }
        catch (OperationCanceledException) when (!caller.IsCancellationRequested)
        {
            late = true;
            return null;
        }
    }

    // Kerf's own bands for ChFi3d's stripes that are off the chamfer's distance first, and if that isn't built, ChFi3d's (unless
    // a chain through a vertex needs Kerf's own anyway, whose corners ChFi3d's stripes beside it don't make)
    private static EngineOutcome? Attempts(Shape shape, MakeChamfer chamfer, ChamferSpec[] specs, Face[] referenceFaces)
    {
        bool anyOff = false;
        foreach (bool own in (ReadOnlySpan<bool>)[true, false])
        {
            try
            {
                if (Built(shape, chamfer, specs, referenceFaces, own, ref anyOff) is { } built)
                {
                    return built;
                }
            }
            catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
            {
            }
            Cancellation.ThrowIfRequested();
            if (!anyOff)
            {
                break;
            }
        }
        return null;
    }

    private static EngineOutcome? Built(Shape shape, MakeChamfer chamfer, ChamferSpec[] specs, Face[] referenceFaces, bool ownWhereOff, ref bool anyOff)
    {
        ChBuilder builder = chamfer.InternalBuilder();
        if (builder._listStripe.IsEmpty() || (builder._badstripes.Extent() != 0 && !ownWhereOff))
        {
            return null;
        }

        // The stripes along edges; those without a spine are corners ChFi3d built, which are built again here
        Stripe[] stripes = [.. builder._listStripe.Where(static s => s.Spine() is { EdgeCount: > 0 })];

        // The corners: where ChFi3d failed, and every end of a stripe that isn't at the edge of the shape
        var corners = new List<Vertex>();
        foreach (Shape v in builder._badvertices)
        {
            corners.Add(ShapeCasts.Vertex(v));
        }
        foreach (Stripe stripe in stripes)
        {
            Spine spine = stripe.Spine()!;
            if (spine.FirstStatus() != State.FreeBoundary && !corners.Any(v => v.IsSame(spine.FirstVertex())))
            {
                corners.Add(spine.FirstVertex());
            }
            if (spine.LastStatus() != State.FreeBoundary && !corners.Any(v => v.IsSame(spine.LastVertex())))
            {
                corners.Add(spine.LastVertex());
            }
        }
        if (corners.Count == 0)
        {
            return null;
        }

        Face[] faces = shape.AllFaces().ToArray();
        var chamfered = new List<Edge>();
        var bands = new List<Band>();

        // Chains that run on through a vertex, as one band of Kerf's own (unit 15V); a vertex inside one is a corner only
        // where another chamfer ends
        List<Chain> chains = Chains(faces, stripes, corners);
        bool chained = chains.Count > 0;
        var off = new List<Stripe>();
        foreach (Stripe stripe in stripes)
        {
            Cancellation.ThrowIfRequested();
            if (chains.Any(c => c.Stripes.Contains(stripe)))
            {
                continue;
            }
            // A stripe ChFi3d couldn't walk has no surface: Kerf's own band, where its chamfer is of one length (unit 15Y)
            if (builder._badstripes.Any(b => ReferenceEquals(b, stripe)))
            {
                Spine walked = stripe.Spine()!;
                if (!ownWhereOff || EqualDistance(specs, referenceFaces, [.. Enumerable.Range(1, walked.EdgeCount).Select(walked.Edges)]) is null)
                {
                    return null;
                }
                off.Add(stripe);
                continue;
            }
            if (Bands(faces, builder, stripe, corners) is not { } own)
            {
                return null;
            }
            // ChFi3d's stripe off the chamfer's distance (as on a loft's sides): Kerf's own instead
            if (ownWhereOff && EqualDistance(specs, referenceFaces, own[0].Edges) is { } distance && own.Any(b => !OnSpec(b, distance, builder._tolapp3d)))
            {
                anyOff = !chained;
                off.Add(stripe);
                continue;
            }
            bands.AddRange(own);
            Spine spine = stripe.Spine()!;
            for (int j = 1; j <= spine.EdgeCount; j++)
            {
                chamfered.Add(spine.Edges(j));
            }
        }

        chains.AddRange(Chains(faces, [.. off], corners, alone: true));
        foreach (Chain chain in chains)
        {
            if (EqualDistance(specs, referenceFaces, [.. chain.Edges.Select(static x => x.Edge)]) is not { } distance
                || chain.Edges.Select(x => IsConvex(faces, x.Edge)).Distinct().ToArray() is not [{ } convex]
                || ChainBands(faces, chain, distance, convex) is not { } chainBands)
            {
                return null;
            }
            bands.AddRange(chainBands);
            chamfered.AddRange(chain.Edges.Select(static x => x.Edge));
        }

        // How far the surfaces are carried, in the widest chamfer's widths, and where the corner's rule holds: three times as far
        // where Kerf's own bands are among them, as the loft's corners need (2 widths left its bottom corners' faces uncut)
        double width = bands.Max(static b => b.Width) * (bands.Any(static b => b.Own) ? OwnReach : 1);

        // How the chamfers combine at each corner
        var cornerRules = new List<Corner>();
        Edge[] edges = shape.AllEdges().ToArray();
        foreach (Vertex corner in corners)
        {
            Edge[] others = [.. edges.Where(e => !chamfered.Any(c => c.IsSame(e)) && e.AllVertices().Any(v => v.IsSame(corner)))];
            Band[] ending = [.. bands.Where(b => b.EndsAt(corner))];
            if (ending.Length == 0)
            {
                continue; // inside a chain, which runs on through it
            }
            Band[] at = [.. ending, .. bands.Where(b => b.Own && !b.EndsAt(corner) && b.Passes(corner))];
            bool?[] kinds = others.Length == 0 && at.Length > 1 && at.All(static b => b.Convex) ? [true] : [.. others.Select(e => IsConvex(faces, e)).Distinct()];
            if (kinds is [true, false] or [false, true])
            {
                // Convex and concave: by the formula of the faces there (unit 15U2)
                if (MixedFormula(faces, corner, Near * width, at) is not { } formula)
                {
                    return null;
                }
                cornerRules.Add(new Corner(corner, corner.Point, false, at, formula));
                continue;
            }
            if (kinds.Length != 1 || kinds[0] is not { } all || at.Length == 0 || at.Any(b => b.Convex != at[0].Convex)
                || (at.Length == 1 && !(at[0].Convex && all)))
            {
                return null;
            }
            cornerRules.Add(new Corner(corner, corner.Point, all, at));
        }

        // The faces of the mixed corners' formulas that aren't their chamfers' own, carried on past their edges (unit 15U2)
        var carried = new List<Carried>();
        foreach (Corner c in cornerRules.Where(static c => c.Mixed is not null))
        {
            foreach (Leaf leaf in Leaves(c.Mixed!))
            {
                if (c.Bands.Any(b => b.Face1.IsSame(leaf.Face) || b.Face2.IsSame(leaf.Face)) || carried.Any(x => x.Input.IsSame(leaf.Face)))
                {
                    continue;
                }
                if (Carry(faces.First(f => f.IsSame(leaf.Face)), Past * width) is not { } face)
                {
                    return null;
                }
                carried.Add(face);
            }
        }

        // Each surface carried past its corners and across its contacts, its parameters kept, and made a face
        foreach (Band band in bands)
        {
            BoundedSurface extended = band.Own ? Across1(band.Original, Across * width / band.Width) : band.Original;
            if (band.Flat)
            {
                // A plane's patch runs by length both ways, so it's the same plane on a larger rectangle
                band.Original.ParameterBounds(out double fu0, out double fu1, out double fv0, out double fv1);
                band.Original.D1(fu0, fv0, out Point3D corner, out Vector3D du, out Vector3D dv);
                extended = FlatPatch(corner - (du * fu0) - (dv * fv0), du, dv, fu0 - (Across * width), fu1 + (Across * width),
                                     fv0 - (band.FirstCorner.IsNull ? 0 : Past * width), fv1 + (band.LastCorner.IsNull ? 0 : Past * width));
            }
            else
            {
                if (!band.LastCorner.IsNull)
                {
                    GeomLib.ExtendSurfByLength(ref extended, Past * width, 1, false, true);
                }
                if (!band.FirstCorner.IsNull)
                {
                    GeomLib.ExtendSurfByLength(ref extended, Past * width, 1, false, false);
                }
                if (!band.Own)
                {
                    GeomLib.ExtendSurfByLength(ref extended, Across * width, 1, true, true);
                    GeomLib.ExtendSurfByLength(ref extended, Across * width, 1, true, false);
                }
            }
            if (extended is not BSplineSurface surface)
            {
                return null;
            }
            surface.ParameterBounds(out double u0, out double u1, out double v0, out double v1);
            Point3D middle = band.Original.EvalD0(band.UMax / 2, (band.VLo + band.VHi) / 2);
            if (surface.EvalD0(band.UMax / 2, (band.VLo + band.VHi) / 2).DistanceTo(middle) > Precision.Confusion())
            {
                return null; // the extension moved the parameters
            }
            band.Surface = surface;
            (band.EU0, band.EU1, band.EV0, band.EV1) = (u0, u1, v0, v1);
            band.Face = Polygon(surface, [new(u0, band.Lo(u0)), new(u1, band.Lo(u1)), new(u1, band.Hi(u1)), new(u0, band.Hi(u0))]);
            if (Project(band, band.EdgePoint) is not { } foot)
            {
                return null;
            }
            band.EdgeSign = Math.Sign(new Vector3D(foot.Point, band.EdgePoint).Dot(foot.Normal));
            if (band.EdgeSign == 0)
            {
                return null;
            }
        }

        // A plane's stripe has to keep some of each of its contacts. Where other chamfers' surfaces take all of one away (the
        // four edges of a cube's top chamfered by half its width or more), what the fuse bounds is those planes' meeting and
        // not the chamfer asked, and the proof, which counts contacts on either face, wouldn't see it (unit 15U3)
        foreach (Band band in bands.Where(static b => b.Flat))
        {
            foreach (double u in (ReadOnlySpan<double>)[0, band.UMax])
            {
                bool left = false;
                for (int k = 0; k < ContactSamples && !left; k++)
                {
                    Point3D x = band.Original.EvalD0(u, band.VLo + ((band.VHi - band.VLo) * (k + 0.5) / ContactSamples));
                    left = !bands.Any(other => !ReferenceEquals(other.Edges, band.Edges) && TakesAway(other, x));
                }
                if (!left)
                {
                    return null;
                }
            }
        }

        // The input's material: behind its faces as one shell orients them, the side the bands' middles say
        Face[] oriented = Oriented(shape, faces);
        var near = new NearestFace(oriented);
        int[] signs = [.. bands.Select(b => (b.Convex ? 1 : -1) * Math.Sign(near.Side(b.Original.EvalD0(b.UMax / 2, (b.VLo + b.VHi) / 2)))).Distinct()];
        if (signs.Length != 1 || signs[0] == 0)
        {
            return null;
        }
        int material = signs[0];
        // In a solid, its classifier says (the nearest face is ambiguous outside it beside an edge)
        SolidClassifier? classifier = shape.AllSolids().Length == 1 ? new SolidClassifier(shape.AllSolids()[0]) : null;
        bool Material(Point3D x)
        {
            if (classifier is null)
            {
                return Math.Sign(near.Side(x)) == material;
            }
            classifier.Perform(x, Precision.Confusion());
            return classifier.State() == Containment.In;
        }

        // The band's side of a point: true on the chamfered shape's material side; null where it says nothing
        bool? SideOf(Band band, Point3D x, bool nearCorner)
        {
            if (Project(band, x) is not { } foot)
            {
                return null;
            }
            double along = new Vector3D(foot.Point, x).Dot(foot.Normal);
            if (!nearCorner)
            {
                double lo = band.FirstCorner.IsNull ? band.Lo(foot.U) : band.VLo, hi = band.LastCorner.IsNull ? band.Hi(foot.U) : band.VHi;
                double distance = x.DistanceTo(foot.Point);
                double v = band.Within(foot.V, lo);
                if (v < lo || v > hi || distance > Near * width || Math.Abs(along) < 0.99 * distance)
                {
                    return null;
                }
            }
            bool edgeSide = Math.Sign(along) == band.EdgeSign;
            return band.Convex ? !edgeSide : edgeSide;
        }
        // Behind a face of a mixed corner's formula: its surface carried on, where it is, else its own
        bool BehindFace(Face face, Point3D x)
        {
            Face shelled = oriented.First(f => f.IsSame(face));
            if (carried.FirstOrDefault(x => x.Input.IsSame(face)) is { } c)
            {
                return Behind(shelled, c.Surface, c.U0, c.U1, c.V0, c.V1, x, material);
            }
            Surface surface = Tool.Surface(face)!;
            surface.ParameterBounds(out double u0, out double u1, out double v0, out double v1);
            return Behind(shelled, surface, u0, u1, v0, v1, x, material);
        }
        bool Region(Point3D x)
        {
            bool inside = Material(x);
            Corner[] close = [.. cornerRules.Where(c => c.Point.DistanceTo(x) < Near * width).OrderBy(c => c.Point.DistanceTo(x))];
            if (close.Length > 0)
            {
                Corner c = close[0];
                if (c.Mixed is { } formula)
                {
                    return Holds(formula, x, BehindFace, (b, p) => SideOf(b, p, true));
                }
                bool[] sides = [.. c.Bands.Select(b => SideOf(b, x, true)).OfType<bool>()];
                if (sides.Length == 0)
                {
                    return inside;
                }
                bool held = c.All ? sides.All(static s => s) : sides.Any(static s => s);
                return c.Bands[0].Convex ? inside && held : inside || held;
            }
            foreach (Band band in bands)
            {
                if (SideOf(band, x, false) is { } side && side != band.Convex)
                {
                    inside = !band.Convex;
                }
            }
            return inside;
        }

        // The fuse, and the pieces that bound the region
        var arguments = new NCollection_List<Shape>();
        foreach (Face face in faces)
        {
            if (carried.FirstOrDefault(x => x.Input.IsSame(face)) is { } c)
            {
                arguments.Append(c.Replacement);
                arguments.Append(c.Ring);
                continue;
            }
            arguments.Append(face);
        }
        foreach (Band band in bands)
        {
            arguments.Append(band.Face);
        }
        BuilderAlgo fuse = Fuse(arguments);
        // The input is frozen, and where the fuse has to raise the tolerance of one of its vertices that a new edge ends on, it
        // fails (OCCT writes it in place): then the fuse of copies, made together so that they share what the arguments do
        // (unit 15U3)
        Copy? copies = null;
        if (fuse.HasErrors)
        {
            copies = new Copy(Builder.MakeCompound([.. arguments]), false);
            var copied = new NCollection_List<Shape>();
            foreach (Shape argument in arguments)
            {
                copied.Append(copies.InternalModifiedShape(argument));
            }
            fuse = Fuse(copied);
        }
        Shape Fused(Shape argument) => copies is null ? argument : copies.InternalModifiedShape(argument);
        if (fuse.HasErrors || !fuse.IsDone || fuse.History() is not { } made)
        {
            return null;
        }
        var context = new IntersectionContext();
        double step = 1e-4 * width;
        // The rings' pieces, of which only those near a mixed corner can be kept
        var ringPieces = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (Carried c in carried)
        {
            foreach (Shape piece in Images(made, Fused(c.Ring)))
            {
                ringPieces.Add(piece);
            }
        }
        var kept = new Dictionary<Shape, Face>(ShapeHasher.Instance);
        var result = new BrepBuilder();
        Shell shell = default;
        result.MakeShell(ref shell);
        foreach (Face piece in fuse.Shape().AllFaces())
        {
            Cancellation.ThrowIfRequested();
            Point3D p = default;
            Point2D uv = default;
            if (!InFace(piece, ref p, ref uv, context) || Tool.Surface(piece) is not { } surface)
            {
                return null;
            }
            if (ringPieces.Contains(piece) && !cornerRules.Any(c => c.Mixed is not null && c.Point.DistanceTo(p) < Near * width))
            {
                continue;
            }
            Vector3D n = Normal(surface, uv.X, uv.Y);
            if (piece.Orientation == Orientation.Reversed)
            {
                n = -n;
            }
            bool behind = Region(p - (step * n)), before = Region(p + (step * n));
            if (behind != before)
            {
                Face face = WithoutInternal(behind ? piece : ShapeCasts.Face(piece.Reversed()));
                kept[piece] = face;
                result.Add(shell, face);
            }
        }
        if (kept.Count == 0)
        {
            return null;
        }

        // A face the extended surfaces cut in parts that were all kept is one face again
        var unifier = new UnifySameDomain(shell, true, true, true);
        unifier.SetSafeInputMode(true);
        unifier.AllowInternalEdges(false);
        unifier.Build();
        Shape built = unifier.InternalShape();
        if (shape.AllSolids().Length == 1 && built.Type == ShapeType.Shell)
        {
            var solidBuilder = new BrepBuilder();
            Solid solid = default;
            solidBuilder.MakeSolid(ref solid);
            solidBuilder.Add(solid, built);
            built = solid;
        }
        if (built.Type != (shape.AllSolids().Length == 1 ? ShapeType.Solid : ShapeType.Shell) || Proved(shape, built, specs, referenceFaces, chamfer, builder._tolapp3d) is not true)
        {
            return null;
        }
        // A solid's volume against the integral apart from the fuse's faces (unit 15V)
        if (shape.AllSolids().Length == 1)
        {
            double measured = MeasuredVolume(built);
            (double Expected, double Allowed)? volume = ExpectedVolume(shape, faces, bands, cornerRules, carried, [.. chamfered], specs, referenceFaces, builder._tolapp3d, Near * width);
            if (volume is not { } v || Math.Abs(measured - v.Expected) > v.Allowed)
            {
                return null;
            }
        }
        History unified = unifier.InternalHistory()!;

        // The history: each input face to its pieces kept, or removed; each chamfered edge to its chamfer's faces
        var history = new History();
        foreach (Face face in faces)
        {
            Shape[] given = carried.FirstOrDefault(x => x.Input.IsSame(face)) is { } c ? [c.Replacement, c.Ring] : [face];
            Face[] own = [.. given.SelectMany(g => Images(made, Fused(g))).Where(kept.ContainsKey).Select(p => kept[p])];
            if (own.Length == 0)
            {
                history.Remove(face);
            }
            foreach (Face piece in own.Where(p => !p.IsSame(face)))
            {
                history.AddModified(face, piece);
            }
        }
        foreach (Band band in bands)
        {
            foreach (Shape piece in made.Modified(Fused(band.Face)).Where(kept.ContainsKey))
            {
                foreach (Edge edge in band.Edges)
                {
                    history.AddGenerated(edge, kept[piece]);
                }
            }
        }
        history.Merge(unified);
        return new EngineOutcome(true, () => built, history);
    }

    // The general fuse of the faces, on the caller's thread, with a progress range so that it sees the limit or a cancellation
    // between its intersections
    private static BuilderAlgo Fuse(NCollection_List<Shape> arguments)
    {
        var fuse = new BuilderAlgo();
        fuse.SetArguments(arguments);
        fuse.SetToFillHistory(true);
        fuse.SetRunParallel(false);
        fuse.Build(new ProgressIndicator().Start());
        Cancellation.ThrowIfRequested();
        return fuse;
    }

    // What the fuse made of an argument: its pieces, or the argument itself where it's whole
    private static IReadOnlyList<Shape> Images(History made, Shape argument)
    {
        IReadOnlyList<Shape> pieces = made.Modified(argument);
        return pieces.Count > 0 ? pieces : made.IsRemoved(argument) ? [] : [argument];
    }

    // The bands of one stripe, one for each of its surfaces; null where a surface isn't a B-spline with its contacts its u-isos,
    // or the stripe's faces aren't its edges'
    private static List<Band>? Bands(Face[] faces, ChBuilder builder, Stripe stripe, List<Vertex> corners)
    {
        NCollection_HSequence<SurfData>? set = stripe.SetOfSurfData();
        Spine spine = stripe.Spine()!;
        if (set is null || set.Length() == 0)
        {
            return null;
        }
        Edge[] own = [.. Enumerable.Range(1, spine.EdgeCount).Select(spine.Edges)];
        if (own.Select(e => IsConvex(faces, e)).Distinct().ToArray() is not [{ } convex])
        {
            return null;
        }
        Vertex first = corners.FirstOrDefault(v => v.IsSame(spine.FirstVertex())), last = corners.FirstOrDefault(v => v.IsSame(spine.LastVertex()));
        var bands = new List<Band>();
        for (int k = 1; k <= set.Length(); k++)
        {
            SurfData data = set.Value(k);
            Surface? computed = builder._ds.Surface(data.Surf()).Surface();
            if (computed is not (BSplineSurface or PlaneSurface))
            {
                return null;
            }
            Shape s1 = builder._ds.Shape(data.IndexOfS1()), s2 = builder._ds.Shape(data.IndexOfS2());
            if (!own.Any(e => OnFace(e, s1) && OnFace(e, s2)))
            {
                return null;
            }
            FaceInterference on1 = data.InterferenceOnS1(), on2 = data.InterferenceOnS2();
            if (on1.PCurveOnSurf() is not { } c1 || on2.PCurveOnSurf() is not { } c2)
            {
                return null;
            }
            Point2D a1 = c1.EvalD0(on1.FirstParameter()), b1 = c1.EvalD0(on1.LastParameter());
            Point2D a2 = c2.EvalD0(on2.FirstParameter()), b2 = c2.EvalD0(on2.LastParameter());
            // ChFiKPart's plane between two planes along a straight edge: a patch of it over its contacts (unit 15U3)
            bool flat = computed is PlaneSurface;
            if (flat)
            {
                if (FlatBand(computed, ref a1, ref b1, ref a2, ref b2) is not { } patch)
                {
                    return null;
                }
                computed = patch;
            }
            var surface = (BSplineSurface)computed;
            surface.ParameterBounds(out double u0, out double umax, out _, out _);
            if (Math.Abs(u0) > Precision.PConfusion() || Math.Abs(a1.X) > Precision.PConfusion() || Math.Abs(b1.X) > Precision.PConfusion()
                || Math.Abs(a2.X - umax) > Precision.PConfusion() || Math.Abs(b2.X - umax) > Precision.PConfusion())
            {
                return null;
            }

            // ChFi3d's u runs from 0 to the stripe's length along its edge whatever the band's width: across a band 0.02 wide
            // and 10 long the surface moves 0.002 for each unit of u. Carried on across by a length, which adds that length
            // in u, its speed jumps 500-fold where the extension starts, and the fuse's curves on such a surface came out up
            // to 0.5 off it (unit 15Y, decision 2182). So u by the band's width, the surface the same
            double width = surface.EvalD0(0, (a1.Y + b1.Y) / 2).DistanceTo(surface.EvalD0(umax, (a2.Y + b2.Y) / 2));
            if (width > Precision.Confusion() && !surface.IsUPeriodic && !flat)
            {
                double scale = width / umax;
                surface = new BSplineSurface(surface.Poles, [.. surface.UKnots.Select(k => k * scale)], [.. surface.VKnots],
                                             [.. surface.UMultiplicities], [.. surface.VMultiplicities], surface.UDegree, surface.VDegree,
                                             false, surface.IsVPeriodic, surface.IsURational || surface.IsVRational ? surface.Weights : default);
                umax = width;
                a2 = new Point2D(umax, a2.Y);
                b2 = new Point2D(umax, b2.Y);
            }
            Point3D middle = surface.EvalD0(umax / 2, (Math.Min(a1.Y, a2.Y) + Math.Max(b1.Y, b2.Y)) / 2);
            Point3D? onEdge = null;
            double nearest = double.MaxValue;
            foreach (Edge e in own)
            {
                if (Tool.Curve(e, out double f, out double l) is not { } curve)
                {
                    continue;
                }
                var projection = new ProjectPointOnCurve(middle, curve, f, l);
                if (projection.NbPoints() > 0 && projection.LowerDistance() < nearest)
                {
                    nearest = projection.LowerDistance();
                    onEdge = projection.NearestPoint();
                }
            }
            if (onEdge is not { } edgePoint)
            {
                return null;
            }
            bands.Add(new Band
            {
                Original = surface,
                UMax = umax,
                A1 = a1,
                A2 = a2,
                B1 = b1,
                B2 = b2,
                Width = width,
                Convex = convex,
                FirstCorner = k == 1 ? first : default,
                LastCorner = k == set.Length() ? last : default,
                EdgePoint = edgePoint,
                Edges = own,
                Face1 = ShapeCasts.Face(s1),
                Face2 = ShapeCasts.Face(s2),
                Flat = flat,
            });
        }
        return bands;
    }

    // A plane's stripe as a patch over its contacts, which are lines along the edge: across from the first contact to the
    // second by the stripe's width, along by length from the first contact's start; the contacts' ends given on it. Null
    // where the contacts aren't two lines in line with each other
    private static BSplineSurface? FlatBand(Surface plane, ref Point2D a1, ref Point2D b1, ref Point2D a2, ref Point2D b2)
    {
        Point3D pa1 = plane.EvalD0(a1.X, a1.Y), pb1 = plane.EvalD0(b1.X, b1.Y), pa2 = plane.EvalD0(a2.X, a2.Y), pb2 = plane.EvalD0(b2.X, b2.Y);
        // One contact may be a point, where the stripe narrows to nothing on a face that ends
        Vector3D along = pa1.DistanceTo(pb1) >= pa2.DistanceTo(pb2) ? new Vector3D(pa1, pb1) : new Vector3D(pa2, pb2);
        if (along.Magnitude <= Precision.Confusion())
        {
            return null;
        }
        along /= along.Magnitude;
        var to = new Vector3D(pa1, pa2);
        Vector3D across = to - (along * to.Dot(along));
        double width = across.Magnitude;
        if (width <= Precision.Confusion())
        {
            return null;
        }
        across /= width;
        Point2D On(Point3D p) => new(new Vector3D(pa1, p).Dot(across), new Vector3D(pa1, p).Dot(along));
        (a1, b1, a2, b2) = (On(pa1), On(pb1), On(pa2), On(pb2));
        double lo = Math.Min(a1.Y, a2.Y), hi = Math.Max(b1.Y, b2.Y);
        if (Math.Abs(b1.X) > Precision.Confusion() || Math.Abs(b2.X - width) > Precision.Confusion() || b1.Y < a1.Y || b2.Y < a2.Y
            || hi - lo <= Precision.Confusion())
        {
            return null;
        }
        (a1, b1, a2, b2) = (new(0, a1.Y), new(0, b1.Y), new(width, a2.Y), new(width, b2.Y));
        return FlatPatch(pa1, across, along, 0, width, lo, hi);
    }

    // The patch of a plane on [u0, u1] by [v0, v1]: the origin and u along one unit vector, v along the other
    private static BSplineSurface FlatPatch(Point3D origin, Vector3D u, Vector3D v, double u0, double u1, double v0, double v1) =>
        new(new Grid<Point3D>(new[] { origin + (u * u0) + (v * v0), origin + (u * u0) + (v * v1), origin + (u * u1) + (v * v0), origin + (u * u1) + (v * v1) }, 2, 2),
            [u0, u1], [v0, v1], [2, 2], [2, 2], 1, 1, false, false);

    // The one length of an equal-distance chamfer on all these edges, or null: Kerf's own bands are of one length
    private static double? EqualDistance(ChamferSpec[] specs, Face[] referenceFaces, Edge[] edges) =>
        CutOf(specs, referenceFaces, edges) is { Cut.Equal: true } asked ? asked.Cut.Distance : null;

    private static bool OnFace(Edge edge, Shape face) => face.AllEdges().Any(e => e.IsSame(edge));

    // Convex (true) or concave (false) at the edge's middle, by its faces as they lie in the shape; null if it hasn't two
    // faces or they're tangent there
    private static bool? IsConvex(Face[] faces, Edge edge)
    {
        Face[] two = [.. faces.Where(f => OnFace(edge, f))];
        if (two.Length != 2 || Tool.Curve(edge, out double first, out double last) is not { } curve)
        {
            return null;
        }
        double t = (first + last) / 2;
        Point3D at = curve.EvalD0(t);
        Edge inFirst = two[0].AllEdges().First(e => e.IsSame(edge)), inSecond = two[1].AllEdges().First(e => e.IsSame(edge));
        AlgoTools3D.GetNormalToFaceOnEdge(inSecond, two[1], t, out Direction3D normal, null);
        if (two[1].Orientation == Orientation.Reversed)
        {
            normal = normal.Reversed();
        }
        Point2D uv = default;
        Point3D into = default;
        if (AlgoTools3D.PointNearEdge(inFirst, two[0], t, 1e-3, ref uv, ref into, new IntersectionContext()) != 0)
        {
            return null;
        }
        var inward = new Vector3D(at, into);
        double s = new Vector3D(normal.X, normal.Y, normal.Z).Dot(inward) / inward.Magnitude;
        return s < -1e-3 ? true : s > 1e-3 ? false : null;
    }

    // The faces as one shell orients them, where they aren't one shell already
    private static Face[] Oriented(Shape shape, Face[] faces)
    {
        if (shape.Type == ShapeType.Shell || shape.AllShells().Length == 1 && shape.AllShells()[0].AllFaces().Length == faces.Length)
        {
            return faces;
        }
        var builder = new BrepBuilder();
        Shell shell = default;
        builder.MakeShell(ref shell);
        foreach (Face face in faces)
        {
            builder.Add(shell, face);
        }
        var fixer = new ShellFixer();
        fixer.FixFaceOrientation(shell);
        return fixer.Shell().AllFaces().ToArray();
    }

    // A convex chamfer's surface, carried on, has the point on its edge's side or on it: its nearest point inside the surface
    private static bool TakesAway(Band band, Point3D x)
    {
        if (!band.Convex || Project(band, x) is not { } foot)
        {
            return false;
        }
        double inU = Precision.PConfusion() * (band.EU1 - band.EU0), inV = Precision.PConfusion() * (band.EV1 - band.EV0);
        return foot.U > band.EU0 + inU && foot.U < band.EU1 - inU && foot.V > band.EV0 + inV && foot.V < band.EV1 - inV
               && new Vector3D(foot.Point, x).Dot(foot.Normal) * band.EdgeSign > -Precision.Confusion();
    }

    private readonly record struct Foot(double U, double V, Point3D Point, Vector3D Normal);

    // The nearest point of the band's extended surface, with its unit normal
    private static Foot? Project(Band band, Point3D p)
    {
        var projection = new ProjectPointOnSurface(p, band.Surface, band.EU0, band.EU1, band.EV0, band.EV1);
        if (projection.NbPoints() == 0)
        {
            return null;
        }
        int best = 1;
        for (int i = 2; i <= projection.NbPoints(); i++)
        {
            if (projection.Distance(i) < projection.Distance(best))
            {
                best = i;
            }
        }
        projection.Parameters(best, out double u, out double v);
        Vector3D n = Normal(band.Surface, u, v);
        return new Foot(u, v, band.Surface.EvalD0(u, v), n);
    }

    private static Vector3D Normal(Surface surface, double u, double v)
    {
        surface.D1(u, v, out _, out Vector3D du, out Vector3D dv);
        Vector3D n = du.Crossed(dv);
        double length = n.Magnitude;
        return length > 0 ? n / length : n;
    }

    // The side of a point's nearest face: its distance along the face's normal as the face lies, so negative behind it
    private sealed class NearestFace(Face[] faces)
    {
        private readonly Box[] _boxes = [.. faces.Select(static f =>
        {
            var box = new Box();
            Bounds.Add(f, ref box);
            return box;
        })];

        public double Side(Point3D p)
        {
            var vertex = new LibraryMakeVertex(p).InternalVertex();
            var point = new Box();
            point.Add(p);
            double best = double.MaxValue, side = 0;
            foreach (int k in Enumerable.Range(0, faces.Length).OrderBy(k => _boxes[k].Distance(point)))
            {
                if (_boxes[k].Distance(point) > best)
                {
                    break;
                }
                var extrema = new ExtPF(vertex, faces[k]);
                if (!extrema.IsDone)
                {
                    continue;
                }
                for (int n = 1; n <= extrema.ExtCount; n++)
                {
                    double d = Math.Sqrt(extrema.SquareDistance(n));
                    if (d >= best)
                    {
                        continue;
                    }
                    extrema.Parameter(n, out double u, out double v);
                    Vector3D normal = Normal(Tool.Surface(faces[k])!, u, v);
                    if (faces[k].Orientation == Orientation.Reversed)
                    {
                        normal = -normal;
                    }
                    best = d;
                    side = new Vector3D(extrema.Point(n), p).Dot(normal);
                }
            }
            return side;
        }
    }

    // The face of the surface bounded by the polygon of uv
    private static Face Polygon(Surface surface, Point2D[] uv)
    {
        var builder = new BrepBuilder();
        Vertex[] vertices = [.. uv.Select(q => new LibraryMakeVertex(surface.EvalD0(q.X, q.Y)).InternalVertex())];
        var wire = new LibraryMakeWire();
        for (int i = 0; i < uv.Length; i++)
        {
            Point2D a = uv[i], b = uv[(i + 1) % uv.Length];
            var line = new LineCurve2D(a, new Direction2D(b.X - a.X, b.Y - a.Y));
            wire.Add(new LibraryMakeEdge(line, surface, vertices[i], vertices[(i + 1) % uv.Length], 0, a.DistanceTo(b)).InternalEdge());
        }
        Face face = default;
        builder.MakeFace(ref face, surface, Precision.Confusion());
        builder.Add(face, wire.InternalWire());
        BrepLibrary.BuildCurves3d(face);
        return face;
    }

    // A point inside the face: PointInFace's, or else one beside the middle of one of its edges, as PointInFace's hatching can
    // miss a narrow piece
    private static bool InFace(Face face, ref Point3D p, ref Point2D uv, IntersectionContext context)
    {
        if (AlgoTools3D.PointInFace(face, ref p, ref uv, context) == 0)
        {
            return true;
        }
        foreach (Edge edge in face.Explore<Edge>(ShapeType.Edge))
        {
            if (Tool.Degenerated(edge) || edge.Orientation is not (Orientation.Forward or Orientation.Reversed))
            {
                continue;
            }
            Tool.Range(edge, out double a, out double b);
            foreach (double step in (ReadOnlySpan<double>)[1e-4, 1e-6])
            {
                if (AlgoTools3D.PointNearEdge(edge, face, 0.5 * (a + b), step, ref uv, ref p) == 0 && context.IsPointInFace(face, uv))
                {
                    return true;
                }
            }
        }
        return false;
    }

    // The face without the cuts that end inside it, which the fuse leaves as internal edges
    private static Face WithoutInternal(Face face)
    {
        if (!face.AllEdges().Any(static e => e.Orientation is Orientation.Internal or Orientation.External)
            && !face.Explore<Edge>(ShapeType.Edge).Any(static e => e.Orientation is Orientation.Internal or Orientation.External))
        {
            return face;
        }
        var builder = new BrepBuilder();
        Face forward = ShapeCasts.Face(face.Oriented(Orientation.Forward));
        Face copy = ShapeCasts.Face(forward.EmptyCopied());
        foreach (Wire wire in forward.Explore<Wire>(ShapeType.Wire))
        {
            Wire kept = ShapeCasts.Wire(wire.EmptyCopied());
            int count = 0;
            foreach (Edge edge in wire.Explore<Edge>(ShapeType.Edge))
            {
                if (edge.Orientation is not (Orientation.Internal or Orientation.External))
                {
                    builder.Add(kept, edge);
                    count++;
                }
            }
            if (count > 0)
            {
                kept.Closed(true);
                builder.Add(copy, kept);
            }
        }
        return ShapeCasts.Face(copy.Oriented(face.Orientation));
    }

    /// <summary>
    /// True if <paramref name="result"/> is the chamfer the specs ask for, read from it alone: its free edges are the input's
    /// or a chamfer face's ends between them, and its chamfer faces' contacts beside their edges are at the specs' distances and
    /// angles, within <paramref name="tolerance"/> (ChFi3d's approximation) and the edges' own.
    /// </summary>
    internal static bool Proved(Shape input, Shape result, ChamferSpec[] specs, Face[] referenceFaces, MakeChamfer contours, double tolerance) =>
        // Five or more on each side of each contour: a chamfer that has lost one whole contact isn't the chamfer asked
        CheckedContacts(input, result, specs, referenceFaces, contours, tolerance) is { } checkedOn
        && Enumerable.Range(0, checkedOn.GetLength(0)).All(i => checkedOn[i, 0] >= FewestChecked && checkedOn[i, 1] >= FewestChecked);

    // The contact points found at their specs on each side of each contour, [contour, side]; null if a free edge isn't the
    // input's or a chamfer's end, a contact is off its spec, or a chamfer face is no contour's
    internal static int[,]? CheckedContacts(Shape input, Shape result, ChamferSpec[] specs, Face[] referenceFaces, MakeChamfer contours, double tolerance)
    {
        Face[] inputFaces = input.AllFaces().ToArray();
        Face[] faces = result.AllFaces().ToArray();
        var context = new IntersectionContext();

        // A point inside each input face, for faces carried on past their edges (unit 15U2)
        Point3D?[] inside = [.. inputFaces.Select(f =>
        {
            Point3D q = default;
            Point2D quv = default;
            return AlgoTools3D.PointInFace(f, ref q, ref quv, context) == 0 ? q : (Point3D?)null;
        })];

        // Each face of the result: the input face it lies on, or none (a chamfer's)
        var origin = new Dictionary<Shape, int>(ShapeHasher.Instance);
        foreach (Face face in faces)
        {
            Point3D p = default;
            Point2D uv = default;
            if (AlgoTools3D.PointInFace(face, ref p, ref uv, context) != 0)
            {
                return null;
            }
            origin[face] = OnInputFace(p);
            // A face carried on past its edges (unit 15U2) may have that point beyond its input face: then the input face whose
            // point inside lies on its surface (a chamfer's surface crosses the input's faces only along curves)
            if (origin[face] < 0 && Tool.Surface(face) is { } own)
            {
                own.ParameterBounds(out double u0, out double u1, out double v0, out double v1);
                origin[face] = Array.FindIndex(inside, q =>
                {
                    if (q is not { } at)
                    {
                        return false;
                    }
                    var onIt = double.IsInfinity(u0) || double.IsInfinity(v0) ? new ProjectPointOnSurface(at, own) : new ProjectPointOnSurface(at, own, u0, u1, v0, v1);
                    return onIt.NbPoints() > 0 && onIt.LowerDistance() < Precision.Confusion();
                });
            }
        }

        // The input face a point lies on, or -1
        int OnInputFace(Point3D p)
        {
            Vertex vertex = new LibraryMakeVertex(p).InternalVertex();
            return Array.FindIndex(inputFaces, f =>
            {
                var extrema = new ExtPF(vertex, f);
                return extrema.IsDone && Enumerable.Range(1, extrema.ExtCount).Any(n => extrema.SquareDistance(n) < Precision.SquareConfusion());
            });
        }
        if (!Closed(input, faces, f => origin[f] >= 0, tolerance))
        {
            return null;
        }

        // The contours, each with its spec and reference face
        var lines = new List<(Edge[] Edges, ChamferSpec Spec, Face Reference)>();
        for (int ic = 1; ic <= contours.ContourCount; ic++)
        {
            Edge[] own = [.. Enumerable.Range(1, contours.NbEdges(ic)).Select(j => contours.Edge(ic, j))];
            int at = Array.FindIndex(specs, s => own.Any(e => e.IsSame(s.Edge)));
            if (at < 0)
            {
                return null;
            }
            lines.Add((own, specs[at], referenceFaces[at]));
        }

        // Contours that run on into each other through a vertex, their edges tangent there, are one chamfer face (unit 15V)
        int[] group = [.. Enumerable.Range(0, lines.Count)];
        int Root(int i) => group[i] == i ? i : group[i] = Root(group[i]);
        for (int i = 0; i < lines.Count; i++)
        {
            for (int j = i + 1; j < lines.Count; j++)
            {
                if (RunOn(lines[i].Edges, lines[j].Edges))
                {
                    group[Root(j)] = Root(i);
                }
            }
        }

        // Each contour's two sides (unit 15U4): for each of its edges, its two faces, the first of them on the side of the
        // first edge's first face. A side is followed from an edge to the next by the face the two share; where they share
        // none, by the face whose normal at their vertex is nearest the last one's. An edge of one face is on both sides
        Face[][][] sides = [.. lines.Select(l => SidesAlong(l.Edges, inputFaces))];

        // Each chamfer face's contacts: first which contours it's of (most of its contact points nearest their edges inside
        // them), then each such point at its distance and angle from its nearest edge of those, counted on its contour's side
        var checkedOn = new int[lines.Count, 2];
        foreach (Face chamferFace in faces.Where(f => origin[f] < 0))
        {
            List<(Point3D Point, Face Face)> contacts = Contacts(chamferFace, 20);
            var votes = new int[lines.Count];
            foreach ((Point3D x, Face face) in contacts)
            {
                if (OnEdge(x, face, lines, _ => true) is { } foot)
                {
                    votes[Root(foot.Line)]++;
                }
            }
            if (votes.Max() == 0)
            {
                return null;
            }
            int chosen = Array.IndexOf(votes, votes.Max());
            int size = Enumerable.Range(0, lines.Count).Count(i => Root(i) == chosen);
            if (size > 1)
            {
                contacts = Contacts(chamferFace, 20 * size);
            }
            foreach ((Point3D x, Face face) in contacts)
            {
                if (OnEdge(x, face, lines, i => Root(i) == chosen) is not { } foot)
                {
                    continue;
                }
                int line = foot.Line;
                ChamferSpec spec = lines[line].Spec;
                bool onReference = lines[line].Reference.IsSame(face);
                double allowed = tolerance + foot.Tolerance;
                double? distance = spec.Angle is not null ? (onReference ? spec.Distance : null) : onReference ? spec.Distance : spec.Distance2 ?? spec.Distance;
                if (distance is { } d && Math.Abs(x.DistanceTo(foot.Point) - d) > allowed)
                {
                    return null;
                }
                if (spec.Angle is { } angle && onReference)
                {
                    Vector3D along = foot.Tangent;
                    Vector3D onFace = NormalAt(face, x).Crossed(along), onChamfer = NormalAt(chamferFace, x).Crossed(along);
                    double between = Math.Acos(Math.Min(1, Math.Abs(onFace.Dot(onChamfer)) / (onFace.Magnitude * onChamfer.Magnitude)));
                    if (Math.Abs(between - angle) > allowed / spec.Distance)
                    {
                        return null;
                    }
                }
                Face[] two = sides[line][foot.Edge];
                if (two[0].IsSame(face))
                {
                    checkedOn[line, 0]++;
                }
                if (two[1].IsSame(face))
                {
                    checkedOn[line, 1]++;
                }
            }
        }

        // The contact points of a chamfer face: along each edge it shares with a face on one of the input's, with that face
        List<(Point3D Point, Face Face)> Contacts(Face chamferFace, int count)
        {
            var contacts = new List<(Point3D Point, Face Face)>();
            foreach (Edge edge in chamferFace.AllEdges())
            {
                Face other = faces.FirstOrDefault(f => !f.IsSame(chamferFace) && f.AllEdges().Any(e => e.IsSame(edge)));
                if (other.IsNull || origin[other] < 0 || Tool.Curve(edge, out double a, out double b) is not { } curve)
                {
                    continue;
                }
                for (int k = 1; k < count; k++)
                {
                    contacts.Add((curve.EvalD0(a + ((b - a) * k / count)), inputFaces[origin[other]]));
                }
            }
            return contacts;
        }
        return checkedOn;
    }

    // The two faces of each edge of a contour, side by side along it: [edge][side]
    private static Face[][] SidesAlong(Edge[] edges, Face[] inputFaces)
    {
        var sides = new Face[edges.Length][];
        for (int j = 0; j < edges.Length; j++)
        {
            Face[] own = [.. inputFaces.Where(f => OnFace(edges[j], f))];
            Face[] two = own.Length switch { 0 => [default, default], 1 => [own[0], own[0]], _ => [own[0], own[1]] };
            if (j > 0 && own.Length > 1)
            {
                Face[] last = sides[j - 1];
                bool same = two[0].IsSame(last[0]) || two[1].IsSame(last[1]), crossed = two[0].IsSame(last[1]) || two[1].IsSame(last[0]);
                if (same == crossed && !last[0].IsNull
                    && edges[j].AllVertices().FirstOrDefault(v => edges[j - 1].AllVertices().Any(w => w.IsSame(v))) is { IsNull: false } shared)
                {
                    Point3D at = shared.Point;
                    double Along(Face a, Face b) => Math.Abs(NormalAt(a, at).Dot(NormalAt(b, at)));
                    crossed = Along(two[0], last[1]) + Along(two[1], last[0]) > Along(two[0], last[0]) + Along(two[1], last[1]);
                    same = !crossed;
                }
                if (crossed && !same)
                {
                    two = [two[1], two[0]];
                }
            }
            sides[j] = two;
        }
        return sides;
    }

    // Two contours run on into each other: an edge of each ends at the same vertex, their tangents there in line
    private static bool RunOn(Edge[] one, Edge[] two)
    {
        foreach (Edge a in one)
        {
            foreach (Edge b in two)
            {
                foreach (Vertex v in a.AllVertices())
                {
                    if (b.AllVertices().Any(w => w.IsSame(v)) && TangentAt(a, v) is { } ta && TangentAt(b, v) is { } tb
                        && Math.Abs(Math.Abs(ta.Dot(tb)) - 1) < Tangent)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private readonly record struct EdgeFoot(int Line, int Edge, Point3D Point, Vector3D Tangent, double Tolerance);

    // The nearest point of a chamfered edge of the face, inside the edge, on one of the contour lines asked
    private static EdgeFoot? OnEdge(Point3D x, Face face, List<(Edge[] Edges, ChamferSpec Spec, Face Reference)> lines, Func<int, bool> among)
    {
        EdgeFoot? found = null;
        double best = double.MaxValue;
        for (int i = 0; i < lines.Count; i++)
        {
            if (!among(i))
            {
                continue;
            }
            for (int j = 0; j < lines[i].Edges.Length; j++)
            {
                Edge edge = lines[i].Edges[j];
                if (!OnFace(edge, face) || Tool.Curve(edge, out double first, out double last) is not { } curve)
                {
                    continue;
                }
                var projection = new ProjectPointOnCurve(x, curve, first, last);
                if (projection.NbPoints() == 0 || projection.LowerDistance() >= best)
                {
                    continue;
                }
                best = projection.LowerDistance();
                double t = projection.LowerDistanceParameter();
                bool inside = t - first > 1e-3 * (last - first) && last - t > 1e-3 * (last - first);
                curve.D1(t, out _, out Vector3D tangent);
                found = inside ? new EdgeFoot(i, j, projection.NearestPoint(), tangent / tangent.Magnitude, Tool.Tolerance(edge)) : null;
            }
        }
        return found;
    }

    private static Vector3D NormalAt(Face face, Point3D x)
    {
        Surface surface = Tool.Surface(face)!;
        var projection = new ProjectPointOnSurface(x, surface);
        projection.LowerDistanceParameters(out double u, out double v);
        return Normal(surface, u, v);
    }

    // Every free edge of the result runs along a free edge of the input, or is a chamfer face's end between two of them
    private static bool Closed(Shape input, Face[] faces, Func<Face, bool> isInputPiece, double tolerance)
    {
        Edge[] inputFree = FreeEdges(input.AllFaces().ToArray(), out _);
        Edge[] free = FreeEdges(faces, out Face[] owners);
        for (int i = 0; i < free.Length; i++)
        {
            if (Tool.Curve(free[i], out double a, out double b) is not { } curve)
            {
                return false;
            }
            double allowed = tolerance + Tool.Tolerance(free[i]);
            if (new[] { 0.25, 0.5, 0.75 }.All(t => DistanceTo(curve.EvalD0(a + (t * (b - a))), inputFree) <= allowed))
            {
                continue;
            }
            if (!isInputPiece(owners[i]) && DistanceTo(curve.EvalD0(a), inputFree) <= allowed && DistanceTo(curve.EvalD0(b), inputFree) <= allowed)
            {
                continue;
            }
            return false;
        }
        return true;
    }

    // The edges of only one of the faces, once in it, with that face
    private static Edge[] FreeEdges(Face[] faces, out Face[] owners)
    {
        var free = new List<Edge>();
        var own = new List<Face>();
        var seen = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (Edge edge in faces.SelectMany(static f => f.AllEdges()))
        {
            if (!seen.Add(edge) || Tool.Degenerated(edge))
            {
                continue;
            }
            Face[] of = [.. faces.Where(f => f.AllEdges().Any(e => e.IsSame(edge)))];
            if (of.Length == 1 && of[0].Explore<Edge>(ShapeType.Edge).Count(e => e.IsSame(edge)) == 1)
            {
                free.Add(edge);
                own.Add(of[0]);
            }
        }
        owners = [.. own];
        return [.. free];
    }

    private static double DistanceTo(Point3D p, Edge[] edges)
    {
        double best = double.MaxValue;
        foreach (Edge edge in edges)
        {
            if (Tool.Curve(edge, out double a, out double b) is not { } curve)
            {
                continue;
            }
            var projection = new ProjectPointOnCurve(p, curve, a, b);
            if (projection.NbPoints() > 0)
            {
                best = Math.Min(best, projection.LowerDistance());
            }
            best = Math.Min(best, Math.Min(p.DistanceTo(curve.EvalD0(a)), p.DistanceTo(curve.EvalD0(b))));
        }
        return best;
    }
}
