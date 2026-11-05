// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10L1: what the typed shapes add to Shape's surface (docs/api/topology.md): the raw assembly factories (the
// modelling layer's UnsafeAccessors' replacement) and the vertices', edges', wires' and faces' own data, read through
// BRep_Tool, TopExp and BRepTools.

using System.Collections.Immutable;
using System.Diagnostics.CodeAnalysis;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;

namespace Kerf.Topology;

public partial struct Compound
{
    /// <summary>A compound of <paramref name="shapes"/>, as they are: raw assembly, nothing is checked.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="shapes"/> is null.</exception>
    /// <exception cref="ArgumentException">A shape is null.</exception>
    public static Compound Create(IEnumerable<Shape> shapes) => ShapeChecks.Make(shapes, Builder.MakeCompound);
}

public partial struct Wire
{
    /// <summary>A wire of <paramref name="edges"/>, in the order given: raw assembly, nothing is checked or connected
    /// (the construction area's wire builder connects).</summary>
    /// <exception cref="ArgumentNullException"><paramref name="edges"/> is null.</exception>
    /// <exception cref="ArgumentException">An edge is null.</exception>
    public static Wire Create(IEnumerable<Edge> edges) => ShapeChecks.Make(edges, Builder.MakeWire);

    /// <summary>The edges in the order they connect, each oriented along the wire (OCCT's <c>BRepTools_WireExplorer</c>).</summary>
    public readonly ImmutableArray<Edge> OrderedEdges()
    {
        if (IsNull)
        {
            return [];
        }

        var edges = ImmutableArray.CreateBuilder<Edge>();
        for (var explorer = new WireExplorer(this); explorer.More(); explorer.Next())
        {
            edges.Add(explorer.Current());
        }

        return edges.ToImmutable();
    }

    /// <summary>The wire's curve, its edges joined end to end, for evaluation and lengths.</summary>
    /// <exception cref="InvalidOperationException">The wire is null.</exception>
    public readonly WireCurve ToCurve() => new(ShapeChecks.NotNull(this));
}

public partial struct Shell
{
    /// <summary>A shell of <paramref name="faces"/>, as they are: raw assembly, nothing is checked or sewn, and the
    /// shell isn't marked closed (the healing area sews).</summary>
    /// <exception cref="ArgumentNullException"><paramref name="faces"/> is null.</exception>
    /// <exception cref="ArgumentException">A face is null.</exception>
    public static Shell Create(IEnumerable<Face> faces) => ShapeChecks.Make(faces, Builder.MakeShell);
}

public partial struct Solid
{
    /// <summary>A solid of <paramref name="shells"/>, as they are oriented: raw assembly, nothing is checked.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="shells"/> is null.</exception>
    /// <exception cref="ArgumentException">A shell is null.</exception>
    public static Solid Create(IEnumerable<Shell> shells) => ShapeChecks.Make(shells, Builder.MakeSolid);
}

public partial struct CompSolid
{
    /// <summary>A composite solid of <paramref name="solids"/>: raw assembly, nothing is checked.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="solids"/> is null.</exception>
    /// <exception cref="ArgumentException">A solid is null.</exception>
    public static CompSolid Create(IEnumerable<Solid> solids) => ShapeChecks.Make(solids, Builder.MakeCompSolid);
}

public partial struct Vertex
{
    /// <summary>The vertex's point, placed.</summary>
    /// <exception cref="InvalidOperationException">The vertex is null.</exception>
    public readonly Point3D Point => Tool.Pnt(ShapeChecks.NotNull(this));

    /// <summary>The vertex's tolerance: the radius of the ball its edges' ends lie in.</summary>
    /// <exception cref="InvalidOperationException">The vertex is null.</exception>
    public readonly double Tolerance => Tool.Tolerance(ShapeChecks.NotNull(this));

    /// <summary>The vertex's (u, v) on <paramref name="face"/>: from the curve on the face of an edge of the face that
    /// holds it, or else by projecting it onto the surface (OCCT's <c>BRep_Tool::Parameters</c>).</summary>
    /// <exception cref="InvalidOperationException">The vertex or the face is null.</exception>
    public readonly Point2D ParametersOn(Face face) => Tool.Parameters(ShapeChecks.NotNull(this), ShapeChecks.NotNull(face));
}

public partial struct Edge
{
    /// <summary>The edge's parameter range on its curve.</summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly Interval Range
    {
        get
        {
            Tool.Range(ShapeChecks.NotNull(this), out double first, out double last);
            return new Interval(first, last);
        }
    }

    /// <summary>The edge's tolerance: the radius of the tube its curves lie in.</summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly double Tolerance => Tool.Tolerance(ShapeChecks.NotNull(this));

    /// <summary>True if the edge is degenerate: a point (a cone's apex, a sphere's pole) with a curve only on its faces.</summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly bool IsDegenerated => Tool.Degenerated(ShapeChecks.NotNull(this));

    /// <summary>The edge's first vertex along its orientation (a reversed edge starts at its curve's last vertex).</summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly Vertex FirstVertex => Exploration.FirstVertex(ShapeChecks.NotNull(this), true);

    /// <summary>The edge's last vertex along its orientation.</summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly Vertex LastVertex => Exploration.LastVertex(ShapeChecks.NotNull(this), true);

    /// <summary>
    /// The edge's 3D curve in its own coordinates, unplaced, its range on it, and the placement that puts it in place (OCCT's
    /// <c>BRep_Tool::Curve</c> with the location): <see cref="TryGetCurve"/> transforms the curve by it.
    /// </summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly bool TryGetLocalCurve([NotNullWhen(true)] out Curve3D? curve, out Interval range, out Placement placement)
    {
        curve = Tool.Curve(ShapeChecks.NotNull(this), out placement, out double first, out double last);
        range = new Interval(first, last);
        return curve is not null;
    }

    /// <summary>The edge's 3D curve, placed, and its range on it; false if the edge has none (a degenerate edge).</summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly bool TryGetCurve([NotNullWhen(true)] out Curve3D? curve, out Interval range)
    {
        curve = Tool.Curve(ShapeChecks.NotNull(this), out double first, out double last);
        range = new Interval(first, last);
        return curve is not null;
    }

    /// <summary>The edge's curve in the parameter space of <paramref name="face"/>, and its range; false if it has none
    /// there.</summary>
    /// <exception cref="InvalidOperationException">The edge or the face is null.</exception>
    public readonly bool TryGetCurveOnFace(Face face, [NotNullWhen(true)] out Curve2D? curve, out Interval range)
    {
        curve = Tool.CurveOnSurface(ShapeChecks.NotNull(this), ShapeChecks.NotNull(face), out double first, out double last);
        range = new Interval(first, last);
        return curve is not null;
    }

    /// <summary>True if the edge is a seam of <paramref name="face"/>: it bounds the face twice (a cylinder's seam).</summary>
    /// <exception cref="InvalidOperationException">The edge or the face is null.</exception>
    public readonly bool IsSeam(Face face) => Tool.IsClosed(ShapeChecks.NotNull(this), ShapeChecks.NotNull(face));

    /// <summary>
    /// The edge's curve in the parameters of <paramref name="plane"/>: its curve on a face of that plane if it has one, or
    /// else its 3D curve projected onto the plane (OCCT's <c>BRep_Tool::CurveOnPlane</c>), with its range. False if it
    /// has neither.
    /// </summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly bool TryGetCurveOnPlane(in Plane3D plane, [NotNullWhen(true)] out Curve2D? curve, out Interval range)
    {
        curve = Tool.CurveOnPlane(ShapeChecks.NotNull(this), new PlaneSurface(plane), new Placement(), out double first, out double last);
        range = new Interval(first, last);
        return curve is not null;
    }

    /// <summary>The parameter of <paramref name="vertex"/> on the edge's curve.</summary>
    /// <exception cref="InvalidOperationException">The edge or the vertex is null.</exception>
    /// <exception cref="ArgumentException">The vertex isn't one of the edge's.</exception>
    public readonly double ParameterOf(Vertex vertex)
        => Tool.Parameter(ShapeChecks.NotNull(vertex), ShapeChecks.NotNull(this), out double parameter)
            ? parameter
            : throw new ArgumentException("The vertex is not on the edge.", nameof(vertex));

    /// <summary>An evaluator of the edge's curve in world coordinates.</summary>
    /// <exception cref="InvalidOperationException">The edge is null.</exception>
    public readonly EdgeCurve ToCurve() => new(ShapeChecks.NotNull(this));
}

public partial struct Face
{
    /// <summary>The face's tolerance.</summary>
    /// <exception cref="InvalidOperationException">The face is null.</exception>
    public readonly double Tolerance => Tool.Tolerance(ShapeChecks.NotNull(this));

    /// <summary>The face's surface, placed; false for a face that is only a mesh (read from STL).</summary>
    /// <exception cref="InvalidOperationException">The face is null.</exception>
    public readonly bool TryGetSurface([NotNullWhen(true)] out Surface? surface)
    {
        surface = Tool.Surface(ShapeChecks.NotNull(this));
        return surface is not null;
    }

    /// <summary>The face's outer wire: its boundary, as opposed to the holes' wires; a null wire if it has none.</summary>
    /// <exception cref="InvalidOperationException">The face is null.</exception>
    public readonly Wire OuterWire => BrepTools.OuterWire(ShapeChecks.NotNull(this));

    /// <summary>The bounds of the face's parameters, from its boundary's curves on its surface.</summary>
    /// <exception cref="InvalidOperationException">The face is null.</exception>
    public readonly (Interval U, Interval V) UVBounds
    {
        get
        {
            BrepTools.UVBounds(ShapeChecks.NotNull(this), out double uMin, out double uMax, out double vMin, out double vMax);
            return (new Interval(uMin, uMax), new Interval(vMin, vMax));
        }
    }

    /// <summary>An evaluator of the face's surface in world coordinates, with the face's orientation.</summary>
    /// <exception cref="InvalidOperationException">The face is null.</exception>
    public readonly FaceSurface ToSurface() => new(ShapeChecks.NotNull(this));
}

/// <summary>The factories' and accessors' shared checks.</summary>
internal static class ShapeChecks
{
    public delegate TResult Maker<TResult>(IEnumerable<Shape> shapes);

    public static TResult Make<TPart, TResult>(IEnumerable<TPart> parts, Maker<TResult> make) where TPart : struct, IShape<TPart>
    {
        ArgumentNullException.ThrowIfNull(parts);
        var shapes = new List<Shape>();
        foreach (TPart part in parts)
        {
            Shape shape = part.AsShape();
            if (shape.IsNull)
            {
                throw new ArgumentException("A shape to assemble is null.", nameof(parts));
            }

            shapes.Add(shape);
        }

        try
        {
            return make(shapes);
        }
        catch (Exception e) when (Refusal.IsFailure(e) && (e is IncompatibleShapesException))
        {
            throw new ArgumentException(FailureDiagnostics.PublicText(e), nameof(parts), e);
        }
    }

    public static TShape NotNull<TShape>(in TShape shape) where TShape : struct, IShape<TShape>
        => shape.AsShape().IsNull ? throw new InvalidOperationException($"The {TShape.Type.ToString().ToLowerInvariant()} is null.") : shape;
}
