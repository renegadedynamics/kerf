// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Foundation;
using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Geometry;

namespace Kerf.Topology;

/// <summary>
/// Traversal and raw assembly, for all nine shape types (C# 14 extension members, so <c>solid.AllFaces()</c> and
/// <c>shape.AllFaces()</c> both work): OCCT's <c>TopoDS_Iterator</c>, <c>TopExp_Explorer</c>, <c>TopExp::MapShapes</c> and
/// <c>TopoDS_Builder</c>.
/// </summary>
public static class ShapeTraversal
{
    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>
        /// Whether the shape is closed, as a reader means it, looked for in its sub-shapes: an edge whose two vertices are the
        /// same, a wire with no free vertex, a face or shell with no free edge, and a solid or compound of which every child is
        /// closed. A vertex isn't. Internal and external sub-shapes are ignored.
        /// </summary>
        /// <exception cref="InvalidOperationException">The shape is null.</exception>
        public bool IsClosed
        {
            get
            {
                Shape shape = self.AsShape();
                return shape.IsNull ? throw new InvalidOperationException("Can't determine if an empty shape is closed") : ClosedInFact(shape);
            }
        }
    }

    private static bool ClosedInFact(Shape shape)
    {
        switch (shape.Type)
        {
            case ShapeType.Vertex:
                return false;
            case ShapeType.Edge:
            case ShapeType.Wire:
            case ShapeType.Shell:
                return Kerf.Engine.Topology.Brep.Tool.IsClosed(shape);
            case ShapeType.Face:
                // A one-face shell: a sphere's seam meets itself, a rectangle's edges are free
                return Kerf.Engine.Topology.Brep.Tool.IsClosedAsFace(shape);
            default:
                bool any = false;
                foreach (Shape child in shape.Children)
                {
                    if (!ClosedInFact(child))
                    {
                        return false;
                    }

                    any = true;
                }

                return any;
        }
    }

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>The direct sub-shapes, with the shape's placement and orientation composed into theirs.</summary>
        public SubShapes<Shape> Children => new(shape.AsShape(), ShapeType.Shape, ShapeType.Shape, children: true);

        /// <summary>The distinct vertices (<see cref="Shape.IsSame"/>), in the order they are first explored.</summary>
        public ImmutableArray<Vertex> AllVertices() => Unique<Vertex>(shape.AsShape());

        /// <summary>The distinct edges (<see cref="Shape.IsSame"/>), in the order they are first explored.</summary>
        public ImmutableArray<Edge> AllEdges() => Unique<Edge>(shape.AsShape());

        /// <summary>The distinct wires (<see cref="Shape.IsSame"/>), in the order they are first explored.</summary>
        public ImmutableArray<Wire> AllWires() => Unique<Wire>(shape.AsShape());

        /// <summary>The distinct faces (<see cref="Shape.IsSame"/>), in the order they are first explored.</summary>
        public ImmutableArray<Face> AllFaces() => Unique<Face>(shape.AsShape());

        /// <summary>The distinct shells (<see cref="Shape.IsSame"/>), in the order they are first explored.</summary>
        public ImmutableArray<Shell> AllShells() => Unique<Shell>(shape.AsShape());

        /// <summary>The distinct solids (<see cref="Shape.IsSame"/>), in the order they are first explored.</summary>
        public ImmutableArray<Solid> AllSolids() => Unique<Solid>(shape.AsShape());

        /// <summary>The leaves of a compound, nested compounds flattened; any other shape is its own only leaf.</summary>
        public ImmutableArray<Shape> TopLevelShapes()
        {
            var leaves = ImmutableArray.CreateBuilder<Shape>();
            AddLeaves(shape.AsShape(), leaves);
            return leaves.ToImmutable();
        }

        /// <summary>The shape moved by <paramref name="transform"/>: the same data with the transform composed into
        /// its placement.</summary>
        /// <exception cref="ArgumentException">The transformation scales or mirrors.</exception>
        public Shape Moved(in Transform3D transform)
        {
            if (Math.Abs(Math.Abs(transform.ScaleFactor) - 1.0) > Placement.ScalePrec() || transform.IsNegative)
            {
                throw new ArgumentException("A shape can only be moved by a rigid transformation: no scale, no mirror.", nameof(transform));
            }

            return shape.AsShape().Moved(new Placement(transform));
        }

        /// <summary>
        /// A new shape of the same type, placement and orientation, with the geometry of this one (a face's surface, an
        /// edge's curve) and <paramref name="children"/> as its sub-shapes. Raw assembly: nothing is checked or sewn.
        /// </summary>
        /// <exception cref="ArgumentNullException"><paramref name="children"/> is null.</exception>
        /// <exception cref="ArgumentException">The shape or a child is null, or a child can't be a sub-shape of it.</exception>
        public Shape WithChildren(IEnumerable<Shape> children)
        {
            ArgumentNullException.ThrowIfNull(children);
            Shape self = shape.AsShape();
            NotNull(self);
            return Rebuilt(self, children);
        }

        /// <summary>
        /// A new shape like this one without the direct sub-shape <paramref name="child"/> (as <c>Children</c> gives
        /// it, compared with <see cref="Shape.IsEqual"/>). Raw assembly: nothing is checked.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null, or <paramref name="child"/> isn't one of its children.</exception>
        public Shape WithoutChild(Shape child)
        {
            Shape self = shape.AsShape();
            NotNull(self);
            var kept = new List<Shape>(self.ChildCount);
            bool found = false;
            foreach (Shape existing in self.Children)
            {
                if (!found && existing.IsEqual(child))
                {
                    found = true;
                }
                else
                {
                    kept.Add(existing);
                }
            }

            return found ? Rebuilt(self, kept) : throw new ArgumentException("The shape has no such child.", nameof(child));
        }
    }

    // Shape's and the typed shapes' Explore<T>: an instance method of each, since a generic extension method would need
    // the receiver's type spelt out beside T (shape.Explore<Solid, Face>())
    internal static SubShapes<T> Explore<T>(in Shape shape, ShapeType? avoid) where T : struct, IShape<T>
    {
        if (T.Type == ShapeType.Shape)
        {
            throw new ArgumentException("Explore needs a shape type: Vertex, Edge, Wire, Face, Shell, Solid, CompSolid or Compound.");
        }

        return new(shape, T.Type, avoid ?? ShapeType.Shape, children: false);
    }

    private static ImmutableArray<T> Unique<T>(in Shape shape) where T : struct, IShape<T>
    {
        if (shape.IsNull)
        {
            return [];
        }

        var found = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, T.Type, found);
        var result = ImmutableArray.CreateBuilder<T>(found.Count);
        foreach (Shape sub in found)
        {
            result.Add(ShapeConversions.Narrow<T>(sub));
        }

        return result.MoveToImmutable();
    }

    private static void AddLeaves(in Shape shape, ImmutableArray<Shape>.Builder leaves)
    {
        if (shape.IsNull)
        {
            return;
        }

        if (shape.Type != ShapeType.Compound)
        {
            leaves.Add(shape);
            return;
        }

        foreach (Shape child in new SubShapes<Shape>(shape, ShapeType.Shape, ShapeType.Shape, children: true))
        {
            AddLeaves(child, leaves);
        }
    }

    private static void NotNull(in Shape shape)
    {
        if (shape.IsNull)
        {
            throw new ArgumentException("The shape is null.", nameof(shape));
        }
    }

    // A frozen copy of the shape's data (its geometry, no sub-shapes) holding the children
    private static Shape Rebuilt(in Shape shape, IEnumerable<Shape> children)
    {
        Shape result = shape.EmptyCopied();
        var builder = new Builder();
        try
        {
            foreach (Shape child in children)
            {
                if (child.IsNull)
                {
                    throw new ArgumentException("A child is null.", nameof(children));
                }

                builder.Add(result, child);
            }
        }
        catch (Exception e) when (Refusal.IsFailure(e) && (e is DomainErrorException))
        {
            throw new ArgumentException($"A child can't be a sub-shape of a {shape.Type}: {FailureDiagnostics.PublicText(e)}", nameof(children), e);
        }

        result.TShape()!.FreezeGraph();
        return result;
    }
}
