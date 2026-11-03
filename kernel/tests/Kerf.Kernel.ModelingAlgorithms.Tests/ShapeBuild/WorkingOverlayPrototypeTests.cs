// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 10H1's two prototypes of the working overlay on one real algorithm, ShapeFix_ShapeTolerance's
/// <c>LimitTolerance</c> (decision 15602). <b>A</b>, kept here as the record: an overlay-aware tool and builder pair
/// that the algorithm calls explicitly, so every read and write it makes, and those of every helper it calls, must be
/// rewritten to take the pair. <b>B</b>, chosen: the context the builder, the TShapes' accessors and the iterator
/// consult (<see cref="WorkingOverlay"/>), so the algorithm runs unchanged between <c>Begin</c> and <c>Finish</c>.
/// Both give the same result.
/// </summary>
public class WorkingOverlayPrototypeTests
{
    /// <summary>Prototype A: the explicit pair. Reads and writes go through it; the result is assembled as B's.</summary>
    private sealed class ExplicitPair
    {
        private readonly Dictionary<TShape, TShape> _working = new(ReferenceEqualityComparer.Instance);
        private readonly Dictionary<TShape, TShape> _final = new(ReferenceEqualityComparer.Instance);

        private TShape Read(TShape t) => _working.TryGetValue(t, out TShape? w) ? w : t;

        private TShape Write(TShape t)
        {
            if (!t.IsFrozen)
            {
                return t;
            }
            if (!_working.TryGetValue(t, out TShape? w))
            {
                w = BrepTools.CopyData(t);
                for (var it = new Iterator(new Shape(t, new Placement(), Orientation.Forward), false, false); it.More(); it.Next())
                {
                    w._shapes.Append(it.Value());
                }
                BrepTools.CopyFlags(t, w);
                _working[t] = w;
            }
            return w;
        }

        // the "tool": BRep_Tool's reads, through the pair
        public double Tolerance(in Vertex v) => ((BrepTVertex)Read(v.TShape()!)).Tolerance();
        public double Tolerance(in Edge e) => ((BrepTEdge)Read(e.TShape()!)).Tolerance();
        public double Tolerance(in Face f) => ((BrepTFace)Read(f.TShape()!)).Tolerance();

        // the "builder": the writes, into the working copies
        public void SetTolerance(in Vertex v, double t) => ((BrepTVertex)Write(v.TShape()!)).Tolerance(t);
        public void SetTolerance(in Edge e, double t) => ((BrepTEdge)Write(e.TShape()!)).Tolerance(t);
        public void SetTolerance(in Face f, double t) => ((BrepTFace)Write(f.TShape()!)).Tolerance(t);

        public Shape Finish(in Shape shape)
        {
            Shape result = shape;
            result.TShape(Assemble(shape.TShape()!));
            return result;
        }

        private TShape Assemble(TShape t)
        {
            if (_final.TryGetValue(t, out TShape? done))
            {
                return done;
            }
            _working.TryGetValue(t, out TShape? w);
            var children = new List<Shape>();
            bool isChanged = w is not null && !BrepTools.IsSameData(t, w, false);
            for (var it = new Iterator(new Shape(t, new Placement(), Orientation.Forward), false, false); it.More(); it.Next())
            {
                Shape child = it.Value();
                TShape now = Assemble(child.TShape()!);
                isChanged |= !ReferenceEquals(now, child.TShape());
                child.TShape(now);
                children.Add(child);
            }
            TShape result = t;
            if (isChanged)
            {
                result = w ?? BrepTools.CopyData(t);
                if (w is null)
                {
                    BrepTools.CopyFlags(t, result);
                }
                result._shapes.Clear();
                foreach (Shape child in children)
                {
                    result._shapes.Append(child);
                }
            }
            _final[t] = result;
            return result;
        }
    }

    // LimitTolerance (styp = Shape) rewritten for prototype A: every Tool read and TShape write of OCCT's code changed.
    private static bool LimitToleranceA(ExplicitPair pair, in Shape shape, double tmin, double tmax)
    {
        bool fait = false;
        for (var ex = new Explorer(shape, ShapeType.Vertex); ex.More(); ex.Next())
        {
            Vertex v = ShapeCasts.Vertex(ex.Current());
            double prec = pair.Tolerance(v);
            if (prec > tmax || prec < tmin)
            {
                pair.SetTolerance(v, prec > tmax ? tmax : tmin);
                fait = true;
            }
        }
        for (var ex = new Explorer(shape, ShapeType.Edge); ex.More(); ex.Next())
        {
            Edge e = ShapeCasts.Edge(ex.Current());
            double prec = pair.Tolerance(e);
            if (prec > tmax || prec < tmin)
            {
                pair.SetTolerance(e, prec > tmax ? tmax : tmin);
                fait = true;
            }
        }
        for (var ex = new Explorer(shape, ShapeType.Face); ex.More(); ex.Next())
        {
            Face f = ShapeCasts.Face(ex.Current());
            double prec = pair.Tolerance(f);
            if (prec > tmax || prec < tmin)
            {
                pair.SetTolerance(f, prec > tmax ? tmax : tmin);
                fait = true;
            }
        }
        return fait;
    }

    [Fact]
    public void Prototypes_A_and_B_give_the_same_result_on_LimitTolerance()
    {
        Shape box = new MakeBox(10, 20, 30).Shape();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);

        var pair = new ExplicitPair();
        Assert.True(LimitToleranceA(pair, box, 1e-3, 1e-2));
        Shape resultA = pair.Finish(box);

        Shape resultB = new ShapeToleranceFixer().ToleranceLimited(box, 1e-3, out bool isDone, 1e-2);
        Assert.True(isDone);

        snapshot.AssertUnchanged(box);
        ShapeSnapshot.Of(resultA).AssertSameValues(ShapeSnapshot.Of(resultB));
        // the same sub-shapes of the input kept: here none (every vertex, edge and face is limited)
        var exA = new List<TShape>();
        var exB = new List<TShape>();
        for (var ex = new Explorer(resultA, ShapeType.Vertex); ex.More(); ex.Next())
        {
            exA.Add(ex.Current().TShape()!);
        }
        for (var ex = new Explorer(resultB, ShapeType.Vertex); ex.More(); ex.Next())
        {
            exB.Add(ex.Current().TShape()!);
        }
        Assert.Equal(exA.Count, exB.Count);
        Assert.False(ReferenceEquals(resultA.TShape(), box.TShape()));
    }
}
