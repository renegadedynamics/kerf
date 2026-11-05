// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShellSewing.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShellSewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Applies BRepBuilderAPI's sewing to each shell of a shape and replaces the sewn shells with
/// <see cref="ReShapeBuilder"/>; the rebuilt solids are oriented to be finite.
/// </summary>
/// <remarks>
/// A working object (decision 21). The shape given is not modified (decision 999): sewing works on a copy of frozen
/// faces (decision 583) and the reshaper builds new shapes. <see cref="ApplySewing"/> freezes what it returns.
/// </remarks>
internal class ShellSewing
{
    private readonly IndexedSet<Shape> _shells = new(ShapeHasher.Instance);
    private readonly ReShapeBuilder _reShape;

    /// <summary>Creates an empty ShellSewing.</summary>
    public ShellSewing()
    {
        _reShape = new ReShapeBuilder();
    }

    /// <summary>
    /// Builds a new shape from the shape given, sewing each of its shells; the rebuilt solids are oriented to be
    /// finite. With <paramref name="tol"/> 0 (the default) the tolerance is the mean of the shape's. Returns a null
    /// shape if no shell was sewn. Frozen.
    /// </summary>
    public Shape ApplySewing(in Shape shape, double tol = 0.0)
    {
        Shape result = InternalApplySewing(shape, tol);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary><see cref="ApplySewing"/> without freezing the result, for ported callers (ShapeProcess).</summary>
    internal Shape InternalApplySewing(in Shape shape, double tol = 0.0)
    {
        if (shape.IsNull)
        {
            return shape;
        }

        double t = tol;
        if (t <= 0.0)
        {
            var stu = new ShapeToleranceAnalysis();
            t = stu.Tolerance(shape, 0); // tolerance moyenne
        }

        Init(shape);
        if (Prepare(t) != 0)
        {
            return Apply(shape, t);
        }

        return new Shape();
    }

    private void Init(in Shape shape)
    {
        if (shape.IsNull)
        {
            return;
        }
        if (shape.Type == ShapeType.Shell)
        {
            _shells.Add(shape);
        }
        else
        {
            for (var exs = new Explorer(shape, ShapeType.Shell); exs.More(); exs.Next())
            {
                _shells.Add(exs.Current());
            }
        }
    }

    private int Prepare(double tol)
    {
        int nb = _shells.Count, ns = 0;
        for (int i = 0; i < nb; i++)
        {
            Shell sl = ShapeCasts.Shell(_shells[i]);
            var ss = new Sewing(tol);
            var exp = new Explorer(sl, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                ss.Add(exp.Current());
            }
            ss.Perform();
            Shape newsh = ss.InternalSewedShape();
            if (!newsh.IsNull)
            {
                _reShape.Replace(sl, newsh);
                ns++;
            }
        }
        return ns;
    }

    private Shape Apply(in Shape shape, double tol)
    {
        if (shape.IsNull || _shells.Count == 0)
        {
            return shape;
        }

        Shape res = _reShape.InternalApply(shape, ShapeType.Face, 2);

        //  A present orienter les solides correctement
        _reShape.Clear();
        int ns = 0;
        for (var exd = new Explorer(shape, ShapeType.Solid); exd.More(); exd.Next())
        {
            Solid sd = ShapeCasts.Solid(exd.Current());
            var bsc3d = new SolidClassifier(sd);
            bsc3d.PerformInfinitePoint(tol);
            if (bsc3d.State() == In)
            {
                _reShape.Replace(sd, sd.Reversed());
                ns++;
            }
        }

        // szv#4:S4163:12Mar99 optimized
        if (ns != 0)
        {
            res = _reShape.InternalApply(res, ShapeType.Shell, 2);
        }

        return res;
    }
}
