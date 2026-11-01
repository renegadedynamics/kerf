// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShellSewing.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShellSewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Applies BRepBuilderAPI's sewing to each shell of a shape and replaces the sewn shells with
/// <see cref="ShapeBuild_ReShape"/>; the rebuilt solids are oriented to be finite.
/// </summary>
/// <remarks>
/// A working object (decision 21). The shape given is not modified (decision 999): sewing works on a copy of frozen
/// faces (decision 583) and the reshaper builds new shapes. <see cref="ApplySewing"/> freezes what it returns.
/// </remarks>
public class ShapeUpgrade_ShellSewing
{
    private readonly NCollection_IndexedMap<TopoDS_Shape> myShells = new(TopTools_ShapeMapHasher.Instance);
    private readonly ShapeBuild_ReShape myReShape;

    /// <summary>Creates an empty ShellSewing.</summary>
    public ShapeUpgrade_ShellSewing()
    {
        myReShape = new ShapeBuild_ReShape();
    }

    /// <summary>
    /// Builds a new shape from the shape given, sewing each of its shells; the rebuilt solids are oriented to be
    /// finite. With <paramref name="tol"/> 0 (the default) the tolerance is the mean of the shape's. Returns a null
    /// shape if no shell was sewn. Frozen.
    /// </summary>
    public TopoDS_Shape ApplySewing(in TopoDS_Shape shape, double tol = 0.0)
    {
        TopoDS_Shape aResult = InternalApplySewing(shape, tol);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary><see cref="ApplySewing"/> without freezing the result, for ported callers (ShapeProcess).</summary>
    internal TopoDS_Shape InternalApplySewing(in TopoDS_Shape shape, double tol = 0.0)
    {
        if (shape.IsNull())
        {
            return shape;
        }

        double t = tol;
        if (t <= 0.0)
        {
            var stu = new ShapeAnalysis_ShapeTolerance();
            t = stu.Tolerance(shape, 0); // tolerance moyenne
        }

        Init(shape);
        if (Prepare(t) != 0)
        {
            return Apply(shape, t);
        }

        return new TopoDS_Shape();
    }

    private void Init(in TopoDS_Shape shape)
    {
        if (shape.IsNull())
        {
            return;
        }
        if (shape.ShapeType() == TopAbs_SHELL)
        {
            myShells.Add(shape);
        }
        else
        {
            for (var exs = new TopExp_Explorer(shape, TopAbs_SHELL); exs.More(); exs.Next())
            {
                myShells.Add(exs.Current());
            }
        }
    }

    private int Prepare(double tol)
    {
        int nb = myShells.Extent(), ns = 0;
        for (int i = 1; i <= nb; i++)
        {
            TopoDS_Shell sl = TopoDS.Shell(myShells.FindKey(i));
            var ss = new BRepBuilderAPI_Sewing(tol);
            var exp = new TopExp_Explorer(sl, TopAbs_FACE);
            for (; exp.More(); exp.Next())
            {
                ss.Add(exp.Current());
            }
            ss.Perform();
            TopoDS_Shape newsh = ss.InternalSewedShape();
            if (!newsh.IsNull())
            {
                myReShape.Replace(sl, newsh);
                ns++;
            }
        }
        return ns;
    }

    private TopoDS_Shape Apply(in TopoDS_Shape shape, double tol)
    {
        if (shape.IsNull() || myShells.Extent() == 0)
        {
            return shape;
        }

        TopoDS_Shape res = myReShape.InternalApply(shape, TopAbs_FACE, 2);

        //  A present orienter les solides correctement
        myReShape.Clear();
        int ns = 0;
        for (var exd = new TopExp_Explorer(shape, TopAbs_SOLID); exd.More(); exd.Next())
        {
            TopoDS_Solid sd = TopoDS.Solid(exd.Current());
            var bsc3d = new BRepClass3d_SolidClassifier(sd);
            bsc3d.PerformInfinitePoint(tol);
            if (bsc3d.State() == TopAbs_IN)
            {
                myReShape.Replace(sd, sd.Reversed());
                ns++;
            }
        }

        // szv#4:S4163:12Mar99 optimized
        if (ns != 0)
        {
            res = myReShape.InternalApply(res, TopAbs_SHELL, 2);
        }

        return res;
    }
}
