// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>The outer shell of a solid.</summary>
public static class BRepClass3d
{
    /// <summary>
    /// The outer shell of <paramref name="aSolid"/>: its only shell without a check, otherwise the first shell (not
    /// INTERNAL) for which a point at infinity is OUT. A null shell if there is none.
    /// </summary>
    /// <remarks>
    /// The solids it classifies are built here, unfrozen, and never published; the shell returned is the solid's own.
    /// </remarks>
    public static Shell OuterShell(in Solid aSolid)
    {
        bool bFound;
        double tol;
        Solid sDx;
        Shell shell = new(), dummy = new();
        var it = new Iterator();
        var bb = new BrepBuilder();
        var sc = new SolidClassifier();
        //
        if (aSolid.IsNull)
        {
            return dummy;
        }

        //
        tol = 1.0e-7;
        bFound = false;
        //
        // if solid has one shell, it will return, without checking orientation
        int shellCounter = 0;
        for (it.Initialize(aSolid); it.More(); it.Next())
        {
            Shape sx = it.Value();
            if (sx.Type == ShapeType.Shell)
            {
                shell = ShapeCasts.Shell(sx);
                shellCounter++;
                if (shellCounter >= 2)
                {
                    break;
                }
            }
        }

        if (shellCounter == 0)
        {
            return dummy;
        }
        else if (shellCounter == 1)
        {
            return shell;
        }

        //
        for (it.Initialize(aSolid); it.More(); it.Next())
        {
            Shape sx = it.Value();
            if (sx.Type == ShapeType.Shell)
            {
                shell = ShapeCasts.Shell(sx);
                if (!IsInternal(shell))
                {
                    sDx = aSolid;
                    sDx.EmptyCopy();
                    bb.Add(sDx, shell);
                    //
                    sc.Load(sDx);
                    sc.PerformInfinitePoint(tol);
                    if (sc.State() == Out)
                    {
                        bFound = true;
                        break;
                    }
                }
            }
        }

        //
        if (!bFound)
        {
            return dummy;
        }

        //
        return shell;
    }

    private static bool IsInternal(in Shell aSx)
    {
        bool bInternal;
        Orientation or;
        var it = new Iterator();
        //
        bInternal = false;
        //
        it.Initialize(aSx);
        if (it.More())
        {
            Shape sy = it.Value();
            or = sy.Orientation;
            bInternal = (or == Internal);
        }

        //
        return bInternal;
    }
}
