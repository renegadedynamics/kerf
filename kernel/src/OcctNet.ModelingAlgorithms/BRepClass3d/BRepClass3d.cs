// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

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
        double aTol;
        Solid aSDx;
        Shell aShell = new(), aDummy = new();
        var aIt = new Iterator();
        var aBB = new BrepBuilder();
        var aSC = new BRepClass3d_SolidClassifier();
        //
        if (aSolid.IsNull)
        {
            return aDummy;
        }

        //
        aTol = 1.0e-7;
        bFound = false;
        //
        // if solid has one shell, it will return, without checking orientation
        int aShellCounter = 0;
        for (aIt.Initialize(aSolid); aIt.More(); aIt.Next())
        {
            Shape aSx = aIt.Value();
            if (aSx.ShapeType() == ShapeType.Shell)
            {
                aShell = ShapeCasts.Shell(aSx);
                aShellCounter++;
                if (aShellCounter >= 2)
                {
                    break;
                }
            }
        }

        if (aShellCounter == 0)
        {
            return aDummy;
        }
        else if (aShellCounter == 1)
        {
            return aShell;
        }

        //
        for (aIt.Initialize(aSolid); aIt.More(); aIt.Next())
        {
            Shape aSx = aIt.Value();
            if (aSx.ShapeType() == ShapeType.Shell)
            {
                aShell = ShapeCasts.Shell(aSx);
                if (!IsInternal(aShell))
                {
                    aSDx = aSolid;
                    aSDx.EmptyCopy();
                    aBB.Add(aSDx, aShell);
                    //
                    aSC.Load(aSDx);
                    aSC.PerformInfinitePoint(aTol);
                    if (aSC.State() == Out)
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
            return aDummy;
        }

        //
        return aShell;
    }

    private static bool IsInternal(in Shell aSx)
    {
        bool bInternal;
        Orientation aOr;
        var aIt = new Iterator();
        //
        bInternal = false;
        //
        aIt.Initialize(aSx);
        if (aIt.More())
        {
            Shape aSy = aIt.Value();
            aOr = aSy.Orientation();
            bInternal = (aOr == Internal);
        }

        //
        return bInternal;
    }
}
