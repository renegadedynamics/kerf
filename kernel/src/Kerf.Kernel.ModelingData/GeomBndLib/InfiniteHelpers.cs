// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_InfiniteHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Helpers that open a bounding box towards infinity along a direction, for unbounded parameters. OCCT's templates
/// over (gp_Dir, Bnd_Box) and (gp_Dir2d, Bnd_Box2d) are its two explicit specialisations, ported as overloads.
/// </summary>
internal static class InfiniteHelpers
{
    // ============================================================
    // 3D specializations (gp_Dir, Bnd_Box)
    // ============================================================

    /// <summary>
    /// Opens the box in the direction of decreasing parameter. For a line P(t) = Origin + t*V, as t -> -Inf: if
    /// V.coord > 0, that coordinate -> -Inf (open min); if V.coord &lt; 0, it -> +Inf (open max).
    /// </summary>
    internal static void OpenMin(in Direction3D dir, ref Box box)
    {
        if (dir.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.X > 0.0)
                box.OpenXmin();
            else
                box.OpenXmax();
        }
        else if (dir.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.Y > 0.0)
                box.OpenYmin();
            else
                box.OpenYmax();
        }
        else if (dir.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.Z > 0.0)
                box.OpenZmin();
            else
                box.OpenZmax();
        }
        else
        {
            box.OpenXmin();
            box.OpenYmin();
            box.OpenZmin();
        }
    }

    /// <summary>Opens the box in the direction of increasing parameter.</summary>
    internal static void OpenMax(in Direction3D dir, ref Box box)
    {
        if (dir.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.X > 0.0)
                box.OpenXmax();
            else
                box.OpenXmin();
        }
        else if (dir.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.Y > 0.0)
                box.OpenYmax();
            else
                box.OpenYmin();
        }
        else if (dir.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.Z > 0.0)
                box.OpenZmax();
            else
                box.OpenZmin();
        }
        else
        {
            box.OpenXmax();
            box.OpenYmax();
            box.OpenZmax();
        }
    }

    /// <summary>Opens the box in both directions along the given direction.</summary>
    internal static void OpenMinMax(in Direction3D dir, ref Box box)
    {
        if (dir.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            box.OpenXmax();
            box.OpenXmin();
        }
        else if (dir.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            box.OpenYmax();
            box.OpenYmin();
        }
        else if (dir.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            box.OpenZmax();
            box.OpenZmin();
        }
        else
        {
            box.OpenXmin();
            box.OpenYmin();
            box.OpenZmin();
            box.OpenXmax();
            box.OpenYmax();
            box.OpenZmax();
        }
    }

    // ============================================================
    // 2D specializations (gp_Dir2d, Bnd_Box2d)
    // ============================================================

    internal static void OpenMin(in Direction2D dir, ref Box2d box)
    {
        if (dir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.X > 0.0)
                box.OpenXmin();
            else
                box.OpenXmax();
        }
        else if (dir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.Y > 0.0)
                box.OpenYmin();
            else
                box.OpenYmax();
        }
        else
        {
            box.OpenXmin();
            box.OpenYmin();
        }
    }

    internal static void OpenMax(in Direction2D dir, ref Box2d box)
    {
        if (dir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.X > 0.0)
                box.OpenXmax();
            else
                box.OpenXmin();
        }
        else if (dir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (dir.Y > 0.0)
                box.OpenYmax();
            else
                box.OpenYmin();
        }
        else
        {
            box.OpenXmax();
            box.OpenYmax();
        }
    }

    internal static void OpenMinMax(in Direction2D dir, ref Box2d box)
    {
        if (dir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            box.OpenXmax();
            box.OpenXmin();
        }
        else if (dir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            box.OpenYmax();
            box.OpenYmin();
        }
        else
        {
            box.OpenXmin();
            box.OpenYmin();
            box.OpenXmax();
            box.OpenYmax();
        }
    }
}
