// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_InfiniteHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Helpers that open a bounding box towards infinity along a direction, for unbounded parameters. OCCT's templates
/// over (gp_Dir, Bnd_Box) and (gp_Dir2d, Bnd_Box2d) are its two explicit specialisations, ported as overloads.
/// </summary>
internal static class GeomBndLib_InfiniteHelpers
{
    // ============================================================
    // 3D specializations (gp_Dir, Bnd_Box)
    // ============================================================

    /// <summary>
    /// Opens the box in the direction of decreasing parameter. For a line P(t) = Origin + t*V, as t -> -Inf: if
    /// V.coord > 0, that coordinate -> -Inf (open min); if V.coord &lt; 0, it -> +Inf (open max).
    /// </summary>
    internal static void OpenMin(in Direction3D theDir, ref Box theBox)
    {
        if (theDir.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.X() > 0.0)
                theBox.OpenXmin();
            else
                theBox.OpenXmax();
        }
        else if (theDir.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.Y() > 0.0)
                theBox.OpenYmin();
            else
                theBox.OpenYmax();
        }
        else if (theDir.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.Z() > 0.0)
                theBox.OpenZmin();
            else
                theBox.OpenZmax();
        }
        else
        {
            theBox.OpenXmin();
            theBox.OpenYmin();
            theBox.OpenZmin();
        }
    }

    /// <summary>Opens the box in the direction of increasing parameter.</summary>
    internal static void OpenMax(in Direction3D theDir, ref Box theBox)
    {
        if (theDir.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.X() > 0.0)
                theBox.OpenXmax();
            else
                theBox.OpenXmin();
        }
        else if (theDir.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.Y() > 0.0)
                theBox.OpenYmax();
            else
                theBox.OpenYmin();
        }
        else if (theDir.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.Z() > 0.0)
                theBox.OpenZmax();
            else
                theBox.OpenZmin();
        }
        else
        {
            theBox.OpenXmax();
            theBox.OpenYmax();
            theBox.OpenZmax();
        }
    }

    /// <summary>Opens the box in both directions along the given direction.</summary>
    internal static void OpenMinMax(in Direction3D theDir, ref Box theBox)
    {
        if (theDir.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            theBox.OpenXmax();
            theBox.OpenXmin();
        }
        else if (theDir.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            theBox.OpenYmax();
            theBox.OpenYmin();
        }
        else if (theDir.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            theBox.OpenZmax();
            theBox.OpenZmin();
        }
        else
        {
            theBox.OpenXmin();
            theBox.OpenYmin();
            theBox.OpenZmin();
            theBox.OpenXmax();
            theBox.OpenYmax();
            theBox.OpenZmax();
        }
    }

    // ============================================================
    // 2D specializations (gp_Dir2d, Bnd_Box2d)
    // ============================================================

    internal static void OpenMin(in Direction2D theDir, ref Box2d theBox)
    {
        if (theDir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.X() > 0.0)
                theBox.OpenXmin();
            else
                theBox.OpenXmax();
        }
        else if (theDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.Y() > 0.0)
                theBox.OpenYmin();
            else
                theBox.OpenYmax();
        }
        else
        {
            theBox.OpenXmin();
            theBox.OpenYmin();
        }
    }

    internal static void OpenMax(in Direction2D theDir, ref Box2d theBox)
    {
        if (theDir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.X() > 0.0)
                theBox.OpenXmax();
            else
                theBox.OpenXmin();
        }
        else if (theDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (theDir.Y() > 0.0)
                theBox.OpenYmax();
            else
                theBox.OpenYmin();
        }
        else
        {
            theBox.OpenXmax();
            theBox.OpenYmax();
        }
    }

    internal static void OpenMinMax(in Direction2D theDir, ref Box2d theBox)
    {
        if (theDir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            theBox.OpenXmax();
            theBox.OpenXmin();
        }
        else if (theDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            theBox.OpenYmax();
            theBox.OpenYmin();
        }
        else
        {
            theBox.OpenXmin();
            theBox.OpenYmin();
            theBox.OpenXmax();
            theBox.OpenYmax();
        }
    }
}
