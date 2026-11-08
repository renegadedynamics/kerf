// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_InfiniteHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

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
            // Kerf: OCCT opened the three minima, which misses a coordinate that grows as the parameter falls
            // (decision 1964)
            OpenAlong(dir.XYZ().Reversed(), ref box);
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
            // Kerf: OCCT opened the three maxima, which misses a coordinate that falls as the parameter grows
            // (decision 1964)
            OpenAlong(dir.XYZ(), ref box);
        }
    }

    /// <summary>
    /// Opens, on each axis the vector moves along, the side it moves towards (Kerf, decision 1964): what a point
    /// going to infinity along it reaches.
    /// </summary>
    internal static void OpenAlong(in Xyz v, ref Box box)
    {
        if (v.X() > 0.0) box.OpenXmax(); else if (v.X() < 0.0) box.OpenXmin();
        if (v.Y() > 0.0) box.OpenYmax(); else if (v.Y() < 0.0) box.OpenYmin();
        if (v.Z() > 0.0) box.OpenZmax(); else if (v.Z() < 0.0) box.OpenZmin();
    }

    /// <inheritdoc cref="OpenAlong(in Xyz, ref Box)"/>
    internal static void OpenAlong(in Xy v, ref Box2d box)
    {
        if (v.X() > 0.0) box.OpenXmax(); else if (v.X() < 0.0) box.OpenXmin();
        if (v.Y() > 0.0) box.OpenYmax(); else if (v.Y() < 0.0) box.OpenYmin();
    }

    /// <summary>
    /// For an arc with one end infinite (Kerf, decision 1964): its finite parameter and the sign of the infinite end.
    /// </summary>
    internal static bool HalfInfinite(double u1, double u2, out double finite, out int towards)
    {
        bool inf1 = global::Kerf.Engine.Foundation.Precision.IsInfinite(u1), inf2 = global::Kerf.Engine.Foundation.Precision.IsInfinite(u2);
        finite = inf1 ? u2 : u1;
        towards = (inf1 ? u1 : u2) < 0.0 ? -1 : 1;
        return inf1 != inf2;
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
            // Kerf: as the 3D form (decision 1964)
            OpenAlong(dir.XY().Reversed(), ref box);
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
            // Kerf: as the 3D form (decision 1964)
            OpenAlong(dir.XY(), ref box);
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
