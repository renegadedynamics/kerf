// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PROJECT.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PROJECT.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>The legacy Booleans' projection functions (<c>FUN_tool_*</c>).</summary>
/// <remarks>
/// OCCT's free functions, as a static class named after the file. Only <c>FUN_tool_bounds</c> (for an edge) is ported,
/// for the face maker of the 2D offset (plan 2.11), and <c>FUN_tool_projPonS</c>, for
/// <see cref="ToolShapeTool.SurfacesSameOriented"/> (unit 7L); the other projections are left out.
/// </remarks>
internal static class Project
{
    /// <summary>The parameter range of the edge's curve.</summary>
    public static void FUN_tool_bounds(in Edge e, out double f, out double l)
    {
        EdgeCurveAdaptor bac = new(e);
        f = bac.FirstParameter();
        l = bac.LastParameter();
    }

    /// <summary>
    /// Projects <paramref name="p"/> on the surface <paramref name="s"/> within its bounds: the parameters
    /// <paramref name="uv"/> and distance <paramref name="dist"/> of the nearest point; false (both unchanged) if
    /// there is none.
    /// </summary>
    public static bool FUN_tool_projPonS(in Point3D p,
                                         Surface s,
                                         ref Point2D uv,
                                         ref double dist,
                                         ExtFlag anExtFlag = ExtFlag.ExtFlag_MINMAX,
                                         ExtAlgo anExtAlgo = ExtAlgo.ExtAlgo_Grad)
    {
        ProjectPointOnSurface ponS = new();
        //
        s.ParameterBounds(out double uMin, out double uMax, out double vMin, out double vMax);
        ponS.Init(s, uMin, uMax, vMin, vMax, anExtAlgo);
        ExtPS extPS = ponS.Extrema();
        extPS.SetFlag(anExtFlag);
        //
        ponS.Perform(p);
        //
        if (!ponS.Extrema().IsDone)
        {
            return false;
        }
        if (ponS.NbPoints() == 0)
        {
            return false;
        }
        dist = ponS.LowerDistance();
        ponS.LowerDistanceParameters(out double u, out double v);
        uv = uv.WithCoord(u, v);
        return true;
    }

    // FUN_tool_getindex, FUN_tool_projPonC, FUN_tool_projPonC2D, FUN_tool_projPonE,
    // FUN_tool_projPonboundedF, FUN_tool_projPonF: not ported (plan 2.11).
}
