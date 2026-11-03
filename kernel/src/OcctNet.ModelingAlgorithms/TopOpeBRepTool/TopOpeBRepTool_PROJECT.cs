// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PROJECT.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PROJECT.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The legacy Booleans' projection functions (<c>FUN_tool_*</c>).</summary>
/// <remarks>
/// OCCT's free functions, as a static class named after the file. Only <c>FUN_tool_bounds</c> (for an edge) is ported,
/// for the face maker of the 2D offset (plan 2.11), and <c>FUN_tool_projPonS</c>, for
/// <see cref="TopOpeBRepTool_ShapeTool.SurfacesSameOriented"/> (unit 7L); the other projections are left out.
/// </remarks>
public static class TopOpeBRepTool_PROJECT
{
    /// <summary>The parameter range of the edge's curve.</summary>
    public static void FUN_tool_bounds(in TopoDS_Edge E, out double f, out double l)
    {
        BRepAdaptor_Curve BAC = new(E);
        f = BAC.FirstParameter();
        l = BAC.LastParameter();
    }

    /// <summary>
    /// Projects <paramref name="P"/> on the surface <paramref name="S"/> within its bounds: the parameters
    /// <paramref name="UV"/> and distance <paramref name="dist"/> of the nearest point; false (both unchanged) if
    /// there is none.
    /// </summary>
    public static bool FUN_tool_projPonS(in Point3D P,
                                         Geom_Surface S,
                                         ref Point2D UV,
                                         ref double dist,
                                         Extrema_ExtFlag anExtFlag = Extrema_ExtFlag.Extrema_ExtFlag_MINMAX,
                                         Extrema_ExtAlgo anExtAlgo = Extrema_ExtAlgo.Extrema_ExtAlgo_Grad)
    {
        GeomAPI_ProjectPointOnSurf PonS = new();
        //
        S.Bounds(out double UMin, out double UMax, out double VMin, out double VMax);
        PonS.Init(S, UMin, UMax, VMin, VMax, anExtAlgo);
        Extrema_ExtPS anExtPS = PonS.Extrema();
        anExtPS.SetFlag(anExtFlag);
        //
        PonS.Perform(P);
        //
        if (!PonS.Extrema().IsDone())
        {
            return false;
        }
        if (PonS.NbPoints() == 0)
        {
            return false;
        }
        dist = PonS.LowerDistance();
        PonS.LowerDistanceParameters(out double U, out double V);
        UV.SetCoord(U, V);
        return true;
    }

    // FUN_tool_getindex, FUN_tool_projPonC, FUN_tool_projPonC2D, FUN_tool_projPonE,
    // FUN_tool_projPonboundedF, FUN_tool_projPonF: not ported (plan 2.11).
}
