// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// TEMPORARY API stub of BOPTools_AlgoTools2D (unit 6B); replaced by the port in this unit before merging.
/// <summary>The 2D tools of the Boolean component.</summary>
public static class BOPTools_AlgoTools2D
{
    /// <summary>Stub.</summary>
    public static void BuildPCurveForEdgeOnFace(in TopoDS_Edge aE, in TopoDS_Face aF, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool EdgeTangent(in TopoDS_Edge anE, double aT, out gp_Vec Tau) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void PointOnSurface(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, out double U, out double V, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void CurveOnSurface(in TopoDS_Edge aE, in TopoDS_Face aF, out Geom2d_Curve? aC, out double aToler, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void CurveOnSurface(in TopoDS_Edge aE, in TopoDS_Face aF, out Geom2d_Curve? aC, out double aFirst, out double aLast, out double aToler, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool HasCurveOnSurface(in TopoDS_Edge aE, in TopoDS_Face aF, out Geom2d_Curve? aC, out double aFirst, out double aLast, out double aToler) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool HasCurveOnSurface(in TopoDS_Edge aE, in TopoDS_Face aF) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void AdjustPCurveOnFace(in TopoDS_Face theF, Geom_Curve theC3D, Geom2d_Curve theC2D, out Geom2d_Curve? theC2DA, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void AdjustPCurveOnFace(in TopoDS_Face theF, double theFirst, double theLast, Geom2d_Curve theC2D, out Geom2d_Curve? theC2DA, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void AdjustPCurveOnSurf(BRepAdaptor_Surface aF, double aT1, double aT2, Geom2d_Curve aC2D, out Geom2d_Curve? aC2DA) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static double IntermediatePoint(double aFirst, double aLast) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static double IntermediatePoint(in TopoDS_Edge anE) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void Make2D(in TopoDS_Edge aE, in TopoDS_Face aF, out Geom2d_Curve? aC, out double aFirst, out double aLast, out double aToler, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void MakePCurveOnFace(in TopoDS_Face aF, Geom_Curve C3D, out Geom2d_Curve? aC, out double aToler, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void MakePCurveOnFace(in TopoDS_Face aF, Geom_Curve C3D, double aT1, double aT2, out Geom2d_Curve? aC, out double aToler, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int AttachExistingPCurve(in TopoDS_Edge aEold, in TopoDS_Edge aEnew, in TopoDS_Face aF, IntTools_Context? aCtx) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void IsEdgeIsoline(in TopoDS_Edge theE, in TopoDS_Face theF, out bool isTheUIso, out bool isTheVIso) => throw new NotImplementedException();
}
