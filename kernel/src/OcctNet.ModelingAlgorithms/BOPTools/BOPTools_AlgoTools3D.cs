// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// TEMPORARY API stub of BOPTools_AlgoTools3D (unit 6B); replaced by the port in this unit before merging.
/// <summary>The 3D tools of the Boolean component.</summary>
public static class BOPTools_AlgoTools3D
{
    /// <summary>Stub.</summary>
    public static bool DoSplitSEAMOnFace(in TopoDS_Edge theESplit, in TopoDS_Face theFace) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool DoSplitSEAMOnFace(in TopoDS_Edge theEOrigin, in TopoDS_Edge theESplit, in TopoDS_Face theFace) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void GetNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, out gp_Dir aD, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void GetNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF, out gp_Dir aD, IntTools_Context? theContext = null) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int SenseFlag(in gp_Dir aNF1, in gp_Dir aNF2) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool GetNormalToSurface(Geom_Surface aS, double U, double V, ref gp_Dir aD) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, out gp_Pnt aPx, out gp_Dir aD, IntTools_Context? theContext) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in TopoDS_Edge theE, in TopoDS_Face theF, double aT, out gp_Pnt aP, out gp_Dir aDNF, double aDt2D) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in TopoDS_Edge theE, in TopoDS_Face theF, double aT, double aDt2D, out gp_Pnt aP, out gp_Dir aDNF, IntTools_Context? theContext) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, double aDt2D, out gp_Pnt2d aP2D, out gp_Pnt aPx, IntTools_Context? theContext) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, double aDt2D, out gp_Pnt2d aP2D, out gp_Pnt aPx) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, out gp_Pnt2d aP2D, out gp_Pnt aPx, IntTools_Context? theContext) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, out gp_Pnt2d aP2D, out gp_Pnt aPx, IntTools_Context? theContext) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static double MinStepIn2d() => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool IsEmptyShape(in TopoDS_Shape aS) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static void OrientEdgeOnFace(in TopoDS_Edge aE, in TopoDS_Face aF, out TopoDS_Edge aER) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int PointInFace(in TopoDS_Face theF, out gp_Pnt theP, out gp_Pnt2d theP2D, IntTools_Context? theContext) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int PointInFace(in TopoDS_Face theF, in TopoDS_Edge theE, double theT, double theDt2D, out gp_Pnt theP, out gp_Pnt2d theP2D, IntTools_Context? theContext) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static int PointInFace(in TopoDS_Face theF, Geom2d_Curve theL, out gp_Pnt theP, out gp_Pnt2d theP2D, IntTools_Context? theContext, double theDt2D = 0.0) => throw new NotImplementedException();
}
