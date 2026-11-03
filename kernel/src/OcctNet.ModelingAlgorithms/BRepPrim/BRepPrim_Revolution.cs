// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Revolution.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Revolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Implements the OneAxis algorithm for a revolution surface.</summary>
internal class BRepPrim_Revolution : BRepPrim_OneAxis
{
    private Geom_Curve? myMeridian;
    private Geom2d_Curve? myPMeridian;

    /// <summary>
    /// A revolution body: <paramref name="M"/> is the meridian and must be in the XZ plane of <paramref name="A"/>;
    /// <paramref name="PM"/> is the meridian in the XZ plane.
    /// </summary>
    public BRepPrim_Revolution(in Frame3D A, double VMin, double VMax, Geom_Curve M, Geom2d_Curve PM)
        : base(new BRepPrim_Builder(), A, VMin, VMax)
    {
        myMeridian = M;
        myPMeridian = PM;
    }

    /// <summary>A revolution body whose meridian is set later. Reserved for derived classes.</summary>
    protected BRepPrim_Revolution(in Frame3D A, double VMin, double VMax)
        : base(new BRepPrim_Builder(), A, VMin, VMax)
    {
    }

    protected void Meridian(Geom_Curve M, Geom2d_Curve PM)
    {
        myMeridian = M;
        myPMeridian = PM;
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override TopoDS_Face MakeEmptyLateralFace()
    {
        Geom_SurfaceOfRevolution S = new(myMeridian!, Axes().Axis());
        TopoDS_Face F = new();
        myBuilder.Builder().MakeFace(ref F, S, global::Kerf.Engine.Foundation.Precision.Confusion());
        return F;
    }

    /// <summary>An edge with a 3D curve made from the meridian rotated by <paramref name="Ang"/> around the axis.</summary>
    public override TopoDS_Edge MakeEmptyMeridianEdge(double Ang)
    {
        TopoDS_Edge E = new();
        Geom_Curve C = (Geom_Curve)myMeridian!.Copy();
        Transform3D T = new();
        T.SetRotation(Axes().Axis(), Ang);
        C.Transform(T);
        myBuilder.Builder().MakeEdge(ref E, C, global::Kerf.Engine.Foundation.Precision.Confusion());
        return E;
    }

    /// <summary>The meridian point at parameter <paramref name="V"/> in the plane XZ.</summary>
    public override Point2D MeridianValue(double V)
    {
        return myPMeridian!.Value(V);
    }

    /// <summary>Sets the pcurve of the edge on the face to the 2D representation of the meridian.</summary>
    public override void SetMeridianPCurve(in TopoDS_Edge E, in TopoDS_Face F)
    {
        myBuilder.Builder().UpdateEdge(E, myPMeridian, F, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
