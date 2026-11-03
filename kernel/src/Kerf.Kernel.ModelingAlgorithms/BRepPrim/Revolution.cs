// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Revolution.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Revolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>Implements the OneAxis algorithm for a revolution surface.</summary>
internal class Revolution : OneAxis
{
    private Curve3D? _meridian;
    private Curve2D? _pMeridian;

    /// <summary>
    /// A revolution body: <paramref name="m"/> is the meridian and must be in the XZ plane of <paramref name="a"/>;
    /// <paramref name="pm"/> is the meridian in the XZ plane.
    /// </summary>
    public Revolution(in Frame3D a, double vMin, double vMax, Curve3D m, Curve2D pm)
        : base(new PrimitiveBuilder(), a, vMin, vMax)
    {
        _meridian = m;
        _pMeridian = pm;
    }

    /// <summary>A revolution body whose meridian is set later. Reserved for derived classes.</summary>
    protected Revolution(in Frame3D a, double vMin, double vMax)
        : base(new PrimitiveBuilder(), a, vMin, vMax)
    {
    }

    protected void Meridian(Curve3D m, Curve2D pm)
    {
        _meridian = m;
        _pMeridian = pm;
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override Face MakeEmptyLateralFace()
    {
        RevolutionSurface s = new(_meridian!, Axes().Axis());
        Face f = new();
        _builder.Builder().MakeFace(ref f, s, global::Kerf.Engine.Foundation.Precision.Confusion());
        return f;
    }

    /// <summary>An edge with a 3D curve made from the meridian rotated by <paramref name="ang"/> around the axis.</summary>
    public override Edge MakeEmptyMeridianEdge(double ang)
    {
        Edge e = new();
        Curve3D c = (Curve3D)_meridian!.Copy();
        Transform3D t = new();
        t = t.WithRotation(Axes().Axis(), ang);
        c.Transform(t);
        _builder.Builder().MakeEdge(ref e, c, global::Kerf.Engine.Foundation.Precision.Confusion());
        return e;
    }

    /// <summary>The meridian point at parameter <paramref name="v"/> in the plane XZ.</summary>
    public override Point2D MeridianValue(double v)
    {
        return _pMeridian!.Value(v);
    }

    /// <summary>Sets the pcurve of the edge on the face to the 2D representation of the meridian.</summary>
    public override void SetMeridianPCurve(in Edge e, in Face f)
    {
        _builder.Builder().UpdateEdge(e, _pMeridian, f, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
