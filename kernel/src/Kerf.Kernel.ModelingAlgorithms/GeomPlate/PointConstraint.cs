// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PointConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PointConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A point constraint of a plate surface: a point to pass through (order 0, or -1 for a point that is only
/// approached), or a point on a surface whose tangent plane (order 1) or curvature (order 2) the plate must follow.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: its setters are public, and GeomPlate_BuildPlateSurface sets the
/// order and the 2D point it finds. It keeps the caller's surface, read only.
/// </remarks>
internal class PointConstraint : Transient
{
    protected int _order;
    private readonly SLProps _lProp;
    private readonly Point3D _point;
    private Point2D _pt2d;
    private readonly Surface? _surf;
    private readonly Vector3D _d11;
    private readonly Vector3D _d12;
    private readonly Vector3D _d21;
    private readonly Vector3D _d22;
    private readonly Vector3D _d23;
    private readonly double _u;
    private readonly double _v;
    private double _tolDist;
    private double _tolAng;
    private double _tolCurv;
    private bool _hasPnt2dOnSurf;

    /// <summary>A point the plate passes through (order 0) or approaches (order -1).</summary>
    public PointConstraint(in Point3D pt, int order, double tolDist = 0.0001)
    {
        _order = order;
        _lProp = new SLProps(2, tolDist);
        _point = pt;
        _pt2d = new Point2D();
        _u = 0.0;
        _v = 0.0;
        _tolDist = tolDist;
        _tolAng = 0.0;
        _tolCurv = 0.0;
        _hasPnt2dOnSurf = false;
        if ((_order > 1) || (_order < -1))
        {
            throw new FailureException("GeomPlate_PointConstraint : the constraint must 0 or -1 with a point");
        }
    }

    /// <summary>The point of <paramref name="surf"/> at (U, V), with the surface's derivatives there.</summary>
    public PointConstraint(double u, double v, Surface surf, int order, double tolDist = 0.0001,
                                     double tolAng = 0.01, double tolCurv = 0.1)
    {
        _order = order;
        _lProp = new SLProps(2, tolDist);
        _surf = surf;
        _pt2d = new Point2D();
        _u = u;
        _v = v;
        _tolDist = tolDist;
        _tolAng = tolAng;
        _tolCurv = tolCurv;
        _hasPnt2dOnSurf = false;
        surf.D2(_u, _v, out _point, out _d11, out _d12, out _d21, out _d22, out _d23);
        _lProp.SetSurface(surf);
    }

    public void D0(out Point3D p)
    {
        p = _point;
    }

    public void D1(out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        p = _point;
        v1 = _d11;
        v2 = _d12;
    }

    public void D2(out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3, out Vector3D v4, out Vector3D v5)
    {
        p = _point;
        v1 = _d11;
        v2 = _d12;
        v3 = _d21;
        v4 = _d22;
        v5 = _d23;
    }

    public void SetG0Criterion(double tolDist)
    {
        _tolDist = tolDist;
    }

    public void SetG1Criterion(double tolAng)
    {
        _tolAng = tolAng;
    }

    public void SetG2Criterion(double tolCurv)
    {
        _tolCurv = tolCurv;
    }

    public double G0Criterion() => _tolDist;

    public double G1Criterion() => _tolAng;

    public double G2Criterion() => _tolCurv;

    /// <summary>The local properties of the surface at the constraint's parameters (the object the constraint keeps).</summary>
    public SLProps LPropSurf()
    {
        _lProp.SetParameters(_u, _v);
        return _lProp;
    }

    public int Order() => _order;

    public void SetOrder(int order)
    {
        _order = order;
    }

    public bool HasPnt2dOnSurf => _hasPnt2dOnSurf;

    public void SetPnt2dOnSurf(in Point2D pnt2d)
    {
        _pt2d = pnt2d;
        _hasPnt2dOnSurf = true;
    }

    public Point2D Pnt2dOnSurf() => _pt2d;
}
