// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Plane.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Plane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>Projects elementary curves on a plane.</summary>
internal class PlaneProjector : Projector
{
    private Plane3D _plane = new();

    /// <summary>Undefined projection.</summary>
    public PlaneProjector()
    {
    }

    /// <summary>Projection on the plane Pl.</summary>
    public PlaneProjector(in Plane3D pl)
    {
        Init(pl);
    }

    /// <summary>Projection of the line L on the plane Pl.</summary>
    public PlaneProjector(in Plane3D pl, in Line3D l)
    {
        Init(pl);
        Project(l);
    }

    /// <summary>Projection of the circle C on the plane Pl.</summary>
    public PlaneProjector(in Plane3D pl, in Circle3D c)
    {
        Init(pl);
        Project(c);
    }

    /// <summary>Projection of the ellipse E on the plane Pl.</summary>
    public PlaneProjector(in Plane3D pl, in Ellipse3D e)
    {
        Init(pl);
        Project(e);
    }

    /// <summary>Projection of the parabola P on the plane Pl.</summary>
    public PlaneProjector(in Plane3D pl, in Parabola3D p)
    {
        Init(pl);
        Project(p);
    }

    /// <summary>Projection of the hyperbola H on the plane Pl.</summary>
    public PlaneProjector(in Plane3D pl, in Hyperbola3D h)
    {
        Init(pl);
        Project(h);
    }

    public void Init(in Plane3D pl)
    {
        _type = OtherCurve;
        _isDone = false;
        _isPeriodic = false;
        _plane = pl;
    }

    private static Point2D EvalPnt2d(in Point3D p, in Plane3D pl)
    {
        Vector3D op = new(pl.Location, p);
        return new Point2D(op.Dot(new Vector3D(pl.Position.XDirection)),
                            op.Dot(new Vector3D(pl.Position.YDirection)));
    }

    private static Direction2D EvalDir2d(in Direction3D d, in Plane3D pl)
    {
        return new Direction2D(d.Dot(pl.Position.XDirection), d.Dot(pl.Position.YDirection));
    }

    /// <summary>What <see cref="Project(in Line3D)"/> finds, without a projector (unit 11B).</summary>
    internal static Line2D ProjectLine(in Plane3D pl, in Line3D l) => new(EvalPnt2d(l.Location, pl), EvalDir2d(l.Direction, pl));

    public override void Project(in Line3D l)
    {
        _type = global::Kerf.Geometry.CurveKind.Line;
        _lin = ProjectLine(_plane, l);
        _isDone = true;
    }

    public override void Project(in Circle3D c)
    {
        _type = global::Kerf.Geometry.CurveKind.Circle;

        Point2D p2d = EvalPnt2d(c.Location, _plane);
        Direction2D x2d = EvalDir2d(c.Position.XDirection, _plane);
        Direction2D y2d = EvalDir2d(c.Position.YDirection, _plane);
        Frame2D ax = new(p2d, x2d, y2d);

        _circ = new Circle2D(ax, c.Radius);
        _isPeriodic = true;
        _isDone = true;
    }

    public override void Project(in Ellipse3D e)
    {
        _type = global::Kerf.Geometry.CurveKind.Ellipse;

        Point2D p2d = EvalPnt2d(e.Location, _plane);
        Direction2D x2d = EvalDir2d(e.Position.XDirection, _plane);
        Direction2D y2d = EvalDir2d(e.Position.YDirection, _plane);
        Frame2D ax = new(p2d, x2d, y2d);

        _elips = new Ellipse2D(ax, e.MajorRadius, e.MinorRadius);
        _isPeriodic = true;
        _isDone = true;
    }

    public override void Project(in Parabola3D p)
    {
        _type = global::Kerf.Geometry.CurveKind.Parabola;

        Point2D p2d = EvalPnt2d(p.Location, _plane);
        Direction2D x2d = EvalDir2d(p.Position.XDirection, _plane);
        Direction2D y2d = EvalDir2d(p.Position.YDirection, _plane);
        Frame2D ax = new(p2d, x2d, y2d);

        _parab = new Parabola2D(ax, p.Focal);
        _isDone = true;
    }

    public override void Project(in Hyperbola3D h)
    {
        _type = global::Kerf.Geometry.CurveKind.Hyperbola;

        Point2D p2d = EvalPnt2d(h.Location, _plane);
        Direction2D x2d = EvalDir2d(h.Position.XDirection, _plane);
        Direction2D y2d = EvalDir2d(h.Position.YDirection, _plane);
        Frame2D ax = new(p2d, x2d, y2d);

        _hypr = new Hyperbola2D(ax, h.MajorRadius, h.MinorRadius);
        _isDone = true;
    }
}
