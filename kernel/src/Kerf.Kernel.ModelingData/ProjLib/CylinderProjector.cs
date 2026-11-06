// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cylinder.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>Projects elementary curves on a cylinder: lines parallel to the axis and circles normal to it.</summary>
internal class CylinderProjector : Projector
{
    private Cylinder3D _cylinder = new();

    /// <summary>Undefined projection.</summary>
    public CylinderProjector()
    {
    }

    /// <summary>Projection on the cylinder Cyl.</summary>
    public CylinderProjector(in Cylinder3D cyl)
    {
        Init(cyl);
    }

    /// <summary>Projection of the line L on the cylinder Cyl.</summary>
    public CylinderProjector(in Cylinder3D cyl, in Line3D l)
    {
        Init(cyl);
        Project(l);
    }

    /// <summary>Projection of the circle C on the cylinder Cyl.</summary>
    public CylinderProjector(in Cylinder3D cyl, in Circle3D c)
    {
        Init(cyl);
        Project(c);
    }

    /// <summary>Projection of the ellipse E on the cylinder Cyl (left to approximation: never done).</summary>
    public CylinderProjector(in Cylinder3D cyl, in Ellipse3D e)
    {
        Init(cyl);
        Project(e);
    }

    public void Init(in Cylinder3D cyl)
    {
        _type = OtherCurve;
        _cylinder = cyl;
        _isPeriodic = false;
        _isDone = false;
    }

    private static Point2D EvalPnt2d(in Point3D p, in Cylinder3D cy)
    {
        Vector3D op = new(cy.Location, p);
        double x = op.Dot(new Vector3D(cy.Position.XDirection));
        double y = op.Dot(new Vector3D(cy.Position.YDirection));
        double z = op.Dot(new Vector3D(cy.Position.Direction));
        double u;

        if (Math.Abs(x) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(y) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            u = Math.Atan2(y, x);
        }
        else
        {
            u = 0.0;
        }
        return new Point2D(u, z);
    }

    public override void Project(in Line3D l)
    {
        // Check the line is parallel to the axis of cylinder.
        // In other cases, the projection is wrong.
        if (l.Direction.XYZ().CrossSquareMagnitude(_cylinder.Position.Direction.XYZ())
            > global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return;
        }

        _type = global::Kerf.Geometry.CurveKind.Line;

        Point2D p2d = EvalPnt2d(l.Location, _cylinder);
        if (p2d.X < 0.0)
        {
            p2d = p2d.WithX(p2d.X + 2 * M_PI);
        }
        double signe = l.Direction.Dot(_cylinder.Position.Direction);
        signe = (signe > 0.0) ? 1.0 : -1.0;
        Direction2D d2d = new(0.0, signe);

        _lin = new Line2D(p2d, d2d);
        _isDone = true;
    }

    public override void Project(in Circle3D c)
    {
        // Check the circle's normal is parallel to the axis of cylinder.
        // In other cases, the projection is wrong.
        CoordinateSystem cylPos = _cylinder.Position;
        Frame3D circPos = c.Position;
        if (cylPos.Direction.XYZ().CrossSquareMagnitude(circPos.Direction.XYZ())
            > global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return;
        }

        _type = global::Kerf.Geometry.CurveKind.Line;

        Direction3D zCyl = cylPos.XDirection.Crossed(cylPos.YDirection);

        double u = cylPos.XDirection.AngleWithRef(circPos.XDirection, zCyl);

        Vector3D op = new(_cylinder.Location, c.Location);
        double v = op.Dot(new Vector3D(cylPos.Direction));

        Point2D p2d1 = new(u, v);
        Direction2D d2d = new();
        if (zCyl.Dot(circPos.Direction) > 0.0)
        {
            d2d = d2d.WithCoord(1.0, 0.0);
        }
        else
        {
            d2d = d2d.WithCoord(-1.0, 0.0);
        }

        _lin = new Line2D(p2d1, d2d);
        _isDone = true;
    }

    // void  ProjLib_Cylinder::Project(const gp_Elips& E)
    public override void Project(in Ellipse3D e)
    {
        // Due to widespread issues with poorly handled periodicity,
        // the projection of an ellipse onto a cylinder is delegated to approximation.
    }

    public override void Project(in Parabola3D p)
    {
        base.Project(p);
    }

    public override void Project(in Hyperbola3D h)
    {
        base.Project(h);
    }
}
