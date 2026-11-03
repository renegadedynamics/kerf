// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cone.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>Projects elementary curves on a cone: lines along a generatrix and circles normal to the axis.</summary>
internal class ConeProjector : Projector
{
    private Cone _cone = new();

    /// <summary>Undefined projection.</summary>
    public ConeProjector()
    {
    }

    /// <summary>Projection on the cone Co.</summary>
    public ConeProjector(in Cone co)
    {
        Init(co);
    }

    /// <summary>Projection of the line L on the cone Co.</summary>
    public ConeProjector(in Cone co, in Line3D l)
    {
        Init(co);
        Project(l);
    }

    /// <summary>Projection of the circle C on the cone Co.</summary>
    public ConeProjector(in Cone co, in Circle3D c)
    {
        Init(co);
        Project(c);
    }

    public void Init(in Cone co)
    {
        _type = OtherCurve;
        _cone = co;
        _isPeriodic = false;
        _isDone = false;
    }

    public override void Project(in Line3D l)
    {
        Point3D pnt = l.Location, apex = _cone.Apex;

        double deltaV = 0.0;

        double u, v;
        if (pnt.IsEqual(apex, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // Take another point in the line L, which does not coincide with the cone apex.
            pnt = pnt.Translated(new Vector3D(l.Direction.XYZ()));
            deltaV = 1.0; // == ||L.Direction()|| == 1.0
        }

        global::Kerf.Engine.Geometry.ElSLib.ConeParameters(_cone.Position, _cone.RefRadius, _cone.SemiAngle, pnt, out u, out v);
        //
        Point3D p;
        Vector3D vu, vv;

        global::Kerf.Engine.Geometry.ElSLib.ConeD1(u, v, _cone.Position, _cone.RefRadius, _cone.SemiAngle, out p, out vu, out vv);

        Direction3D dv = new(vv);
        if (dv.IsParallel(l.Direction, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // L is parallel to U-isoline of the cone.
            _type = global::Kerf.Geometry.CurveKind.Line;

            double sign = Math.CopySign(1.0, l.Direction.Dot(dv));
            Point2D p2d = new(u, v - deltaV * sign);
            Direction2D d2d = new(0.0, sign);

            _lin = new Line2D(p2d, d2d);

            _isDone = true;
        }
    }

    public override void Project(in Circle3D c)
    {
        // OCCT: the type is set before the check below, so a circle not normal to the axis leaves a "Line" result
        // that is not done, and Line() returns the default line instead of throwing
        _type = global::Kerf.Geometry.CurveKind.Line;

        CoordinateSystem conePos = _cone.Position;
        CoordinateSystem circPos = new(c.Position);
        //
        if (!conePos.Direction.IsParallel(circPos.Direction, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            _isDone = false;
            return;
        }
        //
        Direction3D zCone = conePos.XDirection.Crossed(conePos.YDirection);
        Direction3D zCir = circPos.XDirection.Crossed(circPos.YDirection);

        double u, v;
        double x = conePos.XDirection.Dot(circPos.XDirection);
        double y = conePos.YDirection.Dot(circPos.XDirection);
        double z = new Vector3D(_cone.Location, c.Location).Dot(new Vector3D(conePos.Direction));

        // to find point U V, we use the code from ElSLib
        // without applying the Trsf to the point (unnecessary round trip).
        // x/y are direction cosines (dimensionless); use angular tolerance
        // to avoid unstable atan2() around zero.
        if (Math.Abs(x) <= global::Kerf.Engine.Foundation.Precision.Angular() && Math.Abs(y) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            u = 0.0;
        }
        else if (-_cone.RefRadius > z * Math.Tan(_cone.SemiAngle))
        {
            u = Math.Atan2(-y, -x);
        }
        else
        {
            u = Math.Atan2(y, x);
        }
        if (u < 0.0)
        {
            u += 2 * M_PI;
        }

        v = z / Math.Cos(_cone.SemiAngle);

        Point2D p2d1 = new(u, v);
        Direction2D d2d = new();
        if (zCone.Dot(zCir) > 0.0)
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

    public override void Project(in Ellipse3D e)
    {
        base.Project(e);
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
