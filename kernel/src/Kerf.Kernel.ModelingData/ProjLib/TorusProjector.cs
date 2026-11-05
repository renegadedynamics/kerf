// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Torus.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>Projects elementary curves on a torus: circles that are U or V isolines.</summary>
internal class TorusProjector : Projector
{
    private Torus3D _torus = new();

    /// <summary>Undefined projection.</summary>
    public TorusProjector()
    {
    }

    /// <summary>Projection on the torus To.</summary>
    public TorusProjector(in Torus3D to)
    {
        Init(to);
    }

    /// <summary>Projection of the circle C on the torus To.</summary>
    public TorusProjector(in Torus3D to, in Circle3D c)
    {
        Init(to);
        Project(c);
    }

    public void Init(in Torus3D to)
    {
        _type = OtherCurve;
        _torus = to;
        _isPeriodic = false;
        _isDone = false;
    }

    private static Point2D EvalPnt2d(in Vector3D ve, in Torus3D to)
    {
        double x = ve.Dot(new Vector3D(to.Position.XDirection));
        double y = ve.Dot(new Vector3D(to.Position.YDirection));
        double u, v;

        if (Math.Abs(x) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(y) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            u = Math.Atan2(y, x);
        }
        else
        {
            u = 0.0;
        }

        v = 0.0;
        return new Point2D(u, v);
    }

    public override void Project(in Circle3D c)
    {
        _type = global::Kerf.Geometry.CurveKind.Line;

        Vector3D xc = new(c.Position.XDirection);
        Vector3D yc = new(c.Position.YDirection);
        Vector3D xt = new(_torus.Position.XDirection);
        Vector3D yt = new(_torus.Position.YDirection);
        Vector3D zt = new(_torus.Position.Direction);
        Vector3D oc = new(_torus.Location, c.Location);

        //  if (OC.Magnitude() < Precision::Confusion()      ||
        //      OC.IsParallel(myTorus.Position().Direction(),
        //		    Precision::Angular())) {

        if (oc.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion()
            || c.Position.Direction.IsParallel(_torus.Position.Direction, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Iso V
            Point2D p1 = EvalPnt2d(xc, _torus); // evaluate U1
            Point2D p2 = EvalPnt2d(yc, _torus); // evaluate U2
            double z = oc.Dot(new Vector3D(_torus.Position.Direction));
            z /= _torus.MinorRadius;

            double v;

            if (z > 1.0)
            {
                v = M_PI / 2.0; // simple protection
            } // against calculation errors
            else if (z < -1.0)
            {              // it happens that Z is slightly
                v = -M_PI / 2; // greater than 1.
            }
            else
            {
                v = Math.Asin(z);
            }

            if (c.Radius < _torus.MajorRadius)
            {
                v = M_PI - v;
            }
            else if (v < 0.0)
            {
                v += 2 * M_PI;
            }
            p1 = p1.WithY(v);
            p2 = p2.WithY(v);
            Vector2D v2d = new(p1, p2);
            // Normally std::abs( P1.X() - P2.X()) = PI/2
            // If != PI/2, we crossed the period => Reverse the Direction
            if (Math.Abs(p1.X - p2.X) > M_PI)
            {
                v2d = v2d.Reversed();
            }

            Direction2D d2 = new(v2d);
            if (p1.X < 0)
            {
                p1 = p1.WithX(2 * M_PI + p1.X);
            }
            _lin = new Line2D(p1, d2);
        }
        else
        {
            // Iso U  -> U = angle( Xt, OC)
            double u = xt.AngleWithRef(oc, xt ^ yt);
            if (u < 0.0)
            {
                u += 2 * M_PI;
            }

            // Origin of the line
            double v1 = oc.AngleWithRef(xc, oc ^ zt);
            if (v1 < 0.0)
            {
                v1 += 2 * M_PI;
            }

            Point2D p1 = new(u, v1);

            // Direction of the line
            Direction2D d2 = Constants.DY2d();
            if (((oc ^ zt) * (xc ^ yc)) < 0.0)
            {
                d2 = d2.Reversed();
            }

            _lin = new Line2D(p1, d2);
        }
        _isDone = true;
    }

    public override void Project(in Line3D l)
    {
        base.Project(l);
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
