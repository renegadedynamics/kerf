// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Sphere.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>Projects elementary curves on a sphere: circles that are U or V isolines.</summary>
internal class SphereProjector : Projector
{
    private Sphere3D _sphere = new();

    /// <summary>Undefined projection.</summary>
    public SphereProjector()
    {
    }

    /// <summary>Projection on the sphere Sp.</summary>
    public SphereProjector(in Sphere3D sp)
    {
        Init(sp);
    }

    /// <summary>Projection of the circle C on the sphere Sp.</summary>
    public SphereProjector(in Sphere3D sp, in Circle3D c)
    {
        Init(sp);
        Project(c);
    }

    public void Init(in Sphere3D sp)
    {
        _type = OtherCurve;
        _sphere = sp;
        _isPeriodic = false;
        _isDone = false;
    }

    private static Point2D EvalPnt2d(in Vector3D p, in Sphere3D sp)
    {
        double x = p.Dot(new Vector3D(sp.Position.XDirection));
        double y = p.Dot(new Vector3D(sp.Position.YDirection));
        double z = p.Dot(new Vector3D(sp.Position.Direction));
        double u, v;

        if (Math.Abs(x) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(y) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            double uu = Math.Atan2(y, x);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(uu, 0.0, 2 * M_PI);
        }
        else
        {
            u = 0.0;
        }

        if (z > 1.0)
        {
            z = 1.0;
        }
        else if (z < -1.0)
        {
            z = -1.0;
        }
        v = Math.Asin(z);

        return new Point2D(u, v);
    }

    public override void Project(in Circle3D c)
    {
        Point3D o;          // O Location of Sp;
        Direction3D xc, yc, zc; // X Y Z Direction of C;
        Direction3D xs, ys, zs; // X Y Z Direction of Sp;

        // Check the validity :
        //                      Xc & Yc must be perpendicular to Zs ->IsoV;
        //                      O,Zs is in the Plane O,Xc,Yc;       ->IsoU;

        o = _sphere.Location;
        xc = c.Position.XDirection;
        yc = c.Position.YDirection;
        zc = xc ^ yc;
        xs = _sphere.Position.XDirection;
        ys = _sphere.Position.YDirection;
        zs = _sphere.Position.Direction;

        bool isIsoU, isIsoV;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        isIsoU = zc.IsNormal(zs, tol) && o.IsEqual(c.Location, tol);
        isIsoV = xc.IsNormal(zs, tol) && yc.IsNormal(zs, tol);

        Point2D p2d1 = new(), p2d2;
        Direction2D d2d = new();

        if (isIsoU)
        {
            _type = global::Kerf.Geometry.CurveKind.Line;

            p2d1 = EvalPnt2d(new Vector3D(xc), _sphere);
            p2d2 = EvalPnt2d(new Vector3D(yc), _sphere);

            if (isIsoU
                && (Math.Abs(p2d1.Y - M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                    || Math.Abs(p2d1.Y + M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                // then P1 is on the apex of the sphere and U is undefined
                // The value of U is given by P2d2.Y() .
                p2d1 = p2d1.WithX(p2d2.X);
            }
            else if (Math.Abs(Math.Abs(p2d1.X - p2d2.X) - M_PI) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                // then we have U2 = U1 + PI; V2;
                // we have to assume that U1 = U2
                //   so V2 = PI - V2;
                p2d2 = p2d2.WithX(p2d1.X);
                if (p2d2.Y < 0.0)
                {
                    p2d2 = p2d2.WithY(-M_PI - p2d2.Y);
                }
                else
                {
                    p2d2 = p2d2.WithY(M_PI - p2d2.Y);
                }
            }
            else
            {
                p2d2 = p2d2.WithX(p2d1.X);
            }

            d2d = new Direction2D(new Vector2D(p2d1, p2d2));
            _isDone = true;
        }
        else if (isIsoV)
        {
            _type = global::Kerf.Geometry.CurveKind.Line;

            // P2d(U,V) :first point of the PCurve.
            double u = xs.AngleWithRef(xc, xs ^ ys);
            if (u < 0)
            {
                u += 2 * M_PI;
            }
            double z = new Vector3D(o, c.Location).Dot(new Vector3D(zs));
            double v = Math.Asin(z / _sphere.Radius);
            p2d1 = new Point2D(u, v);
            d2d = new Direction2D((xc ^ yc).Dot(xs ^ ys), 0.0);
            _isDone = true;
        }
        _lin = new Line2D(p2d1, d2d);
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

    /// <summary>Sets the point of parameter U on C in the natural restrictions of the sphere.</summary>
    /// <exception cref="NotDoneException">The projection is not done.</exception>
    public void SetInBounds(double u)
    {
        StdFail_NotDone_Raise_if(!_isDone, "ProjLib_Sphere:SetInBounds");

        // first set the y of the first point in -pi/2 pi/2
        double newY, y = global::Kerf.Engine.Geometry.ElCLib.Value(u, _lin).Y;
        newY = global::Kerf.Engine.Geometry.ElCLib.InPeriod(y, -M_PI, M_PI);

        _lin = _lin.Translated(new Vector2D(0.0, newY - y));

        Point2D p = global::Kerf.Engine.Geometry.ElCLib.Value(u, _lin);
        Transform2D trsf = new();
        Axis2D axis;
        double tol = 1.0e-7;
        Direction2D d2d = _lin.Direction;
        //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 Begin
        //   if ((P.Y() > M_PI/2) ||
        if ((p.Y - M_PI / 2 > tol) ||
            //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 End
            (Math.Abs(p.Y - M_PI / 2) < tol && d2d.IsEqual(Constants.DY2d(), tol)))
        {
            axis = new Axis2D(new Point2D(0.0, M_PI / 2.0), Constants.DX2d());
        }
        //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 Begin
        //   else if ((P.Y() < -M_PI/2) ||
        else if ((p.Y + M_PI / 2 < -tol) ||
                 //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 End
                 (Math.Abs(p.Y + M_PI / 2) < tol && d2d.IsOpposite(Constants.DY2d(), tol)))
        {
            axis = new Axis2D(new Point2D(0.0, -M_PI / 2.0), Constants.DX2d());
        }
        else
        {
            return;
        }
        trsf = trsf.WithMirror(axis);
        _lin = _lin.Transformed(trsf);

        _lin = _lin.Translated(new Vector2D(M_PI, 0.0));

        // Now we need to adjust the U parameter
        double newX, x = global::Kerf.Engine.Geometry.ElCLib.Value(u, _lin).X;
        newX = global::Kerf.Engine.Geometry.ElCLib.InPeriod(x, 0.0, 2.0 * M_PI);
        _lin = _lin.Translated(new Vector2D(newX - x, 0.0));
    }
}
