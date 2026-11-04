// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ContAna.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ContAna.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ContAna.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The contours of the quadrics in closed form: a circle on a sphere, two lines on a cylinder, two (or four, for a
/// draft) lines through the apex of a cone; for a view direction, a draft angle, or an eye.
/// </summary>
/// <remarks>A working object, as OCCT's: <c>Perform</c> sets the result, the queries read it.</remarks>
internal sealed class ContAna
{
    private const double Tolpetit = 1.0e-8;

    private bool _done;
    private int _nbSol;
    private CurveKind _typL;
    private Point3D _pt1 = new();
    private Point3D _pt2 = new();
    private Point3D _pt3 = new();
    private Point3D _pt4 = new();
    private Direction3D _dir1 = new();
    private Direction3D _dir2 = new();
    private Direction3D _dir3 = new();
    private Direction3D _dir4 = new();
    private double _prm;

    public ContAna()
    {
        _done = false;
        _nbSol = 0;
        _typL = OtherCurve;
        _prm = 0.0;
    }

    /// <summary>The silhouette of a sphere for the direction <paramref name="d"/>.</summary>
    public void Perform(in Sphere s, in Direction3D d)
    {
        _done = false;
        _typL = global::Kerf.Geometry.CurveKind.Circle;
        _pt1 = s.Location;
        _dir1 = d;
        if (Math.Abs(d.Dot(s.XAxis.Direction)) < 0.9999999999999)
        {
            _dir2 = d.Crossed(s.XAxis.Direction);
        }
        else
        {
            _dir2 = d.Crossed(s.YAxis.Direction);
        }
        _prm = s.Radius;
        _nbSol = 1;
        _done = true;
    }

    /// <summary>The draft contour of angle <paramref name="angle"/> of a sphere for the direction <paramref name="d"/>.</summary>
    public void Perform(in Sphere s, in Direction3D d, double angle)
    {
        _done = false;
        _typL = global::Kerf.Geometry.CurveKind.Circle;

        _dir1 = d;
        if (Math.Abs(d.Dot(s.XAxis.Direction)) < 0.9999999999999)
        {
            _dir2 = d.Crossed(s.XAxis.Direction);
        }
        else
        {
            _dir2 = d.Crossed(s.YAxis.Direction);
        }
        double alpha = (s.IsDirect ? angle : -angle);
        _pt1 = _pt1.WithXYZ(s.Location.XYZ() - s.Radius * Math.Sin(alpha) * d.XYZ());
        _prm = s.Radius * Math.Cos(alpha);
        _nbSol = 1;
        _done = true;
    }

    /// <summary>The silhouette of a sphere for the eye <paramref name="eye"/>.</summary>
    public void Perform(in Sphere s, in Point3D eye)
    {
        _done = false;

        double radius = s.Radius;
        double dist = eye.DistanceTo(s.Location);
        if (dist <= radius)
        {
            _nbSol = 0;
        }
        else
        {
            _prm = radius * Math.Sqrt(1.0 - radius * radius / (dist * dist));
            if (_prm < Tolpetit)
            {
                _nbSol = 0;
            }
            else
            {
                Xyz locxyz = s.Location.XYZ();
                _dir1 = _dir1.WithXYZ(eye.XYZ() - locxyz);
                _pt1 = _pt1.WithXYZ(locxyz + (radius * radius / dist) * _dir1.XYZ());
                if (Math.Abs(_dir1.Dot(s.XAxis.Direction)) < 0.9999999999999)
                {
                    _dir2 = _dir1.Crossed(s.XAxis.Direction);
                }
                else
                {
                    _dir2 = _dir1.Crossed(s.YAxis.Direction);
                }
                _nbSol = 1;
                _typL = global::Kerf.Geometry.CurveKind.Circle;
            }
        }
        _done = true;
    }

    /// <summary>The silhouette of a cylinder for the direction <paramref name="d"/>.</summary>
    public void Perform(in Cylinder c, in Direction3D d)
    {
        _done = false;

        Xyz normale = c.Position.Direction.XYZ();
        normale.Cross(d.XYZ());
        if (normale.Modulus() <= 1e-15)
        {
            _nbSol = 0;
        }
        else
        {
            normale.Normalize();
            _typL = global::Kerf.Geometry.CurveKind.Line;
            _dir1 = c.Position.Direction;
            _dir2 = _dir1;
            _pt1 = _pt1.WithXYZ(c.Location.XYZ() + c.Radius * normale);
            _pt2 = _pt2.WithXYZ(c.Location.XYZ() - c.Radius * normale);
            _nbSol = 2;
        }

        _done = true;
    }

    /// <summary>The draft contour of angle <paramref name="angle"/> of a cylinder for the direction <paramref name="d"/>.</summary>
    public void Perform(in Cylinder c, in Direction3D d, double angle)
    {
        _done = false;

        double coefcos = d.Dot(c.Position.XDirection);
        double coefsin = d.Dot(c.Position.YDirection);
        double coefcst = Math.Cos(M_PI * 0.5 + angle);

        double norm1 = coefcos * coefcos + coefsin * coefsin;
        double norm2 = Math.Sqrt(norm1);

        if (Math.Abs(coefcst) < norm2)
        {
            _typL = global::Kerf.Geometry.CurveKind.Line;
            _nbSol = 2;
            _dir1 = _dir2 = c.Position.Direction;

            if (!c.IsDirect)
            { // The normal is inverted.
                coefcos = -coefcos;
                coefsin = -coefsin;
            }

            // Necessary to solve Coefcos*cos(t) + Coefsin*sin(t) = Coefcst
            // and the origins of solution are in the reference of the
            // cylinder in (R*cost0, R*sint0,0) and (R*cost1,R*sint1,0)
            // By setting cos(phi) = Coefcos/std::sqrt(Coefcos**2 + Coefsin**2) and
            //           sin(phi) = Coefsin/std::sqrt(Coefcos**2 + Coefsin**2)
            // and by using trigonometric relations the values of cosinus
            // and sinus to the solutions are obtained.

            _prm = Math.Sqrt(norm1 - coefcst * coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (coefcos * coefcst - coefsin * _prm) / norm1;
            cost1 = (coefcos * coefcst + coefsin * _prm) / norm1;

            sint0 = (coefcos * _prm + coefsin * coefcst) / norm1;
            sint1 = (-coefcos * _prm + coefsin * coefcst) / norm1;

            Xyz xdir = c.Position.XDirection.XYZ();
            Xyz ydir = c.Position.YDirection.XYZ();
            Xyz dirxyz = new();

            dirxyz.SetLinearForm(cost0, xdir, sint0, ydir);
            dirxyz.Multiply(c.Radius);
            _pt1 = _pt1.WithXYZ(c.Location.XYZ().Added(dirxyz));

            dirxyz.SetLinearForm(cost1, xdir, sint1, ydir);
            dirxyz.Multiply(c.Radius);
            _pt2 = _pt2.WithXYZ(c.Location.XYZ().Added(dirxyz));
        }
        else
        {
            _nbSol = 0;
        }

        _done = true;
    }

    /// <summary>The silhouette of a cylinder for the eye <paramref name="eye"/>.</summary>
    public void Perform(in Cylinder c, in Point3D eye)
    {
        _done = false;

        double radius = c.Radius;
        Line3D theaxis = new(c.Axis);
        double dist = theaxis.DistanceTo(eye);
        if (dist <= radius)
        {
            _nbSol = 0;
        }
        else
        {
            _typL = global::Kerf.Geometry.CurveKind.Line;
            _prm = radius * Math.Sqrt(1.0 - radius * radius / (dist * dist));
            _dir1 = c.Axis.Direction;
            _dir2 = _dir1;
            Xyz axeye = theaxis.Normal(eye).Direction.XYZ(); // orientate the axis to the outside
            Xyz normale = (theaxis.Direction.Crossed(new Direction3D(axeye))).XYZ();
            //      normale.Normalize();
            _pt1 = _pt1.WithXYZ(c.Location.XYZ() + (radius * radius / dist) * axeye);
            _pt2 = _pt2.WithXYZ(_pt1.XYZ() - _prm * normale);
            _pt1 = _pt1.WithXYZ(_pt1.XYZ() + _prm * normale);
            _nbSol = 2;
        }
        _done = true;
    }

    /// <summary>The silhouette of a cone for the direction <paramref name="d"/>.</summary>
    public void Perform(in Cone c, in Direction3D d)
    {
        _done = false;

        double tgtalpha = Math.Tan(c.SemiAngle);

        double coefcos = d.Dot(c.Position.XDirection);
        double coefsin = d.Dot(c.Position.YDirection);
        double coefcst = d.Dot(c.Axis.Direction) * tgtalpha;

        double norm1 = coefcos * coefcos + coefsin * coefsin;
        double norm2 = Math.Sqrt(norm1);

        if (Math.Abs(coefcst) < norm2)
        {
            _typL = global::Kerf.Geometry.CurveKind.Line;
            _nbSol = 2;
            _pt1 = c.Apex;
            _pt2 = _pt1;
            // Necessary to solve Coefcos*cos(t) + Coefsin*sin(t) = Coefcst
            // and director vectors of solutions are
            // cos(t0) * XDirection + sin(t0) * YDirection + ZDirection/Tgtalpha

            _prm = Math.Sqrt(norm1 - coefcst * coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (coefcos * coefcst - coefsin * _prm) / norm1;
            cost1 = (coefcos * coefcst + coefsin * _prm) / norm1;

            sint0 = (coefcos * _prm + coefsin * coefcst) / norm1;
            sint1 = (-coefcos * _prm + coefsin * coefcst) / norm1;

            Xyz xdir = c.Position.XDirection.XYZ();
            Xyz ydir = c.Position.YDirection.XYZ();
            Xyz zdir = c.Axis.Direction.XYZ();
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, xdir, sint0, ydir, 1.0 / tgtalpha, zdir);
            _dir1 = _dir1.WithXYZ(dirxyz);
            _pt1 = _pt1.WithXYZ(_pt1.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, xdir, sint1, ydir, 1.0 / tgtalpha, zdir);
            _dir2 = _dir2.WithXYZ(dirxyz);
            _pt2 = _pt2.WithXYZ(_pt2.XYZ() + dirxyz);
        }
        else
        {
            _nbSol = 0;
        }
        _done = true;
    }

    /// <summary>The draft contours of angle <paramref name="angle"/> of a cone for the direction <paramref name="d"/>.</summary>
    public void Perform(in Cone c, in Direction3D d, double angle)
    {
        _done = false;
        _nbSol = 0;

        double ang = c.SemiAngle;
        double cosa = Math.Cos(ang);
        double sina = Math.Sin(ang);

        double coefcos = d.Dot(c.Position.XDirection);
        double coefsin = d.Dot(c.Position.YDirection);

        double coefcst1 = Math.Cos(M_PI * 0.5 + angle);

        double norm1 = coefcos * coefcos + coefsin * coefsin;
        double norm2 = Math.Sqrt(norm1);

        double coefnz = d.Dot(c.Axis.Direction) * sina;
        double coefcst = (coefcst1 + coefnz) / cosa;

        if (Math.Abs(coefcst) < norm2)
        {
            _typL = global::Kerf.Geometry.CurveKind.Line;
            _nbSol += 2;
            _pt1 = c.Apex;
            _pt2 = _pt1;

            _prm = Math.Sqrt(norm1 - coefcst * coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (coefcos * coefcst - coefsin * _prm) / norm1;
            cost1 = (coefcos * coefcst + coefsin * _prm) / norm1;

            sint0 = (coefcos * _prm + coefsin * coefcst) / norm1;
            sint1 = (-coefcos * _prm + coefsin * coefcst) / norm1;

            Xyz xdir = c.Position.XDirection.XYZ();
            Xyz ydir = c.Position.YDirection.XYZ();
            Xyz zdir = c.Axis.Direction.XYZ();
            if (!c.IsDirect)
            {
                zdir.Reverse();
            }
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, xdir, sint0, ydir, cosa / sina, zdir);
            _dir1 = _dir1.WithXYZ(dirxyz);
            _pt1 = _pt1.WithXYZ(_pt1.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, xdir, sint1, ydir, cosa / sina, zdir);
            _dir2 = _dir2.WithXYZ(dirxyz);
            _pt2 = _pt2.WithXYZ(_pt2.XYZ() + dirxyz);
        }

        coefcst = (coefcst1 - coefnz) / cosa;

        if (Math.Abs(coefcst) < norm2)
        {
            _typL = global::Kerf.Geometry.CurveKind.Line;
            _nbSol += 2;
            _pt3 = c.Apex;
            _pt4 = _pt3;

            _prm = Math.Sqrt(norm1 - coefcst * coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (coefcos * coefcst - coefsin * _prm) / norm1;
            cost1 = (coefcos * coefcst + coefsin * _prm) / norm1;

            sint0 = (coefcos * _prm + coefsin * coefcst) / norm1;
            sint1 = (-coefcos * _prm + coefsin * coefcst) / norm1;

            Xyz xdir = c.Position.XDirection.XYZ();
            Xyz ydir = c.Position.YDirection.XYZ();
            Xyz zdir = c.Axis.Direction.XYZ();
            if (!c.IsDirect)
            {
                zdir.Reverse();
            }
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, xdir, sint0, ydir, -cosa / sina, zdir);
            _dir3 = _dir3.WithXYZ(dirxyz);
            _pt3 = _pt3.WithXYZ(_pt3.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, xdir, sint1, ydir, -cosa / sina, zdir);
            _dir4 = _dir4.WithXYZ(dirxyz);
            _pt4 = _pt4.WithXYZ(_pt4.XYZ() + dirxyz);
            if (_nbSol == 2)
            {
                _pt1 = _pt3;
                _pt2 = _pt4;
                _dir1 = _dir3;
                _dir2 = _dir4;
            }
        }

        _done = true;
    }

    /// <summary>The silhouette of a cone for the eye <paramref name="eye"/>.</summary>
    public void Perform(in Cone c, in Point3D eye)
    {
        _done = false;

        double tgtalpha = Math.Tan(c.SemiAngle);

        Xyz apexeye = eye.XYZ();
        apexeye.Subtract(c.Apex.XYZ());

        double coefcos = apexeye.Dot(c.Position.XDirection.XYZ());
        double coefsin = apexeye.Dot(c.Position.YDirection.XYZ());
        double coefcst = apexeye.Dot(c.Axis.Direction.XYZ()) * tgtalpha;

        double norm1 = coefcos * coefcos + coefsin * coefsin;
        double norm2 = Math.Sqrt(coefcos * coefcos + coefsin * coefsin);

        if (Math.Abs(coefcst) < norm2)
        {
            _typL = global::Kerf.Geometry.CurveKind.Line;
            _nbSol = 2;
            _pt1 = c.Apex;
            _pt2 = _pt1;

            _prm = Math.Sqrt(norm1 - coefcst * coefcst);
            double cost0, sint0, cost1, sint1;

            cost0 = (coefcos * coefcst - coefsin * _prm) / norm1;
            cost1 = (coefcos * coefcst + coefsin * _prm) / norm1;

            sint0 = (coefcos * _prm + coefsin * coefcst) / norm1;
            sint1 = (-coefcos * _prm + coefsin * coefcst) / norm1;

            Xyz xdir = c.Position.XDirection.XYZ();
            Xyz ydir = c.Position.YDirection.XYZ();
            Xyz zdir = c.Axis.Direction.XYZ();
            Xyz dirxyz = new();
            dirxyz.SetLinearForm(cost0, xdir, sint0, ydir, 1.0 / tgtalpha, zdir);
            _dir1 = _dir1.WithXYZ(dirxyz);
            _pt1 = _pt1.WithXYZ(_pt1.XYZ() + dirxyz);
            dirxyz.SetLinearForm(cost1, xdir, sint1, ydir, 1.0 / tgtalpha, zdir);
            _dir2 = _dir2.WithXYZ(dirxyz);
            _pt2 = _pt2.WithXYZ(_pt2.XYZ() + dirxyz);
        }
        else
        {
            _nbSol = 0;
        }
        _done = true;
    }

    public bool IsDone => _done;

    /// <summary>The number of contours; throws StdFail_NotDone before a computation.</summary>
    public int ContourCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _nbSol;
        }
    }

    /// <summary>GeomAbs_Line or GeomAbs_Circle; throws StdFail_NotDone or Standard_DomainError (no contour).</summary>
    public CurveKind TypeContour()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_nbSol == 0)
        {
            throw new DomainErrorException();
        }
        return _typL;
    }

    /// <summary>The circle; throws StdFail_NotDone or Standard_DomainError.</summary>
    public Circle3D Circle()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_typL != global::Kerf.Geometry.CurveKind.Circle || _nbSol == 0)
        {
            throw new DomainErrorException();
        }
        return new Circle3D(new Frame3D(_pt1, _dir1, _dir2), _prm);
    }

    /// <summary>The line <paramref name="index"/>; throws StdFail_NotDone, Standard_DomainError or Standard_OutOfRange.</summary>
    public Line3D Line(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_typL != global::Kerf.Geometry.CurveKind.Line || _nbSol == 0)
        {
            throw new DomainErrorException();
        }
        if (index <= 0 || index > _nbSol)
        {
            throw new OutOfRangeException();
        }
        switch (index)
        {
            case 1:
                return new Line3D(_pt1, _dir1);
            case 2:
                return new Line3D(_pt2, _dir2);
            case 3:
                return new Line3D(_pt3, _dir3);
            case 4:
                return new Line3D(_pt4, _dir4);
        }
        throw new OutOfRangeException("Program error in Contap_ContAna");
    }
}
