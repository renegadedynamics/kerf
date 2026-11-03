// Port of: src/ModelingData/TKGeomBase/GProp/GProp_CelGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_CelGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>Global properties of bounded elementary curves in 3D space (gp lines and circles).</summary>
/// <remarks>A working object: <c>SetLocation</c> and <c>Perform</c> are public, as in OCCT.</remarks>
public class CelGProps : Properties
{
    public CelGProps()
    {
    }

    public void SetLocation(in Point3D cLocation)
    {
        _loc = cLocation;
    }

    public void Perform(in Circle3D c, double u1, double u2)
    {
        c.Location().Coord(out double x0, out double y0, out double z0);
        c.XAxis().Direction().Coord(out double xa1, out double ya1, out double za1);
        c.YAxis().Direction().Coord(out double xa2, out double ya2, out double za2);
        c.Axis().Direction().Coord(out double xa3, out double ya3, out double za3);
        double ray = c.Radius();

        _dim         = ray * Math.Abs(u2 - u1);
        double xloc = ray * (Math.Sin(u2) - Math.Sin(u1)) / (u2 - u1);
        double yloc = ray * (Math.Cos(u1) - Math.Cos(u2)) / (u2 - u1);

        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        _g.SetCoord(xloc * xa1 + yloc * xa2 + x0, xloc * ya1 + yloc * ya2 + y0, z0);

        var dm = new Matrix(1, 3, 1, 3);
        dm[1, 1] = ray * ray * ray * (u2 / 2 - u1 / 2 - Math.Sin(2 * u2) / 4 + Math.Sin(2 * u1) / 4);
        dm[2, 2] = ray * ray * ray * (u2 / 2 - u1 / 2 + Math.Sin(2 * u2) / 4 - Math.Sin(2 * u1) / 4);
        dm[3, 3] = ray * ray * _dim;
        dm[2, 1] = dm[1, 2] = -ray * ray * ray * (Math.Cos(2 * u1) / 4 - Math.Cos(2 * u2) / 4);
        dm[3, 1] = dm[1, 3] = 0.0;
        dm[3, 2] = dm[2, 3] = 0.0;
        var passage = new Matrix(1, 3, 1, 3);
        passage[1, 1] = xa1;
        passage[1, 2] = xa2;
        passage[1, 3] = xa3;
        passage[2, 1] = ya1;
        passage[2, 2] = ya2;
        passage[2, 3] = ya3;
        passage[3, 1] = za1;
        passage[3, 2] = za2;
        passage[3, 3] = za3;

        var j = new Jacobi(dm);
        var v1 = new math_Vector(1, 3);
        var v2 = new math_Vector(1, 3);
        var v3 = new math_Vector(1, 3);
        j.Vector(1, v1);
        v1.Multiply(passage, v1);
        v1.Multiply(j.Value(1));
        j.Vector(2, v2);
        v2.Multiply(passage, v2);
        v2.Multiply(j.Value(2));
        j.Vector(3, v3);
        v3.Multiply(passage, v3);
        v3.Multiply(j.Value(3));

        // OCCT: P V L (each eigenvector scaled by its value) is not the matrix P V L V^T P^T, and math_Jacobi sorts
        // the values, so the result is permuted or wrong except in special cases (docs/occt-issues.md). The
        // V.Multiply(Passage, V) above also reads V while overwriting it, as in OCCT.
        _inertia = new Matrix3x3(new Xyz(v1[1], v2[1], v3[1]), new Xyz(v1[2], v2[2], v3[2]), new Xyz(v1[3], v2[3], v3[3]));

        global::Kerf.Engine.Measurement.GProp.HOperator(_g, _loc, _dim, out Matrix3x3 hop);
        _inertia = _inertia + hop;
    }

    public void Perform(in Line3D c, double u1, double u2)
    {
        Axis3D pos = c.Position();
        Point3D p1  = global::Kerf.Engine.Geometry.ElCLib.LineValue(u1, pos);
        _dim        = Math.Abs(u2 - u1);
        Point3D p2  = global::Kerf.Engine.Geometry.ElCLib.LineValue(u2, pos);
        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        _g.SetCoord((p1.X() + p2.X()) / 2.0, (p1.Y() + p2.Y()) / 2.0, (p1.Z() + p2.Z()) / 2.0);
        pos.Direction().Coord(out double vx, out double vy, out double vz);
        pos.Location().Coord(out double x0, out double y0, out double z0);
        double alfa1 = (vz * vz + vy * vy) / 3.0;
        double alfa2 = vy * y0 + vz * z0;
        double alfa3 = y0 * y0 + z0 * z0;
        double ixx =
            (u2 * (u2 * (u2 * alfa1 + alfa2) + alfa3)) - (u1 * (u1 * (u1 * alfa1 + alfa2) + alfa3));
        alfa1 = (vz * vz + vx * vx) / 3.0;
        alfa2 = vx * x0 + vz * z0;
        alfa3 = x0 * x0 + z0 * z0;
        double iyy =
            (u2 * (u2 * (u2 * alfa1 + alfa2) + alfa3)) - (u1 * (u1 * (u1 * alfa1 + alfa2) + alfa3));
        alfa1 = (vy * vy + vx * vx) / 3.0;
        // OCCT: Izz's u^2 and u coefficients should be Vx X0 + Vy Y0 and X0^2 + Y0^2 (copied from Ixx).
        alfa2 = vy * y0 + vz * z0;
        alfa3 = y0 * y0 + z0 * z0;
        double izz =
            (u2 * (u2 * (u2 * alfa1 + alfa2) + alfa3)) - (u1 * (u1 * (u1 * alfa1 + alfa2) + alfa3));
        alfa1 = (vy * vx) / 3.0;
        alfa2 = (vy * x0 + vx * y0) / 2.0;
        alfa3 = y0 * x0;
        double ixy =
            (u2 * (u2 * (u2 * alfa1 + alfa2) + alfa3)) - (u1 * (u1 * (u1 * alfa1 + alfa2) + alfa3));
        alfa1 = (vz * vx) / 3.0;
        alfa2 = (vz * x0 + vx * z0) / 2;
        alfa3 = z0 * x0;
        double ixz =
            (u2 * (u2 * (u2 * alfa1 + alfa2) + alfa3)) - (u1 * (u1 * (u1 * alfa1 + alfa2) + alfa3));
        alfa1 = (vy * vz) / 3.0;
        alfa2 = (vy * z0 + vz * y0) / 2.0;
        alfa3 = y0 * z0;
        double iyz =
            (u2 * (u2 * (u2 * alfa1 + alfa2) + alfa3)) - (u1 * (u1 * (u1 * alfa1 + alfa2) + alfa3));

        _inertia = new Matrix3x3(new Xyz(ixx, -ixy, -ixz), new Xyz(-ixy, iyy, -iyz), new Xyz(-ixz, -iyz, izz));
    }

    public CelGProps(in Circle3D c, in Point3D cLocation)
    {
        SetLocation(cLocation);
        Perform(c, 0.0, 2.0 * M_PI);
    }

    public CelGProps(in Circle3D c, double u1, double u2, in Point3D cLocation)
    {
        SetLocation(cLocation);
        Perform(c, u1, u2);
    }

    public CelGProps(in Line3D c, double u1, double u2, in Point3D cLocation)
    {
        SetLocation(cLocation);
        Perform(c, u1, u2);
    }
}
