// Port of: src/ModelingData/TKGeomBase/GProp/GProp_CelGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_CelGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>Global properties of bounded elementary curves in 3D space (gp lines and circles).</summary>
/// <remarks>A working object: <c>SetLocation</c> and <c>Perform</c> are public, as in OCCT.</remarks>
public class GProp_CelGProps : GProp_GProps
{
    public GProp_CelGProps()
    {
    }

    public void SetLocation(in Point3D CLocation)
    {
        loc = CLocation;
    }

    public void Perform(in Circle3D C, double U1, double U2)
    {
        C.Location().Coord(out double X0, out double Y0, out double Z0);
        C.XAxis().Direction().Coord(out double Xa1, out double Ya1, out double Za1);
        C.YAxis().Direction().Coord(out double Xa2, out double Ya2, out double Za2);
        C.Axis().Direction().Coord(out double Xa3, out double Ya3, out double Za3);
        double Ray = C.Radius();

        dim         = Ray * Math.Abs(U2 - U1);
        double xloc = Ray * (Math.Sin(U2) - Math.Sin(U1)) / (U2 - U1);
        double yloc = Ray * (Math.Cos(U1) - Math.Cos(U2)) / (U2 - U1);

        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        g.SetCoord(xloc * Xa1 + yloc * Xa2 + X0, xloc * Ya1 + yloc * Ya2 + Y0, Z0);

        var Dm = new Matrix(1, 3, 1, 3);
        Dm[1, 1] = Ray * Ray * Ray * (U2 / 2 - U1 / 2 - Math.Sin(2 * U2) / 4 + Math.Sin(2 * U1) / 4);
        Dm[2, 2] = Ray * Ray * Ray * (U2 / 2 - U1 / 2 + Math.Sin(2 * U2) / 4 - Math.Sin(2 * U1) / 4);
        Dm[3, 3] = Ray * Ray * dim;
        Dm[2, 1] = Dm[1, 2] = -Ray * Ray * Ray * (Math.Cos(2 * U1) / 4 - Math.Cos(2 * U2) / 4);
        Dm[3, 1] = Dm[1, 3] = 0.0;
        Dm[3, 2] = Dm[2, 3] = 0.0;
        var Passage = new Matrix(1, 3, 1, 3);
        Passage[1, 1] = Xa1;
        Passage[1, 2] = Xa2;
        Passage[1, 3] = Xa3;
        Passage[2, 1] = Ya1;
        Passage[2, 2] = Ya2;
        Passage[2, 3] = Ya3;
        Passage[3, 1] = Za1;
        Passage[3, 2] = Za2;
        Passage[3, 3] = Za3;

        var J = new Jacobi(Dm);
        var V1 = new math_Vector(1, 3);
        var V2 = new math_Vector(1, 3);
        var V3 = new math_Vector(1, 3);
        J.Vector(1, V1);
        V1.Multiply(Passage, V1);
        V1.Multiply(J.Value(1));
        J.Vector(2, V2);
        V2.Multiply(Passage, V2);
        V2.Multiply(J.Value(2));
        J.Vector(3, V3);
        V3.Multiply(Passage, V3);
        V3.Multiply(J.Value(3));

        // OCCT: P V L (each eigenvector scaled by its value) is not the matrix P V L V^T P^T, and math_Jacobi sorts
        // the values, so the result is permuted or wrong except in special cases (docs/occt-issues.md). The
        // V.Multiply(Passage, V) above also reads V while overwriting it, as in OCCT.
        inertia = new Matrix3x3(new Xyz(V1[1], V2[1], V3[1]), new Xyz(V1[2], V2[2], V3[2]), new Xyz(V1[3], V2[3], V3[3]));

        GProp.HOperator(g, loc, dim, out Matrix3x3 Hop);
        inertia = inertia + Hop;
    }

    public void Perform(in Line3D C, double U1, double U2)
    {
        Axis3D Pos = C.Position();
        Point3D P1  = global::Kerf.Engine.Geometry.ElCLib.LineValue(U1, Pos);
        dim        = Math.Abs(U2 - U1);
        Point3D P2  = global::Kerf.Engine.Geometry.ElCLib.LineValue(U2, Pos);
        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        g.SetCoord((P1.X() + P2.X()) / 2.0, (P1.Y() + P2.Y()) / 2.0, (P1.Z() + P2.Z()) / 2.0);
        Pos.Direction().Coord(out double Vx, out double Vy, out double Vz);
        Pos.Location().Coord(out double X0, out double Y0, out double Z0);
        double alfa1 = (Vz * Vz + Vy * Vy) / 3.0;
        double alfa2 = Vy * Y0 + Vz * Z0;
        double alfa3 = Y0 * Y0 + Z0 * Z0;
        double Ixx =
            (U2 * (U2 * (U2 * alfa1 + alfa2) + alfa3)) - (U1 * (U1 * (U1 * alfa1 + alfa2) + alfa3));
        alfa1 = (Vz * Vz + Vx * Vx) / 3.0;
        alfa2 = Vx * X0 + Vz * Z0;
        alfa3 = X0 * X0 + Z0 * Z0;
        double Iyy =
            (U2 * (U2 * (U2 * alfa1 + alfa2) + alfa3)) - (U1 * (U1 * (U1 * alfa1 + alfa2) + alfa3));
        alfa1 = (Vy * Vy + Vx * Vx) / 3.0;
        // OCCT: Izz's u^2 and u coefficients should be Vx X0 + Vy Y0 and X0^2 + Y0^2 (copied from Ixx).
        alfa2 = Vy * Y0 + Vz * Z0;
        alfa3 = Y0 * Y0 + Z0 * Z0;
        double Izz =
            (U2 * (U2 * (U2 * alfa1 + alfa2) + alfa3)) - (U1 * (U1 * (U1 * alfa1 + alfa2) + alfa3));
        alfa1 = (Vy * Vx) / 3.0;
        alfa2 = (Vy * X0 + Vx * Y0) / 2.0;
        alfa3 = Y0 * X0;
        double Ixy =
            (U2 * (U2 * (U2 * alfa1 + alfa2) + alfa3)) - (U1 * (U1 * (U1 * alfa1 + alfa2) + alfa3));
        alfa1 = (Vz * Vx) / 3.0;
        alfa2 = (Vz * X0 + Vx * Z0) / 2;
        alfa3 = Z0 * X0;
        double Ixz =
            (U2 * (U2 * (U2 * alfa1 + alfa2) + alfa3)) - (U1 * (U1 * (U1 * alfa1 + alfa2) + alfa3));
        alfa1 = (Vy * Vz) / 3.0;
        alfa2 = (Vy * Z0 + Vz * Y0) / 2.0;
        alfa3 = Y0 * Z0;
        double Iyz =
            (U2 * (U2 * (U2 * alfa1 + alfa2) + alfa3)) - (U1 * (U1 * (U1 * alfa1 + alfa2) + alfa3));

        inertia = new Matrix3x3(new Xyz(Ixx, -Ixy, -Ixz), new Xyz(-Ixy, Iyy, -Iyz), new Xyz(-Ixz, -Iyz, Izz));
    }

    public GProp_CelGProps(in Circle3D C, in Point3D CLocation)
    {
        SetLocation(CLocation);
        Perform(C, 0.0, 2.0 * M_PI);
    }

    public GProp_CelGProps(in Circle3D C, double U1, double U2, in Point3D CLocation)
    {
        SetLocation(CLocation);
        Perform(C, U1, U2);
    }

    public GProp_CelGProps(in Line3D C, double U1, double U2, in Point3D CLocation)
    {
        SetLocation(CLocation);
        Perform(C, U1, U2);
    }
}
