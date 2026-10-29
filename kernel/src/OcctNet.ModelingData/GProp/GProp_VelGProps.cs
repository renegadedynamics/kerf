// Port of: src/ModelingData/TKGeomBase/GProp/GProp_VelGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_VelGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Global properties of solids bounded by elementary surfaces (gp cylinders, cones, spheres and tori).</summary>
/// <remarks>A working object: <c>SetLocation</c> and <c>Perform</c> are public, as in OCCT.</remarks>
public class GProp_VelGProps : GProp_GProps
{
    public GProp_VelGProps()
    {
    }

    public void SetLocation(in gp_Pnt VLocation)
    {
        loc = VLocation;
    }

    public GProp_VelGProps(in gp_Cylinder S, double Alpha1, double Alpha2, double Z1, double Z2, in gp_Pnt VLocation)
    {
        SetLocation(VLocation);
        Perform(S, Alpha1, Alpha2, Z1, Z2);
    }

    public GProp_VelGProps(in gp_Cone S, double Alpha1, double Alpha2, double Z1, double Z2, in gp_Pnt VLocation)
    {
        SetLocation(VLocation);
        Perform(S, Alpha1, Alpha2, Z1, Z2);
    }

    public GProp_VelGProps(in gp_Sphere S, double Teta1, double Teta2, double Alpha1, double Alpha2, in gp_Pnt VLocation)
    {
        SetLocation(VLocation);
        Perform(S, Teta1, Teta2, Alpha1, Alpha2);
    }

    public GProp_VelGProps(in gp_Torus S, double Teta1, double Teta2, double Alpha1, double Alpha2, in gp_Pnt VLocation)
    {
        SetLocation(VLocation);
        Perform(S, Teta1, Teta2, Alpha1, Alpha2);
    }

    public void Perform(in gp_Cylinder S, double Alpha1, double Alpha2, double Z1, double Z2)
    {
        S.Location().Coord(out double X0, out double Y0, out double Z0);
        double Rayon = S.Radius();
        S.Position().XDirection().Coord(out double Xa1, out double Ya1, out double Za1);
        S.Position().YDirection().Coord(out double Xa2, out double Ya2, out double Za2);
        S.Position().Direction().Coord(out double Xa3, out double Ya3, out double Za3);
        dim         = Rayon * Rayon * (Z2 - Z1) / 2.0;
        double SA2  = Math.Sin(Alpha2);
        double SA1  = Math.Sin(Alpha1);
        double CA2  = Math.Cos(Alpha2);
        double CA1  = Math.Cos(Alpha1);
        double Dsin = SA2 - SA1;
        double Dcos = CA1 - CA2;
        double Coef = Rayon / (Alpha2 - Alpha1);

        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        g.SetCoord(X0 + (Coef * (Xa1 * Dsin + Xa2 * Dcos)) + (Xa3 * (Z2 + Z1) / 2.0),
                              Y0 + (Coef * (Ya1 * Dsin + Ya2 * Dcos)) + (Ya3 * (Z2 + Z1) / 2.0),
                              Z0 + (Coef * (Za1 * Dsin + Za2 * Dcos)) + (Za3 * (Z2 + Z1) / 2.0));

        double ICn2  = dim / 2.0 * (Alpha2 - Alpha1 + SA2 * CA2 - SA1 * CA1);
        double ISn2  = dim / 2.0 * (Alpha2 - Alpha1 - SA2 * CA2 + SA1 * CA1);
        double IZ2   = dim * (Alpha2 - Alpha1) * (Z2 * Z2 + Z1 * Z2 + Z1 * Z1);
        double ICnSn = dim * (CA2 * CA2 - CA1 * CA1) / 2.0;
        double ICnz  = dim * (Z2 + Z1) / 2.0 * Dsin;
        double ISnz  = dim * (Z2 + Z1) / 2.0 * Dcos;
        dim          = (Alpha2 - Alpha1) * dim;

        var Dm = new math_Matrix(1, 3, 1, 3);

        Dm[1, 1] = Rayon * Rayon * ISn2 + IZ2;
        Dm[2, 2] = Rayon * Rayon * ICn2 + IZ2;
        // OCCT: the axial moment of a cylinder is Rayon^2 dim / 2.
        Dm[3, 3] = Rayon * Rayon * dim;
        Dm[1, 2] = Dm[2, 1] = -Rayon * Rayon * ICnSn;
        Dm[1, 3] = Dm[3, 1] = -Rayon * ICnz;
        Dm[3, 2] = Dm[2, 3] = -Rayon * ISnz;

        var Passage = new math_Matrix(1, 3, 1, 3);
        Passage[1, 1] = Xa1;
        Passage[1, 2] = Xa2;
        Passage[1, 3] = Xa3;
        Passage[2, 1] = Ya1;
        Passage[2, 2] = Ya2;
        Passage[2, 3] = Ya3;
        Passage[3, 1] = Za1;
        Passage[3, 2] = Za2;
        Passage[3, 3] = Za3;

        var J = new math_Jacobi(Dm);
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
        inertia = new gp_Mat(new gp_XYZ(V1[1], V2[1], V3[1]), new gp_XYZ(V1[2], V2[2], V3[2]), new gp_XYZ(V1[3], V2[3], V3[3]));
        GProp.HOperator(g, loc, dim, out gp_Mat Hop);
        inertia = inertia + Hop;
    }

    public void Perform(in gp_Cone S, double Alpha1, double Alpha2, double Z1, double Z2)
    {
        S.Location().Coord(out double X0, out double Y0, out double Z0);
        S.Position().XDirection().Coord(out double Xa1, out double Ya1, out double Za1);
        S.Position().YDirection().Coord(out double Xa2, out double Ya2, out double Za2);
        S.Position().Direction().Coord(out double Xa3, out double Ya3, out double Za3);
        double t     = S.SemiAngle();
        double Cnt   = Math.Cos(t);
        double Snt   = Math.Sin(t);
        double R     = S.RefRadius();
        double Sn2   = Math.Sin(Alpha2);
        double Sn1   = Math.Sin(Alpha1);
        double Cn2   = Math.Cos(Alpha2);
        double Cn1   = Math.Cos(Alpha1);
        double ZZ    = (Z2 - Z1) * (Z2 - Z1) * Cnt * Snt;
        double Auxi1 = 2 * R + (Z2 + Z1) * Snt;

        // OCCT: not the frustum's volume (Alpha2 - Alpha1) (Z2 - Z1) Cnt (R1^2 + R1 R2 + R2^2) / 6.
        dim = ZZ * (Alpha2 - Alpha1) * Auxi1 / 2.0;

        double R1    = R + Z1 * Snt;
        double R2    = R + Z2 * Snt;
        double Coef0 = (R1 * R1 + R1 * R2 + R2 * R2);
        double Iz    = Cnt * (R * (Z2 + Z1) + 2 * Snt * (Z1 * Z1 + Z1 * Z2 + Z2 * Z2) / 3.0) / Auxi1;
        double Ix    = Coef0 * (Sn2 - Sn1) / (Alpha2 - Alpha1) / Auxi1;
        double Iy    = Coef0 * (Cn1 - Cn2) / (Alpha2 - Alpha1) / Auxi1;

        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        g.SetCoord(X0 + Xa1 * Ix + Xa2 * Iy + Xa3 * Iz,
                              Y0 + Ya1 * Ix + Ya2 * Iy + Ya3 * Iz,
                              Z0 + Za1 * Ix + Za2 * Iy + Za3 * Iz);

        double IR2  = ZZ * (R2 * R2 * R2 + R2 * R2 * R1 + R1 * R1 * R2 + R1 * R1 * R1) / 4.0;
        double ICn2 = IR2 * (Alpha2 - Alpha1 + Cn2 * Sn2 - Cn1 * Sn1) / 2.0;
        // OCCT: the sign of the sin*cos terms should be '-' (a copy of ICn2).
        double ISn2 = IR2 * (Alpha2 - Alpha1 + Cn2 * Sn2 - Cn1 * Sn1) / 2.0;
        double IZ2  = ZZ * Cnt * Cnt * (Alpha2 - Alpha1)
                                  * (Z1 * Z1 * (R / 3 + Z1 * Snt / 4) + Z2 * Z2 * (R / 3 + Z2 * Snt / 4)
                                        + Z1 * Z2 * (R / 3 + Z1 * Snt / 4 + Z2 * Snt / 4));
        double ICnSn = IR2 * (Cn2 * Cn2 - Cn1 * Cn1);
        double ICnz  = (Z1 + Z2) * ZZ * Coef0 * (Sn2 - Sn1) / 3;
        double ISnz  = (Z1 + Z2) * ZZ * Coef0 * (Cn1 - Cn2) / 3;

        var Dm = new math_Matrix(1, 3, 1, 3);
        Dm[1, 1] = ISn2 + IZ2;
        Dm[2, 2] = ICn2 + IZ2;
        Dm[3, 3] = IR2 * (Alpha2 - Alpha1);
        Dm[1, 2] = Dm[2, 1] = -ICnSn;
        Dm[1, 3] = Dm[3, 1] = -ICnz;
        Dm[3, 2] = Dm[2, 3] = -ISnz;

        var Passage = new math_Matrix(1, 3, 1, 3);
        Passage[1, 1] = Xa1;
        Passage[1, 2] = Xa2;
        Passage[1, 3] = Xa3;
        Passage[2, 1] = Ya1;
        Passage[2, 2] = Ya2;
        Passage[2, 3] = Ya3;
        Passage[3, 1] = Za1;
        Passage[3, 2] = Za2;
        Passage[3, 3] = Za3;

        var J = new math_Jacobi(Dm);
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
        inertia = new gp_Mat(new gp_XYZ(V1[1], V2[1], V3[1]), new gp_XYZ(V1[2], V2[2], V3[2]), new gp_XYZ(V1[3], V2[3], V3[3]));
        GProp.HOperator(g, loc, dim, out gp_Mat Hop);
        inertia = inertia + Hop;
    }

    public void Perform(in gp_Sphere S, double Teta1, double Teta2, double Alpha1, double Alpha2)
    {
        S.Location().Coord(out double X0, out double Y0, out double Z0);
        S.Position().XDirection().Coord(out double Xa1, out double Ya1, out double Za1);
        S.Position().YDirection().Coord(out double Xa2, out double Ya2, out double Za2);
        S.Position().Direction().Coord(out double Xa3, out double Ya3, out double Za3);
        double R    = S.Radius();
        double Cnt1 = Math.Cos(Teta1);
        double Snt1 = Math.Sin(Teta1);
        double Cnt2 = Math.Cos(Teta2);
        double Snt2 = Math.Sin(Teta2);
        double Cnf1 = Math.Cos(Alpha1);
        double Snf1 = Math.Sin(Alpha1);
        double Cnf2 = Math.Cos(Alpha2);
        double Snf2 = Math.Sin(Alpha2);

        dim = (Teta2 - Teta1) * R * R * R * (Snf2 - Snf1) / 3.0;

        double Ix = R * (Snt2 - Snt1) / (Teta2 - Teta1) * (Alpha2 - Alpha1 + Snf2 * Cnf2 - Snf1 * Cnf1)
                                / (Snf2 - Snf1) / 2.0;
        double Iy = R * (Cnt1 - Cnt2) / (Teta2 - Teta1) * (Alpha2 - Alpha1 + Snf2 * Cnf2 - Snf1 * Cnf1)
                                / (Snf2 - Snf1) / 2.0;
        double Iz = R * (Snf2 + Snf1) / 2.0;
        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        g.SetCoord(X0 + Ix * Xa1 + Iy * Xa2 + Iz * Xa3,
                              Y0 + Ix * Ya1 + Iy * Ya2 + Iz * Ya3,
                              Z0 + Ix * Za1 + Iy * Za2 + Iz * Za3);

        double IR2   = (Cnf2 * Snf2 * (Cnf2 + 1.0) - Cnf1 * Snf1 * (Cnf1 + 1.0) + Alpha2 - Alpha1) / 9.0;
        double ICn2  = (Teta2 - Teta1 + Cnt2 * Snt2 - Cnt1 * Snt1) * IR2 / 2.0;
        double ISn2  = (Teta2 - Teta1 - Cnt2 * Snt2 + Cnt1 * Snt1) * IR2 / 2.0;
        double ICnSn = (Snt2 * Snt2 - Snt1 * Snt1) * IR2 / 2.0;
        double IZ2   = (Teta2 - Teta1) * (Snf2 * Snf2 * Snf2 - Snf1 * Snf1 * Snf1) / 9.0;
        double ICnz  = (Snt2 - Snt1) * (Cnf1 * Cnf1 * Cnf1 - Cnf2 * Cnf2 * Cnf2) / 9.0;
        double ISnz  = (Cnt1 - Cnt2) * (Cnf1 * Cnf1 * Cnf1 - Cnf2 * Cnf2 * Cnf2) / 9.0;

        var Dm = new math_Matrix(1, 3, 1, 3);
        Dm[1, 1] = ISn2 + IZ2;
        Dm[2, 2] = ICn2 + IZ2;
        Dm[3, 3] = IR2 * (Teta2 - Teta1);
        Dm[1, 2] = Dm[2, 1] = -ICnSn;
        Dm[1, 3] = Dm[3, 1] = -ICnz;
        Dm[3, 2] = Dm[2, 3] = -ISnz;

        var Passage = new math_Matrix(1, 3, 1, 3);
        Passage[1, 1] = Xa1;
        Passage[1, 2] = Xa2;
        Passage[1, 3] = Xa3;
        Passage[2, 1] = Ya1;
        Passage[2, 2] = Ya2;
        Passage[2, 3] = Ya3;
        Passage[3, 1] = Za1;
        Passage[3, 2] = Za2;
        Passage[3, 3] = Za3;

        var J = new math_Jacobi(Dm);
        R = R * R * R * R * R;
        var V1 = new math_Vector(1, 3);
        var V2 = new math_Vector(1, 3);
        var V3 = new math_Vector(1, 3);
        J.Vector(1, V1);
        V1.Multiply(Passage, V1);
        V1.Multiply(R * J.Value(1));
        J.Vector(2, V2);
        V2.Multiply(Passage, V2);
        V2.Multiply(R * J.Value(2));
        J.Vector(3, V3);
        V3.Multiply(Passage, V3);
        V3.Multiply(R * J.Value(3));

        // OCCT: P V L (each eigenvector scaled by its value) is not the matrix P V L V^T P^T, and math_Jacobi sorts
        // the values, so the result is permuted or wrong except in special cases (docs/occt-issues.md). The
        // V.Multiply(Passage, V) above also reads V while overwriting it, as in OCCT.
        inertia = new gp_Mat(new gp_XYZ(V1[1], V2[1], V3[1]), new gp_XYZ(V1[2], V2[2], V3[2]), new gp_XYZ(V1[3], V2[3], V3[3]));
        GProp.HOperator(g, loc, dim, out gp_Mat Hop);
        inertia = inertia + Hop;
    }

    public void Perform(in gp_Torus S, double Teta1, double Teta2, double Alpha1, double Alpha2)
    {
        S.Location().Coord(out double X0, out double Y0, out double Z0);
        S.Position().XDirection().Coord(out double Xa1, out double Ya1, out double Za1);
        S.Position().YDirection().Coord(out double Xa2, out double Ya2, out double Za2);
        S.Position().Direction().Coord(out double Xa3, out double Ya3, out double Za3);
        double RMax = S.MajorRadius();
        double Rmin = S.MinorRadius();
        double Cnt1 = Math.Cos(Teta1);
        double Snt1 = Math.Sin(Teta1);
        // OCCT: Teta2, not Alpha2.
        double Cnt2 = Math.Cos(Alpha2);
        double Snt2 = Math.Sin(Alpha2);
        double Cnf1 = Math.Cos(Alpha1);
        double Snf1 = Math.Sin(Alpha1);
        double Cnf2 = Math.Cos(Alpha2);
        double Snf2 = Math.Sin(Alpha2);

        dim       = RMax * Rmin * Rmin * (Teta2 - Teta1) * (Alpha2 - Alpha1) / 2.0;
        double Ix = (Snt2 - Snt1) / (Teta2 - Teta1) * (Rmin * (Snf2 - Snf1) / (Alpha2 - Alpha1) + RMax);
        double Iy = (Cnt1 - Cnt2) / (Teta2 - Teta1) * (Rmin * (Snf2 - Snf1) / (Alpha2 - Alpha1) + RMax);
        double Iz = Rmin * (Cnf1 - Cnf2) / (Alpha2 - Alpha1);

        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        g.SetCoord(X0 + Ix * Xa1 + Iy * Xa2 + Iz * Xa3,
                              Y0 + Ix * Ya1 + Iy * Ya2 + Iz * Ya3,
                              Z0 + Ix * Za1 + Iy * Za2 + Iz * Za3);

        double IR2 = RMax * RMax + Rmin * Rmin / 2.0 + 2.0 * RMax * Rmin * (Snf2 - Snf1)
                                  + Rmin * Rmin / 2.0 * (Snf2 * Cnf2 - Snf1 * Cnf1);
        double ICn2  = IR2 * (Teta2 - Teta1 + Snt2 * Cnt2 - Snt1 * Cnt1) / 2.0;
        double ISn2  = IR2 * (Teta2 - Teta1 - Snt2 * Cnt2 + Snt1 * Cnt1) / 2.0;
        double ICnSn = IR2 * (Snt2 * Snt2 - Snt1 * Snt1) / 2.0;
        double IZ2   = (Teta2 - Teta1) * Rmin * Rmin * (Alpha2 - Alpha1 - Snf2 * Cnf2 + Snf1 * Cnf1) / 2.0;
        double ICnz  = Rmin * (Snt2 - Snt1) * (Cnf1 - Cnf2) * (RMax + Rmin * (Cnf1 + Cnf2) / 2.0);
        double ISnz  = Rmin * (Cnt2 - Cnt1) * (Cnf1 - Cnf2) * (RMax + Rmin * (Cnf1 + Cnf2) / 2.0);

        var Dm = new math_Matrix(1, 3, 1, 3);
        Dm[1, 1] = ISn2 + IZ2;
        Dm[2, 2] = ICn2 + IZ2;
        Dm[3, 3] = IR2 * (Teta2 - Teta1);
        Dm[1, 2] = Dm[2, 1] = -ICnSn;
        Dm[1, 3] = Dm[3, 1] = -ICnz;
        Dm[3, 2] = Dm[2, 3] = -ISnz;

        var Passage = new math_Matrix(1, 3, 1, 3);
        Passage[1, 1] = Xa1;
        Passage[1, 2] = Xa2;
        Passage[1, 3] = Xa3;
        Passage[2, 1] = Ya1;
        Passage[2, 2] = Ya2;
        Passage[2, 3] = Ya3;
        Passage[3, 1] = Za1;
        Passage[3, 2] = Za2;
        Passage[3, 3] = Za3;

        var J = new math_Jacobi(Dm);
        RMax = RMax * Rmin * Rmin / 2.0;
        var V1 = new math_Vector(1, 3);
        var V2 = new math_Vector(1, 3);
        var V3 = new math_Vector(1, 3);
        J.Vector(1, V1);
        V1.Multiply(Passage, V1);
        V1.Multiply(RMax * J.Value(1));
        J.Vector(2, V2);
        V2.Multiply(Passage, V2);
        V2.Multiply(RMax * J.Value(2));
        J.Vector(3, V3);
        V3.Multiply(Passage, V3);
        V3.Multiply(RMax * J.Value(3));

        // OCCT: P V L (each eigenvector scaled by its value) is not the matrix P V L V^T P^T, and math_Jacobi sorts
        // the values, so the result is permuted or wrong except in special cases (docs/occt-issues.md). The
        // V.Multiply(Passage, V) above also reads V while overwriting it, as in OCCT.
        inertia = new gp_Mat(new gp_XYZ(V1[1], V2[1], V3[1]), new gp_XYZ(V1[2], V2[2], V3[2]), new gp_XYZ(V1[3], V2[3], V3[3]));
        GProp.HOperator(g, loc, dim, out gp_Mat Hop);
        inertia = inertia + Hop;
    }
}
