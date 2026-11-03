// Port of: src/ModelingData/TKGeomBase/GProp/GProp_SelGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_SelGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>Global properties of bounded elementary surfaces in 3D space (gp cylinders, cones, spheres and tori).</summary>
/// <remarks>A working object: <c>SetLocation</c> and <c>Perform</c> are public, as in OCCT.</remarks>
public class SelGProps : Properties
{
    public SelGProps()
    {
    }

    public void SetLocation(in Point3D sLocation)
    {
        _loc = sLocation;
    }

    public void Perform(in Cylinder s, double alpha1, double alpha2, double z1, double z2)
    {
        s.Location.Coord(out double x0, out double y0, out double z0);
        double r = s.Radius;
        s.Position.XDirection.Coord(out double xa1, out double ya1, out double za1);
        s.Position.YDirection.Coord(out double xa2, out double ya2, out double za2);
        s.Position.Direction.Coord(out double xa3, out double ya3, out double za3);
        _dim        = r * (z2 - z1) * (alpha2 - alpha1);
        double sa2 = Math.Sin(alpha2);
        double sa1 = Math.Sin(alpha1);
        double ca2 = Math.Cos(alpha2);
        double ca1 = Math.Cos(alpha1);
        double ix  = r * (sa2 - sa1) / (alpha2 - alpha1);
        double iy  = r * (ca1 - ca2) / (alpha2 - alpha1);
        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        _g = _g.WithCoord(x0 + ix * xa1 + iy * xa2 + xa3 * (z2 + z1) / 2.0,
                              y0 + ix * ya1 + iy * ya2 + ya3 * (z2 + z1) / 2.0,
                              z0 + ix * za1 + iy * za2 + za3 * (z2 + z1) / 2.0);
        double iCn2  = r * r * (alpha2 - alpha1 + sa2 * ca2 - sa1 * ca1) / 2.0;
        double iSn2  = r * r * (alpha2 - alpha1 - sa2 * ca2 + sa1 * ca1) / 2.0;
        double iz2   = (alpha2 - alpha1) * (z2 * z2 + z2 * z1 + z1 * z1) / 3.0;
        double iCnSn = r * r * (sa2 * sa2 - sa1 * sa1) / 2.0;
        double iCnz  = (z2 + z1) * (sa2 - sa1) / 2.0;
        double iSnz  = (z2 + z1) * (ca1 - ca2) / 2.0;

        var dm = new Matrix(1, 3, 1, 3);

        dm[1, 1] = iSn2 + iz2;
        dm[2, 2] = iCn2 + iz2;
        dm[3, 3] = alpha2 - alpha1;
        dm[1, 2] = dm[2, 1] = -iCnSn;
        dm[1, 3] = dm[3, 1] = -iCnz;
        dm[3, 2] = dm[2, 3] = -iSnz;

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
        r = r * (z2 - z1);
        var v1 = new math_Vector(1, 3);
        var v2 = new math_Vector(1, 3);
        var v3 = new math_Vector(1, 3);
        j.Vector(1, v1);
        v1.Multiply(passage, v1);
        v1.Multiply(r * j.Value(1));
        j.Vector(2, v2);
        v2.Multiply(passage, v2);
        v2.Multiply(r * j.Value(2));
        j.Vector(3, v3);
        v3.Multiply(passage, v3);
        v3.Multiply(r * j.Value(3));

        // OCCT: P V L (each eigenvector scaled by its value) is not the matrix P V L V^T P^T, and math_Jacobi sorts
        // the values, so the result is permuted or wrong except in special cases (docs/occt-issues.md). The
        // V.Multiply(Passage, V) above also reads V while overwriting it, as in OCCT.
        _inertia = new Matrix3x3(new Xyz(v1[1], v2[1], v3[1]), new Xyz(v1[2], v2[2], v3[2]), new Xyz(v1[3], v2[3], v3[3]));
        global::Kerf.Engine.Measurement.GProp.HOperator(_g, _loc, _dim, out Matrix3x3 hop);
        _inertia = _inertia + hop;
    }

    public void Perform(in Cone s, double alpha1, double alpha2, double z1, double z2)

    {
        s.Location.Coord(out double x0, out double y0, out double z0);
        s.Position.XDirection.Coord(out double xa1, out double ya1, out double za1);
        s.Position.YDirection.Coord(out double xa2, out double ya2, out double za2);
        s.Position.Direction.Coord(out double xa3, out double ya3, out double za3);
        double t   = s.SemiAngle;
        double cnt = Math.Cos(t);
        double snt = Math.Sin(t);
        double r   = s.RefRadius;
        double sn2 = Math.Sin(alpha2);
        double sn1 = Math.Sin(alpha1);
        double cn2 = Math.Cos(alpha2);
        double cn1 = Math.Cos(alpha1);

        double auxi1 = r + (z2 + z1) * snt / 2.0;
        double auxi2 = (z2 * z2 + z1 * z2 + z1 * z1) / 3.0;
        // OCCT: the area is (Alpha2 - Alpha1) (Z2 - Z1) Auxi1 (Z is along the generatrix); the factor Cnt is wrong.
        _dim          = (alpha2 - alpha1) * cnt * (z2 - z1) * auxi1;

        double ix = (r * r + r * (z2 + z1) * snt + snt * auxi2) / auxi1;
        double iy = ix * (cn1 - cn2) / (alpha2 - alpha1);
        ix        = ix * (sn2 - sn1) / (alpha2 - alpha1);
        double iz = cnt * (r * (z2 + z1) / 2.0 + snt * auxi2) / auxi1;

        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        _g = _g.WithCoord(x0 + xa1 * ix + xa2 * iy + xa3 * iz,
                              y0 + ya1 * ix + ya2 * iy + ya3 * iz,
                              z0 + za1 * ix + za2 * iy + za3 * iz);

        double r1   = r + z1 * snt;
        double r2   = r + z2 * snt;
        double zz   = (z2 - z1) * cnt;
        double ir2  = zz * snt * (r1 * r1 * r1 + r1 * r1 * r2 + r1 * r2 * r2 + r2 * r2 * r2) / 4.0;
        double iCn2 = ir2 * (alpha2 - alpha1 + cn2 * sn2 - cn1 * sn1) / 2.0;
        // OCCT: the sign of the sin*cos terms should be '-' (a copy of ICn2).
        double iSn2 = ir2 * (alpha2 - alpha1 + cn2 * sn2 - cn1 * sn1) / 2.0;
        double iz2  = zz * cnt * cnt * (z2 - z1) * (alpha2 - alpha1)
                                  * (r * auxi2 + snt * (z2 * z2 * z2 + z2 * z2 * z1 + z2 * z1 * z1 + z1 * z1 * z1))
                                  / 4.0;
        double iCnSn = ir2 * (cn2 * cn2 - cn1 * cn1);
        double iCnz  = cnt * snt * zz * (r * (z1 + z2) / 2.0 + auxi2) * (sn2 - sn1);
        double iSnz  = cnt * snt * zz * (r * (z1 + z2) / 2.0 + auxi2) * (cn1 - cn2);

        var dm = new Matrix(1, 3, 1, 3);
        dm[1, 1] = iSn2 + iz2;
        dm[2, 2] = iCn2 + iz2;
        dm[3, 3] = ir2 * (alpha2 - alpha1);
        dm[1, 2] = dm[2, 1] = -iCnSn;
        dm[1, 3] = dm[3, 1] = -iCnz;
        dm[3, 2] = dm[2, 3] = -iSnz;

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

    public void Perform(in Sphere s, double teta1, double teta2, double alpha1, double alpha2)
    {
        s.Location.Coord(out double x0, out double y0, out double z0);
        s.Position.XDirection.Coord(out double xa1, out double ya1, out double za1);
        s.Position.YDirection.Coord(out double xa2, out double ya2, out double za2);
        s.Position.Direction.Coord(out double xa3, out double ya3, out double za3);
        double r    = s.Radius;
        double cnt1 = Math.Cos(teta1);
        double snt1 = Math.Sin(teta1);
        double cnt2 = Math.Cos(teta2);
        double snt2 = Math.Sin(teta2);
        double cnf1 = Math.Cos(alpha1);
        double snf1 = Math.Sin(alpha1);
        double cnf2 = Math.Cos(alpha2);
        double snf2 = Math.Sin(alpha2);
        _dim         = r * r * (teta2 - teta1) * (snf2 - snf1);
        double ix   = r * (snt2 - snt1) / (teta2 - teta1) * (alpha2 - alpha1 + snf2 * cnf2 - snf1 * cnf1)
                                / (snf2 - snf1) / 2.0;
        double iy = r * (cnt1 - cnt2) / (teta2 - teta1) * (alpha2 - alpha1 + snf2 * cnf2 - snf1 * cnf1)
                                / (snf2 - snf1) / 2.0;
        double iz = r * (snf2 + snf1) / 2.0;
        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        _g = _g.WithCoord(x0 + ix * xa1 + iy * xa2 + iz * xa3,
                              y0 + ix * ya1 + iy * ya2 + iz * ya3,
                              z0 + ix * za1 + iy * za2 + iz * za3);

        double ir2   = (cnf2 * snf2 * (cnf2 + 1.0) - cnf1 * snf1 * (cnf1 + 1.0) + alpha2 - alpha1) / 3.0;
        double iCn2  = (teta2 - teta1 + cnt2 * snt2 - cnt1 * snt1) * ir2 / 2.0;
        double iSn2  = (teta2 - teta1 - cnt2 * snt2 + cnt1 * snt1) * ir2 / 2.0;
        double iCnSn = (snt2 * snt2 - snt1 * snt1) * ir2 / 2.0;
        double iz2   = (teta2 - teta1) * (snf2 * snf2 * snf2 - snf1 * snf1 * snf1) / 3.0;
        double iCnz  = (snt2 - snt1) * (cnf1 * cnf1 * cnf1 - cnf2 * cnf2 * cnf2) / 3.0;
        double iSnz  = (cnt1 - cnt2) * (cnf1 * cnf1 * cnf1 - cnf2 * cnf2 * cnf2) / 3.0;

        var dm = new Matrix(1, 3, 1, 3);
        dm[1, 1] = iSn2 + iz2;
        dm[2, 2] = iCn2 + iz2;
        dm[3, 3] = ir2 * (teta2 - teta1);
        dm[1, 2] = dm[2, 1] = -iCnSn;
        dm[1, 3] = dm[3, 1] = -iCnz;
        dm[3, 2] = dm[2, 3] = -iSnz;

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
        r = r * r * r * r;
        var v1 = new math_Vector(1, 3);
        var v2 = new math_Vector(1, 3);
        var v3 = new math_Vector(1, 3);
        j.Vector(1, v1);
        v1.Multiply(passage, v1);
        v1.Multiply(r * j.Value(1));
        j.Vector(2, v2);
        v2.Multiply(passage, v2);
        v2.Multiply(r * j.Value(2));
        j.Vector(3, v3);
        v3.Multiply(passage, v3);
        v3.Multiply(r * j.Value(3));

        // OCCT: P V L (each eigenvector scaled by its value) is not the matrix P V L V^T P^T, and math_Jacobi sorts
        // the values, so the result is permuted or wrong except in special cases (docs/occt-issues.md). The
        // V.Multiply(Passage, V) above also reads V while overwriting it, as in OCCT.
        _inertia = new Matrix3x3(new Xyz(v1[1], v2[1], v3[1]), new Xyz(v1[2], v2[2], v3[2]), new Xyz(v1[3], v2[3], v3[3]));
        global::Kerf.Engine.Measurement.GProp.HOperator(_g, _loc, _dim, out Matrix3x3 hop);
        _inertia = _inertia + hop;
    }

    public void Perform(in Torus s, double teta1, double teta2, double alpha1, double alpha2)
    {
        s.Location.Coord(out double x0, out double y0, out double z0);
        s.XAxis.Direction.Coord(out double xa1, out double ya1, out double za1);
        s.YAxis.Direction.Coord(out double xa2, out double ya2, out double za2);
        s.Axis.Direction.Coord(out double xa3, out double ya3, out double za3);
        double rMax = s.MajorRadius;
        double rmin = s.MinorRadius;
        double cnt1 = Math.Cos(teta1);
        double snt1 = Math.Sin(teta1);
        double cnt2 = Math.Cos(teta2);
        double snt2 = Math.Sin(teta2);
        double cnf1 = Math.Cos(alpha1);
        double snf1 = Math.Sin(alpha1);
        double cnf2 = Math.Cos(alpha2);
        double snf2 = Math.Sin(alpha2);

        _dim       = rMax * rmin * (teta2 - teta1) * (alpha2 - alpha1);
        double ix = (snt2 - snt1) / (teta2 - teta1) * (rmin * (snf2 - snf1) / (alpha2 - alpha1) + rMax);
        double iy = (cnt1 - cnt2) / (teta2 - teta1) * (rmin * (snf2 - snf1) / (alpha2 - alpha1) + rMax);
        double iz = rmin * (cnf1 - cnf2) / (alpha2 - alpha1);

        // OCCT: g is absolute here, but GProp_GProps treats it as relative to loc (wrong when loc is not the origin).
        _g = _g.WithCoord(x0 + ix * xa1 + iy * xa2 + iz * xa3,
                              y0 + ix * ya1 + iy * ya2 + iz * ya3,
                              z0 + ix * za1 + iy * za2 + iz * za3);

        double ir2 =
            rMax * rMax + 2.0 * rMax * rmin * (snf2 - snf1) + rmin * rmin / 2.0 * (snf2 * cnf2 - snf1 * cnf1);
        double iCn2  = ir2 * (teta2 - teta1 + snt2 * cnt2 - snt1 * cnt1) / 2.0;
        double iSn2  = ir2 * (teta2 - teta1 - snt2 * cnt2 + snt1 * cnt1) / 2.0;
        double iCnSn = ir2 * (snt2 * snt2 - snt1 * snt1) / 2.0;
        double iz2   = (teta2 - teta1) * rmin * rmin * (alpha2 - alpha1 - snf2 * cnf2 + snf1 * cnf1) / 2.0;
        double iCnz  = rmin * (snt2 - snt1) * (cnf1 - cnf2) * (rMax + rmin * (cnf1 + cnf2) / 2.0);
        double iSnz  = rmin * (cnt2 - cnt1) * (cnf1 - cnf2) * (rMax + rmin * (cnf1 + cnf2) / 2.0);

        var dm = new Matrix(1, 3, 1, 3);
        dm[1, 1] = iSn2 + iz2;
        dm[2, 2] = iCn2 + iz2;
        dm[3, 3] = ir2 * (teta2 - teta1);
        dm[1, 2] = dm[2, 1] = -iCnSn;
        dm[1, 3] = dm[3, 1] = -iCnz;
        dm[3, 2] = dm[2, 3] = -iSnz;

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
        rMax = rMax * rmin;
        var v1 = new math_Vector(1, 3);
        var v2 = new math_Vector(1, 3);
        var v3 = new math_Vector(1, 3);
        j.Vector(1, v1);
        v1.Multiply(passage, v1);
        v1.Multiply(rMax * j.Value(1));
        j.Vector(2, v2);
        v2.Multiply(passage, v2);
        v2.Multiply(rMax * j.Value(2));
        j.Vector(3, v3);
        v3.Multiply(passage, v3);
        v3.Multiply(rMax * j.Value(3));

        // OCCT: P V L (each eigenvector scaled by its value) is not the matrix P V L V^T P^T, and math_Jacobi sorts
        // the values, so the result is permuted or wrong except in special cases (docs/occt-issues.md). The
        // V.Multiply(Passage, V) above also reads V while overwriting it, as in OCCT.
        _inertia = new Matrix3x3(new Xyz(v1[1], v2[1], v3[1]), new Xyz(v1[2], v2[2], v3[2]), new Xyz(v1[3], v2[3], v3[3]));
        global::Kerf.Engine.Measurement.GProp.HOperator(_g, _loc, _dim, out Matrix3x3 hop);
        _inertia = _inertia + hop;
    }

    public SelGProps(in Cone s, double alpha1, double alpha2, double z1, double z2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, alpha1, alpha2, z1, z2);
    }

    public SelGProps(in Cylinder s, double alpha1, double alpha2, double z1, double z2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, alpha1, alpha2, z1, z2);
    }

    public SelGProps(in Sphere s, double teta1, double teta2, double alpha1, double alpha2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, teta1, teta2, alpha1, alpha2);
    }

    public SelGProps(in Torus s, double teta1, double teta2, double alpha1, double alpha2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, teta1, teta2, alpha1, alpha2);
    }
}
