// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PolynomialConvertor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PolynomialConvertor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Conversion;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// Converts an arc of circle, and its derivatives, into the poles of a polynomial B-spline of degree 7 (one span,
/// G2 at the ends), as <see cref="GeomFill.GetCircle(ParameterisationType, in Vector3D, in Vector3D, in Vector3D, in Point3D, in Point3D, double, in Point3D, NCollection_Array1{Point3D}, NCollection_Array1{double})"/>
/// uses it.
/// </summary>
/// <remarks>A working object; <see cref="Init"/> must be called before <c>Section</c>.</remarks>
public sealed class PolynomialConvertor
{
    // BH = B * H where B is the monomial-to-BSpline conversion matrix on [-1,1], degree 7,
    // and H is the Hermite coefficients matrix. Both are mathematical constants computed once.
    private static readonly Matrix THE_BH_MATRIX = ComputeBH();

    private readonly int _ordre;
    private bool _myinit;
    private readonly Matrix _bh;

    public PolynomialConvertor()
    {
        _ordre = 8;
        _myinit = false;
        _bh = new Matrix(1, _ordre, 1, _ordre);
    }

    /// <summary>Whether <see cref="Init"/> has been called.</summary>
    public bool Initialized() => _myinit;

    /// <summary>Computes the conversion matrix (once).</summary>
    public void Init()
    {
        if (_myinit)
        {
            return;
        }

        _bh.Initialized(THE_BH_MATRIX);
        _myinit = true;
    }

    private static Matrix ComputeBH()
    {
        const int ordre = 8;
        NCollection_Array1<double> coeffs = new(1, ordre * ordre);
        NCollection_Array1<double> inter = new(1, 2), trueInter = new(1, 2);
        inter.SetValue(1, -1);
        inter.SetValue(2, 1);
        trueInter.SetValue(1, -1);
        trueInter.SetValue(2, 1);
        coeffs.Init(0);
        for (int ii = 1; ii <= ordre; ii++)
        {
            coeffs.SetValue(ii + (ii - 1) * ordre, 1);
        }

        CompPolynomialToPoles converter = new CompPolynomialToPoles(ordre,
                                                                                    ordre - 1,
                                                                                    ordre - 1,
                                                                                    coeffs,
                                                                                    inter,
                                                                                    trueInter);
        NCollection_Array2<double> poles = converter.Poles();
        Matrix b = new Matrix(1, ordre, 1, ordre);
        for (int jj = 1; jj <= ordre; jj++)
        {
            for (int ii = 1; ii <= ordre; ii++)
            {
                double term = poles.Value(ii, jj);
                if (Math.Abs(term - 1) < 1.0e-9)
                {
                    term = 1;
                }
                if (Math.Abs(term + 1) < 1.0e-9)
                {
                    term = -1;
                }
                b[ii, jj] = term;
            }
        }

        Matrix h = new Matrix(1, ordre, 1, ordre);
        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(-1, 1, ordre / 2 - 1, ordre / 2 - 1, h);
        h.Transpose();
        return new Matrix(b * h);
    }

    /// <summary>The poles of the arc of Angle from FirstPnt about Center, around the axis Dir.</summary>
    public void Section(in Point3D firstPnt, in Point3D center, in Vector3D dir, double angle,
                        NCollection_Array1<Point3D> poles)
    {
        math_Vector vx = new(1, _ordre), vy = new(1, _ordre);
        math_Vector px, py;
        int ii;
        double Cos_b = Math.Cos(angle), Sin_b = Math.Sin(angle);
        double beta, beta2, beta3;
        Vector3D v1 = new Vector3D(center, firstPnt), v2;
        v2 = dir ^ v1;
        beta = angle / 2;
        beta2 = beta * beta;
        beta3 = beta * beta2;

        // Calcul de la transformation
        Matrix3x3 m = new Matrix3x3(v1.X(), v2.X(), 0, v1.Y(), v2.Y(), 0, v1.Z(), v2.Z(), 0);

        // Calcul des contraintes  -----------
        vx[1] = 1;
        vy[1] = 0;
        vx[2] = 0;
        vy[2] = beta;
        vx[3] = -beta2;
        vy[3] = 0;
        vx[4] = 0;
        vy[4] = -beta3;
        vx[5] = Cos_b;
        vy[5] = Sin_b;
        vx[6] = -beta * Sin_b;
        vy[6] = beta * Cos_b;
        vx[7] = -beta2 * Cos_b;
        vy[7] = -beta2 * Sin_b;
        vx[8] = beta3 * Sin_b;
        vy[8] = -beta3 * Cos_b;

        // Calcul des poles
        px = _bh * vx;
        py = _bh * vy;
        Xyz pnt = new Xyz();
        for (ii = 1; ii <= _ordre; ii++)
        {
            pnt.SetCoord(px[ii], py[ii], 0);
            pnt *= m;
            pnt += center.XYZ();
            poles[ii].ChangeCoord() = pnt;
        }
    }

    /// <summary>The poles of the arc and their first derivatives.</summary>
    public void Section(in Point3D firstPnt, in Vector3D dFirstPnt, in Point3D center, in Vector3D dCenter, in Vector3D dir,
                        in Vector3D dDir, double angle, double dAngle, NCollection_Array1<Point3D> poles,
                        NCollection_Array1<Vector3D> dPoles)
    {
        math_Vector vx = new(1, _ordre), vy = new(1, _ordre), dVx = new(1, _ordre), dVy = new(1, _ordre);
        math_Vector px, py, dPx, dPy;
        int ii;
        double Cos_b = Math.Cos(angle), Sin_b = Math.Sin(angle);
        double beta, beta2, beta3, bprim;
        Vector3D v1 = new Vector3D(center, firstPnt), v1Prim, v2;
        v2 = dir ^ v1;
        beta = angle / 2;
        bprim = dAngle / 2;
        beta2 = beta * beta;
        beta3 = beta * beta2;

        // Calcul des  transformations
        Matrix3x3 m = new Matrix3x3(v1.X(), v2.X(), 0, v1.Y(), v2.Y(), 0, v1.Z(), v2.Z(), 0);
        v1Prim = dFirstPnt - dCenter;
        v2 = (dDir ^ v1) + (dir ^ v1Prim);
        Matrix3x3 mPrim = new Matrix3x3(v1Prim.X(), v2.X(), 0, v1Prim.Y(), v2.Y(), 0, v1Prim.Z(), v2.Z(), 0);

        // Calcul des contraintes  -----------
        vx[1] = 1;
        vy[1] = 0;
        vx[2] = 0;
        vy[2] = beta;
        vx[3] = -beta2;
        vy[3] = 0;
        vx[4] = 0;
        vy[4] = -beta3;
        vx[5] = Cos_b;
        vy[5] = Sin_b;
        vx[6] = -beta * Sin_b;
        vy[6] = beta * Cos_b;
        vx[7] = -beta2 * Cos_b;
        vy[7] = -beta2 * Sin_b;
        vx[8] = beta3 * Sin_b;
        vy[8] = -beta3 * Cos_b;

        double b_bprim = bprim * beta, b2_bprim = bprim * beta2;
        dVx[1] = 0;
        dVy[1] = 0;
        dVx[2] = 0;
        dVy[2] = bprim;
        dVx[3] = -2 * b_bprim;
        dVy[3] = 0;
        dVx[4] = 0;
        dVy[4] = -3 * b2_bprim;
        dVx[5] = -2 * bprim * Sin_b;
        dVy[5] = 2 * bprim * Cos_b;
        dVx[6] = -bprim * Sin_b - 2 * b_bprim * Cos_b;
        dVy[6] = bprim * Cos_b - 2 * b_bprim * Sin_b;
        dVx[7] = 2 * b_bprim * (-Cos_b + beta * Sin_b);
        dVy[7] = -2 * b_bprim * (Sin_b + beta * Cos_b);
        dVx[8] = b2_bprim * (3 * Sin_b + 2 * beta * Cos_b);
        dVy[8] = b2_bprim * (2 * beta * Sin_b - 3 * Cos_b);

        // Calcul des poles
        px = _bh * vx;
        py = _bh * vy;
        dPx = _bh * dVx;
        dPy = _bh * dVy;
        Xyz p = new Xyz(), dp = new Xyz(), aux = new Xyz();

        for (ii = 1; ii <= _ordre; ii++)
        {
            p.SetCoord(px[ii], py[ii], 0);
            poles[ii].ChangeCoord() = m * p + center.XYZ();
            p *= mPrim;
            dp.SetCoord(dPx[ii], dPy[ii], 0);
            dp *= m;
            aux.SetLinearForm(1, p, 1, dp, dCenter.XYZ());
            dPoles[ii].SetXYZ(aux);
        }
    }

    /// <summary>The poles of the arc and their first and second derivatives.</summary>
    public void Section(in Point3D firstPnt, in Vector3D dFirstPnt, in Vector3D d2FirstPnt, in Point3D center,
                        in Vector3D dCenter, in Vector3D d2Center, in Vector3D dir, in Vector3D dDir, in Vector3D d2Dir,
                        double angle, double dAngle, double d2Angle, NCollection_Array1<Point3D> poles,
                        NCollection_Array1<Vector3D> dPoles, NCollection_Array1<Vector3D> d2Poles)
    {
        math_Vector vx = new(1, _ordre), vy = new(1, _ordre), dVx = new(1, _ordre), dVy = new(1, _ordre),
                    d2Vx = new(1, _ordre), d2Vy = new(1, _ordre);
        math_Vector px, py, dPx, dPy, d2Px, d2Py;

        int ii;
        double aux, Cos_b = Math.Cos(angle), Sin_b = Math.Sin(angle);
        double beta, beta2, beta3, bprim, bprim2, bsecn;
        Vector3D v1 = new Vector3D(center, firstPnt), v1Prim, v1Secn, v2;
        v2 = dir ^ v1;
        beta = angle / 2;
        bprim = dAngle / 2;
        bsecn = d2Angle / 2;
        bsecn = d2Angle / 2;
        beta2 = beta * beta;
        beta3 = beta * beta2;
        bprim2 = bprim * bprim;

        // Calcul des  transformations
        Matrix3x3 m = new Matrix3x3(v1.X(), v2.X(), 0, v1.Y(), v2.Y(), 0, v1.Z(), v2.Z(), 0);
        v1Prim = dFirstPnt - dCenter;
        v2 = (dDir ^ v1) + (dir ^ v1Prim);

        Matrix3x3 mPrim = new Matrix3x3(v1Prim.X(), v2.X(), 0, v1Prim.Y(), v2.Y(), 0, v1Prim.Z(), v2.Z(), 0);
        v1Secn = d2FirstPnt - d2Center;
        v2 = dDir ^ v1Prim;
        v2 *= 2;
        v2 += (d2Dir ^ v1) + (dir ^ v1Secn);

        Matrix3x3 mSecn = new Matrix3x3(v1Secn.X(), v2.X(), 0, v1Secn.Y(), v2.Y(), 0, v1Secn.Z(), v2.Z(), 0);

        // Calcul des contraintes  -----------
        vx[1] = 1;
        vy[1] = 0;
        vx[2] = 0;
        vy[2] = beta;
        vx[3] = -beta2;
        vy[3] = 0;
        vx[4] = 0;
        vy[4] = -beta3;
        vx[5] = Cos_b;
        vy[5] = Sin_b;
        vx[6] = -beta * Sin_b;
        vy[6] = beta * Cos_b;
        vx[7] = -beta2 * Cos_b;
        vy[7] = -beta2 * Sin_b;
        vx[8] = beta3 * Sin_b;
        vy[8] = -beta3 * Cos_b;

        double b_bprim = bprim * beta, b2_bprim = bprim * beta2, b_bsecn = bsecn * beta;
        dVx[1] = 0;
        dVy[1] = 0;
        dVx[2] = 0;
        dVy[2] = bprim;
        dVx[3] = -2 * b_bprim;
        dVy[3] = 0;
        dVx[4] = 0;
        dVy[4] = -3 * b2_bprim;
        dVx[5] = -2 * bprim * Sin_b;
        dVy[5] = 2 * bprim * Cos_b;
        dVx[6] = -bprim * Sin_b - 2 * b_bprim * Cos_b;
        dVy[6] = bprim * Cos_b - 2 * b_bprim * Sin_b;
        dVx[7] = 2 * b_bprim * (-Cos_b + beta * Sin_b);
        dVy[7] = -2 * b_bprim * (Sin_b + beta * Cos_b);
        dVx[8] = b2_bprim * (3 * Sin_b + 2 * beta * Cos_b);
        dVy[8] = b2_bprim * (2 * beta * Sin_b - 3 * Cos_b);

        d2Vx[1] = 0;
        d2Vy[1] = 0;
        d2Vx[2] = 0;
        d2Vy[2] = bsecn;
        d2Vx[3] = -2 * (bprim2 + b_bsecn);
        d2Vy[3] = 0;
        d2Vx[4] = 0;
        d2Vy[4] = -3 * beta * (2 * bprim2 + b_bsecn);
        d2Vx[5] = -2 * (bsecn * Sin_b + 2 * bprim2 * Cos_b);
        d2Vy[5] = 2 * (bsecn * Cos_b - 2 * bprim2 * Sin_b);
        d2Vx[6] = (4 * beta * bprim2 - bsecn) * Sin_b - 2 * (2 * bprim2 + b_bsecn) * Cos_b;

        d2Vy[6] = (bsecn - 4 * beta * bprim2) * Cos_b - 2 * (b_bsecn + 2 * bprim2) * Sin_b;

        aux = 2 * (bprim2 + b_bsecn);
        d2Vx[7] = aux * (-Cos_b + beta * Sin_b) + 2 * beta * bprim2 * (2 * beta * Cos_b + 3 * Sin_b);

        d2Vy[7] = -aux * (Sin_b + beta * Cos_b) - 2 * beta * bprim2 * (3 * Cos_b - 2 * beta * Sin_b);

        aux = beta * (2 * bprim2 + b_bsecn);
        d2Vx[8] = aux * (3 * Sin_b + 2 * beta * Cos_b) + 4 * beta2 * bprim2 * (2 * Cos_b - beta * Sin_b);

        d2Vy[8] = aux * (2 * beta * Sin_b - 3 * Cos_b) + 4 * beta2 * bprim2 * (2 * Sin_b + beta * Cos_b);

        // Calcul des poles
        px = _bh * vx;
        py = _bh * vy;
        dPx = _bh * dVx;
        dPy = _bh * dVy;
        d2Px = _bh * d2Vx;
        d2Py = _bh * d2Vy;

        Xyz p = new Xyz(), dp = new Xyz(), d2P = new Xyz(), auxyz = new Xyz();
        for (ii = 1; ii <= _ordre; ii++)
        {
            p.SetCoord(px[ii], py[ii], 0);
            dp.SetCoord(dPx[ii], dPy[ii], 0);
            d2P.SetCoord(d2Px[ii], d2Py[ii], 0);

            poles[ii].ChangeCoord() = m * p + center.XYZ();
            auxyz.SetLinearForm(1, mPrim * p, 1, m * dp, dCenter.XYZ());
            dPoles[ii].SetXYZ(auxyz);
            p *= mSecn;
            dp *= mPrim;
            d2P *= m;
            auxyz.SetLinearForm(1, p, 2, dp, 1, d2P, d2Center.XYZ());
            d2Poles[ii].SetXYZ(auxyz);
        }
    }
}
