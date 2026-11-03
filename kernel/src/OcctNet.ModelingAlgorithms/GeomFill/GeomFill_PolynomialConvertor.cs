// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PolynomialConvertor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PolynomialConvertor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Conversion;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Converts an arc of circle, and its derivatives, into the poles of a polynomial B-spline of degree 7 (one span,
/// G2 at the ends), as <see cref="GeomFill.GetCircle(ParameterisationType, in Vector3D, in Vector3D, in Vector3D, in Point3D, in Point3D, double, in Point3D, NCollection_Array1{Point3D}, NCollection_Array1{double})"/>
/// uses it.
/// </summary>
/// <remarks>A working object; <see cref="Init"/> must be called before <c>Section</c>.</remarks>
public sealed class GeomFill_PolynomialConvertor
{
    // BH = B * H where B is the monomial-to-BSpline conversion matrix on [-1,1], degree 7,
    // and H is the Hermite coefficients matrix. Both are mathematical constants computed once.
    private static readonly Matrix THE_BH_MATRIX = ComputeBH();

    private readonly int Ordre;
    private bool myinit;
    private readonly Matrix BH;

    public GeomFill_PolynomialConvertor()
    {
        Ordre = 8;
        myinit = false;
        BH = new Matrix(1, Ordre, 1, Ordre);
    }

    /// <summary>Whether <see cref="Init"/> has been called.</summary>
    public bool Initialized() => myinit;

    /// <summary>Computes the conversion matrix (once).</summary>
    public void Init()
    {
        if (myinit)
        {
            return;
        }

        BH.Initialized(THE_BH_MATRIX);
        myinit = true;
    }

    private static Matrix ComputeBH()
    {
        const int anOrdre = 8;
        NCollection_Array1<double> aCoeffs = new(1, anOrdre * anOrdre);
        NCollection_Array1<double> anInter = new(1, 2), aTrueInter = new(1, 2);
        anInter.SetValue(1, -1);
        anInter.SetValue(2, 1);
        aTrueInter.SetValue(1, -1);
        aTrueInter.SetValue(2, 1);
        aCoeffs.Init(0);
        for (int ii = 1; ii <= anOrdre; ii++)
        {
            aCoeffs.SetValue(ii + (ii - 1) * anOrdre, 1);
        }

        CompPolynomialToPoles aConverter = new CompPolynomialToPoles(anOrdre,
                                                                                    anOrdre - 1,
                                                                                    anOrdre - 1,
                                                                                    aCoeffs,
                                                                                    anInter,
                                                                                    aTrueInter);
        NCollection_Array2<double> aPoles = aConverter.Poles();
        Matrix aB = new Matrix(1, anOrdre, 1, anOrdre);
        for (int jj = 1; jj <= anOrdre; jj++)
        {
            for (int ii = 1; ii <= anOrdre; ii++)
            {
                double aTerm = aPoles.Value(ii, jj);
                if (Math.Abs(aTerm - 1) < 1.0e-9)
                {
                    aTerm = 1;
                }
                if (Math.Abs(aTerm + 1) < 1.0e-9)
                {
                    aTerm = -1;
                }
                aB[ii, jj] = aTerm;
            }
        }

        Matrix aH = new Matrix(1, anOrdre, 1, anOrdre);
        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(-1, 1, anOrdre / 2 - 1, anOrdre / 2 - 1, aH);
        aH.Transpose();
        return new Matrix(aB * aH);
    }

    /// <summary>The poles of the arc of Angle from FirstPnt about Center, around the axis Dir.</summary>
    public void Section(in Point3D FirstPnt, in Point3D Center, in Vector3D Dir, double Angle,
                        NCollection_Array1<Point3D> Poles)
    {
        math_Vector Vx = new(1, Ordre), Vy = new(1, Ordre);
        math_Vector Px, Py;
        int ii;
        double Cos_b = Math.Cos(Angle), Sin_b = Math.Sin(Angle);
        double beta, beta2, beta3;
        Vector3D V1 = new Vector3D(Center, FirstPnt), V2;
        V2 = Dir ^ V1;
        beta = Angle / 2;
        beta2 = beta * beta;
        beta3 = beta * beta2;

        // Calcul de la transformation
        Matrix3x3 M = new Matrix3x3(V1.X(), V2.X(), 0, V1.Y(), V2.Y(), 0, V1.Z(), V2.Z(), 0);

        // Calcul des contraintes  -----------
        Vx[1] = 1;
        Vy[1] = 0;
        Vx[2] = 0;
        Vy[2] = beta;
        Vx[3] = -beta2;
        Vy[3] = 0;
        Vx[4] = 0;
        Vy[4] = -beta3;
        Vx[5] = Cos_b;
        Vy[5] = Sin_b;
        Vx[6] = -beta * Sin_b;
        Vy[6] = beta * Cos_b;
        Vx[7] = -beta2 * Cos_b;
        Vy[7] = -beta2 * Sin_b;
        Vx[8] = beta3 * Sin_b;
        Vy[8] = -beta3 * Cos_b;

        // Calcul des poles
        Px = BH * Vx;
        Py = BH * Vy;
        Xyz pnt = new Xyz();
        for (ii = 1; ii <= Ordre; ii++)
        {
            pnt.SetCoord(Px[ii], Py[ii], 0);
            pnt *= M;
            pnt += Center.XYZ();
            Poles[ii].ChangeCoord() = pnt;
        }
    }

    /// <summary>The poles of the arc and their first derivatives.</summary>
    public void Section(in Point3D FirstPnt, in Vector3D DFirstPnt, in Point3D Center, in Vector3D DCenter, in Vector3D Dir,
                        in Vector3D DDir, double Angle, double DAngle, NCollection_Array1<Point3D> Poles,
                        NCollection_Array1<Vector3D> DPoles)
    {
        math_Vector Vx = new(1, Ordre), Vy = new(1, Ordre), DVx = new(1, Ordre), DVy = new(1, Ordre);
        math_Vector Px, Py, DPx, DPy;
        int ii;
        double Cos_b = Math.Cos(Angle), Sin_b = Math.Sin(Angle);
        double beta, beta2, beta3, bprim;
        Vector3D V1 = new Vector3D(Center, FirstPnt), V1Prim, V2;
        V2 = Dir ^ V1;
        beta = Angle / 2;
        bprim = DAngle / 2;
        beta2 = beta * beta;
        beta3 = beta * beta2;

        // Calcul des  transformations
        Matrix3x3 M = new Matrix3x3(V1.X(), V2.X(), 0, V1.Y(), V2.Y(), 0, V1.Z(), V2.Z(), 0);
        V1Prim = DFirstPnt - DCenter;
        V2 = (DDir ^ V1) + (Dir ^ V1Prim);
        Matrix3x3 MPrim = new Matrix3x3(V1Prim.X(), V2.X(), 0, V1Prim.Y(), V2.Y(), 0, V1Prim.Z(), V2.Z(), 0);

        // Calcul des contraintes  -----------
        Vx[1] = 1;
        Vy[1] = 0;
        Vx[2] = 0;
        Vy[2] = beta;
        Vx[3] = -beta2;
        Vy[3] = 0;
        Vx[4] = 0;
        Vy[4] = -beta3;
        Vx[5] = Cos_b;
        Vy[5] = Sin_b;
        Vx[6] = -beta * Sin_b;
        Vy[6] = beta * Cos_b;
        Vx[7] = -beta2 * Cos_b;
        Vy[7] = -beta2 * Sin_b;
        Vx[8] = beta3 * Sin_b;
        Vy[8] = -beta3 * Cos_b;

        double b_bprim = bprim * beta, b2_bprim = bprim * beta2;
        DVx[1] = 0;
        DVy[1] = 0;
        DVx[2] = 0;
        DVy[2] = bprim;
        DVx[3] = -2 * b_bprim;
        DVy[3] = 0;
        DVx[4] = 0;
        DVy[4] = -3 * b2_bprim;
        DVx[5] = -2 * bprim * Sin_b;
        DVy[5] = 2 * bprim * Cos_b;
        DVx[6] = -bprim * Sin_b - 2 * b_bprim * Cos_b;
        DVy[6] = bprim * Cos_b - 2 * b_bprim * Sin_b;
        DVx[7] = 2 * b_bprim * (-Cos_b + beta * Sin_b);
        DVy[7] = -2 * b_bprim * (Sin_b + beta * Cos_b);
        DVx[8] = b2_bprim * (3 * Sin_b + 2 * beta * Cos_b);
        DVy[8] = b2_bprim * (2 * beta * Sin_b - 3 * Cos_b);

        // Calcul des poles
        Px = BH * Vx;
        Py = BH * Vy;
        DPx = BH * DVx;
        DPy = BH * DVy;
        Xyz P = new Xyz(), DP = new Xyz(), aux = new Xyz();

        for (ii = 1; ii <= Ordre; ii++)
        {
            P.SetCoord(Px[ii], Py[ii], 0);
            Poles[ii].ChangeCoord() = M * P + Center.XYZ();
            P *= MPrim;
            DP.SetCoord(DPx[ii], DPy[ii], 0);
            DP *= M;
            aux.SetLinearForm(1, P, 1, DP, DCenter.XYZ());
            DPoles[ii].SetXYZ(aux);
        }
    }

    /// <summary>The poles of the arc and their first and second derivatives.</summary>
    public void Section(in Point3D FirstPnt, in Vector3D DFirstPnt, in Vector3D D2FirstPnt, in Point3D Center,
                        in Vector3D DCenter, in Vector3D D2Center, in Vector3D Dir, in Vector3D DDir, in Vector3D D2Dir,
                        double Angle, double DAngle, double D2Angle, NCollection_Array1<Point3D> Poles,
                        NCollection_Array1<Vector3D> DPoles, NCollection_Array1<Vector3D> D2Poles)
    {
        math_Vector Vx = new(1, Ordre), Vy = new(1, Ordre), DVx = new(1, Ordre), DVy = new(1, Ordre),
                    D2Vx = new(1, Ordre), D2Vy = new(1, Ordre);
        math_Vector Px, Py, DPx, DPy, D2Px, D2Py;

        int ii;
        double aux, Cos_b = Math.Cos(Angle), Sin_b = Math.Sin(Angle);
        double beta, beta2, beta3, bprim, bprim2, bsecn;
        Vector3D V1 = new Vector3D(Center, FirstPnt), V1Prim, V1Secn, V2;
        V2 = Dir ^ V1;
        beta = Angle / 2;
        bprim = DAngle / 2;
        bsecn = D2Angle / 2;
        bsecn = D2Angle / 2;
        beta2 = beta * beta;
        beta3 = beta * beta2;
        bprim2 = bprim * bprim;

        // Calcul des  transformations
        Matrix3x3 M = new Matrix3x3(V1.X(), V2.X(), 0, V1.Y(), V2.Y(), 0, V1.Z(), V2.Z(), 0);
        V1Prim = DFirstPnt - DCenter;
        V2 = (DDir ^ V1) + (Dir ^ V1Prim);

        Matrix3x3 MPrim = new Matrix3x3(V1Prim.X(), V2.X(), 0, V1Prim.Y(), V2.Y(), 0, V1Prim.Z(), V2.Z(), 0);
        V1Secn = D2FirstPnt - D2Center;
        V2 = DDir ^ V1Prim;
        V2 *= 2;
        V2 += (D2Dir ^ V1) + (Dir ^ V1Secn);

        Matrix3x3 MSecn = new Matrix3x3(V1Secn.X(), V2.X(), 0, V1Secn.Y(), V2.Y(), 0, V1Secn.Z(), V2.Z(), 0);

        // Calcul des contraintes  -----------
        Vx[1] = 1;
        Vy[1] = 0;
        Vx[2] = 0;
        Vy[2] = beta;
        Vx[3] = -beta2;
        Vy[3] = 0;
        Vx[4] = 0;
        Vy[4] = -beta3;
        Vx[5] = Cos_b;
        Vy[5] = Sin_b;
        Vx[6] = -beta * Sin_b;
        Vy[6] = beta * Cos_b;
        Vx[7] = -beta2 * Cos_b;
        Vy[7] = -beta2 * Sin_b;
        Vx[8] = beta3 * Sin_b;
        Vy[8] = -beta3 * Cos_b;

        double b_bprim = bprim * beta, b2_bprim = bprim * beta2, b_bsecn = bsecn * beta;
        DVx[1] = 0;
        DVy[1] = 0;
        DVx[2] = 0;
        DVy[2] = bprim;
        DVx[3] = -2 * b_bprim;
        DVy[3] = 0;
        DVx[4] = 0;
        DVy[4] = -3 * b2_bprim;
        DVx[5] = -2 * bprim * Sin_b;
        DVy[5] = 2 * bprim * Cos_b;
        DVx[6] = -bprim * Sin_b - 2 * b_bprim * Cos_b;
        DVy[6] = bprim * Cos_b - 2 * b_bprim * Sin_b;
        DVx[7] = 2 * b_bprim * (-Cos_b + beta * Sin_b);
        DVy[7] = -2 * b_bprim * (Sin_b + beta * Cos_b);
        DVx[8] = b2_bprim * (3 * Sin_b + 2 * beta * Cos_b);
        DVy[8] = b2_bprim * (2 * beta * Sin_b - 3 * Cos_b);

        D2Vx[1] = 0;
        D2Vy[1] = 0;
        D2Vx[2] = 0;
        D2Vy[2] = bsecn;
        D2Vx[3] = -2 * (bprim2 + b_bsecn);
        D2Vy[3] = 0;
        D2Vx[4] = 0;
        D2Vy[4] = -3 * beta * (2 * bprim2 + b_bsecn);
        D2Vx[5] = -2 * (bsecn * Sin_b + 2 * bprim2 * Cos_b);
        D2Vy[5] = 2 * (bsecn * Cos_b - 2 * bprim2 * Sin_b);
        D2Vx[6] = (4 * beta * bprim2 - bsecn) * Sin_b - 2 * (2 * bprim2 + b_bsecn) * Cos_b;

        D2Vy[6] = (bsecn - 4 * beta * bprim2) * Cos_b - 2 * (b_bsecn + 2 * bprim2) * Sin_b;

        aux = 2 * (bprim2 + b_bsecn);
        D2Vx[7] = aux * (-Cos_b + beta * Sin_b) + 2 * beta * bprim2 * (2 * beta * Cos_b + 3 * Sin_b);

        D2Vy[7] = -aux * (Sin_b + beta * Cos_b) - 2 * beta * bprim2 * (3 * Cos_b - 2 * beta * Sin_b);

        aux = beta * (2 * bprim2 + b_bsecn);
        D2Vx[8] = aux * (3 * Sin_b + 2 * beta * Cos_b) + 4 * beta2 * bprim2 * (2 * Cos_b - beta * Sin_b);

        D2Vy[8] = aux * (2 * beta * Sin_b - 3 * Cos_b) + 4 * beta2 * bprim2 * (2 * Sin_b + beta * Cos_b);

        // Calcul des poles
        Px = BH * Vx;
        Py = BH * Vy;
        DPx = BH * DVx;
        DPy = BH * DVy;
        D2Px = BH * D2Vx;
        D2Py = BH * D2Vy;

        Xyz P = new Xyz(), DP = new Xyz(), D2P = new Xyz(), auxyz = new Xyz();
        for (ii = 1; ii <= Ordre; ii++)
        {
            P.SetCoord(Px[ii], Py[ii], 0);
            DP.SetCoord(DPx[ii], DPy[ii], 0);
            D2P.SetCoord(D2Px[ii], D2Py[ii], 0);

            Poles[ii].ChangeCoord() = M * P + Center.XYZ();
            auxyz.SetLinearForm(1, MPrim * P, 1, M * DP, DCenter.XYZ());
            DPoles[ii].SetXYZ(auxyz);
            P *= MSecn;
            DP *= MPrim;
            D2P *= M;
            auxyz.SetLinearForm(1, P, 2, DP, 1, D2P, D2Center.XYZ());
            D2Poles[ii].SetXYZ(auxyz);
        }
    }
}
