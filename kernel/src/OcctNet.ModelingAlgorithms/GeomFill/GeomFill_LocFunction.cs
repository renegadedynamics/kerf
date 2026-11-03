// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A location law as a function of 12 reals (the vector and the three columns of the matrix), for the
/// approximation of the law (AdvApprox).
/// </summary>
/// <remarks>A working object: evaluation stores its results in the function. It keeps the caller's law.</remarks>
public sealed class GeomFill_LocFunction
{
    private readonly GeomFill_LocationLaw myLaw;
    private readonly NCollection_Array1<Vector3D> V;
    private readonly NCollection_Array1<Vector3D> DV;
    private readonly NCollection_Array1<Vector3D> D2V;
    // OCCT's members M, DM and D2M are never used and are left out.

    public GeomFill_LocFunction(GeomFill_LocationLaw Law)
    {
        V = new NCollection_Array1<Vector3D>(1, 4);
        DV = new NCollection_Array1<Vector3D>(1, 4);
        D2V = new NCollection_Array1<Vector3D>(1, 4);
        myLaw = Law;
    }

    /// <summary>Computes the location for v = Param.</summary>
    public bool D0(double Param, double First, double Last)
    {
        Matrix3x3 aM = new Matrix3x3();
        bool B;
        B = myLaw.D0(Param, ref aM, ref V[1]);
        V[2].SetXYZ(aM.Column(1));
        V[3].SetXYZ(aM.Column(2));
        V[4].SetXYZ(aM.Column(3));
        return B;
    }

    /// <summary>Computes the location and its first derivative for v = Param.</summary>
    public bool D1(double Param, double First, double Last)
    {
        NCollection_Array1<Point2D> T1 = new(1, 1);
        NCollection_Array1<Vector2D> T2 = new(1, 1);
        Matrix3x3 aM = new Matrix3x3(), aDM = new Matrix3x3();
        bool B;
        B = myLaw.D1(Param, ref aM, ref V[1], ref aDM, ref DV[1], T1, T2);
        V[2].SetXYZ(aM.Column(1));
        V[3].SetXYZ(aM.Column(2));
        V[4].SetXYZ(aM.Column(3));
        DV[2].SetXYZ(aDM.Column(1));
        DV[3].SetXYZ(aDM.Column(2));
        DV[4].SetXYZ(aDM.Column(3));
        return B;
    }

    /// <summary>Computes the location and its first and second derivatives for v = Param.</summary>
    public bool D2(double Param, double First, double Last)
    {
        NCollection_Array1<Point2D> T1 = new(1, 1);
        NCollection_Array1<Vector2D> T2 = new(1, 1), T3 = new(1, 1);
        Matrix3x3 aM = new Matrix3x3(), aDM = new Matrix3x3(), aD2M = new Matrix3x3();
        bool B;
        B = myLaw.D2(Param, ref aM, ref V[1], ref aDM, ref DV[1], ref aD2M, ref D2V[1], T1, T2, T3);
        V[2].SetXYZ(aM.Column(1));
        V[3].SetXYZ(aM.Column(2));
        V[4].SetXYZ(aM.Column(3));
        DV[2].SetXYZ(aDM.Column(1));
        DV[3].SetXYZ(aDM.Column(2));
        DV[4].SetXYZ(aDM.Column(3));
        D2V[2].SetXYZ(aD2M.Column(1));
        D2V[3].SetXYZ(aD2M.Column(2));
        D2V[4].SetXYZ(aD2M.Column(3));
        return B;
    }

    /// <summary>
    /// The derivative of order Order (0 to 2) as 12 reals into Result (OCCT's <c>double&amp;</c> start of an
    /// array); Ier is Order + 1 when the law fails.
    /// </summary>
    public void DN(double Param, double First, double Last, int Order, Span<double> Result, ref int Ier)
    {
        bool B;
        NCollection_Array1<Vector3D>? LocalResult = null;
        Ier = 0;

        switch (Order)
        {
            case 0:
                B = D0(Param, First, Last);
                LocalResult = V;
                break;
            case 1:
                B = D1(Param, First, Last);
                LocalResult = DV;
                break;
            case 2:
                B = D2(Param, First, Last);
                LocalResult = D2V;
                break;
            default:
                B = false;
                break;
        }
        if (!B)
        {
            Ier = Order + 1;
        }

        // OCCT: another order leaves LocalResult null and reads through it; the port throws.
        if (LocalResult is null)
        {
            throw new OutOfRangeException("GeomFill_LocFunction::DN");
        }
        for (int ii = 0; ii <= 11; ii++)
        {
            Vector3D aV = LocalResult[1 + ii / 3];
            Result[ii] = aV.Coord(1 + ii % 3);
        }
    }
}
