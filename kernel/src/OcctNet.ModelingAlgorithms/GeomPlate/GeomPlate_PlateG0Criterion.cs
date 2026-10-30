// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PlateG0Criterion.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PlateG0Criterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AdvApp2Var_CriterionRepartition;
using static Occt.AdvApp2Var_CriterionType;

namespace Occt;

/// <summary>
/// The G0 criterion of a plate's approximation: the largest distance between a patch and the points the plate
/// surface takes at given UV points.
/// </summary>
/// <remarks>The criterion keeps copies of the caller's sequences, as OCCT's.</remarks>
public class GeomPlate_PlateG0Criterion : AdvApp2Var_Criterion
{
    private readonly NCollection_Sequence<gp_XY> myData;
    private readonly NCollection_Sequence<gp_XYZ> myXYZ;

    public GeomPlate_PlateG0Criterion(NCollection_Sequence<gp_XY> Data, NCollection_Sequence<gp_XYZ> G0Data, double Maximum,
                                      AdvApp2Var_CriterionType Type = AdvApp2Var_Absolute,
                                      AdvApp2Var_CriterionRepartition Repart = AdvApp2Var_Regular)
    {
        myData = new NCollection_Sequence<gp_XY>(Data);
        myXYZ = new NCollection_Sequence<gp_XYZ>(G0Data);
        myMaxValue = Maximum;
        myType = Type;
        myRepartition = Repart;
    }

    public override void Value(AdvApp2Var_Patch P, AdvApp2Var_Context C)
    {
        Span<double> UInt = stackalloc double[2], VInt = stackalloc double[2];
        Span<int> MaxNbCoeff = stackalloc int[2], NbCoeff = stackalloc int[2];
        ReadOnlySpan<double> adrCoeff = P.Coefficients(1, C)!.AsSpan();

        MaxNbCoeff[0] = C.ULimit();
        MaxNbCoeff[1] = C.VLimit();
        NbCoeff[0] = P.NbCoeffInU();
        NbCoeff[1] = P.NbCoeffInV();
        UInt[0] = P.U0();
        UInt[1] = P.U1();
        VInt[0] = P.V0();
        VInt[1] = P.V1();

        double up, vp, dist = 0.0;

        int dimension = 3 * NbCoeff[1];
        NCollection_Array1<double> Patch = new NCollection_Array1<double>(1, NbCoeff[0] * dimension);
        NCollection_Array1<double> Point = new NCollection_Array1<double>(1, 3);
        Span<double> Coeffs = Patch.AsSpan();
        Span<double> Digit = Point.AsSpan();

        int k1, k2, pos, ll = 1;
        for (k1 = 1; k1 <= NbCoeff[0]; k1++)
        {
            // JAG 99.04.29    pos = 3*(MaxNbCoeff[0])*(k1-1);
            pos = 3 * (MaxNbCoeff[1]) * (k1 - 1);
            for (k2 = 1; k2 <= NbCoeff[1]; k2++, pos += 3)
            {
                Patch[ll] = adrCoeff[pos];
                Patch[ll + 1] = adrCoeff[pos + 1];
                Patch[ll + 2] = adrCoeff[pos + 2];
                ll += 3;
            }
        }

        int i, NbCtr = myData.Length();
        for (i = 1; i <= NbCtr; i++)
        {
            gp_XY P2d = myData.Value(i);
            gp_Pnt P3d = new gp_Pnt();
            if (UInt[0] < P2d.X() && P2d.X() < UInt[1] && VInt[0] < P2d.Y() && P2d.Y() < VInt[1])
            {
                //   u,v recadres sur (-1,1)
                up = (2 * P2d.X() - UInt[0] - UInt[1]) / (UInt[1] - UInt[0]);
                vp = (2 * P2d.Y() - VInt[0] - VInt[1]) / (VInt[1] - VInt[0]);
                PLib.EvalPoly2Var(up, vp, 0, 0, NbCoeff[0] - 1, NbCoeff[1] - 1, 3, Coeffs, Digit);

                P3d.SetCoord(1, Digit[0]);
                P3d.SetCoord(2, Digit[1]);
                P3d.SetCoord(3, Digit[2]);
                double x = (P3d.Coord(1) - myXYZ.Value(i).Coord(1)),
                       y = (P3d.Coord(2) - myXYZ.Value(i).Coord(2)),
                       z = (P3d.Coord(3) - myXYZ.Value(i).Coord(3)), DistTmp = x * x + y * y + z * z;
                if (DistTmp > dist)
                {
                    dist = DistTmp;
                }
            }
        }
        P.SetCritValue(Math.Sqrt(dist));
    }

    public override bool IsSatisfied(AdvApp2Var_Patch P)
    {
        return (P.CritValue() < myMaxValue);
    }
}
