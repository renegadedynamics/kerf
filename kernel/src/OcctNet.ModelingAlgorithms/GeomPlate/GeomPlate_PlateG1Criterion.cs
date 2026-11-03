// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PlateG1Criterion.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PlateG1Criterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Advanced.CriterionRepartition;
using static Kerf.Engine.Approximation.Advanced.CriterionType;

namespace Occt;

/// <summary>
/// The G1 criterion of a plate's approximation: the largest angle between a patch's normal and the normals the
/// plate surface has at given UV points.
/// </summary>
/// <remarks>The criterion keeps copies of the caller's sequences, as OCCT's.</remarks>
public class GeomPlate_PlateG1Criterion : Criterion
{
    private readonly NCollection_Sequence<Xy> myData;
    private readonly NCollection_Sequence<Xyz> myXYZ;

    public GeomPlate_PlateG1Criterion(NCollection_Sequence<Xy> Data, NCollection_Sequence<Xyz> G1Data, double Maximum,
                                      CriterionType Type = Absolute,
                                      CriterionRepartition Repart = Regular)
    {
        myData = new NCollection_Sequence<Xy>(Data);
        myXYZ = new NCollection_Sequence<Xyz>(G1Data);
        _maxValue = Maximum;
        _type = Type;
        _repartition = Repart;
    }

    public override void Value(Patch P, Context C)
    {
        Span<double> UInt = stackalloc double[2], VInt = stackalloc double[2];
        Span<int> MaxNbCoeff = stackalloc int[2], NbCoeff = stackalloc int[2];
        ReadOnlySpan<double> adrCoeff = P.Coefficients(1, C)!.AsSpan();
        MaxNbCoeff[0] = C.ULimit();
        MaxNbCoeff[1] = C.VLimit();
        NbCoeff[0] = P.CoeffInUCount;
        NbCoeff[1] = P.CoeffInVCount;
        UInt[0] = P.U0();
        UInt[1] = P.U1();
        VInt[0] = P.V0();
        VInt[1] = P.V1();

        double up, vp, ang = 0.0;

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
            Vector3D v1s = new Vector3D(), v2s = new Vector3D(), v3s;
            Vector3D v3h = new Vector3D(myXYZ.Value(i).X(), myXYZ.Value(i).Y(), myXYZ.Value(i).Z());
            Xy P2d = myData.Value(i);
            if (UInt[0] < P2d.X() && P2d.X() < UInt[1] && VInt[0] < P2d.Y() && P2d.Y() < VInt[1])
            {
                //   u,v recadres sur (-1,1)
                up = (2 * P2d.X() - UInt[0] - UInt[1]) / (UInt[1] - UInt[0]);
                vp = (2 * P2d.Y() - VInt[0] - VInt[1]) / (VInt[1] - VInt[0]);
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPoly2Var(up, vp, 1, 0, NbCoeff[0] - 1, NbCoeff[1] - 1, 3, Coeffs, Digit);
                v1s.SetCoord(1, Digit[0]);
                v1s.SetCoord(2, Digit[1]);
                v1s.SetCoord(3, Digit[2]);
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPoly2Var(up, vp, 0, 1, NbCoeff[0] - 1, NbCoeff[1] - 1, 3, Coeffs, Digit);
                v2s.SetCoord(1, Digit[0]);
                v2s.SetCoord(2, Digit[1]);
                v2s.SetCoord(3, Digit[2]);
                v3s = v1s ^ v2s;
                if (v3s.Angle(v3h) > (M_PI / 2))
                {
                    if ((M_PI - v3s.Angle(v3h)) > ang)
                    {
                        ang = (M_PI - v3s.Angle(v3h));
                    }
                }
                else
                {
                    if (v3s.Angle(v3h) > ang)
                    {
                        ang = v3s.Angle(v3h);
                    }
                }
            }
        }
        P.SetCritValue(ang);
    }

    public override bool IsSatisfied(Patch P)
    {
        return (P.CritValue() < _maxValue);
    }
}
