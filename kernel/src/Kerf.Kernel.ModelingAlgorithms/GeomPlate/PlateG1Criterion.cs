// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PlateG1Criterion.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PlateG1Criterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Advanced.CriterionRepartition;
using static Kerf.Engine.Approximation.Advanced.CriterionType;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// The G1 criterion of a plate's approximation: the largest angle between a patch's normal and the normals the
/// plate surface has at given UV points.
/// </summary>
/// <remarks>The criterion keeps copies of the caller's sequences, as OCCT's.</remarks>
public class PlateG1Criterion : Criterion
{
    private readonly NCollection_Sequence<Xy> _data;
    private readonly NCollection_Sequence<Xyz> _xyz;

    public PlateG1Criterion(NCollection_Sequence<Xy> data, NCollection_Sequence<Xyz> g1Data, double maximum,
                                      CriterionType type = Absolute,
                                      CriterionRepartition repart = Regular)
    {
        _data = new NCollection_Sequence<Xy>(data);
        _xyz = new NCollection_Sequence<Xyz>(g1Data);
        _maxValue = maximum;
        _type = type;
        _repartition = repart;
    }

    public override void Value(Patch p, Context c)
    {
        Span<double> uInt = stackalloc double[2], vInt = stackalloc double[2];
        Span<int> maxNbCoeff = stackalloc int[2], nbCoeff = stackalloc int[2];
        ReadOnlySpan<double> adrCoeff = p.Coefficients(1, c)!.AsSpan();
        maxNbCoeff[0] = c.ULimit();
        maxNbCoeff[1] = c.VLimit();
        nbCoeff[0] = p.CoeffInUCount;
        nbCoeff[1] = p.CoeffInVCount;
        uInt[0] = p.U0();
        uInt[1] = p.U1();
        vInt[0] = p.V0();
        vInt[1] = p.V1();

        double up, vp, ang = 0.0;

        int dimension = 3 * nbCoeff[1];
        NCollection_Array1<double> patch = new NCollection_Array1<double>(1, nbCoeff[0] * dimension);
        NCollection_Array1<double> point = new NCollection_Array1<double>(1, 3);
        Span<double> coeffs = patch.AsSpan();
        Span<double> digit = point.AsSpan();

        int k1, k2, pos, ll = 1;
        for (k1 = 1; k1 <= nbCoeff[0]; k1++)
        {
            // JAG 99.04.29    pos = 3*(MaxNbCoeff[0])*(k1-1);
            pos = 3 * (maxNbCoeff[1]) * (k1 - 1);
            for (k2 = 1; k2 <= nbCoeff[1]; k2++, pos += 3)
            {
                patch[ll] = adrCoeff[pos];
                patch[ll + 1] = adrCoeff[pos + 1];
                patch[ll + 2] = adrCoeff[pos + 2];
                ll += 3;
            }
        }

        int i, nbCtr = _data.Length();
        for (i = 1; i <= nbCtr; i++)
        {
            Vector3D v1s = new Vector3D(), v2s = new Vector3D(), v3s;
            Vector3D v3h = new Vector3D(_xyz.Value(i).X(), _xyz.Value(i).Y(), _xyz.Value(i).Z());
            Xy p2d = _data.Value(i);
            if (uInt[0] < p2d.X() && p2d.X() < uInt[1] && vInt[0] < p2d.Y() && p2d.Y() < vInt[1])
            {
                //   u,v recadres sur (-1,1)
                up = (2 * p2d.X() - uInt[0] - uInt[1]) / (uInt[1] - uInt[0]);
                vp = (2 * p2d.Y() - vInt[0] - vInt[1]) / (vInt[1] - vInt[0]);
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPoly2Var(up, vp, 1, 0, nbCoeff[0] - 1, nbCoeff[1] - 1, 3, coeffs, digit);
                v1s = v1s.WithCoord(1, digit[0]);
                v1s = v1s.WithCoord(2, digit[1]);
                v1s = v1s.WithCoord(3, digit[2]);
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPoly2Var(up, vp, 0, 1, nbCoeff[0] - 1, nbCoeff[1] - 1, 3, coeffs, digit);
                v2s = v2s.WithCoord(1, digit[0]);
                v2s = v2s.WithCoord(2, digit[1]);
                v2s = v2s.WithCoord(3, digit[2]);
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
        p.SetCritValue(ang);
    }

    public override bool IsSatisfied(Patch p)
    {
        return (p.CritValue() < _maxValue);
    }
}
