// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Coons.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Coons.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The Coons filling of a grid of poles: the boundary rows are kept and the inner poles blend them with cubic
/// Hermite functions (the bicubically blended Coons patch).
/// </summary>
/// <remarks>
/// P1 and P3 are the rows j = 1 and j = NbVPoles, P2 the column i = 1 and P4 the column i = NbUPoles (for the
/// poles; OCCT's weights put W4 in column 1 and W2 in the last column, occt-issues.md).
/// </remarks>
public class GeomFill_Coons : GeomFill_Filling
{
    public GeomFill_Coons()
    {
    }

    public GeomFill_Coons(NCollection_Array1<Point3D> P1, NCollection_Array1<Point3D> P2, NCollection_Array1<Point3D> P3,
                          NCollection_Array1<Point3D> P4)
    {
        Init(P1, P2, P3, P4);
    }

    public GeomFill_Coons(NCollection_Array1<Point3D> P1, NCollection_Array1<Point3D> P2, NCollection_Array1<Point3D> P3,
                          NCollection_Array1<Point3D> P4, NCollection_Array1<double> W1, NCollection_Array1<double> W2,
                          NCollection_Array1<double> W3, NCollection_Array1<double> W4)
    {
        Init(P1, P2, P3, P4, W1, W2, W3, W4);
    }

    public void Init(NCollection_Array1<Point3D> P1, NCollection_Array1<Point3D> P2, NCollection_Array1<Point3D> P3,
                     NCollection_Array1<Point3D> P4)
    {
        Standard_DomainError_Raise_if(P1.Length() != P3.Length() || P2.Length() != P4.Length(), " ");

        int NPolU = P1.Length();
        int NPolV = P2.Length();

        IsRational = false;
        myPoles = new NCollection_HArray2<Point3D>(1, NPolU, 1, NPolV);

        // The boundaries are not modified
        int i, j, k;
        for (i = 1; i <= NPolU; i++)
        {
            myPoles[i, 1] = P1[i];
            myPoles[i, NPolV] = P3[i];
        }
        for (i = 1; i <= NPolV; i++)
        {
            myPoles[1, i] = P2[i];
            myPoles[NPolU, i] = P4[i];
        }

        // Calcul des coefficients multiplicateurs
        NCollection_Array1<Point3D> Coef = new(1, 4);
        NCollection_Array1<Point3D> Pole = new(1, 4);
        NCollection_Array1<Point3D> CoefU = new(1, NPolU);
        NCollection_Array1<Point3D> CoefV = new(1, NPolV);
        Coef[4] = new Point3D(2.0, -2.0, 0.0);
        Coef[3] = new Point3D(-3.0, 3.0, 0.0);
        Coef[2] = new Point3D(0.0, 0.0, 0.0);
        Coef[1] = new Point3D(1.0, 0.0, 0.0);
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(Coef, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), Pole, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        if (NPolU > 4)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(NPolU - 1, Pole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), CoefU, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
        }
        else
        {
            CoefU.Assign(Pole);
        }

        if (NPolV > 4)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(NPolV - 1, Pole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), CoefV, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
        }
        else
        {
            CoefV.Assign(Pole);
        }

        NCollection_Array1<double> FU = new(2, NPolU - 1);
        NCollection_Array1<double> GU = new(2, NPolU - 1);
        NCollection_Array1<double> FV = new(2, NPolV - 1);
        NCollection_Array1<double> GV = new(2, NPolV - 1);

        double Dummy;
        for (i = 2; i < NPolU; i++)
        {
            CoefU[i].Coord(out FU[i], out GU[i], out Dummy);
        }
        for (i = 2; i < NPolV; i++)
        {
            CoefV[i].Coord(out FV[i], out GV[i], out Dummy);
        }

        // Clacul des poles interieurs
        Point3D P = new();
        for (j = 2; j < NPolV; j++)
        {
            for (i = 2; i < NPolU; i++)
            {
                for (k = 1; k <= 3; k++)
                {
                    P.SetCoord(k,
                               FV[j] * (myPoles.Value(i, 1)).Coord(k) + GV[j] * (myPoles.Value(i, NPolV)).Coord(k)
                               + FU[i] * (myPoles.Value(1, j)).Coord(k) + GU[i] * (myPoles.Value(NPolU, j)).Coord(k)
                               - FU[i] * FV[j] * (myPoles.Value(1, 1)).Coord(k)
                               - FU[i] * GV[j] * (myPoles.Value(1, NPolV)).Coord(k)
                               - GU[i] * FV[j] * (myPoles.Value(NPolU, 1)).Coord(k)
                               - GU[i] * GV[j] * (myPoles.Value(NPolU, NPolV)).Coord(k));
                }
                myPoles[i, j] = P;
            }
        }
    }

    public void Init(NCollection_Array1<Point3D> P1, NCollection_Array1<Point3D> P2, NCollection_Array1<Point3D> P3,
                     NCollection_Array1<Point3D> P4, NCollection_Array1<double> W1, NCollection_Array1<double> W2,
                     NCollection_Array1<double> W3, NCollection_Array1<double> W4)
    {
        Standard_DomainError_Raise_if(W1.Length() != W3.Length() || W2.Length() != W4.Length(), " ");
        Standard_DomainError_Raise_if(W1.Length() != P1.Length() || W2.Length() != P2.Length()
                                      || W3.Length() != P3.Length() || W4.Length() != P4.Length(), " ");

        Init(P1, P2, P3, P4);
        IsRational = true;

        int NPolU = W1.Length();
        int NPolV = W2.Length();

        double NU = NPolU - 1;
        double NV = NPolV - 1;
        myWeights = new NCollection_HArray2<double>(1, NPolU, 1, NPolV);
        // The boundaries are not modified
        int i, j;
        for (i = 1; i <= NPolU; i++)
        {
            myWeights[i, 1] = W1[i];
            myWeights[i, NPolV] = W3[i];
        }
        double PU, PU1, PV, PV1;

        for (j = 2; j <= NPolV - 1; j++)
        {
            PV = (j - 1) / NV;
            PV1 = 1 - PV;
            // OCCT: W4 in the first column and W2 in the last, where the poles have P2 and P4 (occt-issues.md)
            myWeights[1, j] = W4[j];
            myWeights[NPolU, j] = W2[j];
            for (i = 2; i <= NPolU - 1; i++)
            {
                PU = (i - 1) / NU;
                PU1 = 1 - PU;
                double W = PV1 * W1[i] + PV * W3[i] + PU * W2[j] + PU1 * W4[j]
                           - (PU1 * PV1 * W1[1] + PU * PV1 * W2[1] + PU * PV * W3[NPolU] + PU1 * PV * W4[NPolV]);
                myWeights[i, j] = W;
            }
        }
    }
}
