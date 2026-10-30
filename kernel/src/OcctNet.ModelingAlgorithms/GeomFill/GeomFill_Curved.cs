// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Curved.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Curved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The Curved filling of a grid of poles: the inner poles are the average of the linear interpolations between
/// opposite boundaries (the most rounded style). With two rows, the first row is translated along the second.
/// </summary>
/// <remarks>P1 and P3 are the rows j = 1 and j = NbVPoles, P4 the column i = 1 and P2 the column i = NbUPoles.</remarks>
public class GeomFill_Curved : GeomFill_Filling
{
    public GeomFill_Curved()
    {
    }

    public GeomFill_Curved(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2, NCollection_Array1<gp_Pnt> P3,
                           NCollection_Array1<gp_Pnt> P4)
    {
        Init(P1, P2, P3, P4);
    }

    public GeomFill_Curved(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2, NCollection_Array1<gp_Pnt> P3,
                           NCollection_Array1<gp_Pnt> P4, NCollection_Array1<double> W1, NCollection_Array1<double> W2,
                           NCollection_Array1<double> W3, NCollection_Array1<double> W4)
    {
        Init(P1, P2, P3, P4, W1, W2, W3, W4);
    }

    public GeomFill_Curved(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2)
    {
        Init(P1, P2);
    }

    public GeomFill_Curved(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2, NCollection_Array1<double> W1,
                           NCollection_Array1<double> W2)
    {
        Init(P1, P2, W1, W2);
    }

    public void Init(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2, NCollection_Array1<gp_Pnt> P3,
                     NCollection_Array1<gp_Pnt> P4)
    {
        Standard_DomainError_Raise_if(P1.Length() != P3.Length() || P2.Length() != P4.Length(), " ");

        int NPolU = P1.Length();
        int NPolV = P2.Length();

        IsRational = false;

        double NU = NPolU - 1;
        double NV = NPolV - 1;
        myPoles = new NCollection_HArray2<gp_Pnt>(1, NPolU, 1, NPolV);

        // The boundaries are not modified
        int i, j, k;
        for (i = 1; i <= NPolU; i++)
        {
            myPoles[i, 1] = P1[i];
            myPoles[i, NPolV] = P3[i];
        }
        double PU, PU1, PV, PV1;

        for (j = 2; j <= NPolV - 1; j++)
        {
            PV = (j - 1) / NV;
            PV1 = 1 - PV;
            PV /= 2.0;
            PV1 /= 2.0;
            myPoles[1, j] = P4[j];
            myPoles[NPolU, j] = P2[j];
            for (i = 2; i <= NPolU - 1; i++)
            {
                PU = (i - 1) / NU;
                PU1 = 1 - PU;
                PU /= 2.0;
                PU1 /= 2.0;
                gp_Pnt P = new();
                for (k = 1; k <= 3; k++)
                {
                    P.SetCoord(k, PV1 * P1[i].Coord(k) + PV * P3[i].Coord(k) + PU * P2[j].Coord(k) + PU1 * P4[j].Coord(k));
                }
                myPoles[i, j] = P;
            }
        }
    }

    public void Init(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2, NCollection_Array1<gp_Pnt> P3,
                     NCollection_Array1<gp_Pnt> P4, NCollection_Array1<double> W1, NCollection_Array1<double> W2,
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
            PV /= 2.0;
            PV1 /= 2.0;
            myWeights[1, j] = W4[j];
            myWeights[NPolU, j] = W2[j];
            for (i = 2; i <= NPolU - 1; i++)
            {
                PU = (i - 1) / NU;
                PU1 = 1 - PU;
                PU /= 2.0;
                PU1 /= 2.0;
                double W = PV1 * W1[i] + PV * W3[i] + PU * W2[j] + PU1 * W4[j];
                myWeights[i, j] = W;
            }
        }
    }

    public void Init(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2)
    {
        int NPolU = P1.Length();
        int NPolV = P2.Length();

        IsRational = false;

        myPoles = new NCollection_HArray2<gp_Pnt>(1, NPolU, 1, NPolV);

        int i, j;

        for (j = 1; j <= NPolV; j++)
        {
            gp_Vec Tra = new(P2[1], P2[j]);
            for (i = 1; i <= NPolU; i++)
            {
                myPoles[i, j] = P1[i].Translated(Tra);
            }
        }
    }

    public void Init(NCollection_Array1<gp_Pnt> P1, NCollection_Array1<gp_Pnt> P2, NCollection_Array1<double> W1,
                     NCollection_Array1<double> W2)
    {
        Init(P1, P2);
        IsRational = true;

        // Initialisation des poids.
        int NPolU = W1.Length();
        int NPolV = W2.Length();

        myWeights = new NCollection_HArray2<double>(1, NPolU, 1, NPolV);
        for (int j = 1; j <= NPolV; j++)
        {
            double Factor = W2[j] / W1[1];
            for (int i = 1; i <= NPolU; i++)
            {
                myWeights[i, j] = W1[i] * Factor;
            }
        }
    }
}
