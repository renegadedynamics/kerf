// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Stretch.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Stretch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The Stretch filling of a grid of poles: the bilinearly blended Coons patch of the boundary rows (the flattest
/// style).
/// </summary>
/// <remarks>P1 and P3 are the rows j = 1 and j = NbVPoles, P4 the column i = 1 and P2 the column i = NbUPoles.</remarks>
internal class Stretch : Filling
{
    public Stretch()
    {
    }

    public Stretch(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                            NCollection_Array1<Point3D> p4)
    {
        Init(p1, p2, p3, p4);
    }

    public Stretch(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                            NCollection_Array1<Point3D> p4, NCollection_Array1<double> w1, NCollection_Array1<double> w2,
                            NCollection_Array1<double> w3, NCollection_Array1<double> w4)
    {
        Init(p1, p2, p3, p4, w1, w2, w3, w4);
    }

    public void Init(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                     NCollection_Array1<Point3D> p4)
    {
        Standard_DomainError_Raise_if(p1.Length() != p3.Length() || p2.Length() != p4.Length(), " ");

        int nPolU = p1.Length();
        int nPolV = p2.Length();

        _isRational = false;

        double nu = nPolU - 1;
        double nv = nPolV - 1;
        _poles = new NCollection_HArray2<Point3D>(1, nPolU, 1, nPolV);

        // The boundaries are not modified
        int i, j, k;
        for (i = 1; i <= nPolU; i++)
        {
            _poles[i, 1] = p1[i];
            _poles[i, nPolV] = p3[i];
        }
        double pu, pu1, pv, pv1;

        for (j = 2; j <= nPolV - 1; j++)
        {
            pv = (j - 1) / nv;
            pv1 = 1 - pv;
            _poles[1, j] = p4[j];
            _poles[nPolU, j] = p2[j];
            for (i = 2; i <= nPolU - 1; i++)
            {
                pu = (i - 1) / nu;
                pu1 = 1 - pu;
                Point3D p = new();
                for (k = 1; k <= 3; k++)
                {
                    p = p.WithCoord(k, pv1 * p1[i].Coord(k) + pv * p3[i].Coord(k) + pu * p2[j].Coord(k) + pu1 * p4[j].Coord(k)
                                  - (pu1 * pv1 * p1[1].Coord(k) + pu * pv1 * p2[1].Coord(k)
                                     + pu * pv * p3[nPolU].Coord(k) + pu1 * pv * p4[nPolV].Coord(k)));
                }
                _poles[i, j] = p;
            }
        }
    }

    public void Init(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                     NCollection_Array1<Point3D> p4, NCollection_Array1<double> w1, NCollection_Array1<double> w2,
                     NCollection_Array1<double> w3, NCollection_Array1<double> w4)
    {
        Standard_DomainError_Raise_if(w1.Length() != w3.Length() || w2.Length() != w4.Length(), " ");
        Standard_DomainError_Raise_if(w1.Length() != p1.Length() || w2.Length() != p2.Length()
                                      || w3.Length() != p3.Length() || w4.Length() != p4.Length(), " ");

        Init(p1, p2, p3, p4);
        _isRational = true;

        int nPolU = w1.Length();
        int nPolV = w2.Length();

        double nu = nPolU - 1;
        double nv = nPolV - 1;
        _weights = new NCollection_HArray2<double>(1, nPolU, 1, nPolV);
        // The boundaries are not modified
        int i, j;
        for (i = 1; i <= nPolU; i++)
        {
            _weights[i, 1] = w1[i];
            _weights[i, nPolV] = w3[i];
        }
        double pu, pu1, pv, pv1;

        for (j = 2; j <= nPolV - 1; j++)
        {
            pv = (j - 1) / nv;
            pv1 = 1 - pv;
            _weights[1, j] = w4[j];
            _weights[nPolU, j] = w2[j];
            for (i = 2; i <= nPolU - 1; i++)
            {
                pu = (i - 1) / nu;
                pu1 = 1 - pu;
                double w = 0.5 * (pv1 * w1[i] + pv * w3[i] + pu * w2[j] + pu1 * w4[j]);
                _weights[i, j] = w;
            }
        }
    }
}
