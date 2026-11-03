// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceToBezierSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceToBezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Converts a B-spline surface into a grid of adjacent Bezier patches by knot insertion (W. Bohm, "Generating the
/// Bezier points of B-spline curves and surfaces", CAD 13(6), 1981). A row of the grid is the series of patches
/// between two U knots, a column the series between two V knots.
/// </summary>
public sealed class GeomConvert_BSplineSurfaceToBezierSurface
{
    private readonly Geom_BSplineSurface mySurface;

    /// <summary>Prepares the conversion of the whole of BasisSurface (which is not modified).</summary>
    public GeomConvert_BSplineSurfaceToBezierSurface(Geom_BSplineSurface BasisSurface)
    {
        mySurface = (Geom_BSplineSurface)BasisSurface.Copy();
        mySurface.Bounds(out double U1, out double U2, out double V1, out double V2);
        mySurface.Segment(U1, U2, V1, V2);
        mySurface.IncreaseUMultiplicity(mySurface.FirstUKnotIndex(),
                                        mySurface.LastUKnotIndex(),
                                        mySurface.UDegree());
        mySurface.IncreaseVMultiplicity(mySurface.FirstVKnotIndex(),
                                        mySurface.LastVKnotIndex(),
                                        mySurface.VDegree());
    }

    /// <summary>
    /// Prepares the conversion of the patch [U1, U2] x [V1, V2] of BasisSurface. A bound within
    /// ParametricTolerance / 2 of a knot outside the range is moved onto that knot.
    /// </summary>
    /// <exception cref="DomainErrorException">U2 - U1 or V2 - V1 is less than ParametricTolerance.</exception>
    public GeomConvert_BSplineSurfaceToBezierSurface(Geom_BSplineSurface BasisSurface,
                                                     double U1,
                                                     double U2,
                                                     double V1,
                                                     double V2,
                                                     double ParametricTolerance)
    {
        if ((U2 - U1 < ParametricTolerance) || (V2 - V1 < ParametricTolerance))
        {
            throw new DomainErrorException("GeomConvert_BSplineSurfaceToBezierSurface");
        }

        double Uf = U1, Ul = U2, Vf = V1, Vl = V2, PTol = ParametricTolerance / 2;
        int I1, I2;

        mySurface = (Geom_BSplineSurface)BasisSurface.Copy();

        mySurface.LocateU(U1, PTol, out I1, out I2);
        if (I1 == I2)
        { // We are on the knot
            if (mySurface.UKnot(I1) > U1)
            {
                Uf = mySurface.UKnot(I1);
            }
        }

        mySurface.LocateU(U2, PTol, out I1, out I2);
        if (I1 == I2)
        { // We are on the knot
            if (mySurface.UKnot(I1) < U2)
            {
                Ul = mySurface.UKnot(I1);
            }
        }

        mySurface.LocateV(V1, PTol, out I1, out I2);
        if (I1 == I2)
        { // We are on the knot
            if (mySurface.VKnot(I1) > V1)
            {
                Vf = mySurface.VKnot(I1);
            }
        }

        mySurface.LocateV(V2, PTol, out I1, out I2);
        if (I1 == I2)
        { // We are on the knot
            if (mySurface.VKnot(I1) < V2)
            {
                Vl = mySurface.VKnot(I1);
            }
        }

        mySurface.Segment(Uf, Ul, Vf, Vl);
        mySurface.IncreaseUMultiplicity(mySurface.FirstUKnotIndex(),
                                        mySurface.LastUKnotIndex(),
                                        mySurface.UDegree());
        mySurface.IncreaseVMultiplicity(mySurface.FirstVKnotIndex(),
                                        mySurface.LastVKnotIndex(),
                                        mySurface.VDegree());
    }

    /// <summary>A new Bezier surface: the patch (UIndex, VIndex) of the grid, oriented as the B-spline surface.</summary>
    /// <exception cref="OutOfRangeException">UIndex out of [1, NbUPatches] or VIndex out of [1, NbVPatches].</exception>
    public Geom_BezierSurface Patch(int UIndex, int VIndex)
    {
        if (UIndex < 1 || UIndex > mySurface.NbUKnots() - 1 || VIndex < 1
            || VIndex > mySurface.NbVKnots() - 1)
        {
            throw new OutOfRangeException("GeomConvert_BSplineSurfaceToBezierSurface");
        }

        int UDeg = mySurface.UDegree();
        int VDeg = mySurface.VDegree();

        var Poles = new NCollection_Array2<Point3D>(1, UDeg + 1, 1, VDeg + 1);

        Geom_BezierSurface S;
        if (mySurface.IsURational() || mySurface.IsVRational())
        {
            var Weights = new NCollection_Array2<double>(1, UDeg + 1, 1, VDeg + 1);
            for (int i = 1; i <= UDeg + 1; i++)
            {
                int CurI = i + UDeg * (UIndex - 1);
                for (int j = 1; j <= VDeg + 1; j++)
                {
                    Poles[i, j] = mySurface.Pole(CurI, j + VDeg * (VIndex - 1));
                    Weights[i, j] = mySurface.Weight(CurI, j + VDeg * (VIndex - 1));
                }
            }

            S = new Geom_BezierSurface(Poles, Weights);
        }
        else
        {
            for (int i = 1; i <= UDeg + 1; i++)
            {
                int CurI = i + UDeg * (UIndex - 1);
                for (int j = 1; j <= VDeg + 1; j++)
                {
                    Poles[i, j] = mySurface.Pole(CurI, j + VDeg * (VIndex - 1));
                }
            }

            S = new Geom_BezierSurface(Poles);
        }

        return S;
    }

    /// <summary>
    /// Fills Surfaces (bounds 1..NbUPatches x 1..NbVPatches) with new Bezier surfaces, one per patch.
    /// </summary>
    public void Patches(NCollection_Array2<Geom_BezierSurface> Surfaces)
    {
        int NbU = NbUPatches();
        int NbV = NbVPatches();
        for (int i = 1; i <= NbU; i++)
        {
            for (int j = 1; j <= NbV; j++)
            {
                Surfaces[i, j] = Patch(i, j);
            }
        }
    }

    /// <summary>Copies the U knots bounding the patches (NbUPatches + 1 values) into TKnots from its lower bound.</summary>
    public void UKnots(NCollection_Array1<double> TKnots)
    {
        int ii, kk;
        for (ii = 1, kk = TKnots.Lower(); ii <= mySurface.NbUKnots(); ii++, kk++)
        {
            TKnots[kk] = mySurface.UKnot(ii);
        }
    }

    /// <summary>Copies the V knots bounding the patches (NbVPatches + 1 values) into TKnots from its lower bound.</summary>
    public void VKnots(NCollection_Array1<double> TKnots)
    {
        int ii, kk;
        for (ii = 1, kk = TKnots.Lower(); ii <= mySurface.NbVKnots(); ii++, kk++)
        {
            TKnots[kk] = mySurface.VKnot(ii);
        }
    }

    /// <summary>The number of Bezier patches in the U direction.</summary>
    public int NbUPatches() => (mySurface.NbUKnots() - 1);

    /// <summary>The number of Bezier patches in the V direction.</summary>
    public int NbVPatches() => (mySurface.NbVKnots() - 1);
}
