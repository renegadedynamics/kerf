// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceToBezierSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceToBezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a B-spline surface into a grid of adjacent Bezier patches by knot insertion (W. Bohm, "Generating the
/// Bezier points of B-spline curves and surfaces", CAD 13(6), 1981). A row of the grid is the series of patches
/// between two U knots, a column the series between two V knots.
/// </summary>
internal sealed class BSplineSurfaceToBezierSurface
{
    private readonly BSplineSurface _surface;

    /// <summary>Prepares the conversion of the whole of BasisSurface (which is not modified).</summary>
    public BSplineSurfaceToBezierSurface(BSplineSurface basisSurface)
    {
        _surface = (BSplineSurface)basisSurface.Copy();
        _surface.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        _surface.Segment(u1, u2, v1, v2);
        _surface.IncreaseUMultiplicity(_surface.FirstUKnotIndex(),
                                        _surface.LastUKnotIndex(),
                                        _surface.UDegree);
        _surface.IncreaseVMultiplicity(_surface.FirstVKnotIndex(),
                                        _surface.LastVKnotIndex(),
                                        _surface.VDegree);
    }

    /// <summary>
    /// Prepares the conversion of the patch [U1, U2] x [V1, V2] of BasisSurface. A bound within
    /// ParametricTolerance / 2 of a knot outside the range is moved onto that knot.
    /// </summary>
    /// <exception cref="DomainErrorException">U2 - U1 or V2 - V1 is less than ParametricTolerance.</exception>
    public BSplineSurfaceToBezierSurface(BSplineSurface basisSurface,
                                                     double u1,
                                                     double u2,
                                                     double v1,
                                                     double v2,
                                                     double parametricTolerance)
    {
        if ((u2 - u1 < parametricTolerance) || (v2 - v1 < parametricTolerance))
        {
            throw new DomainErrorException("GeomConvert_BSplineSurfaceToBezierSurface");
        }

        double uf = u1, ul = u2, vf = v1, vl = v2, pTol = parametricTolerance / 2;
        int i1, i2;

        _surface = (BSplineSurface)basisSurface.Copy();

        _surface.LocateU(u1, pTol, out i1, out i2);
        if (i1 == i2)
        { // We are on the knot
            if (_surface.UKnot(i1) > u1)
            {
                uf = _surface.UKnot(i1);
            }
        }

        _surface.LocateU(u2, pTol, out i1, out i2);
        if (i1 == i2)
        { // We are on the knot
            if (_surface.UKnot(i1) < u2)
            {
                ul = _surface.UKnot(i1);
            }
        }

        _surface.LocateV(v1, pTol, out i1, out i2);
        if (i1 == i2)
        { // We are on the knot
            if (_surface.VKnot(i1) > v1)
            {
                vf = _surface.VKnot(i1);
            }
        }

        _surface.LocateV(v2, pTol, out i1, out i2);
        if (i1 == i2)
        { // We are on the knot
            if (_surface.VKnot(i1) < v2)
            {
                vl = _surface.VKnot(i1);
            }
        }

        _surface.Segment(uf, ul, vf, vl);
        _surface.IncreaseUMultiplicity(_surface.FirstUKnotIndex(),
                                        _surface.LastUKnotIndex(),
                                        _surface.UDegree);
        _surface.IncreaseVMultiplicity(_surface.FirstVKnotIndex(),
                                        _surface.LastVKnotIndex(),
                                        _surface.VDegree);
    }

    /// <summary>A new Bezier surface: the patch (UIndex, VIndex) of the grid, oriented as the B-spline surface.</summary>
    /// <exception cref="OutOfRangeException">UIndex out of [1, NbUPatches] or VIndex out of [1, NbVPatches].</exception>
    public BezierSurface Patch(int uIndex, int vIndex)
    {
        if (uIndex < 1 || uIndex > _surface.UKnotCount - 1 || vIndex < 1
            || vIndex > _surface.VKnotCount - 1)
        {
            throw new OutOfRangeException("GeomConvert_BSplineSurfaceToBezierSurface");
        }

        int uDeg = _surface.UDegree;
        int vDeg = _surface.VDegree;

        var poles = new NCollection_Array2<Point3D>(1, uDeg + 1, 1, vDeg + 1);

        BezierSurface s;
        if (_surface.IsURational || _surface.IsVRational)
        {
            var weights = new NCollection_Array2<double>(1, uDeg + 1, 1, vDeg + 1);
            for (int i = 1; i <= uDeg + 1; i++)
            {
                int curI = i + uDeg * (uIndex - 1);
                for (int j = 1; j <= vDeg + 1; j++)
                {
                    poles[i, j] = _surface.Pole(curI, j + vDeg * (vIndex - 1));
                    weights[i, j] = _surface.Weight(curI, j + vDeg * (vIndex - 1));
                }
            }

            s = new BezierSurface(poles, weights);
        }
        else
        {
            for (int i = 1; i <= uDeg + 1; i++)
            {
                int curI = i + uDeg * (uIndex - 1);
                for (int j = 1; j <= vDeg + 1; j++)
                {
                    poles[i, j] = _surface.Pole(curI, j + vDeg * (vIndex - 1));
                }
            }

            s = new BezierSurface(poles);
        }

        return s;
    }

    /// <summary>
    /// Fills Surfaces (bounds 1..NbUPatches x 1..NbVPatches) with new Bezier surfaces, one per patch.
    /// </summary>
    public void Patches(NCollection_Array2<BezierSurface> surfaces)
    {
        int nbU = UPatchCount;
        int nbV = VPatchCount;
        for (int i = 1; i <= nbU; i++)
        {
            for (int j = 1; j <= nbV; j++)
            {
                surfaces[i, j] = Patch(i, j);
            }
        }
    }

    /// <summary>Copies the U knots bounding the patches (NbUPatches + 1 values) into TKnots from its lower bound.</summary>
    public void UKnots(NCollection_Array1<double> tKnots)
    {
        int ii, kk;
        for (ii = 1, kk = tKnots.Lower(); ii <= _surface.UKnotCount; ii++, kk++)
        {
            tKnots[kk] = _surface.UKnot(ii);
        }
    }

    /// <summary>Copies the V knots bounding the patches (NbVPatches + 1 values) into TKnots from its lower bound.</summary>
    public void VKnots(NCollection_Array1<double> tKnots)
    {
        int ii, kk;
        for (ii = 1, kk = tKnots.Lower(); ii <= _surface.VKnotCount; ii++, kk++)
        {
            tKnots[kk] = _surface.VKnot(ii);
        }
    }

    /// <summary>The number of Bezier patches in the U direction.</summary>
    public int UPatchCount => (_surface.UKnotCount - 1);

    /// <summary>The number of Bezier patches in the V direction.</summary>
    public int VPatchCount => (_surface.VKnotCount - 1);
}
