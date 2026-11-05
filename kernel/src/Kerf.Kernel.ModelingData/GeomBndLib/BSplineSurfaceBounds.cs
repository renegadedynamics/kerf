// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a B-spline surface (Geom_BSplineSurface): the convex hull of the poles that influence
/// the patch (selected through the knots), grid sampling outside the surface's bounds, sampling + PSO/Powell
/// optimization for <c>BoxOptimal</c>.
/// </summary>
internal sealed class BSplineSurfaceBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly BSplineSurface _geom;

    public BSplineSurfaceBounds(BSplineSurface surf)
    {
        _geom = surf;
    }

    public BSplineSurface Geometry() => _geom;

    /// <summary>
    /// Compute start and finish pole indexes for a given parameter range using knot-based index selection via
    /// BSplCLib::Hunt and BSplCLib::PoleIndex.
    /// </summary>
    private static void ComputePolesIndexes(NCollection_Array1<double> knots,
                                            NCollection_Array1<int> mults,
                                            int degree,
                                            double min,
                                            double max,
                                            int maxPoleIdx,
                                            bool isPeriodic,
                                            ref int outMinIdx,
                                            ref int outMaxIdx)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(knots, min, out outMinIdx);
        outMinIdx = Math.Clamp(outMinIdx, knots.Lower(), knots.Upper());

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(knots, max, out outMaxIdx);
        outMaxIdx++;
        outMaxIdx = Math.Clamp(outMaxIdx, knots.Lower(), knots.Upper());
        int multiplier = mults[outMaxIdx];

        outMinIdx = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(degree, outMinIdx, isPeriodic, mults) + 1;
        outMinIdx = Max(outMinIdx, 1);
        outMaxIdx = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(degree, outMaxIdx, isPeriodic, mults) + 1;
        outMaxIdx += degree - multiplier;
        if (!isPeriodic)
        {
            outMaxIdx = Min(outMaxIdx, maxPoleIdx);
        }
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double tol)
    {
        _geom.ParameterBounds(out double uMin, out double uMax, out double vMin, out double vMax);
        return Box(uMin, uMax, vMin, vMax, tol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        double pTol = global::Kerf.Engine.Foundation.Precision.Parametric(global::Kerf.Engine.Foundation.Precision.Confusion());

        // Get geometry bounds.
        _geom.ParameterBounds(out double uMinParam, out double uMaxParam, out double vMinParam, out double vMaxParam);

        // Check if parameters lie inside geometry bounds for convex hull applicability.
        bool isOutOfBounds = ((uMin - uMinParam) < -pTol || (vMin - vMinParam) < -pTol
                              || (uMax - uMaxParam) > pTol || (vMax - vMaxParam) > pTol);

        if (!isOutOfBounds)
        {
            // Use convex hull algorithm with poles.
            int nbUPoles = _geom.UPoleCount;
            int nbVPoles = _geom.VPoleCount;
            NCollection_Array2<Point3D> poles = _geom.InternalPoles();

            int uMinIdx = 1, uMaxIdx = nbUPoles;
            int vMinIdx = 1, vMaxIdx = nbVPoles;
            bool isUPeriodic = _geom.IsUPeriodic;
            bool isVPeriodic = _geom.IsVPeriodic;

            if (uMin > uMinParam || uMax < uMaxParam)
            {
                NCollection_Array1<int> mults = _geom.InternalUMultiplicities();
                NCollection_Array1<double> knots = _geom.InternalUKnots();
                ComputePolesIndexes(knots,
                                    mults,
                                    _geom.UDegree,
                                    uMin,
                                    uMax,
                                    nbUPoles,
                                    isUPeriodic,
                                    ref uMinIdx,
                                    ref uMaxIdx);
            }
            if (vMin > vMinParam || vMax < vMaxParam)
            {
                NCollection_Array1<int> mults = _geom.InternalVMultiplicities();
                NCollection_Array1<double> knots = _geom.InternalVKnots();
                ComputePolesIndexes(knots,
                                    mults,
                                    _geom.VDegree,
                                    vMin,
                                    vMax,
                                    nbVPoles,
                                    isVPeriodic,
                                    ref vMinIdx,
                                    ref vMaxIdx);
            }

            // Iterate over selected pole range.
            for (int i = uMinIdx; i <= uMaxIdx; i++)
            {
                int ip = i;
                if (isUPeriodic && ip > nbUPoles)
                {
                    ip = ip - nbUPoles;
                }
                for (int j = vMinIdx; j <= vMaxIdx; j++)
                {
                    int jp = j;
                    if (isVPeriodic && jp > nbVPoles)
                    {
                        jp = jp - nbVPoles;
                    }
                    box.Add(poles[ip, jp]);
                }
            }
            box.Enlarge(tol);
            return box;
        }

        // Out of geometry bounds: fall back to grid sampling.
        // OCCT samples the grid without enlarging the box by a deflection estimate (occt-issues.md).
        SurfaceAdaptor gaSurf = new(_geom);
        int nbUSamples = SamplingHelpers.ComputeNbUSamples(gaSurf);
        int nbVSamples = SamplingHelpers.ComputeNbVSamples(gaSurf);

        NCollection_Array1<double> uParams = new(1, nbUSamples);
        NCollection_Array1<double> vParams = new(1, nbVSamples);
        for (int i = 1; i <= nbUSamples; i++)
        {
            uParams.SetValue(i, uMin + ((uMax - uMin) * (i - 1) / (nbUSamples - 1)));
        }
        for (int j = 1; j <= nbVSamples; j++)
        {
            vParams.SetValue(j, vMin + ((vMax - vMin) * (j - 1) / (nbVSamples - 1)));
        }

        SurfaceGrid evaluator = new(gaSurf);
        NCollection_Array2<Point3D> grid = evaluator.EvaluateGrid(uParams, vParams);
        for (int i = grid.LowerRow(); i <= grid.UpperRow(); i++)
        {
            for (int j = grid.LowerCol(); j <= grid.UpperCol(); j++)
            {
                box.Add(grid.Value(i, j));
            }
        }
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double tol)
    {
        _geom.ParameterBounds(out double uMin, out double uMax, out double vMin, out double vMax);
        return BoxOptimal(uMin, uMax, vMin, vMax, tol);
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    /// <remarks>OCCT's body is GeomBndLib_BezierSurface::BoxOptimal's text after the adaptor; it is shared.</remarks>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        SurfaceAdaptor gaSurf = new(_geom);
        return BezierSurfaceBounds.SurfaceBoxOptimal(gaSurf, uMin, uMax, vMin, vMax, tol);
    }
}
