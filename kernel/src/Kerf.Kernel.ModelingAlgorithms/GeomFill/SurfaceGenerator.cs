// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Generator.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Generator.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Generator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A surface through generating curves: a B-spline surface passing through all the curves given to the profiler,
/// of degree 1 in V (a ruled surface between consecutive curves).
/// </summary>
/// <remarks>A working object (decision 21). <see cref="Surface"/> hands out the surface frozen.</remarks>
public class SurfaceGenerator : Profiler
{
    private Surface? _surface;

    public SurfaceGenerator()
    {
    }

    /// <summary>
    /// Converts all curves to B-spline curves, sets them to the common profile and computes the surface (V degree
    /// 1). <paramref name="pTol"/> is used to compare two knots.
    /// </summary>
    public override void Perform(double pTol)
    {
        // Perform the profile of the sections.
        base.Perform(pTol);

        // Create the surface.

        int i, j;
        int nbUPoles = NbPoles();
        int nbVPoles = _sequence.Length();
        int nbUKnots = NbKnots();
        int nbVKnots = nbVPoles;
        bool isUPeriodic = IsPeriodic;
        bool isVPeriodic = false;

        NCollection_Array2<Point3D> Poles = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array2<double> Weights = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array1<double> uKnots = new(1, nbUKnots);
        NCollection_Array1<double> vKnots = new(1, nbVKnots);
        NCollection_Array1<int> uMults = new(1, nbUKnots);
        NCollection_Array1<int> vMults = new(1, nbVKnots);
        vMults.Init(1);

        vMults[1] = vMults[nbVKnots] = 2;

        KnotsAndMults(uKnots, uMults);

        for (j = 1; j <= nbVPoles; j++)
        {
            BSplineCurve3D cj = (BSplineCurve3D)_sequence[j];
            NCollection_Array1<Point3D> poles = cj.InternalPoles();
            NCollection_Array1<double> weights = cj.InternalWeightsArray();
            vKnots[j] = (double)(j - 1);
            for (i = 1; i <= nbUPoles; i++)
            {
                Poles[i, j] = poles[i];
                Weights[i, j] = weights[i];
            }
        }

        _surface = new BSplineSurface(Poles, Weights, uKnots, vKnots, uMults, vMults, Degree(), 1,
                                            isUPeriodic, isVPeriodic);
    }

    /// <summary>The surface (null before <see cref="Perform"/>), frozen.</summary>
    public Surface? Surface()
    {
        _surface?.Freeze();
        return _surface;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal Surface? InternalSurface() => _surface;
}
