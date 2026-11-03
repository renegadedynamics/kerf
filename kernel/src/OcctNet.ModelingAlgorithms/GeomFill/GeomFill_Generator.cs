// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Generator.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Generator.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Generator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A surface through generating curves: a B-spline surface passing through all the curves given to the profiler,
/// of degree 1 in V (a ruled surface between consecutive curves).
/// </summary>
/// <remarks>A working object (decision 21). <see cref="Surface"/> hands out the surface frozen.</remarks>
public class GeomFill_Generator : GeomFill_Profiler
{
    private Geom_Surface? mySurface;

    public GeomFill_Generator()
    {
    }

    /// <summary>
    /// Converts all curves to B-spline curves, sets them to the common profile and computes the surface (V degree
    /// 1). <paramref name="PTol"/> is used to compare two knots.
    /// </summary>
    public override void Perform(double PTol)
    {
        // Perform the profile of the sections.
        base.Perform(PTol);

        // Create the surface.

        int i, j;
        int NbUPoles = NbPoles();
        int NbVPoles = mySequence.Length();
        int NbUKnots = NbKnots();
        int NbVKnots = NbVPoles;
        bool isUPeriodic = IsPeriodic();
        bool isVPeriodic = false;

        NCollection_Array2<Point3D> Poles = new(1, NbUPoles, 1, NbVPoles);
        NCollection_Array2<double> Weights = new(1, NbUPoles, 1, NbVPoles);
        NCollection_Array1<double> UKnots = new(1, NbUKnots);
        NCollection_Array1<double> VKnots = new(1, NbVKnots);
        NCollection_Array1<int> UMults = new(1, NbUKnots);
        NCollection_Array1<int> VMults = new(1, NbVKnots);
        VMults.Init(1);

        VMults[1] = VMults[NbVKnots] = 2;

        KnotsAndMults(UKnots, UMults);

        for (j = 1; j <= NbVPoles; j++)
        {
            Geom_BSplineCurve Cj = (Geom_BSplineCurve)mySequence[j];
            NCollection_Array1<Point3D> aPoles = Cj.InternalPoles();
            NCollection_Array1<double> aWeights = Cj.InternalWeightsArray();
            VKnots[j] = (double)(j - 1);
            for (i = 1; i <= NbUPoles; i++)
            {
                Poles[i, j] = aPoles[i];
                Weights[i, j] = aWeights[i];
            }
        }

        mySurface = new Geom_BSplineSurface(Poles, Weights, UKnots, VKnots, UMults, VMults, Degree(), 1,
                                            isUPeriodic, isVPeriodic);
    }

    /// <summary>The surface (null before <see cref="Perform"/>), frozen.</summary>
    public Geom_Surface? Surface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal Geom_Surface? InternalSurface() => mySurface;
}
