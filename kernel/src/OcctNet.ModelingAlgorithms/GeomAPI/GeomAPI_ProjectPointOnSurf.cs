// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnSurf.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.Extrema_ExtAlgo;

namespace Occt;

/// <summary>
/// Computes all the orthogonal projections of a point onto a surface (the extrema of the distance, through
/// <see cref="Extrema_ExtPS"/>), and the nearest of them.
/// </summary>
/// <remarks>
/// A working object (decision 21). Its <see cref="Extrema_ExtPS"/> is created once and kept: each <c>Init</c>
/// loads the surface into the projector's own <see cref="GeomAdaptor_Surface"/> (which the ExtPS keeps by
/// reference), sets the algorithm and initialises; a flag set with <see cref="SetExtremaFlag"/> is kept across
/// <c>Init</c>, as in OCCT. OCCT's implicit conversions to <c>int</c>, <c>gp_Pnt</c> and <c>double</c> are explicit
/// conversions (decision 28).
/// </remarks>
public class GeomAPI_ProjectPointOnSurf
{
    private bool myIsDone;
    private int myIndex;
    private readonly Extrema_ExtPS myExtPS = new();
    private readonly GeomAdaptor_Surface myGeomAdaptor = new();

    /// <summary>An empty projector, to be initialised with <c>Init</c>.</summary>
    public GeomAPI_ProjectPointOnSurf()
    {
        myIsDone = false;
        myIndex = 0;
    }

    /// <summary>Projects <paramref name="P"/> onto the whole of <paramref name="Surface"/>.</summary>
    public GeomAPI_ProjectPointOnSurf(in Point3D P, Geom_Surface Surface, Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        Init(P, Surface, theProjAlgo);
    }

    /// <summary>Projects <paramref name="P"/> onto the whole of <paramref name="Surface"/> with a tolerance.</summary>
    public GeomAPI_ProjectPointOnSurf(in Point3D P, Geom_Surface Surface, double Tolerance,
                                      Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        Init(P, Surface, Tolerance, theProjAlgo);
    }

    /// <summary>Projects <paramref name="P"/> onto <paramref name="Surface"/> on [Umin, Usup] x [Vmin, Vsup].</summary>
    public GeomAPI_ProjectPointOnSurf(in Point3D P, Geom_Surface Surface, double Umin, double Usup, double Vmin,
                                      double Vsup, Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        Init(P, Surface, Umin, Usup, Vmin, Vsup, theProjAlgo);
    }

    /// <summary>
    /// Projects <paramref name="P"/> onto <paramref name="Surface"/> on [Umin, Usup] x [Vmin, Vsup] with a tolerance.
    /// </summary>
    public GeomAPI_ProjectPointOnSurf(in Point3D P, Geom_Surface Surface, double Umin, double Usup, double Vmin,
                                      double Vsup, double Tolerance, Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        Init(P, Surface, Umin, Usup, Vmin, Vsup, Tolerance, theProjAlgo);
    }

    private void Init()
    {
        myIsDone = myExtPS.IsDone() && (myExtPS.NbExt() > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtPS.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtPS.NbExt(); i++)
            {
                Dist2 = myExtPS.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>Projects <paramref name="P"/> onto the whole of <paramref name="Surface"/>.</summary>
    public void Init(in Point3D P, Geom_Surface Surface, Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        Init(P, Surface, global::Kerf.Engine.Foundation.Precision.Confusion(), theProjAlgo);
    }

    /// <summary>Projects <paramref name="P"/> onto the whole of <paramref name="Surface"/> with a tolerance.</summary>
    public void Init(in Point3D P, Geom_Surface Surface, double Tolerance, Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        Surface.Bounds(out double Umin, out double Usup, out double Vmin, out double Vsup);
        myGeomAdaptor.Load(Surface, Umin, Usup, Vmin, Vsup);
        myExtPS.SetAlgo(theProjAlgo);
        myExtPS.Initialize(myGeomAdaptor, Umin, Usup, Vmin, Vsup, Tolerance, Tolerance);
        myExtPS.Perform(P);
        Init();
    }

    /// <summary>Projects <paramref name="P"/> onto <paramref name="Surface"/> on [Umin, Usup] x [Vmin, Vsup].</summary>
    public void Init(in Point3D P, Geom_Surface Surface, double Umin, double Usup, double Vmin, double Vsup,
                     Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        double Tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myGeomAdaptor.Load(Surface, Umin, Usup, Vmin, Vsup);
        myExtPS.SetAlgo(theProjAlgo);
        myExtPS.Initialize(myGeomAdaptor, Umin, Usup, Vmin, Vsup, Tolerance, Tolerance);
        myExtPS.Perform(P);
        Init();
    }

    /// <summary>
    /// Projects <paramref name="P"/> onto <paramref name="Surface"/> on [Umin, Usup] x [Vmin, Vsup] with a tolerance.
    /// </summary>
    public void Init(in Point3D P, Geom_Surface Surface, double Umin, double Usup, double Vmin, double Vsup,
                     double Tolerance, Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        myGeomAdaptor.Load(Surface, Umin, Usup, Vmin, Vsup);
        myExtPS.SetAlgo(theProjAlgo);
        myExtPS.Initialize(myGeomAdaptor, Umin, Usup, Vmin, Vsup, Tolerance, Tolerance);
        myExtPS.Perform(P);
        Init();
    }

    /// <summary>Loads <paramref name="Surface"/> on [Umin, Usup] x [Vmin, Vsup], for projections by <see cref="Perform"/>.</summary>
    public void Init(Geom_Surface Surface, double Umin, double Usup, double Vmin, double Vsup,
                     Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        double Tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myGeomAdaptor.Load(Surface, Umin, Usup, Vmin, Vsup);
        myExtPS.SetAlgo(theProjAlgo);
        myExtPS.Initialize(myGeomAdaptor, Umin, Usup, Vmin, Vsup, Tolerance, Tolerance);
        myIsDone = false;
    }

    /// <summary>
    /// Loads <paramref name="Surface"/> on [Umin, Usup] x [Vmin, Vsup] with a tolerance, for projections by
    /// <see cref="Perform"/>.
    /// </summary>
    public void Init(Geom_Surface Surface, double Umin, double Usup, double Vmin, double Vsup, double Tolerance,
                     Extrema_ExtAlgo theProjAlgo = Extrema_ExtAlgo_Grad)
    {
        myGeomAdaptor.Load(Surface, Umin, Usup, Vmin, Vsup);
        myExtPS.SetAlgo(theProjAlgo);
        myExtPS.Initialize(myGeomAdaptor, Umin, Usup, Vmin, Vsup, Tolerance, Tolerance);
        myIsDone = false;
    }

    /// <summary>Sets the extrema algorithm (gradient or tree) for the next <c>Init</c>.</summary>
    public void SetExtremaAlgo(Extrema_ExtAlgo theAlgo) => myExtPS.SetAlgo(theAlgo);

    /// <summary>Sets which extrema to compute (minima, maxima or both); kept across <c>Init</c>.</summary>
    public void SetExtremaFlag(Extrema_ExtFlag theExtFlag) => myExtPS.SetFlag(theExtFlag);

    /// <summary>Projects <paramref name="P"/> onto the surface loaded by <c>Init</c>.</summary>
    public void Perform(in Point3D P)
    {
        myExtPS.Perform(P);
        Init();
    }

    /// <summary>True if at least one projection was found.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The number of projections (0 when the computation failed).</summary>
    public int NbPoints()
    {
        if (myIsDone)
        {
            return myExtPS.NbExt();
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The projection of index <paramref name="Index"/> (1..NbPoints).</summary>
    public Point3D Point(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "GeomAPI_ProjectPointOnSurf::Point");
        return (myExtPS.Point(Index)).Value();
    }

    /// <summary>The surface parameters of the projection of index <paramref name="Index"/>.</summary>
    public void Parameters(int Index, out double U, out double V)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "GeomAPI_ProjectPointOnSurf::Parameter");
        (myExtPS.Point(Index)).Parameter(out U, out V);
    }

    /// <summary>The distance from the point to the projection of index <paramref name="Index"/>.</summary>
    public double Distance(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "GeomAPI_ProjectPointOnSurf::Distance");
        return Math.Sqrt(myExtPS.SquareDistance(Index));
    }

    /// <summary>The nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public Point3D NearestPoint()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ProjectPointOnSurf::NearestPoint");

        return (myExtPS.Point(myIndex)).Value();
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbPoints"/>.</summary>
    public static explicit operator int(GeomAPI_ProjectPointOnSurf theProj) => theProj.NbPoints();

    /// <summary>OCCT's <c>operator gp_Pnt</c>: <see cref="NearestPoint"/>.</summary>
    public static explicit operator Point3D(GeomAPI_ProjectPointOnSurf theProj) => theProj.NearestPoint();

    /// <summary>The parameters of the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double U, out double V)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ProjectPointOnSurf::LowerDistanceParameters");

        (myExtPS.Point(myIndex)).Parameter(out U, out V);
    }

    /// <summary>The distance to the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ProjectPointOnSurf::LowerDistance");

        return Math.Sqrt(myExtPS.SquareDistance(myIndex));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(GeomAPI_ProjectPointOnSurf theProj) => theProj.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public Extrema_ExtPS Extrema() => myExtPS;
}
