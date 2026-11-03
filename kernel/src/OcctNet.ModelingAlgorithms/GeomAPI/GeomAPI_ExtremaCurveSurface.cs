// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveSurface.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes the extrema of the distance between a 3D curve and a surface (through <see cref="Extrema_ExtCS"/>), and
/// the nearest pair among them.
/// </summary>
/// <remarks>
/// A working object (decision 21). The adaptors are made by <c>Init</c>, as OCCT's locals; the extrema algorithm
/// keeps the surface adaptor (in OCCT a pointer to a destroyed local). OCCT's implicit conversions to <c>int</c>
/// and <c>double</c> are explicit conversions (decision 28).
/// </remarks>
public class GeomAPI_ExtremaCurveSurface
{
    private bool myIsDone;
    private int myIndex;
    private readonly Extrema_ExtCS myExtCS = new();

    /// <summary>An empty algorithm, to be initialised with <c>Init</c>.</summary>
    public GeomAPI_ExtremaCurveSurface()
    {
        myIsDone = false;
        myIndex = 0;
    }

    /// <summary>The extrema between the whole <paramref name="Curve"/> and <paramref name="Surface"/>.</summary>
    public GeomAPI_ExtremaCurveSurface(Geom_Curve Curve, Geom_Surface Surface)
    {
        Init(Curve, Surface);
    }

    /// <summary>
    /// The extrema between <paramref name="Curve"/> on [Wmin, Wmax] and <paramref name="Surface"/> on
    /// [Umin, Umax] x [Vmin, Vmax].
    /// </summary>
    public GeomAPI_ExtremaCurveSurface(Geom_Curve Curve, Geom_Surface Surface, double Wmin, double Wmax, double Umin,
                                       double Umax, double Vmin, double Vmax)
    {
        Init(Curve, Surface, Wmin, Wmax, Umin, Umax, Vmin, Vmax);
    }

    /// <summary>Computes the extrema between the whole <paramref name="Curve"/> and <paramref name="Surface"/>.</summary>
    public void Init(Geom_Curve Curve, Geom_Surface Surface)
    {
        GeomAdaptor_Curve TheCurve = new(Curve);
        GeomAdaptor_Surface TheSurface = new(Surface);

        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myExtCS.Initialize(TheSurface, Tol, Tol);
        myExtCS.Perform(TheCurve, TheCurve.FirstParameter(), TheCurve.LastParameter());
        myIsDone = myExtCS.IsDone() && (myExtCS.IsParallel() || myExtCS.NbExt() > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtCS.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtCS.NbExt(); i++)
            {
                Dist2 = myExtCS.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>
    /// Computes the extrema between <paramref name="Curve"/> on [Wmin, Wmax] and <paramref name="Surface"/> on
    /// [Umin, Umax] x [Vmin, Vmax].
    /// </summary>
    public void Init(Geom_Curve Curve, Geom_Surface Surface, double Wmin, double Wmax, double Umin, double Umax,
                     double Vmin, double Vmax)
    {
        GeomAdaptor_Curve TheCurve = new(Curve, Wmin, Wmax);
        GeomAdaptor_Surface TheSurface = new(Surface, Umin, Umax, Vmin, Vmax);

        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myExtCS.Initialize(TheSurface, Umin, Umax, Vmin, Vmax, Tol, Tol);
        myExtCS.Perform(TheCurve, Wmin, Wmax);
        myIsDone = myExtCS.IsDone() && (myExtCS.IsParallel() || myExtCS.NbExt() > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtCS.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtCS.NbExt(); i++)
            {
                Dist2 = myExtCS.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>The number of extrema (0 when the computation failed).</summary>
    public int NbExtrema()
    {
        if (myIsDone)
        {
            return myExtCS.NbExt();
        }
        else
        {
            return 0;
        }
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbExtrema"/>.</summary>
    public static explicit operator int(GeomAPI_ExtremaCurveSurface theExt) => theExt.NbExtrema();

    /// <summary>The points of the extremum of index <paramref name="Index"/>, on the curve and on the surface.</summary>
    public void Points(int Index, out Point3D P1, out Point3D P2)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Points");

        myExtCS.Points(Index, out Extrema_POnCurv PC, out Extrema_POnSurf PS);

        P1 = PC.Value();
        P2 = PS.Value();
    }

    /// <summary>The curve parameter and surface parameters of the extremum of index <paramref name="Index"/>.</summary>
    public void Parameters(int Index, out double W, out double U, out double V)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Parameters");

        myExtCS.Points(Index, out Extrema_POnCurv PC, out Extrema_POnSurf PS);

        W = PC.Parameter();
        PS.Parameter(out U, out V);
    }

    /// <summary>The distance of the extremum of index <paramref name="Index"/>.</summary>
    public double Distance(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Distance");

        return Math.Sqrt(myExtCS.SquareDistance(Index));
    }

    /// <summary>True if the curve lies on a surface parallel to the surface.</summary>
    public bool IsParallel() => myExtCS.IsParallel();

    /// <summary>The points of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void NearestPoints(out Point3D PC, out Point3D PS)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaCurveSurface::NearestPoints");

        Points(myIndex, out PC, out PS);
    }

    /// <summary>The parameters of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double W, out double U, out double V)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaCurveSurface::LowerDistanceParameters");

        Parameters(myIndex, out W, out U, out V);
    }

    /// <summary>The distance of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaCurveSurface::LowerDistance");

        return Math.Sqrt(myExtCS.SquareDistance(myIndex));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(GeomAPI_ExtremaCurveSurface theExt) => theExt.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public Extrema_ExtCS Extrema() => myExtCS;
}
