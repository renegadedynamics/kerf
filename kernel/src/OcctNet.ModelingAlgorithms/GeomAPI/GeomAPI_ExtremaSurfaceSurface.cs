// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaSurfaceSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaSurfaceSurface.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaSurfaceSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes the extrema of the distance between two surfaces (through <see cref="Extrema_ExtSS"/>), and the nearest
/// pair among them.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT constructs an <c>Extrema_ExtSS</c> in each <c>Init</c> and copies it into
/// its member; the port keeps the constructed object (decision 224). OCCT's implicit conversions to <c>int</c> and
/// <c>double</c> are explicit conversions (decision 28).
/// </remarks>
public class GeomAPI_ExtremaSurfaceSurface
{
    private bool myIsDone;
    private int myIndex;
    private Extrema_ExtSS myExtSS = new();

    /// <summary>An empty algorithm, to be initialised with <c>Init</c>.</summary>
    public GeomAPI_ExtremaSurfaceSurface()
    {
        myIsDone = false;
        myIndex = 0;
    }

    /// <summary>The extrema between the whole surfaces <paramref name="S1"/> and <paramref name="S2"/>.</summary>
    public GeomAPI_ExtremaSurfaceSurface(Geom_Surface S1, Geom_Surface S2)
    {
        Init(S1, S2);
    }

    /// <summary>The extrema between <paramref name="S1"/> and <paramref name="S2"/> on the given ranges.</summary>
    public GeomAPI_ExtremaSurfaceSurface(Geom_Surface S1, Geom_Surface S2, double U1min, double U1max, double V1min,
                                         double V1max, double U2min, double U2max, double V2min, double V2max)
    {
        Init(S1, S2, U1min, U1max, V1min, V1max, U2min, U2max, V2min, V2max);
    }

    /// <summary>Computes the extrema between the whole surfaces <paramref name="S1"/> and <paramref name="S2"/>.</summary>
    public void Init(Geom_Surface S1, Geom_Surface S2)
    {
        GeomAdaptor_Surface TheSurface1 = new(S1);
        GeomAdaptor_Surface TheSurface2 = new(S2);

        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        Extrema_ExtSS theExtSS = new(TheSurface1, TheSurface2, Tol, Tol);

        myExtSS = theExtSS;
        myIsDone = myExtSS.IsDone() && (myExtSS.NbExt() > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtSS.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtSS.NbExt(); i++)
            {
                Dist2 = myExtSS.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>Computes the extrema between <paramref name="S1"/> and <paramref name="S2"/> on the given ranges.</summary>
    public void Init(Geom_Surface S1, Geom_Surface S2, double U1min, double U1max, double V1min, double V1max,
                     double U2min, double U2max, double V2min, double V2max)
    {
        GeomAdaptor_Surface TheSurface1 = new(S1, U1min, U1max, V1min, V1max);
        GeomAdaptor_Surface TheSurface2 = new(S2, U2min, U2max, V2min, V2max);

        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Extrema_ExtSS theExtSS = new(TheSurface1, TheSurface2, U1min, U1max, V1min, V1max, U2min, U2max, V2min, V2max,
                                     Tol, Tol);

        myExtSS = theExtSS;
        myIsDone = myExtSS.IsDone() && (myExtSS.NbExt() > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtSS.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtSS.NbExt(); i++)
            {
                Dist2 = myExtSS.SquareDistance(i);
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
            return myExtSS.NbExt();
        }
        else
        {
            return 0;
        }
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbExtrema"/>.</summary>
    public static explicit operator int(GeomAPI_ExtremaSurfaceSurface theExt) => theExt.NbExtrema();

    /// <summary>The points of the extremum of index <paramref name="Index"/>, on the first and second surface.</summary>
    public void Points(int Index, out Point3D P1, out Point3D P2)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Points");

        myExtSS.Points(Index, out Extrema_POnSurf PS1, out Extrema_POnSurf PS2);

        P1 = PS1.Value();
        P2 = PS2.Value();
    }

    /// <summary>The parameters of the extremum of index <paramref name="Index"/> on the two surfaces.</summary>
    public void Parameters(int Index, out double U1, out double V1, out double U2, out double V2)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Parameters");

        myExtSS.Points(Index, out Extrema_POnSurf PS1, out Extrema_POnSurf PS2);

        PS1.Parameter(out U1, out V1);
        PS2.Parameter(out U2, out V2);
    }

    /// <summary>The distance of the extremum of index <paramref name="Index"/>.</summary>
    public double Distance(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Distance");

        return Math.Sqrt(myExtSS.SquareDistance(Index));
    }

    /// <summary>True if the surfaces are parallel.</summary>
    public bool IsParallel() => myExtSS.IsParallel();

    /// <summary>The points of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void NearestPoints(out Point3D P1, out Point3D P2)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaSurfaceSurface::NearestPoints");

        Points(myIndex, out P1, out P2);
    }

    /// <summary>The parameters of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double U1, out double V1, out double U2, out double V2)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaSurfaceSurface::LowerDistanceParameters");

        Parameters(myIndex, out U1, out V1, out U2, out V2);
    }

    /// <summary>The distance of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaSurfaceSurface::LowerDistance");

        return Math.Sqrt(myExtSS.SquareDistance(myIndex));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(GeomAPI_ExtremaSurfaceSurface theExt) => theExt.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public Extrema_ExtSS Extrema() => myExtSS;
}
