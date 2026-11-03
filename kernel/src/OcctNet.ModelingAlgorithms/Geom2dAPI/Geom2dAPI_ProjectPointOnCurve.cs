// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ProjectPointOnCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ProjectPointOnCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ProjectPointOnCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>All the orthogonal projections of a 2D point onto a 2D curve.</summary>
/// <remarks>
/// An algorithm object (decision 117). OCCT copies the Extrema_ExtPC2d it builds into its field; the port keeps
/// the object it builds (decision 224). OCCT's implicit conversions to int, gp_Pnt2d and double are explicit
/// operators (decision 327).
/// </remarks>
public class Geom2dAPI_ProjectPointOnCurve
{
    private bool myIsDone;
    private int myIndex;
    private ExtPC2d myExtPC = new();
    private readonly CurveAdaptor2D myC = new();

    /// <summary>An empty projector; use <see cref="Init(in Point2D, Curve2D)"/>.</summary>
    public Geom2dAPI_ProjectPointOnCurve()
    {
        myIndex = -1;
        myIsDone = false;
    }

    /// <summary>The projections of <paramref name="P"/> on <paramref name="Curve"/>.</summary>
    public Geom2dAPI_ProjectPointOnCurve(in Point2D P, Curve2D Curve)
    {
        Init(P, Curve);
    }

    /// <summary>
    /// The projections of <paramref name="P"/> on the part of <paramref name="Curve"/> between the parameters
    /// <paramref name="Umin"/> and <paramref name="Usup"/>.
    /// </summary>
    public Geom2dAPI_ProjectPointOnCurve(in Point2D P, Curve2D Curve, double Umin, double Usup)
    {
        Init(P, Curve, Umin, Usup);
    }

    /// <summary>Computes the projections of <paramref name="P"/> on <paramref name="Curve"/>.</summary>
    public void Init(in Point2D P, Curve2D Curve)
    {
        Init(P, Curve, Curve.FirstParameter(), Curve.LastParameter());
    }

    /// <summary>
    /// Computes the projections of <paramref name="P"/> on the part of <paramref name="Curve"/> between the
    /// parameters <paramref name="Umin"/> and <paramref name="Usup"/>.
    /// </summary>
    public void Init(in Point2D P, Curve2D Curve, double Umin, double Usup)
    {
        myC.Load(Curve, Umin, Usup);
        ExtPC2d theExtPC2d = new(P, myC);
        myExtPC = theExtPC2d;
        myIsDone = myExtPC.IsDone && (myExtPC.ExtCount > 0);

        // evaluate the lower distance and its index;
        if (myIsDone)
        {
            double Dist2, Dist2Min = myExtPC.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtPC.ExtCount; i++)
            {
                Dist2 = myExtPC.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>The number of projections (0 if the computation failed).</summary>
    public int NbPoints()
    {
        if (myIsDone)
        {
            return myExtPC.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The projection of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbPoints].</exception>
    public Point2D Point(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Point");
        return (myExtPC.Point(Index)).Value();
    }

    /// <summary>The parameter on the curve of the projection of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbPoints].</exception>
    public double Parameter(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Parameter");
        return (myExtPC.Point(Index)).Parameter();
    }

    /// <summary>The parameter on the curve of the projection of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbPoints].</exception>
    public void Parameter(int Index, out double U)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Parameter");
        U = (myExtPC.Point(Index)).Parameter();
    }

    /// <summary>The distance between the point and its projection of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbPoints].</exception>
    public double Distance(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Distance");
        return Math.Sqrt(myExtPC.SquareDistance(Index));
    }

    /// <summary>The nearest projection of the point on the curve.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Point2D NearestPoint()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "Geom2dAPI_ProjectPointOnCurve:NearestPoint");
        return (myExtPC.Point(myIndex)).Value();
    }

    /// <summary>The parameter on the curve of the nearest projection.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double LowerDistanceParameter()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "Geom2dAPI_ProjectPointOnCurve:LowerDistanceParameter");
        return (myExtPC.Point(myIndex)).Parameter();
    }

    /// <summary>The distance between the point and its nearest projection.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "Geom2dAPI_ProjectPointOnCurve:LowerDistance");
        return Math.Sqrt(myExtPC.SquareDistance(myIndex));
    }

    /// <summary>The Extrema algorithm used.</summary>
    public ExtPC2d Extrema() => myExtPC;

    /// <summary>OCCT's <c>operator int()</c>: <see cref="NbPoints"/>.</summary>
    public static explicit operator int(Geom2dAPI_ProjectPointOnCurve theProj) => theProj.NbPoints();

    /// <summary>OCCT's <c>operator gp_Pnt2d()</c>: <see cref="NearestPoint"/>.</summary>
    public static explicit operator Point2D(Geom2dAPI_ProjectPointOnCurve theProj) => theProj.NearestPoint();

    /// <summary>OCCT's <c>operator double()</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(Geom2dAPI_ProjectPointOnCurve theProj) => theProj.LowerDistance();
}
