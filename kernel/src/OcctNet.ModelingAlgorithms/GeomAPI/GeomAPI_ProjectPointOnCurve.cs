// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes all the orthogonal projections of a 3D point onto a 3D curve (the extrema of the distance, through
/// <see cref="CurvePointExtrema"/>), and the nearest of them.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init(Curve3D, double, double)"/> followed by several
/// <see cref="Perform"/> calls reuses the loaded curve. OCCT's implicit conversions to <c>int</c>
/// (<see cref="NbPoints"/>), <c>gp_Pnt</c> (<see cref="NearestPoint"/>) and <c>double</c>
/// (<see cref="LowerDistance"/>) are explicit conversions (decision 28).
/// </remarks>
public class GeomAPI_ProjectPointOnCurve
{
    private bool myIsDone;
    private int myIndex;
    private readonly CurvePointExtrema myExtPC = new();
    private readonly CurveAdaptor myC = new();

    /// <summary>An empty projector, to be initialised with <c>Init</c>.</summary>
    public GeomAPI_ProjectPointOnCurve()
    {
        myIsDone = false;
        myIndex = 0;
    }

    /// <summary>Projects <paramref name="P"/> onto the whole of <paramref name="Curve"/>.</summary>
    public GeomAPI_ProjectPointOnCurve(in Point3D P, Curve3D Curve)
    {
        Init(P, Curve);
    }

    /// <summary>Projects <paramref name="P"/> onto <paramref name="Curve"/> on [Umin, Usup].</summary>
    public GeomAPI_ProjectPointOnCurve(in Point3D P, Curve3D Curve, double Umin, double Usup)
    {
        Init(P, Curve, Umin, Usup);
    }

    /// <summary>Projects <paramref name="P"/> onto the whole of <paramref name="Curve"/>.</summary>
    public void Init(in Point3D P, Curve3D Curve)
    {
        myC.Load(Curve);
        myExtPC.Initialize(myC, myC.FirstParameter(), myC.LastParameter());
        myExtPC.Perform(P);

        myIsDone = myExtPC.IsDone && (myExtPC.ExtCount > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
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

    /// <summary>Projects <paramref name="P"/> onto <paramref name="Curve"/> on [Umin, Usup].</summary>
    public void Init(in Point3D P, Curve3D Curve, double Umin, double Usup)
    {
        myC.Load(Curve, Umin, Usup);
        myExtPC.Initialize(myC, myC.FirstParameter(), myC.LastParameter());
        myExtPC.Perform(P);

        myIsDone = myExtPC.IsDone && (myExtPC.ExtCount > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
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

    /// <summary>Loads <paramref name="Curve"/> on [Umin, Usup], for projections by <see cref="Perform"/>.</summary>
    public void Init(Curve3D Curve, double Umin, double Usup)
    {
        myC.Load(Curve, Umin, Usup);
        myExtPC.Initialize(myC, Umin, Usup);
        myIsDone = false;
    }

    /// <summary>Projects <paramref name="aP3D"/> onto the curve loaded by <c>Init</c>.</summary>
    public void Perform(in Point3D aP3D)
    {
        myExtPC.Perform(aP3D);

        myIsDone = myExtPC.IsDone && (myExtPC.ExtCount > 0);
        if (myIsDone)
        {
            // evaluate the lower distance and its index;
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

    /// <summary>The number of projections (0 when the computation failed).</summary>
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

    /// <summary>The projection of index <paramref name="Index"/> (1..NbPoints).</summary>
    public Point3D Point(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Point");
        return (myExtPC.Point(Index)).Value();
    }

    /// <summary>The curve parameter of the projection of index <paramref name="Index"/>.</summary>
    public double Parameter(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Parameter");
        return (myExtPC.Point(Index)).Parameter();
    }

    /// <summary>The curve parameter of the projection of index <paramref name="Index"/>.</summary>
    public void Parameter(int Index, out double U)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Parameter");
        U = (myExtPC.Point(Index)).Parameter();
    }

    /// <summary>The distance from the point to the projection of index <paramref name="Index"/>.</summary>
    public double Distance(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Distance");
        return Math.Sqrt(myExtPC.SquareDistance(Index));
    }

    /// <summary>The nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public Point3D NearestPoint()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ProjectPointOnCurve::NearestPoint");

        return (myExtPC.Point(myIndex)).Value();
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbPoints"/>.</summary>
    public static explicit operator int(GeomAPI_ProjectPointOnCurve theProj) => theProj.NbPoints();

    /// <summary>OCCT's <c>operator gp_Pnt</c>: <see cref="NearestPoint"/>.</summary>
    public static explicit operator Point3D(GeomAPI_ProjectPointOnCurve theProj) => theProj.NearestPoint();

    /// <summary>The parameter of the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistanceParameter()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ProjectPointOnCurve::LowerDistanceParameter");

        return (myExtPC.Point(myIndex)).Parameter();
    }

    /// <summary>The distance to the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ProjectPointOnCurve::LowerDistance");

        return Math.Sqrt(myExtPC.SquareDistance(myIndex));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(GeomAPI_ProjectPointOnCurve theProj) => theProj.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public CurvePointExtrema Extrema() => myExtPC;
}
