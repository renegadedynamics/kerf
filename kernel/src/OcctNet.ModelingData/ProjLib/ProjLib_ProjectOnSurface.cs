// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnSurface.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Projects a curve on a surface along the surface normals. The result (a 3D curve) is an approximation.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Load</c> is public. Keeps the caller's adaptors by reference, as OCCT keeps
/// its handles. <see cref="BSpline"/> freezes the stored result and returns the same object at each call
/// (decision 243). As in OCCT, the result's poles are not the computed ones (occt-issues.md): <c>Load</c> copies
/// each Bezier segment's poles into the wrong array, so the poles after the first degree are the origin.
/// </remarks>
public class ProjLib_ProjectOnSurface
{
    private Adaptor3d_Curve? myCurve;
    private Adaptor3d_Surface? mySurface;
    private double myTolerance;
    private bool myIsDone;
    private Geom_BSplineCurve? myResult;

    private static Point3D OnSurface_Value(double U, Adaptor3d_Curve myCurve, Extrema_ExtPS myExtPS)
    {
        // Try to find the closest solution point.
        myExtPS.Perform(myCurve.Value(U));

        double Dist2Min = RealLast();
        int Index = 0;

        for (int i = 1; i <= myExtPS.NbExt(); i++)
        {
            if (myExtPS.SquareDistance(i) < Dist2Min)
            {
                Index = i;
                Dist2Min = myExtPS.SquareDistance(Index);
            }
        }
        if (Index == 0)
        {
            // OCCT also prints " Extrema non trouve pour U = " to std::cout.
            return new Point3D(0.0, 0.0, 0.0);
        }
        else
        {
            return (myExtPS.Point(Index)).Value();
        }
    }

    private static bool OnSurface_D1(double U, Adaptor3d_Curve myCurve, Extrema_ExtPS myExtPS)
    {
        _ = U;
        _ = myCurve;
        _ = myExtPS;
        return false;
    }

    /// <summary>OCCT's file-local class ProjLib_OnSurface: the foot of the curve's point on the surface.</summary>
    private sealed class ProjLib_OnSurface : AppCont_Function
    {
        private readonly Adaptor3d_Curve myCurve;
        private readonly Extrema_ExtPS myExtPS;

        public ProjLib_OnSurface(Adaptor3d_Curve C, Adaptor3d_Surface S)
        {
            myCurve = C;
            myNbPnt = 1;
            myNbPnt2d = 0;
            double U = myCurve.FirstParameter();
            Point3D P = myCurve.Value(U);
            double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
            myExtPS = new Extrema_ExtPS(P, S, Tol, Tol);
        }

        public override double FirstParameter() => myCurve.FirstParameter();

        public override double LastParameter() => myCurve.LastParameter();

        public override bool Value(double theT, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            thePnt[1] = OnSurface_Value(theT, myCurve, myExtPS);
            return true;
        }

        public override bool D1(double theT, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            return OnSurface_D1(theT, myCurve, myExtPS);
        }
    }

    //=====================================================================//
    //                                                                     //
    //  D E S C R I P T I O N   O F   T H E   C L A S S  :                 //
    //                                                                     //
    //         P r o j L i b _ A p p r o x P r o j e c t O n P l a n e     //
    //                                                                     //
    //=====================================================================//

    /// <summary>An empty projector.</summary>
    public ProjLib_ProjectOnSurface()
    {
        myTolerance = 0.0;
        myIsDone = false;
    }

    /// <summary>A projector normally to the surface <paramref name="S"/>.</summary>
    public ProjLib_ProjectOnSurface(Adaptor3d_Surface S)
    {
        myTolerance = 0.0;
        myIsDone = false;
        mySurface = S;
    }

    /// <summary>Sets the surface; the curve has to be loaded to compute the projection.</summary>
    public void Load(Adaptor3d_Surface S)
    {
        mySurface = S;
        myIsDone = false;
    }

    /// <summary>Computes the projection of <paramref name="C"/> on the surface.</summary>
    public void Load(Adaptor3d_Curve C, double Tolerance)
    {
        myTolerance = Tolerance;
        myCurve = C;
        myIsDone = false;
        if (mySurface is not null)
        {
            ProjLib_OnSurface F = new(myCurve, mySurface);

            int Deg1, Deg2;
            Deg1 = 8;
            Deg2 = 8;

            Approx_FitAndDivide Fit = new(F, Deg1, Deg2, global::Kerf.Engine.Foundation.Precision.Approximation(), global::Kerf.Engine.Foundation.Precision.PApproximation(), true);
            int i;
            int NbCurves = Fit.NbMultiCurves();
            int MaxDeg = 0;

            // To convert the MultiCurve to BSpline, all constituent Bezier curves
            // must have the same degree -> Calculate MaxDeg
            int NbPoles;
            for (i = 1; i <= NbCurves; i++)
            {
                int Deg = Fit.Value(i).Degree();
                MaxDeg = Max(MaxDeg, Deg);
            }
            NbPoles = MaxDeg * NbCurves + 1; // Poles on the BSpline
            NCollection_Array1<Point3D> Poles = new(1, NbPoles);

            NCollection_Array1<Point3D> TempPoles = new(1, MaxDeg + 1); // for degree elevation

            NCollection_Array1<double> Knots = new(1, NbCurves + 1); // Knots of the BSpline

            int Compt = 1;
            for (i = 1; i <= Fit.NbMultiCurves(); i++)
            {
                Fit.Parameters(i, out Knots[i], out Knots[i + 1]);

                AppParCurves_MultiCurve MC = Fit.Value(i);                    // Load the i-th Curve
                NCollection_Array1<Point3D> LocalPoles = new(1, MC.Degree() + 1); // Get the poles
                // OCCT: the poles go to the start of Poles, not into LocalPoles, which stays at the origin; the loops
                // below then copy LocalPoles over Poles, so the result's poles after the first MaxDeg are the
                // origin (the first ones are the last segment's).
                MC.Curve(1, Poles);

                // Possible degree elevation
                int Inc = MaxDeg - MC.Degree();
                if (Inc > 0)
                {
                    // OCCT: passes the increment where BSplCLib::IncreaseDegree takes the new degree.
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(Inc, LocalPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), TempPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
                    // update the poles of the PCurve
                    for (int j = 1; j <= MaxDeg + 1; j++)
                    {
                        Poles.SetValue(Compt, TempPoles[j]);
                        Compt++;
                    }
                }
                else
                {
                    // update the poles of the PCurve
                    for (int j = 1; j <= MaxDeg + 1; j++)
                    {
                        Poles.SetValue(Compt, LocalPoles[j]);
                        Compt++;
                    }
                }

                Compt--;
            }

            // update the fields of ProjLib_Approx

            int NbKnots = NbCurves + 1;

            NCollection_Array1<int> Mults = new(1, NbKnots);
            Mults.SetValue(1, MaxDeg + 1);
            for (i = 2; i <= NbCurves; i++)
            {
                Mults.SetValue(i, MaxDeg);
            }
            Mults.SetValue(NbKnots, MaxDeg + 1);
            myResult = new Geom_BSplineCurve(Poles, Knots, Mults, MaxDeg, false);
            myIsDone = true;
        }
    }

    public bool IsDone() => myIsDone;

    /// <summary>The projected curve, frozen; the same object at each call.</summary>
    /// <exception cref="NoSuchObjectException">The projection is not done.</exception>
    public Geom_BSplineCurve BSpline()
    {
        Standard_NoSuchObject_Raise_if(!myIsDone, "ProjLib_ProjectOnSurface:BSpline");
        myResult!.Freeze();
        return myResult;
    }
}
