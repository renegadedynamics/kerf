// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRoot.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A blend function as an <see cref="Approx_SweepFunction"/> for Approx_SweepApproximation: the section at a
/// parameter is solved again from the walking line's points, interpolated.
/// </summary>
/// <remarks>
/// A working object (decision 21). As OCCT's, it inserts into the line the points whose solving took more than three
/// iterations (<c>SearchPoint</c>), so the line it is given changes. OCCT keeps the function as a raw pointer; here
/// it is a reference.
/// </remarks>
public abstract class BRepBlend_AppFuncRoot : Approx_SweepFunction
{
    private readonly BRepBlend_Line myLine;
    private readonly Blend_AppFunction myFunc;
    private readonly math_Vector myTolerance;
    private Blend_Point myPnt = new();
    private Point3D myBary;
    private readonly math_Vector X1;
    private readonly math_Vector X2;
    private readonly math_Vector XInit;
    private readonly math_Vector Sol;
    private readonly FunctionSetRoot mySolver;

    protected BRepBlend_AppFuncRoot(BRepBlend_Line Line, Blend_AppFunction Func, double Tol3d, double Tol2d)
    {
        myLine = Line;
        myFunc = Func;
        myTolerance = new math_Vector(1, Func.NbVariables());
        X1 = new math_Vector(1, Func.NbVariables());
        X2 = new math_Vector(1, Func.NbVariables());
        XInit = new math_Vector(1, Func.NbVariables());
        Sol = new math_Vector(1, Func.NbVariables());

        int NbPoles, NbKnots, Degree, NbPoles2d;
        int ii;

        //  Tolerances
        Func.GetTolerance(myTolerance, Tol3d);
        int dim = Func.NbVariables();
        for (ii = 1; ii <= dim; ii++)
        {
            if (myTolerance[ii] > Tol2d)
            {
                myTolerance[ii] = Tol2d;
            }
        }

        //  Tables
        Func.GetShape(out NbPoles, out NbKnots, out Degree, out NbPoles2d);

        // Calculation of BaryCentre (rational case).
        if (Func.IsRational())
        {
            double Xmax = -1.0e100, Xmin = 1.0e100, Ymax = -1.0e100, Ymin = 1.0e100, Zmax = -1.0e100,
                   Zmin = 1.0e100;
            Blend_Point P;
            for (ii = 1; ii <= myLine.NbPoints(); ii++)
            {
                P = myLine.Point(ii);
                Xmax = Max(Max(P.PointOnS1().X(), P.PointOnS2().X()), Xmax);
                Xmin = Min(Min(P.PointOnS1().X(), P.PointOnS2().X()), Xmin);
                Ymax = Max(Max(P.PointOnS1().Y(), P.PointOnS2().Y()), Ymax);
                Ymin = Min(Min(P.PointOnS1().Y(), P.PointOnS2().Y()), Ymin);
                Zmax = Max(Max(P.PointOnS1().Z(), P.PointOnS2().Z()), Zmax);
                Zmin = Min(Min(P.PointOnS1().Z(), P.PointOnS2().Z()), Zmin);

                myBary.SetCoord((Xmax + Xmin) / 2, (Ymax + Ymin) / 2, (Zmax + Zmin) / 2);
            }
        }
        else
        {
            myBary.SetCoord(0, 0, 0);
        }

        mySolver = new FunctionSetRoot(Func, myTolerance, 30);
    }

    /// <summary>The section for v = <paramref name="Param"/>; false if it cannot be computed.</summary>
    public override bool D0(double Param, double First, double Last, NCollection_Array1<Point3D> Poles,
                            NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weigths)
    {
        bool Ok = true;
        Blend_AppFunction Func = myFunc;
        Ok = SearchPoint(Func, Param, ref myPnt);

        if (Ok)
        {
            Func.Section(myPnt, Poles, Poles2d, Weigths);
        }
        return Ok;
    }

    /// <summary>The section and its derivative in v for v = <paramref name="Param"/>; false if it cannot be computed.</summary>
    public override bool D1(double Param, double First, double Last, NCollection_Array1<Point3D> Poles,
                            NCollection_Array1<Vector3D> DPoles, NCollection_Array1<Point2D> Poles2d,
                            NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<double> Weigths,
                            NCollection_Array1<double> DWeigths)
    {
        bool Ok = true;
        Blend_AppFunction Func = myFunc;

        Ok = SearchPoint(Func, Param, ref myPnt);

        if (Ok)
        {
            Ok = Func.Section(myPnt, Poles, DPoles, Poles2d, DPoles2d, Weigths, DWeigths);
        }

        return Ok;
    }

    /// <summary>The section and its first two derivatives in v for v = <paramref name="Param"/>.</summary>
    public override bool D2(double Param, double First, double Last, NCollection_Array1<Point3D> Poles,
                            NCollection_Array1<Vector3D> DPoles, NCollection_Array1<Vector3D> D2Poles,
                            NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                            NCollection_Array1<Vector2D> D2Poles2d, NCollection_Array1<double> Weigths,
                            NCollection_Array1<double> DWeigths, NCollection_Array1<double> D2Weigths)
    {
        bool Ok = true;
        Blend_AppFunction Func = myFunc;

        Ok = SearchPoint(Func, Param, ref myPnt);
        if (Ok)
        {
            Ok = Func.Section(myPnt, Poles, DPoles, D2Poles, Poles2d, DPoles2d, D2Poles2d, Weigths, DWeigths,
                              D2Weigths);
        }
        return Ok;
    }

    public override int Nb2dCurves()
    {
        Blend_AppFunction Func = myFunc;
        int i, j, k, nbpol2d;
        Func.GetShape(out i, out j, out k, out nbpol2d);
        return nbpol2d;
    }

    public override void SectionShape(out int NbPoles, out int NbKnots, out int Degree)
    {
        Blend_AppFunction Func = myFunc;
        int ii;
        Func.GetShape(out NbPoles, out NbKnots, out Degree, out ii);
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        myFunc.Knots(TKnots);
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        myFunc.Mults(TMults);
    }

    public override bool IsRational()
    {
        return myFunc.IsRational();
    }

    public override int NbIntervals(Continuity S)
    {
        return myFunc.NbIntervals(S);
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        myFunc.Intervals(T, S);
    }

    public override void SetInterval(double First, double Last)
    {
        myFunc.Set(First, Last);
    }

    public override void Resolution(int Index, double Tol, out double TolU, out double TolV)
    {
        myFunc.Resolution(Index, Tol, out TolU, out TolV);
    }

    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol,
                                      NCollection_Array1<double> Tol3d)
    {
        int ii;
        math_Vector V3d = new(1, Tol3d.Length()), V1d = new(1, Tol3d.Length());
        Blend_AppFunction Func = myFunc;

        Func.GetTolerance(BoundTol, SurfTol, AngleTol, V3d, V1d);
        for (ii = 1; ii <= Tol3d.Length(); ii++)
        {
            Tol3d[ii] = V3d[ii];
        }
    }

    public override void SetTolerance(double Tol3d, double Tol2d)
    {
        Blend_AppFunction Func = myFunc;
        int ii, dim = Func.NbVariables();
        Func.GetTolerance(myTolerance, Tol3d);
        for (ii = 1; ii <= dim; ii++)
        {
            if (myTolerance[ii] > Tol2d)
            {
                myTolerance[ii] = Tol2d;
            }
        }
        mySolver.SetTolerance(myTolerance);
    }

    public override Point3D BarycentreOfSurf()
    {
        return myBary;
    }

    public override double MaximalSection()
    {
        return myFunc.GetSectionSize();
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weigths)
    {
        myFunc.GetMinimalWeight(Weigths);
    }

    /// <summary>The walking point of the solution <paramref name="Sol"/> at <paramref name="Param"/>.</summary>
    public abstract void Point(Blend_AppFunction Func, double Param, math_Vector Sol, ref Blend_Point Pnt);

    /// <summary>The variables of the point <paramref name="Pnt"/>.</summary>
    public abstract void Vec(math_Vector Sol, in Blend_Point Pnt);

    // Finds the solution at Param: (1) an approximate solution from the points already computed, (2) a Newton
    // convergence. It fails on singular surfaces, or with no information on the line.
    private bool SearchPoint(Blend_AppFunction Func, double Param, ref Blend_Point Pnt)
    {
        bool Trouve;
        int dim = Func.NbVariables();
        // (1) Find a point of init
        int I1 = 1, I2 = myLine.NbPoints(), Index;
        double t1, t2;

        //  (1.a) It is checked if it is inside
        if (Param < myLine.Point(I1).Parameter())
        {
            return false;
        }
        if (Param > myLine.Point(I2).Parameter())
        {
            return false;
        }

        //  (1.b) Find the interval
        Trouve = SearchLocation(Param, I1, I2, out Index);

        //  (1.c) If the point is already calculated it is returned
        if (Trouve)
        {
            Pnt = myLine.Point(Index);
            Vec(XInit, Pnt);
        }
        else
        {
            //  (1.d) Initialisation by linear interpolation
            Pnt = myLine.Point(Index);
            Vec(X1, Pnt);
            t1 = Pnt.Parameter();

            Pnt = myLine.Point(Index + 1);
            Vec(X2, Pnt);
            t2 = Pnt.Parameter();

            double Parammt1 = (Param - t1) / (t2 - t1);
            double t2mParam = (t2 - Param) / (t2 - t1);
            for (int i = 1; i <= dim; i++)
            {
                XInit[i] = X2[i] * Parammt1 + X1[i] * t2mParam;
            }
        }

        // (2) Calculation of the solution ------------------------
        Func.Set(Param);
        Func.GetBounds(X1, X2);
        mySolver.Perform(Func, XInit, X1, X2);

        if (!mySolver.IsDone)
        {
            return false;
        }
        mySolver.Root(Sol);

        // (3) Storage of the point
        Point(Func, Param, Sol, ref Pnt);

        // (4) Insertion of the point if the calculation seems long.
        if ((!Trouve) && (mySolver.IterationCount > 3))
        {
            // OCCT_DEBUG trace left out.
            myLine.InsertBefore(Index + 1, Pnt);
        }
        return true;
    }

    // Binary search in the line of the parametric interval containing Param: true and the point's index if the point
    // of parameter Param is already stored.
    private bool SearchLocation(double Param, int FirstIndex, int LastIndex, out int ParamIndex)
    {
        int Ideb = FirstIndex, Ifin = LastIndex, Idemi;
        double Valeur;

        Valeur = myLine.Point(Ideb).Parameter();
        if (Param == Valeur)
        {
            ParamIndex = Ideb;
            return true;
        }

        Valeur = myLine.Point(Ifin).Parameter();
        if (Param == Valeur)
        {
            ParamIndex = Ifin;
            return true;
        }

        while (Ideb + 1 != Ifin)
        {
            Idemi = (Ideb + Ifin) / 2;
            Valeur = myLine.Point(Idemi).Parameter();
            if (Valeur < Param)
            {
                Ideb = Idemi;
            }
            else
            {
                if (Valeur > Param)
                {
                    Ifin = Idemi;
                }
                else
                {
                    ParamIndex = Idemi;
                    return true;
                }
            }
        }

        ParamIndex = Ideb;
        return false;
    }
}
