// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRoot.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppFuncRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// A blend function as an <see cref="SweepFunction"/> for Approx_SweepApproximation: the section at a
/// parameter is solved again from the walking line's points, interpolated.
/// </summary>
/// <remarks>
/// A working object (decision 21). As OCCT's, it inserts into the line the points whose solving took more than three
/// iterations (<c>SearchPoint</c>), so the line it is given changes. OCCT keeps the function as a raw pointer; here
/// it is a reference.
/// </remarks>
public abstract class AppFuncRoot : SweepFunction
{
    private readonly Line _line;
    private readonly AppFunction _func;
    private readonly math_Vector _tolerance;
    private BlendPoint _pnt = new();
    private Point3D _bary;
    private readonly math_Vector _x1;
    private readonly math_Vector _x2;
    private readonly math_Vector _xInit;
    private readonly math_Vector _sol;
    private readonly FunctionSetRoot _solver;

    protected AppFuncRoot(Line line, AppFunction func, double tol3d, double tol2d)
    {
        _line = line;
        _func = func;
        _tolerance = new math_Vector(1, func.NbVariables());
        _x1 = new math_Vector(1, func.NbVariables());
        _x2 = new math_Vector(1, func.NbVariables());
        _xInit = new math_Vector(1, func.NbVariables());
        _sol = new math_Vector(1, func.NbVariables());

        int nbPoles, nbKnots, degree, nbPoles2d;
        int ii;

        //  Tolerances
        func.GetTolerance(_tolerance, tol3d);
        int dim = func.NbVariables();
        for (ii = 1; ii <= dim; ii++)
        {
            if (_tolerance[ii] > tol2d)
            {
                _tolerance[ii] = tol2d;
            }
        }

        //  Tables
        func.GetShape(out nbPoles, out nbKnots, out degree, out nbPoles2d);

        // Calculation of BaryCentre (rational case).
        if (func.IsRational)
        {
            double xmax = -1.0e100, xmin = 1.0e100, ymax = -1.0e100, ymin = 1.0e100, zmax = -1.0e100,
                   zmin = 1.0e100;
            BlendPoint p;
            for (ii = 1; ii <= _line.NbPoints(); ii++)
            {
                p = _line.Point(ii);
                xmax = Max(Max(p.PointOnS1().X(), p.PointOnS2().X()), xmax);
                xmin = Min(Min(p.PointOnS1().X(), p.PointOnS2().X()), xmin);
                ymax = Max(Max(p.PointOnS1().Y(), p.PointOnS2().Y()), ymax);
                ymin = Min(Min(p.PointOnS1().Y(), p.PointOnS2().Y()), ymin);
                zmax = Max(Max(p.PointOnS1().Z(), p.PointOnS2().Z()), zmax);
                zmin = Min(Min(p.PointOnS1().Z(), p.PointOnS2().Z()), zmin);

                _bary = _bary.WithCoord((xmax + xmin) / 2, (ymax + ymin) / 2, (zmax + zmin) / 2);
            }
        }
        else
        {
            _bary = _bary.WithCoord(0, 0, 0);
        }

        _solver = new FunctionSetRoot(func, _tolerance, 30);
    }

    /// <summary>The section for v = <paramref name="param"/>; false if it cannot be computed.</summary>
    public override bool D0(double param, double first, double last, NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Point2D> poles2d, NCollection_Array1<double> weigths)
    {
        bool ok = true;
        AppFunction func = _func;
        ok = SearchPoint(func, param, ref _pnt);

        if (ok)
        {
            func.Section(_pnt, poles, poles2d, weigths);
        }
        return ok;
    }

    /// <summary>The section and its derivative in v for v = <paramref name="param"/>; false if it cannot be computed.</summary>
    public override bool D1(double param, double first, double last, NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Vector3D> dPoles, NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<double> weigths,
                            NCollection_Array1<double> dWeigths)
    {
        bool ok = true;
        AppFunction func = _func;

        ok = SearchPoint(func, param, ref _pnt);

        if (ok)
        {
            ok = func.Section(_pnt, poles, dPoles, poles2d, dPoles2d, weigths, dWeigths);
        }

        return ok;
    }

    /// <summary>The section and its first two derivatives in v for v = <paramref name="param"/>.</summary>
    public override bool D2(double param, double first, double last, NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Vector3D> dPoles, NCollection_Array1<Vector3D> d2Poles,
                            NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<Vector2D> d2Poles2d, NCollection_Array1<double> weigths,
                            NCollection_Array1<double> dWeigths, NCollection_Array1<double> d2Weigths)
    {
        bool ok = true;
        AppFunction func = _func;

        ok = SearchPoint(func, param, ref _pnt);
        if (ok)
        {
            ok = func.Section(_pnt, poles, dPoles, d2Poles, poles2d, dPoles2d, d2Poles2d, weigths, dWeigths,
                              d2Weigths);
        }
        return ok;
    }

    public override int Nb2dCurves()
    {
        AppFunction func = _func;
        int i, j, k, nbpol2d;
        func.GetShape(out i, out j, out k, out nbpol2d);
        return nbpol2d;
    }

    public override void SectionShape(out int nbPoles, out int nbKnots, out int degree)
    {
        AppFunction func = _func;
        int ii;
        func.GetShape(out nbPoles, out nbKnots, out degree, out ii);
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        _func.Knots(tKnots);
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        _func.Mults(tMults);
    }

    public override bool IsRational
    {
        get
        {
            return _func.IsRational;
        }
    }

    public override int NbIntervals(Continuity s)
    {
        return _func.NbIntervals(s);
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _func.Intervals(t, s);
    }

    public override void SetInterval(double first, double last)
    {
        _func.Set(first, last);
    }

    public override void Resolution(int index, double tol, out double tolU, out double tolV)
    {
        _func.Resolution(index, tol, out tolU, out tolV);
    }

    public override void GetTolerance(double boundTol, double surfTol, double angleTol,
                                      NCollection_Array1<double> tol3d)
    {
        int ii;
        math_Vector v3d = new(1, tol3d.Length()), v1d = new(1, tol3d.Length());
        AppFunction func = _func;

        func.GetTolerance(boundTol, surfTol, angleTol, v3d, v1d);
        for (ii = 1; ii <= tol3d.Length(); ii++)
        {
            tol3d[ii] = v3d[ii];
        }
    }

    public override void SetTolerance(double tol3d, double tol2d)
    {
        AppFunction func = _func;
        int ii, dim = func.NbVariables();
        func.GetTolerance(_tolerance, tol3d);
        for (ii = 1; ii <= dim; ii++)
        {
            if (_tolerance[ii] > tol2d)
            {
                _tolerance[ii] = tol2d;
            }
        }
        _solver.SetTolerance(_tolerance);
    }

    public override Point3D BarycentreOfSurf()
    {
        return _bary;
    }

    public override double MaximalSection()
    {
        return _func.GetSectionSize();
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weigths)
    {
        _func.GetMinimalWeight(weigths);
    }

    /// <summary>The walking point of the solution <paramref name="sol"/> at <paramref name="param"/>.</summary>
    public abstract void Point(AppFunction func, double param, math_Vector sol, ref BlendPoint pnt);

    /// <summary>The variables of the point <paramref name="pnt"/>.</summary>
    public abstract void Vec(math_Vector sol, in BlendPoint pnt);

    // Finds the solution at Param: (1) an approximate solution from the points already computed, (2) a Newton
    // convergence. It fails on singular surfaces, or with no information on the line.
    private bool SearchPoint(AppFunction func, double param, ref BlendPoint pnt)
    {
        bool trouve;
        int dim = func.NbVariables();
        // (1) Find a point of init
        int i1 = 1, i2 = _line.NbPoints(), index;
        double t1, t2;

        //  (1.a) It is checked if it is inside
        if (param < _line.Point(i1).Parameter())
        {
            return false;
        }
        if (param > _line.Point(i2).Parameter())
        {
            return false;
        }

        //  (1.b) Find the interval
        trouve = SearchLocation(param, i1, i2, out index);

        //  (1.c) If the point is already calculated it is returned
        if (trouve)
        {
            pnt = _line.Point(index);
            Vec(_xInit, pnt);
        }
        else
        {
            //  (1.d) Initialisation by linear interpolation
            pnt = _line.Point(index);
            Vec(_x1, pnt);
            t1 = pnt.Parameter();

            pnt = _line.Point(index + 1);
            Vec(_x2, pnt);
            t2 = pnt.Parameter();

            double parammt1 = (param - t1) / (t2 - t1);
            double t2mParam = (t2 - param) / (t2 - t1);
            for (int i = 1; i <= dim; i++)
            {
                _xInit[i] = _x2[i] * parammt1 + _x1[i] * t2mParam;
            }
        }

        // (2) Calculation of the solution ------------------------
        func.Set(param);
        func.GetBounds(_x1, _x2);
        _solver.Perform(func, _xInit, _x1, _x2);

        if (!_solver.IsDone)
        {
            return false;
        }
        _solver.Root(_sol);

        // (3) Storage of the point
        Point(func, param, _sol, ref pnt);

        // (4) Insertion of the point if the calculation seems long.
        if ((!trouve) && (_solver.IterationCount > 3))
        {
            // OCCT_DEBUG trace left out.
            _line.InsertBefore(index + 1, pnt);
        }
        return true;
    }

    // Binary search in the line of the parametric interval containing Param: true and the point's index if the point
    // of parameter Param is already stored.
    private bool SearchLocation(double param, int firstIndex, int lastIndex, out int paramIndex)
    {
        int ideb = firstIndex, ifin = lastIndex, idemi;
        double valeur;

        valeur = _line.Point(ideb).Parameter();
        if (param == valeur)
        {
            paramIndex = ideb;
            return true;
        }

        valeur = _line.Point(ifin).Parameter();
        if (param == valeur)
        {
            paramIndex = ifin;
            return true;
        }

        while (ideb + 1 != ifin)
        {
            idemi = (ideb + ifin) / 2;
            valeur = _line.Point(idemi).Parameter();
            if (valeur < param)
            {
                ideb = idemi;
            }
            else
            {
                if (valeur > param)
                {
                    ifin = idemi;
                }
                else
                {
                    paramIndex = idemi;
                    return true;
                }
            }
        }

        paramIndex = ideb;
        return false;
    }
}
