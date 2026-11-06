// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCenGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCenGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// 2D circles tangent to a curve and centred on a point: the circles through the nearest and the farthest extrema
/// of the distance from the point to the curve.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 2 solutions.</remarks>
internal sealed class Circ2dTanCenGeo
{
    private readonly bool _wellDone;
    private readonly int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _pararg1;

    //=========================================================================
    //   Creation d un cercle tangent a une courbe centre en un point.        +
    //=========================================================================
    /// <summary>Circles centred on Pcenter tangent to the qualified curve.</summary>
    /// <exception cref="FailureException">The distance from the point to the curve has no extremum.</exception>
    public Circ2dTanCenGeo(in QCurve qualified1, in Point2D pcenter, double tolerance)
    {
        //========================================================================
        //   Initialisation des champs.                                          +
        //========================================================================
        _wellDone   = false;
        _nbrSol     = 0;
        _cirsol     = new(1, 2);
        _qualifier1 = new(1, 2);
        _pnttg1sol  = new(1, 2);
        _par1sol    = new(1, 2);
        _pararg1    = new(1, 2);

        double                       tol      = Math.Abs(tolerance);
        NCollection_Array1<Point2D> pTan     = new(1, 2);
        NCollection_Array1<int>      index    = new(1, 2);
        NCollection_Array1<double>   theDist2 = new(1, 2);
        NCollection_Array1<double>   theParam = new(1, 2);
        theDist2[1]                = RealLast();
        theDist2[2]                = 0.0;
        int                 i      = 1;
        int                 nbsol  = 0;
        Direction2D            dirx   = new(Direction2D.D.X);
        double              thePar;
        CurveAdaptor2D curve  = qualified1.Qualified();
        ExtPC2d     distmin = new(pcenter,
                                          curve,
                                          ConstraintCurveTool.FirstParameter(curve),
                                          ConstraintCurveTool.LastParameter(curve),
                                          tol);
        if (!distmin.IsDone)
        {
            throw new FailureException();
        }
        int nbext = distmin.ExtCount;
        if (nbext == 0)
        {
            throw new FailureException();
        }
        while (i <= nbext)
        {
            thePar = distmin.Point(i).Parameter();
            if (distmin.SquareDistance(i) < theDist2[1])
            {
                theDist2[1] = distmin.SquareDistance(i);
                theParam[1] = thePar;
                pTan[1]     = distmin.Point(i).Value();
            }
            if (distmin.SquareDistance(i) > theDist2[2])
            {
                theDist2[2] = distmin.SquareDistance(i);
                theParam[2] = thePar;
                pTan[2]     = distmin.Point(i).Value();
            }
            i++;
        }
        // OCCT: Index is never written, so this compares two uninitialised ints (0 and 0 in the port: one solution,
        // the nearest; native's values vary with memory and may give two)
        if (index[1] == index[2])
        {
            nbsol = 1;
        }
        else
        {
            nbsol = 2;
        }
        for (i = 1; i <= nbsol; i++)
        {
            ConstraintCurveTool.D1(curve, theParam[i], out Point2D point1, out Vector2D tan1);
            double   normetan1 = tan1.Magnitude;
            Vector2D vec1      = new(point1, pcenter);
            double   normevec1 = vec1.Magnitude;
            double   dot1;
            if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
            {
                dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
            }
            else
            {
                dot1 = 0.0;
            }
            tol = 1.0e-12;
            if (dot1 <= tol)
            {
                double angle1 = vec1.AngleTo(tan1);
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                    || (qualified1.IsOutside && angle1 >= 0.0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                {
                    _nbrSol++;
                    _cirsol[_nbrSol]     = new Circle2D(new Axis2D(pcenter, dirx), Math.Sqrt(theDist2[i]));
                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                    _pararg1[_nbrSol]    = theParam[i];
                    _par1sol[_nbrSol]    = 0.0;
                    _pnttg1sol[_nbrSol]  = pTan[i];
                    _wellDone           = true;
                }
            }
        }
    }

    //=========================================================================

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions.</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in 1..NbSolutions().</exception>
    public Circle2D ThisSolution(int index)
    {
        if (index > _nbrSol || index <= 0)
        {
            throw new OutOfRangeException();
        }

        return _cirsol[index];
    }

    /// <summary>The qualifier of the argument for the solution of index Index.</summary>
    public void WhichQualifier(int index, out Qualifier qualif1)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
        }
    }

    /// <summary>The tangency point between the solution of index Index and the curve, and its parameters.</summary>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            pntSol = _pnttg1sol[index];
            parSol = _par1sol[index];
            parArg = _pararg1[index];
        }
    }
}
