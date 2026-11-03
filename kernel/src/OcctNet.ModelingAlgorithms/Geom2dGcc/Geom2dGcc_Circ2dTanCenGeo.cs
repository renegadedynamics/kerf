// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCenGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCenGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// 2D circles tangent to a curve and centred on a point: the circles through the nearest and the farthest extrema
/// of the distance from the point to the curve.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 2 solutions.</remarks>
public sealed class Geom2dGcc_Circ2dTanCenGeo
{
    private readonly bool WellDone;
    private readonly int NbrSol;
    private readonly NCollection_Array1<Circle2D> cirsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> pararg1;

    //=========================================================================
    //   Creation d un cercle tangent a une courbe centre en un point.        +
    //=========================================================================
    /// <summary>Circles centred on Pcenter tangent to the qualified curve.</summary>
    /// <exception cref="FailureException">The distance from the point to the curve has no extremum.</exception>
    public Geom2dGcc_Circ2dTanCenGeo(in Geom2dGcc_QCurve Qualified1, in Point2D Pcenter, double Tolerance)
    {
        //========================================================================
        //   Initialisation des champs.                                          +
        //========================================================================
        WellDone   = false;
        NbrSol     = 0;
        cirsol     = new(1, 2);
        qualifier1 = new(1, 2);
        pnttg1sol  = new(1, 2);
        par1sol    = new(1, 2);
        pararg1    = new(1, 2);

        double                       Tol      = Math.Abs(Tolerance);
        NCollection_Array1<Point2D> pTan     = new(1, 2);
        NCollection_Array1<int>      Index    = new(1, 2);
        NCollection_Array1<double>   theDist2 = new(1, 2);
        NCollection_Array1<double>   theParam = new(1, 2);
        theDist2[1]                = RealLast();
        theDist2[2]                = 0.0;
        int                 i      = 1;
        int                 nbsol  = 0;
        Direction2D            dirx   = new(Direction2D.D.X);
        double              thePar;
        Geom2dAdaptor_Curve curve  = Qualified1.Qualified();
        Extrema_ExtPC2d     distmin = new(Pcenter,
                                          curve,
                                          Geom2dGcc_CurveTool.FirstParameter(curve),
                                          Geom2dGcc_CurveTool.LastParameter(curve),
                                          Tol);
        if (!distmin.IsDone())
        {
            throw new FailureException();
        }
        int nbext = distmin.NbExt();
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
        if (Index[1] == Index[2])
        {
            nbsol = 1;
        }
        else
        {
            nbsol = 2;
        }
        for (i = 1; i <= nbsol; i++)
        {
            Geom2dGcc_CurveTool.D1(curve, theParam[i], out Point2D point1, out Vector2D Tan1);
            double   normetan1 = Tan1.Magnitude();
            Vector2D Vec1      = new(point1, Pcenter);
            double   normevec1 = Vec1.Magnitude();
            double   dot1;
            if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
            {
                dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
            }
            else
            {
                dot1 = 0.0;
            }
            Tol = 1.0e-12;
            if (dot1 <= Tol)
            {
                double Angle1 = Vec1.Angle(Tan1);
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Angle1 <= 0.0)
                    || (Qualified1.IsOutside() && Angle1 >= 0.0) || (Qualified1.IsEnclosed() && Angle1 <= 0.0))
                {
                    NbrSol++;
                    cirsol[NbrSol]     = new Circle2D(new Axis2D(Pcenter, dirx), Math.Sqrt(theDist2[i]));
                    qualifier1[NbrSol] = Qualified1.Qualifier();
                    pararg1[NbrSol]    = theParam[i];
                    par1sol[NbrSol]    = 0.0;
                    pnttg1sol[NbrSol]  = pTan[i];
                    WellDone           = true;
                }
            }
        }
    }

    //=========================================================================

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in 1..NbSolutions().</exception>
    public Circle2D ThisSolution(int Index)
    {
        if (Index > NbrSol || Index <= 0)
        {
            throw new OutOfRangeException();
        }

        return cirsol[Index];
    }

    /// <summary>The qualifier of the argument for the solution of index Index.</summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            Qualif1 = qualifier1[Index];
        }
    }

    /// <summary>The tangency point between the solution of index Index and the curve, and its parameters.</summary>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            PntSol = pnttg1sol[Index];
            ParSol = par1sol[Index];
            ParArg = pararg1[Index];
        }
    }
}
