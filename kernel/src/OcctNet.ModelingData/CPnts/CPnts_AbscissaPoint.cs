// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_AbscissaPoint.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_AbscissaPoint.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_AbscissaPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  cases processed: straight segment, arc of circle, parameterized curve
//  curve should be C1
//  for a parameterized curve:
//  calculate the total length of the curve
//  calculate an approached point by assimilating the curve to a staight line
//  calculate the length of the curve between the start point and the approached point
//  by successive iteration find the point and its associated parameter
//  call to FunctionRoot

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The algorithm computing a point on a curve at a given distance from another point on the curve, and the
/// lengths of curves (Gauss integration of the derivative's norm).
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Init</c> keeps a reference to the curve, which must outlive the next
/// <c>Perform</c>.
/// </remarks>
public sealed class CPnts_AbscissaPoint
{
    private bool myDone;
    private double myL;
    private double myParam;
    private double myUMin;
    private double myUMax;
    private readonly CPnts_MyRootFunction myF = new();

    // auxiliary functions to compute the length of the derivative
    private static double f3d(double X, object? C)
    {
        ((Adaptor3d_Curve)C!).D1(X, out _, out Vector3D V);
        return V.Magnitude();
    }

    private static double f2d(double X, object? C)
    {
        ((Adaptor2d_Curve2d)C!).D1(X, out _, out Vector2D V);
        return V.Magnitude();
    }

    // The delegates, allocated once (OCCT passes function pointers).
    private static readonly CPnts_RealFunction s_f3d = f3d;
    private static readonly CPnts_RealFunction s_f2d = f2d;

    private static int order(Adaptor3d_Curve C)
    {
        switch (C.GetType())
        {
            case Line:
                return 2;

            case Parabola:
                return 5;

            case BezierCurve:
                return Min(24, 2 * C.Degree());

            case BSplineCurve:
                return Min(24, 2 * C.NbPoles() - 1);

            default:
                return 10;
        }
    }

    private static int order(Adaptor2d_Curve2d C)
    {
        switch (C.GetType())
        {
            case Line:
                return 2;

            case Parabola:
                return 5;

            case BezierCurve:
                return Min(24, 2 * C.Bezier().Degree());

            case BSplineCurve:
                return Min(24, 2 * C.BSpline().NbPoles() - 1);

            default:
                return 10;
        }
    }

    /// <summary>The length of the curve.</summary>
    public static double Length(Adaptor3d_Curve C)
    {
        return CPnts_AbscissaPoint.Length(C, C.FirstParameter(), C.LastParameter());
    }

    /// <summary>The length of the curve.</summary>
    public static double Length(Adaptor2d_Curve2d C)
    {
        return CPnts_AbscissaPoint.Length(C, C.FirstParameter(), C.LastParameter());
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(Adaptor3d_Curve C, double Tol)
    {
        return CPnts_AbscissaPoint.Length(C, C.FirstParameter(), C.LastParameter(), Tol);
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(Adaptor2d_Curve2d C, double Tol)
    {
        return CPnts_AbscissaPoint.Length(C, C.FirstParameter(), C.LastParameter(), Tol);
    }

    /// <summary>The length of the curve between the parameters U1 and U2.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(Adaptor3d_Curve C, double U1, double U2)
    {
        CPnts_MyGaussFunction FG = new();
        CPnts_RealFunction rf = s_f3d;
        FG.Init(rf, C);
        GaussSingleIntegration TheLength = new(FG, U1, U2, order(C));
        if (!TheLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(TheLength.Value());
    }

    /// <summary>The length of the curve between the parameters U1 and U2.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(Adaptor2d_Curve2d C, double U1, double U2)
    {
        CPnts_MyGaussFunction FG = new();
        CPnts_RealFunction rf = s_f2d;
        FG.Init(rf, C);
        GaussSingleIntegration TheLength = new(FG, U1, U2, order(C));
        if (!TheLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(TheLength.Value());
    }

    /// <summary>The length of the curve between the parameters U1 and U2 with the given tolerance.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(Adaptor3d_Curve C, double U1, double U2, double Tol)
    {
        CPnts_MyGaussFunction FG = new();
        CPnts_RealFunction rf = s_f3d;
        FG.Init(rf, C);
        GaussSingleIntegration TheLength = new(FG, U1, U2, order(C), Tol);
        if (!TheLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(TheLength.Value());
    }

    /// <summary>The length of the curve between the parameters U1 and U2 with the given tolerance.</summary>
    /// <exception cref="ConstructionErrorException">The integration fails.</exception>
    public static double Length(Adaptor2d_Curve2d C, double U1, double U2, double Tol)
    {
        CPnts_MyGaussFunction FG = new();
        CPnts_RealFunction rf = s_f2d;
        FG.Init(rf, C);
        GaussSingleIntegration TheLength = new(FG, U1, U2, order(C), Tol);
        if (!TheLength.IsDone)
        {
            throw new ConstructionErrorException();
        }
        return Math.Abs(TheLength.Value());
    }

    public CPnts_AbscissaPoint()
    {
        myDone = false;
        myL = 0.0;
        myParam = 0.0;
        myUMin = 0.0;
        myUMax = 0.0;
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0.</summary>
    public CPnts_AbscissaPoint(Adaptor3d_Curve C, double Abscissa, double U0, double Resolution)
    {
        //  Init(C);
        Init(C, Resolution); // rbv's modification
        //
        Perform(Abscissa, U0, Resolution);
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0.</summary>
    public CPnts_AbscissaPoint(Adaptor2d_Curve2d C, double Abscissa, double U0, double Resolution)
    {
        Init(C);
        Perform(Abscissa, U0, Resolution);
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0, starting from Ui.</summary>
    public CPnts_AbscissaPoint(Adaptor3d_Curve C, double Abscissa, double U0, double Ui, double Resolution)
    {
        Init(C);
        Perform(Abscissa, U0, Ui, Resolution);
    }

    /// <summary>The point at the distance Abscissa from the point of parameter U0, starting from Ui.</summary>
    public CPnts_AbscissaPoint(Adaptor2d_Curve2d C, double Abscissa, double U0, double Ui, double Resolution)
    {
        Init(C);
        Perform(Abscissa, U0, Ui, Resolution);
    }

    /// <summary>Initializes the algorithm on the whole curve.</summary>
    public void Init(Adaptor3d_Curve C)
    {
        Init(C, C.FirstParameter(), C.LastParameter());
    }

    /// <summary>Initializes the algorithm on the whole curve.</summary>
    public void Init(Adaptor2d_Curve2d C)
    {
        Init(C, C.FirstParameter(), C.LastParameter());
    }

    /// <summary>Initializes the algorithm on the whole curve with the given tolerance (rbv, curvilinear parametrisation).</summary>
    public void Init(Adaptor3d_Curve C, double Tol)
    {
        Init(C, C.FirstParameter(), C.LastParameter(), Tol);
    }

    /// <summary>Initializes the algorithm on the whole curve with the given tolerance.</summary>
    public void Init(Adaptor2d_Curve2d C, double Tol)
    {
        Init(C, C.FirstParameter(), C.LastParameter(), Tol);
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2.</summary>
    public void Init(Adaptor3d_Curve C, double U1, double U2)
    {
        CPnts_RealFunction rf = s_f3d;
        myF.Init(rf, C, order(C));
        myL = CPnts_AbscissaPoint.Length(C, U1, U2);
        myUMin = Min(U1, U2);
        myUMax = Max(U1, U2);
        double DU = myUMax - myUMin;
        myUMin = myUMin - DU;
        myUMax = myUMax + DU;
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2.</summary>
    public void Init(Adaptor2d_Curve2d C, double U1, double U2)
    {
        CPnts_RealFunction rf = s_f2d;
        myF.Init(rf, C, order(C));
        myL = CPnts_AbscissaPoint.Length(C, U1, U2);
        myUMin = Min(U1, U2);
        myUMax = Max(U1, U2);
        double DU = myUMax - myUMin;
        myUMin = myUMin - DU;
        myUMax = myUMax + DU;
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2 with the given tolerance (rbv).</summary>
    public void Init(Adaptor3d_Curve C, double U1, double U2, double Tol)
    {
        CPnts_RealFunction rf = s_f3d;
        myF.Init(rf, C, order(C));
        myL = CPnts_AbscissaPoint.Length(C, U1, U2, Tol);
        myUMin = Min(U1, U2);
        myUMax = Max(U1, U2);
        double DU = myUMax - myUMin;
        myUMin = myUMin - DU;
        myUMax = myUMax + DU;
    }

    /// <summary>Initializes the algorithm on the curve between U1 and U2 with the given tolerance.</summary>
    public void Init(Adaptor2d_Curve2d C, double U1, double U2, double Tol)
    {
        CPnts_RealFunction rf = s_f2d;
        myF.Init(rf, C, order(C));
        myL = CPnts_AbscissaPoint.Length(C, U1, U2, Tol);
        myUMin = Min(U1, U2);
        myUMax = Max(U1, U2);
        double DU = myUMax - myUMin;
        myUMin = myUMin - DU;
        myUMax = myUMax + DU;
    }

    /// <summary>Computes the point at the distance Abscissa from the point of parameter U0.</summary>
    public void Perform(double Abscissa, double U0, double Resolution)
    {
        if (myL < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            //
            //  leave less violently : it is expected that
            //  the increment of the level of myParam will not be great
            //
            myDone = true;
            myParam = U0;
        }
        else
        {
            double Ui = U0 + (Abscissa / myL) * (myUMax - myUMin) / 3.0;
            // exercise : why 3 ?
            Perform(Abscissa, U0, Ui, Resolution);
        }
    }

    /// <summary>Computes the point at the distance Abscissa from the point of parameter U0, starting from Ui.</summary>
    public void Perform(double Abscissa, double U0, double Ui, double Resolution)
    {
        if (myL < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            //
            //  leave less violently :
            //
            myDone = true;
            myParam = U0;
        }
        else
        {
            myDone = false;
            myF.Init(U0, Abscissa);

            FunctionRoot Solution = new(myF, Ui, Resolution, myUMin, myUMax);

            // Temporarily suspend the validity test of the solution
            // it is necessary to make a tolreached as soon as one will make a cdl
            // lbo 21/03/97
            if (Solution.IsDone)
            {
                myDone = true;
                myParam = Solution.Root();
            }
        }
    }

    /// <summary>
    /// Computes the point at the distance Abscissa from the point of parameter U0, starting from Ui, integrating
    /// with the tolerance Resolution / 10 (rbv).
    /// </summary>
    public void AdvPerform(double Abscissa, double U0, double Ui, double Resolution)
    {
        if (myL < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            //
            //  leave less violently :
            //
            myDone = true;
            myParam = U0;
        }
        else
        {
            myDone = false;
            //    myF.Init(U0, Abscissa);
            myF.Init(U0, Abscissa, Resolution / 10); // rbv's modification

            FunctionRoot Solution = new(myF, Ui, Resolution, myUMin, myUMax);

            // Temporarily suspend the validity test of the solution
            // it is necessary to make a tolreached as soon as one will make a cdl
            // lbo 21/03/97
            if (Solution.IsDone)
            {
                myDone = true;
                myParam = Solution.Root();
            }
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => myDone;

    /// <summary>The parameter of the solution.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter()
    {
        StdFail_NotDone_Raise_if(!myDone, "CPnts_AbscissaPoint::Parameter() - no result");
        return myParam;
    }

    /// <summary>Enforces the parameter of the solution (used when the solution is known).</summary>
    public void SetParameter(double P)
    {
        myParam = P;
        myDone = true;
    }
}
