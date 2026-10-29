// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSNorm.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSNorm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The system whose roots are the extrema of the distance between a point P and a surface S:
/// F1(u, v) = (S - P).Su and F2(u, v) = (S - P).Sv, with its derivatives, for math_FunctionSetRoot. Each root found
/// (GetStateNumber) is stored with its square distance, unless it repeats one already found.
/// </summary>
/// <remarks>
/// OCCT keeps a pointer to the surface; the port keeps the adaptor, which must outlive the function.
/// </remarks>
public class Extrema_FuncPSNorm : math_FunctionSetWithDerivatives
{
    private gp_Pnt myP = new();
    private Adaptor3d_Surface? myS;
    private double myU;
    private double myV;
    private gp_Pnt myPs = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPoint = new();
    private bool myPinit;
    private bool mySinit;

    public Extrema_FuncPSNorm()
    {
        myS = null;
        myU = 0.0;
        myV = 0.0;
        myPinit = false;
        mySinit = false;
    }

    public Extrema_FuncPSNorm(in gp_Pnt P, Adaptor3d_Surface S)
    {
        myU = 0.0;
        myV = 0.0;
        myP = P;
        myS = S;
        myPinit = true;
        mySinit = true;
    }

    /// <summary>Sets the surface, and forgets the extrema found.</summary>
    public void Initialize(Adaptor3d_Surface S)
    {
        myS = S;
        mySinit = true;
        myPoint.Clear();
        mySqDist.Clear();
    }

    /// <summary>Sets the point, and forgets the extrema found.</summary>
    public void SetPoint(in gp_Pnt P)
    {
        myP = P;
        myPinit = true;
        myPoint.Clear();
        mySqDist.Clear();
    }

    public override int NbVariables() => 2;

    public override int NbEquations() => 2;

    /// <summary>Calculates Fi(U, V).</summary>
    public override bool Value(math_Vector UV, math_Vector F)
    {
        if (!myPinit || !mySinit)
        {
            throw new Standard_TypeMismatch();
        }
        myU = UV[1];
        myV = UV[2];
        myS!.D1(myU, myV, out myPs, out gp_Vec Dus, out gp_Vec Dvs);

        gp_Vec PPs = new(myP, myPs);

        F[1] = PPs.Dot(Dus);
        F[2] = PPs.Dot(Dvs);

        return true;
    }

    /// <summary>Calculates Fi'(U, V).</summary>
    public override bool Derivatives(math_Vector UV, math_Matrix Df)
    {
        math_Vector F = new(1, 2);
        return Values(UV, F, Df);
    }

    /// <summary>Calculates Fi(U, V) and Fi'(U, V).</summary>
    public override bool Values(math_Vector UV, math_Vector F, math_Matrix Df)
    {
        if (!myPinit || !mySinit)
        {
            throw new Standard_TypeMismatch();
        }
        myU = UV[1];
        myV = UV[2];
        myS!.D2(myU, myV, out myPs, out gp_Vec Dus, out gp_Vec Dvs, out gp_Vec Duus, out gp_Vec Dvvs, out gp_Vec Duvs);

        gp_Vec PPs = new(myP, myPs);

        Df[1, 1] = Dus.SquareMagnitude() + PPs.Dot(Duus);
        Df[1, 2] = Dvs.Dot(Dus) + PPs.Dot(Duvs);
        Df[2, 1] = Df[1, 2];
        Df[2, 2] = Dvs.SquareMagnitude() + PPs.Dot(Dvvs);

        // 3. Value
        F[1] = PPs.Dot(Dus);
        F[2] = PPs.Dot(Dvs);

        return true;
    }

    /// <summary>Saves the extremum found, unless it repeats one already saved.</summary>
    public override int GetStateNumber()
    {
        if (!myPinit || !mySinit)
        {
            throw new Standard_TypeMismatch();
        }
        // comparison of solution with previous solutions
        int i = 1, nbSol = mySqDist.Length();
        double tol2d = Precision.PConfusion() * Precision.PConfusion();

        for (; i <= nbSol; i++)
        {
            myPoint[i].Parameter(out double aU, out double aV);
            if (((myU - aU) * (myU - aU) + (myV - aV) * (myV - aV)) <= tol2d)
            {
                break;
            }
        }
        if (i <= nbSol)
        {
            return 0;
        }
        mySqDist.Append(myPs.SquareDistance(myP));
        myPoint.Append(new Extrema_POnSurf(myU, myV, myPs));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int NbExt() => mySqDist.Length();

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int N)
    {
        if (!myPinit || !mySinit)
        {
            throw new Standard_TypeMismatch();
        }
        return mySqDist.Value(N);
    }

    /// <summary>The Nth extremum.</summary>
    public Extrema_POnSurf Point(int N)
    {
        if (!myPinit || !mySinit)
        {
            throw new Standard_TypeMismatch();
        }
        return myPoint.Value(N);
    }
}
