// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The system whose roots are the extrema of the distance between a curve C and a surface S:
/// F1(t, u, v) = (C(t) - S(u, v)).C'(t), F2 = (C(t) - S(u, v)).Su, F3 = (C(t) - S(u, v)).Sv, with its derivatives,
/// for math_FunctionSetRoot. Each root found (GetStateNumber) is stored with its square distance, unless its curve
/// parameter repeats one already found.
/// </summary>
/// <remarks>
/// OCCT keeps pointers to the curve and the surface; the port keeps the adaptors, which must outlive the function.
/// <see cref="SquareDistances"/>, <see cref="PointsOnCurve"/> and <see cref="PointsOnSurf"/> return the stored
/// sequences themselves, for the caller to change (OCCT's non-const references).
/// </remarks>
public class Extrema_FuncExtCS : FunctionSetWithDerivatives
{
    private Adaptor3d_Curve? myC;
    private Adaptor3d_Surface? myS;
    private Point3D myP1 = new();
    private Point3D myP2 = new();
    private double myt;
    private double myU;
    private double myV;
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<Extrema_POnCurv> myPoint1 = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPoint2 = new();
    private bool myCinit;
    private bool mySinit;

    /*-----------------------------------------------------------------------------
     Function to search for an extremal distance between a curve C and a surface S.
    { F1(t,u,v) = (C(t)-S(u,v)).Dtc(t) }
    { F2(t,u,v) = (C(t)-S(u,v)).Dus(u,v) }
    { F3(t,u,v) = (C(t)-S(u,v)).Dvs(u,v) }
    { Dtf1(t,u,v) = Dtc(t).Dtc(t)+(C(t)-S(u,v)).Dttc(t)
                  = ||Dtc(t)||**2+(C(t)-S(u,v)).Dttc(t) }
    { Duf1(t,u,v) = -Dus(u,v).Dtc(t) }
    { Dvf1(t,u,v) = -Dvs(u,v).Dtc(t) }
    { Dtf2(t,u,v) = Dtc(t).Dus(u,v) }
    { Duf2(t,u,v) = -Dus(u,v).Dus(u,v)+(C(t)-S(u,v)).Duus(u,v)
                  = -||Dus(u,v)||**2+(C(t)-S(u,v)).Duus(u,v) }
    { Dvf2(t,u,v) = -Dvs(u,v).Dus(u,v)+(C(t)-S(u,v)).Duvs(u,v) }
    { Dtf3(t,u,v) = Dtc(t).Dvs(u,v) }
    { Duf3(t,u,v) = -Dus(u,v).Dvs(u,v)+(C(t)-S(u,v)).Duvs(u,v) }
    { Dvf3(t,u,v) = -Dvs(u,v).Dvs(u,v)+(C(t)-S(u,v)).Dvvs(u,v) }
    ----------------------------------------------------------------------------*/

    public Extrema_FuncExtCS()
    {
        myC = null;
        myS = null;
        myt = 0.0;
        myU = 0.0;
        myV = 0.0;
        myCinit = false;
        mySinit = false;
    }

    public Extrema_FuncExtCS(Adaptor3d_Curve C, Adaptor3d_Surface S)
    {
        Initialize(C, S);
    }

    /// <summary>Sets the curve and the surface, and forgets the extrema found.</summary>
    public void Initialize(Adaptor3d_Curve C, Adaptor3d_Surface S)
    {
        myC = C;
        myS = S;
        myCinit = true;
        mySinit = true;
        myPoint1.Clear();
        myPoint2.Clear();
        mySqDist.Clear();
    }

    public override int NbVariables() => (3);

    public override int NbEquations() => (3);

    /// <summary>Calculates Fi(t, u, v).</summary>
    public override bool Value(math_Vector UV, math_Vector F)
    {
        if (!myCinit || !mySinit)
        {
            throw new TypeMismatchException();
        }

        myt = UV[1];
        myU = UV[2];
        myV = UV[3];

        myC!.D1(myt, out myP1, out Vector3D Dtc);
        myS!.D1(myU, myV, out myP2, out Vector3D Dus, out Vector3D Dvs);

        Vector3D P1P2 = new(myP2, myP1);

        F[1] = P1P2.Dot(Dtc);
        F[2] = P1P2.Dot(Dus);
        F[3] = P1P2.Dot(Dvs);

        return true;
    }

    /// <summary>Calculates Fi'(t, u, v).</summary>
    public override bool Derivatives(math_Vector UV, Matrix DF)
    {
        math_Vector F = new(1, 3);
        return Values(UV, F, DF);
    }

    /// <summary>Calculates Fi(t, u, v) and Fi'(t, u, v).</summary>
    public override bool Values(math_Vector UV, math_Vector F, Matrix Df)
    {
        if (!myCinit || !mySinit)
        {
            throw new TypeMismatchException();
        }

        myt = UV[1];
        myU = UV[2];
        myV = UV[3];

        myC!.D2(myt, out myP1, out Vector3D Dtc, out Vector3D Dttc);
        myS!.D2(myU, myV, out myP2, out Vector3D Dus, out Vector3D Dvs, out Vector3D Duus, out Vector3D Dvvs, out Vector3D Duvs);

        Vector3D P1P2 = new(myP2, myP1);

        F[1] = P1P2.Dot(Dtc);
        F[2] = P1P2.Dot(Dus);
        F[3] = P1P2.Dot(Dvs);

        Df[1, 1] = Dtc.SquareMagnitude() + P1P2.Dot(Dttc);
        Df[1, 2] = -Dus.Dot(Dtc);
        Df[1, 3] = -Dvs.Dot(Dtc);

        Df[2, 1] = -Df[1, 2]; // Dtc.Dot(Dus);
        Df[2, 2] = -Dus.SquareMagnitude() + P1P2.Dot(Duus);
        Df[2, 3] = -Dvs.Dot(Dus) + P1P2.Dot(Duvs);

        Df[3, 1] = -Df[1, 3]; // Dtc.Dot(Dvs);
        Df[3, 2] = Df[2, 3]; // -Dus.Dot(Dvs)+P1P2.Dot(Duvs);
        Df[3, 3] = -Dvs.SquareMagnitude() + P1P2.Dot(Dvvs);

        return true;
    }

    /// <summary>Saves the extremum found, unless its curve parameter repeats one already saved.</summary>
    public override int GetStateNumber()
    {
        if (!myCinit || !mySinit)
        {
            throw new TypeMismatchException();
        }
        // comparison of solution with previous solutions
        double tol2d = global::Kerf.Engine.Foundation.Precision.SquarePConfusion();
        int i = 1, nbSol = mySqDist.Length();
        for (; i <= nbSol; i++)
        {
            double aT = myPoint1[i].Parameter();
            aT -= myt;
            aT *= aT;
            if (aT <= tol2d)
            {
                break;
            }
        }
        if (i <= nbSol)
        {
            return 0;
        }
        mySqDist.Append(myP1.SquareDistance(myP2));
        myPoint1.Append(new Extrema_POnCurv(myt, myP1));
        myPoint2.Append(new Extrema_POnSurf(myU, myV, myP2));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int NbExt() => mySqDist.Length();

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int N)
    {
        if (!myCinit || !mySinit)
        {
            throw new TypeMismatchException();
        }
        return mySqDist.Value(N);
    }

    /// <summary>The Nth extremum on the curve.</summary>
    public Extrema_POnCurv PointOnCurve(int N)
    {
        if (!myCinit || !mySinit)
        {
            throw new TypeMismatchException();
        }
        return myPoint1.Value(N);
    }

    /// <summary>The Nth extremum on the surface.</summary>
    public Extrema_POnSurf PointOnSurface(int N)
    {
        if (!myCinit || !mySinit)
        {
            throw new TypeMismatchException();
        }
        return myPoint2.Value(N);
    }

    /// <summary>The stored square distances (shared: the caller may change them).</summary>
    public NCollection_Sequence<double> SquareDistances() => mySqDist;

    /// <summary>The stored points on the curve (shared: the caller may change them).</summary>
    public NCollection_Sequence<Extrema_POnCurv> PointsOnCurve() => myPoint1;

    /// <summary>The stored points on the surface (shared: the caller may change them).</summary>
    public NCollection_Sequence<Extrema_POnSurf> PointsOnSurf() => myPoint2;
}
