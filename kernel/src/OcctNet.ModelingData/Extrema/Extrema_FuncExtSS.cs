// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The system whose roots are the extrema of the distance between two surfaces S1 and S2:
/// (S1(u1, v1) - S2(u2, v2)) dotted with S1u, S1v, S2u and S2v, with its derivatives, for math_FunctionSetRoot.
/// Every root found (GetStateNumber) is stored with its square distance.
/// </summary>
/// <remarks>
/// OCCT keeps pointers to the surfaces; the port keeps the adaptors, which must outlive the function.
/// </remarks>
public class Extrema_FuncExtSS : FunctionSetWithDerivatives
{
    private Adaptor3d_Surface? myS1;
    private Adaptor3d_Surface? myS2;
    private Point3D myP1 = new();
    private Point3D myP2 = new();
    private double myU1;
    private double myV1;
    private double myU2;
    private double myV2;
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPoint1 = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPoint2 = new();
    private bool myS1init;
    private bool myS2init;

    public Extrema_FuncExtSS()
    {
        myS1 = null;
        myS2 = null;
        myU1 = 0.0;
        myV1 = 0.0;
        myU2 = 0.0;
        myV2 = 0.0;
        myS1init = false;
        myS2init = false;
    }

    public Extrema_FuncExtSS(Adaptor3d_Surface S1, Adaptor3d_Surface S2)
    {
        myU1 = 0.0;
        myV1 = 0.0;
        myU2 = 0.0;
        myV2 = 0.0;
        myS1 = S1;
        myS2 = S2;
        myS1init = true;
        myS2init = true;
    }

    /// <summary>Sets the surfaces, and forgets the extrema found.</summary>
    public void Initialize(Adaptor3d_Surface S1, Adaptor3d_Surface S2)
    {
        myS1 = S1;
        myS2 = S2;
        myS1init = true;
        myS2init = true;
        myPoint1.Clear();
        myPoint2.Clear();
        mySqDist.Clear();
    }

    public override int NbVariables() => 4;

    public override int NbEquations() => 4;

    /// <summary>Calculates Fi(U1, V1, U2, V2).</summary>
    public override bool Value(math_Vector UV, math_Vector F)
    {
        if (!myS1init || !myS2init)
        {
            throw new TypeMismatchException();
        }
        myU1 = UV[1];
        myV1 = UV[2];
        myU2 = UV[3];
        myV2 = UV[4];
        myS1!.D1(myU1, myV1, out myP1, out Vector3D Du1s1, out Vector3D Dv1s1);
        myS2!.D1(myU2, myV2, out myP2, out Vector3D Du2s2, out Vector3D Dv2s2);

        Vector3D P1P2 = new(myP2, myP1);

        F[1] = P1P2.Dot(Du1s1);
        F[2] = P1P2.Dot(Dv1s1);
        F[3] = P1P2.Dot(Du2s2);
        F[4] = P1P2.Dot(Dv2s2);

        return true;
    }

    /// <summary>Calculates Fi'(U1, V1, U2, V2).</summary>
    public override bool Derivatives(math_Vector UV, Matrix Df)
    {
        math_Vector F = new(1, 4);
        return Values(UV, F, Df);
    }

    /// <summary>Calculates Fi(U1, V1, U2, V2) and Fi'(U1, V1, U2, V2).</summary>
    public override bool Values(math_Vector UV, math_Vector F, Matrix Df)
    {
        if (!myS1init || !myS2init)
        {
            throw new TypeMismatchException();
        }
        myU1 = UV[1];
        myV1 = UV[2];
        myU2 = UV[3];
        myV2 = UV[4];
        myS1!.D2(myU1, myV1, out myP1, out Vector3D Du1s1, out Vector3D Dv1s1, out Vector3D Du1u1s1, out Vector3D Dv1v1s1,
            out Vector3D Du1v1s1);
        myS2!.D2(myU2, myV2, out myP2, out Vector3D Du2s2, out Vector3D Dv2s2, out Vector3D Du2u2s2, out Vector3D Dv2v2s2,
            out Vector3D Du2v2s2);

        Vector3D P1P2 = new(myP2, myP1);

        F[1] = P1P2.Dot(Du1s1);
        F[2] = P1P2.Dot(Dv1s1);
        F[3] = P1P2.Dot(Du2s2);
        F[4] = P1P2.Dot(Dv2s2);

        Df[1, 1] = Du1s1.SquareMagnitude() + P1P2.Dot(Du1u1s1);
        Df[1, 2] = Dv1s1.Dot(Du1s1) + P1P2.Dot(Du1v1s1);
        Df[1, 3] = -Du2s2.Dot(Du1s1);
        Df[1, 4] = -Dv2s2.Dot(Du1s1);

        Df[2, 1] = Df[1, 2]; // Du1s1.Dot(Dv1s1) + P1P2.Dot(Du1v1s1);
        Df[2, 2] = Dv1s1.SquareMagnitude() + P1P2.Dot(Dv1v1s1);
        Df[2, 3] = -Du2s2.Dot(Dv1s1);
        Df[2, 4] = -Dv2s2.Dot(Dv1s1);

        Df[3, 1] = -Df[1, 3]; // Du1s1.Dot(Du2s2);
        Df[3, 2] = -Df[2, 3]; // Dv1s1.Dot(Du2s2);
        Df[3, 3] = -Du2s2.SquareMagnitude() + P1P2.Dot(Du2u2s2);
        Df[3, 4] = -Dv2s2.Dot(Du2s2) + P1P2.Dot(Du2v2s2);

        Df[4, 1] = -Df[1, 4]; // Du1s1.Dot(Dv2s2);
        Df[4, 2] = -Df[2, 4]; // Dv1s1.Dot(Dv2s2);
        Df[4, 3] = Df[3, 4]; // -Du2s2.Dot(Dv2s2) + P1P2.Dot(Du2v2s2);
        Df[4, 4] = -Dv2s2.SquareMagnitude() + P1P2.Dot(Dv2v2s2);

        return true;
    }

    /// <summary>Saves the extremum found.</summary>
    public override int GetStateNumber()
    {
        if (!myS1init || !myS2init)
        {
            throw new TypeMismatchException();
        }

        mySqDist.Append(myP1.SquareDistance(myP2));
        myPoint1.Append(new Extrema_POnSurf(myU1, myV1, myP1));
        myPoint2.Append(new Extrema_POnSurf(myU2, myV2, myP2));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int NbExt() => mySqDist.Length();

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int N)
    {
        if (!myS1init || !myS2init)
        {
            throw new TypeMismatchException();
        }
        return mySqDist.Value(N);
    }

    /// <summary>The Nth extremum on S1.</summary>
    public Extrema_POnSurf PointOnS1(int N)
    {
        if (!myS1init || !myS2init)
        {
            throw new TypeMismatchException();
        }
        return myPoint1.Value(N);
    }

    /// <summary>The Nth extremum on S2.</summary>
    public Extrema_POnSurf PointOnS2(int N)
    {
        if (!myS1init || !myS2init)
        {
            throw new TypeMismatchException();
        }
        return myPoint2.Value(N);
    }
}
