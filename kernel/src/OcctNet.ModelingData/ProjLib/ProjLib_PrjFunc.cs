// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjFunc.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1997-11-06
// Created by: Roman BORISOV

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function whose zero is the orthogonal projection of a curve point on a surface: the gradient of
/// <c>|S(u, v) - C(t)|</c> in two of the variables (t, u, v), the third one fixed (1: t, 2: u, 3: v).
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's adaptors, as OCCT keeps its pointers: they must outlive
/// the function. <see cref="Values"/> remembers the last point it was evaluated at, which
/// <see cref="Solution"/> returns.
/// </remarks>
public class ProjLib_PrjFunc : FunctionSetWithDerivatives
{
    private readonly Adaptor3d_Curve myCurve;
    private readonly Adaptor3d_Surface mySurface;
    private double myt;
    private double myU;
    private double myV;
    private readonly int myFix;
    private readonly double myNorm;

    /// <exception cref="ConstructionErrorException"><paramref name="Fix"/> is not 1, 2 or 3.</exception>
    public ProjLib_PrjFunc(Adaptor3d_Curve C, double FixVal, Adaptor3d_Surface S, int Fix)
    {
        myCurve = C;
        mySurface = S;
        myt = 0;
        myU = 0;
        myV = 0;
        myFix = Fix;
        myNorm = Min(1.0, Min(mySurface.UResolution(1.0), mySurface.VResolution(1.0)));
        // myNorm=1.;
        switch (myFix)
        {
            case 1:
                myt = FixVal;
                break;
            case 2:
                myU = FixVal;
                break;
            case 3:
                myV = FixVal;
                break;
            default:
                throw new ConstructionErrorException();
        }
    }

    /// <summary>The number of variables of the function: 2.</summary>
    public override int NbVariables() => 2;

    /// <summary>The number of equations of the function: 2.</summary>
    public override int NbEquations() => 2;

    public override bool Value(math_Vector X, math_Vector F)
    {
        Matrix D = new(1, 2, 1, 2);
        return Values(X, F, D);
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        math_Vector F = new(1, 2);
        return Values(X, F, D);
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        double u = 0.0, v = 0.0, t = 0.0;
        switch (myFix)
        {
            case 1:
                t = myt;
                u = X[1];
                v = X[2];
                break;
            case 2:
                t = X[1];
                u = myU;
                v = X[2];
                break;
            case 3:
                t = X[1];
                u = X[2];
                v = myV;
                break;
        }

        /*  if(X(1) > mySup.X() || X(1) < myInf.X()) return false;
          if(X(2) > mySup.Y() || X(2) < myInf.Y()) return false;
        */
        Point3D S, C;
        Vector3D DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DC1_t;
        myCurve.D1(t, out C, out DC1_t);
        mySurface.D2(u, v, out S, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv);

        Vector3D V = new(C, S);

        F[1] = V * DS1_u * myNorm;
        F[2] = V * DS1_v * myNorm;

        switch (myFix)
        {
            case 1:
                D[1, 1] = (DS1_u.SquareMagnitude() + V * DS2_u) * myNorm; // dE1/du
                D[1, 2] = (DS1_v * DS1_u + V * DS2_uv) * myNorm;          // dE1/dv
                D[2, 1] = D[1, 2];                                        // dE2/du
                D[2, 2] = (DS1_v.SquareMagnitude() + V * DS2_v) * myNorm; // dE2/dv
                break;
            case 2:
                D[1, 1] = (-DC1_t * DS1_u) * myNorm;                      // dE1/dt
                D[1, 2] = (DS1_v * DS1_u + V * DS2_uv) * myNorm;          // dE1/dv
                D[2, 1] = (-DC1_t * DS1_v) * myNorm;                      // dE2/dt
                D[2, 2] = (DS1_v.SquareMagnitude() + V * DS2_v) * myNorm; // dE2/dv
                break;
            case 3:
                D[1, 1] = -DC1_t * DS1_u * myNorm;                        // dE1/dt
                D[1, 2] = (DS1_u.SquareMagnitude() + V * DS2_u) * myNorm; // dE1/du
                D[2, 1] = -DC1_t * DS1_v * myNorm;                        // dE2/dt
                D[2, 2] = (DS1_v * DS1_u + V * DS2_uv) * myNorm;          // dE2/du
                break;
        }

        myU = u;
        myV = v;
        myt = t;

        return true;
    }

    /// <summary>
    /// The point of the last evaluation in the two free variables: (u, v) for Fix 1, (t, v) for Fix 2, (t, u) for
    /// Fix 3.
    /// </summary>
    public Point2D Solution()
    {
        switch (myFix)
        {
            case 1:
                return new Point2D(myU, myV);
            case 2:
                return new Point2D(myt, myV);
            case 3:
                return new Point2D(myt, myU);
        }
        // For NT, even if we never reach this point.
        return new Point2D(0.0, 0.0);
    }
}
