// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.Extrema_GlobOptFuncCC;

namespace Occt;

/// <summary>The file-static functions of Extrema_GlobOptFuncCC.cxx, for 3D and 2D curves.</summary>
internal static class Extrema_GlobOptFuncCC
{
    internal static int _NbVariables() => 2;

    // 3d _Value
    internal static bool _Value(Adaptor3d_Curve C1, Adaptor3d_Curve C2, math_Vector X, out double F)
    {
        double u = X[1];
        double v = X[2];

        if (u < C1.FirstParameter() || u > C1.LastParameter() || v < C2.FirstParameter() || v > C2.LastParameter())
        {
            F = 0.0; // OCCT leaves F unchanged (decision 32)
            return false;
        }

        F = C2.Value(v).Distance(C1.Value(u));
        return true;
    }

    // 2d _Value
    internal static bool _Value(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, math_Vector X, out double F)
    {
        double u = X[1];
        double v = X[2];

        if (u < C1.FirstParameter() || u > C1.LastParameter() || v < C2.FirstParameter() || v > C2.LastParameter())
        {
            F = 0.0; // OCCT leaves F unchanged (decision 32)
            return false;
        }

        F = C2.Value(v).Distance(C1.Value(u));
        return true;
    }

    // F = (x2(v) - x1(u))^2 + (y2(v) - y1(u))^2 + (z2(v) - z1(u))^2

    // 3d _Gradient
    internal static bool _Gradient(Adaptor3d_Curve C1, Adaptor3d_Curve C2, math_Vector X, math_Vector G)
    {
        if (X[1] < C1.FirstParameter() || X[1] > C1.LastParameter() || X[2] < C2.FirstParameter()
            || X[2] > C2.LastParameter())
        {
            return false;
        }

        C1.D1(X[1], out Point3D C1D0, out Vector3D C1D1);
        C2.D1(X[2], out Point3D C2D0, out Vector3D C2D1);

        G[1] = -(C2D0.X() - C1D0.X()) * C1D1.X() - (C2D0.Y() - C1D0.Y()) * C1D1.Y()
               - (C2D0.Z() - C1D0.Z()) * C1D1.Z();
        G[2] = (C2D0.X() - C1D0.X()) * C2D1.X() + (C2D0.Y() - C1D0.Y()) * C2D1.Y()
               + (C2D0.Z() - C1D0.Z()) * C2D1.Z();
        G *= 2.0;
        return true;
    }

    // 2d _Gradient
    internal static bool _Gradient(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, math_Vector X, math_Vector G)
    {
        if (X[1] < C1.FirstParameter() || X[1] > C1.LastParameter() || X[2] < C2.FirstParameter()
            || X[2] > C2.LastParameter())
        {
            return false;
        }

        C1.D1(X[1], out Point2D C1D0, out Vector2D C1D1);
        C2.D1(X[2], out Point2D C2D0, out Vector2D C2D1);

        G[1] = -(C2D0.X() - C1D0.X()) * C1D1.X() - (C2D0.Y() - C1D0.Y()) * C1D1.Y();

        G[2] = (C2D0.X() - C1D0.X()) * C2D1.X() + (C2D0.Y() - C1D0.Y()) * C2D1.Y();
        G *= 2.0;

        return true;
    }

    // 3d _Hessian
    internal static bool _Hessian(Adaptor3d_Curve C1, Adaptor3d_Curve C2, math_Vector X, Matrix H)
    {
        if (X[1] < C1.FirstParameter() || X[1] > C1.LastParameter() || X[2] < C2.FirstParameter()
            || X[2] > C2.LastParameter())
        {
            return false;
        }

        C1.D2(X[1], out Point3D C1D0, out Vector3D C1D1, out Vector3D C1D2);
        C2.D2(X[2], out Point3D C2D0, out Vector3D C2D1, out Vector3D C2D2);

        H[1, 1] = C1D1.X() * C1D1.X() + C1D1.Y() * C1D1.Y() + C1D1.Z() * C1D1.Z()
                  - (C2D0.X() - C1D0.X()) * C1D2.X() - (C2D0.Y() - C1D0.Y()) * C1D2.Y()
                  - (C2D0.Z() - C1D0.Z()) * C1D2.Z();

        H[1, 2] = -C2D1.X() * C1D1.X() - C2D1.Y() * C1D1.Y() - C2D1.Z() * C1D1.Z();

        H[2, 1] = H[1, 2];

        H[2, 2] = C2D1.X() * C2D1.X() + C2D1.Y() * C2D1.Y() + C2D1.Z() * C2D1.Z()
                  + (C2D0.X() - C1D0.X()) * C2D2.X() + (C2D0.Y() - C1D0.Y()) * C2D2.Y()
                  + (C2D0.Z() - C1D0.Z()) * C2D2.Z();
        H *= 2.0;
        return true;
    }

    // 2d _Hessian
    internal static bool _Hessian(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, math_Vector X, Matrix H)
    {
        if (X[1] < C1.FirstParameter() || X[1] > C1.LastParameter() || X[2] < C2.FirstParameter()
            || X[2] > C2.LastParameter())
        {
            return false;
        }

        C1.D2(X[1], out Point2D C1D0, out Vector2D C1D1, out Vector2D C1D2);
        C2.D2(X[2], out Point2D C2D0, out Vector2D C2D1, out Vector2D C2D2);

        H[1, 1] = C1D1.X() * C1D1.X() + C1D1.Y() * C1D1.Y() - (C2D0.X() - C1D0.X()) * C1D2.X()
                  - (C2D0.Y() - C1D0.Y()) * C1D2.Y();

        H[1, 2] = -C2D1.X() * C1D1.X() - C2D1.Y() * C1D1.Y();

        H[2, 1] = H[1, 2];

        H[2, 2] = C2D1.X() * C2D1.X() + C2D1.Y() * C2D1.Y() + (C2D0.X() - C1D0.X()) * C2D2.X()
                  + (C2D0.Y() - C1D0.Y()) * C2D2.Y();
        H *= 2.0;
        return true;
    }
}

/// <summary>
/// The distance between two curves (3D or 2D) as a function of their parameters (u, v), for math_GlobOptMin; false
/// outside the curves' ranges.
/// </summary>
/// <remarks>Keeps references to the curves, as OCCT does.</remarks>
public class Extrema_GlobOptFuncCCC0 : MultipleVarFunction
{
    private readonly Adaptor3d_Curve? myC1_3d, myC2_3d;
    private readonly Adaptor2d_Curve2d? myC1_2d, myC2_2d;
    private readonly int myType;

    public Extrema_GlobOptFuncCCC0(Adaptor3d_Curve C1, Adaptor3d_Curve C2)
    {
        myC1_3d = C1;
        myC2_3d = C2;
        myC1_2d = null;
        myC2_2d = null;
        myType = 1;
    }

    public Extrema_GlobOptFuncCCC0(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2)
    {
        myC1_3d = null;
        myC2_3d = null;
        myC1_2d = C1;
        myC2_2d = C2;
        myType = 2;
    }

    public override int VariableCount => _NbVariables();

    public override bool Value(math_Vector X, out double F)
    {
        if (myType == 1)
        {
            return _Value(myC1_3d!, myC2_3d!, X, out F);
        }
        else
        {
            return _Value(myC1_2d!, myC2_2d!, X, out F);
        }
    }
}

/// <summary>The distance between two curves with its gradient (see <see cref="Extrema_GlobOptFuncCCC0"/>); the gradient
/// is the one of the square distance.</summary>
public class Extrema_GlobOptFuncCCC1 : MultipleVarFunctionWithGradient
{
    private readonly Adaptor3d_Curve? myC1_3d, myC2_3d;
    private readonly Adaptor2d_Curve2d? myC1_2d, myC2_2d;
    private readonly int myType;

    public Extrema_GlobOptFuncCCC1(Adaptor3d_Curve C1, Adaptor3d_Curve C2)
    {
        myC1_3d = C1;
        myC2_3d = C2;
        myC1_2d = null;
        myC2_2d = null;
        myType = 1;
    }

    public Extrema_GlobOptFuncCCC1(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2)
    {
        myC1_3d = null;
        myC2_3d = null;
        myC1_2d = C1;
        myC2_2d = C2;
        myType = 2;
    }

    public override int VariableCount => _NbVariables();

    public override bool Value(math_Vector X, out double F)
    {
        if (myType == 1)
        {
            return _Value(myC1_3d!, myC2_3d!, X, out F);
        }
        else
        {
            return _Value(myC1_2d!, myC2_2d!, X, out F);
        }
    }

    public override bool Gradient(math_Vector X, math_Vector G)
    {
        if (myType == 1)
        {
            return _Gradient(myC1_3d!, myC2_3d!, X, G);
        }
        else
        {
            return _Gradient(myC1_2d!, myC2_2d!, X, G);
        }
    }

    public override bool Values(math_Vector X, out double F, math_Vector G) => (Value(X, out F) && Gradient(X, G));
}

/// <summary>The distance between two curves with its gradient and hessian (see <see cref="Extrema_GlobOptFuncCCC0"/>);
/// the gradient and the hessian are those of the square distance.</summary>
public class Extrema_GlobOptFuncCCC2 : MultipleVarFunctionWithHessian
{
    private readonly Adaptor3d_Curve? myC1_3d, myC2_3d;
    private readonly Adaptor2d_Curve2d? myC1_2d, myC2_2d;
    private readonly int myType;

    public Extrema_GlobOptFuncCCC2(Adaptor3d_Curve C1, Adaptor3d_Curve C2)
    {
        myC1_3d = C1;
        myC2_3d = C2;
        myC1_2d = null;
        myC2_2d = null;
        myType = 1;
    }

    public Extrema_GlobOptFuncCCC2(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2)
    {
        myC1_3d = null;
        myC2_3d = null;
        myC1_2d = C1;
        myC2_2d = C2;
        myType = 2;
    }

    public override int VariableCount => _NbVariables();

    public override bool Value(math_Vector X, out double F)
    {
        if (myType == 1)
        {
            return _Value(myC1_3d!, myC2_3d!, X, out F);
        }
        else
        {
            return _Value(myC1_2d!, myC2_2d!, X, out F);
        }
    }

    public override bool Gradient(math_Vector X, math_Vector G)
    {
        if (myType == 1)
        {
            return _Gradient(myC1_3d!, myC2_3d!, X, G);
        }
        else
        {
            return _Gradient(myC1_2d!, myC2_2d!, X, G);
        }
    }

    public override bool Values(math_Vector X, out double F, math_Vector G) => (Value(X, out F) && Gradient(X, G));

    public override bool Values(math_Vector X, out double F, math_Vector G, Matrix H)
    {
        bool isHessianComputed = false;
        if (myType == 1)
        {
            isHessianComputed = _Hessian(myC1_3d!, myC2_3d!, X, H);
        }
        else
        {
            isHessianComputed = _Hessian(myC1_2d!, myC2_2d!, X, H);
        }

        return (Value(X, out F) && Gradient(X, G) && isHessianComputed);
    }
}
