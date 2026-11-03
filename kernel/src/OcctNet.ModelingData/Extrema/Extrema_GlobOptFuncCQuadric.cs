// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCQuadric.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCQuadric.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The square distance between a curve and a quadric as a function of the curve parameter:
/// F(t) = min over the quadric's projection of C(t) and its four corner points of the square distance, for global
/// optimisation over the curve only.
/// </summary>
/// <remarks>
/// The curve and the surface are kept by reference and must outlive the function, as in OCCT. The constructors
/// without a surface leave it unset until <see cref="LoadQuad"/>. Outside the parameter range <see cref="Value"/>
/// returns false and writes 0 (decision 221).
/// </remarks>
public class Extrema_GlobOptFuncCQuadric : MultipleVarFunction
{
    private readonly Adaptor3d_Curve myC;
    private Adaptor3d_Surface? myS;
    private SurfaceKind mySType;
    private Plane3D myPln = new();
    private Cone myCone = new();
    private Cylinder myCylinder = new();
    private Sphere mySphere = new();
    private Torus myTorus = new();
    private readonly Point3D[] myPTrim = [new(), new(), new(), new()];
    // Boundaries
    private readonly double myTf;
    private readonly double myTl;
    private double myUf;
    private double myUl;
    private double myVf;
    private double myVl;

    private void value(double ct, out double F)
    {
        double u, v;
        //
        Point3D aCP = myC.Value(ct);
        switch (mySType)
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myPln, aCP, out u, out v);
                break;
            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myCylinder, aCP, out u, out v);
                break;
            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myCone, aCP, out u, out v);
                break;
            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(mySphere, aCP, out u, out v);
                break;
            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myTorus, aCP, out u, out v);
                break;
            default:
                F = global::Kerf.Engine.Foundation.Precision.Infinite();
                return;
        }
        //
        if (mySType != Plane)
        {
            if (myUl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                u += 2.0 * M_PI;
            }
        }
        if (mySType == SurfaceKind.Torus)
        {
            if (myVl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                v += 2.0 * M_PI;
            }
        }

        F = RealLast();
        if (u >= myUf && u <= myUl && v >= myVf && v <= myVl)
        {
            Point3D aPS = myS!.Value(u, v);
            F = Min(F, aCP.SquareDistance(aPS));
        }
        int i;
        for (i = 0; i < 4; ++i)
        {
            F = Min(F, aCP.SquareDistance(myPTrim[i]));
        }
    }

    private bool checkInputData(math_Vector X, out double ct)
    {
        ct = X[X.Lower()];

        return ct >= myTf && ct <= myTl;
    }

    /// <summary>The function of a curve over its whole range and a quadric over its whole range.</summary>
    public Extrema_GlobOptFuncCQuadric(Adaptor3d_Curve C, Adaptor3d_Surface S)
    {
        myC = C;
        myTf = myC.FirstParameter();
        myTl = myC.LastParameter();
        double anUf = S.FirstUParameter(), anUl = S.LastUParameter();
        double aVf = S.FirstVParameter(), aVl = S.LastVParameter();
        LoadQuad(S, anUf, anUl, aVf, aVl);
    }

    /// <summary>The function of a curve over its whole range; the quadric is set by <see cref="LoadQuad"/>.</summary>
    public Extrema_GlobOptFuncCQuadric(Adaptor3d_Curve C)
    {
        myC = C;
        myTf = myC.FirstParameter();
        myTl = myC.LastParameter();
    }

    /// <summary>The function of a curve over [theTf, theTl]; the quadric is set by <see cref="LoadQuad"/>.</summary>
    public Extrema_GlobOptFuncCQuadric(Adaptor3d_Curve C, double theTf, double theTl)
    {
        myC = C;
        myTf = theTf;
        myTl = theTl;
    }

    /// <summary>Sets the quadric and its parameter range.</summary>
    public void LoadQuad(Adaptor3d_Surface S, double theUf, double theUl, double theVf, double theVl)
    {
        myS = S;
        myUf = theUf;
        myUl = theUl;
        myVf = theVf;
        myVl = theVl;
        //
        if (myS.IsUPeriodic())
        {
            double aTMax = 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (myUf > aTMax || myUf < -global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(myUl - myUf) > aTMax)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, Min(Math.Abs(myUl - myUf) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                    ref myUf, ref myUl);
            }
        }
        if (myS.IsVPeriodic())
        {
            double aTMax = 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (myVf > aTMax || myVf < -global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(myVl - myVf) > aTMax)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, Min(Math.Abs(myVl - myVf) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                    ref myVf, ref myVl);
            }
        }
        myPTrim[0] = myS.Value(myUf, myVf);
        myPTrim[1] = myS.Value(myUl, myVf);
        myPTrim[2] = myS.Value(myUl, myVl);
        myPTrim[3] = myS.Value(myUf, myVl);
        mySType = S.GetType();
        switch (mySType)
        {
            case Plane:
                myPln = myS.Plane();
                break;
            case SurfaceKind.Cylinder:
                myCylinder = myS.Cylinder();
                break;
            case SurfaceKind.Cone:
                myCone = myS.Cone();
                break;
            case SurfaceKind.Sphere:
                mySphere = myS.Sphere();
                break;
            case SurfaceKind.Torus:
                myTorus = myS.Torus();
                break;
            default:
                break;
        }
    }

    public override int VariableCount => 1;

    public override bool Value(math_Vector X, out double F)
    {
        if (!checkInputData(X, out double ct))
        {
            F = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        value(ct, out F);
        return !global::Kerf.Engine.Foundation.Precision.IsInfinite(F);
    }

    /// <summary>The parameters on the quadric (written to theUV) for the point of the curve at theCT.</summary>
    public void QuadricParameters(math_Vector theCT, math_Vector theUV)
    {
        double u, v;
        //
        // Arrays of extremity points parameters correspond to array of corner
        // points  myPTrim[]
        ReadOnlySpan<double> uext = [myUf, myUl, myUl, myUf];
        ReadOnlySpan<double> vext = [myVf, myVf, myVl, myVl];
        Point3D aCP = myC.Value(theCT[1]);
        switch (mySType)
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myPln, aCP, out u, out v);
                break;
            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myCylinder, aCP, out u, out v);
                break;
            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myCone, aCP, out u, out v);
                break;
            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(mySphere, aCP, out u, out v);
                break;
            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(myTorus, aCP, out u, out v);
                break;
            default:
                theUV[1] = myUf;
                theUV[2] = myUl; // OCCT writes the last U parameter as V (myVf intended); kept
                return;
        }
        //
        if (mySType != Plane)
        {
            if (myUl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                u += 2.0 * M_PI;
            }
        }
        if (mySType == SurfaceKind.Torus)
        {
            if (myVl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                v += 2.0 * M_PI;
            }
        }

        double F = RealLast();
        if (u >= myUf && u <= myUl && v >= myVf && v <= myVl)
        {
            Point3D aPS = myS!.Value(u, v);
            F = aCP.SquareDistance(aPS);
        }
        int i;
        for (i = 0; i < 4; ++i)
        {
            double Fi = aCP.SquareDistance(myPTrim[i]);
            if (Fi < F)
            {
                F = Fi;
                u = uext[i];
                v = vext[i];
            }
        }
        theUV[1] = u;
        theUV[2] = v;
    }
}
