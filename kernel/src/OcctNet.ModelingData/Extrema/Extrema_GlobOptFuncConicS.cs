// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncConicS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncConicS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The square distance between a conic and a surface as a function of the surface parameters:
/// F(u, v) = min over the conic's projection of S(u, v) and its two end points of the square distance, for
/// global optimisation over the surface only.
/// </summary>
/// <remarks>
/// The curve and the surface are kept by reference and must outlive the function, as in OCCT. The constructors
/// without a curve leave it unset until <see cref="LoadConic"/> (OCCT leaves the pointer and the curve type
/// uninitialised). Outside the parameter range <see cref="Value"/> returns false and writes 0 (decision 221).
/// </remarks>
public class Extrema_GlobOptFuncConicS : MultipleVarFunction
{
    private Adaptor3d_Curve? myC;
    private readonly Adaptor3d_Surface myS;
    private CurveKind myCType;
    private Line3D myLin = new();
    private Circle3D myCirc = new();
    private Ellipse3D myElips = new();
    private Hyperbola3D myHypr = new();
    private Parabola3D myParab = new();
    private Point3D myCPf = new();
    private Point3D myCPl = new();
    // Boundaries
    private double myTf;
    private double myTl;
    private readonly double myUf;
    private readonly double myUl;
    private readonly double myVf;
    private readonly double myVl;

    // F(u, v) = Conic.SquareDistance(myS(u, v))

    private void value(double su, double sv, out double F)
    {
        double ct;
        Point3D aPS = myS.Value(su, sv);
        switch (myCType)
        {
            case Line:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myLin, aPS);
                break;
            case Circle:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myCirc, aPS);
                break;
            case Ellipse:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myElips, aPS);
                break;
            case Hyperbola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myHypr, aPS);
                break;
            case Parabola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myParab, aPS);
                break;
            default:
                F = global::Kerf.Engine.Foundation.Precision.Infinite();
                return;
        }
        //
        if (myCType == Circle || myCType == Ellipse)
        {
            if (myTl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ct += 2.0 * M_PI;
            }
        }
        F = RealLast();
        if (ct >= myTf && ct <= myTl)
        {
            Point3D aPC = myC!.Value(ct);
            F = Min(F, aPS.SquareDistance(aPC));
        }
        F = Min(F, aPS.SquareDistance(myCPf));
        F = Min(F, aPS.SquareDistance(myCPl));
    }

    private bool checkInputData(math_Vector X, out double su, out double sv)
    {
        int aStartIndex = X.Lower();
        su = X[aStartIndex];
        sv = X[aStartIndex + 1];

        return su >= myUf && su <= myUl && sv >= myVf && sv <= myVl;
    }

    /// <summary>The function over the given surface range; the conic is set by <see cref="LoadConic"/>.</summary>
    public Extrema_GlobOptFuncConicS(Adaptor3d_Surface S, double theUf, double theUl, double theVf, double theVl)
    {
        myS = S;
        myUf = theUf;
        myUl = theUl;
        myVf = theVf;
        myVl = theVl;
    }

    /// <summary>The function over the whole surface; the conic is set by <see cref="LoadConic"/>.</summary>
    public Extrema_GlobOptFuncConicS(Adaptor3d_Surface S)
    {
        myS = S;
        myUf = S.FirstUParameter();
        myUl = S.LastUParameter();
        myVf = S.FirstVParameter();
        myVl = S.LastVParameter();
    }

    /// <summary>The function of a conic over its whole range and a surface.</summary>
    public Extrema_GlobOptFuncConicS(Adaptor3d_Curve C, Adaptor3d_Surface S)
    {
        myS = S;
        myUf = S.FirstUParameter();
        myUl = S.LastUParameter();
        myVf = S.FirstVParameter();
        myVl = S.LastVParameter();
        double aCTf = C.FirstParameter();
        double aCTl = C.LastParameter();
        LoadConic(C, aCTf, aCTl);
    }

    /// <summary>Sets the conic and its parameter range.</summary>
    public void LoadConic(Adaptor3d_Curve C, double theTf, double theTl)
    {
        myC = C;
        myTf = theTf;
        myTl = theTl;
        if (myC.IsPeriodic())
        {
            double aTMax = 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (myTf > aTMax || myTf < -global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(myTl - myTf) > aTMax)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, Min(Math.Abs(myTl - myTf) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                    ref myTf, ref myTl);
            }
        }
        myCPf = myC.Value(myTf);
        myCPl = myC.Value(myTl);
        myCType = myC.GetType();
        switch (myCType)
        {
            case Line:
                myLin = myC.Line();
                break;
            case Circle:
                myCirc = myC.Circle();
                break;
            case Ellipse:
                myElips = myC.Ellipse();
                break;
            case Hyperbola:
                myHypr = myC.Hyperbola();
                break;
            case Parabola:
                myParab = myC.Parabola();
                break;
            default:
                break;
        }
    }

    public override int VariableCount => 2;

    public override bool Value(math_Vector X, out double F)
    {
        if (!checkInputData(X, out double su, out double sv))
        {
            F = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        value(su, sv, out F);
        return !global::Kerf.Engine.Foundation.Precision.IsInfinite(F);
    }

    /// <summary>The parameter on the conic for the point of the surface at theUV.</summary>
    public double ConicParameter(math_Vector theUV)
    {
        double ct;
        Point3D aPS = myS.Value(theUV[1], theUV[2]);
        switch (myCType)
        {
            case Line:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myLin, aPS);
                break;
            case Circle:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myCirc, aPS);
                break;
            case Ellipse:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myElips, aPS);
                break;
            case Hyperbola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myHypr, aPS);
                break;
            case Parabola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(myParab, aPS);
                break;
            default:
                ct = myTf;
                return ct;
        }
        //
        if (myCType == Circle || myCType == Ellipse)
        {
            if (myTl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ct += 2.0 * M_PI;
            }
        }
        double F = RealLast();
        if (ct >= myTf && ct <= myTl)
        {
            Point3D aPC = myC!.Value(ct);
            F = Min(F, aPS.SquareDistance(aPC));
        }
        double Fext = aPS.SquareDistance(myCPf);
        if (Fext < F)
        {
            F = Fext;
            ct = myTf;
        }
        Fext = aPS.SquareDistance(myCPl);
        if (Fext < F)
        {
            F = Fext;
            ct = myTl;
        }
        return ct;
    }
}
