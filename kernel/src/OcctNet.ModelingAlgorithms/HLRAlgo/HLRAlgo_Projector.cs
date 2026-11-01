// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The projector of the hidden-line removal: transforms points and vectors into the viewing coordinate system and
/// projects them, in parallel (axonometric) or perspective (with a focal length), and shoots lines from the eye.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>new()</c> is OCCT's empty constructor (the identity, parallel); <c>default</c>
/// is all zeros, so ported code writes <c>new()</c>. <see cref="Set"/> and <see cref="Scaled"/> are internal
/// mutators, with the constructors and <see cref="WithScaled"/> as their twins. For four exact view matrices (top,
/// front and two axonometric ones) OCCT projects with hard-coded formulas, as the port does. In
/// <see cref="Project(in gp_Pnt, ref gp_Pnt2d)"/> OCCT forgets to write the result for the first of them, so the
/// point is <c>ref</c> and keeps the caller's value there (decision 10104).
/// </remarks>
public partial struct HLRAlgo_Projector : IEquatable<HLRAlgo_Projector>
{
    // formula for derivating a perspective, from Mathematica
    //        X'[t]      X[t] Z'[t]
    // D1 =  -------- + -------------
    //           Z[t]          Z[t] 2
    //       1 - ----   f (1 - ----)
    //            f             f

    private int myType;
    private bool myPersp;
    private double myFocus;
    private gp_Trsf myScaledTrsf;
    private gp_Trsf myTrsf;
    private gp_Trsf myInvTrsf;
    private gp_Vec2d myD1;
    private gp_Vec2d myD2;
    private gp_Vec2d myD3;

    /// <summary>The identity, parallel projector.</summary>
    public HLRAlgo_Projector()
    {
        myType = -1;
        myPersp = false;
        myFocus = 0;
        myScaledTrsf = new gp_Trsf();
        myTrsf = new gp_Trsf();
        myInvTrsf = new gp_Trsf();
        myD1 = new gp_Vec2d();
        myD2 = new gp_Vec2d();
        myD3 = new gp_Vec2d();
        Scaled();
    }

    /// <summary>An axonometric (parallel) projector; <paramref name="CS"/> is the viewing coordinate system.</summary>
    public HLRAlgo_Projector(in gp_Ax2 CS)
        : this()
    {
        myPersp = false;
        myFocus = 0;
        myScaledTrsf.SetTransformation(new gp_Ax3(CS));
        Scaled();
        SetDirection();
    }

    /// <summary>A perspective projector of focal length <paramref name="Focus"/>; <paramref name="CS"/> is the viewing coordinate system.</summary>
    public HLRAlgo_Projector(in gp_Ax2 CS, double Focus)
        : this()
    {
        myPersp = true;
        myFocus = Focus;
        myScaledTrsf.SetTransformation(new gp_Ax3(CS));
        Scaled();
        SetDirection();
    }

    /// <summary>A projector with automatic min-max directions.</summary>
    public HLRAlgo_Projector(in gp_Trsf T, bool Persp, double Focus)
        : this()
    {
        myPersp = Persp;
        myFocus = Focus;
        myScaledTrsf = T;
        Scaled();
        SetDirection();
    }

    /// <summary>A projector with the given min-max directions.</summary>
    public HLRAlgo_Projector(in gp_Trsf T, bool Persp, double Focus, in gp_Vec2d v1, in gp_Vec2d v2, in gp_Vec2d v3)
        : this()
    {
        myPersp = Persp;
        myFocus = Focus;
        myScaledTrsf = T;
        myD1 = v1;
        myD2 = v2;
        myD3 = v3;
        Scaled();
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Set(in gp_Trsf T, bool Persp, double Focus)
    {
        myPersp = Persp;
        myFocus = Focus;
        myScaledTrsf = T;
        Scaled();
        SetDirection();
    }

    private static int TrsfType(in gp_Trsf Trsf)
    {
        gp_Mat Mat = Trsf.VectorialPart();
        if ((Math.Abs(Mat.Value(1, 1) - 1.0) < 1e-15) && (Math.Abs(Mat.Value(2, 2) - 1.0) < 1e-15)
            && (Math.Abs(Mat.Value(3, 3) - 1.0) < 1e-15))
        {
            return (1); //-- top
        }
        else if ((Math.Abs(Mat.Value(1, 1) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(Mat.Value(1, 2) + 0.5) < 1e-15) && (Math.Abs(Mat.Value(1, 3) - 0.5) < 1e-15)

                 && (Math.Abs(Mat.Value(2, 1) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(Mat.Value(2, 2) - 0.5) < 1e-15) && (Math.Abs(Mat.Value(2, 3) + 0.5) < 1e-15)

                 && (Math.Abs(Mat.Value(3, 1)) < 1e-15)
                 && (Math.Abs(Mat.Value(3, 2) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(Mat.Value(3, 3) - 0.7071067811865476) < 1e-15))
        {
            return (0); //--
        }
        else if ((Math.Abs(Mat.Value(1, 1) - 1.0) < 1e-15) && (Math.Abs(Mat.Value(2, 3) - 1.0) < 1e-15)
                 && (Math.Abs(Mat.Value(3, 2) + 1.0) < 1e-15))
        {
            return (2); //-- front
        }
        else if ((Math.Abs(Mat.Value(1, 1) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(Mat.Value(1, 2) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(Mat.Value(1, 3)) < 1e-15)

                 && (Math.Abs(Mat.Value(2, 1) + 0.5) < 1e-15) && (Math.Abs(Mat.Value(2, 2) - 0.5) < 1e-15)
                 && (Math.Abs(Mat.Value(2, 3) - 0.7071067811865476) < 1e-15)

                 && (Math.Abs(Mat.Value(3, 1) - 0.5) < 1e-15) && (Math.Abs(Mat.Value(3, 2) + 0.5) < 1e-15)
                 && (Math.Abs(Mat.Value(3, 3) - 0.7071067811865476) < 1e-15))
        {
            return (3); //-- axo
        }
        return (-1);
    }

    /// <summary>
    /// Computes with the given scale and translation (<paramref name="On"/>), or without them (the default): then a
    /// parallel projector also drops the translation.
    /// </summary>
    internal void Scaled(bool On = false)
    {
        myType = -1;
        myTrsf = myScaledTrsf;
        if (!On)
        {
            myTrsf.SetScaleFactor(1.0);
            if (!myPersp)
            {
                myTrsf.SetTranslationPart(new gp_Vec(0.0, 0.0, 0.0));
                myType = TrsfType(myTrsf);
            }
        }
        myInvTrsf = myTrsf;
        myInvTrsf.Invert();
    }

    /// <summary>A copy computing with the scale and translation (<paramref name="On"/>) or without (<see cref="Scaled"/>'s twin).</summary>
    public readonly HLRAlgo_Projector WithScaled(bool On = false)
    {
        HLRAlgo_Projector aCopy = this;
        aCopy.Scaled(On);
        return aCopy;
    }

    /// <summary>A copy set to <paramref name="T"/>, <paramref name="Persp"/> and <paramref name="Focus"/> (<see cref="Set"/>'s twin).</summary>
    public readonly HLRAlgo_Projector WithSet(in gp_Trsf T, bool Persp, double Focus)
    {
        HLRAlgo_Projector aCopy = this;
        aCopy.Set(T, Persp, Focus);
        return aCopy;
    }

    // Queries

    /// <summary>The min-max directions.</summary>
    public readonly void Directions(out gp_Vec2d D1, out gp_Vec2d D2, out gp_Vec2d D3)
    {
        D1 = myD1;
        D2 = myD2;
        D3 = myD3;
    }

    /// <summary>True if there is a perspective transformation.</summary>
    public readonly bool Perspective() => myPersp;

    /// <summary>The active transformation.</summary>
    public readonly gp_Trsf Transformation() => myTrsf;

    /// <summary>The active inverted transformation.</summary>
    public readonly gp_Trsf InvertedTransformation() => myInvTrsf;

    /// <summary>The original transformation.</summary>
    public readonly gp_Trsf FullTransformation() => myScaledTrsf;

    /// <summary>The focal length; throws Standard_NoSuchObject on a parallel projector.</summary>
    public readonly double Focus()
    {
        Standard_NoSuchObject_Raise_if(!myPersp, "HLRAlgo_Projector::Not a Perspective");
        return myFocus;
    }

    /// <summary>Transforms <paramref name="D"/> by the active transformation.</summary>
    public readonly void Transform(ref gp_Vec D)
    {
        D.Transform(myTrsf);
    }

    /// <summary>Transforms <paramref name="Pnt"/> by the active transformation.</summary>
    public readonly void Transform(ref gp_Pnt Pnt)
    {
        Pnt.Transform(myTrsf);
    }

    /// <summary>
    /// Transforms <paramref name="P"/> and applies the perspective if needed. For OCCT's first standard axonometric
    /// matrix the result is not written (an OCCT slip): <paramref name="Pout"/> keeps the caller's value.
    /// </summary>
    public readonly void Project(in gp_Pnt P, ref gp_Pnt2d Pout)
    {
        if (myType != -1)
        {
            double X, Y;
            switch (myType)
            {
                case 0:
                { //-- axono standard
                    double x07 = P.X() * 0.7071067811865475;
                    double y05 = P.Y() * 0.5;
                    double z05 = P.Z() * 0.5;
                    X = x07 - y05 + z05;
                    Y = x07 + y05 - z05;
                    //-- Z=0.7071067811865475*(P.Y()+P.Z());
                    // OCCT: Pout is not set here (decision 10104)
                    break;
                }
                case 1:
                { //-- top
                    X = P.X();
                    Y = P.Y(); //-- Z=P.Z();
                    Pout.SetCoord(X, Y);
                    break;
                }
                case 2:
                {
                    X = P.X();
                    Y = P.Z(); //-- Z=-P.Y();
                    Pout.SetCoord(X, Y);
                    break;
                }
                case 3:
                {
                    double xmy05 = (P.X() - P.Y()) * 0.5;
                    double z07 = P.Z() * 0.7071067811865476;
                    X = 0.7071067811865476 * (P.X() + P.Y());
                    Y = -xmy05 + z07;
                    Pout.SetCoord(X, Y);
                    //-- Z= xmy05+z07;
                    break;
                }
                default:
                {
                    gp_Pnt P2 = P;
                    Transform(ref P2);
                    if (myPersp)
                    {
                        double R = 1.0 - P2.Z() / myFocus;
                        Pout.SetCoord(P2.X() / R, P2.Y() / R);
                    }
                    else
                    {
                        Pout.SetCoord(P2.X(), P2.Y());
                    }
                    break;
                }
            }
        }
        else
        {
            gp_Pnt P2 = P;
            Transform(ref P2);
            if (myPersp)
            {
                double R = 1.0 - P2.Z() / myFocus;
                Pout.SetCoord(P2.X() / R, P2.Y() / R);
            }
            else
            {
                Pout.SetCoord(P2.X(), P2.Y());
            }
        }
    }

    /// <summary>Transforms <paramref name="P"/> and applies the perspective if needed (X and Y; Z is the depth).</summary>
    public readonly void Project(in gp_Pnt P, out double X, out double Y, out double Z)
    {
        if (myType != -1)
        {
            switch (myType)
            {
                case 0:
                { //-- axono standard
                    double x07 = P.X() * 0.7071067811865475;
                    double y05 = P.Y() * 0.5;
                    double z05 = P.Z() * 0.5;
                    X = x07 - y05 + z05;
                    Y = x07 + y05 - z05;
                    Z = 0.7071067811865475 * (P.Y() + P.Z());
                    break;
                }
                case 1:
                { //-- top
                    X = P.X();
                    Y = P.Y();
                    Z = P.Z();
                    break;
                }
                case 2:
                {
                    X = P.X();
                    Y = P.Z();
                    Z = -P.Y();
                    break;
                }
                case 3:
                {
                    double xmy05 = (P.X() - P.Y()) * 0.5;
                    double z07 = P.Z() * 0.7071067811865476;
                    X = 0.7071067811865476 * (P.X() + P.Y());
                    Y = -xmy05 + z07;
                    Z = xmy05 + z07;
                    break;
                }
                default:
                {
                    gp_Pnt P2 = P;
                    Transform(ref P2);
                    P2.Coord(out X, out Y, out Z);
                    break;
                }
            }
        }
        else
        {
            gp_Pnt P2 = P;
            Transform(ref P2);
            P2.Coord(out X, out Y, out Z);
            if (myPersp)
            {
                double R = 1 - Z / myFocus;
                X = X / R;
                Y = Y / R;
            }
        }
    }

    /// <summary>Transforms the point and its derivative and applies the perspective if needed.</summary>
    public readonly void Project(in gp_Pnt P, in gp_Vec D1, out gp_Pnt2d Pout, out gp_Vec2d D1out)
    {
        Pout = new gp_Pnt2d();
        D1out = new gp_Vec2d();
        gp_Pnt PP = P;
        PP.Transform(myTrsf);
        gp_Vec DD1 = D1;
        DD1.Transform(myTrsf);
        if (myPersp)
        {
            double R = 1.0 - PP.Z() / myFocus;
            Pout.SetCoord(PP.X() / R, PP.Y() / R);
            D1out.SetCoord(DD1.X() / R + PP.X() * DD1.Z() / (myFocus * R * R),
                           DD1.Y() / R + PP.Y() * DD1.Z() / (myFocus * R * R));
        }
        else
        {
            Pout.SetCoord(PP.X(), PP.Y());
            D1out.SetCoord(DD1.X(), DD1.Y());
        }
    }

    /// <summary>The line through the eye towards the 2D point (<paramref name="X"/>, <paramref name="Y"/>).</summary>
    public readonly gp_Lin Shoot(double X, double Y)
    {
        gp_Lin L;
        if (myPersp)
        {
            L = new gp_Lin(new gp_Pnt(0, 0, myFocus), new gp_Dir(X, Y, -myFocus));
        }
        else
        {
            L = new gp_Lin(new gp_Pnt(X, Y, 0), new gp_Dir(gp_Dir.D.NZ));
        }
        L.Transform(myInvTrsf);
        return L;
    }

    private void SetDirection()
    {
        gp_Vec V1 = new(1, 0, 0);
        V1.Transform(myTrsf);
        if ((Math.Abs(V1.X()) + Math.Abs(V1.Y())) < Precision.Angular())
        {
            V1.SetCoord(1, 1, 0);
        }
        gp_Vec2d D1 = new(V1.X(), V1.Y());
        myD1.SetCoord(-D1.Y(), D1.X());
        gp_Vec V2 = new(0, 1, 0);
        V2.Transform(myTrsf);
        if ((Math.Abs(V2.X()) + Math.Abs(V2.Y())) < Precision.Angular())
        {
            V2.SetCoord(1, 1, 0);
        }
        gp_Vec2d D2 = new(V2.X(), V2.Y());
        myD2.SetCoord(-D2.Y(), D2.X());
        gp_Vec V3 = new(0, 0, 1);
        V3.Transform(myTrsf);
        if ((Math.Abs(V3.X()) + Math.Abs(V3.Y())) < Precision.Angular())
        {
            V3.SetCoord(1, 1, 0);
        }
        gp_Vec2d D3 = new(V3.X(), V3.Y());
        myD3.SetCoord(-D3.Y(), D3.X());
    }

    public readonly bool Equals(HLRAlgo_Projector other)
        => myType == other.myType && myPersp == other.myPersp && myFocus.Equals(other.myFocus)
           && myScaledTrsf.Equals(other.myScaledTrsf) && myTrsf.Equals(other.myTrsf)
           && myInvTrsf.Equals(other.myInvTrsf) && myD1.Equals(other.myD1) && myD2.Equals(other.myD2)
           && myD3.Equals(other.myD3);

    public override readonly bool Equals(object? obj) => obj is HLRAlgo_Projector other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myType, myPersp, myFocus, myScaledTrsf);

    public static bool operator ==(in HLRAlgo_Projector theLeft, in HLRAlgo_Projector theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in HLRAlgo_Projector theLeft, in HLRAlgo_Projector theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{gp.Format("HLRAlgo_Projector", myFocus)} persp {myPersp} {myScaledTrsf}";
}
