// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

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
/// <see cref="Project(in Point3D, ref Point2D)"/> OCCT forgets to write the result for the first of them, so the
/// point is <c>ref</c> and keeps the caller's value there (decision 1078).
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
    private Transform3D myScaledTrsf;
    private Transform3D myTrsf;
    private Transform3D myInvTrsf;
    private Vector2D myD1;
    private Vector2D myD2;
    private Vector2D myD3;

    /// <summary>The identity, parallel projector.</summary>
    public HLRAlgo_Projector()
    {
        myType = -1;
        myPersp = false;
        myFocus = 0;
        myScaledTrsf = new Transform3D();
        myTrsf = new Transform3D();
        myInvTrsf = new Transform3D();
        myD1 = new Vector2D();
        myD2 = new Vector2D();
        myD3 = new Vector2D();
        Scaled();
    }

    /// <summary>An axonometric (parallel) projector; <paramref name="CS"/> is the viewing coordinate system.</summary>
    public HLRAlgo_Projector(in Frame3D CS)
        : this()
    {
        myPersp = false;
        myFocus = 0;
        myScaledTrsf.SetTransformation(new CoordinateSystem(CS));
        Scaled();
        SetDirection();
    }

    /// <summary>A perspective projector of focal length <paramref name="Focus"/>; <paramref name="CS"/> is the viewing coordinate system.</summary>
    public HLRAlgo_Projector(in Frame3D CS, double Focus)
        : this()
    {
        myPersp = true;
        myFocus = Focus;
        myScaledTrsf.SetTransformation(new CoordinateSystem(CS));
        Scaled();
        SetDirection();
    }

    /// <summary>A projector with automatic min-max directions.</summary>
    public HLRAlgo_Projector(in Transform3D T, bool Persp, double Focus)
        : this()
    {
        myPersp = Persp;
        myFocus = Focus;
        myScaledTrsf = T;
        Scaled();
        SetDirection();
    }

    /// <summary>A projector with the given min-max directions.</summary>
    public HLRAlgo_Projector(in Transform3D T, bool Persp, double Focus, in Vector2D v1, in Vector2D v2, in Vector2D v3)
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

    internal void Set(in Transform3D T, bool Persp, double Focus)
    {
        myPersp = Persp;
        myFocus = Focus;
        myScaledTrsf = T;
        Scaled();
        SetDirection();
    }

    private static int TrsfType(in Transform3D Trsf)
    {
        Matrix3x3 Mat = Trsf.VectorialPart();
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
                myTrsf.SetTranslationPart(new Vector3D(0.0, 0.0, 0.0));
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
    public readonly HLRAlgo_Projector WithSet(in Transform3D T, bool Persp, double Focus)
    {
        HLRAlgo_Projector aCopy = this;
        aCopy.Set(T, Persp, Focus);
        return aCopy;
    }

    // Queries

    /// <summary>The min-max directions.</summary>
    public readonly void Directions(out Vector2D D1, out Vector2D D2, out Vector2D D3)
    {
        D1 = myD1;
        D2 = myD2;
        D3 = myD3;
    }

    /// <summary>True if there is a perspective transformation.</summary>
    public readonly bool Perspective() => myPersp;

    /// <summary>The active transformation.</summary>
    public readonly Transform3D Transformation() => myTrsf;

    /// <summary>The active inverted transformation.</summary>
    public readonly Transform3D InvertedTransformation() => myInvTrsf;

    /// <summary>The original transformation.</summary>
    public readonly Transform3D FullTransformation() => myScaledTrsf;

    /// <summary>The focal length; throws Standard_NoSuchObject on a parallel projector.</summary>
    public readonly double Focus()
    {
        Standard_NoSuchObject_Raise_if(!myPersp, "HLRAlgo_Projector::Not a Perspective");
        return myFocus;
    }

    /// <summary>Transforms <paramref name="D"/> by the active transformation.</summary>
    public readonly void Transform(ref Vector3D D)
    {
        D.Transform(myTrsf);
    }

    /// <summary>Transforms <paramref name="Pnt"/> by the active transformation.</summary>
    public readonly void Transform(ref Point3D Pnt)
    {
        Pnt.Transform(myTrsf);
    }

    /// <summary>
    /// Transforms <paramref name="P"/> and applies the perspective if needed. For OCCT's first standard axonometric
    /// matrix the result is not written (an OCCT slip): <paramref name="Pout"/> keeps the caller's value.
    /// </summary>
    public readonly void Project(in Point3D P, ref Point2D Pout)
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
                    // OCCT: Pout is not set here (decision 1078)
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
                    Point3D P2 = P;
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
            Point3D P2 = P;
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
    public readonly void Project(in Point3D P, out double X, out double Y, out double Z)
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
                    Point3D P2 = P;
                    Transform(ref P2);
                    P2.Coord(out X, out Y, out Z);
                    break;
                }
            }
        }
        else
        {
            Point3D P2 = P;
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
    public readonly void Project(in Point3D P, in Vector3D D1, out Point2D Pout, out Vector2D D1out)
    {
        Pout = new Point2D();
        D1out = new Vector2D();
        Point3D PP = P;
        PP.Transform(myTrsf);
        Vector3D DD1 = D1;
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
    public readonly Line3D Shoot(double X, double Y)
    {
        Line3D L;
        if (myPersp)
        {
            L = new Line3D(new Point3D(0, 0, myFocus), new Direction3D(X, Y, -myFocus));
        }
        else
        {
            L = new Line3D(new Point3D(X, Y, 0), new Direction3D(Direction3D.D.NZ));
        }
        L.Transform(myInvTrsf);
        return L;
    }

    private void SetDirection()
    {
        Vector3D V1 = new(1, 0, 0);
        V1.Transform(myTrsf);
        if ((Math.Abs(V1.X()) + Math.Abs(V1.Y())) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            V1.SetCoord(1, 1, 0);
        }
        Vector2D D1 = new(V1.X(), V1.Y());
        myD1.SetCoord(-D1.Y(), D1.X());
        Vector3D V2 = new(0, 1, 0);
        V2.Transform(myTrsf);
        if ((Math.Abs(V2.X()) + Math.Abs(V2.Y())) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            V2.SetCoord(1, 1, 0);
        }
        Vector2D D2 = new(V2.X(), V2.Y());
        myD2.SetCoord(-D2.Y(), D2.X());
        Vector3D V3 = new(0, 0, 1);
        V3.Transform(myTrsf);
        if ((Math.Abs(V3.X()) + Math.Abs(V3.Y())) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            V3.SetCoord(1, 1, 0);
        }
        Vector2D D3 = new(V3.X(), V3.Y());
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
        => $"{Constants.Format("HLRAlgo_Projector", myFocus)} persp {myPersp} {myScaledTrsf}";
}
