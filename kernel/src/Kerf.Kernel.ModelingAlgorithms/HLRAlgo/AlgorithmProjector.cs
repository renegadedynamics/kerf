// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Projector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Engine.HiddenLines.Algorithm;

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
internal partial struct AlgorithmProjector : IEquatable<AlgorithmProjector>
{
    // formula for derivating a perspective, from Mathematica
    //        X'[t]      X[t] Z'[t]
    // D1 =  -------- + -------------
    //           Z[t]          Z[t] 2
    //       1 - ----   f (1 - ----)
    //            f             f

    private int _type;
    private bool _persp;
    private double _focus;
    private Transform3D _scaledTrsf;
    private Transform3D _trsf;
    private Transform3D _invTrsf;
    private Vector2D _d1;
    private Vector2D _d2;
    private Vector2D _d3;

    /// <summary>The identity, parallel projector.</summary>
    public AlgorithmProjector()
    {
        _type = -1;
        _persp = false;
        _focus = 0;
        _scaledTrsf = new Transform3D();
        _trsf = new Transform3D();
        _invTrsf = new Transform3D();
        _d1 = new Vector2D();
        _d2 = new Vector2D();
        _d3 = new Vector2D();
        Scaled();
    }

    /// <summary>An axonometric (parallel) projector; <paramref name="cs"/> is the viewing coordinate system.</summary>
    public AlgorithmProjector(in Frame3D cs)
        : this()
    {
        _persp = false;
        _focus = 0;
        _scaledTrsf = _scaledTrsf.WithTransformation(new CoordinateSystem(cs));
        Scaled();
        SetDirection();
    }

    /// <summary>A perspective projector of focal length <paramref name="focus"/>; <paramref name="cs"/> is the viewing coordinate system.</summary>
    public AlgorithmProjector(in Frame3D cs, double focus)
        : this()
    {
        _persp = true;
        _focus = focus;
        _scaledTrsf = _scaledTrsf.WithTransformation(new CoordinateSystem(cs));
        Scaled();
        SetDirection();
    }

    /// <summary>A projector with automatic min-max directions.</summary>
    public AlgorithmProjector(in Transform3D t, bool persp, double focus)
        : this()
    {
        _persp = persp;
        _focus = focus;
        _scaledTrsf = t;
        Scaled();
        SetDirection();
    }

    /// <summary>A projector with the given min-max directions.</summary>
    public AlgorithmProjector(in Transform3D t, bool persp, double focus, in Vector2D v1, in Vector2D v2, in Vector2D v3)
        : this()
    {
        _persp = persp;
        _focus = focus;
        _scaledTrsf = t;
        _d1 = v1;
        _d2 = v2;
        _d3 = v3;
        Scaled();
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Set(in Transform3D t, bool persp, double focus)
    {
        _persp = persp;
        _focus = focus;
        _scaledTrsf = t;
        Scaled();
        SetDirection();
    }

    private static int TrsfType(in Transform3D trsf)
    {
        Matrix3x3 mat = trsf.VectorialPart;
        if ((Math.Abs(mat.Value(1, 1) - 1.0) < 1e-15) && (Math.Abs(mat.Value(2, 2) - 1.0) < 1e-15)
            && (Math.Abs(mat.Value(3, 3) - 1.0) < 1e-15))
        {
            return (1); //-- top
        }
        else if ((Math.Abs(mat.Value(1, 1) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(mat.Value(1, 2) + 0.5) < 1e-15) && (Math.Abs(mat.Value(1, 3) - 0.5) < 1e-15)

                 && (Math.Abs(mat.Value(2, 1) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(mat.Value(2, 2) - 0.5) < 1e-15) && (Math.Abs(mat.Value(2, 3) + 0.5) < 1e-15)

                 && (Math.Abs(mat.Value(3, 1)) < 1e-15)
                 && (Math.Abs(mat.Value(3, 2) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(mat.Value(3, 3) - 0.7071067811865476) < 1e-15))
        {
            return (0); //--
        }
        else if ((Math.Abs(mat.Value(1, 1) - 1.0) < 1e-15) && (Math.Abs(mat.Value(2, 3) - 1.0) < 1e-15)
                 && (Math.Abs(mat.Value(3, 2) + 1.0) < 1e-15))
        {
            return (2); //-- front
        }
        else if ((Math.Abs(mat.Value(1, 1) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(mat.Value(1, 2) - 0.7071067811865476) < 1e-15)
                 && (Math.Abs(mat.Value(1, 3)) < 1e-15)

                 && (Math.Abs(mat.Value(2, 1) + 0.5) < 1e-15) && (Math.Abs(mat.Value(2, 2) - 0.5) < 1e-15)
                 && (Math.Abs(mat.Value(2, 3) - 0.7071067811865476) < 1e-15)

                 && (Math.Abs(mat.Value(3, 1) - 0.5) < 1e-15) && (Math.Abs(mat.Value(3, 2) + 0.5) < 1e-15)
                 && (Math.Abs(mat.Value(3, 3) - 0.7071067811865476) < 1e-15))
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
        _type = -1;
        _trsf = _scaledTrsf;
        if (!On)
        {
            _trsf = _trsf.WithScaleFactor(1.0);
            if (!_persp)
            {
                _trsf = _trsf.WithTranslationPart(new Vector3D(0.0, 0.0, 0.0));
                _type = TrsfType(_trsf);
            }
        }
        _invTrsf = _trsf;
        _invTrsf = _invTrsf.Inverted();
    }

    /// <summary>A copy computing with the scale and translation (<paramref name="On"/>) or without (<see cref="Scaled"/>'s twin).</summary>
    public readonly AlgorithmProjector WithScaled(bool On = false)
    {
        AlgorithmProjector copy = this;
        copy.Scaled(On);
        return copy;
    }

    /// <summary>A copy set to <paramref name="t"/>, <paramref name="persp"/> and <paramref name="focus"/> (<see cref="Set"/>'s twin).</summary>
    public readonly AlgorithmProjector WithSet(in Transform3D t, bool persp, double focus)
    {
        AlgorithmProjector copy = this;
        copy.Set(t, persp, focus);
        return copy;
    }

    // Queries

    /// <summary>The min-max directions.</summary>
    public readonly void Directions(out Vector2D d1, out Vector2D d2, out Vector2D d3)
    {
        d1 = _d1;
        d2 = _d2;
        d3 = _d3;
    }

    /// <summary>True if there is a perspective transformation.</summary>
    public readonly bool Perspective() => _persp;

    /// <summary>The active transformation.</summary>
    public readonly Transform3D Transformation() => _trsf;

    /// <summary>The active inverted transformation.</summary>
    public readonly Transform3D InvertedTransformation() => _invTrsf;

    /// <summary>The original transformation.</summary>
    public readonly Transform3D FullTransformation() => _scaledTrsf;

    /// <summary>The focal length; throws Standard_NoSuchObject on a parallel projector.</summary>
    public readonly double Focus()
    {
        Standard_NoSuchObject_Raise_if(!_persp, "HLRAlgo_Projector::Not a Perspective");
        return _focus;
    }

    /// <summary>Transforms <paramref name="d"/> by the active transformation.</summary>
    public readonly void Transform(ref Vector3D d)
    {
        d = d.Transformed(_trsf);
    }

    /// <summary>Transforms <paramref name="pnt"/> by the active transformation.</summary>
    public readonly void Transform(ref Point3D pnt)
    {
        pnt = pnt.Transformed(_trsf);
    }

    /// <summary>
    /// Transforms <paramref name="p"/> and applies the perspective if needed. For OCCT's first standard axonometric
    /// matrix the result is not written (an OCCT slip): <paramref name="pout"/> keeps the caller's value.
    /// </summary>
    public readonly void Project(in Point3D p, ref Point2D pout)
    {
        if (_type != -1)
        {
            double x, y;
            switch (_type)
            {
                case 0:
                { //-- axono standard
                    double x07 = p.X * 0.7071067811865475;
                    double y05 = p.Y * 0.5;
                    double z05 = p.Z * 0.5;
                    x = x07 - y05 + z05;
                    y = x07 + y05 - z05;
                    //-- Z=0.7071067811865475*(P.Y()+P.Z());
                    // OCCT: Pout is not set here (decision 1078)
                    break;
                }
                case 1:
                { //-- top
                    x = p.X;
                    y = p.Y; //-- Z=P.Z();
                    pout = pout.WithCoord(x, y);
                    break;
                }
                case 2:
                {
                    x = p.X;
                    y = p.Z; //-- Z=-P.Y();
                    pout = pout.WithCoord(x, y);
                    break;
                }
                case 3:
                {
                    double xmy05 = (p.X - p.Y) * 0.5;
                    double z07 = p.Z * 0.7071067811865476;
                    x = 0.7071067811865476 * (p.X + p.Y);
                    y = -xmy05 + z07;
                    pout = pout.WithCoord(x, y);
                    //-- Z= xmy05+z07;
                    break;
                }
                default:
                {
                    Point3D p2 = p;
                    Transform(ref p2);
                    if (_persp)
                    {
                        double r = 1.0 - p2.Z / _focus;
                        pout = pout.WithCoord(p2.X / r, p2.Y / r);
                    }
                    else
                    {
                        pout = pout.WithCoord(p2.X, p2.Y);
                    }
                    break;
                }
            }
        }
        else
        {
            Point3D p2 = p;
            Transform(ref p2);
            if (_persp)
            {
                double r = 1.0 - p2.Z / _focus;
                pout = pout.WithCoord(p2.X / r, p2.Y / r);
            }
            else
            {
                pout = pout.WithCoord(p2.X, p2.Y);
            }
        }
    }

    /// <summary>Transforms <paramref name="p"/> and applies the perspective if needed (X and Y; Z is the depth).</summary>
    public readonly void Project(in Point3D p, out double x, out double y, out double z)
    {
        if (_type != -1)
        {
            switch (_type)
            {
                case 0:
                { //-- axono standard
                    double x07 = p.X * 0.7071067811865475;
                    double y05 = p.Y * 0.5;
                    double z05 = p.Z * 0.5;
                    x = x07 - y05 + z05;
                    y = x07 + y05 - z05;
                    z = 0.7071067811865475 * (p.Y + p.Z);
                    break;
                }
                case 1:
                { //-- top
                    x = p.X;
                    y = p.Y;
                    z = p.Z;
                    break;
                }
                case 2:
                {
                    x = p.X;
                    y = p.Z;
                    z = -p.Y;
                    break;
                }
                case 3:
                {
                    double xmy05 = (p.X - p.Y) * 0.5;
                    double z07 = p.Z * 0.7071067811865476;
                    x = 0.7071067811865476 * (p.X + p.Y);
                    y = -xmy05 + z07;
                    z = xmy05 + z07;
                    break;
                }
                default:
                {
                    Point3D p2 = p;
                    Transform(ref p2);
                    p2.Coord(out x, out y, out z);
                    break;
                }
            }
        }
        else
        {
            Point3D p2 = p;
            Transform(ref p2);
            p2.Coord(out x, out y, out z);
            if (_persp)
            {
                double r = 1 - z / _focus;
                x = x / r;
                y = y / r;
            }
        }
    }

    /// <summary>Transforms the point and its derivative and applies the perspective if needed.</summary>
    public readonly void Project(in Point3D p, in Vector3D d1, out Point2D pout, out Vector2D d1out)
    {
        pout = new Point2D();
        d1out = new Vector2D();
        Point3D pp = p;
        pp = pp.Transformed(_trsf);
        Vector3D dd1 = d1;
        dd1 = dd1.Transformed(_trsf);
        if (_persp)
        {
            double r = 1.0 - pp.Z / _focus;
            pout = pout.WithCoord(pp.X / r, pp.Y / r);
            d1out = d1out.WithCoord(dd1.X / r + pp.X * dd1.Z / (_focus * r * r),
                           dd1.Y / r + pp.Y * dd1.Z / (_focus * r * r));
        }
        else
        {
            pout = pout.WithCoord(pp.X, pp.Y);
            d1out = d1out.WithCoord(dd1.X, dd1.Y);
        }
    }

    /// <summary>The line through the eye towards the 2D point (<paramref name="x"/>, <paramref name="y"/>).</summary>
    public readonly Line3D Shoot(double x, double y)
    {
        Line3D l;
        if (_persp)
        {
            l = new Line3D(new Point3D(0, 0, _focus), new Direction3D(x, y, -_focus));
        }
        else
        {
            l = new Line3D(new Point3D(x, y, 0), new Direction3D(Direction3D.D.NZ));
        }
        l = l.Transformed(_invTrsf);
        return l;
    }

    private void SetDirection()
    {
        Vector3D v1 = new(1, 0, 0);
        v1 = v1.Transformed(_trsf);
        if ((Math.Abs(v1.X) + Math.Abs(v1.Y)) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            v1 = v1.WithCoord(1, 1, 0);
        }
        Vector2D d1 = new(v1.X, v1.Y);
        _d1 = _d1.WithCoord(-d1.Y, d1.X);
        Vector3D v2 = new(0, 1, 0);
        v2 = v2.Transformed(_trsf);
        if ((Math.Abs(v2.X) + Math.Abs(v2.Y)) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            v2 = v2.WithCoord(1, 1, 0);
        }
        Vector2D d2 = new(v2.X, v2.Y);
        _d2 = _d2.WithCoord(-d2.Y, d2.X);
        Vector3D v3 = new(0, 0, 1);
        v3 = v3.Transformed(_trsf);
        if ((Math.Abs(v3.X) + Math.Abs(v3.Y)) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            v3 = v3.WithCoord(1, 1, 0);
        }
        Vector2D d3 = new(v3.X, v3.Y);
        _d3 = _d3.WithCoord(-d3.Y, d3.X);
    }

    public readonly bool Equals(AlgorithmProjector other)
        => _type == other._type && _persp == other._persp && _focus.Equals(other._focus)
           && _scaledTrsf.Equals(other._scaledTrsf) && _trsf.Equals(other._trsf)
           && _invTrsf.Equals(other._invTrsf) && _d1.Equals(other._d1) && _d2.Equals(other._d2)
           && _d3.Equals(other._d3);

    public override readonly bool Equals(object? obj) => obj is AlgorithmProjector other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_type, _persp, _focus, _scaledTrsf);

    public static bool operator ==(in AlgorithmProjector left, in AlgorithmProjector right)
        => left.Equals(right);

    public static bool operator !=(in AlgorithmProjector left, in AlgorithmProjector right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("HLRAlgo_Projector", _focus)} persp {_persp} {_scaledTrsf}";
}
