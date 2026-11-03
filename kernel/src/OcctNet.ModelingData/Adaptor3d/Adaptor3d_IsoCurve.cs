// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_IsoCurve.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_IsoCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// An isoparametric curve on a surface: the curve of constant U (IsoU, parametrized by V) or of constant V (IsoV,
/// parametrized by U) of an <see cref="Adaptor3d_Surface"/>, restricted to a parameter range.
/// </summary>
/// <remarks>An adaptor is a working object: <see cref="Load(Adaptor3d_Surface)"/> is public, as in OCCT.</remarks>
public class Adaptor3d_IsoCurve : Adaptor3d_Curve
{
    private Adaptor3d_Surface? mySurface;
    private IsoType myIso;
    private double myFirst;
    private double myLast;
    private double myParameter;

    /// <summary>An empty adaptor: no surface, no iso.</summary>
    public Adaptor3d_IsoCurve()
    {
        myIso = NoneIso;
        myFirst = 0.0;
        myLast = 0.0;
        myParameter = 0.0;
    }

    /// <summary>An adaptor on the surface S, without an iso: load one before use.</summary>
    public Adaptor3d_IsoCurve(Adaptor3d_Surface S)
    {
        mySurface = S;
        myIso = NoneIso;
        myFirst = 0.0;
        myLast = 0.0;
        myParameter = 0.0;
    }

    /// <summary>The iso of type theIso at theParam on S, over the surface's whole range.</summary>
    /// <exception cref="NoSuchObjectException">theIso is GeomAbs_NoneIso.</exception>
    public Adaptor3d_IsoCurve(Adaptor3d_Surface S, IsoType theIso, double theParam)
    {
        mySurface = S;
        myIso = NoneIso;
        myFirst = 0.0;
        myLast = 0.0;
        myParameter = 0.0;
        Load(theIso, theParam);
    }

    /// <summary>The iso of type theIso at theParam on theS, between theWFirst and theWLast (within the surface's range).</summary>
    public Adaptor3d_IsoCurve(Adaptor3d_Surface theS, IsoType theIso, double theParam, double theWFirst,
                              double theWLast)
    {
        mySurface = theS;
        myIso = theIso;
        myFirst = theWFirst;
        myLast = theWLast;
        myParameter = theParam;
        Load(theIso, theParam, theWFirst, theWLast);
    }

    // OCCT's implicit copy constructor: shares the surface.
    private Adaptor3d_IsoCurve(Adaptor3d_IsoCurve theOther)
    {
        mySurface = theOther.mySurface;
        myIso = theOther.myIso;
        myFirst = theOther.myFirst;
        myLast = theOther.myLast;
        myParameter = theOther.myParameter;
    }

    /// <summary>A copy with a shallow copy of the surface adaptor.</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        Adaptor3d_IsoCurve aCopy = new Adaptor3d_IsoCurve();

        if (mySurface is not null)
        {
            aCopy.mySurface = mySurface.ShallowCopy();
        }
        aCopy.myIso = myIso;
        aCopy.myFirst = myFirst;
        aCopy.myLast = myLast;
        aCopy.myParameter = myParameter;

        return aCopy;
    }

    /// <summary>Changes the surface; the iso is reset to GeomAbs_NoneIso.</summary>
    public void Load(Adaptor3d_Surface S)
    {
        mySurface = S;
        myIso = NoneIso;
    }

    /// <summary>Changes the iso, over the surface's whole range in the other direction.</summary>
    /// <exception cref="NoSuchObjectException">Iso is GeomAbs_NoneIso.</exception>
    public void Load(IsoType Iso, double Param)
    {
        switch (Iso)
        {
            case IsoU:
                Load(Iso, Param, mySurface!.FirstVParameter(), mySurface.LastVParameter());
                break;

            case IsoV:
                Load(Iso, Param, mySurface!.FirstUParameter(), mySurface.LastUParameter());
                break;

            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
    }

    /// <summary>
    /// Changes the iso and its range, which is cut to the surface's; on a periodic surface the parameters are
    /// brought into the first period.
    /// </summary>
    public void Load(IsoType Iso, double Param, double WFirst, double WLast)
    {
        myIso = Iso;
        myParameter = Param;
        myFirst = WFirst;
        myLast = WLast;

        if (myIso == IsoU)
        {
            myFirst = Max(myFirst, mySurface!.FirstVParameter());
            myLast = Min(myLast, mySurface.LastVParameter());
        }
        else
        {
            myFirst = Max(myFirst, mySurface!.FirstUParameter());
            myLast = Min(myLast, mySurface.LastUParameter());
        }

        // Adjust the parameters on periodic surfaces

        double dummy = myParameter;

        if (mySurface.IsUPeriodic())
        {

            if (myIso == IsoU)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(mySurface.FirstUParameter(),
                                      mySurface.FirstUParameter() + mySurface.UPeriod(),
                                      mySurface.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref myParameter,
                                      ref dummy);
            }
            else
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(mySurface.FirstUParameter(),
                                      mySurface.FirstUParameter() + mySurface.UPeriod(),
                                      mySurface.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref myFirst,
                                      ref myLast);
            }
        }

        if (mySurface.IsVPeriodic())
        {

            if (myIso == IsoV)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(mySurface.FirstVParameter(),
                                      mySurface.FirstVParameter() + mySurface.VPeriod(),
                                      mySurface.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref myParameter,
                                      ref dummy);
            }
            else
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(mySurface.FirstVParameter(),
                                      mySurface.FirstVParameter() + mySurface.VPeriod(),
                                      mySurface.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref myFirst,
                                      ref myLast);
            }
        }
    }

    /// <summary>The surface (null if none is loaded).</summary>
    public Adaptor3d_Surface? Surface() => mySurface;

    /// <summary>The type of iso.</summary>
    public IsoType Iso() => myIso;

    /// <summary>The constant parameter of the iso.</summary>
    public double Parameter() => myParameter;

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    public override Continuity Continuity()
    {
        switch (myIso)
        {
            case IsoU:
                return mySurface!.VContinuity();
            case IsoV:
                return mySurface!.UContinuity();
            case NoneIso:
            default:
                break;
        }

        throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
    }

    public override int NbIntervals(Continuity S)
    {
        if (myIso == NoneIso)
        {
            throw new NoSuchObjectException();
        }

        bool UIso = (myIso == IsoU);

        int nbInter = UIso ? mySurface!.NbVIntervals(S) : mySurface!.NbUIntervals(S);

        NCollection_Array1<double> T = new NCollection_Array1<double>(1, nbInter + 1);

        if (UIso)
        {
            mySurface.VIntervals(T, S);
        }
        else
        {
            mySurface.UIntervals(T, S);
        }

        if (nbInter == 1)
        {
            return nbInter;
        }

        int first = 1;
        while (T[first] <= myFirst)
        {
            first++;
        }
        int last = nbInter + 1;
        while (T[last] >= myLast)
        {
            last--;
        }
        return (last - first + 2);
    }

    public override void Intervals(NCollection_Array1<double> TI, Continuity S)
    {
        if (myIso == NoneIso)
        {
            throw new NoSuchObjectException();
        }

        bool UIso = (myIso == IsoU);

        int nbInter = UIso ? mySurface!.NbVIntervals(S) : mySurface!.NbUIntervals(S);

        NCollection_Array1<double> T = new NCollection_Array1<double>(1, nbInter + 1);

        if (UIso)
        {
            mySurface.VIntervals(T, S);
        }
        else
        {
            mySurface.UIntervals(T, S);
        }

        if (nbInter == 1)
        {
            TI[TI.Lower()] = myFirst;
            TI[TI.Lower() + 1] = myLast;
            return;
        }

        int first = 1;
        while (T[first] <= myFirst)
        {
            first++;
        }
        int last = nbInter + 1;
        while (T[last] >= myLast)
        {
            last--;
        }

        int i = TI.Lower(), j;
        for (j = first - 1; j <= last + 1; j++)
        {
            TI[i] = T[j];
            i++;
        }
        TI[TI.Lower()] = myFirst;
        TI[TI.Lower() + last - first + 2] = myLast;
    }

    /// <summary>A copy (sharing the surface) restricted to [First, Last]; Tol is not used.</summary>
    public override Adaptor3d_Curve Trim(double First, double Last, double Tol)
    {
        _ = Tol;
        Adaptor3d_IsoCurve HI = new Adaptor3d_IsoCurve(this);
        HI.Load(myIso, myParameter, First, Last);
        return HI;
    }

    public override bool IsClosed()
    {
        switch (myIso)
        {
            case IsoU:
                return mySurface!.IsVClosed();
            case IsoV:
                return mySurface!.IsUClosed();
            case NoneIso:
            default:
                break;
        }

        throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
    }

    public override bool IsPeriodic()
    {
        switch (myIso)
        {
            case IsoU:
                return mySurface!.IsVPeriodic();
            case IsoV:
                return mySurface!.IsUPeriodic();
            case NoneIso:
            default:
                break;
        }

        throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
    }

    public override double Period()
    {
        switch (myIso)
        {
            case IsoU:
                return mySurface!.VPeriod();
            case IsoV:
                return mySurface!.UPeriod();
            case NoneIso:
            default:
                break;
        }

        throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
    }

    /// <summary>The point of parameter theU.</summary>
    public sealed override Point3D EvalD0(double theU)
    {
        switch (myIso)
        {
            case IsoU:
                return mySurface!.EvalD0(myParameter, theU);
            case IsoV:
                return mySurface!.EvalD0(theU, myParameter);
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Point3D();
    }

    /// <summary>The point and first derivative at theU.</summary>
    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        switch (myIso)
        {
            case IsoU:
                {
                    Geom_Surface.ResD1 aRes = mySurface!.EvalD1(myParameter, theU);
                    return new Geom_Curve.ResD1 { Point = aRes.Point, D1 = aRes.D1V };
                }
            case IsoV:
                {
                    Geom_Surface.ResD1 aRes = mySurface!.EvalD1(theU, myParameter);
                    return new Geom_Curve.ResD1 { Point = aRes.Point, D1 = aRes.D1U };
                }
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Geom_Curve.ResD1();
    }

    /// <summary>The point and first two derivatives at theU.</summary>
    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        switch (myIso)
        {
            case IsoU:
                {
                    Geom_Surface.ResD2 aRes = mySurface!.EvalD2(myParameter, theU);
                    return new Geom_Curve.ResD2 { Point = aRes.Point, D1 = aRes.D1V, D2 = aRes.D2V };
                }
            case IsoV:
                {
                    Geom_Surface.ResD2 aRes = mySurface!.EvalD2(theU, myParameter);
                    return new Geom_Curve.ResD2 { Point = aRes.Point, D1 = aRes.D1U, D2 = aRes.D2U };
                }
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Geom_Curve.ResD2();
    }

    /// <summary>The point and first three derivatives at theU.</summary>
    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        switch (myIso)
        {
            case IsoU:
                {
                    Geom_Surface.ResD3 aRes = mySurface!.EvalD3(myParameter, theU);
                    return new Geom_Curve.ResD3 { Point = aRes.Point, D1 = aRes.D1V, D2 = aRes.D2V, D3 = aRes.D3V };
                }
            case IsoV:
                {
                    Geom_Surface.ResD3 aRes = mySurface!.EvalD3(theU, myParameter);
                    return new Geom_Curve.ResD3 { Point = aRes.Point, D1 = aRes.D1U, D2 = aRes.D2U, D3 = aRes.D3U };
                }
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Geom_Curve.ResD3();
    }

    /// <summary>The derivative of order theN at theU.</summary>
    public sealed override Vector3D EvalDN(double theU, int theN)
    {
        switch (myIso)
        {
            case IsoU:
                return mySurface!.EvalDN(myParameter, theU, 0, theN);
            case IsoV:
                return mySurface!.EvalDN(theU, myParameter, theN, 0);
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Vector3D();
    }

    /// <summary>Precision::Parametric(R3D), whatever the surface.</summary>
    public override double Resolution(double R3D)
    {
        // Peut-on faire mieux ??
        return global::Kerf.Engine.Foundation.Precision.Parametric(R3D);
    }

    public override CurveKind GetType()
    {
        switch (mySurface!.GetType())
        {
            case Plane:
                return global::Kerf.Geometry.CurveKind.Line;

            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            return global::Kerf.Geometry.CurveKind.Line;

                        case IsoV:
                            return global::Kerf.Geometry.CurveKind.Circle;

                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                return global::Kerf.Geometry.CurveKind.Circle;

            case BezierSurface:
                return BezierCurve;

            case BSplineSurface:
                return BSplineCurve;

            case SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            return mySurface.BasisCurve().GetType();

                        case IsoV:
                            return global::Kerf.Geometry.CurveKind.Circle;

                        case NoneIso:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                    break;
                }

            case SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            return global::Kerf.Geometry.CurveKind.Line;

                        case IsoV:
                            return mySurface.BasisCurve().GetType();

                        case NoneIso:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                    break;
                }

            default:
                return OtherCurve;
        }

        // portage WNT
        return OtherCurve;
    }

    /// <summary>The line through the point and tangent at parameter 0.</summary>
    public override Line3D Line()
    {
        Point3D P;
        Vector3D V;
        D1(0, out P, out V);
        return new Line3D(P, new Direction3D(V));
    }

    private static void computeHR(in CoordinateSystem axes, in Point3D P, out double h, out double radius)
    {
        Vector3D V = new Vector3D(axes.Location(), P);
        h = V * new Vector3D(axes.Direction());
        radius = V * new Vector3D(axes.XDirection());
    }

    public override Circle3D Circle()
    {
        CoordinateSystem axes;
        double radius, h = 0.0;

        switch (mySurface!.GetType())
        {
            case SurfaceKind.Cylinder:
                {
                    Cylinder cyl = mySurface.Cylinder();

                    switch (myIso)
                    {

                        case IsoU:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:UIso");
                            }
                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position(), cyl.Radius(), myParameter);
                            }
                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Cone:
                {
                    Cone cone = mySurface.Cone();

                    switch (myIso)
                    {

                        case IsoU:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:UIso");
                            }
                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.ConeVIso(cone.Position(), cone.RefRadius(), cone.SemiAngle(), myParameter);
                            }
                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Sphere:
                {
                    Sphere sph = mySurface.Sphere();

                    switch (myIso)
                    {

                        case IsoU:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.SphereUIso(sph.Position(), sph.Radius(), myParameter);
                            }

                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.SphereVIso(sph.Position(), sph.Radius(), myParameter);
                            }

                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Torus:
                {
                    Torus tor = mySurface.Torus();

                    switch (myIso)
                    {

                        case IsoU:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.TorusUIso(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), myParameter);
                            }

                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.TorusVIso(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), myParameter);
                            }

                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceOfRevolution:
                {
                    if (myIso == IsoV)
                    {
                        Point3D aVal0 = Value(0.0);
                        Axis3D Ax1 = mySurface.AxeOfRevolution();
                        if (new Line3D(Ax1).Contains(aVal0, global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            return new Circle3D(new Frame3D(aVal0, Ax1.Direction()), 0);
                        }
                        else
                        {
                            Vector3D DX = new Vector3D(Ax1.Location(), aVal0);
                            axes = new CoordinateSystem(Ax1.Location(), Ax1.Direction(), new Direction3D(DX));
                            computeHR(axes, aVal0, out h, out radius);
                            Vector3D VT = new Vector3D(axes.Direction());
                            axes.Translate(VT * h);
                            return new Circle3D(axes.Ax2(), radius);
                        }
                    }
                    else
                    {
                        return mySurface.BasisCurve().Circle().Rotated(mySurface.AxeOfRevolution(), myParameter);
                    }
                }

            case SurfaceOfExtrusion:
                {
                    return mySurface.BasisCurve().Circle().Translated(myParameter * new Vector3D(mySurface.Direction()));
                }
            default:
                {
                    throw new NoSuchObjectException("Adaptor3d_IsoCurve:Circle");
                }
        }
        // portage WNT
        return new Circle3D();
    }

    public override Ellipse3D Ellipse()
    {
        switch (mySurface!.GetType())
        {
            case SurfaceOfExtrusion:
                {
                    return mySurface.BasisCurve().Ellipse().Translated(myParameter * new Vector3D(mySurface.Direction()));
                }
            default:
                {
                    throw new NoSuchObjectException("Adaptor3d_IsoCurve:Ellipse");
                }
        }
    }

    public override Hyperbola3D Hyperbola() => throw new NoSuchObjectException("Adaptor3d_IsoCurve:Hyperbola");

    public override Parabola3D Parabola() => throw new NoSuchObjectException("Adaptor3d_IsoCurve:Parabola");

    public override int Degree()
    {
        int degree = 0;
        SurfaceKind type = mySurface!.GetType();
        switch (type)
        {
            case BezierSurface:
            case BSplineSurface:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            degree = mySurface.VDegree();
                            break;
                        case IsoV:
                            degree = mySurface.UDegree();
                            break;
                        case NoneIso:
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            degree = mySurface.BasisCurve().Degree();
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case IsoV:
                            degree = mySurface.BasisCurve().Degree();
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            default:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return degree;
    }

    public override bool IsRational()
    {
        bool is_rational = false;
        SurfaceKind type = mySurface!.GetType();
        switch (type)
        {
            case BezierSurface:
            case BSplineSurface:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            is_rational = mySurface.IsVRational();
                            break;
                        case IsoV:
                            is_rational = mySurface.IsURational();
                            break;
                        case NoneIso:
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            is_rational = mySurface.BasisCurve().IsRational();
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case IsoV:
                            is_rational = mySurface.BasisCurve().IsRational();
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            default:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return is_rational;
    }

    public override int NbPoles()
    {
        int nb_poles = 0;
        SurfaceKind type = mySurface!.GetType();
        switch (type)
        {
            case BezierSurface:
            case BSplineSurface:
                switch (myIso)
                {
                    case IsoU:
                        nb_poles = mySurface.NbVPoles();
                        break;
                    case IsoV:
                        nb_poles = mySurface.NbUPoles();
                        break;
                    case NoneIso:
                    default:
                        throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                }
                break;
            case SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            {
                                nb_poles = mySurface.BasisCurve().NbPoles();
                            }
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case IsoV:
                            {
                                nb_poles = mySurface.BasisCurve().NbPoles();
                            }
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;

            default:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return nb_poles;
    }

    public override int NbKnots()
    {
        int nb_knots = 0;
        SurfaceKind type = mySurface!.GetType();
        switch (type)
        {
            case BSplineSurface:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            nb_knots = mySurface.NbVKnots();
                            break;
                        case IsoV:
                            nb_knots = mySurface.NbUKnots();
                            break;
                        case NoneIso:
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case IsoU:
                            {
                                nb_knots = mySurface.BasisCurve().NbKnots();
                                break;
                            }
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case IsoV:
                            {
                                nb_knots = mySurface.BasisCurve().NbKnots();
                                break;
                            }
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            default:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return nb_knots;
    }

    /// <summary>
    /// The iso as a new Bezier curve: the surface's iso, or a rotated (revolution) or translated (extrusion) copy of
    /// the basis curve. Not segmented to the iso's range.
    /// </summary>
    public override Geom_BezierCurve Bezier()
    {
        Geom_BezierCurve C;
        if (mySurface!.GetType() == SurfaceOfRevolution)
        {
            C = mySurface.BasisCurve().Bezier();
            C = (Geom_BezierCurve)C.Copy();
            C.Rotate(mySurface.AxeOfRevolution(), myParameter);
        }
        else if (mySurface.GetType() == SurfaceOfExtrusion)
        {
            C = mySurface.BasisCurve().Bezier();
            C = (Geom_BezierCurve)C.Copy();
            C.Translate(myParameter * new Vector3D(mySurface.Direction()));
        }
        else if (myIso == IsoU)
        {
            C = (Geom_BezierCurve)mySurface.Bezier().UIso(myParameter);
        }
        else
        {
            C = (Geom_BezierCurve)mySurface.Bezier().VIso(myParameter);
        }
        //  C->Segment(myFirst,myLast);
        return C;
    }

    /// <summary>
    /// The iso as a new B-spline curve: the surface's iso, or a rotated (revolution) or translated (extrusion) copy
    /// of the basis curve. Not segmented to the iso's range.
    /// </summary>
    public override Geom_BSplineCurve BSpline()
    {
        Geom_BSplineCurve C;
        if (mySurface!.GetType() == SurfaceOfRevolution)
        {
            C = mySurface.BasisCurve().BSpline();
            C = (Geom_BSplineCurve)C.Copy();
            C.Rotate(mySurface.AxeOfRevolution(), myParameter);
        }
        else if (mySurface.GetType() == SurfaceOfExtrusion)
        {
            C = mySurface.BasisCurve().BSpline();
            C = (Geom_BSplineCurve)C.Copy();
            C.Translate(myParameter * new Vector3D(mySurface.Direction()));
        }
        else if (myIso == IsoU)
        {
            C = (Geom_BSplineCurve)mySurface.BSpline().UIso(myParameter);
        }
        else
        {
            C = (Geom_BSplineCurve)mySurface.BSpline().VIso(myParameter);
        }
        //  C->Segment(myFirst,myLast);
        return C;
    }
}
