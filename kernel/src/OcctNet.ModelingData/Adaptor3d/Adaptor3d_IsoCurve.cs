// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_IsoCurve.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_IsoCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// An isoparametric curve on a surface: the curve of constant U (IsoU, parametrized by V) or of constant V (IsoV,
/// parametrized by U) of an <see cref="Adaptor3d_Surface"/>, restricted to a parameter range.
/// </summary>
/// <remarks>An adaptor is a working object: <see cref="Load(Adaptor3d_Surface)"/> is public, as in OCCT.</remarks>
public class Adaptor3d_IsoCurve : Adaptor3d_Curve
{
    private Adaptor3d_Surface? mySurface;
    private GeomAbs_IsoType myIso;
    private double myFirst;
    private double myLast;
    private double myParameter;

    /// <summary>An empty adaptor: no surface, no iso.</summary>
    public Adaptor3d_IsoCurve()
    {
        myIso = GeomAbs_NoneIso;
        myFirst = 0.0;
        myLast = 0.0;
        myParameter = 0.0;
    }

    /// <summary>An adaptor on the surface S, without an iso: load one before use.</summary>
    public Adaptor3d_IsoCurve(Adaptor3d_Surface S)
    {
        mySurface = S;
        myIso = GeomAbs_NoneIso;
        myFirst = 0.0;
        myLast = 0.0;
        myParameter = 0.0;
    }

    /// <summary>The iso of type theIso at theParam on S, over the surface's whole range.</summary>
    /// <exception cref="Standard_NoSuchObject">theIso is GeomAbs_NoneIso.</exception>
    public Adaptor3d_IsoCurve(Adaptor3d_Surface S, GeomAbs_IsoType theIso, double theParam)
    {
        mySurface = S;
        myIso = GeomAbs_NoneIso;
        myFirst = 0.0;
        myLast = 0.0;
        myParameter = 0.0;
        Load(theIso, theParam);
    }

    /// <summary>The iso of type theIso at theParam on theS, between theWFirst and theWLast (within the surface's range).</summary>
    public Adaptor3d_IsoCurve(Adaptor3d_Surface theS, GeomAbs_IsoType theIso, double theParam, double theWFirst,
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
        myIso = GeomAbs_NoneIso;
    }

    /// <summary>Changes the iso, over the surface's whole range in the other direction.</summary>
    /// <exception cref="Standard_NoSuchObject">Iso is GeomAbs_NoneIso.</exception>
    public void Load(GeomAbs_IsoType Iso, double Param)
    {
        switch (Iso)
        {
            case GeomAbs_IsoU:
                Load(Iso, Param, mySurface!.FirstVParameter(), mySurface.LastVParameter());
                break;

            case GeomAbs_IsoV:
                Load(Iso, Param, mySurface!.FirstUParameter(), mySurface.LastUParameter());
                break;

            case GeomAbs_NoneIso:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
    }

    /// <summary>
    /// Changes the iso and its range, which is cut to the surface's; on a periodic surface the parameters are
    /// brought into the first period.
    /// </summary>
    public void Load(GeomAbs_IsoType Iso, double Param, double WFirst, double WLast)
    {
        myIso = Iso;
        myParameter = Param;
        myFirst = WFirst;
        myLast = WLast;

        if (myIso == GeomAbs_IsoU)
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

            if (myIso == GeomAbs_IsoU)
            {
                ElCLib.AdjustPeriodic(mySurface.FirstUParameter(),
                                      mySurface.FirstUParameter() + mySurface.UPeriod(),
                                      mySurface.UResolution(Precision.Confusion()),
                                      ref myParameter,
                                      ref dummy);
            }
            else
            {
                ElCLib.AdjustPeriodic(mySurface.FirstUParameter(),
                                      mySurface.FirstUParameter() + mySurface.UPeriod(),
                                      mySurface.UResolution(Precision.Confusion()),
                                      ref myFirst,
                                      ref myLast);
            }
        }

        if (mySurface.IsVPeriodic())
        {

            if (myIso == GeomAbs_IsoV)
            {
                ElCLib.AdjustPeriodic(mySurface.FirstVParameter(),
                                      mySurface.FirstVParameter() + mySurface.VPeriod(),
                                      mySurface.VResolution(Precision.Confusion()),
                                      ref myParameter,
                                      ref dummy);
            }
            else
            {
                ElCLib.AdjustPeriodic(mySurface.FirstVParameter(),
                                      mySurface.FirstVParameter() + mySurface.VPeriod(),
                                      mySurface.VResolution(Precision.Confusion()),
                                      ref myFirst,
                                      ref myLast);
            }
        }
    }

    /// <summary>The surface (null if none is loaded).</summary>
    public Adaptor3d_Surface? Surface() => mySurface;

    /// <summary>The type of iso.</summary>
    public GeomAbs_IsoType Iso() => myIso;

    /// <summary>The constant parameter of the iso.</summary>
    public double Parameter() => myParameter;

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    public override GeomAbs_Shape Continuity()
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                return mySurface!.VContinuity();
            case GeomAbs_IsoV:
                return mySurface!.UContinuity();
            case GeomAbs_NoneIso:
            default:
                break;
        }

        throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
    }

    public override int NbIntervals(GeomAbs_Shape S)
    {
        if (myIso == GeomAbs_NoneIso)
        {
            throw new Standard_NoSuchObject();
        }

        bool UIso = (myIso == GeomAbs_IsoU);

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

    public override void Intervals(NCollection_Array1<double> TI, GeomAbs_Shape S)
    {
        if (myIso == GeomAbs_NoneIso)
        {
            throw new Standard_NoSuchObject();
        }

        bool UIso = (myIso == GeomAbs_IsoU);

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
            case GeomAbs_IsoU:
                return mySurface!.IsVClosed();
            case GeomAbs_IsoV:
                return mySurface!.IsUClosed();
            case GeomAbs_NoneIso:
            default:
                break;
        }

        throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
    }

    public override bool IsPeriodic()
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                return mySurface!.IsVPeriodic();
            case GeomAbs_IsoV:
                return mySurface!.IsUPeriodic();
            case GeomAbs_NoneIso:
            default:
                break;
        }

        throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
    }

    public override double Period()
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                return mySurface!.VPeriod();
            case GeomAbs_IsoV:
                return mySurface!.UPeriod();
            case GeomAbs_NoneIso:
            default:
                break;
        }

        throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
    }

    /// <summary>The point of parameter theU.</summary>
    public sealed override gp_Pnt EvalD0(double theU)
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                return mySurface!.EvalD0(myParameter, theU);
            case GeomAbs_IsoV:
                return mySurface!.EvalD0(theU, myParameter);
            case GeomAbs_NoneIso:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return new gp_Pnt();
    }

    /// <summary>The point and first derivative at theU.</summary>
    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                {
                    Geom_Surface.ResD1 aRes = mySurface!.EvalD1(myParameter, theU);
                    return new Geom_Curve.ResD1 { Point = aRes.Point, D1 = aRes.D1V };
                }
            case GeomAbs_IsoV:
                {
                    Geom_Surface.ResD1 aRes = mySurface!.EvalD1(theU, myParameter);
                    return new Geom_Curve.ResD1 { Point = aRes.Point, D1 = aRes.D1U };
                }
            case GeomAbs_NoneIso:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Geom_Curve.ResD1();
    }

    /// <summary>The point and first two derivatives at theU.</summary>
    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                {
                    Geom_Surface.ResD2 aRes = mySurface!.EvalD2(myParameter, theU);
                    return new Geom_Curve.ResD2 { Point = aRes.Point, D1 = aRes.D1V, D2 = aRes.D2V };
                }
            case GeomAbs_IsoV:
                {
                    Geom_Surface.ResD2 aRes = mySurface!.EvalD2(theU, myParameter);
                    return new Geom_Curve.ResD2 { Point = aRes.Point, D1 = aRes.D1U, D2 = aRes.D2U };
                }
            case GeomAbs_NoneIso:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Geom_Curve.ResD2();
    }

    /// <summary>The point and first three derivatives at theU.</summary>
    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                {
                    Geom_Surface.ResD3 aRes = mySurface!.EvalD3(myParameter, theU);
                    return new Geom_Curve.ResD3 { Point = aRes.Point, D1 = aRes.D1V, D2 = aRes.D2V, D3 = aRes.D3V };
                }
            case GeomAbs_IsoV:
                {
                    Geom_Surface.ResD3 aRes = mySurface!.EvalD3(theU, myParameter);
                    return new Geom_Curve.ResD3 { Point = aRes.Point, D1 = aRes.D1U, D2 = aRes.D2U, D3 = aRes.D3U };
                }
            case GeomAbs_NoneIso:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Geom_Curve.ResD3();
    }

    /// <summary>The derivative of order theN at theU.</summary>
    public sealed override gp_Vec EvalDN(double theU, int theN)
    {
        switch (myIso)
        {
            case GeomAbs_IsoU:
                return mySurface!.EvalDN(myParameter, theU, 0, theN);
            case GeomAbs_IsoV:
                return mySurface!.EvalDN(theU, myParameter, theN, 0);
            case GeomAbs_NoneIso:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return new gp_Vec();
    }

    /// <summary>Precision::Parametric(R3D), whatever the surface.</summary>
    public override double Resolution(double R3D)
    {
        // Peut-on faire mieux ??
        return Precision.Parametric(R3D);
    }

    public override GeomAbs_CurveType GetType()
    {
        switch (mySurface!.GetType())
        {
            case GeomAbs_Plane:
                return GeomAbs_Line;

            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            return GeomAbs_Line;

                        case GeomAbs_IsoV:
                            return GeomAbs_Circle;

                        case GeomAbs_NoneIso:
                            {
                                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case GeomAbs_Sphere:
            case GeomAbs_Torus:
                return GeomAbs_Circle;

            case GeomAbs_BezierSurface:
                return GeomAbs_BezierCurve;

            case GeomAbs_BSplineSurface:
                return GeomAbs_BSplineCurve;

            case GeomAbs_SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            return mySurface.BasisCurve().GetType();

                        case GeomAbs_IsoV:
                            return GeomAbs_Circle;

                        case GeomAbs_NoneIso:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                    break;
                }

            case GeomAbs_SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            return GeomAbs_Line;

                        case GeomAbs_IsoV:
                            return mySurface.BasisCurve().GetType();

                        case GeomAbs_NoneIso:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                    break;
                }

            default:
                return GeomAbs_OtherCurve;
        }

        // portage WNT
        return GeomAbs_OtherCurve;
    }

    /// <summary>The line through the point and tangent at parameter 0.</summary>
    public override gp_Lin Line()
    {
        gp_Pnt P;
        gp_Vec V;
        D1(0, out P, out V);
        return new gp_Lin(P, new gp_Dir(V));
    }

    private static void computeHR(in gp_Ax3 axes, in gp_Pnt P, out double h, out double radius)
    {
        gp_Vec V = new gp_Vec(axes.Location(), P);
        h = V * new gp_Vec(axes.Direction());
        radius = V * new gp_Vec(axes.XDirection());
    }

    public override gp_Circ Circle()
    {
        gp_Ax3 axes;
        double radius, h = 0.0;

        switch (mySurface!.GetType())
        {
            case GeomAbs_Cylinder:
                {
                    gp_Cylinder cyl = mySurface.Cylinder();

                    switch (myIso)
                    {

                        case GeomAbs_IsoU:
                            {
                                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:UIso");
                            }
                        case GeomAbs_IsoV:
                            {
                                return ElSLib.CylinderVIso(cyl.Position(), cyl.Radius(), myParameter);
                            }
                        case GeomAbs_NoneIso:
                            {
                                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case GeomAbs_Cone:
                {
                    gp_Cone cone = mySurface.Cone();

                    switch (myIso)
                    {

                        case GeomAbs_IsoU:
                            {
                                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:UIso");
                            }
                        case GeomAbs_IsoV:
                            {
                                return ElSLib.ConeVIso(cone.Position(), cone.RefRadius(), cone.SemiAngle(), myParameter);
                            }
                        case GeomAbs_NoneIso:
                            {
                                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case GeomAbs_Sphere:
                {
                    gp_Sphere sph = mySurface.Sphere();

                    switch (myIso)
                    {

                        case GeomAbs_IsoU:
                            {
                                return ElSLib.SphereUIso(sph.Position(), sph.Radius(), myParameter);
                            }

                        case GeomAbs_IsoV:
                            {
                                return ElSLib.SphereVIso(sph.Position(), sph.Radius(), myParameter);
                            }

                        case GeomAbs_NoneIso:
                            {
                                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case GeomAbs_Torus:
                {
                    gp_Torus tor = mySurface.Torus();

                    switch (myIso)
                    {

                        case GeomAbs_IsoU:
                            {
                                return ElSLib.TorusUIso(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), myParameter);
                            }

                        case GeomAbs_IsoV:
                            {
                                return ElSLib.TorusVIso(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), myParameter);
                            }

                        case GeomAbs_NoneIso:
                            {
                                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case GeomAbs_SurfaceOfRevolution:
                {
                    if (myIso == GeomAbs_IsoV)
                    {
                        gp_Pnt aVal0 = Value(0.0);
                        gp_Ax1 Ax1 = mySurface.AxeOfRevolution();
                        if (new gp_Lin(Ax1).Contains(aVal0, Precision.Confusion()))
                        {
                            return new gp_Circ(new gp_Ax2(aVal0, Ax1.Direction()), 0);
                        }
                        else
                        {
                            gp_Vec DX = new gp_Vec(Ax1.Location(), aVal0);
                            axes = new gp_Ax3(Ax1.Location(), Ax1.Direction(), new gp_Dir(DX));
                            computeHR(axes, aVal0, out h, out radius);
                            gp_Vec VT = new gp_Vec(axes.Direction());
                            axes.Translate(VT * h);
                            return new gp_Circ(axes.Ax2(), radius);
                        }
                    }
                    else
                    {
                        return mySurface.BasisCurve().Circle().Rotated(mySurface.AxeOfRevolution(), myParameter);
                    }
                }

            case GeomAbs_SurfaceOfExtrusion:
                {
                    return mySurface.BasisCurve().Circle().Translated(myParameter * new gp_Vec(mySurface.Direction()));
                }
            default:
                {
                    throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:Circle");
                }
        }
        // portage WNT
        return new gp_Circ();
    }

    public override gp_Elips Ellipse()
    {
        switch (mySurface!.GetType())
        {
            case GeomAbs_SurfaceOfExtrusion:
                {
                    return mySurface.BasisCurve().Ellipse().Translated(myParameter * new gp_Vec(mySurface.Direction()));
                }
            default:
                {
                    throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:Ellipse");
                }
        }
    }

    public override gp_Hypr Hyperbola() => throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:Hyperbola");

    public override gp_Parab Parabola() => throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:Parabola");

    public override int Degree()
    {
        int degree = 0;
        GeomAbs_SurfaceType type = mySurface!.GetType();
        switch (type)
        {
            case GeomAbs_BezierSurface:
            case GeomAbs_BSplineSurface:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            degree = mySurface.VDegree();
                            break;
                        case GeomAbs_IsoV:
                            degree = mySurface.UDegree();
                            break;
                        case GeomAbs_NoneIso:
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case GeomAbs_SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            degree = mySurface.BasisCurve().Degree();
                            break;
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case GeomAbs_SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoV:
                            degree = mySurface.BasisCurve().Degree();
                            break;
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            default:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return degree;
    }

    public override bool IsRational()
    {
        bool is_rational = false;
        GeomAbs_SurfaceType type = mySurface!.GetType();
        switch (type)
        {
            case GeomAbs_BezierSurface:
            case GeomAbs_BSplineSurface:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            is_rational = mySurface.IsVRational();
                            break;
                        case GeomAbs_IsoV:
                            is_rational = mySurface.IsURational();
                            break;
                        case GeomAbs_NoneIso:
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case GeomAbs_SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            is_rational = mySurface.BasisCurve().IsRational();
                            break;
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case GeomAbs_SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoV:
                            is_rational = mySurface.BasisCurve().IsRational();
                            break;
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            default:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return is_rational;
    }

    public override int NbPoles()
    {
        int nb_poles = 0;
        GeomAbs_SurfaceType type = mySurface!.GetType();
        switch (type)
        {
            case GeomAbs_BezierSurface:
            case GeomAbs_BSplineSurface:
                switch (myIso)
                {
                    case GeomAbs_IsoU:
                        nb_poles = mySurface.NbVPoles();
                        break;
                    case GeomAbs_IsoV:
                        nb_poles = mySurface.NbUPoles();
                        break;
                    case GeomAbs_NoneIso:
                    default:
                        throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                }
                break;
            case GeomAbs_SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            {
                                nb_poles = mySurface.BasisCurve().NbPoles();
                            }
                            break;
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case GeomAbs_SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoV:
                            {
                                nb_poles = mySurface.BasisCurve().NbPoles();
                            }
                            break;
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;

            default:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
        }
        return nb_poles;
    }

    public override int NbKnots()
    {
        int nb_knots = 0;
        GeomAbs_SurfaceType type = mySurface!.GetType();
        switch (type)
        {
            case GeomAbs_BSplineSurface:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            nb_knots = mySurface.NbVKnots();
                            break;
                        case GeomAbs_IsoV:
                            nb_knots = mySurface.NbUKnots();
                            break;
                        case GeomAbs_NoneIso:
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case GeomAbs_SurfaceOfRevolution:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoU:
                            {
                                nb_knots = mySurface.BasisCurve().NbKnots();
                                break;
                            }
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case GeomAbs_SurfaceOfExtrusion:
                {
                    switch (myIso)
                    {
                        case GeomAbs_IsoV:
                            {
                                nb_knots = mySurface.BasisCurve().NbKnots();
                                break;
                            }
                        default:
                            throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            default:
                throw new Standard_NoSuchObject("Adaptor3d_IsoCurve:NoneIso");
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
        if (mySurface!.GetType() == GeomAbs_SurfaceOfRevolution)
        {
            C = mySurface.BasisCurve().Bezier();
            C = (Geom_BezierCurve)C.Copy();
            C.Rotate(mySurface.AxeOfRevolution(), myParameter);
        }
        else if (mySurface.GetType() == GeomAbs_SurfaceOfExtrusion)
        {
            C = mySurface.BasisCurve().Bezier();
            C = (Geom_BezierCurve)C.Copy();
            C.Translate(myParameter * new gp_Vec(mySurface.Direction()));
        }
        else if (myIso == GeomAbs_IsoU)
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
        if (mySurface!.GetType() == GeomAbs_SurfaceOfRevolution)
        {
            C = mySurface.BasisCurve().BSpline();
            C = (Geom_BSplineCurve)C.Copy();
            C.Rotate(mySurface.AxeOfRevolution(), myParameter);
        }
        else if (mySurface.GetType() == GeomAbs_SurfaceOfExtrusion)
        {
            C = mySurface.BasisCurve().BSpline();
            C = (Geom_BSplineCurve)C.Copy();
            C.Translate(myParameter * new gp_Vec(mySurface.Direction()));
        }
        else if (myIso == GeomAbs_IsoU)
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
