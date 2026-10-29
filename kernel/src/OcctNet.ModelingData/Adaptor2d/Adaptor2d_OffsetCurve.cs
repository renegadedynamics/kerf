// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_OffsetCurve.hxx
// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The offset of an adapted 2D curve, between two parameters: the point at distance Offset from the curve
/// along its normal (the tangent rotated by -90 degrees).
/// </summary>
/// <remarks>An adaptor is a working object: <see cref="Load(Adaptor2d_Curve2d)"/> is public, as in OCCT.</remarks>
public class Adaptor2d_OffsetCurve : Adaptor2d_Curve2d
{
    private Adaptor2d_Curve2d? myCurve;
    private double myOffset;
    private double myFirst;
    private double myLast;

    /// <summary>An empty adaptor; load a curve before use.</summary>
    public Adaptor2d_OffsetCurve()
    {
        myOffset = 0.0;
        myFirst = 0.0;
        myLast = 0.0;
    }

    /// <summary>The curve theCurve with offset 0 (and parameters 0 to 0 until an offset is loaded).</summary>
    public Adaptor2d_OffsetCurve(Adaptor2d_Curve2d theCurve)
    {
        myCurve = theCurve;
        myOffset = 0.0;
        myFirst = 0.0;
        myLast = 0.0;
    }

    /// <summary>The offset of theCurve by theOffset, over the whole curve.</summary>
    public Adaptor2d_OffsetCurve(Adaptor2d_Curve2d theCurve, double theOffset)
    {
        myCurve = theCurve;
        myOffset = theOffset;
        myFirst = theCurve.FirstParameter();
        myLast = theCurve.LastParameter();
    }

    /// <summary>The offset of theCurve by theOffset, between theWFirst and theWLast.</summary>
    public Adaptor2d_OffsetCurve(Adaptor2d_Curve2d theCurve, double theOffset, double theWFirst, double theWLast)
    {
        myCurve = theCurve;
        myOffset = theOffset;
        myFirst = theWFirst;
        myLast = theWLast;
    }

    // OCCT's implicit copy constructor: shares the basis adaptor.
    private Adaptor2d_OffsetCurve(Adaptor2d_OffsetCurve theOther)
    {
        myCurve = theOther.myCurve;
        myOffset = theOther.myOffset;
        myFirst = theOther.myFirst;
        myLast = theOther.myLast;
    }

    /// <summary>A copy with a shallow copy of the basis adaptor.</summary>
    public override Adaptor2d_Curve2d ShallowCopy()
    {
        Adaptor2d_OffsetCurve aCopy = new Adaptor2d_OffsetCurve();

        if (myCurve is not null)
        {
            aCopy.myCurve = myCurve.ShallowCopy();
        }
        aCopy.myOffset = myOffset;
        aCopy.myFirst = myFirst;
        aCopy.myLast = myLast;

        return aCopy;
    }

    /// <summary>Loads the basis curve S, with offset 0 (the parameter range is unchanged).</summary>
    public void Load(Adaptor2d_Curve2d S)
    {
        myCurve = S;
        myOffset = 0.0;
    }

    /// <summary>Loads the offset, over the whole basis curve.</summary>
    public void Load(double Offset)
    {
        myOffset = Offset;
        myFirst = myCurve!.FirstParameter();
        myLast = myCurve.LastParameter();
    }

    /// <summary>Loads the offset and the parameter range.</summary>
    public void Load(double Offset, double WFirst, double WLast)
    {
        myOffset = Offset;
        myFirst = WFirst;
        myLast = WLast;
    }

    /// <summary>The basis curve (null if none is loaded).</summary>
    public Adaptor2d_Curve2d? Curve() => myCurve;

    /// <summary>The offset value.</summary>
    public double Offset() => myOffset;

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    /// <summary>The continuity, lowered from the basis curve's by one step of C3, C2, G2, C1, G1, C0.</summary>
    /// <exception cref="Standard_TypeMismatch">The basis curve is C0.</exception>
    public override GeomAbs_Shape Continuity()
    {
        switch (myCurve!.Continuity())
        {
            case GeomAbs_CN:
                return GeomAbs_CN;
            case GeomAbs_C3:
                return GeomAbs_C2;
            case GeomAbs_C2:
                return GeomAbs_G2;
            case GeomAbs_G2:
                return GeomAbs_C1;
            case GeomAbs_C1:
                return GeomAbs_G1;
            case GeomAbs_G1:
                return GeomAbs_C0;
            case GeomAbs_C0:
                // No Continuity !!
                throw new Standard_TypeMismatch("Adaptor2d_OffsetCurve::IntervalContinuity");
        }

        // portage WNT
        return GeomAbs_C0;
    }

    public override int NbIntervals(GeomAbs_Shape S)
    {
        GeomAbs_Shape Sh;
        if (S >= GeomAbs_C2)
        {
            Sh = GeomAbs_CN;
        }
        else
        {
            Sh = (GeomAbs_Shape)((int)S + 2);
        }

        int nbInter = myCurve!.NbIntervals(Sh);

        if (nbInter == 1)
        {
            return nbInter;
        }

        NCollection_Array1<double> T = new NCollection_Array1<double>(1, nbInter + 1);

        myCurve.Intervals(T, Sh);

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
        GeomAbs_Shape Sh;
        if (S >= GeomAbs_C2)
        {
            Sh = GeomAbs_CN;
        }
        else
        {
            Sh = (GeomAbs_Shape)((int)S + 2);
        }

        int nbInter = myCurve!.NbIntervals(Sh);

        if (nbInter == 1)
        {
            TI[TI.Lower()] = myFirst;
            TI[TI.Lower() + 1] = myLast;
            return;
        }

        NCollection_Array1<double> T = new NCollection_Array1<double>(1, nbInter + 1);
        myCurve.Intervals(T, Sh);

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

    /// <summary>The same offset between First and Last (sharing the basis adaptor).</summary>
    public override Adaptor2d_Curve2d Trim(double First, double Last, double Tol)
    {
        Adaptor2d_OffsetCurve HO = new Adaptor2d_OffsetCurve(this);
        HO.Load(myOffset, First, Last);
        return HO;
    }

    /// <summary>
    /// For a non-zero offset: true if the basis curve is closed and its end tangents are parallel and in the
    /// same sense (false for a C0 basis).
    /// </summary>
    public override bool IsClosed()
    {
        if (myOffset == 0.0)
        {
            return myCurve!.IsClosed();
        }
        else
        {
            if (myCurve!.Continuity() == GeomAbs_C0)
            {
                return false;
            }
            else
            {
                if (myCurve.IsClosed())
                {
                    gp_Vec2d Dummy0, Dummy1;
                    gp_Pnt2d P;
                    myCurve.D1(myCurve.FirstParameter(), out P, out Dummy0);
                    myCurve.D1(myCurve.LastParameter(), out P, out Dummy1);
                    return Dummy0.IsParallel(Dummy1, Precision.Angular())
                           && !(Dummy0.IsOpposite(Dummy1, Precision.Angular()));
                }
                else
                {
                    return false;
                }
            }
        }
    }

    public override bool IsPeriodic() => myCurve!.IsPeriodic();

    public override double Period() => myCurve!.Period();

    public override gp_Pnt2d Value(double U)
    {
        if (myOffset != 0.0)
        {
            gp_Pnt2d aP;
            gp_Vec2d aV;
            myCurve!.D1(U, out aP, out aV);
            Geom2d_OffsetCurveUtils.CalculateD0(ref aP, aV, myOffset);
            return aP;
        }
        else
        {
            return myCurve!.Value(U);
        }
    }

    public override void D0(double U, out gp_Pnt2d P) => P = Value(U);

    public override void D1(double U, out gp_Pnt2d P, out gp_Vec2d V)
    {
        if (myOffset != 0.0)
        {
            gp_Vec2d aV2;
            myCurve!.D2(U, out P, out V, out aV2);
            Geom2d_OffsetCurveUtils.CalculateD1(ref P, ref V, aV2, myOffset);
        }
        else
        {
            myCurve!.D1(U, out P, out V);
        }
    }

    public override void D2(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        if (myOffset != 0.0)
        {
            gp_Vec2d aV3;
            myCurve!.D3(U, out P, out V1, out V2, out aV3);
            Geom2d_OffsetCurveUtils.CalculateD2(ref P, ref V1, ref V2, aV3, false, myOffset);
        }
        else
        {
            myCurve!.D2(U, out P, out V1, out V2);
        }
    }

    /// <remarks>OCCT's D2 and D3 here are wrong where the basis curve's speed varies (occt-issues.md).</remarks>
    public override void D3(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        if (myOffset != 0.0)
        {
            gp_Vec2d aV4 = myCurve!.DN(U, 4);
            myCurve.D3(U, out P, out V1, out V2, out V3);
            Geom2d_OffsetCurveUtils.CalculateD3(ref P, ref V1, ref V2, ref V3, aV4, false, myOffset);
        }
        else
        {
            myCurve!.D3(U, out P, out V1, out V2, out V3);
        }
    }

    /// <exception cref="Standard_NotImplemented">Always.</exception>
    public override gp_Vec2d DN(double U, int N) => throw new Standard_NotImplemented("Adaptor2d_OffsetCurve::DN");

    /// <summary>Precision::PConfusion(R3d).</summary>
    public override double Resolution(double R3d) => Precision.PConfusion(R3d);

    /// <summary>The basis curve's type for a zero offset; Line or Circle for the offset of one; OffsetCurve otherwise.</summary>
    public override GeomAbs_CurveType GetType()
    {
        if (myOffset == 0.0)
        {
            return myCurve!.GetType();
        }
        else
        {
            switch (myCurve!.GetType())
            {
                case GeomAbs_Line:
                    return GeomAbs_Line;

                case GeomAbs_Circle:
                    return GeomAbs_Circle;

                default:
                    return GeomAbs_OffsetCurve;
            }
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset curve is not a line.</exception>
    public override gp_Lin2d Line()
    {
        if (GetType() == GeomAbs_Line)
        {
            gp_Pnt2d P;
            gp_Vec2d V;
            D1(0, out P, out V);
            return new gp_Lin2d(P, new gp_Dir2d(V));
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve::Line");
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset curve is not a circle, or its radius is 0.</exception>
    public override gp_Circ2d Circle()
    {
        if (GetType() == GeomAbs_Circle)
        {
            if (myOffset == 0.0)
            {
                return myCurve!.Circle();
            }
            else
            {
                gp_Circ2d C1 = myCurve!.Circle();
                double radius = C1.Radius();
                gp_Ax22d axes = C1.Axis();
                gp_Dir2d Xd = axes.XDirection();
                gp_Dir2d Yd = axes.YDirection();
                double Crossed = Xd.X() * Yd.Y() - Xd.Y() * Yd.X();
                double Signe = (Crossed > 0.0) ? 1.0 : -1.0;

                radius += Signe * myOffset;
                if (radius > 0.0)
                {
                    return new gp_Circ2d(axes, radius);
                }
                else if (radius < 0.0)
                {
                    radius = -radius;
                    axes.SetXDirection((axes.XDirection()).Reversed());
                    return new gp_Circ2d(axes, radius);
                }
                else
                { // Cercle de rayon Nul
                    throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve::Circle");
                }
            }
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve::Circle");
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0 or the basis is not an ellipse.</exception>
    public override gp_Elips2d Ellipse()
    {
        if (myCurve!.GetType() == GeomAbs_Ellipse && myOffset == 0.0)
        {
            return myCurve.Ellipse();
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve:Ellipse");
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0 or the basis is not a hyperbola.</exception>
    public override gp_Hypr2d Hyperbola()
    {
        if (myCurve!.GetType() == GeomAbs_Hyperbola && myOffset == 0.0)
        {
            return myCurve.Hyperbola();
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve:Hyperbola");
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0 or the basis is not a parabola.</exception>
    public override gp_Parab2d Parabola()
    {
        if (myCurve!.GetType() == GeomAbs_Parabola && myOffset == 0.0)
        {
            return myCurve.Parabola();
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve:Parabola");
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0 or the basis is not a Bezier or B-spline curve.</exception>
    public override int Degree()
    {
        GeomAbs_CurveType type = myCurve!.GetType();
        if ((type == GeomAbs_BezierCurve || type == GeomAbs_BSplineCurve) && myOffset == 0.0)
        {
            return myCurve.Degree();
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve::Degree");
        }
    }

    /// <summary>The basis curve's IsRational for a zero offset, false otherwise.</summary>
    public override bool IsRational()
    {
        if (myOffset == 0.0)
        {
            return myCurve!.IsRational();
        }
        return false;
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0 or the basis is not a Bezier or B-spline curve.</exception>
    public override int NbPoles()
    {
        GeomAbs_CurveType type = myCurve!.GetType();
        if ((type == GeomAbs_BezierCurve || type == GeomAbs_BSplineCurve) && myOffset == 0.0)
        {
            return myCurve.NbPoles();
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve::NbPoles");
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0.</exception>
    public override int NbKnots()
    {
        if (myOffset == 0.0)
        {
            return myCurve!.NbKnots();
        }
        else
        {
            throw new Standard_NoSuchObject("Adaptor2d_OffsetCurve::NbKnots");
        }
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0 or the curve is not a Bezier curve.</exception>
    public override Geom2d_BezierCurve Bezier()
    {
        Standard_NoSuchObject_Raise_if(myOffset != 0.0e0 || GetType() != GeomAbs_BezierCurve,
                                       "Adaptor2d_OffsetCurve::Bezier() - wrong curve type");
        return myCurve!.Bezier();
    }

    /// <exception cref="Standard_NoSuchObject">The offset is not 0 or the curve is not a B-spline curve.</exception>
    public override Geom2d_BSplineCurve BSpline()
    {
        Standard_NoSuchObject_Raise_if(myOffset != 0.0e0 || GetType() != GeomAbs_BSplineCurve,
                                       "Adaptor2d_OffsetCurve::BSpline() - wrong curve type");
        return myCurve!.BSpline();
    }

    private static int nbPoints(Adaptor2d_Curve2d theCurve)
    {
        int nbs = 20;

        if (theCurve.GetType() == GeomAbs_BezierCurve)
        {
            nbs = Max(nbs, 3 + theCurve.NbPoles());
        }
        else if (theCurve.GetType() == GeomAbs_BSplineCurve)
        {
            nbs = Max(nbs, theCurve.NbKnots() * theCurve.Degree());
        }

        if (nbs > 300)
        {
            nbs = 300;
        }
        return nbs;
    }

    /// <summary>The basis curve's number of samples: 20, or more for Bezier and B-spline curves (300 at most).</summary>
    public override int NbSamples() => nbPoints(myCurve!);
}
