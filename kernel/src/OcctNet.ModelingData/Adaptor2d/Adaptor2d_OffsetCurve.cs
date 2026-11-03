// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_OffsetCurve.hxx
// Port of: src/ModelingData/TKG2d/Adaptor2d/Adaptor2d_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

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
    /// <exception cref="TypeMismatchException">The basis curve is C0.</exception>
    public override Continuity Continuity()
    {
        switch (myCurve!.Continuity())
        {
            case CN:
                return CN;
            case C3:
                return C2;
            case C2:
                return G2;
            case G2:
                return C1;
            case C1:
                return G1;
            case G1:
                return C0;
            case C0:
                // No Continuity !!
                throw new TypeMismatchException("Adaptor2d_OffsetCurve::IntervalContinuity");
        }

        // portage WNT
        return C0;
    }

    public override int NbIntervals(Continuity S)
    {
        Continuity Sh;
        if (S >= C2)
        {
            Sh = CN;
        }
        else
        {
            Sh = (Continuity)((int)S + 2);
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

    public override void Intervals(NCollection_Array1<double> TI, Continuity S)
    {
        Continuity Sh;
        if (S >= C2)
        {
            Sh = CN;
        }
        else
        {
            Sh = (Continuity)((int)S + 2);
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
            if (myCurve!.Continuity() == C0)
            {
                return false;
            }
            else
            {
                if (myCurve.IsClosed())
                {
                    Vector2D Dummy0, Dummy1;
                    Point2D P;
                    myCurve.D1(myCurve.FirstParameter(), out P, out Dummy0);
                    myCurve.D1(myCurve.LastParameter(), out P, out Dummy1);
                    return Dummy0.IsParallel(Dummy1, global::Kerf.Engine.Foundation.Precision.Angular())
                           && !(Dummy0.IsOpposite(Dummy1, global::Kerf.Engine.Foundation.Precision.Angular()));
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

    public override Point2D Value(double U)
    {
        if (myOffset != 0.0)
        {
            Point2D aP;
            Vector2D aV;
            myCurve!.D1(U, out aP, out aV);
            Geom2d_OffsetCurveUtils.CalculateD0(ref aP, aV, myOffset);
            return aP;
        }
        else
        {
            return myCurve!.Value(U);
        }
    }

    public override void D0(double U, out Point2D P) => P = Value(U);

    public override void D1(double U, out Point2D P, out Vector2D V)
    {
        if (myOffset != 0.0)
        {
            Vector2D aV2;
            myCurve!.D2(U, out P, out V, out aV2);
            Geom2d_OffsetCurveUtils.CalculateD1(ref P, ref V, aV2, myOffset);
        }
        else
        {
            myCurve!.D1(U, out P, out V);
        }
    }

    public override void D2(double U, out Point2D P, out Vector2D V1, out Vector2D V2)
    {
        if (myOffset != 0.0)
        {
            Vector2D aV3;
            myCurve!.D3(U, out P, out V1, out V2, out aV3);
            Geom2d_OffsetCurveUtils.CalculateD2(ref P, ref V1, ref V2, aV3, false, myOffset);
        }
        else
        {
            myCurve!.D2(U, out P, out V1, out V2);
        }
    }

    /// <remarks>OCCT's D2 and D3 here are wrong where the basis curve's speed varies (occt-issues.md).</remarks>
    public override void D3(double U, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3)
    {
        if (myOffset != 0.0)
        {
            Vector2D aV4 = myCurve!.DN(U, 4);
            myCurve.D3(U, out P, out V1, out V2, out V3);
            Geom2d_OffsetCurveUtils.CalculateD3(ref P, ref V1, ref V2, ref V3, aV4, false, myOffset);
        }
        else
        {
            myCurve!.D3(U, out P, out V1, out V2, out V3);
        }
    }

    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public override Vector2D DN(double U, int N) => throw new MethodNotImplementedException("Adaptor2d_OffsetCurve::DN");

    /// <summary>Precision::PConfusion(R3d).</summary>
    public override double Resolution(double R3d) => global::Kerf.Engine.Foundation.Precision.PConfusion(R3d);

    /// <summary>The basis curve's type for a zero offset; Line or Circle for the offset of one; OffsetCurve otherwise.</summary>
    public override CurveKind GetType()
    {
        if (myOffset == 0.0)
        {
            return myCurve!.GetType();
        }
        else
        {
            switch (myCurve!.GetType())
            {
                case global::Kerf.Geometry.CurveKind.Line:
                    return global::Kerf.Geometry.CurveKind.Line;

                case global::Kerf.Geometry.CurveKind.Circle:
                    return global::Kerf.Geometry.CurveKind.Circle;

                default:
                    return OffsetCurve;
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The offset curve is not a line.</exception>
    public override Line2D Line()
    {
        if (GetType() == global::Kerf.Geometry.CurveKind.Line)
        {
            Point2D P;
            Vector2D V;
            D1(0, out P, out V);
            return new Line2D(P, new Direction2D(V));
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Line");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset curve is not a circle, or its radius is 0.</exception>
    public override Circle2D Circle()
    {
        if (GetType() == global::Kerf.Geometry.CurveKind.Circle)
        {
            if (myOffset == 0.0)
            {
                return myCurve!.Circle();
            }
            else
            {
                Circle2D C1 = myCurve!.Circle();
                double radius = C1.Radius();
                Frame2D axes = C1.Axis();
                Direction2D Xd = axes.XDirection();
                Direction2D Yd = axes.YDirection();
                double Crossed = Xd.X() * Yd.Y() - Xd.Y() * Yd.X();
                double Signe = (Crossed > 0.0) ? 1.0 : -1.0;

                radius += Signe * myOffset;
                if (radius > 0.0)
                {
                    return new Circle2D(axes, radius);
                }
                else if (radius < 0.0)
                {
                    radius = -radius;
                    axes.SetXDirection((axes.XDirection()).Reversed());
                    return new Circle2D(axes, radius);
                }
                else
                { // Cercle de rayon Nul
                    throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Circle");
                }
            }
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Circle");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not an ellipse.</exception>
    public override Ellipse2D Ellipse()
    {
        if (myCurve!.GetType() == global::Kerf.Geometry.CurveKind.Ellipse && myOffset == 0.0)
        {
            return myCurve.Ellipse();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve:Ellipse");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a hyperbola.</exception>
    public override Hyperbola2D Hyperbola()
    {
        if (myCurve!.GetType() == global::Kerf.Geometry.CurveKind.Hyperbola && myOffset == 0.0)
        {
            return myCurve.Hyperbola();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve:Hyperbola");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a parabola.</exception>
    public override Parabola2D Parabola()
    {
        if (myCurve!.GetType() == global::Kerf.Geometry.CurveKind.Parabola && myOffset == 0.0)
        {
            return myCurve.Parabola();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve:Parabola");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a Bezier or B-spline curve.</exception>
    public override int Degree()
    {
        CurveKind type = myCurve!.GetType();
        if ((type == BezierCurve || type == BSplineCurve) && myOffset == 0.0)
        {
            return myCurve.Degree();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::Degree");
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

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the basis is not a Bezier or B-spline curve.</exception>
    public override int NbPoles()
    {
        CurveKind type = myCurve!.GetType();
        if ((type == BezierCurve || type == BSplineCurve) && myOffset == 0.0)
        {
            return myCurve.NbPoles();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::NbPoles");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0.</exception>
    public override int NbKnots()
    {
        if (myOffset == 0.0)
        {
            return myCurve!.NbKnots();
        }
        else
        {
            throw new NoSuchObjectException("Adaptor2d_OffsetCurve::NbKnots");
        }
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the curve is not a Bezier curve.</exception>
    public override Geom2d_BezierCurve Bezier()
    {
        Standard_NoSuchObject_Raise_if(myOffset != 0.0e0 || GetType() != BezierCurve,
                                       "Adaptor2d_OffsetCurve::Bezier() - wrong curve type");
        return myCurve!.Bezier();
    }

    /// <exception cref="NoSuchObjectException">The offset is not 0 or the curve is not a B-spline curve.</exception>
    public override Geom2d_BSplineCurve BSpline()
    {
        Standard_NoSuchObject_Raise_if(myOffset != 0.0e0 || GetType() != BSplineCurve,
                                       "Adaptor2d_OffsetCurve::BSpline() - wrong curve type");
        return myCurve!.BSpline();
    }

    private static int nbPoints(Adaptor2d_Curve2d theCurve)
    {
        int nbs = 20;

        if (theCurve.GetType() == BezierCurve)
        {
            nbs = Max(nbs, 3 + theCurve.NbPoles());
        }
        else if (theCurve.GetType() == BSplineCurve)
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
