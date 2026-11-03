// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Curve.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Curve.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// An edge's 3D curve seen through a projector: the 2D curve of its projection (points, derivatives, conic types) and
/// the 3D curve it comes from, with the conversions between the parameters on both.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (decision 1092): HLRBRep_EdgeData holds one by value and HLRBRep_Data updates it.
/// OCCT keeps a pointer to HLRBRep_Data's projector; the port keeps the box that holds it
/// (<see cref="StrongBox{T}"/>), so the projector set later by <c>HLRBRep_Data.Update</c> is the one every curve sees
/// (decision 1093). <see cref="Copy"/> is OCCT's implicit copy (the adaptor copied, the projector shared).
/// OCCT's <see cref="D3"/> writes nothing and <see cref="DN"/> returns the null vector; both are kept.
/// </remarks>
public sealed class HLRBRep_Curve
{
    private readonly BRepAdaptor_Curve myCurve;
    private CurveKind myType;
    private StrongBox<HLRAlgo_Projector>? myProj;
    private double myOX;
    private double myOZ;
    private double myVX;
    private double myVZ;
    private double myOF;

    /// <summary>Creates an undefined curve.</summary>
    public HLRBRep_Curve()
    {
        myCurve = new BRepAdaptor_Curve();
    }

    private HLRBRep_Curve(HLRBRep_Curve theOther)
    {
        myCurve = (BRepAdaptor_Curve)theOther.myCurve.ShallowCopy();
        myType = theOther.myType;
        myProj = theOther.myProj;
        myOX = theOther.myOX;
        myOZ = theOther.myOZ;
        myVX = theOther.myVX;
        myVZ = theOther.myVZ;
        myOF = theOther.myOF;
    }

    /// <summary>OCCT's implicit copy: the adaptor copied, the projector shared.</summary>
    public HLRBRep_Curve Copy() => new(this);

    /// <summary>Sets the projector (OCCT's pointer to the projector HLRBRep_Data holds).</summary>
    public void Projector(StrongBox<HLRAlgo_Projector> Proj) => myProj = Proj;

    private ref readonly HLRAlgo_Projector Proj() => ref myProj!.Value;

    /// <summary>Returns the 3D curve.</summary>
    public BRepAdaptor_Curve Curve() => myCurve;

    /// <summary>Sets the 3D curve to be projected.</summary>
    public void Curve(in TopoDS_Edge E)
    {
        myCurve.Initialize(E);
    }

    /// <summary>Returns the 3D curve.</summary>
    public BRepAdaptor_Curve GetCurve() => myCurve;

    /// <summary>Returns the parameter on the 2d curve from the parameter on the 3d curve.</summary>
    public double Parameter2d(double P3d)
    {
        // Mathematical formula for lines

        //        myOF P3d (myOF myVX - myOZ myVX + myOX myVZ)
        // Res -> --------------------------------------------
        //        (-myOF + myOZ) (-myOF + myOZ + P3d myVZ)

        switch (myType)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                if (Proj().Perspective())
                {
                    double FmOZ = myOF - myOZ;
                    return myOF * P3d * (myVX * FmOZ + myOX * myVZ) / (FmOZ * (FmOZ - P3d * myVZ));
                }
                return P3d * myVX;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                return P3d + myOX;

            default: // implemented to avoid gcc compiler warnings
                break;
        }
        return P3d;
    }

    /// <summary>Returns the parameter on the 3d curve from the parameter on the 2d curve.</summary>
    public double Parameter3d(double P2d)
    {
        // Mathematical formula for lines

        //                                 2
        //                   (-myOF + myOZ)  P2d
        // P3d -> -----------------------------------------------------
        //        (myOF - myOZ) (myOF myVX + P2d myVZ) + myOF myOX myVZ

        if (myType == global::Kerf.Geometry.CurveKind.Line)
        {
            if (Proj().Perspective())
            {
                double FmOZ = myOF - myOZ;
                return P2d * FmOZ * FmOZ / (FmOZ * (myOF * myVX + P2d * myVZ) + myOF * myOX * myVZ);
            }
            return ((myVX <= Constants.Resolution()) ? P2d : (P2d / myVX));
        }

        else if (myType == global::Kerf.Geometry.CurveKind.Ellipse)
        {
            return P2d - myOX;
        }

        return P2d;
    }

    /// <summary>Updates the min-max boxes and the internal data; returns the tolerance to enlarge them by.</summary>
    public double Update(Span<double> TotMin, Span<double> TotMax)
    {
        CurveKind typ = HLRBRep_BCurveTool.GetType(myCurve);
        myType = OtherCurve;

        switch (typ)
        {

            case global::Kerf.Geometry.CurveKind.Line:
                myType = typ;
                break;

            case global::Kerf.Geometry.CurveKind.Circle:
                if (!Proj().Perspective())
                {
                    Direction3D D1 = HLRBRep_BCurveTool.Circle(myCurve).Axis().Direction();
                    D1.Transform(Proj().Transformation());
                    if (D1.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        myType = global::Kerf.Geometry.CurveKind.Circle;
                    }
                    else if (Abs(D1.Dot(Constants.DZ())) < global::Kerf.Engine.Foundation.Precision.Angular() * 10)
                    { //*10: The minor radius of ellipse should not be too small.
                        myType = OtherCurve;
                    }
                    else
                    {
                        myType = global::Kerf.Geometry.CurveKind.Ellipse;
                        // compute the angle offset
                        Direction3D D3 = D1.Crossed(Constants.DZ());
                        Direction3D D2 = HLRBRep_BCurveTool.Circle(myCurve).XAxis().Direction();
                        D2.Transform(Proj().Transformation());
                        myOX = D3.AngleWithRef(D2, D1);
                    }
                }
                break;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                if (!Proj().Perspective())
                {
                    Direction3D D1 = HLRBRep_BCurveTool.Ellipse(myCurve).Axis().Direction();
                    D1.Transform(Proj().Transformation());
                    if (D1.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        myOX = 0.0; // no offset on the angle
                        myType = global::Kerf.Geometry.CurveKind.Ellipse;
                    }
                }
                break;

            case BezierCurve:
                if (HLRBRep_BCurveTool.Degree(myCurve) == 1)
                {
                    myType = global::Kerf.Geometry.CurveKind.Line;
                }
                else if (!Proj().Perspective())
                {
                    myType = typ;
                }
                break;

            case BSplineCurve:
                if (!Proj().Perspective())
                {
                    myType = typ;
                }
                break;

            default:
                break;
        }

        if (myType == global::Kerf.Geometry.CurveKind.Line)
        {
            // compute the values for a line
            Line3D L;
            double l3d = 1.0; // length of the 3d bezier curve
            if (HLRBRep_BCurveTool.GetType(myCurve) == global::Kerf.Geometry.CurveKind.Line)
            {
                L = HLRBRep_BCurveTool.Line(myCurve);
            }
            else
            { // bezier degree 1
                HLRBRep_BCurveTool.D1(myCurve, 0, out Point3D PL, out Vector3D VL);
                L = new Line3D(PL, new Direction3D(VL));
                l3d = PL.Distance(HLRBRep_BCurveTool.Value(myCurve, 1.0));
            }
            Point3D P = L.Location();
            Vector3D V = new(L.Direction());
            P.Transform(Proj().Transformation());
            V.Transform(Proj().Transformation());
            if (Proj().Perspective())
            {
                D1(0.0, out Point2D F, out Vector2D VFX);
                VFX.Normalize();
                myVX = (VFX.X() * V.X() + VFX.Y() * V.Y()) * l3d;
                double l = -(VFX.X() * F.X() + VFX.Y() * F.Y());
                F.SetCoord(F.X() + VFX.X() * l, F.Y() + VFX.Y() * l);
                myOX = VFX.X() * (P.X() - F.X()) + VFX.Y() * (P.Y() - F.Y());
                Vector3D VFZ = new(-F.X(), -F.Y(), Proj().Focus());
                myOF = VFZ.Magnitude();
                VFZ /= myOF;
                myVZ = VFZ * V;
                myVZ *= l3d;
                myOZ = VFZ * new Vector3D(P.X() - F.X(), P.Y() - F.Y(), P.Z());
            }
            else
            {
                myVX = Sqrt(V.X() * V.X() + V.Y() * V.Y()) * l3d;
            }
        }
        return (UpdateMinMax(TotMin, TotMax));
    }

    /// <summary>Updates the min-max boxes; returns the tolerance to enlarge them by.</summary>
    public double UpdateMinMax(Span<double> TotMin, Span<double> TotMax)
    {
        double a = HLRBRep_BCurveTool.FirstParameter(myCurve);
        double b = HLRBRep_BCurveTool.LastParameter(myCurve);
        double x, y, z, tolMinMax = 0;
        Proj().Project(Value3D(a), out x, out y, out z);
        HLRAlgo.UpdateMinMax(x, y, z, TotMin, TotMax);

        if (myType != global::Kerf.Geometry.CurveKind.Line)
        {
            int nbPnt = 30;
            int i;
            double step = (b - a) / (nbPnt + 1);
            double xa, ya, za, xb = 0.0, yb = 0.0, zb = 0.0;
            double dx1, dy1, dz1, dd1;
            double dx2, dy2, dz2, dd2;

            for (i = 1; i <= nbPnt; i++)
            {
                a += step;
                xa = xb;
                ya = yb;
                za = zb;
                xb = x;
                yb = y;
                zb = z;
                Proj().Project(Value3D(a), out x, out y, out z);
                HLRAlgo.UpdateMinMax(x, y, z, TotMin, TotMax);
                if (i >= 2)
                {
                    dx1 = x - xa;
                    dy1 = y - ya;
                    dz1 = z - za;
                    dd1 = Sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);
                    if (dd1 > 0)
                    {
                        dx2 = xb - xa;
                        dy2 = yb - ya;
                        dz2 = zb - za;
                        dd2 = Sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
                        if (dd2 > 0)
                        {
                            double p = (dx1 * dx2 + dy1 * dy2 + dz1 * dz2) / (dd1 * dd2);
                            dx1 = xa + p * dx1 - xb;
                            dy1 = ya + p * dy1 - yb;
                            dz1 = za + p * dz1 - zb;
                            dd1 = Sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);
                            if (dd1 > tolMinMax)
                            {
                                tolMinMax = dd1;
                            }
                        }
                    }
                }
            }
        }
        Proj().Project(Value3D(b), out x, out y, out z);
        HLRAlgo.UpdateMinMax(x, y, z, TotMin, TotMax);
        return tolMinMax;
    }

    /// <summary>The Z coordinate, in the viewing coordinate system, of the point of parameter U.</summary>
    public double Z(double U)
    {
        HLRBRep_BCurveTool.D0(myCurve, U, out Point3D P3d);
        P3d.Transform(Proj().Transformation());
        return P3d.Z();
    }

    /// <summary>The 3D point of parameter U.</summary>
    public Point3D Value3D(double U)
    {
        D0(U, out Point3D P);
        return P;
    }

    /// <summary>The 3D point of parameter U.</summary>
    public void D0(double U, out Point3D P)
    {
        HLRBRep_BCurveTool.D0(myCurve, U, out P);
    }

    /// <summary>The 3D point of parameter U and the first derivative.</summary>
    public void D1(double U, out Point3D P, out Vector3D V)
    {
        HLRBRep_BCurveTool.D1(myCurve, U, out P, out V);
    }

    /// <summary>
    /// The 2D point and tangent at the start (or the end) of the curve. If the first derivative is null, looks a little
    /// after the start (or before the end) with the second derivative.
    /// </summary>
    /// <exception cref="UndefinedDerivativeException">The tangent is undefined.</exception>
    public void Tangent(bool AtStart, out Point2D P, out Direction2D D)
    {
        double U = AtStart ? HLRBRep_BCurveTool.FirstParameter(myCurve) : HLRBRep_BCurveTool.LastParameter(myCurve);

        D0(U, out P);
        HLRBRep_CLProps CLP = new(2, Epsilon(1.0));
        HLRBRep_Curve aCurve = this;
        CLP.SetCurve(aCurve);
        CLP.SetParameter(U);
        StdFail_UndefinedDerivative_Raise_if(!CLP.IsTangentDefined(), "HLRBRep_Curve::Tangent");
        CLP.Tangent(out D);
    }

    public double FirstParameter() => Parameter2d(HLRBRep_BCurveTool.FirstParameter(myCurve));

    public double LastParameter() => Parameter2d(HLRBRep_BCurveTool.LastParameter(myCurve));

    public Continuity Continuity() => HLRBRep_BCurveTool.Continuity(myCurve);

    /// <summary>The number of intervals of continuity S.</summary>
    public int NbIntervals(Continuity S) => HLRBRep_BCurveTool.NbIntervals(myCurve, S);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S.</summary>
    public void Intervals(NCollection_Array1<double> Tab, Continuity Sh)
    {
        HLRBRep_BCurveTool.Intervals(myCurve, Tab, Sh);
    }

    public bool IsClosed() => HLRBRep_BCurveTool.IsClosed(myCurve);

    public bool IsPeriodic() => HLRBRep_BCurveTool.IsPeriodic(myCurve);

    public double Period() => HLRBRep_BCurveTool.Period(myCurve);

    /// <summary>The 2D point of parameter U (a parameter on the 3D curve).</summary>
    public Point2D Value(double U)
    {
        D0(U, out Point2D P);
        return P;
    }

    /// <summary>The 2D point of parameter U.</summary>
    public void D0(double U, out Point2D P)
    {
        /* gp_Pnt P3d;
        HLRBRep_BCurveTool::D0(myCurve,U,P3d);
        P3d.Transform(((HLRAlgo_Projector*) myProj)->Transformation());
        if (((HLRAlgo_Projector*) myProj)->Perspective()) {
          double R = 1.-P3d.Z()/((HLRAlgo_Projector*) myProj)->Focus();
          P.SetCoord(P3d.X()/R,P3d.Y()/R);
        }
        else P.SetCoord(P3d.X(),P3d.Y()); */
        HLRBRep_BCurveTool.D0(myCurve, U, out Point3D P3d);
        P = new Point2D(); // C++'s default point, which the first axonometric view leaves (decision 1078)
        Proj().Project(P3d, ref P);
    }

    /// <summary>The 2D point of parameter U and the first derivative.</summary>
    public void D1(double U, out Point2D P, out Vector2D V)
    {
        // Mathematical formula for lines

        //        X'[t]      X[t] Z'[t]
        // D1 =  -------- + -------------
        //           Z[t]          Z[t] 2
        //       1 - ----   f (1 - ----)
        //            f             f

        HLRBRep_BCurveTool.D1(myCurve, U, out Point3D P3D, out Vector3D V13D);
        if (Proj().Perspective())
        {
            P3D.Transform(Proj().Transformation());
            V13D.Transform(Proj().Transformation());

            double f = Proj().Focus();
            double R = 1.0 - P3D.Z() / f;
            double e = V13D.Z() / (f * R * R);
            P = new Point2D(P3D.X() / R, P3D.Y() / R);
            V = new Vector2D(V13D.X() / R + P3D.X() * e, V13D.Y() / R + P3D.Y() * e);
        }
        else
        {
            // OCC155
            Proj().Project(P3D, V13D, out P, out V);
        }
    }

    /// <summary>The 2D point of parameter U and the first two derivatives.</summary>
    public void D2(double U, out Point2D P, out Vector2D V1, out Vector2D V2)
    {
        // Mathematical formula for lines

        //                                   2
        //       2 X'[t] Z'[t]   2 X[t] Z'[t]      X''[t]     X[t] Z''[t]
        // D2 =  ------------- + -------------- + -------- + -------------
        //              Z[t] 2    2      Z[t] 3       Z[t]          Z[t] 2
        //       f (1 - ----)    f  (1 - ----)    1 - ----   f (1 - ----)
        //               f                f            f             f

        HLRBRep_BCurveTool.D2(myCurve, U, out Point3D P3D, out Vector3D V13D, out Vector3D V23D);
        P3D.Transform(Proj().Transformation());
        V13D.Transform(Proj().Transformation());
        V23D.Transform(Proj().Transformation());
        if (Proj().Perspective())
        {
            double f = Proj().Focus();
            double R = 1.0 - P3D.Z() / f;
            double q = f * R * R;
            double e = V13D.Z() / q;
            double c = e * V13D.Z() / (f * R);
            P = new Point2D(P3D.X() / R, P3D.Y() / R);
            V1 = new Vector2D(V13D.X() / R + P3D.X() * e, V13D.Y() / R + P3D.Y() * e);
            V2 = new Vector2D(V23D.X() / R + 2 * V13D.X() * e + P3D.X() * V23D.Z() / q + 2 * P3D.X() * c,
                              V23D.Y() / R + 2 * V13D.Y() * e + P3D.Y() * V23D.Z() / q + 2 * P3D.Y() * c);
        }
        else
        {
            P = new Point2D(P3D.X(), P3D.Y());
            V1 = new Vector2D(V13D.X(), V13D.Y());
            V2 = new Vector2D(V23D.X(), V23D.Y());
        }
    }

    /// <summary>OCCT's third derivative: writes nothing (the arguments keep their values).</summary>
    public void D3(double U, ref Point2D P, ref Vector2D V1, ref Vector2D V2, ref Vector2D V3) { } // OCCT

    /// <summary>OCCT's derivative of order N: the null vector.</summary>
    public Vector2D DN(double U, int N) => new(); // OCCT

    /// <summary>The parametric resolution for the 3D resolution R3d.</summary>
    public double Resolution(double R3d) => HLRBRep_BCurveTool.Resolution(myCurve, R3d);

    /// <summary>The type of the projected curve.</summary>
    public new CurveKind GetType() => myType;

    public Line2D Line()
    {
        D1(0.0, out Point2D P, out Vector2D V);
        return new Line2D(P, new Direction2D(V));
    }

    public Circle2D Circle()
    {
        Circle3D C = HLRBRep_BCurveTool.Circle(myCurve);
        C.Transform(Proj().Transformation());
        return ProjLib.Project(new Plane3D(new CoordinateSystem(Constants.XOY())), C);
    }

    public Ellipse2D Ellipse()
    {
        if (HLRBRep_BCurveTool.GetType(myCurve) == global::Kerf.Geometry.CurveKind.Ellipse)
        {
            Ellipse3D E = HLRBRep_BCurveTool.Ellipse(myCurve);
            E.Transform(Proj().Transformation());
            return ProjLib.Project(new Plane3D(new CoordinateSystem(Constants.XOY())), E);
        }
        // this is a circle
        Circle3D C = HLRBRep_BCurveTool.Circle(myCurve);
        C.Transform(Proj().Transformation());
        Direction3D D1 = C.Axis().Direction();
        Direction3D D3 = D1.Crossed(Constants.DZ());
        Direction3D D2 = D1.Crossed(D3);
        double rap = Sqrt(D2.X() * D2.X() + D2.Y() * D2.Y());
        Direction2D d = new(D1.Y(), -D1.X());
        Point2D p = new(C.Location().X(), C.Location().Y());
        Ellipse2D El = new(new Axis2D(p, d), C.Radius(), C.Radius() * rap);
        if (D1.Z() < 0)
        {
            El.Reverse();
        }
        return El;
    }

    public Hyperbola2D Hyperbola() => new();

    public Parabola2D Parabola() => new();

    public bool IsRational() => HLRBRep_BCurveTool.IsRational(myCurve);

    public int Degree() => HLRBRep_BCurveTool.Degree(myCurve);

    public int NbPoles() => HLRBRep_BCurveTool.NbPoles(myCurve);

    /// <summary>The projected poles of the Bezier or B-spline curve.</summary>
    public void Poles(NCollection_Array1<Point2D> TP)
    {
        int i1 = TP.Lower();
        int i2 = TP.Upper();
        NCollection_Array1<Point3D> TP3 = new(i1, i2);
        if (HLRBRep_BCurveTool.GetType(myCurve) == BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = HLRBRep_BCurveTool.BSpline(myCurve);
            NCollection_Array1<Point3D> aSrcPoles = aBSpl.InternalPoles();
            for (int i = i1; i <= i2; i++)
            {
                TP3[i] = aSrcPoles[i];
            }
        }
        else
        {
            Geom_BezierCurve aBez = HLRBRep_BCurveTool.Bezier(myCurve);
            NCollection_Array1<Point3D> aSrcPoles = aBez.InternalPoles();
            for (int i = i1; i <= i2; i++)
            {
                TP3[i] = aSrcPoles[i];
            }
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref TP3[i]);
            TP[i] = new Point2D(TP3[i].X(), TP3[i].Y());
        }
    }

    /// <summary>The projected poles of the given B-spline curve.</summary>
    public void Poles(Geom_BSplineCurve aCurve, NCollection_Array1<Point2D> TP)
    {
        int i1 = TP.Lower();
        int i2 = TP.Upper();
        NCollection_Array1<Point3D> TP3 = new(i1, i2);
        NCollection_Array1<Point3D> aSrcPoles = aCurve.InternalPoles();
        for (int i = i1; i <= i2; i++)
        {
            TP3[i] = aSrcPoles[i];
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref TP3[i]);
            TP[i] = new Point2D(TP3[i].X(), TP3[i].Y());
        }
    }

    /// <summary>The projected poles and the weights of the Bezier or B-spline curve.</summary>
    public void PolesAndWeights(NCollection_Array1<Point2D> TP, NCollection_Array1<double> TW)
    {
        int i1 = TP.Lower();
        int i2 = TP.Upper();
        NCollection_Array1<Point3D> TP3 = new(i1, i2);
        if (HLRBRep_BCurveTool.GetType(myCurve) == BSplineCurve)
        {
            Geom_BSplineCurve HB = (HLRBRep_BCurveTool.BSpline(myCurve));
            NCollection_Array1<Point3D> aSrcPoles = HB.InternalPoles();
            NCollection_Array1<double> aSrcWeights = HB.InternalWeightsArray();
            for (int i = i1; i <= i2; i++)
            {
                TP3[i] = aSrcPoles[i];
            }
            for (int i = i1; i <= i2; i++)
            {
                TW[i] = aSrcWeights[i];
            }
        }
        else
        {
            Geom_BezierCurve HB = (HLRBRep_BCurveTool.Bezier(myCurve));
            NCollection_Array1<Point3D> aSrcPoles = HB.InternalPoles();
            NCollection_Array1<double> aSrcWeights = HB.InternalWeightsArray();
            for (int i = i1; i <= i2; i++)
            {
                TP3[i] = aSrcPoles[i];
            }
            for (int i = i1; i <= i2; i++)
            {
                TW[i] = aSrcWeights[i];
            }
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref TP3[i]);
            TP[i] = new Point2D(TP3[i].X(), TP3[i].Y());
        }
    }

    /// <summary>The projected poles and the weights of the given B-spline curve.</summary>
    public void PolesAndWeights(Geom_BSplineCurve aCurve, NCollection_Array1<Point2D> TP,
                                NCollection_Array1<double> TW)
    {
        int i1 = TP.Lower();
        int i2 = TP.Upper();
        NCollection_Array1<Point3D> TP3 = new(i1, i2);
        NCollection_Array1<Point3D> aSrcPoles = aCurve.InternalPoles();
        NCollection_Array1<double> aSrcWeights = aCurve.InternalWeightsArray();
        for (int i = i1; i <= i2; i++)
        {
            TP3[i] = aSrcPoles[i];
        }
        for (int i = i1; i <= i2; i++)
        {
            TW[i] = aSrcWeights[i];
        }
        for (int i = i1; i <= i2; i++)
        {
            Proj().Transform(ref TP3[i]);
            TP[i] = new Point2D(TP3[i].X(), TP3[i].Y());
        }
    }

    public int NbKnots() => HLRBRep_BCurveTool.NbKnots(myCurve);

    /// <summary>The knots of the B-spline curve (nothing for other curves).</summary>
    public void Knots(NCollection_Array1<double> kn)
    {
        if (HLRBRep_BCurveTool.GetType(myCurve) == BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = HLRBRep_BCurveTool.BSpline(myCurve);
            NCollection_Array1<double> aSrcKnots = aBSpl.InternalKnots();
            for (int i = kn.Lower(); i <= kn.Upper(); i++)
            {
                kn[i] = aSrcKnots[i];
            }
        }
    }

    /// <summary>The multiplicities of the B-spline curve (nothing for other curves).</summary>
    public void Multiplicities(NCollection_Array1<int> mu)
    {
        if (HLRBRep_BCurveTool.GetType(myCurve) == BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = HLRBRep_BCurveTool.BSpline(myCurve);
            NCollection_Array1<int> aSrcMults = aBSpl.InternalMultiplicities();
            for (int i = mu.Lower(); i <= mu.Upper(); i++)
            {
                mu[i] = aSrcMults[i];
            }
        }
    }
}
