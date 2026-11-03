// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnPlane.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectOnPlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// 09-Aug-95 : xab : changed the ProjLib_ProjectOnPlane in the case
//                   of the line and the parameteriation is kept

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// Projects a curve on a plane along a direction (the plane's normal by default). The projected curve is a line,
/// conic, Bezier or B-spline where it can be computed exactly or by apex search; otherwise it is approximated by a
/// B-spline. Lines, conics and polynomial curves keep their parametrization when requested.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Load"/> is public. Keeps the caller's curve adaptor by reference, as
/// OCCT keeps the handle. The curves it builds are frozen when handed out through <see cref="Bezier"/>,
/// <see cref="BSpline"/> or <see cref="GetResult"/> (decision 243): the same objects at every call.
/// </remarks>
public class ProjLib_ProjectOnPlane : Adaptor3d_Curve
{
    private const double aParabolaLimit = 20000.0;
    private const double aHyperbolaLimit = 10.0;

    private Adaptor3d_Curve? myCurve;
    private CoordinateSystem myPlane = new();
    private Direction3D myDirection = new();
    private bool myKeepParam;
    private double myFirstPar;
    private double myLastPar;
    private double myTolerance;
    private CurveKind myType;
    private GeomAdaptor_Curve? myResult;
    private bool myIsApprox;

    private static Point3D OnPlane_Value(double U, Adaptor3d_Curve aCurvePtr, in CoordinateSystem Pl, in Direction3D D)
    {
        //                   PO . Z                /  Z = Pl.Direction()
        // Proj(u) = P(u) + -------  * D     with  \  O = Pl.Location()
        //                   D  . Z

        Point3D Point = aCurvePtr.Value(U);

        Vector3D PO = new(Point, Pl.Location());

        double Alpha = PO * new Vector3D(Pl.Direction());
        Alpha /= D * Pl.Direction();
        Point.SetXYZ(Point.XYZ() + Alpha * D.XYZ());

        return Point;
    }

    private static Vector3D OnPlane_DN(double U, int DerivativeRequest, Adaptor3d_Curve aCurvePtr, in CoordinateSystem Pl, in Direction3D D)
    {
        //                   PO . Z                /  Z = Pl.Direction()
        // Proj(u) = P(u) + -------  * D     with  \  O = Pl.Location()
        //                   D  . Z

        Vector3D Vector = aCurvePtr.DN(U, DerivativeRequest);

        Direction3D Z = Pl.Direction();

        double Alpha = Vector * new Vector3D(Z);
        Alpha /= D * Z;

        Vector.SetXYZ(Vector.XYZ() - Alpha * D.XYZ());
        return Vector;
    }

    private static bool OnPlane_D1(double U, out Point3D P, out Vector3D V, Adaptor3d_Curve aCurvePtr, in CoordinateSystem Pl, in Direction3D D)
    {
        double Alpha;
        Point3D Point;
        Vector3D Vector;

        Direction3D Z = Pl.Direction();

        aCurvePtr.D1(U, out Point, out Vector);

        // evaluate the point as in `OnPlane_Value`
        Vector3D PO = new(Point, Pl.Location());
        Alpha = PO * new Vector3D(Z);
        Alpha /= D * Z;
        P = new Point3D();
        P.SetXYZ(Point.XYZ() + Alpha * D.XYZ());

        // evaluate the derivative.
        //
        //   d(Proj)  d(P)       1        d(P)
        //   ------ = ---  - -------- * ( --- . Z ) * D
        //     dU     dU     ( D . Z)     dU
        //

        Alpha = Vector * new Vector3D(Z);
        Alpha /= D * Z;

        V = new Vector3D();
        V.SetXYZ(Vector.XYZ() - Alpha * D.XYZ());

        return true;
    }

    private static bool OnPlane_D2(double U,
                                   out Point3D P,
                                   out Vector3D V1,
                                   out Vector3D V2,
                                   Adaptor3d_Curve aCurvePtr,
                                   in CoordinateSystem Pl,
                                   in Direction3D D)
    {
        double Alpha;
        Point3D Point;
        Vector3D Vector1, Vector2;

        Direction3D Z = Pl.Direction();

        aCurvePtr.D2(U, out Point, out Vector1, out Vector2);

        // evaluate the point as in `OnPlane_Value`
        Vector3D PO = new(Point, Pl.Location());
        Alpha = PO * new Vector3D(Z);
        Alpha /= D * Z;
        P = new Point3D();
        P.SetXYZ(Point.XYZ() + Alpha * D.XYZ());

        // evaluate the derivative.
        //
        //   d(Proj)  d(P)       1        d(P)
        //   ------ = ---  - -------- * ( --- . Z ) * D
        //     dU     dU     ( D . Z)     dU
        //

        Alpha = Vector1 * new Vector3D(Z);
        Alpha /= D * Z;

        V1 = new Vector3D();
        V1.SetXYZ(Vector1.XYZ() - Alpha * D.XYZ());

        Alpha = Vector2 * new Vector3D(Z);
        Alpha /= D * Z;

        V2 = new Vector3D();
        V2.SetXYZ(Vector2.XYZ() - Alpha * D.XYZ());
        return true;
    }

    private static bool OnPlane_D3(double U,
                                   out Point3D P,
                                   out Vector3D V1,
                                   out Vector3D V2,
                                   out Vector3D V3,
                                   Adaptor3d_Curve aCurvePtr,
                                   in CoordinateSystem Pl,
                                   in Direction3D D)
    {
        double Alpha;
        Point3D Point;
        Vector3D Vector1, Vector2, Vector3;

        Direction3D Z = Pl.Direction();

        aCurvePtr.D3(U, out Point, out Vector1, out Vector2, out Vector3);

        // evaluate the point as in `OnPlane_Value`
        Vector3D PO = new(Point, Pl.Location());
        Alpha = PO * new Vector3D(Z);
        Alpha /= D * Z;
        P = new Point3D();
        P.SetXYZ(Point.XYZ() + Alpha * D.XYZ());

        // evaluate the derivative.
        //
        //   d(Proj)  d(P)       1        d(P)
        //   ------ = ---  - -------- * ( --- . Z ) * D
        //     dU     dU     ( D . Z)     dU
        //

        Alpha = Vector1 * new Vector3D(Z);
        Alpha /= D * Z;

        V1 = new Vector3D();
        V1.SetXYZ(Vector1.XYZ() - Alpha * D.XYZ());

        Alpha = Vector2 * new Vector3D(Z);
        Alpha /= D * Z;

        V2 = new Vector3D();
        V2.SetXYZ(Vector2.XYZ() - Alpha * D.XYZ());
        Alpha = Vector3 * new Vector3D(Z);
        Alpha /= D * Z;

        V3 = new Vector3D();
        V3.SetXYZ(Vector3.XYZ() - Alpha * D.XYZ());
        return true;
    }

    /// <summary>OCCT's file-local class ProjLib_OnPlane: the projected point, approximated by PerformApprox.</summary>
    private sealed class ProjLib_OnPlane : AppCont_Function
    {
        private readonly Adaptor3d_Curve myCurve;
        private readonly CoordinateSystem myPlane;
        private readonly Direction3D myDirection;

        public ProjLib_OnPlane(Adaptor3d_Curve C, in CoordinateSystem Pl, in Direction3D D)
        {
            myCurve = C;
            myPlane = Pl;
            myDirection = D;
            myNbPnt = 1;
            myNbPnt2d = 0;
        }

        public override double FirstParameter() => myCurve.FirstParameter();

        public override double LastParameter() => myCurve.LastParameter();

        public override bool Value(double theT, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            thePnt[1] = OnPlane_Value(theT, myCurve, myPlane, myDirection);
            return true;
        }

        public override bool D1(double theT, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            return OnPlane_D1(theT, out _, out theVec[1], myCurve, myPlane, myDirection);
        }
    }

    /// <summary>OCCT's file-local class ProjLib_MaxCurvature: minus the curvature, to be minimised.</summary>
    private sealed class ProjLib_MaxCurvature : Function
    {
        private readonly LProp_CLProps3d myProps;

        public ProjLib_MaxCurvature(LProp_CLProps3d theProps)
        {
            myProps = theProps;
        }

        public override bool Value(double X, out double F)
        {
            myProps.SetParameter(X);
            F = -myProps.Curvature();
            return true;
        }
    }

    //=====================================================================//
    //                                                                     //
    //  D E S C R I P T I O N   O F   T H E   C L A S S  :                 //
    //                                                                     //
    //         P r o j L i b _ A p p r o x P r o j e c t O n P l a n e     //
    //                                                                     //
    //=====================================================================//

    private static void PerformApprox(Adaptor3d_Curve C, in CoordinateSystem Pl, in Direction3D D, ref Geom_BSplineCurve? BSplineCurvePtr)
    {
        ProjLib_OnPlane F = new(C, Pl, D);

        int Deg1, Deg2;
        Deg1 = 8;
        Deg2 = 8;
        if (C.GetType() == global::Kerf.Geometry.CurveKind.Parabola)
        {
            Deg1 = 2;
            Deg2 = 2;
        }
        int aNbSegm = 100;
        if (C.GetType() == global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            Deg1 = 14;
            Deg2 = 14;
            aNbSegm = 1000;
        }
        Approx_FitAndDivide Fit = new(Deg1, Deg2, global::Kerf.Engine.Foundation.Precision.Approximation(), global::Kerf.Engine.Foundation.Precision.PApproximation(), true);
        Fit.SetMaxSegments(aNbSegm);
        Fit.Perform(F);
        if (!Fit.IsAllApproximated())
        {
            return;
        }
        int i;
        int NbCurves = Fit.NbMultiCurves();
        int MaxDeg = 0;

        // To convert the MultiCurve to BSpline, all constituent Bezier curves
        // must have the same degree -> Calculate MaxDeg
        int NbPoles;
        for (i = 1; i <= NbCurves; i++)
        {
            int Deg = Fit.Value(i).Degree();
            MaxDeg = Max(MaxDeg, Deg);
        }
        NbPoles = MaxDeg * NbCurves + 1; // Poles on the BSpline

        NCollection_Array1<Point3D> Poles = new(1, NbPoles);

        NCollection_Array1<Point3D> TempPoles = new(1, MaxDeg + 1); // for degree elevation

        NCollection_Array1<double> Knots = new(1, NbCurves + 1); // Knots of the BSpline

        int Compt = 1;
        double anErrMax = 0.0, anErr3d, anErr2d;
        for (i = 1; i <= Fit.NbMultiCurves(); i++)
        {
            Fit.Parameters(i, out Knots[i], out Knots[i + 1]);
            Fit.Error(i, out anErr3d, out anErr2d);
            anErrMax = Max(anErrMax, anErr3d);
            AppParCurves_MultiCurve MC = Fit.Value(i);                    // Load the i-th Curve
            NCollection_Array1<Point3D> LocalPoles = new(1, MC.Degree() + 1); // Get the poles
            MC.Curve(1, LocalPoles);

            // Possible degree elevation
            if (MaxDeg > MC.Degree())
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(MaxDeg, LocalPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), TempPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
                // update the poles of the PCurve
                for (int j = 1; j <= MaxDeg + 1; j++)
                {
                    Poles.SetValue(Compt, TempPoles[j]);
                    Compt++;
                }
            }
            else
            {
                // update the poles of the PCurve
                for (int j = 1; j <= MaxDeg + 1; j++)
                {
                    Poles.SetValue(Compt, LocalPoles[j]);
                    Compt++;
                }
            }

            Compt--;
        }

        // update the fields of ProjLib_Approx

        int NbKnots = NbCurves + 1;

        NCollection_Array1<int> Mults = new(1, NbKnots);
        Mults.SetValue(1, MaxDeg + 1);
        for (i = 2; i <= NbCurves; i++)
        {
            Mults.SetValue(i, MaxDeg);
        }
        Mults.SetValue(NbKnots, MaxDeg + 1);
        BSplineCurvePtr = new Geom_BSplineCurve(Poles, Knots, Mults, MaxDeg, false);

        // Try to smooth
        int m1 = MaxDeg - 1;
        for (i = 2; i < NbKnots; ++i)
        {
            if (BSplineCurvePtr.Multiplicity(i) == MaxDeg)
            {
                BSplineCurvePtr.RemoveKnot(i, m1, anErrMax);
            }
        }
    }

    /// <summary>An empty projector: the plane is the global XOY plane.</summary>
    public ProjLib_ProjectOnPlane()
    {
        myKeepParam = false;
        myFirstPar = 0.0;
        myLastPar = 0.0;
        myTolerance = 0.0;
        myType = OtherCurve;
        myIsApprox = false;
    }

    /// <summary>A projector on <paramref name="Pl"/> along its normal.</summary>
    public ProjLib_ProjectOnPlane(in CoordinateSystem Pl)
    {
        myPlane = Pl;
        myDirection = Pl.Direction();
        myKeepParam = false;
        myFirstPar = 0.0;
        myLastPar = 0.0;
        myTolerance = 0.0;
        myType = OtherCurve;
        myIsApprox = false;
    }

    /// <summary>A projector on <paramref name="Pl"/> along <paramref name="D"/> (not checked to be oblique to the
    /// plane).</summary>
    public ProjLib_ProjectOnPlane(in CoordinateSystem Pl, in Direction3D D)
    {
        myPlane = Pl;
        myDirection = D;
        myKeepParam = false;
        myFirstPar = 0.0;
        myLastPar = 0.0;
        myTolerance = 0.0;
        myType = OtherCurve;
        myIsApprox = false;
        //  if ( std::abs(D * Pl.Direction()) < Precision::Confusion()) {
        //    throw Standard_ConstructionError
        //      ("ProjLib_ProjectOnPlane:  The Direction and the Plane are parallel");
        //  }
    }

    /// <summary>A copy sharing the plane and direction, with shallow copies of the curve and the result adaptors.</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        ProjLib_ProjectOnPlane aCopy = new();

        if (myCurve is not null)
        {
            aCopy.myCurve = myCurve.ShallowCopy();
        }
        aCopy.myPlane = myPlane;
        aCopy.myDirection = myDirection;
        aCopy.myKeepParam = myKeepParam;
        aCopy.myFirstPar = myFirstPar;
        aCopy.myLastPar = myLastPar;
        aCopy.myTolerance = myTolerance;
        aCopy.myType = myType;
        if (myResult is not null)
        {
            aCopy.myResult = myResult.ShallowCopy() as GeomAdaptor_Curve;
        }
        aCopy.myIsApprox = myIsApprox;

        return aCopy;
    }

    private static Point3D ProjectPnt(in CoordinateSystem ThePlane, in Direction3D TheDir, in Point3D Point)
    {
        Vector3D PO = new(Point, ThePlane.Location());

        double Alpha = PO * new Vector3D(ThePlane.Direction());
        Alpha /= TheDir * ThePlane.Direction();

        Point3D P = new();
        P.SetXYZ(Point.XYZ() + Alpha * TheDir.XYZ());

        return P;
    }

    private static Vector3D ProjectVec(in CoordinateSystem ThePlane, in Direction3D TheDir, in Vector3D Vec)
    {
        Vector3D D = Vec;
        Vector3D Z = new(ThePlane.Direction());

        D -= ((Vec * Z) / (TheDir * new Direction3D(Z))) * new Vector3D(TheDir);

        return D;
    }

    /// <summary>
    /// Projects <paramref name="C"/>. With <paramref name="KeepParametrization"/>, the projection keeps the curve's
    /// parameters (lines become degree-1 B-splines, conics that don't stay orthogonal are approximated); Bezier and
    /// B-spline curves always keep them.
    /// </summary>
    public void Load(Adaptor3d_Curve C, double Tolerance, bool KeepParametrization = true)
    {
        myCurve = C;
        myType = OtherCurve;
        myIsApprox = false;
        myTolerance = Tolerance;

        Geom_BSplineCurve? ApproxCurve = null;

        Geom_Line GeomLinePtr;
        Geom_Circle? GeomCirclePtr = null;
        Geom_Ellipse? GeomEllipsePtr = null;
        Geom_Hyperbola? GeomHyperbolaPtr = null;
        Geom_Parabola? GeomParabolaPtr = null;

        Line3D aLine;
        Ellipse3D Elips;
        //  gp_Hypr  Hypr ;

        CurveKind Type = C.GetType();

        Frame3D Axis = new();
        double R1 = 0.0, R2 = 0.0;

        myKeepParam = KeepParametrization;

        switch (Type)
        {
            case global::Kerf.Geometry.CurveKind.Line:
            {
                //     P(u) = O + u * Xc
                // ==> Q(u) = f(P(u))
                //          = f(O) + u * f(Xc)

                Line3D L = myCurve.Line();
                Vector3D Xc = ProjectVec(myPlane, myDirection, new Vector3D(L.Direction()));

                if (Xc.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
                { // line orthogonal to the plane
                    myType = BSplineCurve;
                    Point3D P = ProjectPnt(myPlane, myDirection, L.Location());
                    NCollection_Array1<int> Mults = new(1, 2);
                    Mults.Init(2);
                    NCollection_Array1<Point3D> Poles = new(1, 2);
                    Poles.Init(P);
                    NCollection_Array1<double> Knots = new(1, 2);
                    Knots[1] = myCurve.FirstParameter();
                    Knots[2] = myCurve.LastParameter();
                    Geom_BSplineCurve BSP = new(Poles, Knots, Mults, 1);

                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                    myResult = new GeomAdaptor_Curve(BSP);
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                }
                else if (Math.Abs(Xc.Magnitude() - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    myType = global::Kerf.Geometry.CurveKind.Line;
                    Point3D P = ProjectPnt(myPlane, myDirection, L.Location());
                    myFirstPar = myCurve.FirstParameter();
                    myLastPar = myCurve.LastParameter();
                    aLine = new Line3D(P, new Direction3D(Xc));
                    GeomLinePtr = new Geom_Line(aLine);

                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                    myResult = new GeomAdaptor_Curve(GeomLinePtr, myCurve.FirstParameter(), myCurve.LastParameter());
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                }
                else
                {
                    myType = global::Kerf.Geometry.CurveKind.Line;
                    Point3D P = ProjectPnt(myPlane, myDirection, L.Location());
                    aLine = new Line3D(P, new Direction3D(Xc));
                    double Udeb, Ufin;

                    // eval the first and last parameters of the projected curve
                    Udeb = myCurve.FirstParameter();
                    Ufin = myCurve.LastParameter();
                    Point3D P1 = ProjectPnt(myPlane, myDirection, myCurve.Value(Udeb));
                    Point3D P2 = ProjectPnt(myPlane, myDirection, myCurve.Value(Ufin));
                    myFirstPar = new Vector3D(aLine.Direction()).Dot(new Vector3D(P, P1));
                    myLastPar = new Vector3D(aLine.Direction()).Dot(new Vector3D(P, P2));
                    GeomLinePtr = new Geom_Line(aLine);
                    if (!myKeepParam)
                    {
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                        myResult = new GeomAdaptor_Curve(GeomLinePtr, myFirstPar, myLastPar);
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                    }
                    else
                    {
                        myType = BSplineCurve;
                        //
                        // make a linear BSpline of degree 1 between the end points of
                        // the projected line
                        //
                        Geom_TrimmedCurve NewTrimCurvePtr = new(GeomLinePtr, myFirstPar, myLastPar);

                        Geom_BSplineCurve NewCurvePtr = GeomConvert.CurveToBSplineCurve(NewTrimCurvePtr);
                        NCollection_Array1<double> BsplineKnots = new(NewCurvePtr.InternalKnots());

                        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(myCurve.FirstParameter(), myCurve.LastParameter(), BsplineKnots);

                        NewCurvePtr.SetKnots(BsplineKnots);
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                        myResult = new GeomAdaptor_Curve(NewCurvePtr);
                        //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                    }
                }
                break;
            }
            case global::Kerf.Geometry.CurveKind.Circle:
            case global::Kerf.Geometry.CurveKind.Ellipse:
            {
                if (Type == global::Kerf.Geometry.CurveKind.Circle)
                {
                    // For circle and ellipse we have the following relations:
                    // (Note: for circle R1 = R2 = R)
                    //     P(u) = O + R1 * std::cos(u) * Xc + R2 * std::sin(u) * Yc
                    // ==> Q(u) = f(P(u))
                    //          = f(O) + R1 * std::cos(u) * f(Xc) + R2 * std::sin(u) * f(Yc)

                    Circle3D Circ = myCurve.Circle();
                    Axis = Circ.Position();
                    R1 = R2 = Circ.Radius();
                }
                // [[fallthrough]] from the circle case
                if (Type == global::Kerf.Geometry.CurveKind.Ellipse)
                {
                    Ellipse3D E = myCurve.Ellipse();
                    Axis = E.Position();
                    R1 = E.MajorRadius();
                    R2 = E.MinorRadius();
                }

                // Common Code  for CIRCLE & ELLIPSE begin here
                Direction3D X = Axis.XDirection();
                Direction3D Y = Axis.YDirection();
                Vector3D VDx = ProjectVec(myPlane, myDirection, new Vector3D(X));
                Vector3D VDy = ProjectVec(myPlane, myDirection, new Vector3D(Y));
                Direction3D Dx = new(), Dy = new();

                double Tol2 = myTolerance * myTolerance;
                if (VDx.SquareMagnitude() < Tol2 || VDy.SquareMagnitude() < Tol2 || VDx.CrossSquareMagnitude(VDy) < Tol2)
                {
                    myIsApprox = true;
                }

                if (!myIsApprox)
                {
                    Dx = new Direction3D(VDx);
                    Dy = new Direction3D(VDy);
                    Point3D O = Axis.Location();
                    Point3D P = ProjectPnt(myPlane, myDirection, O);
                    Point3D Px = ProjectPnt(myPlane, myDirection, O.Translated(R1 * new Vector3D(X)));
                    Point3D Py = ProjectPnt(myPlane, myDirection, O.Translated(R2 * new Vector3D(Y)));
                    double Major = P.Distance(Px);
                    double Minor = P.Distance(Py);

                    if (myKeepParam)
                    {
                        myIsApprox = !new Direction3D(VDx).IsNormal(new Direction3D(VDy), global::Kerf.Engine.Foundation.Precision.Angular());
                    }
                    else
                    {
                        // Since it is not necessary to keep the same parameter for the point on the original and
                        // on the projected curves, we will use the following approach to find axes of the
                        // projected ellipse and provide the canonical curve:
                        // https://www.geometrictools.com/Documentation/ParallelProjectionEllipse.pdf
                        Matrix aMatrA = new(1, 2, 1, 2);
                        // A = Jp^T * Pr(Je), where
                        //   Pr(Je) - projection of axes of original ellipse to the target plane
                        //   Jp - X and Y axes of the target plane
                        aMatrA[1, 1] = myPlane.XDirection().XYZ().Dot(VDx.XYZ());
                        aMatrA[1, 2] = myPlane.XDirection().XYZ().Dot(VDy.XYZ());
                        aMatrA[2, 1] = myPlane.YDirection().XYZ().Dot(VDx.XYZ());
                        aMatrA[2, 2] = myPlane.YDirection().XYZ().Dot(VDy.XYZ());

                        Matrix aMatrDelta2 = new(1, 2, 1, 2, 0.0);
                        //           | 1/MajorRad^2       0       |
                        // Delta^2 = |                            |
                        //           |      0        1/MajorRad^2 |
                        aMatrDelta2[1, 1] = 1.0 / (R1 * R1);
                        aMatrDelta2[2, 2] = 1.0 / (R2 * R2);

                        Matrix aMatrAInv = aMatrA.Inverse();
                        Matrix aMatrM = aMatrAInv.Transposed() * aMatrDelta2 * aMatrAInv;

                        // perform eigenvalues calculation
                        Jacobi anEigenCalc = new(aMatrM);
                        if (anEigenCalc.IsDone)
                        {
                            // radii of the projected ellipse
                            Minor = 1.0 / Math.Sqrt(anEigenCalc.Value(1));
                            Major = 1.0 / Math.Sqrt(anEigenCalc.Value(2));

                            // calculate the rotation angle for the plane axes to meet the correct axes of the
                            // projected ellipse (swap eigenvectors in respect to major and minor axes)
                            Matrix anEigenVec = anEigenCalc.Vectors();
                            Transform2D aTrsfInPlane = new();
                            aTrsfInPlane.SetValues(anEigenVec[1, 2], anEigenVec[1, 1], 0.0, anEigenVec[2, 2], anEigenVec[2, 1], 0.0);
                            Transform3D aRot = new();
                            aRot.SetRotation(new Axis3D(P, myPlane.Direction()), aTrsfInPlane.RotationPart());

                            Dx = myPlane.XDirection().Transformed(aRot);
                            Dy = myPlane.YDirection().Transformed(aRot);
                        }
                        else
                        {
                            myIsApprox = true;
                        }
                    }

                    if (!myIsApprox)
                    {
                        Frame3D Axe = new(P, Dx ^ Dy, Dx);

                        if (Math.Abs(Major - Minor) < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            myType = global::Kerf.Geometry.CurveKind.Circle;
                            Circle3D Circ = new(Axe, Major);
                            GeomCirclePtr = new Geom_Circle(Circ);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                            myResult = new GeomAdaptor_Curve(GeomCirclePtr);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                        }
                        else if (Major > Minor)
                        {
                            myType = global::Kerf.Geometry.CurveKind.Ellipse;
                            Elips = new Ellipse3D(Axe, Major, Minor);

                            GeomEllipsePtr = new Geom_Ellipse(Elips);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                            myResult = new GeomAdaptor_Curve(GeomEllipsePtr);
                            //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                        }
                        else
                        {
                            myIsApprox = true;
                        }
                    }
                }

                // No way to build the canonical curve, approximate as B-spline
                if (myIsApprox)
                {
                    myType = BSplineCurve;
                    PerformApprox(myCurve, myPlane, myDirection, ref ApproxCurve);
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                    // OCCT: when the approximation fails ApproxCurve stays null and GeomAdaptor_Curve throws
                    // Standard_NullObject.
                    myResult = new GeomAdaptor_Curve(ApproxCurve!);
                    //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
                }
                else if (GeomCirclePtr is not null || GeomEllipsePtr is not null)
                {
                    Geom_Curve? aResultCurve = GeomCirclePtr;
                    if (aResultCurve is null)
                    {
                        aResultCurve = GeomEllipsePtr;
                    }
                    // start and end parameters of the projected curve
                    double aParFirst = myCurve.FirstParameter();
                    double aParLast = myCurve.LastParameter();
                    Point3D aPntFirst = ProjectPnt(myPlane, myDirection, myCurve.Value(aParFirst));
                    Point3D aPntLast = ProjectPnt(myPlane, myDirection, myCurve.Value(aParLast));
                    GeomLib_Tool.Parameter(aResultCurve, aPntFirst, global::Kerf.Engine.Foundation.Precision.Confusion(), out myFirstPar);
                    GeomLib_Tool.Parameter(aResultCurve, aPntLast, global::Kerf.Engine.Foundation.Precision.Confusion(), out myLastPar);
                    while (myLastPar <= myFirstPar)
                    {
                        myLastPar += myResult!.Period();
                    }
                }
            }
            break;
            case global::Kerf.Geometry.CurveKind.Parabola:
            {
                //     P(u) = O + (u*u)/(4*f) * Xc + u * Yc
                // ==> Q(u) = f(P(u))
                //          = f(O) + (u*u)/(4*f) * f(Xc) + u * f(Yc)

                Parabola3D Parab = myCurve.Parabola();
                Frame3D AxeRef = Parab.Position();
                Vector3D Xc = ProjectVec(myPlane, myDirection, new Vector3D(AxeRef.XDirection()));
                Vector3D Yc = ProjectVec(myPlane, myDirection, new Vector3D(AxeRef.YDirection()));
                Point3D P = ProjectPnt(myPlane, myDirection, AxeRef.Location());

                myIsApprox = false;

                if ((Math.Abs(Yc.Magnitude() - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion()) && (Xc.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    myType = global::Kerf.Geometry.CurveKind.Line;
                    aLine = new Line3D(P, new Direction3D(Yc));
                    GeomLinePtr = new Geom_Line(aLine);
                }
                else if (Xc.IsNormal(Yc, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    myType = global::Kerf.Geometry.CurveKind.Parabola;
                    // OCCT: the focal of the image is Focal * |f(Yc)|^2 / |f(Xc)|; this ignores |f(Yc)|, so the
                    // parabola is wrong unless f(Yc) stays unit (ProjLib_ProjectOnPlaneTests).
                    double F = Parab.Focal() / Xc.Magnitude();
                    Parabola3D aProjParab = new(new Frame3D(P, new Direction3D(Xc ^ Yc), new Direction3D(Xc)), F);
                    GeomParabolaPtr = new Geom_Parabola(aProjParab);
                }
                else if (Yc.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion() || Yc.IsParallel(Xc, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    myIsApprox = true;
                }
                else if (!myKeepParam)
                {
                    // Try building parabola with help of apex position
                    Geom_Curve? aParabola = null;
                    myIsApprox = !BuildParabolaByApex(ref aParabola);
                    GeomParabolaPtr = aParabola as Geom_Parabola;
                }
                else
                {
                    myIsApprox = true;
                }

                if (!myIsApprox)
                {
                    // OCCT: in the line case GeomParabolaPtr is null (the line built above is never used), and
                    // GetTrimmedResult dereferences it (a crash in OCCT; NullReferenceException in the port).
                    GetTrimmedResult(GeomParabolaPtr);
                }
                else
                {
                    BuildByApprox(aParabolaLimit);
                }
            }
            break;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
            {
                //     P(u) = O + R1 * std::cosh(u) * Xc + R2 * std::sinh(u) * Yc
                // ==> Q(u) = f(P(u))
                //          = f(O) + R1 * std::cosh(u) * f(Xc) + R2 * std::sinh(u) * f(Yc)

                Hyperbola3D Hypr = myCurve.Hyperbola();
                Frame3D AxeRef = Hypr.Position();
                Vector3D Xc = ProjectVec(myPlane, myDirection, new Vector3D(AxeRef.XDirection()));
                Vector3D Yc = ProjectVec(myPlane, myDirection, new Vector3D(AxeRef.YDirection()));
                Point3D P = ProjectPnt(myPlane, myDirection, AxeRef.Location());
                double aR1 = Hypr.MajorRadius();
                double aR2 = Hypr.MinorRadius();
                Direction3D Z = myPlane.Direction();
                myIsApprox = false;

                if (Xc.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    myType = global::Kerf.Geometry.CurveKind.Hyperbola;
                    Direction3D X = new Direction3D(Yc) ^ Z;
                    Hypr = new Hyperbola3D(new Frame3D(P, Z, X), 0.0, aR2 * Yc.Magnitude());
                    GeomHyperbolaPtr = new Geom_Hyperbola(Hypr);
                }
                else if (Yc.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    myType = global::Kerf.Geometry.CurveKind.Hyperbola;
                    Hypr = new Hyperbola3D(new Frame3D(P, Z, new Direction3D(Xc)), aR1 * Xc.Magnitude(), 0.0);
                    GeomHyperbolaPtr = new Geom_Hyperbola(Hypr);
                }
                else if (Xc.IsNormal(Yc, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    myType = global::Kerf.Geometry.CurveKind.Hyperbola;
                    Hypr = new Hyperbola3D(new Frame3D(P, new Direction3D(Xc ^ Yc), new Direction3D(Xc)),
                                       aR1 * Xc.Magnitude(),
                                       aR2 * Yc.Magnitude());
                    GeomHyperbolaPtr = new Geom_Hyperbola(Hypr);
                }
                else if (Yc.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion() || Yc.IsParallel(Xc, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    myIsApprox = true;
                }
                else if (!myKeepParam)
                {
                    Geom_Curve? aHyperbola = null;
                    myIsApprox = !BuildHyperbolaByApex(ref aHyperbola);
                    GeomHyperbolaPtr = aHyperbola as Geom_Hyperbola;
                }
                else
                {
                    myIsApprox = true;
                }
                if (!myIsApprox)
                {
                    GetTrimmedResult(GeomHyperbolaPtr);
                }
                else
                {
                    BuildByApprox(aHyperbolaLimit);
                }
            }
            break;
            case BezierCurve:
            {
                Geom_BezierCurve BezierCurvePtr = myCurve.Bezier();
                int NbPoles = BezierCurvePtr.NbPoles();

                // The poles are set on a copy: the caller's curve is unchanged, as in OCCT.
                Geom_BezierCurve ProjCu = (Geom_BezierCurve)BezierCurvePtr.Copy();

                myKeepParam = true;
                myIsApprox = false;
                myType = Type;
                for (int i = 1; i <= NbPoles; i++)
                {
                    ProjCu.SetPole(i, ProjectPnt(myPlane, myDirection, BezierCurvePtr.Pole(i)));
                }

                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                myResult = new GeomAdaptor_Curve(ProjCu);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
            }
            break;
            case BSplineCurve:
            {
                Geom_BSplineCurve BSplineCurvePtr = myCurve.BSpline();
                //
                //    make a copy of the curve and projects its poles
                //
                Geom_BSplineCurve ProjectedBSplinePtr = (Geom_BSplineCurve)BSplineCurvePtr.Copy();

                myKeepParam = true;
                myIsApprox = false;
                myType = Type;
                for (int i = 1; i <= BSplineCurvePtr.NbPoles(); i++)
                {
                    ProjectedBSplinePtr.SetPole(i, ProjectPnt(myPlane, myDirection, BSplineCurvePtr.Pole(i)));
                }

                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                myResult = new GeomAdaptor_Curve(ProjectedBSplinePtr);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
            }
            break;
            default:
            {
                myKeepParam = true;
                myIsApprox = true;
                myType = BSplineCurve;
                PerformApprox(myCurve, myPlane, myDirection, ref ApproxCurve);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:29 2002 Begin
                // OCCT: when the approximation fails ApproxCurve stays null and GeomAdaptor_Curve throws
                // Standard_NullObject.
                myResult = new GeomAdaptor_Curve(ApproxCurve!);
                //  Modified by Sergey KHROMOV - Tue Jan 29 16:57:30 2002 End
            }
            break;
        }
    }

    public CoordinateSystem GetPlane() => myPlane;

    public Direction3D GetDirection() => myDirection;

    /// <summary>The loaded curve (the caller's adaptor), null before <see cref="Load"/>.</summary>
    public Adaptor3d_Curve? GetCurve() => myCurve;

    /// <summary>
    /// The adaptor of the projected curve, null before <see cref="Load"/> (OCCT's handle: the object itself, not a
    /// copy). Its curve is frozen.
    /// </summary>
    public GeomAdaptor_Curve? GetResult()
    {
        myResult?.Curve()?.Freeze();
        return myResult;
    }

    public override double FirstParameter()
    {
        if (myKeepParam || myIsApprox)
        {
            return myCurve!.FirstParameter();
        }
        else
        {
            return myFirstPar;
        }
    }

    public override double LastParameter()
    {
        if (myKeepParam || myIsApprox)
        {
            return myCurve!.LastParameter();
        }
        else
        {
            return myLastPar;
        }
    }

    public override Continuity Continuity() => myCurve!.Continuity();

    public override int NbIntervals(Continuity S) => myCurve!.NbIntervals(S);

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        myCurve!.Intervals(T, S);
    }

    /// <exception cref="MethodNotImplementedException">The projection has no result (type OtherCurve).</exception>
    public override Adaptor3d_Curve Trim(double First, double Last, double Tolerance)
    {
        if (myType != OtherCurve)
        {
            return myResult!.Trim(First, Last, Tolerance);
        }
        else
        {
            throw new MethodNotImplementedException("ProjLib_ProjectOnPlane::Trim() - curve of unsupported type");
        }
    }

    public override bool IsClosed() => myCurve!.IsClosed();

    public override bool IsPeriodic()
    {
        if (myIsApprox)
        {
            return false;
        }
        else
        {
            return myCurve!.IsPeriodic();
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is not periodic.</exception>
    public override double Period()
    {
        if (!IsPeriodic())
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane::Period");
        }

        if (myIsApprox)
        {
            return 0.0; // OCCT: returns false (unreachable: an approximation is not periodic)
        }
        else
        {
            return myCurve!.Period();
        }
    }

    public sealed override Point3D EvalD0(double theU)
    {
        if (myType != OtherCurve)
        {
            return myResult!.EvalD0(theU);
        }
        else
        {
            return OnPlane_Value(theU, myCurve!, myPlane, myDirection);
        }
    }

    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        if (myType != OtherCurve)
        {
            return myResult!.EvalD1(theU);
        }
        else
        {
            Geom_Curve.ResD1 aRes = new();
            OnPlane_D1(theU, out aRes.Point, out aRes.D1, myCurve!, myPlane, myDirection);
            return aRes;
        }
    }

    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        if (myType != OtherCurve)
        {
            return myResult!.EvalD2(theU);
        }
        else
        {
            Geom_Curve.ResD2 aRes = new();
            OnPlane_D2(theU, out aRes.Point, out aRes.D1, out aRes.D2, myCurve!, myPlane, myDirection);
            return aRes;
        }
    }

    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        if (myType != OtherCurve)
        {
            return myResult!.EvalD3(theU);
        }
        else
        {
            Geom_Curve.ResD3 aRes = new();
            OnPlane_D3(theU, out aRes.Point, out aRes.D1, out aRes.D2, out aRes.D3, myCurve!, myPlane, myDirection);
            return aRes;
        }
    }

    public sealed override Vector3D EvalDN(double theU, int theN)
    {
        if (myType != OtherCurve)
        {
            return myResult!.EvalDN(theU, theN);
        }
        else
        {
            return OnPlane_DN(theU, theN, myCurve!, myPlane, myDirection);
        }
    }

    public override double Resolution(double Tolerance)
    {
        if (myType != OtherCurve)
        {
            return myResult!.Resolution(Tolerance);
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The type of the projected curve. Hides <see cref="object.GetType"/> (decision 98).</summary>
    public override CurveKind GetType() => myType;

    /// <exception cref="NoSuchObjectException">The projection is not a line.</exception>
    public override Line3D Line()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Line)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Line");
        }

        return myResult!.Line();
    }

    /// <exception cref="NoSuchObjectException">The projection is not a circle.</exception>
    public override Circle3D Circle()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Circle)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Circle");
        }

        return myResult!.Circle();
    }

    /// <exception cref="NoSuchObjectException">The projection is not an ellipse.</exception>
    public override Ellipse3D Ellipse()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Ellipse)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Ellipse");
        }

        return myResult!.Ellipse();
    }

    /// <exception cref="NoSuchObjectException">The projection is not a hyperbola.</exception>
    public override Hyperbola3D Hyperbola()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Hyperbola");
        }

        return myResult!.Hyperbola();
    }

    /// <exception cref="NoSuchObjectException">The projection is not a parabola.</exception>
    public override Parabola3D Parabola()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Parabola)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Parabola");
        }

        return myResult!.Parabola();
    }

    /// <exception cref="NoSuchObjectException">The projection is neither a Bezier nor a B-spline curve.</exception>
    public override int Degree()
    {
        if ((GetType() != BSplineCurve) && (GetType() != BezierCurve))
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Degree");
        }

        if (myIsApprox)
        {
            return myResult!.Degree();
        }
        else
        {
            return myCurve!.Degree();
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is neither a Bezier nor a B-spline curve.</exception>
    public override bool IsRational()
    {
        if ((GetType() != BSplineCurve) && (GetType() != BezierCurve))
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:IsRational");
        }

        if (myIsApprox)
        {
            return myResult!.IsRational();
        }
        else
        {
            return myCurve!.IsRational();
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is neither a Bezier nor a B-spline curve.</exception>
    public override int NbPoles()
    {
        if ((GetType() != BSplineCurve) && (GetType() != BezierCurve))
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:NbPoles");
        }

        if (myIsApprox)
        {
            return myResult!.NbPoles();
        }
        else
        {
            return myCurve!.NbPoles();
        }
    }

    /// <exception cref="NoSuchObjectException">The projection is not a B-spline curve.</exception>
    public override int NbKnots()
    {
        if (GetType() != BSplineCurve)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:NbKnots");
        }

        if (myIsApprox)
        {
            return myResult!.NbKnots();
        }
        else
        {
            return myCurve!.NbKnots();
        }
    }

    /// <summary>The projected Bezier curve, frozen; the same object at each call.</summary>
    /// <exception cref="NoSuchObjectException">The projection is not a Bezier curve.</exception>
    public override Geom_BezierCurve Bezier()
    {
        Geom_BezierCurve aBezier = InternalBezier();
        aBezier.Freeze();
        return aBezier;
    }

    /// <summary>The projected B-spline curve, frozen; the same object at each call.</summary>
    /// <exception cref="NoSuchObjectException">The projection is not a B-spline curve.</exception>
    public override Geom_BSplineCurve BSpline()
    {
        Geom_BSplineCurve aBSpline = InternalBSpline();
        aBSpline.Freeze();
        return aBSpline;
    }

    /// <summary>OCCT's <c>Bezier()</c> without freezing, for ported code that discards the projector.</summary>
    internal Geom_BezierCurve InternalBezier()
    {
        if (myType != BezierCurve)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:Bezier");
        }

        return myResult!.Bezier();
    }

    /// <summary>OCCT's <c>BSpline()</c> without freezing, for ported code that discards the projector.</summary>
    internal Geom_BSplineCurve InternalBSpline()
    {
        if (myType != BSplineCurve)
        {
            throw new NoSuchObjectException("ProjLib_ProjectOnPlane:BSpline");
        }

        return myResult!.BSpline();
    }

    protected void GetTrimmedResult(Geom_Curve? theProjCurve)
    {
        Line3D aLin = new();
        Parabola3D aParab = new();
        Hyperbola3D aHypr = new();
        if (myType == global::Kerf.Geometry.CurveKind.Line)
        {
            aLin = ((Geom_Line)theProjCurve!).Lin();
        }
        else if (myType == global::Kerf.Geometry.CurveKind.Parabola)
        {
            aParab = ((Geom_Parabola)theProjCurve!).Parab();
        }
        else if (myType == global::Kerf.Geometry.CurveKind.Hyperbola)
        {
            aHypr = ((Geom_Hyperbola)theProjCurve!).Hypr();
        }

        myFirstPar = theProjCurve!.FirstParameter();
        myLastPar = theProjCurve.LastParameter();
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(myCurve!.FirstParameter()))
        {
            Point3D aP = myCurve.Value(myCurve.FirstParameter());
            aP = ProjectPnt(myPlane, myDirection, aP);
            if (myType == global::Kerf.Geometry.CurveKind.Line)
            {
                myFirstPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin, aP);
            }
            else if (myType == global::Kerf.Geometry.CurveKind.Parabola)
            {
                myFirstPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aParab, aP);
            }
            else if (myType == global::Kerf.Geometry.CurveKind.Hyperbola)
            {
                myFirstPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aHypr, aP);
            }
            else
            {
                GeomLib_Tool.Parameter(theProjCurve, aP, global::Kerf.Engine.Foundation.Precision.Confusion(), out myFirstPar);
            }
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(myCurve.LastParameter()))
        {
            Point3D aP = myCurve.Value(myCurve.LastParameter());
            aP = ProjectPnt(myPlane, myDirection, aP);
            if (myType == global::Kerf.Geometry.CurveKind.Line)
            {
                myLastPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin, aP);
            }
            else if (myType == global::Kerf.Geometry.CurveKind.Parabola)
            {
                myLastPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aParab, aP);
            }
            else if (myType == global::Kerf.Geometry.CurveKind.Hyperbola)
            {
                myLastPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aHypr, aP);
            }
            else
            {
                GeomLib_Tool.Parameter(theProjCurve, aP, global::Kerf.Engine.Foundation.Precision.Confusion(), out myLastPar);
            }
        }
        myResult = new GeomAdaptor_Curve(theProjCurve, myFirstPar, myLastPar);
    }

    /// <summary>Builds the projected parabola from its apex, found as the point of maximal curvature.</summary>
    protected bool BuildParabolaByApex(ref Geom_Curve? theGeomParabolaPtr)
    {
        //
        // Searching parabola apex as point with maximal curvature
        double aF = myCurve!.Parabola().Focal();
        CurveKind aCurType = myType;
        myType = OtherCurve; // To provide correct calculation of derivativesb by projection for
                                     // copy of instance;
        Adaptor3d_Curve aProjCrv = ShallowCopy();
        myType = aCurType;
        LProp_CLProps3d aProps = new(aProjCrv, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        ProjLib_MaxCurvature aMaxCur = new(aProps);
        BrentMinimum aSolver = new(global::Kerf.Engine.Foundation.Precision.PConfusion());
        aSolver.Perform(aMaxCur, -10.0 * aF, 0.0, 10.0 * aF);

        if (!aSolver.IsDone)
        {
            return false;
        }

        double aT;
        aT = aSolver.Location();
        aProps.SetParameter(aT);
        Point3D aP0 = aProps.Value();
        Vector3D aDY = aProps.D1();
        Direction3D anYDir = new(aDY);
        Direction3D anXDir;
        double aCurv = aProps.Curvature();
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurv) || aCurv < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }
        aProps.Normal(out anXDir);
        //
        Line3D anXLine = new(aP0, anXDir);
        Point3D aP1 = Value(aT + 10.0 * aF);
        //
        double anX = global::Kerf.Engine.Geometry.ElCLib.LineParameter(anXLine.Position(), aP1);
        double anY = anXLine.Distance(aP1);
        double aNewF = anY * anY / 4.0 / anX;
        Direction3D anN = anXDir ^ anYDir;
        Frame3D anA2 = new(aP0, anN, anXDir);
        gce_MakeParab aMkParab = new(anA2, aNewF);
        if (!aMkParab.IsDone())
        {
            return false;
        }

        Parabola3D aProjParab = aMkParab.Value();

        myType = global::Kerf.Geometry.CurveKind.Parabola;
        theGeomParabolaPtr = new Geom_Parabola(aProjParab);
        // GetTrimmedResult(theGeomParabolaPtr);

        return true;
    }

    /// <summary>Builds the projected hyperbola from its apex, found as the point of maximal curvature.</summary>
    protected bool BuildHyperbolaByApex(ref Geom_Curve? theGeomHyperbolaPtr)
    {
        // Try to build hyperbola with help of apex position
        CurveKind aCurType = myType;
        myType = OtherCurve; // To provide correct calculation of derivativesb by projection for
                                     // copy of instance;
        Adaptor3d_Curve aProjCrv = ShallowCopy();
        myType = aCurType;
        // Searching hyperbola apex as point with maximal curvature
        LProp_CLProps3d aProps = new(aProjCrv, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        ProjLib_MaxCurvature aMaxCur = new(aProps);
        BrentMinimum aSolver = new(global::Kerf.Engine.Foundation.Precision.PConfusion());
        aSolver.Perform(aMaxCur, -5.0, 0.0, 5.0);

        if (aSolver.IsDone)
        {
            double aT;
            aT = aSolver.Location();
            aProps.SetParameter(aT);
            double aCurv = aProps.Curvature();
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurv) || aCurv < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
            else
            {
                Hyperbola3D Hypr = myCurve!.Hyperbola();
                Frame3D AxeRef = Hypr.Position();
                Point3D P = ProjectPnt(myPlane, myDirection, AxeRef.Location());
                Direction3D Z = myPlane.Direction();
                Point3D aP0 = aProps.Value();
                Direction3D anXDir = new gce_MakeDir(P, aP0).Value();
                Direction3D anYDir = new gce_MakeDir(aProps.D1()).Value();
                //
                double aMajRad = P.Distance(aP0);
                Point3D aP1 = Value(aT + 1.0);
                Vector3D aV = new(P, aP1);
                double anX = aV * new Vector3D(anXDir);
                double anY = aV * new Vector3D(anYDir);
                double aMinRad = anY / Math.Sqrt(anX * anX / aMajRad / aMajRad - 1.0);
                Frame3D anA2 = new(P, Z, anXDir);
                Hyperbola3D anHypr = new(anA2, aMajRad, aMinRad);
                theGeomHyperbolaPtr = new Geom_Hyperbola(anHypr);
                myType = global::Kerf.Geometry.CurveKind.Hyperbola;
            }
        }
        else
        {
            return false;
        }
        return true;
    }

    /// <summary>Approximates the projection by a B-spline, bounding an infinite curve by
    /// <paramref name="theLimitParameter"/>.</summary>
    protected void BuildByApprox(double theLimitParameter)
    {
        myType = BSplineCurve;
        Geom_BSplineCurve? anApproxCurve = null;
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myCurve!.FirstParameter()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(myCurve.LastParameter()))
        {
            // To avoid exception in approximation
            double f = Max(-theLimitParameter, myCurve.FirstParameter());
            double l = Min(theLimitParameter, myCurve.LastParameter());
            Adaptor3d_Curve aTrimCurve = myCurve.Trim(f, l, global::Kerf.Engine.Foundation.Precision.Confusion());
            PerformApprox(aTrimCurve, myPlane, myDirection, ref anApproxCurve);
        }
        else
        {
            PerformApprox(myCurve, myPlane, myDirection, ref anApproxCurve);
        }
        // OCCT: a failed approximation leaves anApproxCurve null and dereferences it (NullReferenceException in
        // the port).
        myFirstPar = anApproxCurve!.FirstParameter();
        myLastPar = anApproxCurve.LastParameter();
        myResult = new GeomAdaptor_Curve(anApproxCurve);
    }
}
