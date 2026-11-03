// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The surface of revolution of an adapted curve about an axis: U in [0, 2*PI] is the angle, V the curve's
/// parameter. <see cref="GetType"/> recognises planes, cylinders, cones, spheres and tori.
/// </summary>
/// <remarks>Evaluation is the base adaptor's (Geom_RevolutionUtils over the curve adaptor).</remarks>
public class GeomAdaptor_SurfaceOfRevolution : GeomAdaptor_Surface
{
    private Adaptor3d_Curve? myBasisCurve; // revolved curve
    private Axis3D myAxis; // axis of revolution
    private bool myHaveAxis; // whether axis of revolution is initialized
    private CoordinateSystem myAxeRev; // auxiliary trihedron according to the curve position

    /// <summary>An adaptor without a curve or axis.</summary>
    public GeomAdaptor_SurfaceOfRevolution()
    {
        myHaveAxis = false;
        myAxeRev = new CoordinateSystem();
    }

    /// <summary>The revolution of C, the axis to be loaded.</summary>
    public GeomAdaptor_SurfaceOfRevolution(Adaptor3d_Curve C)
    {
        myHaveAxis = false;
        myAxeRev = new CoordinateSystem();
        Load(C);
    }

    /// <summary>The revolution of C about V.</summary>
    /// <exception cref="ConstructionErrorException">The axis and the meridian are confused.</exception>
    public GeomAdaptor_SurfaceOfRevolution(Adaptor3d_Curve C, in Axis3D V)
    {
        myHaveAxis = false;
        myAxeRev = new CoordinateSystem();
        Load(C);
        Load(V);
    }

    /// <summary>A shallow copy: shallow copies of the curve adaptors.</summary>
    public override Adaptor3d_Surface ShallowCopy()
    {
        GeomAdaptor_SurfaceOfRevolution aCopy = new GeomAdaptor_SurfaceOfRevolution();

        if (myBasisCurve is not null)
        {
            aCopy.myBasisCurve = myBasisCurve.ShallowCopy();
        }
        aCopy.myAxis = myAxis;
        aCopy.myHaveAxis = myHaveAxis;
        aCopy.myAxeRev = myAxeRev;

        aCopy.mySurface = mySurface;
        aCopy.myUFirst = myUFirst;
        aCopy.myULast = myULast;
        aCopy.myVFirst = myVFirst;
        aCopy.myVLast = myVLast;
        aCopy.myTolU = myTolU;
        aCopy.myTolV = myTolV;
        aCopy.mySurfaceType = mySurfaceType;

        // Copy surface data variant
        if (mySurfaceData is RevolutionData aRevData)
        {
            RevolutionData aNewData = new RevolutionData();
            aNewData.BasisCurve = aRevData.BasisCurve.ShallowCopy();
            aNewData.Axis = aRevData.Axis;
            aCopy.mySurfaceData = aNewData;
        }

        return aCopy;
    }

    /// <summary>Changes the revolved curve (and recomputes the trihedron if the axis is loaded).</summary>
    public void Load(Adaptor3d_Curve C)
    {
        myBasisCurve = C;
        if (myHaveAxis)
        {
            Load(myAxis); // to evaluate the new myAxeRev.
        }
    }

    /// <summary>Changes the axis of revolution (the curve must be loaded).</summary>
    /// <exception cref="ConstructionErrorException">The axis and the meridian are confused.</exception>
    public void Load(in Axis3D V)
    {
        myHaveAxis = true;
        myAxis = V;

        mySurfaceType = SurfaceOfRevolution;

        // Populate revolution surface data for fast evaluation
        RevolutionData aRevData = new RevolutionData();
        aRevData.BasisCurve = myBasisCurve!;
        aRevData.Axis = myAxis;
        mySurfaceData = aRevData;

        // Eval myAxeRev : axe of revolution ( Determination de Ox).
        Point3D P, Q;
        Point3D O = myAxis.Location();
        Direction3D Ox;
        Direction3D Oz = myAxis.Direction();
        bool yrev = false;
        if (myBasisCurve!.GetType() == Line)
        {
            if ((myBasisCurve.Line().Direction()).Dot(Oz) < 0.0)
            {
                yrev = true;
                Oz.Reverse();
            }
        }

        if (myBasisCurve.GetType() == Circle)
        {
            Q = P = (myBasisCurve.Circle()).Location();
        }
        else
        {
            double First = myBasisCurve.FirstParameter();
            P = Value(0.0, 0.0); // which does not mean much
            if (GetType() == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                if (new Line3D(myAxis).Distance(P) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    Q = global::Kerf.Engine.Geometry.ElCLib.Value(1.0, myBasisCurve.Line());
                }
                else
                {
                    Q = P;
                }
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(First))
            {
                Q = P;
            }
            else
            {
                Q = Value(0.0, First);
            }
        }

        Direction3D DZ = myAxis.Direction();
        O.SetXYZ(O.XYZ() + (new Vector3D(O, P) * new Vector3D(DZ)) * DZ.XYZ());
        if (new Line3D(myAxis).Distance(Q) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Ox = new Direction3D(Q.XYZ() - O.XYZ());
        }
        else
        {
            double First = myBasisCurve.FirstParameter();
            double Last = myBasisCurve.LastParameter();
            int Ratio = 1;
            double Dist;
            Point3D PP;
            do
            {
                PP = myBasisCurve.Value(First + (Last - First) / Ratio);
                Dist = new Line3D(myAxis).Distance(PP);
                Ratio++;
            } while (Dist < global::Kerf.Engine.Foundation.Precision.Confusion() && Ratio < 100);

            if (Ratio >= 100)
            {
                throw new ConstructionErrorException("Adaptor3d_SurfaceOfRevolution : Axe and meridian are confused");
            }
            // C++: (Oz ^ gp_Vec(...)) ^ Oz, with the vector converted to a gp_Dir (normalised), and gp_Dir's
            // operator^ normalising each product.
            Ox = (Oz ^ new Direction3D(new Vector3D(PP.XYZ() - O.XYZ()))) ^ Oz;
        }

        myAxeRev = new CoordinateSystem(O, Oz, Ox);

        if (yrev)
        {
            myAxeRev.YReverse();
        }
        else if (myBasisCurve.GetType() == Circle)
        {
            Direction3D DC = (myBasisCurve.Circle()).Axis().Direction();
            if ((Ox.Crossed(Oz)).Dot(DC) < 0.0)
            {
                myAxeRev.ZReverse();
            }
        }
    }

    public override Axis3D AxeOfRevolution() => myAxis;

    public override double FirstUParameter() => 0.0;

    public override double LastUParameter() => 2 * Math.PI;

    public override double FirstVParameter() => myBasisCurve!.FirstParameter();

    public override double LastVParameter() => myBasisCurve!.LastParameter();

    public override Continuity UContinuity() => CN;

    public override Continuity VContinuity() => myBasisCurve!.Continuity();

    public override int NbUIntervals(Continuity S) => 1;

    public override int NbVIntervals(Continuity S) => myBasisCurve!.NbIntervals(S);

    public override void UIntervals(NCollection_Array1<double> T, Continuity S)
    {
        T[T.Lower()] = 0.0;
        T[T.Lower() + 1] = 2 * Math.PI;
    }

    public override void VIntervals(NCollection_Array1<double> T, Continuity S) => myBasisCurve!.Intervals(T, S);

    /// <summary>The whole surface again: only [0, 2*PI] is accepted.</summary>
    /// <exception cref="OutOfRangeException">First or Last differ from 0 or 2*PI by more than Precision::PConfusion().</exception>
    public override Adaptor3d_Surface UTrim(double First, double Last, double Tol)
    {
        double Eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Standard_OutOfRange_Raise_if(Math.Abs(First) > Eps || Math.Abs(Last - 2.0 * Math.PI) > Eps,
                                     "GeomAdaptor_SurfaceOfRevolution : UTrim : Parameters out of range");

        GeomAdaptor_SurfaceOfRevolution HR = new GeomAdaptor_SurfaceOfRevolution(myBasisCurve!, myAxis);
        return HR;
    }

    /// <summary>The revolution of the curve trimmed between First and Last.</summary>
    public override Adaptor3d_Surface VTrim(double First, double Last, double Tol)
    {
        Adaptor3d_Curve HC = BasisCurve().Trim(First, Last, Tol);
        GeomAdaptor_SurfaceOfRevolution HR = new GeomAdaptor_SurfaceOfRevolution(HC, myAxis);
        return HR;
    }

    public override bool IsUClosed() => true;

    public override bool IsVClosed() => myBasisCurve!.IsClosed();

    public override bool IsUPeriodic() => true;

    public override double UPeriod() => 2 * Math.PI;

    public override bool IsVPeriodic() => myBasisCurve!.IsPeriodic();

    public override double VPeriod() => myBasisCurve!.Period();

    public override double UResolution(double R3d) => global::Kerf.Engine.Foundation.Precision.Parametric(R3d);

    public override double VResolution(double R3d) => myBasisCurve!.Resolution(R3d);

    /// <summary>
    /// GeomAbs_Plane, Cylinder or Cone for a line (normal to, parallel to or crossing the axis), GeomAbs_Sphere or
    /// Torus for a circle coplanar with the axis; GeomAbs_SurfaceOfRevolution otherwise.
    /// </summary>
    public override SurfaceKind GetType()
    {
        double TolConf = global::Kerf.Engine.Foundation.Precision.Confusion();
        double TolAng = global::Kerf.Engine.Foundation.Precision.Angular();
        double TolConeSemiAng = global::Kerf.Engine.Foundation.Precision.Confusion();

        switch (myBasisCurve!.GetType())
        {
            case Line:
            {
                Axis3D Axe = myBasisCurve.Line().Position();

                if (myAxis.IsParallel(Axe, TolAng))
                {
                    Point3D P = Value(0.0, 0.0);
                    double R = new Vector3D(myAxeRev.Location(), P) * new Vector3D(myAxeRev.XDirection());
                    if (R > TolConf)
                    {
                        return global::Kerf.Geometry.SurfaceKind.Cylinder;
                    }
                }
                else if (myAxis.IsNormal(Axe, TolAng))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                else
                {
                    double uf = myBasisCurve.FirstParameter();
                    double ul = myBasisCurve.LastParameter();
                    bool istrim = (!global::Kerf.Engine.Foundation.Precision.IsInfinite(uf) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(ul));
                    if (istrim)
                    {
                        Point3D pf = myBasisCurve.Value(uf);
                        Point3D pl = myBasisCurve.Value(ul);
                        double len = pf.Distance(pl);
                        // Compute the distance projected onto the axis.
                        Vector3D vlin = new Vector3D(pf, pl);
                        Vector3D vaxe = new Vector3D(myAxis.Direction());
                        double projlen = Math.Abs(vaxe.Dot(vlin));
                        if ((len - projlen) <= TolConf)
                        {
                            Point3D P = Value(0.0, 0.0);
                            double R = new Vector3D(myAxeRev.Location(), P) * new Vector3D(myAxeRev.XDirection());
                            if (R > TolConf)
                            {
                                return global::Kerf.Geometry.SurfaceKind.Cylinder;
                            }
                        }
                        else if (projlen <= TolConf)
                        {
                            return global::Kerf.Geometry.SurfaceKind.Plane;
                        }
                    }
                    Vector3D V = new Vector3D(myAxis.Location(), myBasisCurve.Line().Location());
                    Vector3D W = new Vector3D(Axe.Direction());
                    Vector3D AxisDir = new Vector3D(myAxis.Direction());
                    double proj = Math.Abs(W.Dot(AxisDir));
                    if (Math.Abs(V.DotCross(AxisDir, W)) <= TolConf
                        && (proj >= TolConeSemiAng && proj <= 1.0 - TolConeSemiAng))
                    {
                        return global::Kerf.Geometry.SurfaceKind.Cone;
                    }
                }
                break;
            } // case GeomAbs_Line:
            //
            case Circle:
            {
                double MajorRadius, aR;
                Line3D aLin = new Line3D(myAxis);
                //
                Circle3D C = myBasisCurve.Circle();
                Point3D aLC = C.Location();
                aR = C.Radius();
                //

                if (!C.Position().IsCoplanar(myAxis, TolConf, TolAng))
                {
                    return SurfaceOfRevolution;
                }
                else if (aLin.Distance(aLC) <= TolConf)
                {
                    return global::Kerf.Geometry.SurfaceKind.Sphere;
                }
                else
                {
                    MajorRadius = aLin.Distance(aLC);
                    if (MajorRadius > aR)
                    {
                        return global::Kerf.Geometry.SurfaceKind.Torus;
                    }
                }
                break;
            }
            //
            default:
                break;
        }

        return SurfaceOfRevolution;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a plane.</exception>
    public override Plane3D Plane()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Plane, "GeomAdaptor_SurfaceOfRevolution:Plane");

        CoordinateSystem Axe = myAxeRev;
        Point3D aPonCurve = Value(0.0, 0.0);
        double aDot = (aPonCurve.XYZ() - myAxis.Location().XYZ()).Dot(myAxis.Direction().XYZ());

        Point3D P = new Point3D(myAxis.Location().XYZ() + aDot * myAxis.Direction().XYZ());
        Axe.SetLocation(P);
        if (Axe.XDirection().Dot(myBasisCurve!.Line().Direction()) >= -global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Axe.XReverse();
        }

        return new Plane3D(Axe);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cylinder.</exception>
    public override Cylinder Cylinder()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Cylinder, "GeomAdaptor_SurfaceOfRevolution::Cylinder");

        Point3D P = Value(0.0, 0.0);
        double R = new Vector3D(myAxeRev.Location(), P) * new Vector3D(myAxeRev.XDirection());
        return new Cylinder(myAxeRev, R);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cone.</exception>
    public override Cone Cone()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Cone, "GeomAdaptor_SurfaceOfRevolution:Cone");

        CoordinateSystem Axe = myAxeRev;
        Direction3D ldir = (myBasisCurve!.Line()).Direction();
        double Angle = (Axe.Direction()).Angle(ldir);
        Point3D P0 = Value(0.0, 0.0);
        double R = (Axe.Location()).Distance(P0);
        if (R >= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Point3D O = Axe.Location();
            Vector3D OP0 = new Vector3D(O, P0);
            double t = OP0.Dot(new Vector3D(Axe.XDirection()));
            t /= ldir.Dot(Axe.XDirection());
            OP0.Add(-t * new Vector3D(ldir));
            if (OP0.Dot(new Vector3D(Axe.Direction())) > 0.0)
            {
                Angle = -Angle;
            }
        }
        return new Cone(Axe, Angle, R);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a sphere.</exception>
    public override Sphere Sphere()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Sphere, "GeomAdaptor_SurfaceOfRevolution:Sphere");

        Circle3D C = myBasisCurve!.Circle();
        CoordinateSystem Axe = myAxeRev;
        Axe.SetLocation(C.Location());
        return new Sphere(Axe, C.Radius());
    }

    /// <exception cref="NoSuchObjectException">The surface is not a torus.</exception>
    public override Torus Torus()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Torus, "GeomAdaptor_SurfaceOfRevolution:Torus");

        Circle3D C = myBasisCurve!.Circle();
        double MajorRadius = new Line3D(myAxis).Distance(C.Location());
        return new Torus(myAxeRev, MajorRadius, C.Radius());
    }

    public override int VDegree() => myBasisCurve!.Degree();

    public override int NbVPoles() => myBasisCurve!.NbPoles();

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override int NbVKnots() => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::NbVKnots");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsURational() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::IsURational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsVRational() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::IsVRational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Geom_BezierSurface Bezier() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::Bezier");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Geom_BSplineSurface BSpline() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfRevolution::BSpline");

    /// <summary>The trihedron of the revolution: the axis, with X towards the curve.</summary>
    public CoordinateSystem Axis() => myAxeRev;

    /// <summary>The revolved curve (the adaptor given to Load).</summary>
    public override Adaptor3d_Curve BasisCurve() => myBasisCurve!;
}
