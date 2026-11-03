// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfLinearExtrusion.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfLinearExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The surface of linear extrusion of an adapted curve along a direction: U is the curve's parameter, V the
/// distance along the direction (unbounded). <see cref="GetType"/> recognises planes and cylinders.
/// </summary>
/// <remarks>Evaluation is the base adaptor's (Geom_ExtrusionUtils over the curve adaptor).</remarks>
public class GeomAdaptor_SurfaceOfLinearExtrusion : GeomAdaptor_Surface
{
    private Adaptor3d_Curve? myBasisCurve; // extruded curve
    private Direction3D myDirection; // direction of extrusion
    private bool myHaveDir; // whether the direction of extrusion is initialized

    /// <summary>An adaptor without a curve or direction.</summary>
    public GeomAdaptor_SurfaceOfLinearExtrusion()
    {
        myDirection = new Direction3D();
        myHaveDir = false;
    }

    /// <summary>The extrusion of C, the direction to be loaded.</summary>
    public GeomAdaptor_SurfaceOfLinearExtrusion(Adaptor3d_Curve C)
    {
        myDirection = new Direction3D();
        myHaveDir = false;
        Load(C);
    }

    /// <summary>The extrusion of C along V.</summary>
    public GeomAdaptor_SurfaceOfLinearExtrusion(Adaptor3d_Curve C, in Direction3D V)
    {
        myDirection = new Direction3D();
        myHaveDir = false;
        Load(C);
        Load(V);
    }

    /// <summary>A shallow copy: shallow copies of the curve adaptors.</summary>
    public override Adaptor3d_Surface ShallowCopy()
    {
        GeomAdaptor_SurfaceOfLinearExtrusion aCopy = new GeomAdaptor_SurfaceOfLinearExtrusion();

        if (myBasisCurve is not null)
        {
            aCopy.myBasisCurve = myBasisCurve.ShallowCopy();
        }
        aCopy.myDirection = myDirection;
        aCopy.myHaveDir = myHaveDir;

        aCopy.mySurface = mySurface;
        aCopy.myUFirst = myUFirst;
        aCopy.myULast = myULast;
        aCopy.myVFirst = myVFirst;
        aCopy.myVLast = myVLast;
        aCopy.myTolU = myTolU;
        aCopy.myTolV = myTolV;
        aCopy.mySurfaceType = mySurfaceType;

        // Copy surface data variant
        if (mySurfaceData is ExtrusionData anExtData)
        {
            ExtrusionData aNewData = new ExtrusionData();
            aNewData.BasisCurve = anExtData.BasisCurve.ShallowCopy();
            aNewData.Direction = anExtData.Direction;
            aCopy.mySurfaceData = aNewData;
        }

        return aCopy;
    }

    /// <summary>Changes the extruded curve.</summary>
    public void Load(Adaptor3d_Curve C)
    {
        myBasisCurve = C;
        if (myHaveDir)
        {
            Load(myDirection);
        }
    }

    /// <summary>Changes the direction of extrusion (the curve must be loaded).</summary>
    public void Load(in Direction3D V)
    {
        myHaveDir = true;
        myDirection = V;

        mySurfaceType = SurfaceOfExtrusion;

        // Populate extrusion surface data for fast evaluation
        ExtrusionData anExtData = new ExtrusionData();
        anExtData.BasisCurve = myBasisCurve!;
        anExtData.Direction = myDirection.XYZ();
        mySurfaceData = anExtData;
    }

    public override double FirstUParameter() => myBasisCurve!.FirstParameter();

    public override double LastUParameter() => myBasisCurve!.LastParameter();

    public override double FirstVParameter() => RealFirst();

    public override double LastVParameter() => RealLast();

    public override Continuity UContinuity() => myBasisCurve!.Continuity();

    public override Continuity VContinuity() => CN;

    public override int NbUIntervals(Continuity S) => myBasisCurve!.NbIntervals(S);

    public override int NbVIntervals(Continuity S) => 1;

    public override void UIntervals(NCollection_Array1<double> T, Continuity S) => myBasisCurve!.Intervals(T, S);

    public override void VIntervals(NCollection_Array1<double> T, Continuity S)
    {
        T[T.Lower()] = FirstVParameter();
        T[T.Lower() + 1] = LastVParameter();
    }

    /// <summary>
    /// The extrusion of the curve trimmed between First and Last: OCCT trims the curve, which is the U direction
    /// (occt-issues.md).
    /// </summary>
    public override Adaptor3d_Surface VTrim(double First, double Last, double Tol)
    {
        // OCCT: VTrim trims the basis curve, which carries U, and UTrim (below) trims nothing.
        Adaptor3d_Curve HC = BasisCurve().Trim(First, Last, Tol);
        GeomAdaptor_SurfaceOfLinearExtrusion HR = new GeomAdaptor_SurfaceOfLinearExtrusion(HC, myDirection);
        return HR;
    }

    /// <summary>The same extrusion, untrimmed (OCCT ignores the parameters: occt-issues.md).</summary>
    public override Adaptor3d_Surface UTrim(double First, double Last, double Tol)
    {
        GeomAdaptor_SurfaceOfLinearExtrusion HR = new GeomAdaptor_SurfaceOfLinearExtrusion(myBasisCurve!, myDirection);
        return HR;
    }

    public override bool IsUClosed() => myBasisCurve!.IsClosed();

    public override bool IsVClosed() => false;

    public override bool IsUPeriodic() => myBasisCurve!.IsPeriodic();

    public override double UPeriod() => myBasisCurve!.Period();

    public override bool IsVPeriodic() => false;

    /// <exception cref="DomainErrorException">Always (not periodic in V).</exception>
    public override double VPeriod() => throw new DomainErrorException("GeomAdaptor_SurfaceOfLinearExtrusion::VPeriod");

    public override double UResolution(double R3d) => myBasisCurve!.Resolution(R3d);

    public override double VResolution(double R3d) => R3d;

    /// <summary>
    /// GeomAbs_Plane for an extruded line not parallel to the direction, or a conic whose plane contains it;
    /// GeomAbs_Cylinder for a circle extruded along its axis; GeomAbs_SurfaceOfExtrusion otherwise.
    /// </summary>
    public override SurfaceKind GetType()
    {
        switch (myBasisCurve!.GetType())
        {
            case Line:
            {
                Direction3D D = myBasisCurve.Line().Direction();
                if (!myDirection.IsParallel(D, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Circle:
            {
                Direction3D D = (myBasisCurve.Circle()).Axis().Direction();
                if (myDirection.IsParallel(D, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Cylinder;
                }
                else if (myDirection.IsNormal(D, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Ellipse:
            {
                Direction3D D = (myBasisCurve.Ellipse()).Axis().Direction();
                if (myDirection.IsNormal(D, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Parabola:
            {
                Direction3D D = (myBasisCurve.Parabola()).Axis().Direction();
                if (myDirection.IsNormal(D, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            case Hyperbola:
            {
                Direction3D D = (myBasisCurve.Hyperbola()).Axis().Direction();
                if (myDirection.IsNormal(D, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return global::Kerf.Geometry.SurfaceKind.Plane;
                }
                break;
            }

            default:
                break;
        }

        return SurfaceOfExtrusion;
    }

    /// <exception cref="NoSuchObjectException">The surface is not a plane.</exception>
    public override Plane3D Plane()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Plane, "GeomAdaptor_SurfaceOfLinearExtrusion::Plane");

        Point3D P = new Point3D();
        Vector3D D1u = new Vector3D(), newZ = new Vector3D();
        double UFirst = myBasisCurve!.FirstParameter();
        double ULast = myBasisCurve.LastParameter();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(UFirst) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(ULast))
        {
            UFirst = -100.0;
            ULast = 100.0;
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(UFirst))
        {
            UFirst = ULast - 200.0;
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(ULast))
        {
            ULast = UFirst + 200.0;
        }
        double deltau = (ULast - UFirst) / 20.0;
        for (int i = 1; i <= 21; i++)
        {
            double prm = UFirst + (i - 1) * deltau;
            myBasisCurve.D1(prm, out P, out D1u);
            newZ = D1u.Normalized().Crossed(new Vector3D(myDirection));
            if (newZ.Magnitude() > 1.0e-12)
            {
                break;
            }
        }
        CoordinateSystem Ax3 = new CoordinateSystem(P, new Direction3D(newZ), new Direction3D(D1u));
        if (myDirection.Dot(Ax3.YDirection()) < 0.0)
        {
            Ax3.YReverse();
        }
        return new Plane3D(Ax3);
    }

    /// <exception cref="NoSuchObjectException">The surface is not a cylinder.</exception>
    public override Cylinder Cylinder()
    {
        Standard_NoSuchObject_Raise_if(GetType() != global::Kerf.Geometry.SurfaceKind.Cylinder, "GeomAdaptor_SurfaceOfLinearExtrusion::Cylinder");

        Circle3D C = myBasisCurve!.Circle();
        CoordinateSystem Ax3 = new CoordinateSystem(C.Position());
        if (myDirection.Dot((C.Axis()).Direction()) < 0.0)
        {
            Ax3.ZReverse();
        }
        return new Cylinder(Ax3, C.Radius());
    }

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Cone Cone() => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Cone");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Sphere Sphere() => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Sphere");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Torus Torus() => throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Torus");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Axis3D AxeOfRevolution() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Axes");

    public override int UDegree() => myBasisCurve!.Degree();

    public override int NbUPoles() => myBasisCurve!.NbPoles();

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsURational() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::IsURational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override bool IsVRational() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::IsVRational");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Geom_BezierSurface Bezier() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::Bezier");

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Geom_BSplineSurface BSpline() =>
        throw new NoSuchObjectException("GeomAdaptor_SurfaceOfLinearExtrusion::BSpline");

    public override Direction3D Direction() => myDirection;

    /// <summary>The extruded curve (the adaptor given to Load).</summary>
    public override Adaptor3d_Curve BasisCurve() => myBasisCurve!;
}
