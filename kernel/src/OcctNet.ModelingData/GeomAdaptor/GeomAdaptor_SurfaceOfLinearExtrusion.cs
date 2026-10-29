// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfLinearExtrusion.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfLinearExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The surface of linear extrusion of an adapted curve along a direction: U is the curve's parameter, V the
/// distance along the direction (unbounded). <see cref="GetType"/> recognises planes and cylinders.
/// </summary>
/// <remarks>Evaluation is the base adaptor's (Geom_ExtrusionUtils over the curve adaptor).</remarks>
public class GeomAdaptor_SurfaceOfLinearExtrusion : GeomAdaptor_Surface
{
    private Adaptor3d_Curve? myBasisCurve; // extruded curve
    private gp_Dir myDirection; // direction of extrusion
    private bool myHaveDir; // whether the direction of extrusion is initialized

    /// <summary>An adaptor without a curve or direction.</summary>
    public GeomAdaptor_SurfaceOfLinearExtrusion()
    {
        myDirection = new gp_Dir();
        myHaveDir = false;
    }

    /// <summary>The extrusion of C, the direction to be loaded.</summary>
    public GeomAdaptor_SurfaceOfLinearExtrusion(Adaptor3d_Curve C)
    {
        myDirection = new gp_Dir();
        myHaveDir = false;
        Load(C);
    }

    /// <summary>The extrusion of C along V.</summary>
    public GeomAdaptor_SurfaceOfLinearExtrusion(Adaptor3d_Curve C, in gp_Dir V)
    {
        myDirection = new gp_Dir();
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
    public void Load(in gp_Dir V)
    {
        myHaveDir = true;
        myDirection = V;

        mySurfaceType = GeomAbs_SurfaceOfExtrusion;

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

    public override GeomAbs_Shape UContinuity() => myBasisCurve!.Continuity();

    public override GeomAbs_Shape VContinuity() => GeomAbs_CN;

    public override int NbUIntervals(GeomAbs_Shape S) => myBasisCurve!.NbIntervals(S);

    public override int NbVIntervals(GeomAbs_Shape S) => 1;

    public override void UIntervals(NCollection_Array1<double> T, GeomAbs_Shape S) => myBasisCurve!.Intervals(T, S);

    public override void VIntervals(NCollection_Array1<double> T, GeomAbs_Shape S)
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

    /// <exception cref="Standard_DomainError">Always (not periodic in V).</exception>
    public override double VPeriod() => throw new Standard_DomainError("GeomAdaptor_SurfaceOfLinearExtrusion::VPeriod");

    public override double UResolution(double R3d) => myBasisCurve!.Resolution(R3d);

    public override double VResolution(double R3d) => R3d;

    /// <summary>
    /// GeomAbs_Plane for an extruded line not parallel to the direction, or a conic whose plane contains it;
    /// GeomAbs_Cylinder for a circle extruded along its axis; GeomAbs_SurfaceOfExtrusion otherwise.
    /// </summary>
    public override GeomAbs_SurfaceType GetType()
    {
        switch (myBasisCurve!.GetType())
        {
            case GeomAbs_Line:
            {
                gp_Dir D = myBasisCurve.Line().Direction();
                if (!myDirection.IsParallel(D, Precision.Angular()))
                {
                    return GeomAbs_Plane;
                }
                break;
            }

            case GeomAbs_Circle:
            {
                gp_Dir D = (myBasisCurve.Circle()).Axis().Direction();
                if (myDirection.IsParallel(D, Precision.Angular()))
                {
                    return GeomAbs_Cylinder;
                }
                else if (myDirection.IsNormal(D, Precision.Angular()))
                {
                    return GeomAbs_Plane;
                }
                break;
            }

            case GeomAbs_Ellipse:
            {
                gp_Dir D = (myBasisCurve.Ellipse()).Axis().Direction();
                if (myDirection.IsNormal(D, Precision.Angular()))
                {
                    return GeomAbs_Plane;
                }
                break;
            }

            case GeomAbs_Parabola:
            {
                gp_Dir D = (myBasisCurve.Parabola()).Axis().Direction();
                if (myDirection.IsNormal(D, Precision.Angular()))
                {
                    return GeomAbs_Plane;
                }
                break;
            }

            case GeomAbs_Hyperbola:
            {
                gp_Dir D = (myBasisCurve.Hyperbola()).Axis().Direction();
                if (myDirection.IsNormal(D, Precision.Angular()))
                {
                    return GeomAbs_Plane;
                }
                break;
            }

            default:
                break;
        }

        return GeomAbs_SurfaceOfExtrusion;
    }

    /// <exception cref="Standard_NoSuchObject">The surface is not a plane.</exception>
    public override gp_Pln Plane()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_Plane, "GeomAdaptor_SurfaceOfLinearExtrusion::Plane");

        gp_Pnt P = new gp_Pnt();
        gp_Vec D1u = new gp_Vec(), newZ = new gp_Vec();
        double UFirst = myBasisCurve!.FirstParameter();
        double ULast = myBasisCurve.LastParameter();
        if (Precision.IsNegativeInfinite(UFirst) && Precision.IsPositiveInfinite(ULast))
        {
            UFirst = -100.0;
            ULast = 100.0;
        }
        else if (Precision.IsNegativeInfinite(UFirst))
        {
            UFirst = ULast - 200.0;
        }
        else if (Precision.IsPositiveInfinite(ULast))
        {
            ULast = UFirst + 200.0;
        }
        double deltau = (ULast - UFirst) / 20.0;
        for (int i = 1; i <= 21; i++)
        {
            double prm = UFirst + (i - 1) * deltau;
            myBasisCurve.D1(prm, out P, out D1u);
            newZ = D1u.Normalized().Crossed(new gp_Vec(myDirection));
            if (newZ.Magnitude() > 1.0e-12)
            {
                break;
            }
        }
        gp_Ax3 Ax3 = new gp_Ax3(P, new gp_Dir(newZ), new gp_Dir(D1u));
        if (myDirection.Dot(Ax3.YDirection()) < 0.0)
        {
            Ax3.YReverse();
        }
        return new gp_Pln(Ax3);
    }

    /// <exception cref="Standard_NoSuchObject">The surface is not a cylinder.</exception>
    public override gp_Cylinder Cylinder()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_Cylinder, "GeomAdaptor_SurfaceOfLinearExtrusion::Cylinder");

        gp_Circ C = myBasisCurve!.Circle();
        gp_Ax3 Ax3 = new gp_Ax3(C.Position());
        if (myDirection.Dot((C.Axis()).Direction()) < 0.0)
        {
            Ax3.ZReverse();
        }
        return new gp_Cylinder(Ax3, C.Radius());
    }

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override gp_Cone Cone() => throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::Cone");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override gp_Sphere Sphere() => throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::Sphere");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override gp_Torus Torus() => throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::Torus");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override gp_Ax1 AxeOfRevolution() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::Axes");

    public override int UDegree() => myBasisCurve!.Degree();

    public override int NbUPoles() => myBasisCurve!.NbPoles();

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override bool IsURational() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::IsURational");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override bool IsVRational() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::IsVRational");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override Geom_BezierSurface Bezier() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::Bezier");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override Geom_BSplineSurface BSpline() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfLinearExtrusion::BSpline");

    public override gp_Dir Direction() => myDirection;

    /// <summary>The extruded curve (the adaptor given to Load).</summary>
    public override Adaptor3d_Curve BasisCurve() => myBasisCurve!;
}
