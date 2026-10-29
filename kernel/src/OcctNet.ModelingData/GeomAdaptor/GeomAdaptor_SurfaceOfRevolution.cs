// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The surface of revolution of an adapted curve about an axis: U in [0, 2*PI] is the angle, V the curve's
/// parameter. <see cref="GetType"/> recognises planes, cylinders, cones, spheres and tori.
/// </summary>
/// <remarks>Evaluation is the base adaptor's (Geom_RevolutionUtils over the curve adaptor).</remarks>
public class GeomAdaptor_SurfaceOfRevolution : GeomAdaptor_Surface
{
    private Adaptor3d_Curve? myBasisCurve; // revolved curve
    private gp_Ax1 myAxis; // axis of revolution
    private bool myHaveAxis; // whether axis of revolution is initialized
    private gp_Ax3 myAxeRev; // auxiliary trihedron according to the curve position

    /// <summary>An adaptor without a curve or axis.</summary>
    public GeomAdaptor_SurfaceOfRevolution()
    {
        myHaveAxis = false;
        myAxeRev = new gp_Ax3();
    }

    /// <summary>The revolution of C, the axis to be loaded.</summary>
    public GeomAdaptor_SurfaceOfRevolution(Adaptor3d_Curve C)
    {
        myHaveAxis = false;
        myAxeRev = new gp_Ax3();
        Load(C);
    }

    /// <summary>The revolution of C about V.</summary>
    /// <exception cref="Standard_ConstructionError">The axis and the meridian are confused.</exception>
    public GeomAdaptor_SurfaceOfRevolution(Adaptor3d_Curve C, in gp_Ax1 V)
    {
        myHaveAxis = false;
        myAxeRev = new gp_Ax3();
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
    /// <exception cref="Standard_ConstructionError">The axis and the meridian are confused.</exception>
    public void Load(in gp_Ax1 V)
    {
        myHaveAxis = true;
        myAxis = V;

        mySurfaceType = GeomAbs_SurfaceOfRevolution;

        // Populate revolution surface data for fast evaluation
        RevolutionData aRevData = new RevolutionData();
        aRevData.BasisCurve = myBasisCurve!;
        aRevData.Axis = myAxis;
        mySurfaceData = aRevData;

        // Eval myAxeRev : axe of revolution ( Determination de Ox).
        gp_Pnt P, Q;
        gp_Pnt O = myAxis.Location();
        gp_Dir Ox;
        gp_Dir Oz = myAxis.Direction();
        bool yrev = false;
        if (myBasisCurve!.GetType() == GeomAbs_Line)
        {
            if ((myBasisCurve.Line().Direction()).Dot(Oz) < 0.0)
            {
                yrev = true;
                Oz.Reverse();
            }
        }

        if (myBasisCurve.GetType() == GeomAbs_Circle)
        {
            Q = P = (myBasisCurve.Circle()).Location();
        }
        else
        {
            double First = myBasisCurve.FirstParameter();
            P = Value(0.0, 0.0); // which does not mean much
            if (GetType() == GeomAbs_Cone)
            {
                if (new gp_Lin(myAxis).Distance(P) <= Precision.Confusion())
                {
                    Q = ElCLib.Value(1.0, myBasisCurve.Line());
                }
                else
                {
                    Q = P;
                }
            }
            else if (Precision.IsInfinite(First))
            {
                Q = P;
            }
            else
            {
                Q = Value(0.0, First);
            }
        }

        gp_Dir DZ = myAxis.Direction();
        O.SetXYZ(O.XYZ() + (new gp_Vec(O, P) * new gp_Vec(DZ)) * DZ.XYZ());
        if (new gp_Lin(myAxis).Distance(Q) > Precision.Confusion())
        {
            Ox = new gp_Dir(Q.XYZ() - O.XYZ());
        }
        else
        {
            double First = myBasisCurve.FirstParameter();
            double Last = myBasisCurve.LastParameter();
            int Ratio = 1;
            double Dist;
            gp_Pnt PP;
            do
            {
                PP = myBasisCurve.Value(First + (Last - First) / Ratio);
                Dist = new gp_Lin(myAxis).Distance(PP);
                Ratio++;
            } while (Dist < Precision.Confusion() && Ratio < 100);

            if (Ratio >= 100)
            {
                throw new Standard_ConstructionError("Adaptor3d_SurfaceOfRevolution : Axe and meridian are confused");
            }
            // C++: (Oz ^ gp_Vec(...)) ^ Oz, with the vector converted to a gp_Dir (normalised), and gp_Dir's
            // operator^ normalising each product.
            Ox = (Oz ^ new gp_Dir(new gp_Vec(PP.XYZ() - O.XYZ()))) ^ Oz;
        }

        myAxeRev = new gp_Ax3(O, Oz, Ox);

        if (yrev)
        {
            myAxeRev.YReverse();
        }
        else if (myBasisCurve.GetType() == GeomAbs_Circle)
        {
            gp_Dir DC = (myBasisCurve.Circle()).Axis().Direction();
            if ((Ox.Crossed(Oz)).Dot(DC) < 0.0)
            {
                myAxeRev.ZReverse();
            }
        }
    }

    public override gp_Ax1 AxeOfRevolution() => myAxis;

    public override double FirstUParameter() => 0.0;

    public override double LastUParameter() => 2 * Math.PI;

    public override double FirstVParameter() => myBasisCurve!.FirstParameter();

    public override double LastVParameter() => myBasisCurve!.LastParameter();

    public override GeomAbs_Shape UContinuity() => GeomAbs_CN;

    public override GeomAbs_Shape VContinuity() => myBasisCurve!.Continuity();

    public override int NbUIntervals(GeomAbs_Shape S) => 1;

    public override int NbVIntervals(GeomAbs_Shape S) => myBasisCurve!.NbIntervals(S);

    public override void UIntervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        T[T.Lower()] = 0.0;
        T[T.Lower() + 1] = 2 * Math.PI;
    }

    public override void VIntervals(NCollection_Array1<double> T, GeomAbs_Shape S) => myBasisCurve!.Intervals(T, S);

    /// <summary>The whole surface again: only [0, 2*PI] is accepted.</summary>
    /// <exception cref="Standard_OutOfRange">First or Last differ from 0 or 2*PI by more than Precision::PConfusion().</exception>
    public override Adaptor3d_Surface UTrim(double First, double Last, double Tol)
    {
        double Eps = Precision.PConfusion();
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

    public override double UResolution(double R3d) => Precision.Parametric(R3d);

    public override double VResolution(double R3d) => myBasisCurve!.Resolution(R3d);

    /// <summary>
    /// GeomAbs_Plane, Cylinder or Cone for a line (normal to, parallel to or crossing the axis), GeomAbs_Sphere or
    /// Torus for a circle coplanar with the axis; GeomAbs_SurfaceOfRevolution otherwise.
    /// </summary>
    public override GeomAbs_SurfaceType GetType()
    {
        double TolConf = Precision.Confusion();
        double TolAng = Precision.Angular();
        double TolConeSemiAng = Precision.Confusion();

        switch (myBasisCurve!.GetType())
        {
            case GeomAbs_Line:
            {
                gp_Ax1 Axe = myBasisCurve.Line().Position();

                if (myAxis.IsParallel(Axe, TolAng))
                {
                    gp_Pnt P = Value(0.0, 0.0);
                    double R = new gp_Vec(myAxeRev.Location(), P) * new gp_Vec(myAxeRev.XDirection());
                    if (R > TolConf)
                    {
                        return GeomAbs_Cylinder;
                    }
                }
                else if (myAxis.IsNormal(Axe, TolAng))
                {
                    return GeomAbs_Plane;
                }
                else
                {
                    double uf = myBasisCurve.FirstParameter();
                    double ul = myBasisCurve.LastParameter();
                    bool istrim = (!Precision.IsInfinite(uf) && !Precision.IsInfinite(ul));
                    if (istrim)
                    {
                        gp_Pnt pf = myBasisCurve.Value(uf);
                        gp_Pnt pl = myBasisCurve.Value(ul);
                        double len = pf.Distance(pl);
                        // Compute the distance projected onto the axis.
                        gp_Vec vlin = new gp_Vec(pf, pl);
                        gp_Vec vaxe = new gp_Vec(myAxis.Direction());
                        double projlen = Math.Abs(vaxe.Dot(vlin));
                        if ((len - projlen) <= TolConf)
                        {
                            gp_Pnt P = Value(0.0, 0.0);
                            double R = new gp_Vec(myAxeRev.Location(), P) * new gp_Vec(myAxeRev.XDirection());
                            if (R > TolConf)
                            {
                                return GeomAbs_Cylinder;
                            }
                        }
                        else if (projlen <= TolConf)
                        {
                            return GeomAbs_Plane;
                        }
                    }
                    gp_Vec V = new gp_Vec(myAxis.Location(), myBasisCurve.Line().Location());
                    gp_Vec W = new gp_Vec(Axe.Direction());
                    gp_Vec AxisDir = new gp_Vec(myAxis.Direction());
                    double proj = Math.Abs(W.Dot(AxisDir));
                    if (Math.Abs(V.DotCross(AxisDir, W)) <= TolConf
                        && (proj >= TolConeSemiAng && proj <= 1.0 - TolConeSemiAng))
                    {
                        return GeomAbs_Cone;
                    }
                }
                break;
            } // case GeomAbs_Line:
            //
            case GeomAbs_Circle:
            {
                double MajorRadius, aR;
                gp_Lin aLin = new gp_Lin(myAxis);
                //
                gp_Circ C = myBasisCurve.Circle();
                gp_Pnt aLC = C.Location();
                aR = C.Radius();
                //

                if (!C.Position().IsCoplanar(myAxis, TolConf, TolAng))
                {
                    return GeomAbs_SurfaceOfRevolution;
                }
                else if (aLin.Distance(aLC) <= TolConf)
                {
                    return GeomAbs_Sphere;
                }
                else
                {
                    MajorRadius = aLin.Distance(aLC);
                    if (MajorRadius > aR)
                    {
                        return GeomAbs_Torus;
                    }
                }
                break;
            }
            //
            default:
                break;
        }

        return GeomAbs_SurfaceOfRevolution;
    }

    /// <exception cref="Standard_NoSuchObject">The surface is not a plane.</exception>
    public override gp_Pln Plane()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_Plane, "GeomAdaptor_SurfaceOfRevolution:Plane");

        gp_Ax3 Axe = myAxeRev;
        gp_Pnt aPonCurve = Value(0.0, 0.0);
        double aDot = (aPonCurve.XYZ() - myAxis.Location().XYZ()).Dot(myAxis.Direction().XYZ());

        gp_Pnt P = new gp_Pnt(myAxis.Location().XYZ() + aDot * myAxis.Direction().XYZ());
        Axe.SetLocation(P);
        if (Axe.XDirection().Dot(myBasisCurve!.Line().Direction()) >= -Precision.Confusion())
        {
            Axe.XReverse();
        }

        return new gp_Pln(Axe);
    }

    /// <exception cref="Standard_NoSuchObject">The surface is not a cylinder.</exception>
    public override gp_Cylinder Cylinder()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_Cylinder, "GeomAdaptor_SurfaceOfRevolution::Cylinder");

        gp_Pnt P = Value(0.0, 0.0);
        double R = new gp_Vec(myAxeRev.Location(), P) * new gp_Vec(myAxeRev.XDirection());
        return new gp_Cylinder(myAxeRev, R);
    }

    /// <exception cref="Standard_NoSuchObject">The surface is not a cone.</exception>
    public override gp_Cone Cone()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_Cone, "GeomAdaptor_SurfaceOfRevolution:Cone");

        gp_Ax3 Axe = myAxeRev;
        gp_Dir ldir = (myBasisCurve!.Line()).Direction();
        double Angle = (Axe.Direction()).Angle(ldir);
        gp_Pnt P0 = Value(0.0, 0.0);
        double R = (Axe.Location()).Distance(P0);
        if (R >= Precision.Confusion())
        {
            gp_Pnt O = Axe.Location();
            gp_Vec OP0 = new gp_Vec(O, P0);
            double t = OP0.Dot(new gp_Vec(Axe.XDirection()));
            t /= ldir.Dot(Axe.XDirection());
            OP0.Add(-t * new gp_Vec(ldir));
            if (OP0.Dot(new gp_Vec(Axe.Direction())) > 0.0)
            {
                Angle = -Angle;
            }
        }
        return new gp_Cone(Axe, Angle, R);
    }

    /// <exception cref="Standard_NoSuchObject">The surface is not a sphere.</exception>
    public override gp_Sphere Sphere()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_Sphere, "GeomAdaptor_SurfaceOfRevolution:Sphere");

        gp_Circ C = myBasisCurve!.Circle();
        gp_Ax3 Axe = myAxeRev;
        Axe.SetLocation(C.Location());
        return new gp_Sphere(Axe, C.Radius());
    }

    /// <exception cref="Standard_NoSuchObject">The surface is not a torus.</exception>
    public override gp_Torus Torus()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_Torus, "GeomAdaptor_SurfaceOfRevolution:Torus");

        gp_Circ C = myBasisCurve!.Circle();
        double MajorRadius = new gp_Lin(myAxis).Distance(C.Location());
        return new gp_Torus(myAxeRev, MajorRadius, C.Radius());
    }

    public override int VDegree() => myBasisCurve!.Degree();

    public override int NbVPoles() => myBasisCurve!.NbPoles();

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override int NbVKnots() => throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfRevolution::NbVKnots");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override bool IsURational() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfRevolution::IsURational");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override bool IsVRational() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfRevolution::IsVRational");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override Geom_BezierSurface Bezier() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfRevolution::Bezier");

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override Geom_BSplineSurface BSpline() =>
        throw new Standard_NoSuchObject("GeomAdaptor_SurfaceOfRevolution::BSpline");

    /// <summary>The trihedron of the revolution: the axis, with X towards the curve.</summary>
    public gp_Ax3 Axis() => myAxeRev;

    /// <summary>The revolved curve (the adaptor given to Load).</summary>
    public override Adaptor3d_Curve BasisCurve() => myBasisCurve!;
}
