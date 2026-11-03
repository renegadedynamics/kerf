// Port of: src/DataExchange/TKDESTEP/StepToGeom/StepToGeom.hxx
// Port of: src/DataExchange/TKDESTEP/StepToGeom/StepToGeom.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.Continuity;
using static Occt.StepData_Logical;
using static Occt.StepGeom_KnotType;
using static Occt.StepGeom_TrimmingPreference;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The conversion of STEP's geometric entities (StepGeom) into Geom and Geom2d: points, directions, vectors,
/// placements, curves, surfaces and transformations. Each method returns null when the entity can't be converted.
/// </summary>
/// <remarks>
/// The units of the file are applied through <see cref="StepData_Factors"/> (lengths multiplied by
/// <see cref="StepData_Factors.LengthFactor"/>, angles by <see cref="StepData_Factors.PlaneAngleFactor"/>); the
/// overloads without factors use the defaults (millimetres and radians). The results are new, unfrozen geometry.
/// <c>MakeYprRotation</c> was added by unit 8F (decision 1238).
/// </remarks>
public static class StepToGeom
{
    //=============================================================================
    // Creation d' un Ax1Placement de Geom a partir d' un axis1_placement de Step
    //=============================================================================

    /// <summary>Converts an axis1_placement.</summary>
    public static Axis1Placement? MakeAxis1Placement(StepGeom_Axis1Placement SA, in StepData_Factors theLocalFactors)
    {
        CartesianPoint? P = MakeCartesianPoint(SA.Location(), theLocalFactors);
        if (P is not null)
        {
            // sln 22.10.2001. CTS23496: If problems with creation of axis direction occur default direction
            // is used
            Direction3D D = new Direction3D(Direction3D.D.Z);
            if (SA.HasAxis())
            {
                Direction? D1 = MakeDirection(SA.Axis());
                if (D1 is not null)
                {
                    D = D1.Dir();
                }
            }
            return new Axis1Placement(P.Pnt(), D);
        }
        return null;
    }

    /// <summary>Converts an axis1_placement (default factors).</summary>
    public static Axis1Placement? MakeAxis1Placement(StepGeom_Axis1Placement SA) => MakeAxis1Placement(SA, new StepData_Factors());

    //=============================================================================
    // Creation d' un Axis2Placement de Geom a partir d' un axis2_placement_3d de Step
    //=============================================================================

    /// <summary>Converts an axis2_placement_3d.</summary>
    public static Axis2Placement? MakeAxis2Placement(StepGeom_Axis2Placement3d SA, in StepData_Factors theLocalFactors)
    {
        CartesianPoint? P = MakeCartesianPoint(SA.Location(), theLocalFactors);
        if (P is not null)
        {
            Point3D Pgp = P.Pnt();

            // sln 22.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used (MakeLine(...) function)
            Direction3D Ngp = new Direction3D(Direction3D.D.Z);
            if (SA.HasAxis())
            {
                Direction? D = MakeDirection(SA.Axis());
                if (D is not null)
                {
                    Ngp = D.Dir();
                }
            }

            Frame3D gpAx2 = new Frame3D();
            bool isDefaultDirectionUsed = true;
            if (SA.HasRefDirection())
            {
                Direction? D = MakeDirection(SA.RefDirection());
                if (D is not null)
                {
                    Direction3D Vxgp = D.Dir();
                    if (!Ngp.IsParallel(Vxgp, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        gpAx2 = new Frame3D(Pgp, Ngp, Vxgp);
                        isDefaultDirectionUsed = false;
                    }
                }
            }
            if (isDefaultDirectionUsed)
            {
                gpAx2 = new Frame3D(Pgp, Ngp);
            }

            return new Axis2Placement(gpAx2);
        }
        return null;
    }

    /// <summary>Converts an axis2_placement_3d (default factors).</summary>
    public static Axis2Placement? MakeAxis2Placement(StepGeom_Axis2Placement3d SA) => MakeAxis2Placement(SA, new StepData_Factors());

    //=============================================================================
    // Creation of an AxisPlacement from a Kinematic SuParameters for Step
    //=============================================================================

    /// <summary>Converts the kinematic su_parameters into a placement.</summary>
    public static Axis2Placement MakeAxis2Placement(StepGeom_SuParameters theSP)
    {
        double aLocX = theSP.A() * Math.Cos(theSP.Gamma()) + theSP.B() * Math.Sin(theSP.Gamma()) * Math.Sin(theSP.Alpha());
        double aLocY = theSP.A() * Math.Sin(theSP.Gamma()) - theSP.B() * Math.Cos(theSP.Gamma()) * Math.Sin(theSP.Alpha());
        double aLocZ = theSP.C() + theSP.B() * Math.Cos(theSP.Alpha());
        double anAsisX = Math.Sin(theSP.Gamma()) * Math.Sin(theSP.Alpha());
        double anAxisY = -Math.Cos(theSP.Gamma()) * Math.Sin(theSP.Alpha());
        double anAxisZ = Math.Cos(theSP.Alpha());
        double aDirX = Math.Cos(theSP.Gamma()) * Math.Cos(theSP.Beta())
                       - Math.Sin(theSP.Gamma()) * Math.Cos(theSP.Alpha()) * Math.Sin(theSP.Beta());
        double aDirY = Math.Sin(theSP.Gamma()) * Math.Cos(theSP.Beta())
                       + Math.Cos(theSP.Gamma()) * Math.Cos(theSP.Alpha()) * Math.Sin(theSP.Beta());
        double aDirZ = Math.Sin(theSP.Alpha()) * Math.Sin(theSP.Beta());
        Point3D Pgp = new Point3D(aLocX, aLocY, aLocZ);
        Direction3D Ngp = new Direction3D(anAsisX, anAxisY, anAxisZ);
        Direction3D Vxgp = new Direction3D(aDirX, aDirY, aDirZ);
        Frame3D gpAx2 = new Frame3D(Pgp, Ngp, Vxgp);
        return new Axis2Placement(gpAx2);
    }

    //=============================================================================
    // Creation d' un AxisPlacement de Geom2d a partir d' un axis2_placement_3d de Step
    //=============================================================================

    /// <summary>Converts an axis2_placement_2d.</summary>
    public static AxisPlacement2? MakeAxisPlacement(StepGeom_Axis2Placement2d SA, in StepData_Factors theLocalFactors)
    {
        CartesianPoint2? P = MakeCartesianPoint2d(SA.Location(), theLocalFactors);
        if (P is not null)
        {
            // sln 23.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used
            Direction2D Vxgp = new Direction2D(Direction2D.D.X);
            if (SA.HasRefDirection())
            {
                Direction2? Vx = MakeDirection2d(SA.RefDirection());
                if (Vx is not null)
                {
                    Vxgp = Vx.Dir2d();
                }
            }

            return new AxisPlacement2(P.Pnt2d(), Vxgp);
        }
        return null;
    }

    /// <summary>Converts an axis2_placement_2d (default factors).</summary>
    public static AxisPlacement2? MakeAxisPlacement(StepGeom_Axis2Placement2d SA) => MakeAxisPlacement(SA, new StepData_Factors());

    //=============================================================================
    // Creation d' une BoundedCurve de Geom a partir d' une BoundedCurve de Step
    //=============================================================================

    /// <summary>Converts a bounded_curve (B-splines, Bezier, uniform and quasi-uniform curves, trimmed curves, polylines).</summary>
    public static BoundedCurve3D? MakeBoundedCurve(StepGeom_BoundedCurve SC, in StepData_Factors theLocalFactors)
    {
        if (SC is StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            return MakeBSplineCurve(SC as StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve, theLocalFactors);
        }
        if (SC is StepGeom_BSplineCurveWithKnots)
        {
            return MakeBSplineCurve(SC as StepGeom_BSplineCurveWithKnots, theLocalFactors);
        }
        if (SC is StepGeom_TrimmedCurve)
        {
            return MakeTrimmedCurve(SC as StepGeom_TrimmedCurve, theLocalFactors);
        }

        // STEP BezierCurve, UniformCurve and QuasiUniformCurve are transformed into
        // STEP BSplineCurve before being mapped onto CAS.CADE/SF
        if (SC is StepGeom_BezierCurve)
        {
            StepGeom_BezierCurve BzC = SC as StepGeom_BezierCurve;
            int aDegree = BzC.Degree();
            if (aDegree < 1 || aDegree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepGeom_BSplineCurveWithKnots BSPL = new StepGeom_BSplineCurveWithKnots();
            BSPL.SetDegree(aDegree);
            BSPL.SetControlPointsList(BzC.ControlPointsList());
            BSPL.SetCurveForm(BzC.CurveForm());
            BSPL.SetClosedCurve(BzC.ClosedCurve());
            BSPL.SetSelfIntersect(BzC.SelfIntersect());
            // Compute Knots and KnotsMultiplicity
            NCollection_HArray1<int> Kmult = new NCollection_HArray1<int>(1, 2);
            NCollection_HArray1<double> Knots = new NCollection_HArray1<double>(1, 2);
            Kmult.SetValue(1, BzC.Degree() + 1);
            Kmult.SetValue(2, BzC.Degree() + 1);
            Knots.SetValue(1, 0.0);
            Knots.SetValue(2, 1.0);
            BSPL.SetKnotMultiplicities(Kmult);
            BSPL.SetKnots(Knots);

            return MakeBSplineCurve(BSPL, theLocalFactors);
        }

        if (SC is StepGeom_UniformCurve)
        {
            StepGeom_UniformCurve UC = SC as StepGeom_UniformCurve;
            int aDegree = UC.Degree();
            if (aDegree < 1 || aDegree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepGeom_BSplineCurveWithKnots BSPL = new StepGeom_BSplineCurveWithKnots();
            BSPL.SetDegree(aDegree);
            BSPL.SetControlPointsList(UC.ControlPointsList());
            BSPL.SetCurveForm(UC.CurveForm());
            BSPL.SetClosedCurve(UC.ClosedCurve());
            BSPL.SetSelfIntersect(UC.SelfIntersect());

            // Compute Knots and KnotsMultiplicity
            int nbK = BSPL.NbControlPointsList() + BSPL.Degree() + 1;
            NCollection_HArray1<int> Kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> Knots = new NCollection_HArray1<double>(1, nbK);
            for (int iUC = 1; iUC <= nbK; iUC++)
            {
                Kmult.SetValue(iUC, 1);
                Knots.SetValue(iUC, iUC - 1.0);
            }
            BSPL.SetKnotMultiplicities(Kmult);
            BSPL.SetKnots(Knots);

            return MakeBSplineCurve(BSPL, theLocalFactors);
        }

        if (SC is StepGeom_QuasiUniformCurve)
        {
            StepGeom_QuasiUniformCurve QUC = SC as StepGeom_QuasiUniformCurve;
            int aDegree = QUC.Degree();
            if (aDegree < 1 || aDegree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepGeom_BSplineCurveWithKnots BSPL = new StepGeom_BSplineCurveWithKnots();
            BSPL.SetDegree(aDegree);
            BSPL.SetControlPointsList(QUC.ControlPointsList());
            BSPL.SetCurveForm(QUC.CurveForm());
            BSPL.SetClosedCurve(QUC.ClosedCurve());
            BSPL.SetSelfIntersect(QUC.SelfIntersect());

            // Compute Knots and KnotsMultiplicity
            int nbK = BSPL.NbControlPointsList() - BSPL.Degree() + 1;
            NCollection_HArray1<int> Kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> Knots = new NCollection_HArray1<double>(1, nbK);
            for (int iQUC = 1; iQUC <= nbK; iQUC++)
            {
                Kmult.SetValue(iQUC, 1);
                Knots.SetValue(iQUC, iQUC - 1.0);
            }
            Kmult.SetValue(1, BSPL.Degree() + 1);
            Kmult.SetValue(nbK, BSPL.Degree() + 1);
            BSPL.SetKnotMultiplicities(Kmult);
            BSPL.SetKnots(Knots);

            return MakeBSplineCurve(BSPL, theLocalFactors);
        }

        if (SC is StepGeom_UniformCurveAndRationalBSplineCurve)
        {
            StepGeom_UniformCurveAndRationalBSplineCurve RUC = SC as StepGeom_UniformCurveAndRationalBSplineCurve;
            int aDegree = RUC.Degree();
            if (aDegree < 1 || aDegree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve RBSPL = new StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve();

            // Compute Knots and KnotsMultiplicity
            int nbK = RUC.NbControlPointsList() + aDegree + 1;
            NCollection_HArray1<int> Kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> Knots = new NCollection_HArray1<double>(1, nbK);
            for (int iUC = 1; iUC <= nbK; iUC++)
            {
                Kmult.SetValue(iUC, 1);
                Knots.SetValue(iUC, iUC - 1.0);
            }

            // Initialize the BSplineCurveWithKnotsAndRationalBSplineCurve
            RBSPL.Init(RUC.Name(),
                       aDegree,
                       RUC.ControlPointsList(),
                       RUC.CurveForm(),
                       RUC.ClosedCurve(),
                       RUC.SelfIntersect(),
                       Kmult,
                       Knots,
                       StepGeom_KnotType.StepGeom_ktUnspecified,
                       RUC.WeightsData());

            return MakeBSplineCurve(RBSPL, theLocalFactors);
        }

        if (SC is StepGeom_QuasiUniformCurveAndRationalBSplineCurve)
        {
            StepGeom_QuasiUniformCurveAndRationalBSplineCurve RQUC = SC as StepGeom_QuasiUniformCurveAndRationalBSplineCurve;
            int aDegree = RQUC.Degree();
            if (aDegree < 1 || aDegree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve RBSPL = new StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve();

            // Compute Knots and KnotsMultiplicity
            int nbK = RQUC.NbControlPointsList() - aDegree + 1;
            NCollection_HArray1<int> Kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> Knots = new NCollection_HArray1<double>(1, nbK);
            for (int iRQUC = 1; iRQUC <= nbK; iRQUC++)
            {
                Kmult.SetValue(iRQUC, 1);
                Knots.SetValue(iRQUC, iRQUC - 1.0);
            }
            Kmult.SetValue(1, aDegree + 1);
            Kmult.SetValue(nbK, aDegree + 1);
            // Initialize the BSplineCurveWithKnotsAndRationalBSplineCurve
            RBSPL.Init(RQUC.Name(),
                       aDegree,
                       RQUC.ControlPointsList(),
                       RQUC.CurveForm(),
                       RQUC.ClosedCurve(),
                       RQUC.SelfIntersect(),
                       Kmult,
                       Knots,
                       StepGeom_KnotType.StepGeom_ktUnspecified,
                       RQUC.WeightsData());

            return MakeBSplineCurve(RBSPL, theLocalFactors);
        }

        if (SC is StepGeom_Polyline)
        { //: n6 abv 15 Feb 99
            return MakePolyline(SC as StepGeom_Polyline, theLocalFactors);
        }

        return null;
    }

    /// <summary>Converts a bounded_curve (default factors).</summary>
    public static BoundedCurve3D? MakeBoundedCurve(StepGeom_BoundedCurve SC) => MakeBoundedCurve(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une BoundedCurve de Geom a partir d' une BoundedCurve de Step
    //=============================================================================

    /// <summary>Converts a bounded_curve into a 2D curve.</summary>
    public static BoundedCurve2D? MakeBoundedCurve2d(StepGeom_BoundedCurve SC, in StepData_Factors theLocalFactors)
    {
        if (SC is StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            return MakeBSplineCurve2d(SC as StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve, theLocalFactors);
        }
        if (SC is StepGeom_BSplineCurveWithKnots)
        {
            return MakeBSplineCurve2d(SC as StepGeom_BSplineCurveWithKnots, theLocalFactors);
        }
        if (SC is StepGeom_TrimmedCurve)
        {
            return MakeTrimmedCurve2d(SC as StepGeom_TrimmedCurve, theLocalFactors);
        }
        if (SC is StepGeom_Polyline)
        { //: n6 abv 15 Feb 99
            return MakePolyline2d(SC as StepGeom_Polyline, theLocalFactors);
        }
        return null;
    }

    /// <summary>Converts a bounded_curve into a 2D curve (default factors).</summary>
    public static BoundedCurve2D? MakeBoundedCurve2d(StepGeom_BoundedCurve SC) => MakeBoundedCurve2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une BoundedSurface de Geom a partir d' une BoundedSurface de Step
    //=============================================================================

    /// <summary>Converts a bounded_surface (B-splines, Bezier, uniform and quasi-uniform surfaces, rectangular trimmed surfaces).</summary>
    public static BoundedSurface? MakeBoundedSurface(StepGeom_BoundedSurface SS, in StepData_Factors theLocalFactors)
    {
        if (SS is StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface)
        {
            return MakeBSplineSurface(SS as StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface, theLocalFactors);
        }
        if (SS is StepGeom_BSplineSurfaceWithKnots)
        {
            return MakeBSplineSurface(SS as StepGeom_BSplineSurfaceWithKnots, theLocalFactors);
        }
        if (SS is StepGeom_RectangularTrimmedSurface)
        {
            return MakeRectangularTrimmedSurface(SS as StepGeom_RectangularTrimmedSurface, theLocalFactors);
        }

        // STEP BezierSurface, UniformSurface and QuasiUniformSurface are transformed
        // into STEP BSplineSurface before being mapped onto CAS.CADE/SF
        if (SS is StepGeom_BezierSurface)
        {
            StepGeom_BezierSurface BzS = SS as StepGeom_BezierSurface;
            StepGeom_BSplineSurfaceWithKnots BSPL = new StepGeom_BSplineSurfaceWithKnots();
            BSPL.SetUDegree(BzS.UDegree());
            BSPL.SetVDegree(BzS.VDegree());
            BSPL.SetControlPointsList(BzS.ControlPointsList());
            BSPL.SetSurfaceForm(BzS.SurfaceForm());
            BSPL.SetUClosed(BzS.UClosed());
            BSPL.SetVClosed(BzS.VClosed());
            BSPL.SetSelfIntersect(BzS.SelfIntersect());

            // Compute Knots and KnotsMultiplicity
            NCollection_HArray1<int> UKmult = new NCollection_HArray1<int>(1, 2);
            NCollection_HArray1<int> VKmult = new NCollection_HArray1<int>(1, 2);
            NCollection_HArray1<double> UKnots = new NCollection_HArray1<double>(1, 2);
            NCollection_HArray1<double> VKnots = new NCollection_HArray1<double>(1, 2);
            UKmult.SetValue(1, BzS.UDegree() + 1);
            UKmult.SetValue(2, BzS.UDegree() + 1);
            VKmult.SetValue(1, BzS.VDegree() + 1);
            VKmult.SetValue(2, BzS.VDegree() + 1);
            UKnots.SetValue(1, 0.0);
            UKnots.SetValue(2, 1.0);
            VKnots.SetValue(1, 0.0);
            VKnots.SetValue(2, 1.0);
            BSPL.SetUMultiplicities(UKmult);
            BSPL.SetVMultiplicities(VKmult);
            BSPL.SetUKnots(UKnots);
            BSPL.SetVKnots(VKnots);

            return MakeBSplineSurface(BSPL, theLocalFactors);
        }

        if (SS is StepGeom_UniformSurface)
        {
            StepGeom_UniformSurface US = SS as StepGeom_UniformSurface;
            StepGeom_BSplineSurfaceWithKnots BSPL = new StepGeom_BSplineSurfaceWithKnots();
            BSPL.SetUDegree(US.UDegree());
            BSPL.SetVDegree(US.VDegree());
            BSPL.SetControlPointsList(US.ControlPointsList());
            BSPL.SetSurfaceForm(US.SurfaceForm());
            BSPL.SetUClosed(US.UClosed());
            BSPL.SetVClosed(US.VClosed());
            BSPL.SetSelfIntersect(US.SelfIntersect());

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = BSPL.NbControlPointsListI() + BSPL.UDegree() + 1;
            NCollection_HArray1<int> UKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> UKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                UKmult.SetValue(iU, 1);
                UKnots.SetValue(iU, iU - 1.0);
            }
            BSPL.SetUMultiplicities(UKmult);
            BSPL.SetUKnots(UKnots);

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = BSPL.NbControlPointsListJ() + BSPL.VDegree() + 1;
            NCollection_HArray1<int> VKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> VKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                VKmult.SetValue(iV, 1);
                VKnots.SetValue(iV, iV - 1.0);
            }
            BSPL.SetVMultiplicities(VKmult);
            BSPL.SetVKnots(VKnots);

            return MakeBSplineSurface(BSPL, theLocalFactors);
        }

        if (SS is StepGeom_QuasiUniformSurface)
        {
            StepGeom_QuasiUniformSurface QUS = SS as StepGeom_QuasiUniformSurface;
            StepGeom_BSplineSurfaceWithKnots BSPL = new StepGeom_BSplineSurfaceWithKnots();
            BSPL.SetUDegree(QUS.UDegree());
            BSPL.SetVDegree(QUS.VDegree());
            BSPL.SetControlPointsList(QUS.ControlPointsList());
            BSPL.SetSurfaceForm(QUS.SurfaceForm());
            BSPL.SetUClosed(QUS.UClosed());
            BSPL.SetVClosed(QUS.VClosed());
            BSPL.SetSelfIntersect(QUS.SelfIntersect());

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = BSPL.NbControlPointsListI() - BSPL.UDegree() + 1;
            NCollection_HArray1<int> UKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> UKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                UKmult.SetValue(iU, 1);
                UKnots.SetValue(iU, iU - 1.0);
            }
            UKmult.SetValue(1, BSPL.UDegree() + 1);
            UKmult.SetValue(nbKU, BSPL.UDegree() + 1);
            BSPL.SetUMultiplicities(UKmult);
            BSPL.SetUKnots(UKnots);

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = BSPL.NbControlPointsListJ() - BSPL.VDegree() + 1;
            NCollection_HArray1<int> VKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> VKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                VKmult.SetValue(iV, 1);
                VKnots.SetValue(iV, iV - 1.0);
            }
            VKmult.SetValue(1, BSPL.VDegree() + 1);
            VKmult.SetValue(nbKV, BSPL.VDegree() + 1);
            BSPL.SetVMultiplicities(VKmult);
            BSPL.SetVKnots(VKnots);

            return MakeBSplineSurface(BSPL, theLocalFactors);
        }

        if (SS is StepGeom_UniformSurfaceAndRationalBSplineSurface)
        {
            StepGeom_UniformSurfaceAndRationalBSplineSurface RUS = SS as StepGeom_UniformSurfaceAndRationalBSplineSurface;
            StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface RBSPL = new StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface();

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = RUS.NbControlPointsListI() + RUS.UDegree() + 1;
            NCollection_HArray1<int> UKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> UKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                UKmult.SetValue(iU, 1);
                UKnots.SetValue(iU, iU - 1.0);
            }

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = RUS.NbControlPointsListJ() + RUS.VDegree() + 1;
            NCollection_HArray1<int> VKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> VKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                VKmult.SetValue(iV, 1);
                VKnots.SetValue(iV, iV - 1.0);
            }

            // Initialize the BSplineSurfaceWithKnotsAndRationalBSplineSurface
            RBSPL.Init(RUS.Name(),
                       RUS.UDegree(),
                       RUS.VDegree(),
                       RUS.ControlPointsList(),
                       RUS.SurfaceForm(),
                       RUS.UClosed(),
                       RUS.VClosed(),
                       RUS.SelfIntersect(),
                       UKmult,
                       VKmult,
                       UKnots,
                       VKnots,
                       StepGeom_KnotType.StepGeom_ktUnspecified,
                       RUS.WeightsData());

            return MakeBSplineSurface(RBSPL, theLocalFactors);
        }

        if (SS is StepGeom_QuasiUniformSurfaceAndRationalBSplineSurface)
        {
            StepGeom_QuasiUniformSurfaceAndRationalBSplineSurface RQUS = SS as StepGeom_QuasiUniformSurfaceAndRationalBSplineSurface;
            StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface RBSPL = new StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface();

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = RQUS.NbControlPointsListI() - RQUS.UDegree() + 1;
            NCollection_HArray1<int> UKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> UKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                UKmult.SetValue(iU, 1);
                UKnots.SetValue(iU, iU - 1.0);
            }
            UKmult.SetValue(1, RQUS.UDegree() + 1);
            UKmult.SetValue(nbKU, RQUS.UDegree() + 1);

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = RQUS.NbControlPointsListJ() - RQUS.VDegree() + 1;
            NCollection_HArray1<int> VKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> VKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                VKmult.SetValue(iV, 1);
                VKnots.SetValue(iV, iV - 1.0);
            }
            VKmult.SetValue(1, RQUS.VDegree() + 1);
            VKmult.SetValue(nbKV, RQUS.VDegree() + 1);

            // Initialize the BSplineSurfaceWithKnotsAndRationalBSplineSurface
            RBSPL.Init(RQUS.Name(),
                       RQUS.UDegree(),
                       RQUS.VDegree(),
                       RQUS.ControlPointsList(),
                       RQUS.SurfaceForm(),
                       RQUS.UClosed(),
                       RQUS.VClosed(),
                       RQUS.SelfIntersect(),
                       UKmult,
                       VKmult,
                       UKnots,
                       VKnots,
                       StepGeom_KnotType.StepGeom_ktUnspecified,
                       RQUS.WeightsData());
            return MakeBSplineSurface(RBSPL, theLocalFactors);
        }

        return null;
    }

    /// <summary>Converts a bounded_surface (default factors).</summary>
    public static BoundedSurface? MakeBoundedSurface(StepGeom_BoundedSurface SS) => MakeBoundedSurface(SS, new StepData_Factors());

    //=============================================================================
    // Template function for use in MakeBSplineCurve / MakeBSplineCurve2d
    // (its two instantiations: OCCT's template over the point, curve and maker types)
    //=============================================================================

    // The knots and multiplicities of a B-spline curve with its repeated knots merged and its end multiplicities
    // capped at Degree + 1 (the template's common part); null when the curve can't be made.
    private static bool MakeBSplineCurveKnots(StepGeom_BSplineCurve theStepGeom_BSplineCurve,
                                              out StepGeom_BSplineCurveWithKnots aBSplineCurveWithKnots,
                                              out StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve aBSplineCurveWithKnotsAndRationalBSplineCurve,
                                              out NCollection_Array1<double> aUniqueKnots,
                                              out NCollection_Array1<int> aUniqueKnotMultiplicities,
                                              out int aFirstMuultypisityDifference,
                                              out int aLastMuultypisityDifference)
    {
        aBSplineCurveWithKnotsAndRationalBSplineCurve = null;
        aUniqueKnots = null;
        aUniqueKnotMultiplicities = null;
        aFirstMuultypisityDifference = 0;
        aLastMuultypisityDifference = 0;

        if (theStepGeom_BSplineCurve is StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            aBSplineCurveWithKnotsAndRationalBSplineCurve = theStepGeom_BSplineCurve as StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve;
            aBSplineCurveWithKnots = aBSplineCurveWithKnotsAndRationalBSplineCurve.BSplineCurveWithKnots();
        }
        else
        {
            aBSplineCurveWithKnots = theStepGeom_BSplineCurve as StepGeom_BSplineCurveWithKnots;
        }

        int aDegree = aBSplineCurveWithKnots.Degree();
        int NbKnots = aBSplineCurveWithKnots.NbKnotMultiplicities();

        NCollection_HArray1<int> aKnotMultiplicities = aBSplineCurveWithKnots.KnotMultiplicities();
        NCollection_HArray1<double> aKnots = aBSplineCurveWithKnots.Knots();

        // Count number of unique knots
        int NbUniqueKnots = 0;
        double lastKnot = RealFirst();
        for (int i = 1; i <= NbKnots; ++i)
        {
            if (aKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                NbUniqueKnots++;
                lastKnot = aKnots.Value(i);
            }
        }
        if (NbUniqueKnots <= 1)
        {
            return false;
        }
        aUniqueKnots = new NCollection_Array1<double>(1, NbUniqueKnots);
        aUniqueKnotMultiplicities = new NCollection_Array1<int>(1, NbUniqueKnots);
        lastKnot = aKnots.Value(1);
        aUniqueKnots.SetValue(1, aKnots.Value(1));
        aUniqueKnotMultiplicities.SetValue(1, aKnotMultiplicities.Value(1));
        int aKnotPosition = 1;
        for (int i = 2; i <= NbKnots; i++)
        {
            if (aKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                aKnotPosition++;
                aUniqueKnots.SetValue(aKnotPosition, aKnots.Value(i));
                aUniqueKnotMultiplicities.SetValue(aKnotPosition, aKnotMultiplicities.Value(i));
                lastKnot = aKnots.Value(i);
            }
            else
            {
                // Knot not unique, increase multiplicity
                int aCurrentMultiplicity = aUniqueKnotMultiplicities.Value(aKnotPosition);
                aUniqueKnotMultiplicities.SetValue(aKnotPosition, aCurrentMultiplicity + aKnotMultiplicities.Value(i));
            }
        }

        for (int i = 1; i <= NbUniqueKnots; ++i)
        {
            int aCurrentVal = aUniqueKnotMultiplicities.Value(i);
            if (aCurrentVal > aDegree + 1)
            {
                if (i == 1)
                {
                    aFirstMuultypisityDifference = aCurrentVal - aDegree - 1;
                }
                if (i == NbUniqueKnots)
                {
                    aLastMuultypisityDifference = aCurrentVal - aDegree - 1;
                }
                // OCCT_DEBUG output left out
                aCurrentVal = aDegree + 1;
            }
            aUniqueKnotMultiplicities.SetValue(i, aCurrentVal);
        }
        return true;
    }

    // Does the curve descriptor LOOK like a periodic descriptor? (the template's common part)
    private static bool ShouldBePeriodic(NCollection_Array1<int> aUniqueKnotMultiplicities, int NbPoles, int aDegree)
    {
        int NbUniqueKnots = aUniqueKnotMultiplicities.Length();
        int aSummaryMuultypisity = 0;
        for (int i = 1; i <= NbUniqueKnots; i++)
        {
            aSummaryMuultypisity += aUniqueKnotMultiplicities.Value(i);
        }

        bool shouldBePeriodic;
        if (aSummaryMuultypisity == (NbPoles + aDegree + 1))
        {
            shouldBePeriodic = false;
        }
        else if ((aUniqueKnotMultiplicities.Value(1) == aUniqueKnotMultiplicities.Value(NbUniqueKnots))
                 && ((aSummaryMuultypisity - aUniqueKnotMultiplicities.Value(1)) == NbPoles))
        {
            shouldBePeriodic = true;
        }
        else
        {
            // --- What is that ??? ---
            shouldBePeriodic = false;
        }
        return shouldBePeriodic;
    }

    // The weights of the poles kept (the template's common part)
    private static NCollection_Array1<double> UniqueWeights(StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve theCurve, int NbPoles,
                                                            int aFirstMuultypisityDifference, int aLastMuultypisityDifference)
    {
        NCollection_HArray1<double> aWeights = theCurve.WeightsData();
        NCollection_Array1<double> aUniqueWeights =
            new NCollection_Array1<double>(1, NbPoles - aFirstMuultypisityDifference - aLastMuultypisityDifference);
        for (int i = 1 + aFirstMuultypisityDifference; i <= NbPoles - aLastMuultypisityDifference; ++i)
        {
            aUniqueWeights.SetValue(i - aFirstMuultypisityDifference, aWeights.Value(i));
        }
        return aUniqueWeights;
    }

    //=============================================================================
    // Creation d' une BSplineCurve de Geom a partir d' une BSplineCurve de Step
    //=============================================================================

    /// <summary>Converts a B-spline curve with knots (rational or not).</summary>
    public static BSplineCurve3D? MakeBSplineCurve(StepGeom_BSplineCurve theStepGeom_BSplineCurve, in StepData_Factors theLocalFactors)
    {
        if (!MakeBSplineCurveKnots(theStepGeom_BSplineCurve, out StepGeom_BSplineCurveWithKnots aBSplineCurveWithKnots,
                                   out StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve aBSplineCurveWithKnotsAndRationalBSplineCurve,
                                   out NCollection_Array1<double> aUniqueKnots, out NCollection_Array1<int> aUniqueKnotMultiplicities,
                                   out int aFirstMuultypisityDifference, out int aLastMuultypisityDifference))
        {
            return null;
        }

        int aDegree = aBSplineCurveWithKnots.Degree();
        int NbPoles = aBSplineCurveWithKnots.NbControlPointsList();
        NCollection_HArray1<StepGeom_CartesianPoint> aControlPointsList = aBSplineCurveWithKnots.ControlPointsList();
        int aSummaryMuultypisityDifference = aFirstMuultypisityDifference + aLastMuultypisityDifference;
        int NbUniquePoles = NbPoles - aSummaryMuultypisityDifference;
        if (NbUniquePoles <= 0)
        {
            return null;
        }
        NCollection_Array1<Point3D> Poles = new NCollection_Array1<Point3D>(1, NbPoles - aSummaryMuultypisityDifference);

        for (int i = 1 + aFirstMuultypisityDifference; i <= NbPoles - aLastMuultypisityDifference; ++i)
        {
            CartesianPoint? aPoint = MakeCartesianPoint(aControlPointsList.Value(i), theLocalFactors);
            if (aPoint is not null)
            {
                Point3D aGpPnt = aPoint.Pnt();
                Poles.SetValue(i - aFirstMuultypisityDifference, aGpPnt);
            }
            else
            {
                return null;
            }
        }

        bool shouldBePeriodic = ShouldBePeriodic(aUniqueKnotMultiplicities, NbPoles, aDegree);

        BSplineCurve3D aBSplineCurve;
        if (theStepGeom_BSplineCurve is StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            NCollection_Array1<double> aUniqueWeights =
                UniqueWeights(aBSplineCurveWithKnotsAndRationalBSplineCurve, NbPoles, aFirstMuultypisityDifference, aLastMuultypisityDifference);
            aBSplineCurve = new BSplineCurve3D(Poles, aUniqueWeights, aUniqueKnots, aUniqueKnotMultiplicities, aDegree, shouldBePeriodic);
        }
        else
        {
            aBSplineCurve = new BSplineCurve3D(Poles, aUniqueKnots, aUniqueKnotMultiplicities, aDegree, shouldBePeriodic);
        }

        // abv 04.07.00 CAX-IF TRJ4: trj4_k1_top-md-203.stp #716 (face #581):
        // force periodicity on closed curves
        // OCCT: the logical is tested as a C++ bool, so UNKNOWN counts as closed (occt-issues.md, 8D)
        if (theStepGeom_BSplineCurve.ClosedCurve() != StepData_Logical.StepData_LFalse && aBSplineCurve.Degree() > 1 && aBSplineCurve.IsClosed)
        {
            aBSplineCurve.SetPeriodic();
        }
        return aBSplineCurve;
    }

    /// <summary>Converts a B-spline curve with knots (default factors).</summary>
    public static BSplineCurve3D? MakeBSplineCurve(StepGeom_BSplineCurve theStepGeom_BSplineCurve) =>
        MakeBSplineCurve(theStepGeom_BSplineCurve, new StepData_Factors());

    //=============================================================================
    // Creation d' une BSplineCurve de Geom2d a partir d' une
    // BSplineCurveWithKnotsAndRationalBSplineCurve de Step
    //=============================================================================

    /// <summary>Converts a B-spline curve with knots (rational or not) into a 2D curve.</summary>
    public static BSplineCurve2D? MakeBSplineCurve2d(StepGeom_BSplineCurve theStepGeom_BSplineCurve, in StepData_Factors theLocalFactors)
    {
        if (!MakeBSplineCurveKnots(theStepGeom_BSplineCurve, out StepGeom_BSplineCurveWithKnots aBSplineCurveWithKnots,
                                   out StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve aBSplineCurveWithKnotsAndRationalBSplineCurve,
                                   out NCollection_Array1<double> aUniqueKnots, out NCollection_Array1<int> aUniqueKnotMultiplicities,
                                   out int aFirstMuultypisityDifference, out int aLastMuultypisityDifference))
        {
            return null;
        }

        int aDegree = aBSplineCurveWithKnots.Degree();
        int NbPoles = aBSplineCurveWithKnots.NbControlPointsList();
        NCollection_HArray1<StepGeom_CartesianPoint> aControlPointsList = aBSplineCurveWithKnots.ControlPointsList();
        int aSummaryMuultypisityDifference = aFirstMuultypisityDifference + aLastMuultypisityDifference;
        int NbUniquePoles = NbPoles - aSummaryMuultypisityDifference;
        if (NbUniquePoles <= 0)
        {
            return null;
        }
        NCollection_Array1<Point2D> Poles = new NCollection_Array1<Point2D>(1, NbPoles - aSummaryMuultypisityDifference);

        for (int i = 1 + aFirstMuultypisityDifference; i <= NbPoles - aLastMuultypisityDifference; ++i)
        {
            CartesianPoint2? aPoint = MakeCartesianPoint2d(aControlPointsList.Value(i), theLocalFactors);
            if (aPoint is not null)
            {
                Point2D aGpPnt = aPoint.Pnt2d();
                Poles.SetValue(i - aFirstMuultypisityDifference, aGpPnt);
            }
            else
            {
                return null;
            }
        }

        bool shouldBePeriodic = ShouldBePeriodic(aUniqueKnotMultiplicities, NbPoles, aDegree);

        BSplineCurve2D aBSplineCurve;
        if (theStepGeom_BSplineCurve is StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            NCollection_Array1<double> aUniqueWeights =
                UniqueWeights(aBSplineCurveWithKnotsAndRationalBSplineCurve, NbPoles, aFirstMuultypisityDifference, aLastMuultypisityDifference);
            aBSplineCurve = new BSplineCurve2D(Poles, aUniqueWeights, aUniqueKnots, aUniqueKnotMultiplicities, aDegree, shouldBePeriodic);
        }
        else
        {
            aBSplineCurve = new BSplineCurve2D(Poles, aUniqueKnots, aUniqueKnotMultiplicities, aDegree, shouldBePeriodic);
        }

        // abv 04.07.00 CAX-IF TRJ4: trj4_k1_top-md-203.stp #716 (face #581):
        // force periodicity on closed curves
        // OCCT: the logical is tested as a C++ bool, so UNKNOWN counts as closed (occt-issues.md, 8D)
        if (theStepGeom_BSplineCurve.ClosedCurve() != StepData_Logical.StepData_LFalse && aBSplineCurve.Degree() > 1 && aBSplineCurve.IsClosed)
        {
            aBSplineCurve.SetPeriodic();
        }
        return aBSplineCurve;
    }

    /// <summary>Converts a B-spline curve with knots into a 2D curve (default factors).</summary>
    public static BSplineCurve2D? MakeBSplineCurve2d(StepGeom_BSplineCurve theStepGeom_BSplineCurve) =>
        MakeBSplineCurve2d(theStepGeom_BSplineCurve, new StepData_Factors());

    //=============================================================================
    // Creation d' une BSplineSurface de Geom a partir d' une
    // BSplineSurface de Step
    //=============================================================================

    /// <summary>Converts a B-spline surface with knots (rational or not).</summary>
    public static BSplineSurface? MakeBSplineSurface(StepGeom_BSplineSurface SS, in StepData_Factors theLocalFactors)
    {
        int i, j;
        StepGeom_BSplineSurfaceWithKnots BS;
        StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface BSR = null;

        if (SS is StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface)
        {
            BSR = SS as StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface;
            BS = BSR.BSplineSurfaceWithKnots();
        }
        else
        {
            BS = SS as StepGeom_BSplineSurfaceWithKnots;
        }

        int UDeg = BS.UDegree();
        int VDeg = BS.VDegree();
        int NUPoles = BS.NbControlPointsListI();
        int NVPoles = BS.NbControlPointsListJ();
        NCollection_HArray2<StepGeom_CartesianPoint> aControlPointsList = BS.ControlPointsList();
        NCollection_Array2<Point3D> Poles = new NCollection_Array2<Point3D>(1, NUPoles, 1, NVPoles);
        for (i = 1; i <= NUPoles; i++)
        {
            for (j = 1; j <= NVPoles; j++)
            {
                CartesianPoint? P = MakeCartesianPoint(aControlPointsList.Value(i, j), theLocalFactors);
                if (P is not null)
                {
                    Poles.SetValue(i, j, P.Pnt());
                }
                else
                {
                    return null;
                }
            }
        }
        int NUKnots = BS.NbUMultiplicities();
        NCollection_HArray1<int> aUMultiplicities = BS.UMultiplicities();
        NCollection_HArray1<double> aUKnots = BS.UKnots();

        // count number of unique uknots
        double lastKnot = RealFirst();
        int NUKnotsUnique = 0;
        for (i = 1; i <= NUKnots; i++)
        {
            if (aUKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                NUKnotsUnique++;
                lastKnot = aUKnots.Value(i);
            }
        }

        // set umultiplicities and uknots
        NCollection_Array1<int> UMult = new NCollection_Array1<int>(1, NUKnotsUnique);
        NCollection_Array1<double> KUn = new NCollection_Array1<double>(1, NUKnotsUnique);
        int pos = 1;
        lastKnot = aUKnots.Value(1);
        KUn.SetValue(1, aUKnots.Value(1));
        UMult.SetValue(1, aUMultiplicities.Value(1));
        for (i = 2; i <= NUKnots; i++)
        {
            if (aUKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                pos++;
                KUn.SetValue(pos, aUKnots.Value(i));
                UMult.SetValue(pos, aUMultiplicities.Value(i));
                lastKnot = aUKnots.Value(i);
            }
            else
            {
                // Knot not unique, increase multiplicity
                int curMult = UMult.Value(pos);
                UMult.SetValue(pos, curMult + aUMultiplicities.Value(i));
            }
        }
        int NVKnots = BS.NbVMultiplicities();
        NCollection_HArray1<int> aVMultiplicities = BS.VMultiplicities();
        NCollection_HArray1<double> aVKnots = BS.VKnots();

        // count number of unique vknots
        lastKnot = RealFirst();
        int NVKnotsUnique = 0;
        for (i = 1; i <= NVKnots; i++)
        {
            if (aVKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                NVKnotsUnique++;
                lastKnot = aVKnots.Value(i);
            }
        }

        // set vmultiplicities and vknots
        NCollection_Array1<int> VMult = new NCollection_Array1<int>(1, NVKnotsUnique);
        NCollection_Array1<double> KVn = new NCollection_Array1<double>(1, NVKnotsUnique);
        pos = 1;
        lastKnot = aVKnots.Value(1);
        KVn.SetValue(1, aVKnots.Value(1));
        VMult.SetValue(1, aVMultiplicities.Value(1));
        for (i = 2; i <= NVKnots; i++)
        {
            if (aVKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                pos++;
                KVn.SetValue(pos, aVKnots.Value(i));
                VMult.SetValue(pos, aVMultiplicities.Value(i));
                lastKnot = aVKnots.Value(i);
            }
            else
            {
                // Knot not unique, increase multiplicity
                int curMult = VMult.Value(pos);
                VMult.SetValue(pos, curMult + aVMultiplicities.Value(i));
            }
        }

        // --- Does the Surface Descriptor LOOKS like a U and/or V Periodic ---
        // --- Descriptor ? ---

        // --- U Periodic ? ---

        int SumMult = 0;
        for (i = 1; i <= NUKnotsUnique; i++)
        {
            SumMult += UMult.Value(i);
        }

        bool shouldBeUPeriodic = false;
        if (SumMult == (NUPoles + UDeg + 1))
        {
            // shouldBeUPeriodic = false;
        }
        else if ((UMult.Value(1) == UMult.Value(NUKnotsUnique)) && ((SumMult - UMult.Value(1)) == NUPoles))
        {
            shouldBeUPeriodic = true;
        }

        // --- V Periodic ? ---

        SumMult = 0;
        for (i = 1; i <= NVKnotsUnique; i++)
        {
            SumMult += VMult.Value(i);
        }

        bool shouldBeVPeriodic = false;
        if (SumMult == (NVPoles + VDeg + 1))
        {
            // shouldBeVPeriodic = false;
        }
        else if ((VMult.Value(1) == VMult.Value(NVKnotsUnique)) && ((SumMult - VMult.Value(1)) == NVPoles))
        {
            shouldBeVPeriodic = true;
        }

        BSplineSurface CS;
        if (SS is StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface)
        {
            NCollection_HArray2<double> aWeight = BSR.WeightsData();
            NCollection_Array2<double> W = new NCollection_Array2<double>(1, NUPoles, 1, NVPoles);
            for (i = 1; i <= NUPoles; i++)
            {
                for (j = 1; j <= NVPoles; j++)
                {
                    W.SetValue(i, j, aWeight.Value(i, j));
                }
            }
            CS = new BSplineSurface(Poles, W, KUn, KVn, UMult, VMult, UDeg, VDeg, shouldBeUPeriodic, shouldBeVPeriodic);
        }
        else
        {
            CS = new BSplineSurface(Poles, KUn, KVn, UMult, VMult, UDeg, VDeg, shouldBeUPeriodic, shouldBeVPeriodic);
        }
        return CS;
    }

    /// <summary>Converts a B-spline surface with knots (default factors).</summary>
    public static BSplineSurface? MakeBSplineSurface(StepGeom_BSplineSurface SS) => MakeBSplineSurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' un CartesianPoint de Geom a partir d' un CartesianPoint de Step
    //=============================================================================

    /// <summary>Converts a 3D cartesian_point (null if it hasn't three coordinates).</summary>
    public static CartesianPoint? MakeCartesianPoint(StepGeom_CartesianPoint SP, in StepData_Factors theLocalFactors)
    {
        if (SP is not null && SP.NbCoordinates() == 3)
        {
            double LF = theLocalFactors.LengthFactor();
            double X = SP.CoordinatesValue(1) * LF;
            double Y = SP.CoordinatesValue(2) * LF;
            double Z = SP.CoordinatesValue(3) * LF;
            return new CartesianPoint(X, Y, Z);
        }
        return null;
    }

    /// <summary>Converts a 3D cartesian_point (default factors).</summary>
    public static CartesianPoint? MakeCartesianPoint(StepGeom_CartesianPoint SP) => MakeCartesianPoint(SP, new StepData_Factors());

    //=============================================================================
    // Creation d' un CartesianPoint de Geom2d a partir d' un CartesianPoint de
    // Step
    //=============================================================================

    /// <summary>Converts a 2D cartesian_point (null if it hasn't two coordinates). The factors aren't used.</summary>
    public static CartesianPoint2? MakeCartesianPoint2d(StepGeom_CartesianPoint SP, in StepData_Factors theLocalFactors)
    {
        // OCCT: SP isn't checked for null (a null point is a null dereference; occt-issues.md, 8D)
        if (SP.NbCoordinates() == 2)
        {
            double X = SP.CoordinatesValue(1);
            double Y = SP.CoordinatesValue(2);
            return new CartesianPoint2(X, Y);
        }
        return null;
    }

    /// <summary>Converts a 2D cartesian_point.</summary>
    public static CartesianPoint2? MakeCartesianPoint2d(StepGeom_CartesianPoint SP) => MakeCartesianPoint2d(SP, new StepData_Factors());

    //=============================================================================
    // Creation d' un Circle de Geom a partir d' un Circle de Step
    //=============================================================================

    /// <summary>Converts a circle placed by an axis2_placement_3d.</summary>
    public static CircleCurve3D? MakeCircle(StepGeom_Circle SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 2)
        {
            Axis2Placement? A = MakeAxis2Placement(AxisSelect.Value() as StepGeom_Axis2Placement3d, theLocalFactors);
            if (A is not null)
            {
                return new CircleCurve3D(A.Ax2(), SC.Radius() * theLocalFactors.LengthFactor());
            }
        }
        return null;
    }

    /// <summary>Converts a circle (default factors).</summary>
    public static CircleCurve3D? MakeCircle(StepGeom_Circle SC) => MakeCircle(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un Circle de Geom2d a partir d' un Circle de Step
    //=============================================================================

    /// <summary>Converts a circle placed by an axis2_placement_2d.</summary>
    public static CircleCurve2D? MakeCircle2d(StepGeom_Circle SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 1)
        {
            AxisPlacement2? A1 = MakeAxisPlacement(AxisSelect.Value() as StepGeom_Axis2Placement2d, theLocalFactors);
            if (A1 is not null)
            {
                return new CircleCurve2D(A1.Ax2d(), SC.Radius());
            }
        }
        return null;
    }

    /// <summary>Converts a 2D circle (default factors).</summary>
    public static CircleCurve2D? MakeCircle2d(StepGeom_Circle SC) => MakeCircle2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une Conic de Geom a partir d' une Conic de Step
    //=============================================================================

    /// <summary>Converts a conic (circle, ellipse, hyperbola, parabola).</summary>
    public static ConicCurve3D? MakeConic(StepGeom_Conic SC, in StepData_Factors theLocalFactors)
    {
        if (SC is StepGeom_Circle)
        {
            return MakeCircle(SC as StepGeom_Circle, theLocalFactors);
        }
        if (SC is StepGeom_Ellipse)
        {
            return MakeEllipse(SC as StepGeom_Ellipse, theLocalFactors);
        }
        if (SC is StepGeom_Hyperbola)
        {
            return MakeHyperbola(SC as StepGeom_Hyperbola, theLocalFactors);
        }
        if (SC is StepGeom_Parabola)
        {
            return MakeParabola(SC as StepGeom_Parabola, theLocalFactors);
        }
        // Attention : Other conic shall be implemented !
        return null;
    }

    /// <summary>Converts a conic (default factors).</summary>
    public static ConicCurve3D? MakeConic(StepGeom_Conic SC) => MakeConic(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une Conic de Geom2d a partir d' une Conic de Step
    //=============================================================================

    /// <summary>Converts a conic into a 2D conic.</summary>
    public static ConicCurve2D? MakeConic2d(StepGeom_Conic SC, in StepData_Factors theLocalFactors)
    {
        if (SC is StepGeom_Circle)
        {
            return MakeCircle2d(SC as StepGeom_Circle, theLocalFactors);
        }
        if (SC is StepGeom_Ellipse)
        {
            return MakeEllipse2d(SC as StepGeom_Ellipse, theLocalFactors);
        }
        if (SC is StepGeom_Hyperbola)
        {
            return MakeHyperbola2d(SC as StepGeom_Hyperbola, theLocalFactors);
        }
        if (SC is StepGeom_Parabola)
        {
            return MakeParabola2d(SC as StepGeom_Parabola, theLocalFactors);
        }
        // Attention : Other conic shall be implemented !
        return null;
    }

    /// <summary>Converts a conic into a 2D conic (default factors).</summary>
    public static ConicCurve2D? MakeConic2d(StepGeom_Conic SC) => MakeConic2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une ConicalSurface de Geom a partir d' une ConicalSurface de
    // Step
    //=============================================================================

    /// <summary>Converts a conical_surface (its semi-angle at least Precision::Angular()).</summary>
    public static ConicalSurface? MakeConicalSurface(StepGeom_ConicalSurface SS, in StepData_Factors theLocalFactors)
    {
        Axis2Placement? A = MakeAxis2Placement(SS.Position(), theLocalFactors);
        if (A is not null)
        {
            double R = SS.Radius() * theLocalFactors.LengthFactor();
            double Ang = SS.SemiAngle() * theLocalFactors.PlaneAngleFactor();
            // #2(K3-3) rln 12/02/98 ProSTEP ct_turbine-A.stp entity #518, #3571 (gp::Resolution() is too
            // little)
            return new ConicalSurface(new CoordinateSystem(A.Ax2()), Max(Ang, global::Kerf.Engine.Foundation.Precision.Angular()), R);
        }
        return null;
    }

    /// <summary>Converts a conical_surface (default factors).</summary>
    public static ConicalSurface? MakeConicalSurface(StepGeom_ConicalSurface SS) => MakeConicalSurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' une Curve de Geom a partir d' une Curve de Step
    //=============================================================================

    /// <summary>Converts a curve: lines, conics, bounded curves, curve replicas, 3D offset curves and surface curves.</summary>
    public static Curve3D? MakeCurve(StepGeom_Curve SC, in StepData_Factors theLocalFactors)
    {
        if (SC is null)
        {
            return null;
        }
        // OCCT recurses without limit along a file's references (replicas, offsets, trimmed and surface curves): a
        // stack overflow in C++, which ends a .NET process, so the port throws when the stack runs low (decision 1240)
        RuntimeHelpers.EnsureSufficientExecutionStack();
        if (SC is StepGeom_Line)
        {
            return MakeLine(SC as StepGeom_Line, theLocalFactors);
        }
        if (SC is StepGeom_TrimmedCurve)
        {
            return MakeTrimmedCurve(SC as StepGeom_TrimmedCurve, theLocalFactors);
        }
        if (SC is StepGeom_Conic)
        {
            return MakeConic(SC as StepGeom_Conic, theLocalFactors);
        }
        if (SC is StepGeom_BoundedCurve)
        {
            return MakeBoundedCurve(SC as StepGeom_BoundedCurve, theLocalFactors);
        }
        if (SC is StepGeom_CurveReplica)
        { //: n7 abv 16 Feb 99
            StepGeom_CurveReplica CR = SC as StepGeom_CurveReplica;
            StepGeom_Curve PC = CR.ParentCurve();
            StepGeom_CartesianTransformationOperator3d T = CR.Transformation() as StepGeom_CartesianTransformationOperator3d;
            // protect against cyclic references and wrong type of cartop
            if (T is not null && PC != SC)
            {
                Curve3D? C1 = MakeCurve(PC, theLocalFactors);
                if (C1 is not null)
                {
                    Transform3D T1 = new Transform3D();
                    if (MakeTransformation3d(T, ref T1, theLocalFactors))
                    {
                        C1.Transform(T1);
                        return C1;
                    }
                }
            }
        }
        else if (SC is StepGeom_OffsetCurve3d)
        { //: o2 abv 17 Feb 99
            StepGeom_OffsetCurve3d OC = SC as StepGeom_OffsetCurve3d;
            StepGeom_Curve BC = OC.BasisCurve();
            if (BC != SC)
            { // protect against loop
                Curve3D? C1 = MakeCurve(BC, theLocalFactors);
                if (C1 is not null)
                {
                    Direction? RD = MakeDirection(OC.RefDirection());
                    if (RD is not null)
                    {
                        return new OffsetCurve3D(C1, -OC.Distance(), RD.Dir());
                    }
                }
            }
        }
        else if (SC is StepGeom_SurfaceCurve)
        { //: o5 abv 17 Feb 99
            StepGeom_SurfaceCurve SurfC = SC as StepGeom_SurfaceCurve;
            return MakeCurve(SurfC.Curve3d(), theLocalFactors);
        }
        return null;
    }

    /// <summary>Converts a curve (default factors).</summary>
    public static Curve3D? MakeCurve(StepGeom_Curve SC) => MakeCurve(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une Curve de Geom2d a partir d' une Curve de Step
    //=============================================================================

    /// <summary>Converts a curve into a 2D curve: lines, conics, bounded curves and curve replicas.</summary>
    public static Curve2D? MakeCurve2d(StepGeom_Curve SC, in StepData_Factors theLocalFactors)
    {
        // as in MakeCurve (decision 1240)
        RuntimeHelpers.EnsureSufficientExecutionStack();
        if (SC is StepGeom_Line)
        {
            return MakeLine2d(SC as StepGeom_Line, theLocalFactors);
        }
        if (SC is StepGeom_Conic)
        {
            return MakeConic2d(SC as StepGeom_Conic, theLocalFactors);
        }
        if (SC is StepGeom_BoundedCurve)
        {
            return MakeBoundedCurve2d(SC as StepGeom_BoundedCurve, theLocalFactors);
        }
        if (SC is StepGeom_CurveReplica)
        { //: n7 abv 16 Feb 99
            StepGeom_CurveReplica CR = SC as StepGeom_CurveReplica;
            StepGeom_Curve PC = CR.ParentCurve();
            StepGeom_CartesianTransformationOperator2d T = CR.Transformation() as StepGeom_CartesianTransformationOperator2d;
            // protect against cyclic references and wrong type of cartop
            if (T is not null && PC != SC)
            {
                Curve2D? C1 = MakeCurve2d(PC, theLocalFactors);
                if (C1 is not null)
                {
                    Transform2D T1 = new Transform2D();
                    if (MakeTransformation2d(T, ref T1, theLocalFactors))
                    {
                        C1.Transform(T1);
                        return C1;
                    }
                }
            }
        }
        return null;
    }

    /// <summary>Converts a curve into a 2D curve (default factors).</summary>
    public static Curve2D? MakeCurve2d(StepGeom_Curve SC) => MakeCurve2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une CylindricalSurface de Geom a partir d' une
    // CylindricalSurface de Step
    //=============================================================================

    /// <summary>Converts a cylindrical_surface.</summary>
    public static CylindricalSurface? MakeCylindricalSurface(StepGeom_CylindricalSurface SS, in StepData_Factors theLocalFactors)
    {
        Axis2Placement? A = MakeAxis2Placement(SS.Position(), theLocalFactors);
        if (A is not null)
        {
            return new CylindricalSurface(new CoordinateSystem(A.Ax2()), SS.Radius() * theLocalFactors.LengthFactor());
        }
        return null;
    }

    /// <summary>Converts a cylindrical_surface (default factors).</summary>
    public static CylindricalSurface? MakeCylindricalSurface(StepGeom_CylindricalSurface SS) =>
        MakeCylindricalSurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' un Direction de Geom a partir d' un Direction de Step
    //=============================================================================

    /// <summary>Converts a 3D direction (null if it has fewer than three ratios, an infinite one, or a null magnitude).</summary>
    public static Direction? MakeDirection(StepGeom_Direction SD)
    {
        // OCCT: SD isn't checked for null (occt-issues.md, 8D)
        if (SD.NbDirectionRatios() >= 3)
        {
            double X = SD.DirectionRatiosValue(1);
            double Y = SD.DirectionRatiosValue(2);
            double Z = SD.DirectionRatiosValue(3);
            // 5.08.2021. Unstable test bugs xde bug24759: Y is very large value - FPE in SquareModulus
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(Y) || global::Kerf.Engine.Foundation.Precision.IsInfinite(Z))
            {
                return null;
            }
            // sln 22.10.2001. CTS23496: Direction is not created if it has null magnitude
            if (new Xyz(X, Y, Z).SquareModulus() > Constants.Resolution() * Constants.Resolution())
            {
                return new Direction(X, Y, Z);
            }
        }
        return null;
    }

    //=============================================================================
    // Creation d' un Direction de Geom2d a partir d' un Direction de Step
    //=============================================================================

    /// <summary>Converts a 2D direction (null if it has fewer than two ratios or a null magnitude).</summary>
    public static Direction2? MakeDirection2d(StepGeom_Direction SD)
    {
        // OCCT: SD isn't checked for null (occt-issues.md, 8D)
        if (SD.NbDirectionRatios() >= 2)
        {
            double X = SD.DirectionRatiosValue(1);
            double Y = SD.DirectionRatiosValue(2);
            // sln 23.10.2001. CTS23496: Direction is not created if it has null magnitude
            if (new Xy(X, Y).SquareModulus() > Constants.Resolution() * Constants.Resolution())
            {
                return new Direction2(X, Y);
            }
        }
        return null;
    }

    //=============================================================================
    // Creation d' une ElementarySurface de Geom a partir d' une
    // ElementarySurface de Step
    //=============================================================================

    /// <summary>Converts an elementary surface (plane, cylinder, cone, sphere, torus).</summary>
    public static ElementarySurface? MakeElementarySurface(StepGeom_ElementarySurface SS, in StepData_Factors theLocalFactors)
    {
        if (SS is StepGeom_Plane)
        {
            return MakePlane(SS as StepGeom_Plane, theLocalFactors);
        }
        if (SS is StepGeom_CylindricalSurface)
        {
            return MakeCylindricalSurface(SS as StepGeom_CylindricalSurface, theLocalFactors);
        }
        if (SS is StepGeom_ConicalSurface)
        {
            return MakeConicalSurface(SS as StepGeom_ConicalSurface, theLocalFactors);
        }
        if (SS is StepGeom_SphericalSurface)
        {
            return MakeSphericalSurface(SS as StepGeom_SphericalSurface, theLocalFactors);
        }
        if (SS is StepGeom_ToroidalSurface)
        {
            return MakeToroidalSurface(SS as StepGeom_ToroidalSurface, theLocalFactors);
        }
        return null;
    }

    /// <summary>Converts an elementary surface (default factors).</summary>
    public static ElementarySurface? MakeElementarySurface(StepGeom_ElementarySurface SS) =>
        MakeElementarySurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' un Ellipse de Geom a partir d' un Ellipse de Step
    //=============================================================================

    /// <summary>Converts an ellipse (its axes swapped when the first semi-axis is the smaller).</summary>
    public static EllipseCurve3D? MakeEllipse(StepGeom_Ellipse SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 2)
        {
            Axis2Placement? A1 = MakeAxis2Placement(AxisSelect.Value() as StepGeom_Axis2Placement3d, theLocalFactors);
            if (A1 is not null)
            {
                Frame3D A = A1.Ax2();
                double LF = theLocalFactors.LengthFactor();
                double majorR = SC.SemiAxis1() * LF;
                double minorR = SC.SemiAxis2() * LF;
                if (majorR - minorR >= 0.0)
                { //: o9 abv 19 Feb 99
                    return new EllipseCurve3D(A, majorR, minorR);
                }
                //: o9 abv 19 Feb 99
                else
                {
                    A.SetXDirection(A.XDirection() ^ A.Direction());
                    return new EllipseCurve3D(A, minorR, majorR);
                }
            }
        }
        return null;
    }

    /// <summary>Converts an ellipse (default factors).</summary>
    public static EllipseCurve3D? MakeEllipse(StepGeom_Ellipse SC) => MakeEllipse(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un Ellipse de Geom2d a partir d' un Ellipse de Step
    //=============================================================================

    /// <summary>Converts an ellipse into a 2D ellipse.</summary>
    public static EllipseCurve2D? MakeEllipse2d(StepGeom_Ellipse SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 1)
        {
            AxisPlacement2? A1 = MakeAxisPlacement(AxisSelect.Value() as StepGeom_Axis2Placement2d, theLocalFactors);
            if (A1 is not null)
            {
                Frame2D A = new Frame2D(A1.Ax2d());
                double majorR = SC.SemiAxis1();
                double minorR = SC.SemiAxis2();
                if (majorR - minorR >= 0.0)
                { //: o9 abv 19 Feb 99: bm4_id_punch_b.stp #678: protection
                    return new EllipseCurve2D(A, majorR, minorR);
                }
                else
                {
                    Direction2D X = A.XDirection();
                    A.SetXDirection(new Direction2D(X.X(), -X.Y()));
                    return new EllipseCurve2D(A, minorR, majorR);
                }
            }
        }
        return null;
    }

    /// <summary>Converts an ellipse into a 2D ellipse (default factors).</summary>
    public static EllipseCurve2D? MakeEllipse2d(StepGeom_Ellipse SC) => MakeEllipse2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un Hyperbola de Geom a partir d' un Hyperbola de Step
    //=============================================================================

    /// <summary>Converts a hyperbola.</summary>
    public static HyperbolaCurve3D? MakeHyperbola(StepGeom_Hyperbola SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 2)
        {
            Axis2Placement? A1 = MakeAxis2Placement(AxisSelect.Value() as StepGeom_Axis2Placement3d, theLocalFactors);
            if (A1 is not null)
            {
                Frame3D A = A1.Ax2();
                double LF = theLocalFactors.LengthFactor();
                return new HyperbolaCurve3D(A, SC.SemiAxis() * LF, SC.SemiImagAxis() * LF);
            }
        }
        return null;
    }

    /// <summary>Converts a hyperbola (default factors).</summary>
    public static HyperbolaCurve3D? MakeHyperbola(StepGeom_Hyperbola SC) => MakeHyperbola(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un Hyperbola de Geom2d a partir d' un Hyperbola de Step
    //=============================================================================

    /// <summary>Converts a hyperbola into a 2D hyperbola.</summary>
    public static HyperbolaCurve2D? MakeHyperbola2d(StepGeom_Hyperbola SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 1)
        {
            AxisPlacement2? A1 = MakeAxisPlacement(AxisSelect.Value() as StepGeom_Axis2Placement2d, theLocalFactors);
            if (A1 is not null)
            {
                Frame2D A = new Frame2D(A1.Ax2d());
                return new HyperbolaCurve2D(A, SC.SemiAxis(), SC.SemiImagAxis());
            }
        }
        return null;
    }

    /// <summary>Converts a hyperbola into a 2D hyperbola (default factors).</summary>
    public static HyperbolaCurve2D? MakeHyperbola2d(StepGeom_Hyperbola SC) => MakeHyperbola2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une Line de Geom a partir d' une Line de Step
    //=============================================================================

    /// <summary>Converts a line (null if its vector is shorter than Precision::Confusion()).</summary>
    public static LineCurve3D? MakeLine(StepGeom_Line SC, in StepData_Factors theLocalFactors)
    {
        CartesianPoint? P = MakeCartesianPoint(SC.Pnt(), theLocalFactors);
        if (P is not null)
        {
            // sln 22.10.2001. CTS23496: Line is not created if direction have not been successfully created
            VectorWithMagnitude? D = MakeVectorWithMagnitude(SC.Dir(), theLocalFactors);
            if (D is not null)
            {
                if (D.Vec().SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return null;
                }
                Direction3D V = new Direction3D(D.Vec());
                return new LineCurve3D(P.Pnt(), V);
            }
        }
        return null;
    }

    /// <summary>Converts a line (default factors).</summary>
    public static LineCurve3D? MakeLine(StepGeom_Line SC) => MakeLine(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' une Line de Geom2d a partir d' une Line de Step
    //=============================================================================

    /// <summary>Converts a line into a 2D line.</summary>
    public static LineCurve2D? MakeLine2d(StepGeom_Line SC, in StepData_Factors theLocalFactors)
    {
        CartesianPoint2? P = MakeCartesianPoint2d(SC.Pnt(), theLocalFactors);
        if (P is not null)
        {
            // sln 23.10.2001. CTS23496: Line is not created if direction have not been successfully created
            VectorWithMagnitude2? D = MakeVectorWithMagnitude2d(SC.Dir());
            if (D is not null)
            {
                Direction2D D1 = new Direction2D(D.Vec2d());
                return new LineCurve2D(P.Pnt2d(), D1);
            }
        }
        return null;
    }

    /// <summary>Converts a line into a 2D line (default factors).</summary>
    public static LineCurve2D? MakeLine2d(StepGeom_Line SC) => MakeLine2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un Parabola de Geom a partir d' un Parabola de Step
    //=============================================================================

    /// <summary>Converts a parabola.</summary>
    public static ParabolaCurve3D? MakeParabola(StepGeom_Parabola SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 2)
        {
            Axis2Placement? A = MakeAxis2Placement(AxisSelect.Value() as StepGeom_Axis2Placement3d, theLocalFactors);
            if (A is not null)
            {
                return new ParabolaCurve3D(A.Ax2(), SC.FocalDist() * theLocalFactors.LengthFactor());
            }
        }
        return null;
    }

    /// <summary>Converts a parabola (default factors).</summary>
    public static ParabolaCurve3D? MakeParabola(StepGeom_Parabola SC) => MakeParabola(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un Parabola de Geom2d a partir d' un Parabola de Step
    //=============================================================================

    /// <summary>Converts a parabola into a 2D parabola.</summary>
    public static ParabolaCurve2D? MakeParabola2d(StepGeom_Parabola SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement AxisSelect = SC.Position();
        if (AxisSelect.CaseNum(AxisSelect.Value()) == 1)
        {
            AxisPlacement2? A1 = MakeAxisPlacement(AxisSelect.Value() as StepGeom_Axis2Placement2d, theLocalFactors);
            if (A1 is not null)
            {
                Frame2D A = new Frame2D(A1.Ax2d());
                return new ParabolaCurve2D(A, SC.FocalDist());
            }
        }
        return null;
    }

    /// <summary>Converts a parabola into a 2D parabola (default factors).</summary>
    public static ParabolaCurve2D? MakeParabola2d(StepGeom_Parabola SC) => MakeParabola2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un Plane de Geom a partir d' un plane de Step
    //=============================================================================

    /// <summary>Converts a plane.</summary>
    public static PlaneSurface? MakePlane(StepGeom_Plane SP, in StepData_Factors theLocalFactors)
    {
        Axis2Placement? A = MakeAxis2Placement(SP.Position(), theLocalFactors);
        if (A is not null)
        {
            return new PlaneSurface(new CoordinateSystem(A.Ax2()));
        }
        return null;
    }

    /// <summary>Converts a plane (default factors).</summary>
    public static PlaneSurface? MakePlane(StepGeom_Plane SP) => MakePlane(SP, new StepData_Factors());

    //=================================================================================================

    /// <summary>Converts a polyline into a B-spline curve of degree 1.</summary>
    public static BSplineCurve3D? MakePolyline(StepGeom_Polyline SPL, in StepData_Factors theLocalFactors)
    {
        if (SPL is null)
        {
            return null;
        }

        int nbp = SPL.NbPoints();
        if (nbp > 1)
        {
            NCollection_Array1<Point3D> Poles = new NCollection_Array1<Point3D>(1, nbp);
            NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, nbp);
            NCollection_Array1<int> Mults = new NCollection_Array1<int>(1, nbp);

            for (int i = 1; i <= nbp; i++)
            {
                CartesianPoint? P = MakeCartesianPoint(SPL.PointsValue(i), theLocalFactors);
                if (P is not null)
                {
                    Poles.SetValue(i, P.Pnt());
                }
                else
                {
                    return null;
                }
                Knots.SetValue(i, (double)(i - 1));
                Mults.SetValue(i, 1);
            }
            Mults.SetValue(1, 2);
            Mults.SetValue(nbp, 2);

            return new BSplineCurve3D(Poles, Knots, Mults, 1);
        }
        return null;
    }

    /// <summary>Converts a polyline (default factors).</summary>
    public static BSplineCurve3D? MakePolyline(StepGeom_Polyline SPL) => MakePolyline(SPL, new StepData_Factors());

    //=================================================================================================

    /// <summary>Converts a polyline into a 2D B-spline curve of degree 1.</summary>
    public static BSplineCurve2D? MakePolyline2d(StepGeom_Polyline SPL, in StepData_Factors theLocalFactors)
    {
        if (SPL is null)
        {
            return null;
        }

        int nbp = SPL.NbPoints();
        if (nbp > 1)
        {
            NCollection_Array1<Point2D> Poles = new NCollection_Array1<Point2D>(1, nbp);
            NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, nbp);
            NCollection_Array1<int> Mults = new NCollection_Array1<int>(1, nbp);

            for (int i = 1; i <= nbp; i++)
            {
                CartesianPoint2? P = MakeCartesianPoint2d(SPL.PointsValue(i), theLocalFactors);
                if (P is not null)
                {
                    Poles.SetValue(i, P.Pnt2d());
                }
                else
                {
                    return null;
                }
                Knots.SetValue(i, (double)(i - 1));
                Mults.SetValue(i, 1);
            }
            Mults.SetValue(1, 2);
            Mults.SetValue(nbp, 2);

            return new BSplineCurve2D(Poles, Knots, Mults, 1);
        }
        return null;
    }

    /// <summary>Converts a polyline into a 2D curve (default factors).</summary>
    public static BSplineCurve2D? MakePolyline2d(StepGeom_Polyline SPL) => MakePolyline2d(SPL, new StepData_Factors());

    //=============================================================================
    // Creation d' une RectangularTrimmedSurface de Geom a partir d' une
    // RectangularTrimmedSurface de Step
    //=============================================================================

    /// <summary>Converts a rectangular_trimmed_surface (its parameters scaled by the basis surface's units).</summary>
    public static TrimmedSurface? MakeRectangularTrimmedSurface(StepGeom_RectangularTrimmedSurface SS, in StepData_Factors theLocalFactors)
    {
        Surface? theBasis = MakeSurface(SS.BasisSurface(), theLocalFactors);
        if (theBasis is not null)
        {
            // -----------------------------------------
            // Modification of the Trimming Parameters ?
            // -----------------------------------------

            double uFact = 1.0;
            double vFact = 1.0;
            double LengthFact = theLocalFactors.LengthFactor();
            double AngleFact = theLocalFactors.PlaneAngleFactor(); // abv 30.06.00 trj4_k1_geo-tc-214.stp #1477: PI/180.;

            if (theBasis is SphericalSurface || theBasis is ToroidalSurface)
            {
                uFact = vFact = AngleFact;
            }
            else if (theBasis is CylindricalSurface)
            {
                uFact = AngleFact;
                vFact = LengthFact;
            }
            else if (theBasis is RevolutionSurface)
            {
                uFact = AngleFact;
            }
            else if (theBasis is ConicalSurface)
            {
                ConicalSurface conicS = theBasis as ConicalSurface;
                uFact = AngleFact;
                vFact = LengthFact / Math.Cos(conicS.SemiAngle());
            }
            else if (theBasis is PlaneSurface)
            {
                uFact = vFact = LengthFact;
            }

            double U1 = SS.U1() * uFact;
            double U2 = SS.U2() * uFact;
            double V1 = SS.V1() * vFact;
            double V2 = SS.V2() * vFact;

            return new TrimmedSurface(theBasis, U1, U2, V1, V2, SS.Usense(), SS.Vsense());
        }
        return null;
    }

    /// <summary>Converts a rectangular_trimmed_surface (default factors).</summary>
    public static TrimmedSurface? MakeRectangularTrimmedSurface(StepGeom_RectangularTrimmedSurface SS) =>
        MakeRectangularTrimmedSurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' une SphericalSurface de Geom a partir d' une
    // SphericalSurface de Step
    //=============================================================================

    /// <summary>Converts a spherical_surface.</summary>
    public static SphericalSurface? MakeSphericalSurface(StepGeom_SphericalSurface SS, in StepData_Factors theLocalFactors)
    {
        Axis2Placement? A = MakeAxis2Placement(SS.Position(), theLocalFactors);
        if (A is not null)
        {
            return new SphericalSurface(new CoordinateSystem(A.Ax2()), SS.Radius() * theLocalFactors.LengthFactor());
        }
        return null;
    }

    /// <summary>Converts a spherical_surface (default factors).</summary>
    public static SphericalSurface? MakeSphericalSurface(StepGeom_SphericalSurface SS) => MakeSphericalSurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' une Surface de Geom a partir d' une Surface de Step
    //=============================================================================

    /// <summary>
    /// Converts a surface: bounded, elementary and swept surfaces, offset surfaces (a C0 basis made C1 through Shape
    /// Healing's <see cref="ShapeAlgo.AlgoContainer"/>) and surface replicas. Null on any failure.
    /// </summary>
    public static Surface? MakeSurface(StepGeom_Surface SS, in StepData_Factors theLocalFactors)
    {
        // sln 01.10.2001 BUC61003. If entry shell is NULL do nothing
        if (SS is null)
        {
            return null;
        }

        try
        {
            // OCC_CATCH_SIGNALS
            // as in MakeCurve (decision 1240): the exception is caught below, as OCCT catches its stack overflow
            RuntimeHelpers.EnsureSufficientExecutionStack();
            if (SS is StepGeom_BoundedSurface)
            {
                return MakeBoundedSurface(SS as StepGeom_BoundedSurface, theLocalFactors);
            }
            if (SS is StepGeom_ElementarySurface)
            {
                StepGeom_ElementarySurface S1 = SS as StepGeom_ElementarySurface;
                if (S1.Position() is null)
                {
                    return null;
                }

                return MakeElementarySurface(S1, theLocalFactors);
            }
            if (SS is StepGeom_SweptSurface)
            {
                return MakeSweptSurface(SS as StepGeom_SweptSurface, theLocalFactors);
            }
            if (SS is StepGeom_OffsetSurface)
            { //: d4 abv 12 Mar 98
                StepGeom_OffsetSurface OS = SS as StepGeom_OffsetSurface;

                Surface? aBasisSurface = MakeSurface(OS.BasisSurface(), theLocalFactors);
                if (aBasisSurface is not null)
                {
                    // sln 03.10.01. BUC61003. creation of  offset surface is corrected
                    double anOffset = OS.Distance() * theLocalFactors.LengthFactor();
                    if (aBasisSurface.Continuity() == C0)
                    {
                        MakeFace aBFace = new MakeFace(aBasisSurface, global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (aBFace.IsDone)
                        {
                            // OCCT: the process-wide container (null before ShapeAlgo.Init, as in OCCT)
                            Shape aResult = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().C0ShapeToC1Shape(aBFace.Face(), Math.Abs(anOffset));
                            if (aResult.ShapeType() == ShapeType.Face)
                            {
                                aBasisSurface = Tool.Surface(ShapeCasts.Face(aResult));
                            }
                        }
                    }
                    if (aBasisSurface.Continuity() != C0)
                    {
                        return new OffsetSurface(aBasisSurface, anOffset);
                    }
                }
            }
            else if (SS is StepGeom_SurfaceReplica)
            { //: n7 abv 16 Feb 99
                StepGeom_SurfaceReplica SR = SS as StepGeom_SurfaceReplica;
                StepGeom_Surface PS = SR.ParentSurface();
                StepGeom_CartesianTransformationOperator3d T = SR.Transformation();
                // protect against cyclic references and wrong type of cartop
                if (T is not null && PS != SS)
                {
                    Surface? S1 = MakeSurface(PS, theLocalFactors);
                    if (S1 is not null)
                    {
                        Transform3D T1 = new Transform3D();
                        if (MakeTransformation3d(T, ref T1, theLocalFactors))
                        {
                            S1.Transform(T1);
                            return S1;
                        }
                    }
                }
            }
        }
        // OCCT catches Standard_Failure under OCC_CATCH_SIGNALS, so system signals too: their .NET counterparts are
        // the other exceptions (decision 1183)
        catch (Exception anException) when (anException is not OutOfMemoryException and not OperationCanceledException)
        {
            // OCCT_DEBUG output left out
        }
        return null;
    }

    /// <summary>Converts a surface (default factors).</summary>
    public static Surface? MakeSurface(StepGeom_Surface SS) => MakeSurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' une SurfaceOfLinearExtrusion de Geom a partir d' une
    // SurfaceOfLinearExtrusion de Step
    //=============================================================================

    /// <summary>Converts a surface_of_linear_extrusion (null for a line extruded along itself).</summary>
    public static ExtrusionSurface? MakeSurfaceOfLinearExtrusion(StepGeom_SurfaceOfLinearExtrusion SS, in StepData_Factors theLocalFactors)
    {
        Curve3D? C = MakeCurve(SS.SweptCurve(), theLocalFactors);
        if (C is not null)
        {
            // sln 23.10.2001. CTS23496: Surface is not created if extrusion axis have not been successfully
            // created
            VectorWithMagnitude? V = MakeVectorWithMagnitude(SS.ExtrusionAxis(), theLocalFactors);
            if (V is not null)
            {
                Direction3D D = new Direction3D(V.Vec());
                LineCurve3D aLine = C as LineCurve3D;
                if (aLine is not null && aLine.Lin().Direction().IsParallel(D, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return null;
                }
                return new ExtrusionSurface(C, D);
            }
        }
        return null;
    }

    /// <summary>Converts a surface_of_linear_extrusion (default factors).</summary>
    public static ExtrusionSurface? MakeSurfaceOfLinearExtrusion(StepGeom_SurfaceOfLinearExtrusion SS) =>
        MakeSurfaceOfLinearExtrusion(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' une SurfaceOfRevolution de Geom a partir d' une
    // SurfaceOfRevolution de Step
    //=============================================================================

    /// <summary>Converts a surface_of_revolution (a circle or ellipse through the axis trimmed to a half).</summary>
    public static RevolutionSurface? MakeSurfaceOfRevolution(StepGeom_SurfaceOfRevolution SS, in StepData_Factors theLocalFactors)
    {
        Curve3D? C = MakeCurve(SS.SweptCurve(), theLocalFactors);
        if (C is not null)
        {
            Axis1Placement? A1 = MakeAxis1Placement(SS.AxisPosition(), theLocalFactors);
            if (A1 is not null)
            {
                Axis3D A = A1.Ax1();
                // skl for OCC952 (one bad case revolution of circle)
                if (C is CircleCurve3D || C is EllipseCurve3D)
                {
                    ConicCurve3D conic = C as ConicCurve3D;
                    Point3D pc = conic.Location();
                    Line3D rl = new Line3D(A);
                    if (rl.Distance(pc) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    { // pc lies on A2
                        Direction3D dirline = A.Direction();
                        Direction3D norm = conic.Axis().Direction();
                        Direction3D xAxis = conic.XAxis().Direction();
                        // checking A2 lies on plane of circle
                        if (dirline.IsNormal(norm, global::Kerf.Engine.Foundation.Precision.Angular())
                            && (dirline.IsParallel(xAxis, global::Kerf.Engine.Foundation.Precision.Angular()) || C is CircleCurve3D))
                        {
                            // change parametrization for trimming
                            Frame3D axnew = new Frame3D(pc, norm, dirline.Reversed());
                            conic.SetPosition(axnew);
                            C = new TrimmedCurve3D(conic, 0.0, M_PI);
                        }
                    }
                }
                return new RevolutionSurface(C, A);
            }
        }
        return null;
    }

    /// <summary>Converts a surface_of_revolution (default factors).</summary>
    public static RevolutionSurface? MakeSurfaceOfRevolution(StepGeom_SurfaceOfRevolution SS) =>
        MakeSurfaceOfRevolution(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' une SweptSurface de prostep a partir d' une
    // SweptSurface de Geom
    //=============================================================================

    /// <summary>Converts a swept surface (linear extrusion or revolution).</summary>
    public static SweptSurface? MakeSweptSurface(StepGeom_SweptSurface SS, in StepData_Factors theLocalFactors)
    {
        if (SS is StepGeom_SurfaceOfLinearExtrusion)
        {
            return MakeSurfaceOfLinearExtrusion(SS as StepGeom_SurfaceOfLinearExtrusion, theLocalFactors);
        }
        if (SS is StepGeom_SurfaceOfRevolution)
        {
            return MakeSurfaceOfRevolution(SS as StepGeom_SurfaceOfRevolution, theLocalFactors);
        }
        return null;
    }

    /// <summary>Converts a swept surface (default factors).</summary>
    public static SweptSurface? MakeSweptSurface(StepGeom_SweptSurface SS) => MakeSweptSurface(SS, new StepData_Factors());

    //=============================================================================
    // Creation d' une ToroidalSurface de Geom a partir d' une
    // ToroidalSurface de Step
    //=============================================================================

    /// <summary>Converts a toroidal_surface (its radii taken positive).</summary>
    public static ToroidalSurface? MakeToroidalSurface(StepGeom_ToroidalSurface SS, in StepData_Factors theLocalFactors)
    {
        Axis2Placement? A = MakeAxis2Placement(SS.Position(), theLocalFactors);
        if (A is not null)
        {
            double LF = theLocalFactors.LengthFactor();
            return new ToroidalSurface(new CoordinateSystem(A.Ax2()), Math.Abs(SS.MajorRadius() * LF), Math.Abs(SS.MinorRadius() * LF));
        }
        return null;
    }

    /// <summary>Converts a toroidal_surface (default factors).</summary>
    public static ToroidalSurface? MakeToroidalSurface(StepGeom_ToroidalSurface SS) => MakeToroidalSurface(SS, new StepData_Factors());

    //=================================================================================================

    /// <summary>
    /// Converts a 2D cartesian transformation operator into the rigid motion it describes (its inverse, as OCCT
    /// takes it); false if its origin can't be converted (<paramref name="CT"/> unchanged).
    /// </summary>
    public static bool MakeTransformation2d(StepGeom_CartesianTransformationOperator2d SCTO, ref Transform2D CT, in StepData_Factors theLocalFactors)
    {
        //  NB : on ne s interesse ici qu au deplacement rigide
        CartesianPoint2? CP = MakeCartesianPoint2d(SCTO.LocalOrigin(), theLocalFactors);
        if (CP is not null)
        {
            Direction2D D1 = new Direction2D(Direction2D.D.X);
            // sln 23.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used
            StepGeom_Direction A = SCTO.Axis1();
            if (A is not null)
            {
                Direction2? D = MakeDirection2d(A);
                if (D is not null)
                {
                    D1 = D.Dir2d();
                }
            }
            Axis2D result = new Axis2D(CP.Pnt2d(), D1);
            CT.SetTransformation(result);
            CT = CT.Inverted();
            return true;
        }
        return false;
    }

    /// <summary>Converts a 2D cartesian transformation operator (default factors).</summary>
    public static bool MakeTransformation2d(StepGeom_CartesianTransformationOperator2d SCTO, ref Transform2D CT) =>
        MakeTransformation2d(SCTO, ref CT, new StepData_Factors());

    //=================================================================================================

    /// <summary>
    /// Converts a 3D cartesian transformation operator into the transformation it describes (its inverse, as OCCT
    /// takes it, with its scale); false if its origin can't be converted (<paramref name="CT"/> unchanged).
    /// </summary>
    public static bool MakeTransformation3d(StepGeom_CartesianTransformationOperator3d SCTO, ref Transform3D CT, in StepData_Factors theLocalFactors)
    {
        CartesianPoint? CP = MakeCartesianPoint(SCTO.LocalOrigin(), theLocalFactors);
        if (CP is not null)
        {
            Point3D Pgp = CP.Pnt();

            // sln 23.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used
            Direction3D D1 = new Direction3D(Direction3D.D.X);
            StepGeom_Direction A1 = SCTO.Axis1();
            if (A1 is not null)
            {
                Direction? D = MakeDirection(A1);
                if (D is not null)
                {
                    D1 = D.Dir();
                }
            }

            Direction3D D2 = new Direction3D(Direction3D.D.Y);
            StepGeom_Direction A2 = SCTO.Axis2();
            if (A2 is not null)
            {
                Direction? D = MakeDirection(A2);
                if (D is not null)
                {
                    D2 = D.Dir();
                }
            }

            bool isDefaultDirectionUsed = true;
            Direction3D D3 = new Direction3D();
            StepGeom_Direction A3 = SCTO.Axis3();
            if (A3 is not null)
            {
                Direction? D = MakeDirection(A3);
                if (D is not null)
                {
                    D3 = D.Dir();
                    isDefaultDirectionUsed = false;
                }
            }
            if (isDefaultDirectionUsed)
            {
                D3 = D1.Crossed(D2);
            }

            CoordinateSystem result = new CoordinateSystem(Pgp, D3, D1);
            CT.SetTransformation(result);
            if (SCTO.HasScale())
            {
                CT.SetScaleFactor(SCTO.Scale());
            }
            CT = CT.Inverted(); //: n8 abv 16 Feb 99: tr8_as2_db.stp: reverse for accordance with LV tool
            return true;
        }
        return false;
    }

    /// <summary>Converts a 3D cartesian transformation operator (default factors).</summary>
    public static bool MakeTransformation3d(StepGeom_CartesianTransformationOperator3d SCTO, ref Transform3D CT) =>
        MakeTransformation3d(SCTO, ref CT, new StepData_Factors());

    // ----------------------------------------------------------------
    // ExtractParameter
    // ----------------------------------------------------------------
    //: o6 abv 18 Feb 99: parameter Factor added
    //: p3 abv 23 Feb 99: parameter Shift added
    private static bool ExtractParameter(Curve3D aGeomCurve,
                                         NCollection_HArray1<StepGeom_TrimmingSelect> TS,
                                         int nbSel,
                                         int MasterRep,
                                         double Factor,
                                         double Shift,
                                         ref double aParam,
                                         in StepData_Factors theLocalFactors)
    {
        StepGeom_CartesianPoint aPoint;
        int i;
        //: S4136  double precBrep = BRepAPI::Precision();
        for (i = 1; i <= nbSel; i++)
        {
            StepGeom_TrimmingSelect theSel = TS.Value(i);
            if (MasterRep == 2 && theSel.CaseMember() > 0)
            {
                aParam = Shift + Factor * theSel.ParameterValue();
                return true;
            }
            else if (MasterRep == 1 && theSel.CaseNumber() > 0)
            {
                aPoint = theSel.CartesianPoint();
                CartesianPoint? theGeomPnt = MakeCartesianPoint(aPoint, theLocalFactors);
                // OCCT: a point that isn't 3D gives a null point here, dereferenced (occt-issues.md, 8D)
                Point3D thegpPnt = theGeomPnt.Pnt();

                //: S4136: use advanced algorithm
                CurveAnalysis sac = new CurveAnalysis();
                sac.Project(aGeomCurve, thegpPnt, global::Kerf.Engine.Foundation.Precision.Confusion(), out Point3D p, out aParam);
                return true;
            }
        }
        // if the MasterRepresentation is unspecified:
        // if a ParameterValue exists, it is preferred

        for (i = 1; i <= nbSel; i++)
        {
            StepGeom_TrimmingSelect theSel = TS.Value(i);
            if (theSel.CaseMember() > 0)
            {
                aParam = Shift + Factor * theSel.ParameterValue();

                return true;
            }
        }
        // if no ParameterValue exists, it is created from the CartesianPointValue

        for (i = 1; i <= nbSel; i++)
        {
            StepGeom_TrimmingSelect theSel = TS.Value(i);
            if (theSel.CaseNumber() > 0)
            {
                aPoint = theSel.CartesianPoint();
                CartesianPoint? theGeomPnt = MakeCartesianPoint(aPoint, theLocalFactors);
                // OCCT: as above, a point that isn't 3D is dereferenced null
                Point3D thegpPnt = theGeomPnt.Pnt();
                // Project Point On Curve
                CurveAnalysis sac = new CurveAnalysis();
                sac.Project(aGeomCurve, thegpPnt, global::Kerf.Engine.Foundation.Precision.Confusion(), out Point3D p, out aParam);
                return true;
            }
        }
        return false; // I suppose
    }

    //=============================================================================
    // Creation d' une Trimmed Curve de Geom a partir d' une Trimmed Curve de Step
    //=============================================================================

    /// <summary>
    /// Converts a trimmed_curve: its trims taken as parameters (scaled by the basis curve's units) or as points
    /// projected on the curve, as its master representation says.
    /// </summary>
    public static TrimmedCurve3D? MakeTrimmedCurve(StepGeom_TrimmedCurve SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Curve theSTEPCurve = SC.BasisCurve();
        Curve3D? theCurve = MakeCurve(theSTEPCurve, theLocalFactors);
        if (theCurve is null)
        {
            return null;
        }

        NCollection_HArray1<StepGeom_TrimmingSelect> theTrimSel1 = SC.Trim1();
        NCollection_HArray1<StepGeom_TrimmingSelect> theTrimSel2 = SC.Trim2();
        int nbSel1 = SC.NbTrim1();
        int nbSel2 = SC.NbTrim2();

        int MasterRep;
        switch (SC.MasterRepresentation())
        {
            case StepGeom_TrimmingPreference.StepGeom_tpCartesian:
                MasterRep = 1;
                break;
            case StepGeom_TrimmingPreference.StepGeom_tpParameter:
                MasterRep = 2;
                break;
            default:
                MasterRep = 0;
                break;
        }

        // gka 18.02.04 analysis for case when MasterRep = .Unspecified
        // and parameters are specified as CARTESIAN_POINT
        bool isPoint = false;
        if (MasterRep == 0 || (MasterRep == 2 && nbSel1 > 1 && nbSel2 > 1))
        {
            int ii;
            for (ii = 1; ii <= nbSel1; ii++)
            {
                if (!(theTrimSel1.Value(ii).CartesianPoint() is null))
                {
                    for (ii = 1; ii <= nbSel2; ii++)
                    {
                        if (!(theTrimSel2.Value(ii).CartesianPoint() is null))
                        {
                            isPoint = true;
                            break;
                        }
                    }
                    break;
                }
            }
        }

        //: o6 abv 18 Feb 99: computation of factor moved
        double fact = 1.0, shift = 0.0;
        if (theSTEPCurve is StepGeom_Line)
        {
            StepGeom_Line theLine = theSTEPCurve as StepGeom_Line;
            fact = theLine.Dir().Magnitude() * theLocalFactors.LengthFactor();
        }
        else if (theSTEPCurve is StepGeom_Circle || theSTEPCurve is StepGeom_Ellipse)
        {
            //    if (trim1 > 2.1*M_PI || trim2 > 2.1*M_PI) fact = M_PI / 180.;
            fact = theLocalFactors.PlaneAngleFactor();
            //: p3 abv 23 Feb 99: shift on pi/2 on ellipse with R1 < R2
            StepGeom_Ellipse ellipse = theSTEPCurve as StepGeom_Ellipse;
            if (ellipse is not null && ellipse.SemiAxis1() - ellipse.SemiAxis2() < 0.0)
            {
                shift = 0.5 * M_PI;
            }

            // skl 04.02.2002 for OCC133: we can not make TrimmedCurve if
            // there is no X-direction in StepGeom_Axis2Placement3d
            StepGeom_Conic conic = theSTEPCurve as StepGeom_Conic;
            // CKY 6-FEB-2004 for Airbus-MedialAxis :
            // this restriction does not apply for trimming by POINTS
            if (conic is not null && MasterRep != 1)
            {
                StepGeom_Axis2Placement a2p = conic.Position();
                if (a2p.CaseNum(a2p.Value()) == 2)
                {
                    if (!a2p.Axis2Placement3d().HasRefDirection())
                    {
                        ////gka 18.02.04 analysis for case when MasterRep = .Unspecified
                        // and parameters are specified as CARTESIAN_POINT
                        if (isPoint /*&& !MasterRep*/)
                        {
                            MasterRep = 1;
                        }
                        else
                        {
                            if (SC.SenseAgreement())
                            {
                                return new TrimmedCurve3D(theCurve, 0.0, 2.0 * M_PI, true);
                            }
                            else
                            {
                                return new TrimmedCurve3D(theCurve, 2.0 * M_PI, 0.0, false);
                            }
                        }
                    }
                }
            }
        }

        double trim1 = 0.0;
        double trim2 = 0.0;
        bool FoundParam1 = ExtractParameter(theCurve, theTrimSel1, nbSel1, MasterRep, fact, shift, ref trim1, theLocalFactors);
        bool FoundParam2 = ExtractParameter(theCurve, theTrimSel2, nbSel2, MasterRep, fact, shift, ref trim2, theLocalFactors);

        if (FoundParam1 && FoundParam2)
        {
            double cf = theCurve.FirstParameter();
            double cl = theCurve.LastParameter();
            //: abv 09.04.99: S4136: bm2_ug_t4-B.stp #70610: protect against OutOfRange
            if (!theCurve.IsPeriodic)
            {
                if (trim1 < cf)
                {
                    trim1 = cf;
                }
                else if (trim1 > cl)
                {
                    trim1 = cl;
                }
                if (trim2 < cf)
                {
                    trim2 = cf;
                }
                else if (trim2 > cl)
                {
                    trim2 = cl;
                }
            }
            if (Math.Abs(trim1 - trim2) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (theCurve.IsPeriodic)
                {
                    global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref trim1, ref trim2);
                }
                else if (theCurve.IsClosed)
                {
                    if (Math.Abs(trim1 - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        trim2 += cl;
                    }
                    else
                    {
                        trim1 -= cl;
                    }
                }
                else
                {
                    return null;
                }
            }
            //  CKY 16-DEC-1997 : USA60035 le texte de Part42 parle de degres
            //    mais des systemes ecrivent en radians. Exploiter UnitsMethods
            //: o6    trim1 = trim1 * fact;
            //: o6    trim2 = trim2 * fact;
            if (SC.SenseAgreement())
            {
                return new TrimmedCurve3D(theCurve, trim1, trim2, true);
            }
            else
            { //: abv 29.09.00 PRO20362: reverse parameters in case of reversed curve
                return new TrimmedCurve3D(theCurve, trim2, trim1, false);
            }
        }
        return null;
    }

    /// <summary>Converts a trimmed_curve (default factors).</summary>
    public static TrimmedCurve3D? MakeTrimmedCurve(StepGeom_TrimmedCurve SC) => MakeTrimmedCurve(SC, new StepData_Factors());

    //=============================================================================
    // Creation d'une Trimmed Curve de Geom2d a partir d' une Trimmed Curve de Step
    //=============================================================================
    // Shall be completed to treat trimming with points

    /// <summary>
    /// Converts a trimmed_curve into a 2D B-spline curve (trims by parameter only; a B-spline basis curve is
    /// returned untrimmed).
    /// </summary>
    public static BSplineCurve2D? MakeTrimmedCurve2d(StepGeom_TrimmedCurve SC, in StepData_Factors theLocalFactors)
    {
        StepGeom_Curve BasisCurve = SC.BasisCurve();
        Curve2D? theGeomBasis = MakeCurve2d(BasisCurve, theLocalFactors);
        if (theGeomBasis is null)
        {
            return null;
        }

        if (theGeomBasis is BSplineCurve2D)
        {
            return theGeomBasis as BSplineCurve2D;
        }

        NCollection_HArray1<StepGeom_TrimmingSelect> theTrimSel1 = SC.Trim1();
        NCollection_HArray1<StepGeom_TrimmingSelect> theTrimSel2 = SC.Trim2();
        int nbSel1 = SC.NbTrim1();
        int nbSel2 = SC.NbTrim2();
        if ((nbSel1 == 1) && (nbSel2 == 1) && (theTrimSel1.Value(1).CaseMember() > 0) && (theTrimSel2.Value(1).CaseMember() > 0))
        {
            double u1 = theTrimSel1.Value(1).ParameterValue();
            double u2 = theTrimSel2.Value(1).ParameterValue();
            double fact = 1.0, shift = 0.0;

            if (BasisCurve is StepGeom_Line)
            {
                StepGeom_Line theLine = BasisCurve as StepGeom_Line;
                fact = theLine.Dir().Magnitude();
            }
            else if (BasisCurve is StepGeom_Circle || BasisCurve is StepGeom_Ellipse)
            {
                //      if (u1 > 2.1*M_PI || u2 > 2.1*M_PI) fact = M_PI / 180.;
                fact = theLocalFactors.PlaneAngleFactor();
                //: p3 abv 23 Feb 99: shift on pi/2 on ellipse with R1 < R2
                StepGeom_Ellipse ellipse = BasisCurve as StepGeom_Ellipse;
                if (ellipse is not null && ellipse.SemiAxis1() - ellipse.SemiAxis2() < 0.0)
                {
                    shift = 0.5 * M_PI;
                }
            }
            else if (BasisCurve is StepGeom_Parabola || BasisCurve is StepGeom_Hyperbola)
            {
                // LATER !!!
            }
            //    CKY 16-DEC-1997 : USA60035 le texte de Part42 parle de degres
            //      mais des systemes ecrivent en radians. Exploiter UnitsMethods

            double newU1 = shift + u1 * fact;
            double newU2 = shift + u2 * fact;

            TrimmedCurve2D theTrimmed = new TrimmedCurve2D(theGeomBasis, newU1, newU2, SC.SenseAgreement());
            return Geom2dConvert2D.CurveToBSplineCurve(theTrimmed);
        }
        return null;
    }

    /// <summary>Converts a trimmed_curve into a 2D B-spline curve (default factors).</summary>
    public static BSplineCurve2D? MakeTrimmedCurve2d(StepGeom_TrimmedCurve SC) => MakeTrimmedCurve2d(SC, new StepData_Factors());

    //=============================================================================
    // Creation d' un VectorWithMagnitude de Geom a partir d' un Vector de Step
    //=============================================================================

    /// <summary>Converts a vector (its magnitude a length).</summary>
    public static VectorWithMagnitude? MakeVectorWithMagnitude(StepGeom_Vector SV, in StepData_Factors theLocalFactors)
    {
        // sln 22.10.2001. CTS23496: Vector is not created if direction have not been successfully created
        Direction? D = MakeDirection(SV.Orientation());
        if (D is not null)
        {
            Vector3D V = new Vector3D(D.Dir().XYZ() * SV.Magnitude() * theLocalFactors.LengthFactor());
            return new VectorWithMagnitude(V);
        }
        return null;
    }

    /// <summary>Converts a vector (default factors).</summary>
    public static VectorWithMagnitude? MakeVectorWithMagnitude(StepGeom_Vector SV) => MakeVectorWithMagnitude(SV, new StepData_Factors());

    //=============================================================================
    // Creation d' un VectorWithMagnitude de Geom2d a partir d' un Vector de Step
    //=============================================================================

    /// <summary>Converts a vector into a 2D vector.</summary>
    public static VectorWithMagnitude2? MakeVectorWithMagnitude2d(StepGeom_Vector SV)
    {
        // sln 23.10.2001. CTS23496: Vector is not created if direction have not been successfully created
        // (MakeVectorWithMagnitude2d(...) function)
        Direction2? D = MakeDirection2d(SV.Orientation());
        if (D is not null)
        {
            Vector2D V = new Vector2D(D.Dir2d().XY() * SV.Magnitude());
            return new VectorWithMagnitude2(V);
        }
        return null;
    }

    /// <summary>
    /// Converts a spatial rotation (a ypr_rotation, or a rotation_about_direction in the plane angle unit of the
    /// context) into yaw, pitch and roll angles; null when it can't.
    /// </summary>
    /// <remarks>OCCT.NET: ported by unit 8F, with STEPConstruct_UnitContext (decision 1238).</remarks>
    public static NCollection_HArray1<double>? MakeYprRotation(StepKinematics_SpatialRotation SR, StepRepr_GlobalUnitAssignedContext? theCntxt)
    {
        // If rotation is already a ypr_rotation, return it immediately
        NCollection_HArray1<double>? anYPRRotation = null;
        if (SR.YprRotation() is not null && SR.YprRotation().Length() == 3)
        {
            return SR.YprRotation();
        }

        if (SR.RotationAboutDirection() is null
            || SR.RotationAboutDirection().DirectionOfAxis().NbDirectionRatios() != 3
            || theCntxt is null)
        {
            return null;
        }
        // rotation is a rotation_about_direction
        Direction anAxis = new Direction(SR.RotationAboutDirection().DirectionOfAxis().DirectionRatiosValue(1),
                                                   SR.RotationAboutDirection().DirectionOfAxis().DirectionRatiosValue(2),
                                                   SR.RotationAboutDirection().DirectionOfAxis().DirectionRatiosValue(3));
        double anAngle = SR.RotationAboutDirection().RotationAngle();
        if (Math.Abs(anAngle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            // a zero rotation is converted trivially
            anYPRRotation = new NCollection_HArray1<double>(1, 3);
            anYPRRotation.SetValue(1, 0.0);
            anYPRRotation.SetValue(2, 0.0);
            anYPRRotation.SetValue(3, 0.0);
            return anYPRRotation;
        }
        double dx = anAxis.X();
        double dy = anAxis.Y();
        double dz = anAxis.Z();
        NCollection_Sequence<StepBasic_NamedUnit> aPaUnits = new NCollection_Sequence<StepBasic_NamedUnit>();
        for (int anInd = 1; anInd <= theCntxt.Units().Length(); ++anInd)
        {
            if (theCntxt.UnitsValue(anInd).IsKind(typeof(StepBasic_ConversionBasedUnitAndPlaneAngleUnit))
                || theCntxt.UnitsValue(anInd).IsKind(typeof(StepBasic_SiUnitAndPlaneAngleUnit)))
            {
                aPaUnits.Append(theCntxt.UnitsValue(anInd));
            }
        }
        if (aPaUnits.Length() != 1)
        {
            return anYPRRotation;
        }
        StepBasic_NamedUnit? aPau = aPaUnits.Value(1);
        while (aPau is not null && aPau.IsKind(typeof(StepBasic_ConversionBasedUnitAndPlaneAngleUnit)))
        {
            StepBasic_ConversionBasedUnitAndPlaneAngleUnit aConverUnit = (StepBasic_ConversionBasedUnitAndPlaneAngleUnit)aPau;
            StepBasic_MeasureWithUnit? aConversionFactor = null;
            Transient aTransientFactor = aConverUnit.ConversionFactor();
            if (aTransientFactor.IsKind(typeof(StepBasic_MeasureWithUnit)))
            {
                aConversionFactor = aTransientFactor as StepBasic_MeasureWithUnit;
            }
            else if (aTransientFactor.IsKind(typeof(StepRepr_ReprItemAndMeasureWithUnit)))
            {
                StepRepr_ReprItemAndMeasureWithUnit aReprMeasureItem = (StepRepr_ReprItemAndMeasureWithUnit)aTransientFactor;
                aConversionFactor = aReprMeasureItem.GetMeasureWithUnit();
            }
            anAngle = anAngle * aConversionFactor!.ValueComponent();
            aPau = aConversionFactor.UnitComponent().NamedUnit();
        }
        if (aPau is null)
        {
            return anYPRRotation;
        }
        StepBasic_SiUnitAndPlaneAngleUnit? aSiUnit = aPau as StepBasic_SiUnitAndPlaneAngleUnit;
        if (aSiUnit is null || aSiUnit.Name() != StepBasic_SiUnitName.StepBasic_sunRadian)
        {
            return anYPRRotation;
        }
        anAngle = (!aSiUnit.HasPrefix() ? 1.0 : STEPConstruct_UnitContext.ConvertSiPrefix(aSiUnit.Prefix())) * anAngle;
        double anUcf = SR.RotationAboutDirection().RotationAngle() / anAngle;
        double aSA = Math.Sin(anAngle);
        double aCA = Math.Cos(anAngle);
        double aYaw = 0, aPitch = 0, aRoll = 0;

        // axis parallel either to x-axis or to z-axis?
        if (Math.Abs(dy) < global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(dx * dz) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            while (anAngle <= -Math.PI)
            {
                anAngle = anAngle + 2 * Math.PI;
            }
            while (anAngle > Math.PI)
            {
                anAngle = anAngle - 2 * Math.PI;
            }

            aYaw = anUcf * anAngle;
            if (Math.Abs(anAngle - Math.PI) >= global::Kerf.Engine.Foundation.Precision.Angular())
            {
                aRoll = -aYaw;
            }
            else
            {
                aRoll = aYaw;
            }
            anYPRRotation = new NCollection_HArray1<double>(1, 3);
            anYPRRotation.SetValue(1, 0.0);
            anYPRRotation.SetValue(2, 0.0);
            anYPRRotation.SetValue(3, 0.0);
            if (Math.Abs(dx) >= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (dx > 0.0)
                {
                    anYPRRotation.SetValue(3, aYaw);
                }
                else
                {
                    anYPRRotation.SetValue(3, aRoll);
                }
            }
            else
            {
                if (dz > 0.0)
                {
                    anYPRRotation.SetValue(1, aYaw);
                }
                else
                {
                    anYPRRotation.SetValue(1, aRoll);
                }
            }
            return anYPRRotation;
        }

        // axis parallel to y-axis - use y-axis as pitch axis
        if (Math.Abs(dy) >= global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(dx) < global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(dz) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            if (aCA >= 0.0)
            {
                aYaw = 0.0;
                aRoll = 0.0;
            }
            else
            {
                aYaw = anUcf * Math.PI;
                aRoll = aYaw;
            }
            aPitch = anUcf * Math.Atan2(aSA, Math.Abs(aCA));
            if (dy < 0.0)
            {
                aPitch = -aPitch;
            }
            anYPRRotation = new NCollection_HArray1<double>(1, 3);
            anYPRRotation.SetValue(1, aYaw);
            anYPRRotation.SetValue(2, aPitch);
            anYPRRotation.SetValue(3, aRoll);
            return anYPRRotation;
        }
        // axis not parallel to any axis of coordinate system
        // compute rotation matrix
        double aCm1 = 1 - aCA;

        double[,] aRotMat = {
            { dx * dx * aCm1 + aCA, dx * dy * aCm1 - dz * aSA, dx * dz * aCm1 + dy * aSA },
            { dx * dy * aCm1 + dz * aSA, dy * dy * aCm1 + aCA, dy * dz * aCm1 - dx * aSA },
            { dx * dz * aCm1 - dy * aSA, dy * dz * aCm1 + dx * aSA, dz * dz * aCm1 + aCA } };

        // aRotMat[1][3] equals SIN(pitch_angle)
        if (Math.Abs(Math.Abs(aRotMat[0, 2] - 1.0)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // |aPitch| = PI/2
            if (Math.Abs(aRotMat[0, 2] - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aPitch = Math.PI / 2;
            }
            else
            {
                aPitch = -Math.PI / 2;
            }
            // In this case, only the sum or difference of roll and yaw angles
            // is relevant and can be evaluated from the matrix.
            // According to IP "rectangular pitch angle" for ypr_rotation,
            // the roll angle is set to zero.
            aRoll = 0.0;
            aYaw = Math.Atan2(aRotMat[1, 0], aRotMat[1, 1]);
            // result of ATAN is in the range[-PI / 2, PI / 2].
            // Here all four quadrants are needed.

            if (aRotMat[1, 1] < 0.0)
            {
                if (aYaw <= 0.0)
                {
                    aYaw = aYaw + Math.PI;
                }
                else
                {
                    aYaw = aYaw - Math.PI;
                }
            }
        }
        else
        {
            // COS (pitch_angle) not equal to zero
            aYaw = Math.Atan2(-aRotMat[0, 1], aRotMat[0, 0]);

            if (aRotMat[0, 0] < 0.0)
            {
                if (aYaw < 0.0 || Math.Abs(aYaw) < global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    aYaw = aYaw + Math.PI;
                }
                else
                {
                    aYaw = aYaw - Math.PI;
                }
            }
            double aSY = Math.Sin(aYaw);
            double aCY = Math.Cos(aYaw);
            double aSR = Math.Sin(aRoll);
            double aCR = Math.Cos(aRoll);

            if (Math.Abs(aSY) > Math.Abs(aCY) && Math.Abs(aSY) > Math.Abs(aSR) && Math.Abs(aSY) > Math.Abs(aCR))
            {
                aCm1 = -aRotMat[0, 1] / aSY;
            }
            else
            {
                if (Math.Abs(aCY) > Math.Abs(aSR) && Math.Abs(aCY) > Math.Abs(aCR))
                {
                    aCm1 = aRotMat[0, 0] / aCY;
                }
                else if (Math.Abs(aSR) > Math.Abs(aCR))
                {
                    aCm1 = -aRotMat[1, 2] / aSR;
                }
                else
                {
                    aCm1 = aRotMat[2, 2] / aCR;
                }
            }
            aPitch = Math.Atan2(aRotMat[0, 2], aCm1);
        }
        aYaw = aYaw * anUcf;
        aPitch = aPitch * anUcf;
        aRoll = aRoll * anUcf;
        anYPRRotation = new NCollection_HArray1<double>(1, 3);
        anYPRRotation.SetValue(1, aYaw);
        anYPRRotation.SetValue(2, aPitch);
        anYPRRotation.SetValue(3, aRoll);

        return anYPRRotation;
    }
}
