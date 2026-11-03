// Port of: src/DataExchange/TKDESTEP/StepToGeom/StepToGeom.hxx
// Port of: src/DataExchange/TKDESTEP/StepToGeom/StepToGeom.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Kinematics;
using Kerf.Engine.Exchange.Step.Representation;
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
using Occt;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Exchange.Step.Data.Logical;
using static Kerf.Engine.Exchange.Step.Geometry.StepKnotType;
using static Kerf.Engine.Exchange.Step.Geometry.StepTrimmingPreference;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Import.Geometry;

/// <summary>
/// The conversion of STEP's geometric entities (StepGeom) into Geom and Geom2d: points, directions, vectors,
/// placements, curves, surfaces and transformations. Each method returns null when the entity can't be converted.
/// </summary>
/// <remarks>
/// The units of the file are applied through <see cref="Factors"/> (lengths multiplied by
/// <see cref="Factors.LengthFactor"/>, angles by <see cref="Factors.PlaneAngleFactor"/>); the
/// overloads without factors use the defaults (millimetres and radians). The results are new, unfrozen geometry.
/// <c>MakeYprRotation</c> was added by unit 8F (decision 1238).
/// </remarks>
public static class StepToGeom
{
    //=============================================================================
    // Creation d' un Ax1Placement de Geom a partir d' un axis1_placement de Step
    //=============================================================================

    /// <summary>Converts an axis1_placement.</summary>
    public static Axis1Placement? MakeAxis1Placement(StepAxis1Placement sa, in Factors localFactors)
    {
        CartesianPoint? p = MakeCartesianPoint(sa.Location(), localFactors);
        if (p is not null)
        {
            // sln 22.10.2001. CTS23496: If problems with creation of axis direction occur default direction
            // is used
            Direction3D d = new Direction3D(Direction3D.D.Z);
            if (sa.HasAxis)
            {
                Direction? d1 = MakeDirection(sa.Axis());
                if (d1 is not null)
                {
                    d = d1.Dir();
                }
            }
            return new Axis1Placement(p.Pnt(), d);
        }
        return null;
    }

    /// <summary>Converts an axis1_placement (default factors).</summary>
    public static Axis1Placement? MakeAxis1Placement(StepAxis1Placement sa) => MakeAxis1Placement(sa, new Factors());

    //=============================================================================
    // Creation d' un Axis2Placement de Geom a partir d' un axis2_placement_3d de Step
    //=============================================================================

    /// <summary>Converts an axis2_placement_3d.</summary>
    public static Axis2Placement? MakeAxis2Placement(StepAxis2Placement3d sa, in Factors localFactors)
    {
        CartesianPoint? p = MakeCartesianPoint(sa.Location(), localFactors);
        if (p is not null)
        {
            Point3D pgp = p.Pnt();

            // sln 22.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used (MakeLine(...) function)
            Direction3D ngp = new Direction3D(Direction3D.D.Z);
            if (sa.HasAxis)
            {
                Direction? d = MakeDirection(sa.Axis());
                if (d is not null)
                {
                    ngp = d.Dir();
                }
            }

            Frame3D gpAx2 = new Frame3D();
            bool isDefaultDirectionUsed = true;
            if (sa.HasRefDirection)
            {
                Direction? d = MakeDirection(sa.RefDirection());
                if (d is not null)
                {
                    Direction3D vxgp = d.Dir();
                    if (!ngp.IsParallel(vxgp, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        gpAx2 = new Frame3D(pgp, ngp, vxgp);
                        isDefaultDirectionUsed = false;
                    }
                }
            }
            if (isDefaultDirectionUsed)
            {
                gpAx2 = new Frame3D(pgp, ngp);
            }

            return new Axis2Placement(gpAx2);
        }
        return null;
    }

    /// <summary>Converts an axis2_placement_3d (default factors).</summary>
    public static Axis2Placement? MakeAxis2Placement(StepAxis2Placement3d sa) => MakeAxis2Placement(sa, new Factors());

    //=============================================================================
    // Creation of an AxisPlacement from a Kinematic SuParameters for Step
    //=============================================================================

    /// <summary>Converts the kinematic su_parameters into a placement.</summary>
    public static Axis2Placement MakeAxis2Placement(StepSuParameters sp)
    {
        double locX = sp.A() * Math.Cos(sp.Gamma()) + sp.B() * Math.Sin(sp.Gamma()) * Math.Sin(sp.Alpha());
        double locY = sp.A() * Math.Sin(sp.Gamma()) - sp.B() * Math.Cos(sp.Gamma()) * Math.Sin(sp.Alpha());
        double locZ = sp.C() + sp.B() * Math.Cos(sp.Alpha());
        double asisX = Math.Sin(sp.Gamma()) * Math.Sin(sp.Alpha());
        double axisY = -Math.Cos(sp.Gamma()) * Math.Sin(sp.Alpha());
        double axisZ = Math.Cos(sp.Alpha());
        double dirX = Math.Cos(sp.Gamma()) * Math.Cos(sp.Beta())
                       - Math.Sin(sp.Gamma()) * Math.Cos(sp.Alpha()) * Math.Sin(sp.Beta());
        double dirY = Math.Sin(sp.Gamma()) * Math.Cos(sp.Beta())
                       + Math.Cos(sp.Gamma()) * Math.Cos(sp.Alpha()) * Math.Sin(sp.Beta());
        double dirZ = Math.Sin(sp.Alpha()) * Math.Sin(sp.Beta());
        Point3D pgp = new Point3D(locX, locY, locZ);
        Direction3D ngp = new Direction3D(asisX, axisY, axisZ);
        Direction3D vxgp = new Direction3D(dirX, dirY, dirZ);
        Frame3D gpAx2 = new Frame3D(pgp, ngp, vxgp);
        return new Axis2Placement(gpAx2);
    }

    //=============================================================================
    // Creation d' un AxisPlacement de Geom2d a partir d' un axis2_placement_3d de Step
    //=============================================================================

    /// <summary>Converts an axis2_placement_2d.</summary>
    public static AxisPlacement2? MakeAxisPlacement(StepAxis2Placement2d sa, in Factors localFactors)
    {
        CartesianPoint2? p = MakeCartesianPoint2d(sa.Location(), localFactors);
        if (p is not null)
        {
            // sln 23.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used
            Direction2D vxgp = new Direction2D(Direction2D.D.X);
            if (sa.HasRefDirection)
            {
                Direction2? vx = MakeDirection2d(sa.RefDirection());
                if (vx is not null)
                {
                    vxgp = vx.Dir2d();
                }
            }

            return new AxisPlacement2(p.Pnt2d(), vxgp);
        }
        return null;
    }

    /// <summary>Converts an axis2_placement_2d (default factors).</summary>
    public static AxisPlacement2? MakeAxisPlacement(StepAxis2Placement2d sa) => MakeAxisPlacement(sa, new Factors());

    //=============================================================================
    // Creation d' une BoundedCurve de Geom a partir d' une BoundedCurve de Step
    //=============================================================================

    /// <summary>Converts a bounded_curve (B-splines, Bezier, uniform and quasi-uniform curves, trimmed curves, polylines).</summary>
    public static BoundedCurve3D? MakeBoundedCurve(StepBoundedCurve sc, in Factors localFactors)
    {
        if (sc is StepBSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            return MakeBSplineCurve(sc as StepBSplineCurveWithKnotsAndRationalBSplineCurve, localFactors);
        }
        if (sc is StepBSplineCurveWithKnots)
        {
            return MakeBSplineCurve(sc as StepBSplineCurveWithKnots, localFactors);
        }
        if (sc is StepTrimmedCurve)
        {
            return MakeTrimmedCurve(sc as StepTrimmedCurve, localFactors);
        }

        // STEP BezierCurve, UniformCurve and QuasiUniformCurve are transformed into
        // STEP BSplineCurve before being mapped onto CAS.CADE/SF
        if (sc is StepBezierCurve)
        {
            StepBezierCurve bzC = sc as StepBezierCurve;
            int degree = bzC.Degree();
            if (degree < 1 || degree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepBSplineCurveWithKnots bspl = new StepBSplineCurveWithKnots();
            bspl.SetDegree(degree);
            bspl.SetControlPointsList(bzC.ControlPointsList());
            bspl.SetCurveForm(bzC.CurveForm());
            bspl.SetClosedCurve(bzC.ClosedCurve());
            bspl.SetSelfIntersect(bzC.SelfIntersect());
            // Compute Knots and KnotsMultiplicity
            NCollection_HArray1<int> kmult = new NCollection_HArray1<int>(1, 2);
            NCollection_HArray1<double> knots = new NCollection_HArray1<double>(1, 2);
            kmult.SetValue(1, bzC.Degree() + 1);
            kmult.SetValue(2, bzC.Degree() + 1);
            knots.SetValue(1, 0.0);
            knots.SetValue(2, 1.0);
            bspl.SetKnotMultiplicities(kmult);
            bspl.SetKnots(knots);

            return MakeBSplineCurve(bspl, localFactors);
        }

        if (sc is StepUniformCurve)
        {
            StepUniformCurve uc = sc as StepUniformCurve;
            int degree = uc.Degree();
            if (degree < 1 || degree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepBSplineCurveWithKnots bspl = new StepBSplineCurveWithKnots();
            bspl.SetDegree(degree);
            bspl.SetControlPointsList(uc.ControlPointsList());
            bspl.SetCurveForm(uc.CurveForm());
            bspl.SetClosedCurve(uc.ClosedCurve());
            bspl.SetSelfIntersect(uc.SelfIntersect());

            // Compute Knots and KnotsMultiplicity
            int nbK = bspl.NbControlPointsList() + bspl.Degree() + 1;
            NCollection_HArray1<int> kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> knots = new NCollection_HArray1<double>(1, nbK);
            for (int iUC = 1; iUC <= nbK; iUC++)
            {
                kmult.SetValue(iUC, 1);
                knots.SetValue(iUC, iUC - 1.0);
            }
            bspl.SetKnotMultiplicities(kmult);
            bspl.SetKnots(knots);

            return MakeBSplineCurve(bspl, localFactors);
        }

        if (sc is StepQuasiUniformCurve)
        {
            StepQuasiUniformCurve quc = sc as StepQuasiUniformCurve;
            int degree = quc.Degree();
            if (degree < 1 || degree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepBSplineCurveWithKnots bspl = new StepBSplineCurveWithKnots();
            bspl.SetDegree(degree);
            bspl.SetControlPointsList(quc.ControlPointsList());
            bspl.SetCurveForm(quc.CurveForm());
            bspl.SetClosedCurve(quc.ClosedCurve());
            bspl.SetSelfIntersect(quc.SelfIntersect());

            // Compute Knots and KnotsMultiplicity
            int nbK = bspl.NbControlPointsList() - bspl.Degree() + 1;
            NCollection_HArray1<int> kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> knots = new NCollection_HArray1<double>(1, nbK);
            for (int iQUC = 1; iQUC <= nbK; iQUC++)
            {
                kmult.SetValue(iQUC, 1);
                knots.SetValue(iQUC, iQUC - 1.0);
            }
            kmult.SetValue(1, bspl.Degree() + 1);
            kmult.SetValue(nbK, bspl.Degree() + 1);
            bspl.SetKnotMultiplicities(kmult);
            bspl.SetKnots(knots);

            return MakeBSplineCurve(bspl, localFactors);
        }

        if (sc is StepUniformCurveAndRationalBSplineCurve)
        {
            StepUniformCurveAndRationalBSplineCurve ruc = sc as StepUniformCurveAndRationalBSplineCurve;
            int degree = ruc.Degree();
            if (degree < 1 || degree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepBSplineCurveWithKnotsAndRationalBSplineCurve rbspl = new StepBSplineCurveWithKnotsAndRationalBSplineCurve();

            // Compute Knots and KnotsMultiplicity
            int nbK = ruc.NbControlPointsList() + degree + 1;
            NCollection_HArray1<int> kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> knots = new NCollection_HArray1<double>(1, nbK);
            for (int iUC = 1; iUC <= nbK; iUC++)
            {
                kmult.SetValue(iUC, 1);
                knots.SetValue(iUC, iUC - 1.0);
            }

            // Initialize the BSplineCurveWithKnotsAndRationalBSplineCurve
            rbspl.Init(ruc.Name(),
                       degree,
                       ruc.ControlPointsList(),
                       ruc.CurveForm(),
                       ruc.ClosedCurve(),
                       ruc.SelfIntersect(),
                       kmult,
                       knots,
                       StepKnotType.KtUnspecified,
                       ruc.WeightsData());

            return MakeBSplineCurve(rbspl, localFactors);
        }

        if (sc is StepQuasiUniformCurveAndRationalBSplineCurve)
        {
            StepQuasiUniformCurveAndRationalBSplineCurve rquc = sc as StepQuasiUniformCurveAndRationalBSplineCurve;
            int degree = rquc.Degree();
            if (degree < 1 || degree > BSplineCurve3D.MaxDegree())
            {
                return null;
            }
            StepBSplineCurveWithKnotsAndRationalBSplineCurve rbspl = new StepBSplineCurveWithKnotsAndRationalBSplineCurve();

            // Compute Knots and KnotsMultiplicity
            int nbK = rquc.NbControlPointsList() - degree + 1;
            NCollection_HArray1<int> kmult = new NCollection_HArray1<int>(1, nbK);
            NCollection_HArray1<double> knots = new NCollection_HArray1<double>(1, nbK);
            for (int iRQUC = 1; iRQUC <= nbK; iRQUC++)
            {
                kmult.SetValue(iRQUC, 1);
                knots.SetValue(iRQUC, iRQUC - 1.0);
            }
            kmult.SetValue(1, degree + 1);
            kmult.SetValue(nbK, degree + 1);
            // Initialize the BSplineCurveWithKnotsAndRationalBSplineCurve
            rbspl.Init(rquc.Name(),
                       degree,
                       rquc.ControlPointsList(),
                       rquc.CurveForm(),
                       rquc.ClosedCurve(),
                       rquc.SelfIntersect(),
                       kmult,
                       knots,
                       StepKnotType.KtUnspecified,
                       rquc.WeightsData());

            return MakeBSplineCurve(rbspl, localFactors);
        }

        if (sc is StepPolyline)
        { //: n6 abv 15 Feb 99
            return MakePolyline(sc as StepPolyline, localFactors);
        }

        return null;
    }

    /// <summary>Converts a bounded_curve (default factors).</summary>
    public static BoundedCurve3D? MakeBoundedCurve(StepBoundedCurve sc) => MakeBoundedCurve(sc, new Factors());

    //=============================================================================
    // Creation d' une BoundedCurve de Geom a partir d' une BoundedCurve de Step
    //=============================================================================

    /// <summary>Converts a bounded_curve into a 2D curve.</summary>
    public static BoundedCurve2D? MakeBoundedCurve2d(StepBoundedCurve sc, in Factors localFactors)
    {
        if (sc is StepBSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            return MakeBSplineCurve2d(sc as StepBSplineCurveWithKnotsAndRationalBSplineCurve, localFactors);
        }
        if (sc is StepBSplineCurveWithKnots)
        {
            return MakeBSplineCurve2d(sc as StepBSplineCurveWithKnots, localFactors);
        }
        if (sc is StepTrimmedCurve)
        {
            return MakeTrimmedCurve2d(sc as StepTrimmedCurve, localFactors);
        }
        if (sc is StepPolyline)
        { //: n6 abv 15 Feb 99
            return MakePolyline2d(sc as StepPolyline, localFactors);
        }
        return null;
    }

    /// <summary>Converts a bounded_curve into a 2D curve (default factors).</summary>
    public static BoundedCurve2D? MakeBoundedCurve2d(StepBoundedCurve sc) => MakeBoundedCurve2d(sc, new Factors());

    //=============================================================================
    // Creation d' une BoundedSurface de Geom a partir d' une BoundedSurface de Step
    //=============================================================================

    /// <summary>Converts a bounded_surface (B-splines, Bezier, uniform and quasi-uniform surfaces, rectangular trimmed surfaces).</summary>
    public static BoundedSurface? MakeBoundedSurface(StepBoundedSurface ss, in Factors localFactors)
    {
        if (ss is StepBSplineSurfaceWithKnotsAndRationalBSplineSurface)
        {
            return MakeBSplineSurface(ss as StepBSplineSurfaceWithKnotsAndRationalBSplineSurface, localFactors);
        }
        if (ss is StepBSplineSurfaceWithKnots)
        {
            return MakeBSplineSurface(ss as StepBSplineSurfaceWithKnots, localFactors);
        }
        if (ss is StepRectangularTrimmedSurface)
        {
            return MakeRectangularTrimmedSurface(ss as StepRectangularTrimmedSurface, localFactors);
        }

        // STEP BezierSurface, UniformSurface and QuasiUniformSurface are transformed
        // into STEP BSplineSurface before being mapped onto CAS.CADE/SF
        if (ss is StepBezierSurface)
        {
            StepBezierSurface bzS = ss as StepBezierSurface;
            StepBSplineSurfaceWithKnots bspl = new StepBSplineSurfaceWithKnots();
            bspl.SetUDegree(bzS.UDegree());
            bspl.SetVDegree(bzS.VDegree());
            bspl.SetControlPointsList(bzS.ControlPointsList());
            bspl.SetSurfaceForm(bzS.SurfaceForm());
            bspl.SetUClosed(bzS.UClosed());
            bspl.SetVClosed(bzS.VClosed());
            bspl.SetSelfIntersect(bzS.SelfIntersect());

            // Compute Knots and KnotsMultiplicity
            NCollection_HArray1<int> uKmult = new NCollection_HArray1<int>(1, 2);
            NCollection_HArray1<int> vKmult = new NCollection_HArray1<int>(1, 2);
            NCollection_HArray1<double> uKnots = new NCollection_HArray1<double>(1, 2);
            NCollection_HArray1<double> vKnots = new NCollection_HArray1<double>(1, 2);
            uKmult.SetValue(1, bzS.UDegree() + 1);
            uKmult.SetValue(2, bzS.UDegree() + 1);
            vKmult.SetValue(1, bzS.VDegree() + 1);
            vKmult.SetValue(2, bzS.VDegree() + 1);
            uKnots.SetValue(1, 0.0);
            uKnots.SetValue(2, 1.0);
            vKnots.SetValue(1, 0.0);
            vKnots.SetValue(2, 1.0);
            bspl.SetUMultiplicities(uKmult);
            bspl.SetVMultiplicities(vKmult);
            bspl.SetUKnots(uKnots);
            bspl.SetVKnots(vKnots);

            return MakeBSplineSurface(bspl, localFactors);
        }

        if (ss is StepUniformSurface)
        {
            StepUniformSurface us = ss as StepUniformSurface;
            StepBSplineSurfaceWithKnots bspl = new StepBSplineSurfaceWithKnots();
            bspl.SetUDegree(us.UDegree());
            bspl.SetVDegree(us.VDegree());
            bspl.SetControlPointsList(us.ControlPointsList());
            bspl.SetSurfaceForm(us.SurfaceForm());
            bspl.SetUClosed(us.UClosed());
            bspl.SetVClosed(us.VClosed());
            bspl.SetSelfIntersect(us.SelfIntersect());

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = bspl.NbControlPointsListI() + bspl.UDegree() + 1;
            NCollection_HArray1<int> uKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> uKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                uKmult.SetValue(iU, 1);
                uKnots.SetValue(iU, iU - 1.0);
            }
            bspl.SetUMultiplicities(uKmult);
            bspl.SetUKnots(uKnots);

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = bspl.NbControlPointsListJ() + bspl.VDegree() + 1;
            NCollection_HArray1<int> vKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> vKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                vKmult.SetValue(iV, 1);
                vKnots.SetValue(iV, iV - 1.0);
            }
            bspl.SetVMultiplicities(vKmult);
            bspl.SetVKnots(vKnots);

            return MakeBSplineSurface(bspl, localFactors);
        }

        if (ss is StepQuasiUniformSurface)
        {
            StepQuasiUniformSurface qus = ss as StepQuasiUniformSurface;
            StepBSplineSurfaceWithKnots bspl = new StepBSplineSurfaceWithKnots();
            bspl.SetUDegree(qus.UDegree());
            bspl.SetVDegree(qus.VDegree());
            bspl.SetControlPointsList(qus.ControlPointsList());
            bspl.SetSurfaceForm(qus.SurfaceForm());
            bspl.SetUClosed(qus.UClosed());
            bspl.SetVClosed(qus.VClosed());
            bspl.SetSelfIntersect(qus.SelfIntersect());

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = bspl.NbControlPointsListI() - bspl.UDegree() + 1;
            NCollection_HArray1<int> uKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> uKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                uKmult.SetValue(iU, 1);
                uKnots.SetValue(iU, iU - 1.0);
            }
            uKmult.SetValue(1, bspl.UDegree() + 1);
            uKmult.SetValue(nbKU, bspl.UDegree() + 1);
            bspl.SetUMultiplicities(uKmult);
            bspl.SetUKnots(uKnots);

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = bspl.NbControlPointsListJ() - bspl.VDegree() + 1;
            NCollection_HArray1<int> vKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> vKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                vKmult.SetValue(iV, 1);
                vKnots.SetValue(iV, iV - 1.0);
            }
            vKmult.SetValue(1, bspl.VDegree() + 1);
            vKmult.SetValue(nbKV, bspl.VDegree() + 1);
            bspl.SetVMultiplicities(vKmult);
            bspl.SetVKnots(vKnots);

            return MakeBSplineSurface(bspl, localFactors);
        }

        if (ss is StepUniformSurfaceAndRationalBSplineSurface)
        {
            StepUniformSurfaceAndRationalBSplineSurface rus = ss as StepUniformSurfaceAndRationalBSplineSurface;
            StepBSplineSurfaceWithKnotsAndRationalBSplineSurface rbspl = new StepBSplineSurfaceWithKnotsAndRationalBSplineSurface();

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = rus.NbControlPointsListI() + rus.UDegree() + 1;
            NCollection_HArray1<int> uKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> uKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                uKmult.SetValue(iU, 1);
                uKnots.SetValue(iU, iU - 1.0);
            }

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = rus.NbControlPointsListJ() + rus.VDegree() + 1;
            NCollection_HArray1<int> vKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> vKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                vKmult.SetValue(iV, 1);
                vKnots.SetValue(iV, iV - 1.0);
            }

            // Initialize the BSplineSurfaceWithKnotsAndRationalBSplineSurface
            rbspl.Init(rus.Name(),
                       rus.UDegree(),
                       rus.VDegree(),
                       rus.ControlPointsList(),
                       rus.SurfaceForm(),
                       rus.UClosed(),
                       rus.VClosed(),
                       rus.SelfIntersect(),
                       uKmult,
                       vKmult,
                       uKnots,
                       vKnots,
                       StepKnotType.KtUnspecified,
                       rus.WeightsData());

            return MakeBSplineSurface(rbspl, localFactors);
        }

        if (ss is StepQuasiUniformSurfaceAndRationalBSplineSurface)
        {
            StepQuasiUniformSurfaceAndRationalBSplineSurface rqus = ss as StepQuasiUniformSurfaceAndRationalBSplineSurface;
            StepBSplineSurfaceWithKnotsAndRationalBSplineSurface rbspl = new StepBSplineSurfaceWithKnotsAndRationalBSplineSurface();

            // Compute Knots and KnotsMultiplicity for U Direction
            int nbKU = rqus.NbControlPointsListI() - rqus.UDegree() + 1;
            NCollection_HArray1<int> uKmult = new NCollection_HArray1<int>(1, nbKU);
            NCollection_HArray1<double> uKnots = new NCollection_HArray1<double>(1, nbKU);
            for (int iU = 1; iU <= nbKU; iU++)
            {
                uKmult.SetValue(iU, 1);
                uKnots.SetValue(iU, iU - 1.0);
            }
            uKmult.SetValue(1, rqus.UDegree() + 1);
            uKmult.SetValue(nbKU, rqus.UDegree() + 1);

            // Compute Knots and KnotsMultiplicity for V Direction
            int nbKV = rqus.NbControlPointsListJ() - rqus.VDegree() + 1;
            NCollection_HArray1<int> vKmult = new NCollection_HArray1<int>(1, nbKV);
            NCollection_HArray1<double> vKnots = new NCollection_HArray1<double>(1, nbKV);
            for (int iV = 1; iV <= nbKV; iV++)
            {
                vKmult.SetValue(iV, 1);
                vKnots.SetValue(iV, iV - 1.0);
            }
            vKmult.SetValue(1, rqus.VDegree() + 1);
            vKmult.SetValue(nbKV, rqus.VDegree() + 1);

            // Initialize the BSplineSurfaceWithKnotsAndRationalBSplineSurface
            rbspl.Init(rqus.Name(),
                       rqus.UDegree(),
                       rqus.VDegree(),
                       rqus.ControlPointsList(),
                       rqus.SurfaceForm(),
                       rqus.UClosed(),
                       rqus.VClosed(),
                       rqus.SelfIntersect(),
                       uKmult,
                       vKmult,
                       uKnots,
                       vKnots,
                       StepKnotType.KtUnspecified,
                       rqus.WeightsData());
            return MakeBSplineSurface(rbspl, localFactors);
        }

        return null;
    }

    /// <summary>Converts a bounded_surface (default factors).</summary>
    public static BoundedSurface? MakeBoundedSurface(StepBoundedSurface ss) => MakeBoundedSurface(ss, new Factors());

    //=============================================================================
    // Template function for use in MakeBSplineCurve / MakeBSplineCurve2d
    // (its two instantiations: OCCT's template over the point, curve and maker types)
    //=============================================================================

    // The knots and multiplicities of a B-spline curve with its repeated knots merged and its end multiplicities
    // capped at Degree + 1 (the template's common part); null when the curve can't be made.
    private static bool MakeBSplineCurveKnots(StepBSplineCurve stepGeom_BSplineCurve,
                                              out StepBSplineCurveWithKnots aBSplineCurveWithKnots,
                                              out StepBSplineCurveWithKnotsAndRationalBSplineCurve aBSplineCurveWithKnotsAndRationalBSplineCurve,
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

        if (stepGeom_BSplineCurve is StepBSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            aBSplineCurveWithKnotsAndRationalBSplineCurve = stepGeom_BSplineCurve as StepBSplineCurveWithKnotsAndRationalBSplineCurve;
            aBSplineCurveWithKnots = aBSplineCurveWithKnotsAndRationalBSplineCurve.BSplineCurveWithKnots();
        }
        else
        {
            aBSplineCurveWithKnots = stepGeom_BSplineCurve as StepBSplineCurveWithKnots;
        }

        int degree = aBSplineCurveWithKnots.Degree();
        int nbKnots = aBSplineCurveWithKnots.KnotMultiplicityCount;

        NCollection_HArray1<int> knotMultiplicities = aBSplineCurveWithKnots.KnotMultiplicities();
        NCollection_HArray1<double> knots = aBSplineCurveWithKnots.Knots();

        // Count number of unique knots
        int nbUniqueKnots = 0;
        double lastKnot = RealFirst();
        for (int i = 1; i <= nbKnots; ++i)
        {
            if (knots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                nbUniqueKnots++;
                lastKnot = knots.Value(i);
            }
        }
        if (nbUniqueKnots <= 1)
        {
            return false;
        }
        aUniqueKnots = new NCollection_Array1<double>(1, nbUniqueKnots);
        aUniqueKnotMultiplicities = new NCollection_Array1<int>(1, nbUniqueKnots);
        lastKnot = knots.Value(1);
        aUniqueKnots.SetValue(1, knots.Value(1));
        aUniqueKnotMultiplicities.SetValue(1, knotMultiplicities.Value(1));
        int knotPosition = 1;
        for (int i = 2; i <= nbKnots; i++)
        {
            if (knots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                knotPosition++;
                aUniqueKnots.SetValue(knotPosition, knots.Value(i));
                aUniqueKnotMultiplicities.SetValue(knotPosition, knotMultiplicities.Value(i));
                lastKnot = knots.Value(i);
            }
            else
            {
                // Knot not unique, increase multiplicity
                int currentMultiplicity = aUniqueKnotMultiplicities.Value(knotPosition);
                aUniqueKnotMultiplicities.SetValue(knotPosition, currentMultiplicity + knotMultiplicities.Value(i));
            }
        }

        for (int i = 1; i <= nbUniqueKnots; ++i)
        {
            int currentVal = aUniqueKnotMultiplicities.Value(i);
            if (currentVal > degree + 1)
            {
                if (i == 1)
                {
                    aFirstMuultypisityDifference = currentVal - degree - 1;
                }
                if (i == nbUniqueKnots)
                {
                    aLastMuultypisityDifference = currentVal - degree - 1;
                }
                // OCCT_DEBUG output left out
                currentVal = degree + 1;
            }
            aUniqueKnotMultiplicities.SetValue(i, currentVal);
        }
        return true;
    }

    // Does the curve descriptor LOOK like a periodic descriptor? (the template's common part)
    private static bool ShouldBePeriodic(NCollection_Array1<int> aUniqueKnotMultiplicities, int nbPoles, int aDegree)
    {
        int nbUniqueKnots = aUniqueKnotMultiplicities.Length();
        int summaryMuultypisity = 0;
        for (int i = 1; i <= nbUniqueKnots; i++)
        {
            summaryMuultypisity += aUniqueKnotMultiplicities.Value(i);
        }

        bool shouldBePeriodic;
        if (summaryMuultypisity == (nbPoles + aDegree + 1))
        {
            shouldBePeriodic = false;
        }
        else if ((aUniqueKnotMultiplicities.Value(1) == aUniqueKnotMultiplicities.Value(nbUniqueKnots))
                 && ((summaryMuultypisity - aUniqueKnotMultiplicities.Value(1)) == nbPoles))
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
    private static NCollection_Array1<double> UniqueWeights(StepBSplineCurveWithKnotsAndRationalBSplineCurve curve, int nbPoles,
                                                            int aFirstMuultypisityDifference, int aLastMuultypisityDifference)
    {
        NCollection_HArray1<double> weights = curve.WeightsData();
        NCollection_Array1<double> uniqueWeights =
            new NCollection_Array1<double>(1, nbPoles - aFirstMuultypisityDifference - aLastMuultypisityDifference);
        for (int i = 1 + aFirstMuultypisityDifference; i <= nbPoles - aLastMuultypisityDifference; ++i)
        {
            uniqueWeights.SetValue(i - aFirstMuultypisityDifference, weights.Value(i));
        }
        return uniqueWeights;
    }

    //=============================================================================
    // Creation d' une BSplineCurve de Geom a partir d' une BSplineCurve de Step
    //=============================================================================

    /// <summary>Converts a B-spline curve with knots (rational or not).</summary>
    public static BSplineCurve3D? MakeBSplineCurve(StepBSplineCurve stepGeom_BSplineCurve, in Factors localFactors)
    {
        if (!MakeBSplineCurveKnots(stepGeom_BSplineCurve, out StepBSplineCurveWithKnots bSplineCurveWithKnots,
                                   out StepBSplineCurveWithKnotsAndRationalBSplineCurve bSplineCurveWithKnotsAndRationalBSplineCurve,
                                   out NCollection_Array1<double> uniqueKnots, out NCollection_Array1<int> uniqueKnotMultiplicities,
                                   out int firstMuultypisityDifference, out int lastMuultypisityDifference))
        {
            return null;
        }

        int degree = bSplineCurveWithKnots.Degree();
        int nbPoles = bSplineCurveWithKnots.NbControlPointsList();
        NCollection_HArray1<StepCartesianPoint> controlPointsList = bSplineCurveWithKnots.ControlPointsList();
        int summaryMuultypisityDifference = firstMuultypisityDifference + lastMuultypisityDifference;
        int nbUniquePoles = nbPoles - summaryMuultypisityDifference;
        if (nbUniquePoles <= 0)
        {
            return null;
        }
        NCollection_Array1<Point3D> poles = new NCollection_Array1<Point3D>(1, nbPoles - summaryMuultypisityDifference);

        for (int i = 1 + firstMuultypisityDifference; i <= nbPoles - lastMuultypisityDifference; ++i)
        {
            CartesianPoint? point = MakeCartesianPoint(controlPointsList.Value(i), localFactors);
            if (point is not null)
            {
                Point3D gpPnt = point.Pnt();
                poles.SetValue(i - firstMuultypisityDifference, gpPnt);
            }
            else
            {
                return null;
            }
        }

        bool shouldBePeriodic = ShouldBePeriodic(uniqueKnotMultiplicities, nbPoles, degree);

        BSplineCurve3D bSplineCurve;
        if (stepGeom_BSplineCurve is StepBSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            NCollection_Array1<double> uniqueWeights =
                UniqueWeights(bSplineCurveWithKnotsAndRationalBSplineCurve, nbPoles, firstMuultypisityDifference, lastMuultypisityDifference);
            bSplineCurve = new BSplineCurve3D(poles, uniqueWeights, uniqueKnots, uniqueKnotMultiplicities, degree, shouldBePeriodic);
        }
        else
        {
            bSplineCurve = new BSplineCurve3D(poles, uniqueKnots, uniqueKnotMultiplicities, degree, shouldBePeriodic);
        }

        // abv 04.07.00 CAX-IF TRJ4: trj4_k1_top-md-203.stp #716 (face #581):
        // force periodicity on closed curves
        // OCCT: the logical is tested as a C++ bool, so UNKNOWN counts as closed (occt-issues.md, 8D)
        if (stepGeom_BSplineCurve.ClosedCurve() != Logical.LFalse && bSplineCurve.Degree() > 1 && bSplineCurve.IsClosed)
        {
            bSplineCurve.SetPeriodic();
        }
        return bSplineCurve;
    }

    /// <summary>Converts a B-spline curve with knots (default factors).</summary>
    public static BSplineCurve3D? MakeBSplineCurve(StepBSplineCurve stepGeom_BSplineCurve) =>
        MakeBSplineCurve(stepGeom_BSplineCurve, new Factors());

    //=============================================================================
    // Creation d' une BSplineCurve de Geom2d a partir d' une
    // BSplineCurveWithKnotsAndRationalBSplineCurve de Step
    //=============================================================================

    /// <summary>Converts a B-spline curve with knots (rational or not) into a 2D curve.</summary>
    public static BSplineCurve2D? MakeBSplineCurve2d(StepBSplineCurve stepGeom_BSplineCurve, in Factors localFactors)
    {
        if (!MakeBSplineCurveKnots(stepGeom_BSplineCurve, out StepBSplineCurveWithKnots bSplineCurveWithKnots,
                                   out StepBSplineCurveWithKnotsAndRationalBSplineCurve bSplineCurveWithKnotsAndRationalBSplineCurve,
                                   out NCollection_Array1<double> uniqueKnots, out NCollection_Array1<int> uniqueKnotMultiplicities,
                                   out int firstMuultypisityDifference, out int lastMuultypisityDifference))
        {
            return null;
        }

        int degree = bSplineCurveWithKnots.Degree();
        int nbPoles = bSplineCurveWithKnots.NbControlPointsList();
        NCollection_HArray1<StepCartesianPoint> controlPointsList = bSplineCurveWithKnots.ControlPointsList();
        int summaryMuultypisityDifference = firstMuultypisityDifference + lastMuultypisityDifference;
        int nbUniquePoles = nbPoles - summaryMuultypisityDifference;
        if (nbUniquePoles <= 0)
        {
            return null;
        }
        NCollection_Array1<Point2D> poles = new NCollection_Array1<Point2D>(1, nbPoles - summaryMuultypisityDifference);

        for (int i = 1 + firstMuultypisityDifference; i <= nbPoles - lastMuultypisityDifference; ++i)
        {
            CartesianPoint2? point = MakeCartesianPoint2d(controlPointsList.Value(i), localFactors);
            if (point is not null)
            {
                Point2D gpPnt = point.Pnt2d();
                poles.SetValue(i - firstMuultypisityDifference, gpPnt);
            }
            else
            {
                return null;
            }
        }

        bool shouldBePeriodic = ShouldBePeriodic(uniqueKnotMultiplicities, nbPoles, degree);

        BSplineCurve2D bSplineCurve;
        if (stepGeom_BSplineCurve is StepBSplineCurveWithKnotsAndRationalBSplineCurve)
        {
            NCollection_Array1<double> uniqueWeights =
                UniqueWeights(bSplineCurveWithKnotsAndRationalBSplineCurve, nbPoles, firstMuultypisityDifference, lastMuultypisityDifference);
            bSplineCurve = new BSplineCurve2D(poles, uniqueWeights, uniqueKnots, uniqueKnotMultiplicities, degree, shouldBePeriodic);
        }
        else
        {
            bSplineCurve = new BSplineCurve2D(poles, uniqueKnots, uniqueKnotMultiplicities, degree, shouldBePeriodic);
        }

        // abv 04.07.00 CAX-IF TRJ4: trj4_k1_top-md-203.stp #716 (face #581):
        // force periodicity on closed curves
        // OCCT: the logical is tested as a C++ bool, so UNKNOWN counts as closed (occt-issues.md, 8D)
        if (stepGeom_BSplineCurve.ClosedCurve() != Logical.LFalse && bSplineCurve.Degree() > 1 && bSplineCurve.IsClosed)
        {
            bSplineCurve.SetPeriodic();
        }
        return bSplineCurve;
    }

    /// <summary>Converts a B-spline curve with knots into a 2D curve (default factors).</summary>
    public static BSplineCurve2D? MakeBSplineCurve2d(StepBSplineCurve stepGeom_BSplineCurve) =>
        MakeBSplineCurve2d(stepGeom_BSplineCurve, new Factors());

    //=============================================================================
    // Creation d' une BSplineSurface de Geom a partir d' une
    // BSplineSurface de Step
    //=============================================================================

    /// <summary>Converts a B-spline surface with knots (rational or not).</summary>
    public static BSplineSurface? MakeBSplineSurface(StepBSplineSurface ss, in Factors localFactors)
    {
        int i, j;
        StepBSplineSurfaceWithKnots bs;
        StepBSplineSurfaceWithKnotsAndRationalBSplineSurface bsr = null;

        if (ss is StepBSplineSurfaceWithKnotsAndRationalBSplineSurface)
        {
            bsr = ss as StepBSplineSurfaceWithKnotsAndRationalBSplineSurface;
            bs = bsr.BSplineSurfaceWithKnots();
        }
        else
        {
            bs = ss as StepBSplineSurfaceWithKnots;
        }

        int uDeg = bs.UDegree();
        int vDeg = bs.VDegree();
        int nuPoles = bs.NbControlPointsListI();
        int nvPoles = bs.NbControlPointsListJ();
        NCollection_HArray2<StepCartesianPoint> controlPointsList = bs.ControlPointsList();
        NCollection_Array2<Point3D> poles = new NCollection_Array2<Point3D>(1, nuPoles, 1, nvPoles);
        for (i = 1; i <= nuPoles; i++)
        {
            for (j = 1; j <= nvPoles; j++)
            {
                CartesianPoint? p = MakeCartesianPoint(controlPointsList.Value(i, j), localFactors);
                if (p is not null)
                {
                    poles.SetValue(i, j, p.Pnt());
                }
                else
                {
                    return null;
                }
            }
        }
        int nuKnots = bs.UMultiplicityCount;
        NCollection_HArray1<int> uMultiplicities = bs.UMultiplicities();
        NCollection_HArray1<double> uKnots = bs.UKnots();

        // count number of unique uknots
        double lastKnot = RealFirst();
        int nuKnotsUnique = 0;
        for (i = 1; i <= nuKnots; i++)
        {
            if (uKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                nuKnotsUnique++;
                lastKnot = uKnots.Value(i);
            }
        }

        // set umultiplicities and uknots
        NCollection_Array1<int> uMult = new NCollection_Array1<int>(1, nuKnotsUnique);
        NCollection_Array1<double> kUn = new NCollection_Array1<double>(1, nuKnotsUnique);
        int pos = 1;
        lastKnot = uKnots.Value(1);
        kUn.SetValue(1, uKnots.Value(1));
        uMult.SetValue(1, uMultiplicities.Value(1));
        for (i = 2; i <= nuKnots; i++)
        {
            if (uKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                pos++;
                kUn.SetValue(pos, uKnots.Value(i));
                uMult.SetValue(pos, uMultiplicities.Value(i));
                lastKnot = uKnots.Value(i);
            }
            else
            {
                // Knot not unique, increase multiplicity
                int curMult = uMult.Value(pos);
                uMult.SetValue(pos, curMult + uMultiplicities.Value(i));
            }
        }
        int nvKnots = bs.VMultiplicityCount;
        NCollection_HArray1<int> vMultiplicities = bs.VMultiplicities();
        NCollection_HArray1<double> vKnots = bs.VKnots();

        // count number of unique vknots
        lastKnot = RealFirst();
        int nvKnotsUnique = 0;
        for (i = 1; i <= nvKnots; i++)
        {
            if (vKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                nvKnotsUnique++;
                lastKnot = vKnots.Value(i);
            }
        }

        // set vmultiplicities and vknots
        NCollection_Array1<int> vMult = new NCollection_Array1<int>(1, nvKnotsUnique);
        NCollection_Array1<double> kVn = new NCollection_Array1<double>(1, nvKnotsUnique);
        pos = 1;
        lastKnot = vKnots.Value(1);
        kVn.SetValue(1, vKnots.Value(1));
        vMult.SetValue(1, vMultiplicities.Value(1));
        for (i = 2; i <= nvKnots; i++)
        {
            if (vKnots.Value(i) - lastKnot > Epsilon(Math.Abs(lastKnot)))
            {
                pos++;
                kVn.SetValue(pos, vKnots.Value(i));
                vMult.SetValue(pos, vMultiplicities.Value(i));
                lastKnot = vKnots.Value(i);
            }
            else
            {
                // Knot not unique, increase multiplicity
                int curMult = vMult.Value(pos);
                vMult.SetValue(pos, curMult + vMultiplicities.Value(i));
            }
        }

        // --- Does the Surface Descriptor LOOKS like a U and/or V Periodic ---
        // --- Descriptor ? ---

        // --- U Periodic ? ---

        int sumMult = 0;
        for (i = 1; i <= nuKnotsUnique; i++)
        {
            sumMult += uMult.Value(i);
        }

        bool shouldBeUPeriodic = false;
        if (sumMult == (nuPoles + uDeg + 1))
        {
            // shouldBeUPeriodic = false;
        }
        else if ((uMult.Value(1) == uMult.Value(nuKnotsUnique)) && ((sumMult - uMult.Value(1)) == nuPoles))
        {
            shouldBeUPeriodic = true;
        }

        // --- V Periodic ? ---

        sumMult = 0;
        for (i = 1; i <= nvKnotsUnique; i++)
        {
            sumMult += vMult.Value(i);
        }

        bool shouldBeVPeriodic = false;
        if (sumMult == (nvPoles + vDeg + 1))
        {
            // shouldBeVPeriodic = false;
        }
        else if ((vMult.Value(1) == vMult.Value(nvKnotsUnique)) && ((sumMult - vMult.Value(1)) == nvPoles))
        {
            shouldBeVPeriodic = true;
        }

        BSplineSurface cs;
        if (ss is StepBSplineSurfaceWithKnotsAndRationalBSplineSurface)
        {
            NCollection_HArray2<double> weight = bsr.WeightsData();
            NCollection_Array2<double> w = new NCollection_Array2<double>(1, nuPoles, 1, nvPoles);
            for (i = 1; i <= nuPoles; i++)
            {
                for (j = 1; j <= nvPoles; j++)
                {
                    w.SetValue(i, j, weight.Value(i, j));
                }
            }
            cs = new BSplineSurface(poles, w, kUn, kVn, uMult, vMult, uDeg, vDeg, shouldBeUPeriodic, shouldBeVPeriodic);
        }
        else
        {
            cs = new BSplineSurface(poles, kUn, kVn, uMult, vMult, uDeg, vDeg, shouldBeUPeriodic, shouldBeVPeriodic);
        }
        return cs;
    }

    /// <summary>Converts a B-spline surface with knots (default factors).</summary>
    public static BSplineSurface? MakeBSplineSurface(StepBSplineSurface ss) => MakeBSplineSurface(ss, new Factors());

    //=============================================================================
    // Creation d' un CartesianPoint de Geom a partir d' un CartesianPoint de Step
    //=============================================================================

    /// <summary>Converts a 3D cartesian_point (null if it hasn't three coordinates).</summary>
    public static CartesianPoint? MakeCartesianPoint(StepCartesianPoint sp, in Factors localFactors)
    {
        if (sp is not null && sp.CoordinateCount == 3)
        {
            double lf = localFactors.LengthFactor();
            double x = sp.CoordinatesValue(1) * lf;
            double y = sp.CoordinatesValue(2) * lf;
            double z = sp.CoordinatesValue(3) * lf;
            return new CartesianPoint(x, y, z);
        }
        return null;
    }

    /// <summary>Converts a 3D cartesian_point (default factors).</summary>
    public static CartesianPoint? MakeCartesianPoint(StepCartesianPoint sp) => MakeCartesianPoint(sp, new Factors());

    //=============================================================================
    // Creation d' un CartesianPoint de Geom2d a partir d' un CartesianPoint de
    // Step
    //=============================================================================

    /// <summary>Converts a 2D cartesian_point (null if it hasn't two coordinates). The factors aren't used.</summary>
    public static CartesianPoint2? MakeCartesianPoint2d(StepCartesianPoint sp, in Factors localFactors)
    {
        // OCCT: SP isn't checked for null (a null point is a null dereference; occt-issues.md, 8D)
        if (sp.CoordinateCount == 2)
        {
            double x = sp.CoordinatesValue(1);
            double y = sp.CoordinatesValue(2);
            return new CartesianPoint2(x, y);
        }
        return null;
    }

    /// <summary>Converts a 2D cartesian_point.</summary>
    public static CartesianPoint2? MakeCartesianPoint2d(StepCartesianPoint sp) => MakeCartesianPoint2d(sp, new Factors());

    //=============================================================================
    // Creation d' un Circle de Geom a partir d' un Circle de Step
    //=============================================================================

    /// <summary>Converts a circle placed by an axis2_placement_3d.</summary>
    public static CircleCurve3D? MakeCircle(StepCircle sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 2)
        {
            Axis2Placement? a = MakeAxis2Placement(axisSelect.Value() as StepAxis2Placement3d, localFactors);
            if (a is not null)
            {
                return new CircleCurve3D(a.Ax2(), sc.Radius() * localFactors.LengthFactor());
            }
        }
        return null;
    }

    /// <summary>Converts a circle (default factors).</summary>
    public static CircleCurve3D? MakeCircle(StepCircle sc) => MakeCircle(sc, new Factors());

    //=============================================================================
    // Creation d' un Circle de Geom2d a partir d' un Circle de Step
    //=============================================================================

    /// <summary>Converts a circle placed by an axis2_placement_2d.</summary>
    public static CircleCurve2D? MakeCircle2d(StepCircle sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 1)
        {
            AxisPlacement2? a1 = MakeAxisPlacement(axisSelect.Value() as StepAxis2Placement2d, localFactors);
            if (a1 is not null)
            {
                return new CircleCurve2D(a1.Ax2d(), sc.Radius());
            }
        }
        return null;
    }

    /// <summary>Converts a 2D circle (default factors).</summary>
    public static CircleCurve2D? MakeCircle2d(StepCircle sc) => MakeCircle2d(sc, new Factors());

    //=============================================================================
    // Creation d' une Conic de Geom a partir d' une Conic de Step
    //=============================================================================

    /// <summary>Converts a conic (circle, ellipse, hyperbola, parabola).</summary>
    public static ConicCurve3D? MakeConic(StepConic sc, in Factors localFactors)
    {
        if (sc is StepCircle)
        {
            return MakeCircle(sc as StepCircle, localFactors);
        }
        if (sc is StepEllipse)
        {
            return MakeEllipse(sc as StepEllipse, localFactors);
        }
        if (sc is StepHyperbola)
        {
            return MakeHyperbola(sc as StepHyperbola, localFactors);
        }
        if (sc is StepParabola)
        {
            return MakeParabola(sc as StepParabola, localFactors);
        }
        // Attention : Other conic shall be implemented !
        return null;
    }

    /// <summary>Converts a conic (default factors).</summary>
    public static ConicCurve3D? MakeConic(StepConic sc) => MakeConic(sc, new Factors());

    //=============================================================================
    // Creation d' une Conic de Geom2d a partir d' une Conic de Step
    //=============================================================================

    /// <summary>Converts a conic into a 2D conic.</summary>
    public static ConicCurve2D? MakeConic2d(StepConic sc, in Factors localFactors)
    {
        if (sc is StepCircle)
        {
            return MakeCircle2d(sc as StepCircle, localFactors);
        }
        if (sc is StepEllipse)
        {
            return MakeEllipse2d(sc as StepEllipse, localFactors);
        }
        if (sc is StepHyperbola)
        {
            return MakeHyperbola2d(sc as StepHyperbola, localFactors);
        }
        if (sc is StepParabola)
        {
            return MakeParabola2d(sc as StepParabola, localFactors);
        }
        // Attention : Other conic shall be implemented !
        return null;
    }

    /// <summary>Converts a conic into a 2D conic (default factors).</summary>
    public static ConicCurve2D? MakeConic2d(StepConic sc) => MakeConic2d(sc, new Factors());

    //=============================================================================
    // Creation d' une ConicalSurface de Geom a partir d' une ConicalSurface de
    // Step
    //=============================================================================

    /// <summary>Converts a conical_surface (its semi-angle at least Precision::Angular()).</summary>
    public static ConicalSurface? MakeConicalSurface(StepConicalSurface ss, in Factors localFactors)
    {
        Axis2Placement? a = MakeAxis2Placement(ss.Position(), localFactors);
        if (a is not null)
        {
            double r = ss.Radius() * localFactors.LengthFactor();
            double ang = ss.SemiAngle() * localFactors.PlaneAngleFactor();
            // #2(K3-3) rln 12/02/98 ProSTEP ct_turbine-A.stp entity #518, #3571 (gp::Resolution() is too
            // little)
            return new ConicalSurface(new CoordinateSystem(a.Ax2()), Max(ang, global::Kerf.Engine.Foundation.Precision.Angular()), r);
        }
        return null;
    }

    /// <summary>Converts a conical_surface (default factors).</summary>
    public static ConicalSurface? MakeConicalSurface(StepConicalSurface ss) => MakeConicalSurface(ss, new Factors());

    //=============================================================================
    // Creation d' une Curve de Geom a partir d' une Curve de Step
    //=============================================================================

    /// <summary>Converts a curve: lines, conics, bounded curves, curve replicas, 3D offset curves and surface curves.</summary>
    public static Curve3D? MakeCurve(StepCurve sc, in Factors localFactors)
    {
        if (sc is null)
        {
            return null;
        }
        // OCCT recurses without limit along a file's references (replicas, offsets, trimmed and surface curves): a
        // stack overflow in C++, which ends a .NET process, so the port throws when the stack runs low (decision 1240)
        RuntimeHelpers.EnsureSufficientExecutionStack();
        if (sc is StepLine)
        {
            return MakeLine(sc as StepLine, localFactors);
        }
        if (sc is StepTrimmedCurve)
        {
            return MakeTrimmedCurve(sc as StepTrimmedCurve, localFactors);
        }
        if (sc is StepConic)
        {
            return MakeConic(sc as StepConic, localFactors);
        }
        if (sc is StepBoundedCurve)
        {
            return MakeBoundedCurve(sc as StepBoundedCurve, localFactors);
        }
        if (sc is StepCurveReplica)
        { //: n7 abv 16 Feb 99
            StepCurveReplica cr = sc as StepCurveReplica;
            StepCurve pc = cr.ParentCurve();
            StepCartesianTransformationOperator3d t = cr.Transformation() as StepCartesianTransformationOperator3d;
            // protect against cyclic references and wrong type of cartop
            if (t is not null && pc != sc)
            {
                Curve3D? c1 = MakeCurve(pc, localFactors);
                if (c1 is not null)
                {
                    Transform3D t1 = new Transform3D();
                    if (MakeTransformation3d(t, ref t1, localFactors))
                    {
                        c1.Transform(t1);
                        return c1;
                    }
                }
            }
        }
        else if (sc is StepOffsetCurve3d)
        { //: o2 abv 17 Feb 99
            StepOffsetCurve3d oc = sc as StepOffsetCurve3d;
            StepCurve bc = oc.BasisCurve();
            if (bc != sc)
            { // protect against loop
                Curve3D? c1 = MakeCurve(bc, localFactors);
                if (c1 is not null)
                {
                    Direction? rd = MakeDirection(oc.RefDirection());
                    if (rd is not null)
                    {
                        return new OffsetCurve3D(c1, -oc.Distance(), rd.Dir());
                    }
                }
            }
        }
        else if (sc is StepSurfaceCurve)
        { //: o5 abv 17 Feb 99
            StepSurfaceCurve surfC = sc as StepSurfaceCurve;
            return MakeCurve(surfC.Curve3d(), localFactors);
        }
        return null;
    }

    /// <summary>Converts a curve (default factors).</summary>
    public static Curve3D? MakeCurve(StepCurve sc) => MakeCurve(sc, new Factors());

    //=============================================================================
    // Creation d' une Curve de Geom2d a partir d' une Curve de Step
    //=============================================================================

    /// <summary>Converts a curve into a 2D curve: lines, conics, bounded curves and curve replicas.</summary>
    public static Curve2D? MakeCurve2d(StepCurve sc, in Factors localFactors)
    {
        // as in MakeCurve (decision 1240)
        RuntimeHelpers.EnsureSufficientExecutionStack();
        if (sc is StepLine)
        {
            return MakeLine2d(sc as StepLine, localFactors);
        }
        if (sc is StepConic)
        {
            return MakeConic2d(sc as StepConic, localFactors);
        }
        if (sc is StepBoundedCurve)
        {
            return MakeBoundedCurve2d(sc as StepBoundedCurve, localFactors);
        }
        if (sc is StepCurveReplica)
        { //: n7 abv 16 Feb 99
            StepCurveReplica cr = sc as StepCurveReplica;
            StepCurve pc = cr.ParentCurve();
            StepCartesianTransformationOperator2d t = cr.Transformation() as StepCartesianTransformationOperator2d;
            // protect against cyclic references and wrong type of cartop
            if (t is not null && pc != sc)
            {
                Curve2D? c1 = MakeCurve2d(pc, localFactors);
                if (c1 is not null)
                {
                    Transform2D t1 = new Transform2D();
                    if (MakeTransformation2d(t, ref t1, localFactors))
                    {
                        c1.Transform(t1);
                        return c1;
                    }
                }
            }
        }
        return null;
    }

    /// <summary>Converts a curve into a 2D curve (default factors).</summary>
    public static Curve2D? MakeCurve2d(StepCurve sc) => MakeCurve2d(sc, new Factors());

    //=============================================================================
    // Creation d' une CylindricalSurface de Geom a partir d' une
    // CylindricalSurface de Step
    //=============================================================================

    /// <summary>Converts a cylindrical_surface.</summary>
    public static CylindricalSurface? MakeCylindricalSurface(StepCylindricalSurface ss, in Factors localFactors)
    {
        Axis2Placement? a = MakeAxis2Placement(ss.Position(), localFactors);
        if (a is not null)
        {
            return new CylindricalSurface(new CoordinateSystem(a.Ax2()), ss.Radius() * localFactors.LengthFactor());
        }
        return null;
    }

    /// <summary>Converts a cylindrical_surface (default factors).</summary>
    public static CylindricalSurface? MakeCylindricalSurface(StepCylindricalSurface ss) =>
        MakeCylindricalSurface(ss, new Factors());

    //=============================================================================
    // Creation d' un Direction de Geom a partir d' un Direction de Step
    //=============================================================================

    /// <summary>Converts a 3D direction (null if it has fewer than three ratios, an infinite one, or a null magnitude).</summary>
    public static Direction? MakeDirection(StepDirection sd)
    {
        // OCCT: SD isn't checked for null (occt-issues.md, 8D)
        if (sd.DirectionRatioCount >= 3)
        {
            double x = sd.DirectionRatiosValue(1);
            double y = sd.DirectionRatiosValue(2);
            double z = sd.DirectionRatiosValue(3);
            // 5.08.2021. Unstable test bugs xde bug24759: Y is very large value - FPE in SquareModulus
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(x) || global::Kerf.Engine.Foundation.Precision.IsInfinite(y) || global::Kerf.Engine.Foundation.Precision.IsInfinite(z))
            {
                return null;
            }
            // sln 22.10.2001. CTS23496: Direction is not created if it has null magnitude
            if (new Xyz(x, y, z).SquareModulus() > Constants.Resolution() * Constants.Resolution())
            {
                return new Direction(x, y, z);
            }
        }
        return null;
    }

    //=============================================================================
    // Creation d' un Direction de Geom2d a partir d' un Direction de Step
    //=============================================================================

    /// <summary>Converts a 2D direction (null if it has fewer than two ratios or a null magnitude).</summary>
    public static Direction2? MakeDirection2d(StepDirection sd)
    {
        // OCCT: SD isn't checked for null (occt-issues.md, 8D)
        if (sd.DirectionRatioCount >= 2)
        {
            double x = sd.DirectionRatiosValue(1);
            double y = sd.DirectionRatiosValue(2);
            // sln 23.10.2001. CTS23496: Direction is not created if it has null magnitude
            if (new Xy(x, y).SquareModulus() > Constants.Resolution() * Constants.Resolution())
            {
                return new Direction2(x, y);
            }
        }
        return null;
    }

    //=============================================================================
    // Creation d' une ElementarySurface de Geom a partir d' une
    // ElementarySurface de Step
    //=============================================================================

    /// <summary>Converts an elementary surface (plane, cylinder, cone, sphere, torus).</summary>
    public static ElementarySurface? MakeElementarySurface(StepElementarySurface ss, in Factors localFactors)
    {
        if (ss is StepPlane)
        {
            return MakePlane(ss as StepPlane, localFactors);
        }
        if (ss is StepCylindricalSurface)
        {
            return MakeCylindricalSurface(ss as StepCylindricalSurface, localFactors);
        }
        if (ss is StepConicalSurface)
        {
            return MakeConicalSurface(ss as StepConicalSurface, localFactors);
        }
        if (ss is StepSphericalSurface)
        {
            return MakeSphericalSurface(ss as StepSphericalSurface, localFactors);
        }
        if (ss is StepToroidalSurface)
        {
            return MakeToroidalSurface(ss as StepToroidalSurface, localFactors);
        }
        return null;
    }

    /// <summary>Converts an elementary surface (default factors).</summary>
    public static ElementarySurface? MakeElementarySurface(StepElementarySurface ss) =>
        MakeElementarySurface(ss, new Factors());

    //=============================================================================
    // Creation d' un Ellipse de Geom a partir d' un Ellipse de Step
    //=============================================================================

    /// <summary>Converts an ellipse (its axes swapped when the first semi-axis is the smaller).</summary>
    public static EllipseCurve3D? MakeEllipse(StepEllipse sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 2)
        {
            Axis2Placement? a1 = MakeAxis2Placement(axisSelect.Value() as StepAxis2Placement3d, localFactors);
            if (a1 is not null)
            {
                Frame3D a = a1.Ax2();
                double lf = localFactors.LengthFactor();
                double majorR = sc.SemiAxis1() * lf;
                double minorR = sc.SemiAxis2() * lf;
                if (majorR - minorR >= 0.0)
                { //: o9 abv 19 Feb 99
                    return new EllipseCurve3D(a, majorR, minorR);
                }
                //: o9 abv 19 Feb 99
                else
                {
                    a.SetXDirection(a.XDirection() ^ a.Direction());
                    return new EllipseCurve3D(a, minorR, majorR);
                }
            }
        }
        return null;
    }

    /// <summary>Converts an ellipse (default factors).</summary>
    public static EllipseCurve3D? MakeEllipse(StepEllipse sc) => MakeEllipse(sc, new Factors());

    //=============================================================================
    // Creation d' un Ellipse de Geom2d a partir d' un Ellipse de Step
    //=============================================================================

    /// <summary>Converts an ellipse into a 2D ellipse.</summary>
    public static EllipseCurve2D? MakeEllipse2d(StepEllipse sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 1)
        {
            AxisPlacement2? a1 = MakeAxisPlacement(axisSelect.Value() as StepAxis2Placement2d, localFactors);
            if (a1 is not null)
            {
                Frame2D a = new Frame2D(a1.Ax2d());
                double majorR = sc.SemiAxis1();
                double minorR = sc.SemiAxis2();
                if (majorR - minorR >= 0.0)
                { //: o9 abv 19 Feb 99: bm4_id_punch_b.stp #678: protection
                    return new EllipseCurve2D(a, majorR, minorR);
                }
                else
                {
                    Direction2D x = a.XDirection();
                    a.SetXDirection(new Direction2D(x.X(), -x.Y()));
                    return new EllipseCurve2D(a, minorR, majorR);
                }
            }
        }
        return null;
    }

    /// <summary>Converts an ellipse into a 2D ellipse (default factors).</summary>
    public static EllipseCurve2D? MakeEllipse2d(StepEllipse sc) => MakeEllipse2d(sc, new Factors());

    //=============================================================================
    // Creation d' un Hyperbola de Geom a partir d' un Hyperbola de Step
    //=============================================================================

    /// <summary>Converts a hyperbola.</summary>
    public static HyperbolaCurve3D? MakeHyperbola(StepHyperbola sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 2)
        {
            Axis2Placement? a1 = MakeAxis2Placement(axisSelect.Value() as StepAxis2Placement3d, localFactors);
            if (a1 is not null)
            {
                Frame3D a = a1.Ax2();
                double lf = localFactors.LengthFactor();
                return new HyperbolaCurve3D(a, sc.SemiAxis() * lf, sc.SemiImagAxis() * lf);
            }
        }
        return null;
    }

    /// <summary>Converts a hyperbola (default factors).</summary>
    public static HyperbolaCurve3D? MakeHyperbola(StepHyperbola sc) => MakeHyperbola(sc, new Factors());

    //=============================================================================
    // Creation d' un Hyperbola de Geom2d a partir d' un Hyperbola de Step
    //=============================================================================

    /// <summary>Converts a hyperbola into a 2D hyperbola.</summary>
    public static HyperbolaCurve2D? MakeHyperbola2d(StepHyperbola sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 1)
        {
            AxisPlacement2? a1 = MakeAxisPlacement(axisSelect.Value() as StepAxis2Placement2d, localFactors);
            if (a1 is not null)
            {
                Frame2D a = new Frame2D(a1.Ax2d());
                return new HyperbolaCurve2D(a, sc.SemiAxis(), sc.SemiImagAxis());
            }
        }
        return null;
    }

    /// <summary>Converts a hyperbola into a 2D hyperbola (default factors).</summary>
    public static HyperbolaCurve2D? MakeHyperbola2d(StepHyperbola sc) => MakeHyperbola2d(sc, new Factors());

    //=============================================================================
    // Creation d' une Line de Geom a partir d' une Line de Step
    //=============================================================================

    /// <summary>Converts a line (null if its vector is shorter than Precision::Confusion()).</summary>
    public static LineCurve3D? MakeLine(StepLine sc, in Factors localFactors)
    {
        CartesianPoint? p = MakeCartesianPoint(sc.Pnt(), localFactors);
        if (p is not null)
        {
            // sln 22.10.2001. CTS23496: Line is not created if direction have not been successfully created
            VectorWithMagnitude? d = MakeVectorWithMagnitude(sc.Dir(), localFactors);
            if (d is not null)
            {
                if (d.Vec().SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return null;
                }
                Direction3D v = new Direction3D(d.Vec());
                return new LineCurve3D(p.Pnt(), v);
            }
        }
        return null;
    }

    /// <summary>Converts a line (default factors).</summary>
    public static LineCurve3D? MakeLine(StepLine sc) => MakeLine(sc, new Factors());

    //=============================================================================
    // Creation d' une Line de Geom2d a partir d' une Line de Step
    //=============================================================================

    /// <summary>Converts a line into a 2D line.</summary>
    public static LineCurve2D? MakeLine2d(StepLine sc, in Factors localFactors)
    {
        CartesianPoint2? p = MakeCartesianPoint2d(sc.Pnt(), localFactors);
        if (p is not null)
        {
            // sln 23.10.2001. CTS23496: Line is not created if direction have not been successfully created
            VectorWithMagnitude2? d = MakeVectorWithMagnitude2d(sc.Dir());
            if (d is not null)
            {
                Direction2D d1 = new Direction2D(d.Vec2d());
                return new LineCurve2D(p.Pnt2d(), d1);
            }
        }
        return null;
    }

    /// <summary>Converts a line into a 2D line (default factors).</summary>
    public static LineCurve2D? MakeLine2d(StepLine sc) => MakeLine2d(sc, new Factors());

    //=============================================================================
    // Creation d' un Parabola de Geom a partir d' un Parabola de Step
    //=============================================================================

    /// <summary>Converts a parabola.</summary>
    public static ParabolaCurve3D? MakeParabola(StepParabola sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 2)
        {
            Axis2Placement? a = MakeAxis2Placement(axisSelect.Value() as StepAxis2Placement3d, localFactors);
            if (a is not null)
            {
                return new ParabolaCurve3D(a.Ax2(), sc.FocalDist() * localFactors.LengthFactor());
            }
        }
        return null;
    }

    /// <summary>Converts a parabola (default factors).</summary>
    public static ParabolaCurve3D? MakeParabola(StepParabola sc) => MakeParabola(sc, new Factors());

    //=============================================================================
    // Creation d' un Parabola de Geom2d a partir d' un Parabola de Step
    //=============================================================================

    /// <summary>Converts a parabola into a 2D parabola.</summary>
    public static ParabolaCurve2D? MakeParabola2d(StepParabola sc, in Factors localFactors)
    {
        StepAxis2Placement axisSelect = sc.Position();
        if (axisSelect.CaseNum(axisSelect.Value()) == 1)
        {
            AxisPlacement2? a1 = MakeAxisPlacement(axisSelect.Value() as StepAxis2Placement2d, localFactors);
            if (a1 is not null)
            {
                Frame2D a = new Frame2D(a1.Ax2d());
                return new ParabolaCurve2D(a, sc.FocalDist());
            }
        }
        return null;
    }

    /// <summary>Converts a parabola into a 2D parabola (default factors).</summary>
    public static ParabolaCurve2D? MakeParabola2d(StepParabola sc) => MakeParabola2d(sc, new Factors());

    //=============================================================================
    // Creation d' un Plane de Geom a partir d' un plane de Step
    //=============================================================================

    /// <summary>Converts a plane.</summary>
    public static PlaneSurface? MakePlane(StepPlane sp, in Factors localFactors)
    {
        Axis2Placement? a = MakeAxis2Placement(sp.Position(), localFactors);
        if (a is not null)
        {
            return new PlaneSurface(new CoordinateSystem(a.Ax2()));
        }
        return null;
    }

    /// <summary>Converts a plane (default factors).</summary>
    public static PlaneSurface? MakePlane(StepPlane sp) => MakePlane(sp, new Factors());

    //=================================================================================================

    /// <summary>Converts a polyline into a B-spline curve of degree 1.</summary>
    public static BSplineCurve3D? MakePolyline(StepPolyline spl, in Factors localFactors)
    {
        if (spl is null)
        {
            return null;
        }

        int nbp = spl.NbPoints();
        if (nbp > 1)
        {
            NCollection_Array1<Point3D> poles = new NCollection_Array1<Point3D>(1, nbp);
            NCollection_Array1<double> knots = new NCollection_Array1<double>(1, nbp);
            NCollection_Array1<int> mults = new NCollection_Array1<int>(1, nbp);

            for (int i = 1; i <= nbp; i++)
            {
                CartesianPoint? p = MakeCartesianPoint(spl.PointsValue(i), localFactors);
                if (p is not null)
                {
                    poles.SetValue(i, p.Pnt());
                }
                else
                {
                    return null;
                }
                knots.SetValue(i, (double)(i - 1));
                mults.SetValue(i, 1);
            }
            mults.SetValue(1, 2);
            mults.SetValue(nbp, 2);

            return new BSplineCurve3D(poles, knots, mults, 1);
        }
        return null;
    }

    /// <summary>Converts a polyline (default factors).</summary>
    public static BSplineCurve3D? MakePolyline(StepPolyline spl) => MakePolyline(spl, new Factors());

    //=================================================================================================

    /// <summary>Converts a polyline into a 2D B-spline curve of degree 1.</summary>
    public static BSplineCurve2D? MakePolyline2d(StepPolyline spl, in Factors localFactors)
    {
        if (spl is null)
        {
            return null;
        }

        int nbp = spl.NbPoints();
        if (nbp > 1)
        {
            NCollection_Array1<Point2D> poles = new NCollection_Array1<Point2D>(1, nbp);
            NCollection_Array1<double> knots = new NCollection_Array1<double>(1, nbp);
            NCollection_Array1<int> mults = new NCollection_Array1<int>(1, nbp);

            for (int i = 1; i <= nbp; i++)
            {
                CartesianPoint2? p = MakeCartesianPoint2d(spl.PointsValue(i), localFactors);
                if (p is not null)
                {
                    poles.SetValue(i, p.Pnt2d());
                }
                else
                {
                    return null;
                }
                knots.SetValue(i, (double)(i - 1));
                mults.SetValue(i, 1);
            }
            mults.SetValue(1, 2);
            mults.SetValue(nbp, 2);

            return new BSplineCurve2D(poles, knots, mults, 1);
        }
        return null;
    }

    /// <summary>Converts a polyline into a 2D curve (default factors).</summary>
    public static BSplineCurve2D? MakePolyline2d(StepPolyline spl) => MakePolyline2d(spl, new Factors());

    //=============================================================================
    // Creation d' une RectangularTrimmedSurface de Geom a partir d' une
    // RectangularTrimmedSurface de Step
    //=============================================================================

    /// <summary>Converts a rectangular_trimmed_surface (its parameters scaled by the basis surface's units).</summary>
    public static TrimmedSurface? MakeRectangularTrimmedSurface(StepRectangularTrimmedSurface ss, in Factors localFactors)
    {
        Surface? theBasis = MakeSurface(ss.BasisSurface(), localFactors);
        if (theBasis is not null)
        {
            // -----------------------------------------
            // Modification of the Trimming Parameters ?
            // -----------------------------------------

            double uFact = 1.0;
            double vFact = 1.0;
            double lengthFact = localFactors.LengthFactor();
            double angleFact = localFactors.PlaneAngleFactor(); // abv 30.06.00 trj4_k1_geo-tc-214.stp #1477: PI/180.;

            if (theBasis is SphericalSurface || theBasis is ToroidalSurface)
            {
                uFact = vFact = angleFact;
            }
            else if (theBasis is CylindricalSurface)
            {
                uFact = angleFact;
                vFact = lengthFact;
            }
            else if (theBasis is RevolutionSurface)
            {
                uFact = angleFact;
            }
            else if (theBasis is ConicalSurface)
            {
                ConicalSurface conicS = theBasis as ConicalSurface;
                uFact = angleFact;
                vFact = lengthFact / Math.Cos(conicS.SemiAngle());
            }
            else if (theBasis is PlaneSurface)
            {
                uFact = vFact = lengthFact;
            }

            double u1 = ss.U1() * uFact;
            double u2 = ss.U2() * uFact;
            double v1 = ss.V1() * vFact;
            double v2 = ss.V2() * vFact;

            return new TrimmedSurface(theBasis, u1, u2, v1, v2, ss.Usense(), ss.Vsense());
        }
        return null;
    }

    /// <summary>Converts a rectangular_trimmed_surface (default factors).</summary>
    public static TrimmedSurface? MakeRectangularTrimmedSurface(StepRectangularTrimmedSurface ss) =>
        MakeRectangularTrimmedSurface(ss, new Factors());

    //=============================================================================
    // Creation d' une SphericalSurface de Geom a partir d' une
    // SphericalSurface de Step
    //=============================================================================

    /// <summary>Converts a spherical_surface.</summary>
    public static SphericalSurface? MakeSphericalSurface(StepSphericalSurface ss, in Factors localFactors)
    {
        Axis2Placement? a = MakeAxis2Placement(ss.Position(), localFactors);
        if (a is not null)
        {
            return new SphericalSurface(new CoordinateSystem(a.Ax2()), ss.Radius() * localFactors.LengthFactor());
        }
        return null;
    }

    /// <summary>Converts a spherical_surface (default factors).</summary>
    public static SphericalSurface? MakeSphericalSurface(StepSphericalSurface ss) => MakeSphericalSurface(ss, new Factors());

    //=============================================================================
    // Creation d' une Surface de Geom a partir d' une Surface de Step
    //=============================================================================

    /// <summary>
    /// Converts a surface: bounded, elementary and swept surfaces, offset surfaces (a C0 basis made C1 through Shape
    /// Healing's <see cref="ShapeAlgo.AlgoContainer"/>) and surface replicas. Null on any failure.
    /// </summary>
    public static Surface? MakeSurface(StepSurface ss, in Factors localFactors)
    {
        // sln 01.10.2001 BUC61003. If entry shell is NULL do nothing
        if (ss is null)
        {
            return null;
        }

        try
        {
            // OCC_CATCH_SIGNALS
            // as in MakeCurve (decision 1240): the exception is caught below, as OCCT catches its stack overflow
            RuntimeHelpers.EnsureSufficientExecutionStack();
            if (ss is StepBoundedSurface)
            {
                return MakeBoundedSurface(ss as StepBoundedSurface, localFactors);
            }
            if (ss is StepElementarySurface)
            {
                StepElementarySurface s1 = ss as StepElementarySurface;
                if (s1.Position() is null)
                {
                    return null;
                }

                return MakeElementarySurface(s1, localFactors);
            }
            if (ss is StepSweptSurface)
            {
                return MakeSweptSurface(ss as StepSweptSurface, localFactors);
            }
            if (ss is StepOffsetSurface)
            { //: d4 abv 12 Mar 98
                StepOffsetSurface os = ss as StepOffsetSurface;

                Surface? basisSurface = MakeSurface(os.BasisSurface(), localFactors);
                if (basisSurface is not null)
                {
                    // sln 03.10.01. BUC61003. creation of  offset surface is corrected
                    double offset = os.Distance() * localFactors.LengthFactor();
                    if (basisSurface.Continuity() == C0)
                    {
                        MakeFace bFace = new MakeFace(basisSurface, global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (bFace.IsDone)
                        {
                            // OCCT: the process-wide container (null before ShapeAlgo.Init, as in OCCT)
                            Shape result = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().C0ShapeToC1Shape(bFace.Face(), Math.Abs(offset));
                            if (result.ShapeType() == ShapeType.Face)
                            {
                                basisSurface = Tool.Surface(ShapeCasts.Face(result));
                            }
                        }
                    }
                    if (basisSurface.Continuity() != C0)
                    {
                        return new OffsetSurface(basisSurface, offset);
                    }
                }
            }
            else if (ss is StepSurfaceReplica)
            { //: n7 abv 16 Feb 99
                StepSurfaceReplica sr = ss as StepSurfaceReplica;
                StepSurface ps = sr.ParentSurface();
                StepCartesianTransformationOperator3d t = sr.Transformation();
                // protect against cyclic references and wrong type of cartop
                if (t is not null && ps != ss)
                {
                    Surface? s1 = MakeSurface(ps, localFactors);
                    if (s1 is not null)
                    {
                        Transform3D t1 = new Transform3D();
                        if (MakeTransformation3d(t, ref t1, localFactors))
                        {
                            s1.Transform(t1);
                            return s1;
                        }
                    }
                }
            }
        }
        // OCCT catches Standard_Failure under OCC_CATCH_SIGNALS, so system signals too: their .NET counterparts are
        // the other exceptions (decision 1183)
        catch (Exception exception) when (exception is not OutOfMemoryException and not OperationCanceledException)
        {
            // OCCT_DEBUG output left out
        }
        return null;
    }

    /// <summary>Converts a surface (default factors).</summary>
    public static Surface? MakeSurface(StepSurface ss) => MakeSurface(ss, new Factors());

    //=============================================================================
    // Creation d' une SurfaceOfLinearExtrusion de Geom a partir d' une
    // SurfaceOfLinearExtrusion de Step
    //=============================================================================

    /// <summary>Converts a surface_of_linear_extrusion (null for a line extruded along itself).</summary>
    public static ExtrusionSurface? MakeSurfaceOfLinearExtrusion(StepSurfaceOfLinearExtrusion ss, in Factors localFactors)
    {
        Curve3D? c = MakeCurve(ss.SweptCurve(), localFactors);
        if (c is not null)
        {
            // sln 23.10.2001. CTS23496: Surface is not created if extrusion axis have not been successfully
            // created
            VectorWithMagnitude? v = MakeVectorWithMagnitude(ss.ExtrusionAxis(), localFactors);
            if (v is not null)
            {
                Direction3D d = new Direction3D(v.Vec());
                LineCurve3D line = c as LineCurve3D;
                if (line is not null && line.Lin().Direction().IsParallel(d, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return null;
                }
                return new ExtrusionSurface(c, d);
            }
        }
        return null;
    }

    /// <summary>Converts a surface_of_linear_extrusion (default factors).</summary>
    public static ExtrusionSurface? MakeSurfaceOfLinearExtrusion(StepSurfaceOfLinearExtrusion ss) =>
        MakeSurfaceOfLinearExtrusion(ss, new Factors());

    //=============================================================================
    // Creation d' une SurfaceOfRevolution de Geom a partir d' une
    // SurfaceOfRevolution de Step
    //=============================================================================

    /// <summary>Converts a surface_of_revolution (a circle or ellipse through the axis trimmed to a half).</summary>
    public static RevolutionSurface? MakeSurfaceOfRevolution(StepSurfaceOfRevolution ss, in Factors localFactors)
    {
        Curve3D? c = MakeCurve(ss.SweptCurve(), localFactors);
        if (c is not null)
        {
            Axis1Placement? a1 = MakeAxis1Placement(ss.AxisPosition(), localFactors);
            if (a1 is not null)
            {
                Axis3D a = a1.Ax1();
                // skl for OCC952 (one bad case revolution of circle)
                if (c is CircleCurve3D || c is EllipseCurve3D)
                {
                    ConicCurve3D conic = c as ConicCurve3D;
                    Point3D pc = conic.Location();
                    Line3D rl = new Line3D(a);
                    if (rl.Distance(pc) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    { // pc lies on A2
                        Direction3D dirline = a.Direction();
                        Direction3D norm = conic.Axis().Direction();
                        Direction3D xAxis = conic.XAxis().Direction();
                        // checking A2 lies on plane of circle
                        if (dirline.IsNormal(norm, global::Kerf.Engine.Foundation.Precision.Angular())
                            && (dirline.IsParallel(xAxis, global::Kerf.Engine.Foundation.Precision.Angular()) || c is CircleCurve3D))
                        {
                            // change parametrization for trimming
                            Frame3D axnew = new Frame3D(pc, norm, dirline.Reversed());
                            conic.SetPosition(axnew);
                            c = new TrimmedCurve3D(conic, 0.0, M_PI);
                        }
                    }
                }
                return new RevolutionSurface(c, a);
            }
        }
        return null;
    }

    /// <summary>Converts a surface_of_revolution (default factors).</summary>
    public static RevolutionSurface? MakeSurfaceOfRevolution(StepSurfaceOfRevolution ss) =>
        MakeSurfaceOfRevolution(ss, new Factors());

    //=============================================================================
    // Creation d' une SweptSurface de prostep a partir d' une
    // SweptSurface de Geom
    //=============================================================================

    /// <summary>Converts a swept surface (linear extrusion or revolution).</summary>
    public static SweptSurface? MakeSweptSurface(StepSweptSurface ss, in Factors localFactors)
    {
        if (ss is StepSurfaceOfLinearExtrusion)
        {
            return MakeSurfaceOfLinearExtrusion(ss as StepSurfaceOfLinearExtrusion, localFactors);
        }
        if (ss is StepSurfaceOfRevolution)
        {
            return MakeSurfaceOfRevolution(ss as StepSurfaceOfRevolution, localFactors);
        }
        return null;
    }

    /// <summary>Converts a swept surface (default factors).</summary>
    public static SweptSurface? MakeSweptSurface(StepSweptSurface ss) => MakeSweptSurface(ss, new Factors());

    //=============================================================================
    // Creation d' une ToroidalSurface de Geom a partir d' une
    // ToroidalSurface de Step
    //=============================================================================

    /// <summary>Converts a toroidal_surface (its radii taken positive).</summary>
    public static ToroidalSurface? MakeToroidalSurface(StepToroidalSurface ss, in Factors localFactors)
    {
        Axis2Placement? a = MakeAxis2Placement(ss.Position(), localFactors);
        if (a is not null)
        {
            double lf = localFactors.LengthFactor();
            return new ToroidalSurface(new CoordinateSystem(a.Ax2()), Math.Abs(ss.MajorRadius() * lf), Math.Abs(ss.MinorRadius() * lf));
        }
        return null;
    }

    /// <summary>Converts a toroidal_surface (default factors).</summary>
    public static ToroidalSurface? MakeToroidalSurface(StepToroidalSurface ss) => MakeToroidalSurface(ss, new Factors());

    //=================================================================================================

    /// <summary>
    /// Converts a 2D cartesian transformation operator into the rigid motion it describes (its inverse, as OCCT
    /// takes it); false if its origin can't be converted (<paramref name="ct"/> unchanged).
    /// </summary>
    public static bool MakeTransformation2d(StepCartesianTransformationOperator2d scto, ref Transform2D ct, in Factors localFactors)
    {
        //  NB : on ne s interesse ici qu au deplacement rigide
        CartesianPoint2? cp = MakeCartesianPoint2d(scto.LocalOrigin(), localFactors);
        if (cp is not null)
        {
            Direction2D d1 = new Direction2D(Direction2D.D.X);
            // sln 23.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used
            StepDirection a = scto.Axis1();
            if (a is not null)
            {
                Direction2? d = MakeDirection2d(a);
                if (d is not null)
                {
                    d1 = d.Dir2d();
                }
            }
            Axis2D result = new Axis2D(cp.Pnt2d(), d1);
            ct.SetTransformation(result);
            ct = ct.Inverted();
            return true;
        }
        return false;
    }

    /// <summary>Converts a 2D cartesian transformation operator (default factors).</summary>
    public static bool MakeTransformation2d(StepCartesianTransformationOperator2d scto, ref Transform2D ct) =>
        MakeTransformation2d(scto, ref ct, new Factors());

    //=================================================================================================

    /// <summary>
    /// Converts a 3D cartesian transformation operator into the transformation it describes (its inverse, as OCCT
    /// takes it, with its scale); false if its origin can't be converted (<paramref name="ct"/> unchanged).
    /// </summary>
    public static bool MakeTransformation3d(StepCartesianTransformationOperator3d scto, ref Transform3D ct, in Factors localFactors)
    {
        CartesianPoint? cp = MakeCartesianPoint(scto.LocalOrigin(), localFactors);
        if (cp is not null)
        {
            Point3D pgp = cp.Pnt();

            // sln 23.10.2001. CTS23496: If problems with creation of direction occur default direction is
            // used
            Direction3D d1 = new Direction3D(Direction3D.D.X);
            StepDirection a1 = scto.Axis1();
            if (a1 is not null)
            {
                Direction? d = MakeDirection(a1);
                if (d is not null)
                {
                    d1 = d.Dir();
                }
            }

            Direction3D d2 = new Direction3D(Direction3D.D.Y);
            StepDirection a2 = scto.Axis2();
            if (a2 is not null)
            {
                Direction? d = MakeDirection(a2);
                if (d is not null)
                {
                    d2 = d.Dir();
                }
            }

            bool isDefaultDirectionUsed = true;
            Direction3D d3 = new Direction3D();
            StepDirection a3 = scto.Axis3();
            if (a3 is not null)
            {
                Direction? d = MakeDirection(a3);
                if (d is not null)
                {
                    d3 = d.Dir();
                    isDefaultDirectionUsed = false;
                }
            }
            if (isDefaultDirectionUsed)
            {
                d3 = d1.Crossed(d2);
            }

            CoordinateSystem result = new CoordinateSystem(pgp, d3, d1);
            ct.SetTransformation(result);
            if (scto.HasScale)
            {
                ct.SetScaleFactor(scto.Scale());
            }
            ct = ct.Inverted(); //: n8 abv 16 Feb 99: tr8_as2_db.stp: reverse for accordance with LV tool
            return true;
        }
        return false;
    }

    /// <summary>Converts a 3D cartesian transformation operator (default factors).</summary>
    public static bool MakeTransformation3d(StepCartesianTransformationOperator3d scto, ref Transform3D ct) =>
        MakeTransformation3d(scto, ref ct, new Factors());

    // ----------------------------------------------------------------
    // ExtractParameter
    // ----------------------------------------------------------------
    //: o6 abv 18 Feb 99: parameter Factor added
    //: p3 abv 23 Feb 99: parameter Shift added
    private static bool ExtractParameter(Curve3D aGeomCurve,
                                         NCollection_HArray1<StepTrimmingSelect> ts,
                                         int nbSel,
                                         int masterRep,
                                         double factor,
                                         double shift,
                                         ref double aParam,
                                         in Factors localFactors)
    {
        StepCartesianPoint point;
        int i;
        //: S4136  double precBrep = BRepAPI::Precision();
        for (i = 1; i <= nbSel; i++)
        {
            StepTrimmingSelect theSel = ts.Value(i);
            if (masterRep == 2 && theSel.CaseMember() > 0)
            {
                aParam = shift + factor * theSel.ParameterValue();
                return true;
            }
            else if (masterRep == 1 && theSel.CaseNumber() > 0)
            {
                point = theSel.CartesianPoint();
                CartesianPoint? theGeomPnt = MakeCartesianPoint(point, localFactors);
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
            StepTrimmingSelect theSel = ts.Value(i);
            if (theSel.CaseMember() > 0)
            {
                aParam = shift + factor * theSel.ParameterValue();

                return true;
            }
        }
        // if no ParameterValue exists, it is created from the CartesianPointValue

        for (i = 1; i <= nbSel; i++)
        {
            StepTrimmingSelect theSel = ts.Value(i);
            if (theSel.CaseNumber() > 0)
            {
                point = theSel.CartesianPoint();
                CartesianPoint? theGeomPnt = MakeCartesianPoint(point, localFactors);
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
    public static TrimmedCurve3D? MakeTrimmedCurve(StepTrimmedCurve sc, in Factors localFactors)
    {
        StepCurve theSTEPCurve = sc.BasisCurve();
        Curve3D? theCurve = MakeCurve(theSTEPCurve, localFactors);
        if (theCurve is null)
        {
            return null;
        }

        NCollection_HArray1<StepTrimmingSelect> theTrimSel1 = sc.Trim1();
        NCollection_HArray1<StepTrimmingSelect> theTrimSel2 = sc.Trim2();
        int nbSel1 = sc.Trim1Count;
        int nbSel2 = sc.Trim2Count;

        int masterRep;
        switch (sc.MasterRepresentation())
        {
            case StepTrimmingPreference.TpCartesian:
                masterRep = 1;
                break;
            case StepTrimmingPreference.TpParameter:
                masterRep = 2;
                break;
            default:
                masterRep = 0;
                break;
        }

        // gka 18.02.04 analysis for case when MasterRep = .Unspecified
        // and parameters are specified as CARTESIAN_POINT
        bool isPoint = false;
        if (masterRep == 0 || (masterRep == 2 && nbSel1 > 1 && nbSel2 > 1))
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
        if (theSTEPCurve is StepLine)
        {
            StepLine theLine = theSTEPCurve as StepLine;
            fact = theLine.Dir().Magnitude() * localFactors.LengthFactor();
        }
        else if (theSTEPCurve is StepCircle || theSTEPCurve is StepEllipse)
        {
            //    if (trim1 > 2.1*M_PI || trim2 > 2.1*M_PI) fact = M_PI / 180.;
            fact = localFactors.PlaneAngleFactor();
            //: p3 abv 23 Feb 99: shift on pi/2 on ellipse with R1 < R2
            StepEllipse ellipse = theSTEPCurve as StepEllipse;
            if (ellipse is not null && ellipse.SemiAxis1() - ellipse.SemiAxis2() < 0.0)
            {
                shift = 0.5 * M_PI;
            }

            // skl 04.02.2002 for OCC133: we can not make TrimmedCurve if
            // there is no X-direction in StepGeom_Axis2Placement3d
            StepConic conic = theSTEPCurve as StepConic;
            // CKY 6-FEB-2004 for Airbus-MedialAxis :
            // this restriction does not apply for trimming by POINTS
            if (conic is not null && masterRep != 1)
            {
                StepAxis2Placement a2p = conic.Position();
                if (a2p.CaseNum(a2p.Value()) == 2)
                {
                    if (!a2p.Axis2Placement3d().HasRefDirection)
                    {
                        ////gka 18.02.04 analysis for case when MasterRep = .Unspecified
                        // and parameters are specified as CARTESIAN_POINT
                        if (isPoint /*&& !MasterRep*/)
                        {
                            masterRep = 1;
                        }
                        else
                        {
                            if (sc.SenseAgreement())
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
        bool foundParam1 = ExtractParameter(theCurve, theTrimSel1, nbSel1, masterRep, fact, shift, ref trim1, localFactors);
        bool foundParam2 = ExtractParameter(theCurve, theTrimSel2, nbSel2, masterRep, fact, shift, ref trim2, localFactors);

        if (foundParam1 && foundParam2)
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
            if (sc.SenseAgreement())
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
    public static TrimmedCurve3D? MakeTrimmedCurve(StepTrimmedCurve sc) => MakeTrimmedCurve(sc, new Factors());

    //=============================================================================
    // Creation d'une Trimmed Curve de Geom2d a partir d' une Trimmed Curve de Step
    //=============================================================================
    // Shall be completed to treat trimming with points

    /// <summary>
    /// Converts a trimmed_curve into a 2D B-spline curve (trims by parameter only; a B-spline basis curve is
    /// returned untrimmed).
    /// </summary>
    public static BSplineCurve2D? MakeTrimmedCurve2d(StepTrimmedCurve sc, in Factors localFactors)
    {
        StepCurve basisCurve = sc.BasisCurve();
        Curve2D? theGeomBasis = MakeCurve2d(basisCurve, localFactors);
        if (theGeomBasis is null)
        {
            return null;
        }

        if (theGeomBasis is BSplineCurve2D)
        {
            return theGeomBasis as BSplineCurve2D;
        }

        NCollection_HArray1<StepTrimmingSelect> theTrimSel1 = sc.Trim1();
        NCollection_HArray1<StepTrimmingSelect> theTrimSel2 = sc.Trim2();
        int nbSel1 = sc.Trim1Count;
        int nbSel2 = sc.Trim2Count;
        if ((nbSel1 == 1) && (nbSel2 == 1) && (theTrimSel1.Value(1).CaseMember() > 0) && (theTrimSel2.Value(1).CaseMember() > 0))
        {
            double u1 = theTrimSel1.Value(1).ParameterValue();
            double u2 = theTrimSel2.Value(1).ParameterValue();
            double fact = 1.0, shift = 0.0;

            if (basisCurve is StepLine)
            {
                StepLine theLine = basisCurve as StepLine;
                fact = theLine.Dir().Magnitude();
            }
            else if (basisCurve is StepCircle || basisCurve is StepEllipse)
            {
                //      if (u1 > 2.1*M_PI || u2 > 2.1*M_PI) fact = M_PI / 180.;
                fact = localFactors.PlaneAngleFactor();
                //: p3 abv 23 Feb 99: shift on pi/2 on ellipse with R1 < R2
                StepEllipse ellipse = basisCurve as StepEllipse;
                if (ellipse is not null && ellipse.SemiAxis1() - ellipse.SemiAxis2() < 0.0)
                {
                    shift = 0.5 * M_PI;
                }
            }
            else if (basisCurve is StepParabola || basisCurve is StepHyperbola)
            {
                // LATER !!!
            }
            //    CKY 16-DEC-1997 : USA60035 le texte de Part42 parle de degres
            //      mais des systemes ecrivent en radians. Exploiter UnitsMethods

            double newU1 = shift + u1 * fact;
            double newU2 = shift + u2 * fact;

            TrimmedCurve2D theTrimmed = new TrimmedCurve2D(theGeomBasis, newU1, newU2, sc.SenseAgreement());
            return Geom2dConvert2D.CurveToBSplineCurve(theTrimmed);
        }
        return null;
    }

    /// <summary>Converts a trimmed_curve into a 2D B-spline curve (default factors).</summary>
    public static BSplineCurve2D? MakeTrimmedCurve2d(StepTrimmedCurve sc) => MakeTrimmedCurve2d(sc, new Factors());

    //=============================================================================
    // Creation d' un VectorWithMagnitude de Geom a partir d' un Vector de Step
    //=============================================================================

    /// <summary>Converts a vector (its magnitude a length).</summary>
    public static VectorWithMagnitude? MakeVectorWithMagnitude(StepVector sv, in Factors localFactors)
    {
        // sln 22.10.2001. CTS23496: Vector is not created if direction have not been successfully created
        Direction? d = MakeDirection(sv.Orientation());
        if (d is not null)
        {
            Vector3D v = new Vector3D(d.Dir().XYZ() * sv.Magnitude() * localFactors.LengthFactor());
            return new VectorWithMagnitude(v);
        }
        return null;
    }

    /// <summary>Converts a vector (default factors).</summary>
    public static VectorWithMagnitude? MakeVectorWithMagnitude(StepVector sv) => MakeVectorWithMagnitude(sv, new Factors());

    //=============================================================================
    // Creation d' un VectorWithMagnitude de Geom2d a partir d' un Vector de Step
    //=============================================================================

    /// <summary>Converts a vector into a 2D vector.</summary>
    public static VectorWithMagnitude2? MakeVectorWithMagnitude2d(StepVector sv)
    {
        // sln 23.10.2001. CTS23496: Vector is not created if direction have not been successfully created
        // (MakeVectorWithMagnitude2d(...) function)
        Direction2? d = MakeDirection2d(sv.Orientation());
        if (d is not null)
        {
            Vector2D v = new Vector2D(d.Dir2d().XY() * sv.Magnitude());
            return new VectorWithMagnitude2(v);
        }
        return null;
    }

    /// <summary>
    /// Converts a spatial rotation (a ypr_rotation, or a rotation_about_direction in the plane angle unit of the
    /// context) into yaw, pitch and roll angles; null when it can't.
    /// </summary>
    /// <remarks>OCCT.NET: ported by unit 8F, with STEPConstruct_UnitContext (decision 1238).</remarks>
    public static NCollection_HArray1<double>? MakeYprRotation(SpatialRotation sr, GlobalUnitAssignedContext? cntxt)
    {
        // If rotation is already a ypr_rotation, return it immediately
        NCollection_HArray1<double>? yprRotation = null;
        if (sr.YprRotation() is not null && sr.YprRotation().Length() == 3)
        {
            return sr.YprRotation();
        }

        if (sr.RotationAboutDirection() is null
            || sr.RotationAboutDirection().DirectionOfAxis().DirectionRatioCount != 3
            || cntxt is null)
        {
            return null;
        }
        // rotation is a rotation_about_direction
        Direction axis = new Direction(sr.RotationAboutDirection().DirectionOfAxis().DirectionRatiosValue(1),
                                                   sr.RotationAboutDirection().DirectionOfAxis().DirectionRatiosValue(2),
                                                   sr.RotationAboutDirection().DirectionOfAxis().DirectionRatiosValue(3));
        double angle = sr.RotationAboutDirection().RotationAngle();
        if (Math.Abs(angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            // a zero rotation is converted trivially
            yprRotation = new NCollection_HArray1<double>(1, 3);
            yprRotation.SetValue(1, 0.0);
            yprRotation.SetValue(2, 0.0);
            yprRotation.SetValue(3, 0.0);
            return yprRotation;
        }
        double dx = axis.X();
        double dy = axis.Y();
        double dz = axis.Z();
        NCollection_Sequence<NamedUnit> paUnits = new NCollection_Sequence<NamedUnit>();
        for (int ind = 1; ind <= cntxt.Units().Length(); ++ind)
        {
            if (cntxt.UnitsValue(ind).IsKind(typeof(ConversionBasedUnitAndPlaneAngleUnit))
                || cntxt.UnitsValue(ind).IsKind(typeof(SiUnitAndPlaneAngleUnit)))
            {
                paUnits.Append(cntxt.UnitsValue(ind));
            }
        }
        if (paUnits.Length() != 1)
        {
            return yprRotation;
        }
        NamedUnit? pau = paUnits.Value(1);
        while (pau is not null && pau.IsKind(typeof(ConversionBasedUnitAndPlaneAngleUnit)))
        {
            ConversionBasedUnitAndPlaneAngleUnit converUnit = (ConversionBasedUnitAndPlaneAngleUnit)pau;
            MeasureWithUnit? conversionFactor = null;
            Transient transientFactor = converUnit.ConversionFactor();
            if (transientFactor.IsKind(typeof(MeasureWithUnit)))
            {
                conversionFactor = transientFactor as MeasureWithUnit;
            }
            else if (transientFactor.IsKind(typeof(ReprItemAndMeasureWithUnit)))
            {
                ReprItemAndMeasureWithUnit reprMeasureItem = (ReprItemAndMeasureWithUnit)transientFactor;
                conversionFactor = reprMeasureItem.GetMeasureWithUnit();
            }
            angle = angle * conversionFactor!.ValueComponent();
            pau = conversionFactor.UnitComponent().NamedUnit();
        }
        if (pau is null)
        {
            return yprRotation;
        }
        SiUnitAndPlaneAngleUnit? siUnit = pau as SiUnitAndPlaneAngleUnit;
        if (siUnit is null || siUnit.Name() != SiUnitName.SunRadian)
        {
            return yprRotation;
        }
        angle = (!siUnit.HasPrefix ? 1.0 : UnitContext.ConvertSiPrefix(siUnit.Prefix())) * angle;
        double ucf = sr.RotationAboutDirection().RotationAngle() / angle;
        double sa = Math.Sin(angle);
        double ca = Math.Cos(angle);
        double yaw = 0, pitch = 0, roll = 0;

        // axis parallel either to x-axis or to z-axis?
        if (Math.Abs(dy) < global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(dx * dz) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            while (angle <= -Math.PI)
            {
                angle = angle + 2 * Math.PI;
            }
            while (angle > Math.PI)
            {
                angle = angle - 2 * Math.PI;
            }

            yaw = ucf * angle;
            if (Math.Abs(angle - Math.PI) >= global::Kerf.Engine.Foundation.Precision.Angular())
            {
                roll = -yaw;
            }
            else
            {
                roll = yaw;
            }
            yprRotation = new NCollection_HArray1<double>(1, 3);
            yprRotation.SetValue(1, 0.0);
            yprRotation.SetValue(2, 0.0);
            yprRotation.SetValue(3, 0.0);
            if (Math.Abs(dx) >= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (dx > 0.0)
                {
                    yprRotation.SetValue(3, yaw);
                }
                else
                {
                    yprRotation.SetValue(3, roll);
                }
            }
            else
            {
                if (dz > 0.0)
                {
                    yprRotation.SetValue(1, yaw);
                }
                else
                {
                    yprRotation.SetValue(1, roll);
                }
            }
            return yprRotation;
        }

        // axis parallel to y-axis - use y-axis as pitch axis
        if (Math.Abs(dy) >= global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(dx) < global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(dz) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            if (ca >= 0.0)
            {
                yaw = 0.0;
                roll = 0.0;
            }
            else
            {
                yaw = ucf * Math.PI;
                roll = yaw;
            }
            pitch = ucf * Math.Atan2(sa, Math.Abs(ca));
            if (dy < 0.0)
            {
                pitch = -pitch;
            }
            yprRotation = new NCollection_HArray1<double>(1, 3);
            yprRotation.SetValue(1, yaw);
            yprRotation.SetValue(2, pitch);
            yprRotation.SetValue(3, roll);
            return yprRotation;
        }
        // axis not parallel to any axis of coordinate system
        // compute rotation matrix
        double cm1 = 1 - ca;

        double[,] rotMat = {
            { dx * dx * cm1 + ca, dx * dy * cm1 - dz * sa, dx * dz * cm1 + dy * sa },
            { dx * dy * cm1 + dz * sa, dy * dy * cm1 + ca, dy * dz * cm1 - dx * sa },
            { dx * dz * cm1 - dy * sa, dy * dz * cm1 + dx * sa, dz * dz * cm1 + ca } };

        // aRotMat[1][3] equals SIN(pitch_angle)
        if (Math.Abs(Math.Abs(rotMat[0, 2] - 1.0)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // |aPitch| = PI/2
            if (Math.Abs(rotMat[0, 2] - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                pitch = Math.PI / 2;
            }
            else
            {
                pitch = -Math.PI / 2;
            }
            // In this case, only the sum or difference of roll and yaw angles
            // is relevant and can be evaluated from the matrix.
            // According to IP "rectangular pitch angle" for ypr_rotation,
            // the roll angle is set to zero.
            roll = 0.0;
            yaw = Math.Atan2(rotMat[1, 0], rotMat[1, 1]);
            // result of ATAN is in the range[-PI / 2, PI / 2].
            // Here all four quadrants are needed.

            if (rotMat[1, 1] < 0.0)
            {
                if (yaw <= 0.0)
                {
                    yaw = yaw + Math.PI;
                }
                else
                {
                    yaw = yaw - Math.PI;
                }
            }
        }
        else
        {
            // COS (pitch_angle) not equal to zero
            yaw = Math.Atan2(-rotMat[0, 1], rotMat[0, 0]);

            if (rotMat[0, 0] < 0.0)
            {
                if (yaw < 0.0 || Math.Abs(yaw) < global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    yaw = yaw + Math.PI;
                }
                else
                {
                    yaw = yaw - Math.PI;
                }
            }
            double sy = Math.Sin(yaw);
            double cy = Math.Cos(yaw);
            double aSR = Math.Sin(roll);
            double cr = Math.Cos(roll);

            if (Math.Abs(sy) > Math.Abs(cy) && Math.Abs(sy) > Math.Abs(aSR) && Math.Abs(sy) > Math.Abs(cr))
            {
                cm1 = -rotMat[0, 1] / sy;
            }
            else
            {
                if (Math.Abs(cy) > Math.Abs(aSR) && Math.Abs(cy) > Math.Abs(cr))
                {
                    cm1 = rotMat[0, 0] / cy;
                }
                else if (Math.Abs(aSR) > Math.Abs(cr))
                {
                    cm1 = -rotMat[1, 2] / aSR;
                }
                else
                {
                    cm1 = rotMat[2, 2] / cr;
                }
            }
            pitch = Math.Atan2(rotMat[0, 2], cm1);
        }
        yaw = yaw * ucf;
        pitch = pitch * ucf;
        roll = roll * ucf;
        yprRotation = new NCollection_HArray1<double>(1, 3);
        yprRotation.SetValue(1, yaw);
        yprRotation.SetValue(2, pitch);
        yprRotation.SetValue(3, roll);

        return yprRotation;
    }
}
