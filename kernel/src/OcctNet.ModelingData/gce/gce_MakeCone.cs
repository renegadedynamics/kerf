// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCone.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Cone: from an axis system, a semi-angle and a reference radius, coaxial to a cone through a
/// point or at a signed offset, from four points, from an axis and two points, or from two axis points and two
/// section radii.
/// </summary>
public sealed class gce_MakeCone : gce_Root
{
    private Cone TheCone = new();

    /// <summary>
    /// A cone from an axis system, a semi-angle Ang (radians, in ]0, PI/2[) and a reference radius. Fails with
    /// gce_NegativeRadius if Radius is negative and gce_BadAngle if Ang is out of range.
    /// </summary>
    public gce_MakeCone(in Frame3D A2, double Ang, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            if (Ang <= Constants.Resolution() || M_PI / 2 - Ang <= Constants.Resolution())
            {
                TheError = gce_BadAngle;
            }
            else
            {
                TheError = gce_Done;
                TheCone = new Cone(new CoordinateSystem(A2), Ang, Radius);
            }
        }
    }

    /// <summary>
    /// A cone from four points: P1 and P2 define the axis, the distance from P3 to it the base radius, and the
    /// distance from P4 to it the radius of the section through P4.
    /// </summary>
    public gce_MakeCone(in Point3D P1, in Point3D P2, in Point3D P3, in Point3D P4)
    {
        // P1 and P2 define the cone axis. The distance from P3 to the axis gives
        // the base radius, and the distance from P4 to the axis gives the radius
        // of the section passing through P4.
        if (P1.Distance(P2) < Constants.Resolution() || P3.Distance(P4) < Constants.Resolution())
        {
            TheError = gce_ConfusedPoints;
            return;
        }
        Direction3D D1 = new(P2.XYZ() - P1.XYZ());
        double cos = D1.Dot(new Direction3D(P4.XYZ() - P1.XYZ()));
        double dist = P1.Distance(P4);
        Point3D PP4 = new(P1.XYZ() + cos * dist * D1.XYZ());
        cos = D1.Dot(new Direction3D(P3.XYZ() - P1.XYZ()));
        dist = P1.Distance(P3);
        Point3D PP3 = new(P1.XYZ() + cos * dist * D1.XYZ());
        double Dist13 = PP3.Distance(P1);
        double Dist14 = PP4.Distance(P1);
        if (Math.Abs(Dist13 - Dist14) < Constants.Resolution())
        {
            TheError = gce_NullAngle;
            return;
        }
        Line3D L1 = new(P1, D1);
        double Dist3 = L1.Distance(P3);
        double Dist4 = L1.Distance(P4);
        double DifRad = Dist3 - Dist4;
        double angle = Math.Abs(Math.Atan(DifRad / (Dist13 - Dist14)));
        if (Math.Abs(M_PI / 2.0 - angle) < Constants.Resolution() || Math.Abs(angle) < Constants.Resolution())
        {
            TheError = gce_NullRadius;
            return;
        }
        double R1 = PP3.Distance(P3);
        double R2 = PP4.Distance(P4);
        if (R1 < 0.0 || R2 < 0.0)
        {
            TheError = gce_NegativeRadius;
            return;
        }
        Direction3D DD1 = new(PP4.XYZ() - PP3.XYZ());
        Direction3D D2 = new();
        double x = DD1.X();
        double y = DD1.Y();
        double z = DD1.Z();
        if (Math.Abs(x) > Constants.Resolution())
        {
            D2 = new Direction3D(-y, x, 0.0);
        }
        else if (Math.Abs(y) > Constants.Resolution())
        {
            D2 = new Direction3D(0.0, -z, y);
        }
        else if (Math.Abs(z) > Constants.Resolution())
        {
            D2 = new Direction3D(z, 0.0, -x);
        }
        if (R1 > R2)
        {
            angle *= -1;
        }
        TheCone = new Cone(new CoordinateSystem(new Frame3D(PP3, DD1, D2)), angle, R1);
        TheError = gce_Done;
    }

    /// <summary>
    /// A cone from its axis and two points: the distance from P1 to the axis gives the base radius, from P2 the radius
    /// of the section through P2. The status is that of the four-point construction.
    /// </summary>
    public gce_MakeCone(in Axis3D Axis, in Point3D P1, in Point3D P2)
    {
        // The distance from P1 to the axis gives the base radius, and
        // the distance from P2 to the axis gives the section radius at P2.
        Point3D P3 = Axis.Location();
        Point3D P4 = new(P3.XYZ() + Axis.Direction().XYZ());
        gce_MakeCone Cone = new(P3, P4, P1, P2);
        if (Cone.IsDone())
        {
            TheCone = Cone.Value();
            TheError = gce_Done;
        }
        else
        {
            TheError = Cone.Status();
        }
    }

    /// <summary>
    /// A cone coaxial to Cone through the point P (the nearest such cone when there are two); fails with
    /// gce_NegativeRadius when no non-negative radius exists.
    /// </summary>
    public gce_MakeCone(in Cone Cone, in Point3D P)
    {
        Xyz aVecToPoint = P.XYZ() - Cone.Location().XYZ();
        double aVParam = aVecToPoint.Dot(Cone.Axis().Direction().XYZ());
        double aRadius = new Line3D(Cone.Axis()).Distance(P);
        double aTan = Math.Tan(Cone.SemiAngle());
        double aCos = Math.Cos(Cone.SemiAngle());
        double aCandidate1 = aRadius - aVParam * aTan;
        double aCandidate2 = -aRadius - aVParam * aTan;
        // Accept tiny negative values as zero (rounding noise from analytic formulas),
        // then clamp to 0.0 before constructing gp_Cone.
        static bool isNonNegativeWithTol(double theValue) => theValue >= -Constants.Resolution();
        static double clampToZero(double theValue) => theValue < 0.0 ? 0.0 : theValue;
        double aResultRadius = -1.0;
        if (isNonNegativeWithTol(aCandidate1) && isNonNegativeWithTol(aCandidate2))
        {
            // Keep the nearest parallel cone to preserve predictable behavior.
            double aCand1 = clampToZero(aCandidate1);
            double aCand2 = clampToZero(aCandidate2);
            double aDist1 = Math.Abs((aCand1 - Cone.RefRadius()) * aCos);
            double aDist2 = Math.Abs((aCand2 - Cone.RefRadius()) * aCos);
            aResultRadius = (aDist1 <= aDist2) ? aCand1 : aCand2;
        }
        else if (isNonNegativeWithTol(aCandidate1))
        {
            aResultRadius = clampToZero(aCandidate1);
        }
        else if (isNonNegativeWithTol(aCandidate2))
        {
            aResultRadius = clampToZero(aCandidate2);
        }
        if (aResultRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
            return;
        }
        TheCone = new Cone(Cone.Position(), Cone.SemiAngle(), aResultRadius);
        TheError = gce_Done;
    }

    /// <summary>
    /// A cone coaxial to Cone at the signed distance Dist; fails with gce_NullAngle if the semi-angle's cosine is
    /// too small and gce_NegativeRadius if the radius would be negative.
    /// </summary>
    public gce_MakeCone(in Cone Cone, double Dist)
    {
        // gp_Cone keeps |SemiAngle| in ]gp::Resolution(), PI/2 - gp::Resolution()[,
        // thus aCos is expected to be positive and not too small.
        double aCos = Math.Cos(Cone.SemiAngle());
        if (Math.Abs(aCos) <= Constants.Resolution())
        {
            TheError = gce_NullAngle;
            return;
        }
        double aRadius = Cone.RefRadius() + Dist / aCos;
        if (aRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
            return;
        }
        TheCone = new Cone(Cone.Position(), Cone.SemiAngle(), aRadius);
        TheError = gce_Done;
    }

    /// <summary>
    /// A cone from the axis of a line and two points (as the gp_Ax1 overload). The status is that of the four-point
    /// construction.
    /// </summary>
    public gce_MakeCone(in Line3D Axis, in Point3D P1, in Point3D P2)
    {
        Point3D P3 = Axis.Location();
        Point3D P4 = new(P3.XYZ() + Axis.Direction().XYZ());
        gce_MakeCone Cone = new(P3, P4, P1, P2);
        if (Cone.IsDone())
        {
            TheCone = Cone.Value();
            TheError = gce_Done;
        }
        else
        {
            TheError = Cone.Status();
        }
    }

    /// <summary>
    /// A cone whose axis passes through P1 and P2, with section radius R1 at P1 and R2 at P2. Fails with gce_NullAxis
    /// if the points coincide, gce_NegativeRadius if a radius is negative and gce_NullAngle for a zero or right angle.
    /// </summary>
    public gce_MakeCone(in Point3D P1, in Point3D P2, double R1, double R2)
    {
        // Cone defined by two points (axis) and two radii (section radii at each point).
        double dist = P1.Distance(P2);
        if (dist < Constants.Resolution())
        {
            TheError = gce_NullAxis;
        }
        else
        {
            if (R1 < 0.0 || R2 < 0.0)
            {
                TheError = gce_NegativeRadius;
            }
            else
            {
                double Angle = Math.Abs(Math.Atan((R1 - R2) / dist));
                if (Math.Abs(M_PI / 2.0 - Angle) < Constants.Resolution() || Math.Abs(Angle) < Constants.Resolution())
                {
                    TheError = gce_NullAngle;
                }
                else
                {
                    Direction3D D1 = new(P2.XYZ() - P1.XYZ());
                    Direction3D D2 = new();
                    double x = D1.X();
                    double y = D1.Y();
                    double z = D1.Z();
                    if (Math.Abs(x) > Constants.Resolution())
                    {
                        D2 = new Direction3D(-y, x, 0.0);
                    }
                    else if (Math.Abs(y) > Constants.Resolution())
                    {
                        D2 = new Direction3D(0.0, -z, y);
                    }
                    else if (Math.Abs(z) > Constants.Resolution())
                    {
                        D2 = new Direction3D(z, 0.0, -x);
                    }
                    if (R1 > R2)
                    {
                        Angle *= -1;
                    }
                    TheCone = new Cone(new CoordinateSystem(new Frame3D(P1, D1, D2)), Angle, R1);
                    TheError = gce_Done;
                }
            }
        }
    }

    /// <summary>The constructed cone.</summary>
    /// <exception cref="NotDoneException">No cone was constructed.</exception>
    public Cone Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeCone::Value() - no result");
        return TheCone;
    }

    /// <summary>Alias for Value().</summary>
    public Cone Operator() => Value();

    /// <summary>The constructed cone (OCCT's implicit conversion operator).</summary>
    public static explicit operator Cone(gce_MakeCone theMaker) => theMaker.Operator();
}
