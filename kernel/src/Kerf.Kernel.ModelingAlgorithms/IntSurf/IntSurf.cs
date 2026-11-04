// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>Resources for the packages of surface/surface intersections.</summary>
internal static class IntSurf
{
    /// <summary>
    /// The transitions at the intersection point of two lines: <paramref name="tgFirst"/> and
    /// <paramref name="tgSecond"/> are their tangents, <paramref name="normale"/> orients TgFirst ^ TgSecond (OCCT:
    /// the intersection line, the restriction and the surface normal).
    /// </summary>
    /// <remarks>
    /// The transitions are <c>ref</c>, as OCCT's references: an in or out transition keeps the situation and
    /// opposite flag the caller's object had.
    /// </remarks>
    public static void MakeTransition(in Vector3D tgFirst, in Vector3D tgSecond, in Direction3D normale,
                                      ref WalkingTransition tFirst, ref WalkingTransition tSecond)
    {
        // Compute the mixed product of normal, tangent 1, tangent 2
        // to get the type of transition.

        Vector3D pvect = tgSecond.Crossed(tgFirst);

        double nTgSecond = tgSecond.Magnitude;
        double nTgFirst = tgFirst.Magnitude;
        double nTgSecondNTgFirstAngular = nTgSecond * nTgFirst * global::Kerf.Engine.Foundation.Precision.Angular();

        if (nTgFirst <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            tFirst.SetValue(true, WalkingTypeTrans.Undecided);
            tSecond.SetValue(true, WalkingTypeTrans.Undecided);
        }
        else if ((nTgSecond <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (pvect.Magnitude <= nTgSecondNTgFirstAngular))
        {
            tFirst.SetValue(true, WalkingSituation.Unknown, tgFirst.Dot(tgSecond) < 0.0);
            tSecond.SetValue(true, WalkingSituation.Unknown, tgFirst.Dot(tgSecond) < 0.0);
        }
        else
        {
            double yu = pvect.Dot(new Vector3D(normale));
            yu /= nTgSecond * nTgFirst;
            if (yu > 0.0001)
            {
                tFirst.SetValue(false, WalkingTypeTrans.In);
                tSecond.SetValue(false, WalkingTypeTrans.Out);
            }
            else if (yu < -0.0001)
            {
                tFirst.SetValue(false, WalkingTypeTrans.Out);
                tSecond.SetValue(false, WalkingTypeTrans.In);
            }
            else
            {
                tFirst.SetValue(true, WalkingTypeTrans.Undecided);
                tSecond.SetValue(true, WalkingTypeTrans.Undecided);
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="arrOfPeriod"/> (four values) with the U and V periods of the first surface, then of the
    /// second; 0 in a direction that is not periodic.
    /// </summary>
    public static void SetPeriod(SurfaceAdaptorBase firstSurf, SurfaceAdaptorBase secondSurf,
                                 Span<double> arrOfPeriod)
    {
        arrOfPeriod[0] = firstSurf.IsUPeriodic ? firstSurf.UPeriod() : 0.0;
        arrOfPeriod[1] = firstSurf.IsVPeriodic ? firstSurf.VPeriod() : 0.0;
        arrOfPeriod[2] = secondSurf.IsUPeriodic ? secondSurf.UPeriod() : 0.0;
        arrOfPeriod[3] = secondSurf.IsVPeriodic ? secondSurf.VPeriod() : 0.0;
    }
}
