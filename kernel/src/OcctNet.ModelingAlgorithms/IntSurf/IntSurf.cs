// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>Resources for the packages of surface/surface intersections.</summary>
public static class IntSurf
{
    /// <summary>
    /// The transitions at the intersection point of two lines: <paramref name="TgFirst"/> and
    /// <paramref name="TgSecond"/> are their tangents, <paramref name="Normale"/> orients TgFirst ^ TgSecond (OCCT:
    /// the intersection line, the restriction and the surface normal).
    /// </summary>
    /// <remarks>
    /// The transitions are <c>ref</c>, as OCCT's references: an in or out transition keeps the situation and
    /// opposite flag the caller's object had.
    /// </remarks>
    public static void MakeTransition(in Vector3D TgFirst, in Vector3D TgSecond, in Direction3D Normale,
                                      ref IntSurf_Transition TFirst, ref IntSurf_Transition TSecond)
    {
        // Compute the mixed product of normal, tangent 1, tangent 2
        // to get the type of transition.

        Vector3D pvect = TgSecond.Crossed(TgFirst);

        double NTgSecond = TgSecond.Magnitude();
        double NTgFirst = TgFirst.Magnitude();
        double NTgSecondNTgFirstAngular = NTgSecond * NTgFirst * global::Kerf.Engine.Foundation.Precision.Angular();

        if (NTgFirst <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            TFirst.SetValue(true, IntSurf_Undecided);
            TSecond.SetValue(true, IntSurf_Undecided);
        }
        else if ((NTgSecond <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (pvect.Magnitude() <= NTgSecondNTgFirstAngular))
        {
            TFirst.SetValue(true, IntSurf_Unknown, TgFirst.Dot(TgSecond) < 0.0);
            TSecond.SetValue(true, IntSurf_Unknown, TgFirst.Dot(TgSecond) < 0.0);
        }
        else
        {
            double yu = pvect.Dot(new Vector3D(Normale));
            yu /= NTgSecond * NTgFirst;
            if (yu > 0.0001)
            {
                TFirst.SetValue(false, IntSurf_In);
                TSecond.SetValue(false, IntSurf_Out);
            }
            else if (yu < -0.0001)
            {
                TFirst.SetValue(false, IntSurf_Out);
                TSecond.SetValue(false, IntSurf_In);
            }
            else
            {
                TFirst.SetValue(true, IntSurf_Undecided);
                TSecond.SetValue(true, IntSurf_Undecided);
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="theArrOfPeriod"/> (four values) with the U and V periods of the first surface, then of the
    /// second; 0 in a direction that is not periodic.
    /// </summary>
    public static void SetPeriod(SurfaceAdaptorBase theFirstSurf, SurfaceAdaptorBase theSecondSurf,
                                 Span<double> theArrOfPeriod)
    {
        theArrOfPeriod[0] = theFirstSurf.IsUPeriodic ? theFirstSurf.UPeriod() : 0.0;
        theArrOfPeriod[1] = theFirstSurf.IsVPeriodic ? theFirstSurf.VPeriod() : 0.0;
        theArrOfPeriod[2] = theSecondSurf.IsUPeriodic ? theSecondSurf.UPeriod() : 0.0;
        theArrOfPeriod[3] = theSecondSurf.IsVPeriodic ? theSecondSurf.VPeriod() : 0.0;
    }
}
