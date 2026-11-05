// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.GeomOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Geom_Plane, the cylindrical, conical, spherical and toroidal surfaces, Geom_SurfaceOfRevolution,
/// Geom_SurfaceOfLinearExtrusion and Geom_RectangularTrimmedSurface against native OCCT: evaluation (D0 to D3 and
/// DN), bounds, closure, periodicity, continuity, reversed parameters, U and V reversal, transformation (with
/// TransformParameters and ParametricTransformation), the geometric queries of each class, trimming, and the
/// iso-curves (GeomIsoCurveCompare). The swept surfaces revolve or extrude circles and non-periodic B-spline
/// curves.
/// </summary>
public class Geom_SurfacesOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(300, 5);

    // ---------------------------------------------------------------------------------------------------------
    // Comparison helpers

    private static void SameDir(Oracle theOracle, N.gp_Dir theN, in Direction3D theP, string theWhat)
    {
        theOracle.Same(theN.X(), theP.X, theWhat + " X");
        theOracle.Same(theN.Y(), theP.Y, theWhat + " Y");
        theOracle.Same(theN.Z(), theP.Z, theWhat + " Z");
    }

    private static void SameAx3(Oracle theOracle, N.gp_Ax3 theN, in CoordinateSystem theP, string theWhat)
    {
        using N.gp_Pnt aLoc = theN.Location();
        using N.gp_Dir aDir = theN.Direction();
        using N.gp_Dir aXDir = theN.XDirection();
        using N.gp_Dir aYDir = theN.YDirection();
        theOracle.Same(aLoc, theP.Location, theWhat + " location");
        SameDir(theOracle, aDir, theP.Direction, theWhat + " direction");
        SameDir(theOracle, aXDir, theP.XDirection, theWhat + " X direction");
        SameDir(theOracle, aYDir, theP.YDirection, theWhat + " Y direction");
    }

    /// <summary>D0 to D3, and DN for every order with Nu + Nv from 1 to theMaxN, at (U, V).</summary>
    private static void CompareAt(Oracle theOracle, N.Geom_Surface theN, Surface theP, double U, double V,
                                  string theWhat, int theMaxN = 4, double theTol = Oracle.Tolerance)
    {
        using (N.gp_Pnt aNP = theN.EvalD0(U, V))
        {
            theOracle.Same(aNP, theP.Value(U, V), theWhat + " D0", theTol);
        }

        using (var aNP = new N.gp_Pnt(0, 0, 0))
        using (var aNU = new N.gp_Vec(0, 0, 0))
        using (var aNV = new N.gp_Vec(0, 0, 0))
        {
            theN.D1(U, V, aNP, aNU, aNV);
            theP.D1(U, V, out Point3D P, out Vector3D D1U, out Vector3D D1V);
            theOracle.Same(aNP, P, theWhat + " D1 P", theTol);
            theOracle.Same(aNU, D1U, theWhat + " D1U", theTol);
            theOracle.Same(aNV, D1V, theWhat + " D1V", theTol);
        }

        using (var aNP = new N.gp_Pnt(0, 0, 0))
        using (var aNU = new N.gp_Vec(0, 0, 0))
        using (var aNV = new N.gp_Vec(0, 0, 0))
        using (var aNUU = new N.gp_Vec(0, 0, 0))
        using (var aNVV = new N.gp_Vec(0, 0, 0))
        using (var aNUV = new N.gp_Vec(0, 0, 0))
        {
            theN.D2(U, V, aNP, aNU, aNV, aNUU, aNVV, aNUV);
            theP.D2(U, V, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV);
            theOracle.Same(aNP, P, theWhat + " D2 P", theTol);
            theOracle.Same(aNU, D1U, theWhat + " D2 D1U", theTol);
            theOracle.Same(aNV, D1V, theWhat + " D2 D1V", theTol);
            theOracle.Same(aNUU, D2U, theWhat + " D2U", theTol);
            theOracle.Same(aNVV, D2V, theWhat + " D2V", theTol);
            theOracle.Same(aNUV, D2UV, theWhat + " D2UV", theTol);
        }

        using (var aNP = new N.gp_Pnt(0, 0, 0))
        using (var aNU = new N.gp_Vec(0, 0, 0))
        using (var aNV = new N.gp_Vec(0, 0, 0))
        using (var aNUU = new N.gp_Vec(0, 0, 0))
        using (var aNVV = new N.gp_Vec(0, 0, 0))
        using (var aNUV = new N.gp_Vec(0, 0, 0))
        using (var aNUUU = new N.gp_Vec(0, 0, 0))
        using (var aNVVV = new N.gp_Vec(0, 0, 0))
        using (var aNUUV = new N.gp_Vec(0, 0, 0))
        using (var aNUVV = new N.gp_Vec(0, 0, 0))
        {
            theN.D3(U, V, aNP, aNU, aNV, aNUU, aNVV, aNUV, aNUUU, aNVVV, aNUUV, aNUVV);
            theP.D3(U, V, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV,
                    out Vector3D D3U, out Vector3D D3V, out Vector3D D3UUV, out Vector3D D3UVV);
            theOracle.Same(aNP, P, theWhat + " D3 P", theTol);
            theOracle.Same(aNU, D1U, theWhat + " D3 D1U", theTol);
            theOracle.Same(aNV, D1V, theWhat + " D3 D1V", theTol);
            theOracle.Same(aNUU, D2U, theWhat + " D3 D2U", theTol);
            theOracle.Same(aNVV, D2V, theWhat + " D3 D2V", theTol);
            theOracle.Same(aNUV, D2UV, theWhat + " D3 D2UV", theTol);
            theOracle.Same(aNUUU, D3U, theWhat + " D3U", theTol);
            theOracle.Same(aNVVV, D3V, theWhat + " D3V", theTol);
            theOracle.Same(aNUUV, D3UUV, theWhat + " D3UUV", theTol);
            theOracle.Same(aNUVV, D3UVV, theWhat + " D3UVV", theTol);
        }

        for (int aN = 1; aN <= theMaxN; aN++)
        {
            for (int aNu = 0; aNu <= aN; aNu++)
            {
                using N.gp_Vec aNDN = theN.DN(U, V, aNu, aN - aNu);
                Vector3D aPDN = theP.DN(U, V, aNu, aN - aNu);
                if (theP is ToroidalSurface && TorusDNReversedByOcct(aNu, aN - aNu))
                {
                    aPDN = aPDN.Reversed();
                }
                theOracle.Same(aNDN, aPDN, $"{theWhat} DN({aNu}, {aN - aNu})", theTol);
            }
        }
    }

    // Plan 3.2's third level (occt-issues.md, ElSLib::TorusDN; Kerf decision 1486): OCCT's mixed torus derivatives
    // of orders (Nu + Nv) = 0 mod 4 (both odd) and 2 mod 4 (both even) have the wrong sign, which the port's don't:
    // native's are compared with the port's reversed.
    private static bool TorusDNReversedByOcct(int theNu, int theNv)
        => theNu > 0 && theNv > 0
           && ((theNu % 2 == 1 && theNv % 2 == 1 && (theNu + theNv) % 4 == 0) || (theNu % 2 == 0 && theNv % 2 == 0 && (theNu + theNv) % 4 == 2));

    /// <summary>Bounds, closure, periodicity (with periods), continuity and IsCN.</summary>
    private static void CompareProperties(Oracle theOracle, N.Geom_Surface theN, Surface theP, string theWhat,
                                          bool theContinuity = true)
    {
        double aNU1 = 0, aNU2 = 0, aNV1 = 0, aNV2 = 0;
        theN.Bounds(ref aNU1, ref aNU2, ref aNV1, ref aNV2);
        theP.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        theOracle.Same(aNU1, aU1, theWhat + " U1");
        theOracle.Same(aNU2, aU2, theWhat + " U2");
        theOracle.Same(aNV1, aV1, theWhat + " V1");
        theOracle.Same(aNV2, aV2, theWhat + " V2");
        theOracle.Same(theN.IsUClosed(), theP.IsUClosed, theWhat + " IsUClosed");
        theOracle.Same(theN.IsVClosed(), theP.IsVClosed, theWhat + " IsVClosed");
        theOracle.Same(theN.IsUPeriodic(), theP.IsUPeriodic, theWhat + " IsUPeriodic");
        theOracle.Same(theN.IsVPeriodic(), theP.IsVPeriodic, theWhat + " IsVPeriodic");
        if (theP.IsUPeriodic)
        {
            theOracle.Same(theN.UPeriod(), theP.UPeriod, theWhat + " UPeriod");
        }
        if (theP.IsVPeriodic)
        {
            theOracle.Same(theN.VPeriod(), theP.VPeriod, theWhat + " VPeriod");
        }
        if (theContinuity)
        {
            theOracle.Same((int)theN.Continuity(), (int)theP.Continuity, theWhat + " Continuity");
            for (int aN = 0; aN <= 3; aN++)
            {
                theOracle.Same(theN.IsCNu(aN), theP.IsCNu(aN), theWhat + " IsCNu");
                theOracle.Same(theN.IsCNv(aN), theP.IsCNv(aN), theWhat + " IsCNv");
            }
        }
    }

    /// <summary>A parameter in [theFirst, theLast], or around the origin where a bound is infinite.</summary>
    private static double Param(Oracle theOracle, double theFirst, double theLast)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            return theOracle.Real(-20, 20);
        }
        return theOracle.Real(theFirst, theLast);
    }

    private static void CompareEvaluation(Oracle theOracle, N.Geom_Surface theN, Surface theP, string theWhat,
                                          int theMaxN = 4, double theTol = Oracle.Tolerance, int theCount = 4)
    {
        theP.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        for (int k = 0; k < theCount; k++)
        {
            double U = Param(theOracle, aU1, aU2);
            double V = Param(theOracle, aV1, aV2);
            // Outside the first period too
            if (k == 0 && theP.IsUPeriodic)
            {
                U += 3 * theP.UPeriod;
            }
            if (k == 1 && theP.IsVPeriodic)
            {
                V -= 2 * theP.VPeriod;
            }
            CompareAt(theOracle, theN, theP, U, V, theWhat, theMaxN, theTol);
        }
    }

    /// <summary>TransformParameters and ParametricTransformation for T.</summary>
    private static void CompareParametricTransformation(Oracle theOracle, N.Geom_Surface theN, Surface theP,
                                                        N.gp_Trsf theNT, in Transform3D theT, string theWhat)
    {
        theP.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        double U = Param(theOracle, aU1, aU2), V = Param(theOracle, aV1, aV2);
        double aNU = U, aNV = V, aPU = U, aPV = V;
        theN.TransformParameters(ref aNU, ref aNV, theNT);
        theP.TransformParameters(ref aPU, ref aPV, theT);
        theOracle.Same(aNU, aPU, theWhat + " TransformParameters U");
        theOracle.Same(aNV, aPV, theWhat + " TransformParameters V");

        using N.gp_GTrsf2d aNG = theN.ParametricTransformation(theNT);
        GeneralTransform2D aPG = theP.ParametricTransformation(theT);
        for (int r = 1; r <= 2; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                theOracle.Same(aNG.Value(r, c), aPG.Value(r, c), $"{theWhat} ParametricTransformation({r}, {c})");
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Elementary surfaces

    private static (N.Geom_ElementarySurface Native, ElementarySurface Port) RandomElementary(Oracle theOracle, int theKind)
    {
        (N.gp_Ax3 aNAx3, CoordinateSystem anAx3) = Ax3s(theOracle);
        using var aNAx3Owner = aNAx3;
        switch (theKind)
        {
            case 0:
                return (new N.Geom_Plane(aNAx3), new PlaneSurface(anAx3));
            case 1:
            {
                double R = theOracle.Real(0.1, 20);
                return (new N.Geom_CylindricalSurface(aNAx3, R), new CylindricalSurface(anAx3, R));
            }
            case 2:
            {
                double anAng = theOracle.Real(0.05, 1.5) * (theOracle.Int(0, 1) == 0 ? 1 : -1);
                double R = theOracle.Int(0, 4) == 0 ? 0.0 : theOracle.Real(0.1, 20);
                return (new N.Geom_ConicalSurface(aNAx3, anAng, R), new ConicalSurface(anAx3, anAng, R));
            }
            case 3:
            {
                double R = theOracle.Real(0.1, 20);
                return (new N.Geom_SphericalSurface(aNAx3, R), new SphericalSurface(anAx3, R));
            }
            default:
            {
                double r = theOracle.Real(0.1, 5);
                double R = theOracle.Int(0, 4) == 0 ? theOracle.Real(0.0, r) : theOracle.Real(r, 20);
                return (new N.Geom_ToroidalSurface(aNAx3, R, r), new ToroidalSurface(anAx3, R, r));
            }
        }
    }

    private static void CompareElementaryQueries(Oracle theOracle, N.Geom_ElementarySurface theN, ElementarySurface theP,
                                                 string theWhat)
    {
        using (N.gp_Ax3 aNPos = theN.Position())
        {
            SameAx3(theOracle, aNPos, theP.Position, theWhat + " position");
        }

        double A1 = 0, A2 = 0, A3 = 0, B1 = 0, B2 = 0, B3 = 0, C1 = 0, C2 = 0, C3 = 0, D = 0;
        double a1, a2, a3, b1, b2, b3, c1, c2, c3, d;
        switch (theP)
        {
            case PlaneSurface aPlane:
                using (var aNPlane = N.Geom_Plane.DownCast(theN)!)
                {
                    aNPlane.Coefficients(ref A1, ref A2, ref A3, ref D);
                }
                aPlane.Coefficients(out a1, out a2, out a3, out d);
                theOracle.Same(A1, a1, theWhat + " A");
                theOracle.Same(A2, a2, theWhat + " B");
                theOracle.Same(A3, a3, theWhat + " C");
                theOracle.Same(D, d, theWhat + " D");
                return;
            case CylindricalSurface aCyl:
            {
                using var aNCyl = N.Geom_CylindricalSurface.DownCast(theN)!;
                aNCyl.Coefficients(ref A1, ref A2, ref A3, ref B1, ref B2, ref B3, ref C1, ref C2, ref C3, ref D);
                aCyl.Coefficients(out a1, out a2, out a3, out b1, out b2, out b3, out c1, out c2, out c3, out d);
                theOracle.Same(aNCyl.Radius(), aCyl.Radius, theWhat + " Radius");
                break;
            }
            case ConicalSurface aCone:
            {
                using var aNCone = N.Geom_ConicalSurface.DownCast(theN)!;
                aNCone.Coefficients(ref A1, ref A2, ref A3, ref B1, ref B2, ref B3, ref C1, ref C2, ref C3, ref D);
                aCone.Coefficients(out a1, out a2, out a3, out b1, out b2, out b3, out c1, out c2, out c3, out d);
                theOracle.Same(aNCone.SemiAngle(), aCone.SemiAngle, theWhat + " SemiAngle");
                theOracle.Same(aNCone.RefRadius(), aCone.RefRadius, theWhat + " RefRadius");
                if (Math.Abs(aCone.SemiAngle) > 1e-3)
                {
                    using N.gp_Pnt anApex = aNCone.Apex();
                    theOracle.Same(anApex, aCone.Apex, theWhat + " Apex");
                }
                // Plan 3.2's third level (occt-issues.md, Geom_ConicalSurface::Coefficients; Kerf decision 1494): OCCT's
                // linear and constant terms were wrong away from the origin or with a radius; the port's are gp_Cone's
                aCone.Cone.Coefficients(out _, out _, out _, out _, out _, out _, out double aGC1, out double aGC2, out double aGC3,
                                          out double aGD);
                (C1, C2, C3, D) = (aGC1, aGC2, aGC3, aGD);
                break;
            }
            case SphericalSurface aSph:
            {
                using var aNSph = N.Geom_SphericalSurface.DownCast(theN)!;
                aNSph.Coefficients(ref A1, ref A2, ref A3, ref B1, ref B2, ref B3, ref C1, ref C2, ref C3, ref D);
                aSph.Coefficients(out a1, out a2, out a3, out b1, out b2, out b3, out c1, out c2, out c3, out d);
                theOracle.Same(aNSph.Area(), aSph.Area, theWhat + " Area");
                theOracle.Same(aNSph.Volume(), aSph.Volume, theWhat + " Volume");
                break;
            }
            default:
            {
                using var aNTor = N.Geom_ToroidalSurface.DownCast(theN)!;
                var aTor = (ToroidalSurface)theP;
                theOracle.Same(aNTor.Area(), aTor.Area, theWhat + " Area");
                theOracle.Same(aNTor.Volume(), aTor.Volume, theWhat + " Volume");
                theOracle.Same(aNTor.MajorRadius(), aTor.MajorRadius, theWhat + " MajorRadius");
                theOracle.Same(aNTor.MinorRadius(), aTor.MinorRadius, theWhat + " MinorRadius");
                using var aNCoef = new N.TColStd_Array1OfReal(1, 35);
                aNTor.Coefficients(aNCoef);
                var aCoef = new NCollection_Array1<double>(1, 35);
                aTor.Coefficients(aCoef);
                for (int i = 1; i <= 35; i++)
                {
                    // The quartic's coefficients reach R^4 (1e5): relative to the largest one
                    theOracle.Same(aNCoef.Value(i) / 1e5, aCoef[i] / 1e5, theWhat + " torus coefficient");
                }
                return;
            }
        }

        theOracle.Same(A1, a1, theWhat + " A1");
        theOracle.Same(A2, a2, theWhat + " A2");
        theOracle.Same(A3, a3, theWhat + " A3");
        theOracle.Same(B1, b1, theWhat + " B1");
        theOracle.Same(B2, b2, theWhat + " B2");
        theOracle.Same(B3, b3, theWhat + " B3");
        theOracle.Same(C1, c1, theWhat + " C1");
        theOracle.Same(C2, c2, theWhat + " C2");
        theOracle.Same(C3, c3, theWhat + " C3");
        theOracle.Same(D, d, theWhat + " D");
    }

    [Fact]
    public void Elementary_surfaces()
    {
        string[] aNames = ["Geom_Plane", "Geom_CylindricalSurface", "Geom_ConicalSurface", "Geom_SphericalSurface", "Geom_ToroidalSurface"];
        for (int aKind = 0; aKind < 5; aKind++)
        {
        var anOracle = new Oracle(output, 20261029 + aKind);
        for (int i = 0; i < Cases / 5; i++)
        {
            (N.Geom_ElementarySurface aN, ElementarySurface aP) = RandomElementary(anOracle, aKind);
            string aWhat = global::Kerf.Engine.Foundation.OcctTypeNames.Of(aP.GetType());
            using (aN)
            {
                CompareProperties(anOracle, aN, aP, aWhat);
                CompareElementaryQueries(anOracle, aN, aP, aWhat);
                CompareEvaluation(anOracle, aN, aP, aWhat);
                double U = anOracle.Real(-10, 10), V = anOracle.Real(-10, 10);
                anOracle.Same(aN.UReversedParameter(U), aP.UReversedParameter(U), aWhat + " UReversedParameter");
                anOracle.Same(aN.VReversedParameter(V), aP.VReversedParameter(V), aWhat + " VReversedParameter");

                // Reversal
                ElementarySurface aPU = (ElementarySurface)aP.UReversed();
                using (var aNU = (N.Geom_ElementarySurface)N.Geom_ElementarySurface.DownCast(aN.Copy())!)
                {
                    aNU.UReverse();
                    CompareElementaryQueries(anOracle, aNU, aPU, aWhat + " UReversed");
                    CompareEvaluation(anOracle, aNU, aPU, aWhat + " UReversed", 3, Oracle.Tolerance, 2);
                }
                ElementarySurface aPV = (ElementarySurface)aP.VReversed();
                using (var aNV = (N.Geom_ElementarySurface)N.Geom_ElementarySurface.DownCast(aN.Copy())!)
                {
                    aNV.VReverse();
                    CompareElementaryQueries(anOracle, aNV, aPV, aWhat + " VReversed");
                    CompareEvaluation(anOracle, aNV, aPV, aWhat + " VReversed", 3, Oracle.Tolerance, 2);
                }

                // Transformation
                (N.gp_Trsf aNT, Transform3D aT) = RandomTrsf(anOracle);
                using (aNT)
                {
                    CompareParametricTransformation(anOracle, aN, aP, aNT, aT, aWhat);
                    ElementarySurface aPT = (ElementarySurface)aP.Transformed(aT);
                    using var aNTr = (N.Geom_ElementarySurface)N.Geom_ElementarySurface.DownCast(aN.Copy())!;
                    aNTr.Transform(aNT);
                    CompareElementaryQueries(anOracle, aNTr, aPT, aWhat + " transformed");
                    CompareEvaluation(anOracle, aNTr, aPT, aWhat + " transformed", 3, Oracle.Tolerance, 2);
                }
            }
        }
        anOracle.Report(aNames[aKind]);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Swept surfaces

    /// <summary>A circle (theKind 0) or a non-periodic B-spline curve, native and port.</summary>
    private static (N.Geom_Curve Native, Curve3D Port) RandomCurve(Oracle theOracle, int theKind)
    {
        if (theKind == 0)
        {
            (N.gp_Ax2 aNAx2, Frame3D anAx2) = Ax2s(theOracle);
            using var aNAx2Owner = aNAx2;
            double R = theOracle.Real(0.1, 10);
            return (new N.Geom_Circle(aNAx2, R), new CircleCurve3D(anAx2, R));
        }

        int aDeg = theOracle.Int(1, 5);
        theOracle.BSplineKnots(aDeg, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
        int aNbPoles = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, false, aMults);
        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = new Point3D(theOracle.Real(-10, 10), theOracle.Real(-10, 10), theOracle.Real(-10, 10));
        }
        NCollection_Array1<double>? aWeights = theOracle.Int(0, 1) == 0 ? null : theOracle.Weights(aNbPoles, true);
        return (Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDeg, false),
                aWeights is null ? new BSplineCurve3D(aPoles, aKnots, aMults, aDeg)
                                 : new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg));
    }

    private static void CompareCurveAt(Oracle theOracle, N.Geom_Curve theN, Curve3D theP, double U, string theWhat)
    {
        using N.gp_Pnt aNP = theN.EvalD0(U);
        theOracle.Same(aNP, theP.EvalD0(U), theWhat);
    }

    private static (N.Geom_SweptSurface Native, SweptSurface Port) RandomSwept(Oracle theOracle, bool theRevolution,
                                                                                   int theCurveKind)
    {
        (N.Geom_Curve aNC, Curve3D aPC) = RandomCurve(theOracle, theCurveKind);
        using (aNC)
        {
            if (theRevolution)
            {
                (N.gp_Ax1 aNAxis, Axis3D anAxis) = Ax1s(theOracle);
                using var aNAxisOwner = aNAxis;
                return (new N.Geom_SurfaceOfRevolution(aNC, aNAxis), new RevolutionSurface(aPC, anAxis));
            }
            (N.gp_Dir aNDir, Direction3D aDir) = Dirs(theOracle);
            using var aNDirOwner = aNDir;
            return (new N.Geom_SurfaceOfLinearExtrusion(aNC, aNDir), new ExtrusionSurface(aPC, aDir));
        }
    }

    private static void CompareSweptQueries(Oracle theOracle, N.Geom_SweptSurface theN, SweptSurface theP, string theWhat)
    {
        using (N.gp_Dir aNDir = theN.Direction())
        {
            SameDir(theOracle, aNDir, theP.Direction, theWhat + " Direction");
        }
        if (theP is RevolutionSurface aRev)
        {
            using var aNRev = N.Geom_SurfaceOfRevolution.DownCast(theN)!;
            using N.gp_Pnt aNLoc = aNRev.Location();
            theOracle.Same(aNLoc, aRev.Location, theWhat + " Location");
        }
        using N.Geom_Curve aNBasis = theN.BasisCurve()!;
        Curve3D aBasis = theP.BasisCurve;
        theOracle.Same(aNBasis.FirstParameter(), aBasis.FirstParameter, theWhat + " basis first");
        theOracle.Same(aNBasis.LastParameter(), aBasis.LastParameter, theWhat + " basis last");
        double U = theOracle.Real(aBasis.FirstParameter, aBasis.LastParameter);
        CompareCurveAt(theOracle, aNBasis, aBasis, U, theWhat + " basis D0");
    }

    /// <summary>
    /// The iso-curves of a swept surface: revolution UIso (the rotated basis curve) and VIso (a circle),
    /// extrusion UIso (a line) and VIso (the translated basis curve).
    /// </summary>
    private static void CompareIso(Oracle theOracle, N.Geom_SweptSurface theN, SweptSurface theP, string theWhat)
    {
        GeomIsoCurveCompare.SameIsos(theOracle, theN, theP, theWhat);
    }

    [Fact]
    public void Swept_surfaces()
    {
        foreach (int aCase in Tape.Budget(4))
        {
        bool aRevolution = aCase < 2;
        int aCurveKind = aCase % 2;
        var anOracle = new Oracle(output, 20261039 + aCase);
        for (int i = 0; i < Cases / 4; i++)
        {
            (N.Geom_SweptSurface aN, SweptSurface aP) = RandomSwept(anOracle, aRevolution, aCurveKind);
            string aWhat = global::Kerf.Engine.Foundation.OcctTypeNames.Of(aP.GetType()) + (aCurveKind == 0 ? " of a circle" : " of a B-spline");
            const bool aContinuity = true;
            using (aN)
            {
                CompareProperties(anOracle, aN, aP, aWhat, aContinuity);
                CompareSweptQueries(anOracle, aN, aP, aWhat);
                CompareEvaluation(anOracle, aN, aP, aWhat);
                CompareIso(anOracle, aN, aP, aWhat);
                aP.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
                double U = Param(anOracle, aU1, aU2), V = Param(anOracle, aV1, aV2);
                anOracle.Same(aN.UReversedParameter(U), aP.UReversedParameter(U), aWhat + " UReversedParameter");
                anOracle.Same(aN.VReversedParameter(V), aP.VReversedParameter(V), aWhat + " VReversedParameter");

                SweptSurface aPU = (SweptSurface)aP.UReversed();
                using (var aNU = (N.Geom_SweptSurface)N.Geom_SweptSurface.DownCast(aN.Copy())!)
                {
                    aNU.UReverse();
                    CompareSweptQueries(anOracle, aNU, aPU, aWhat + " UReversed");
                    CompareProperties(anOracle, aNU, aPU, aWhat + " UReversed", aContinuity);
                    CompareEvaluation(anOracle, aNU, aPU, aWhat + " UReversed", 3, Oracle.Tolerance, 2);
                }
                SweptSurface aPV = (SweptSurface)aP.VReversed();
                using (var aNV = (N.Geom_SweptSurface)N.Geom_SweptSurface.DownCast(aN.Copy())!)
                {
                    aNV.VReverse();
                    CompareSweptQueries(anOracle, aNV, aPV, aWhat + " VReversed");
                    CompareProperties(anOracle, aNV, aPV, aWhat + " VReversed", aContinuity);
                    CompareEvaluation(anOracle, aNV, aPV, aWhat + " VReversed", 3, Oracle.Tolerance, 2);
                }

                (N.gp_Trsf aNT, Transform3D aT) = RandomTrsf(anOracle);
                using (aNT)
                {
                    CompareParametricTransformation(anOracle, aN, aP, aNT, aT, aWhat);
                    SweptSurface aPT = (SweptSurface)aP.Transformed(aT);
                    using var aNTr = (N.Geom_SweptSurface)N.Geom_SweptSurface.DownCast(aN.Copy())!;
                    aNTr.Transform(aNT);
                    CompareSweptQueries(anOracle, aNTr, aPT, aWhat + " transformed");
                    CompareProperties(anOracle, aNTr, aPT, aWhat + " transformed", aContinuity);
                    CompareEvaluation(anOracle, aNTr, aPT, aWhat + " transformed", 3, Oracle.Tolerance, 2);
                    CompareIso(anOracle, aNTr, aPT, aWhat + " transformed");
                }
            }
        }
        anOracle.Report((aRevolution ? "Geom_SurfaceOfRevolution" : "Geom_SurfaceOfLinearExtrusion")
                        + (aCurveKind == 0 ? " of circles" : " of B-spline curves"));
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Rectangular trimmed surfaces

    private static (N.Geom_Surface Native, Surface Port) RandomBSplineSurface(Oracle theOracle)
    {
        int aUDeg = theOracle.Int(1, 4), aVDeg = theOracle.Int(1, 4);
        bool aUPer = theOracle.Int(0, 1) == 1, aVPer = theOracle.Int(0, 1) == 1;
        theOracle.BSplineKnots(aUDeg, aUPer, out NCollection_Array1<double> aUKnots, out NCollection_Array1<int> aUMults);
        theOracle.BSplineKnots(aVDeg, aVPer, out NCollection_Array1<double> aVKnots, out NCollection_Array1<int> aVMults);
        int aNbU = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aUDeg, aUPer, aUMults), aNbV = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aVDeg, aVPer, aVMults);
        var aPoles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
        var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                aPoles[i, j] = new Point3D(theOracle.Real(-10, 10), theOracle.Real(-10, 10), theOracle.Real(-10, 10));
                aWeights[i, j] = theOracle.Real(0.5, 1.5);
            }
        }
        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
        using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(aWeights);
        using N.TColStd_Array1OfReal aNUKnots = Oracle.ToNative(aUKnots), aNVKnots = Oracle.ToNative(aVKnots);
        using N.TColStd_Array1OfInteger aNUMults = Oracle.ToNative(aUMults), aNVMults = Oracle.ToNative(aVMults);
        return (new N.Geom_BSplineSurface(aNPoles, aNWeights, aNUKnots, aNVKnots, aNUMults, aNVMults, aUDeg, aVDeg, aUPer, aVPer),
                new BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, aUPer, aVPer));
    }

    private static (N.Geom_Surface Native, Surface Port) RandomBasis(Oracle theOracle, int theKind)
    {
        if (theKind < 5)
        {
            (N.Geom_ElementarySurface aN, ElementarySurface aP) = RandomElementary(theOracle, theKind);
            return (aN, aP);
        }
        if (theKind < 7)
        {
            (N.Geom_SweptSurface aN, SweptSurface aP) = RandomSwept(theOracle, theKind == 5, 0);
            return (aN, aP);
        }
        return RandomBSplineSurface(theOracle);
    }

    /// <summary>Two distinct trim values for a direction of bounds [theFirst, theLast].</summary>
    private static (double, double) RandomTrim(Oracle theOracle, double theFirst, double theLast, bool thePeriodic)
    {
        double aT1, aT2;
        do
        {
            if (thePeriodic)
            {
                double aPeriod = theLast - theFirst;
                aT1 = theOracle.Real(theFirst - aPeriod, theLast + aPeriod);
                aT2 = theOracle.Int(0, 5) == 0 ? aT1 + aPeriod : theOracle.Real(theFirst - aPeriod, theLast + aPeriod);
            }
            else
            {
                aT1 = Param(theOracle, theFirst, theLast);
                aT2 = Param(theOracle, theFirst, theLast);
            }
        } while (Math.Abs(aT2 - aT1) < 1e-3);
        return (aT1, aT2);
    }

    private static (N.Geom_RectangularTrimmedSurface Native, TrimmedSurface Port) RandomTrimmed(
        Oracle theOracle, N.Geom_Surface theN, Surface theP)
    {
        theP.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        int aMode = theOracle.Int(0, 2); // 0: both directions, 1: U only, 2: V only
        bool aUSense = theOracle.Int(0, 1) == 1, aVSense = theOracle.Int(0, 1) == 1;
        (double aTU1, double aTU2) = RandomTrim(theOracle, aU1, aU2, theP.IsUPeriodic);
        (double aTV1, double aTV2) = RandomTrim(theOracle, aV1, aV2, theP.IsVPeriodic);
        if (aMode == 0)
        {
            return (new N.Geom_RectangularTrimmedSurface(theN, aTU1, aTU2, aTV1, aTV2, aUSense, aVSense),
                    new TrimmedSurface(theP, aTU1, aTU2, aTV1, aTV2, aUSense, aVSense));
        }
        if (aMode == 1)
        {
            return (new N.Geom_RectangularTrimmedSurface(theN, aTU1, aTU2, true, aUSense),
                    new TrimmedSurface(theP, aTU1, aTU2, true, aUSense));
        }
        return (new N.Geom_RectangularTrimmedSurface(theN, aTV1, aTV2, false, aVSense),
                new TrimmedSurface(theP, aTV1, aTV2, false, aVSense));
    }

    private static void CompareTrimmed(Oracle theOracle, N.Geom_RectangularTrimmedSurface theN,
                                       TrimmedSurface theP, string theWhat)
    {
        CompareProperties(theOracle, theN, theP, theWhat);
        CompareEvaluation(theOracle, theN, theP, theWhat, 3, Oracle.Tolerance, 2);
        double U = theOracle.Real(-10, 10), V = theOracle.Real(-10, 10);
        theOracle.Same(theN.UReversedParameter(U), theP.UReversedParameter(U), theWhat + " UReversedParameter");
        theOracle.Same(theN.VReversedParameter(V), theP.VReversedParameter(V), theWhat + " VReversedParameter");
    }

    [Fact]
    public void Rectangular_trimmed_surfaces()
    {
        string[] aNames = ["planes", "cylinders", "cones", "spheres", "tori", "surfaces of revolution", "extrusions", "B-spline surfaces"];
        for (int aKind = 0; aKind < 8; aKind++)
        {
        var anOracle = new Oracle(output, 20261049 + aKind);
        for (int i = 0; i < Cases / 4; i++)
        {
            (N.Geom_Surface aNB, Surface aPB) = RandomBasis(anOracle, aKind);
            using (aNB)
            {
                (N.Geom_RectangularTrimmedSurface aN, TrimmedSurface aP) = RandomTrimmed(anOracle, aNB, aPB);
                string aWhat = "trimmed " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(aPB.GetType());
                using (aN)
                {
                    CompareTrimmed(anOracle, aN, aP, aWhat);

                    // Trimmed again: the basis is unwrapped and the other direction's trim kept
                    (N.Geom_RectangularTrimmedSurface aN2, TrimmedSurface aP2) = RandomTrimmed(anOracle, aN, aP);
                    using (aN2)
                    {
                        CompareTrimmed(anOracle, aN2, aP2, aWhat + " retrimmed");
                    }

                    // Copy
                    using (var aNC = (N.Geom_RectangularTrimmedSurface)N.Geom_RectangularTrimmedSurface.DownCast(aN.Copy())!)
                    {
                        CompareTrimmed(anOracle, aNC, (TrimmedSurface)aP.Copy(), aWhat + " copy");
                    }

                    // Reversal
                    using (var aNU = (N.Geom_RectangularTrimmedSurface)N.Geom_RectangularTrimmedSurface.DownCast(aN.Copy())!)
                    {
                        aNU.UReverse();
                        CompareTrimmed(anOracle, aNU, (TrimmedSurface)aP.UReversed(), aWhat + " UReversed");
                    }
                    using (var aNV = (N.Geom_RectangularTrimmedSurface)N.Geom_RectangularTrimmedSurface.DownCast(aN.Copy())!)
                    {
                        aNV.VReverse();
                        CompareTrimmed(anOracle, aNV, (TrimmedSurface)aP.VReversed(), aWhat + " VReversed");
                    }

                    // Transformation
                    (N.gp_Trsf aNT, Transform3D aT) = RandomTrsf(anOracle);
                    using (aNT)
                    {
                        CompareParametricTransformation(anOracle, aN, aP, aNT, aT, aWhat);
                        using var aNTr = (N.Geom_RectangularTrimmedSurface)N.Geom_RectangularTrimmedSurface.DownCast(aN.Copy())!;
                        aNTr.Transform(aNT);
                        CompareTrimmed(anOracle, aNTr, (TrimmedSurface)aP.Transformed(aT), aWhat + " transformed");
                    }
                }
            }
        }
        anOracle.Report("Geom_RectangularTrimmedSurface of " + aNames[aKind]);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Iso-curves

    /// <summary>The iso-curves of a surface, of its U and V reversed copies, and of a transformed copy.</summary>
    private static void CompareIsos(Oracle theOracle, N.Geom_Surface theN, Surface theP, string theWhat)
    {
        GeomIsoCurveCompare.SameIsos(theOracle, theN, theP, theWhat);
        using (var aNU = N.Geom_Surface.DownCast(theN.Copy())!)
        {
            aNU.UReverse();
            GeomIsoCurveCompare.SameIsos(theOracle, aNU, theP.UReversed(), theWhat + " UReversed");
        }
        using (var aNV = N.Geom_Surface.DownCast(theN.Copy())!)
        {
            aNV.VReverse();
            GeomIsoCurveCompare.SameIsos(theOracle, aNV, theP.VReversed(), theWhat + " VReversed");
        }
        (N.gp_Trsf aNT, Transform3D aT) = RandomTrsf(theOracle);
        using (aNT)
        using (var aNTr = N.Geom_Surface.DownCast(theN.Copy())!)
        {
            aNTr.Transform(aNT);
            GeomIsoCurveCompare.SameIsos(theOracle, aNTr, (Surface)theP.Transformed(aT), theWhat + " transformed");
        }
    }

    [Fact]
    public void Iso_curves()
    {
        string[] aNames = ["planes", "cylinders", "cones", "spheres", "tori", "surfaces of revolution of circles",
                           "extrusions of circles", "B-spline surfaces", "surfaces of revolution of B-spline curves",
                           "extrusions of B-spline curves"];
        for (int aKind = 0; aKind < aNames.Length; aKind++)
        {
        var anOracle = new Oracle(output, 20261029 + 100 + aKind);
        for (int i = 0; i < Cases / 10; i++)
        {
            (N.Geom_Surface aNB, Surface aPB) = aKind < 8 ? RandomBasis(anOracle, aKind)
                                                               : RandomSwept(anOracle, aKind == 8, 1);
            using (aNB)
            {
                string aWhat = global::Kerf.Engine.Foundation.OcctTypeNames.Of(aPB.GetType());
                CompareIsos(anOracle, aNB, aPB, aWhat);

                // Trimmed: the iso-curves of a trimmed direction are trimmed curves
                (N.Geom_RectangularTrimmedSurface aN, TrimmedSurface aP) = RandomTrimmed(anOracle, aNB, aPB);
                using (aN)
                {
                    CompareIsos(anOracle, aN, aP, "trimmed " + aWhat);
                }
            }
        }
        anOracle.Report("UIso and VIso of " + aNames[aKind] + " (and of trimmed ones)");
        }
    }
}
