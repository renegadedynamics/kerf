// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.AdaptorOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The surface local properties against native OCCT's BRepLProp_SLProps (GeomLProp_SLPropsBase on a
/// BRepAdaptor_Surface, which evaluates through a GeomAdaptor_Surface): the port's LProp_SLProps3d on a
/// GeomAdaptor_Surface runs the same template code on the same evaluator, and GeomLProp_SLProps runs it on the
/// Geom_Surface directly.
/// </summary>
public class SLPropsOracleTests(ITestOutputHelper output)
{
    private static Exception? Try(Action theAction) => Catch(theAction);

    private static void Compare(Oracle theO, N.BRepLProp_SLProps theN, SLProps3d theP, string theWhat)
    {
        theO.Same(theN.Value(), theP.Value(), theWhat + " Value");
        theO.Same(theN.D1U(), theP.D1U(), theWhat + " D1U");
        theO.Same(theN.D1V(), theP.D1V(), theWhat + " D1V");
        theO.Same(theN.D2U(), theP.D2U(), theWhat + " D2U");
        theO.Same(theN.D2V(), theP.D2V(), theWhat + " D2V");
        theO.Same(theN.DUV(), theP.DUV(), theWhat + " DUV");

        bool aTanU = theN.IsTangentUDefined();
        theO.Same(aTanU, theP.IsTangentUDefined(), theWhat + " IsTangentUDefined");
        if (aTanU)
        {
            using var aND = new N.gp_Dir();
            theN.TangentU(aND);
            theP.TangentU(out Direction3D aD);
            SameDir(theO, aND, aD, theWhat + " TangentU");
        }

        bool aTanV = theN.IsTangentVDefined();
        theO.Same(aTanV, theP.IsTangentVDefined(), theWhat + " IsTangentVDefined");
        if (aTanV)
        {
            using var aND = new N.gp_Dir();
            theN.TangentV(aND);
            theP.TangentV(out Direction3D aD);
            SameDir(theO, aND, aD, theWhat + " TangentV");
        }

        bool aNormal = theN.IsNormalDefined();
        theO.Same(aNormal, theP.IsNormalDefined, theWhat + " IsNormalDefined");
        if (aNormal)
        {
            using N.gp_Dir aND = theN.Normal();
            SameDir(theO, aND, theP.Normal(), theWhat + " Normal");
        }

        bool aCurv = theN.IsCurvatureDefined();
        theO.Same(aCurv, theP.IsCurvatureDefined(), theWhat + " IsCurvatureDefined");
        if (aCurv)
        {
            theO.Same(theN.MaxCurvature(), theP.MaxCurvature(), theWhat + " MaxCurvature");
            theO.Same(theN.MinCurvature(), theP.MinCurvature(), theWhat + " MinCurvature");
            theO.Same(theN.MeanCurvature(), theP.MeanCurvature(), theWhat + " MeanCurvature");
            theO.Same(theN.GaussianCurvature(), theP.GaussianCurvature(), theWhat + " GaussianCurvature");
            theO.Same(theN.IsUmbilic(), theP.IsUmbilic(), theWhat + " IsUmbilic");
            using var aNMax = new N.gp_Dir();
            using var aNMin = new N.gp_Dir();
            theN.CurvatureDirections(aNMax, aNMin);
            theP.CurvatureDirections(out Direction3D aMax, out Direction3D aMin);
            SameDir(theO, aNMax, aMax, theWhat + " max direction");
            SameDir(theO, aNMin, aMin, theWhat + " min direction");
        }
    }

    // the Geom_Surface evaluation differs from the adaptor's (B-spline caches) in the last bits, which the
    // curvatures amplify near singular points: values within 1e-9 relative, flags equal
    private static void CompareGeom(Oracle theO, N.BRepLProp_SLProps theN, SLProps theP, string theWhat)
    {
        const double aTol = 1e-9;
        theO.Same(theN.Value(), theP.Value(), theWhat + " Value", aTol);
        theO.Same(theN.D1U(), theP.D1U(), theWhat + " D1U", aTol);
        theO.Same(theN.D2V(), theP.D2V(), theWhat + " D2V", aTol);
        if (theN.IsCurvatureDefined() && theP.IsCurvatureDefined())
        {
            theO.Same(theN.MeanCurvature(), theP.MeanCurvature(), theWhat + " MeanCurvature", aTol);
            theO.Same(theN.GaussianCurvature(), theP.GaussianCurvature(), theWhat + " GaussianCurvature", aTol);
        }
    }

    [Fact]
    public void SLProps_on_random_surfaces()
    {
        var anOracle = new Oracle(output);
        var aGeomOracle = new Oracle(output);
        int aCurvatureFirst = 0, aStale = 0;
        using var aBuilder = new N.BRep_Builder();
        foreach (int i in Tape.Budget(1100))
        {
            int aKind = i % SurfaceKinds.Length;
            (N.Geom_Surface aNS, Surface aPS) = RandomSurface(anOracle, aKind);
            using (aNS)
            {
                using var aFace = new N.TopoDS_Face();
                aBuilder.MakeFace(aFace, aNS, 1e-7);
                // without the face's restriction: the adaptor on the whole surface, as GeomAdaptor_Surface(S)
                using var aNAdaptor = new N.BRepAdaptor_Surface(aFace, false);
                var aPAdaptor = new SurfaceAdaptor(aPS);

                double aU1 = aPAdaptor.FirstUParameter(), aU2 = aPAdaptor.LastUParameter();
                double aV1 = aPAdaptor.FirstVParameter(), aV2 = aPAdaptor.LastVParameter();
                for (int j = 0; j < 6; j++)
                {
                    double aU = Param(anOracle, aU1, aU2), aV = Param(anOracle, aV1, aV2);
                    if (j == 0)
                    {
                        aU = aU1;
                    }
                    else if (j == 1 && !global::Kerf.Engine.Foundation.Precision.IsInfinite(aV2))
                    {
                        aV = aV2;
                    }

                    string aWhat = $"{SurfaceKinds[aKind]} #{i} ({aU}, {aV})";
                    int aN = anOracle.Int(0, 2);
                    double aTol = anOracle.Bool() ? 1e-7 : 1e-3;

                    // the curvatures asked first, before IsCurvatureDefined: native returns the stale value, the port
                    // the point's (plan 3.2's third level: occt-issues.md, unit 3J's LProp_SurfaceUtils row; Kerf
                    // decision 16711). A difference is intended when the port's value is the one it gives after
                    // IsCurvatureDefined, which Compare then checks against native's.
                    N.BRepLProp_SLProps? aNProps = null;
                    SLProps3d? aPProps = null;
                    Exception? aNE = Try(() => aNProps = new N.BRepLProp_SLProps(aNAdaptor, aU, aV, aN, aTol));
                    Exception? aPE = Try(() => aPProps = new SLProps3d(aPAdaptor, aU, aV, aN, aTol));
                    if (!SameOutcome(aNE, aPE, aWhat + " constructor"))
                    {
                        continue;
                    }

                    using (aNProps)
                    {
                        double aNMax = double.NaN, aPMax = double.NaN;
                        aNE = Try(() => aNMax = aNProps!.MaxCurvature());
                        aPE = Try(() => aPMax = aPProps!.MaxCurvature());
                        if (SameOutcome(aNE, aPE, aWhat + " MaxCurvature first"))
                        {
                            if (aNMax != aPMax && aPProps!.IsCurvatureDefined() && aPMax == aPProps.MaxCurvature())
                            {
                                aStale++;
                            }
                            else
                            {
                                anOracle.Same(aNMax, aPMax, aWhat + " MaxCurvature first");
                            }
                            aCurvatureFirst++;
                        }

                        Compare(anOracle, aNProps!, aPProps!, aWhat);

                        // GeomLProp_SLProps, on the Geom_Surface itself, evaluates without the adaptor's caches
                        var aGProps = new SLProps(aPS, aU, aV, aN, aTol);
                        aGProps.IsCurvatureDefined();
                        CompareGeom(aGeomOracle, aNProps!, aGProps, aWhat);

                        // a second parameter on the same object: the curvatures first again
                        double aU3 = Param(anOracle, aU1, aU2), aV3 = Param(anOracle, aV1, aV2);
                        aNE = Try(() => aNProps!.SetParameters(aU3, aV3));
                        aPE = Try(() => aPProps!.SetParameters(aU3, aV3));
                        if (SameOutcome(aNE, aPE, aWhat + " SetParameters"))
                        {
                            aNE = Try(() => aNMax = aNProps!.GaussianCurvature());
                            aPE = Try(() => aPMax = aPProps!.GaussianCurvature());
                            if (SameOutcome(aNE, aPE, aWhat + " GaussianCurvature first"))
                            {
                                if (aNMax != aPMax && aPProps!.IsCurvatureDefined() && aPMax == aPProps.GaussianCurvature())
                                {
                                    aStale++;
                                }
                                else
                                {
                                    anOracle.Same(aNMax, aPMax, aWhat + " GaussianCurvature first");
                                }
                            }

                            bool aNU = false, aPU = false;
                            aNE = Try(() => aNU = aNProps!.IsUmbilic());
                            aPE = Try(() => aPU = aPProps!.IsUmbilic());
                            if (SameOutcome(aNE, aPE, aWhat + " IsUmbilic first"))
                            {
                                if (aNU != aPU && aPU == aPProps!.IsUmbilic())
                                {
                                    aStale++;
                                }
                                else
                                {
                                    anOracle.Same(aNU, aPU, aWhat + " IsUmbilic first");
                                }
                            }
                        }
                    }
                }
            }
        }

        output.WriteLine($"{aCurvatureFirst} first calls of MaxCurvature (before IsCurvatureDefined) compared; {aStale} native stale values (intended differences)");
        anOracle.Report("LProp_SLProps3d on GeomAdaptor_Surface vs BRepLProp_SLProps");
        aGeomOracle.Report("GeomLProp_SLProps on Geom_Surface vs BRepLProp_SLProps (tolerance 1e-9)");
    }

    /// <summary>
    /// OCCT's MaxCurvature, MinCurvature, MeanCurvature and GaussianCurvature return the previous value when they are
    /// called before IsCurvatureDefined; the port's return the point's (occt-issues.md; Kerf decision 16711).
    /// </summary>
    [Fact]
    public void Curvature_asked_first_returns_the_stale_value()
    {
        using var aNOrigin = new N.gp_Pnt(0, 0, 0);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNAx3 = new N.gp_Ax3(aNOrigin, aNZ);
        using var aNSphere = new N.Geom_SphericalSurface(aNAx3, 2.0);
        using var aBuilder = new N.BRep_Builder();
        using var aFace = new N.TopoDS_Face();
        aBuilder.MakeFace(aFace, aNSphere, 1e-7);
        using var aNAdaptor = new N.BRepAdaptor_Surface(aFace, false);
        using var aNProps = new N.BRepLProp_SLProps(aNAdaptor, 0.3, 0.2, 2, 1e-7);

        var aPSphere = new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        var aPProps = new SLProps(aPSphere, 0.3, 0.2, 2, 1e-7);

        Assert.Equal(0.0, aNProps.MaxCurvature());
        Assert.Equal(-0.5, aPProps.MaxCurvature(), 1e-15);
        Assert.Equal(aNProps.MaxCurvature(), aPProps.MaxCurvature());
        Assert.Equal(-0.5, aPProps.MaxCurvature(), 1e-15);
    }
}
