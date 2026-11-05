// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Custom;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// ShapeCustom's package functions (unit 7A) against native OCCT's, which build123d.NET binds: BSplineRestriction
/// (build123d.NET's <c>ToSplines</c>) on primitives and Boolean results, and all six functions (DirectFaces,
/// ScaleShape, ConvertToRevolution, SweptToElementary, ConvertToBSpline, BSplineRestriction) on the heal group's public
/// inputs, data/occ's shapes and generated swept, mirrored and conical shapes. Native OCCT writes each input; both
/// sides read it (the port's frozen, as a caller's), and the results are compared as BRep text, the Free column masked
/// (open question 1), then canonically (decision 581); the port's input is checked unchanged and its result frozen
/// (decision 973).
/// </summary>
/// <remarks>
/// ShapeCustom_RestrictionParameters' setters aren't bound: BSplineRestriction runs with the default parameters here,
/// and with others in the recorded tests (tools/shcustomrec).
/// </remarks>
public class ShapeCustomOracleTests(ITestOutputHelper output)
{
    /// <summary>One operation, on both sides.</summary>
    internal sealed record Op(string Name, Func<N.TopoDS_Shape, N.TopoDS_Shape> Native, Func<Shape, Shape> Port);

    private static N.GeomAbs_Shape NC(Continuity theShape) => (N.GeomAbs_Shape)(int)theShape;

    /// <summary>BSplineRestriction with the default restriction parameters, as build123d.NET's ToSplines calls it.</summary>
    internal static Op ToSplines(int theDegree, double theTolerance, bool theNurbs)
        => BSplineRestriction($"ToSplines({theDegree}, {theTolerance}, {theNurbs})", theTolerance, theTolerance, theDegree, 1,
                              Continuity.C0, Continuity.C0, true, !theNurbs);

    internal static Op BSplineRestriction(string theName, double theTol3d, double theTol2d, int theMaxDegree, int theMaxNbSegment,
                                          Continuity theCont3d, Continuity theCont2d, bool theDegree, bool theRational)
        => new(theName,
               s =>
               {
                   using var aParameters = new N.ShapeCustom_RestrictionParameters();
                   return N.ShapeCustom.BSplineRestriction(s, theTol3d, theTol2d, theMaxDegree, theMaxNbSegment, NC(theCont3d),
                                                           NC(theCont2d), theDegree, theRational, aParameters);
               },
               s => Customize.BSplineRestriction(s, theTol3d, theTol2d, theMaxDegree, theMaxNbSegment, theCont3d, theCont2d,
                                                   theDegree, theRational, new RestrictionParameters()));

    // Draw's bsplres defaults (SWDRAW_ShapeCustom): the modification's own defaults
    private static readonly Op s_bsplres = BSplineRestriction("bsplres", 0.01, 1e-6, 9, 10000, Continuity.C1,
                                                              Continuity.C2, true, false);

    internal static readonly Op[] s_modifications =
    [
        new("DirectFaces", N.ShapeCustom.DirectFaces, s => Customize.DirectFaces(s)),
        new("ScaleShape(2.5)", s => N.ShapeCustom.ScaleShape(s, 2.5), s => Customize.ScaleShape(s, 2.5)),
        new("ScaleShape(0.001)", s => N.ShapeCustom.ScaleShape(s, 0.001), s => Customize.ScaleShape(s, 0.001)),
        new("ConvertToRevolution", N.ShapeCustom.ConvertToRevolution, s => Customize.ConvertToRevolution(s)),
        new("SweptToElementary", N.ShapeCustom.SweptToElementary, s => Customize.SweptToElementary(s)),
        new("ConvertToBSpline", s => N.ShapeCustom.ConvertToBSpline(s, true, true, true, false),
            s => Customize.ConvertToBSpline(s, true, true, true, false)),
        new("ConvertToBSpline(planes, no offsets)", s => N.ShapeCustom.ConvertToBSpline(s, true, true, false, true),
            s => Customize.ConvertToBSpline(s, true, true, false, true)),
    ];

    private static (Shape?, string?) RunPort(Op theOp, Shape theShape)
    {
        try
        {
            return (theOp.Port(theShape), null);
        }
        catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
        {
            return (null, ShapeFixOracleRun.ErrorOf(e));
        }
        catch (NullReferenceException)
        {
            // where native dereferences null (SweptToElementary's NewCurve2d on the modifier's null face,
            // occt-issues.md): an access violation natively, which build123d.NET's signal handler reports (decision 917)
            return (null, "OSD_Exception_ACCESS_VIOLATION");
        }
    }

    private static (N.TopoDS_Shape?, string?) RunNative(Op theOp, N.TopoDS_Shape theShape)
    {
        try
        {
            return (theOp.Native(theShape), null);
        }
        catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
        {
            return (null, ShapeFixOracleRun.ErrorOf(e));
        }
    }

    internal static void Compare(ShapeFixOracleRun theRun, string theName, N.TopoDS_Shape theShape, Op theOp)
    {
        if (theRun.Read(theShape) is not (N.TopoDS_Shape aNIn, Shape aPIn))
        {
            return;
        }
        string aPInText = theRun.PortText(aPIn);
        (N.TopoDS_Shape? aNRes, string? aNError) = RunNative(theOp, aNIn);
        (Shape? aPRes, string? aPError) = RunPort(theOp, aPIn);
        // The Free and Checked columns are masked where native clears the Checked bit of its own input's sub-shapes that
        // the modification leaves alone (BRepTools_Modifier's SetShapeFlags(S, S): Modified(true) clears Checked), which
        // the port's frozen input keeps (decision 347)
        theRun.Check($"{theName} [{theOp.Name}]", aNRes, aNError, "", aPRes, aPError, "", aNIn, aPIn, aPInText);
    }

    [Fact]
    public void BSplineRestriction_as_ToSplines_on_primitives_and_Boolean_results()
    {
        var aWatch = Stopwatch.StartNew();
        var anOracle = new Oracle(output, 20261101);
        using var aRun = new ShapeFixOracleRun(output, "shcustom");
        int[] aDegrees = [1, 2, 3, 3, 5, 8];
        double[] aTolerances = [1e-3, 1e-3, 1e-2, 1e-4];
        foreach ((string aName, N.TopoDS_Shape aShape) in Tape.Budget(Primitives(anOracle).Concat(ShapeUpgradeUnifySameDomainOracleTests.BooleanResults(anOracle, 120))))
        {
            Compare(aRun, aName, aShape, ToSplines(3, 1e-3, false)); // build123d.NET's defaults
            Compare(aRun, aName, aShape, ToSplines(aDegrees[anOracle.Int(0, aDegrees.Length - 1)],
                                                   aTolerances[anOracle.Int(0, aTolerances.Length - 1)], anOracle.Int(0, 1) == 0));
        }
        aRun.Report("BSplineRestriction as ToSplines on primitives and Boolean results", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }

    [Fact]
    public void All_functions_on_generated_swept_mirrored_and_conical_shapes()
    {
        var aWatch = Stopwatch.StartNew();
        var anOracle = new Oracle(output, 20261102);
        using var aRun = new ShapeFixOracleRun(output, "shcustom");
        foreach ((string aName, N.TopoDS_Shape aShape) in Tape.Budget(Primitives(anOracle).Concat(Swept(anOracle))))
        {
            foreach (Op anOp in s_modifications)
            {
                Compare(aRun, aName, aShape, anOp);
            }
            Compare(aRun, aName, aShape, s_bsplres);
            Compare(aRun, aName, aShape, ToSplines(3, 1e-3, false));
        }
        aRun.Report("ShapeCustom on generated shapes", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }

    /// <summary>The six functions on the heal group's public inputs and data/occ's shapes (up to 300 kB).</summary>
    [Fact]
    public void All_functions_on_the_heal_groups_and_data_occs_shapes()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "shcustom");
        foreach ((string aFile, string _) in Tape.Budget(aRun.Inputs(300_000), isSpread: true))
        {
            if (ShapeFixOracleRun.NativeRead(aFile) is not N.TopoDS_Shape aShape)
            {
                continue;
            }
            string aName = Path.GetFileName(aFile);
            foreach (Op anOp in s_modifications)
            {
                Compare(aRun, aName, aShape, anOp);
            }
        }
        aRun.Report("ShapeCustom's modifications on the heal group's and data/occ's shapes", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }

    /// <summary>BSplineRestriction on the heal group's public inputs and data/occ's shapes (up to 100 kB).</summary>
    [Fact]
    public void BSplineRestriction_on_the_heal_groups_and_data_occs_shapes()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "shcustom");
        foreach ((string aFile, string _) in Tape.Budget(aRun.Inputs(100_000), isSpread: true))
        {
            if (ShapeFixOracleRun.NativeRead(aFile) is not N.TopoDS_Shape aShape)
            {
                continue;
            }
            string aName = Path.GetFileName(aFile);
            Compare(aRun, aName, aShape, ToSplines(3, 1e-3, false));
            Compare(aRun, aName, aShape, s_bsplres);
        }
        aRun.Report("BSplineRestriction on the heal group's and data/occ's shapes", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Generated inputs

    private static N.TopoDS_Shape Shape(N.BRepBuilderAPI_MakeShape theMaker)
    {
        using (theMaker)
        {
            return theMaker.Shape();
        }
    }

    private static Frame3D Frame(Oracle theO)
    {
        Direction3D aZ = theO.Dir();
        Direction3D aRef = theO.Dir();
        Direction3D aX = aZ.Crossed(aRef.IsParallel(aZ, 0.1) ? (aZ.IsParallel(new Direction3D(1, 0, 0), 0.1) ? new Direction3D(0, 1, 0) : new Direction3D(1, 0, 0)) : aRef);
        return new Frame3D(new Point3D(theO.Real(-20, 20), theO.Real(-20, 20), theO.Real(-20, 20)), aZ, aX);
    }

    private static N.TopoDS_Shape Mirrored(N.TopoDS_Shape theShape, in Frame3D thePlane)
    {
        using var aT = new N.gp_Trsf();
        using N.gp_Ax2 aA = Oracle.ToNative(thePlane);
        aT.SetMirror(aA);
        return Shape(new N.BRepBuilderAPI_Transform(theShape, aT, true));
    }

    /// <summary>Primitives at random placements, a cone of negative semi-angle, and some mirrored (indirect surfaces).</summary>
    internal static IEnumerable<(string Name, N.TopoDS_Shape Shape)> Primitives(Oracle theO)
    {
        for (int i = 0; i < 24; i++)
        {
            Frame3D aF = Frame(theO);
            using N.gp_Ax2 aA = Oracle.ToNative(aF);
            double a = theO.Real(1, 10), b = theO.Real(1, 10), c = theO.Real(1, 10);
            (string aName, N.TopoDS_Shape aShape) = (i % 6) switch
            {
                0 => ("box", Shape(new N.BRepPrimAPI_MakeBox(aA, a, b, c))),
                1 => ("cylinder", Shape(new N.BRepPrimAPI_MakeCylinder(aA, a, c))),
                2 => ("cone", Shape(new N.BRepPrimAPI_MakeCone(aA, a, a * theO.Real(0, 0.8), c))),
                3 => ("cone of negative semi-angle", Shape(new N.BRepPrimAPI_MakeCone(aA, a * theO.Real(0, 0.8), a, c))),
                4 => ("sphere", Shape(new N.BRepPrimAPI_MakeSphere(aA, a))),
                _ => ("torus", Shape(new N.BRepPrimAPI_MakeTorus(aA, a + b, Math.Min(a, b) * 0.5))),
            };
            yield return ($"{aName} {i}", aShape);
            if (i % 2 == 0)
            {
                yield return ($"mirrored {aName} {i}", Mirrored(aShape, Frame(theO)));
            }
        }
    }

    /// <summary>
    /// Prisms and revolutions of lines, circles and B-spline curves (surfaces of extrusion and revolution, some of them
    /// elementary in disguise), some placed at a location.
    /// </summary>
    internal static IEnumerable<(string Name, N.TopoDS_Shape Shape)> Swept(Oracle theO)
    {
        for (int i = 0; i < 24; i++)
        {
            Frame3D aF = Frame(theO);
            using N.gp_Ax2 aA = Oracle.ToNative(aF);
            double r = theO.Real(1, 10);
            N.TopoDS_Shape anEdge;
            string aCurve;
            switch (i % 3)
            {
                case 0:
                {
                    using var aCirc = new N.gp_Circ(aA, r);
                    anEdge = Shape(new N.BRepBuilderAPI_MakeEdge(aCirc, 0, theO.Real(1, 2 * Math.PI)));
                    aCurve = "arc";
                    break;
                }
                case 1:
                {
                    using N.gp_Pnt aP1 = Oracle.ToNative(aF.Location);
                    using N.gp_Pnt aP2 = Oracle.ToNative(aF.Location.Translated(new Vector3D(aF.XDirection) * r));
                    anEdge = Shape(new N.BRepBuilderAPI_MakeEdge(aP1, aP2));
                    aCurve = "segment";
                    break;
                }
                default:
                {
                    // a cubic B-spline in the frame's XZ plane
                    var aPoles = new N.TColgp_Array1OfPnt(1, 5);
                    for (int k = 1; k <= 5; k++)
                    {
                        Point3D aP = aF.Location.Translated(new Vector3D(aF.XDirection) * (r + theO.Real(0, 3))
                                                             + new Vector3D(aF.Direction) * (k * 2.0));
                        using N.gp_Pnt aNP = Oracle.ToNative(aP);
                        aPoles.SetValue(k, aNP);
                    }
                    using var aBuilder = new N.GeomAPI_PointsToBSpline(aPoles);
                    anEdge = Shape(new N.BRepBuilderAPI_MakeEdge(aBuilder.Curve()));
                    aCurve = "B-spline";
                    break;
                }
            }
            if (i % 2 == 0)
            {
                using N.gp_Vec aV = Oracle.ToNative(new Vector3D(aF.YDirection) * theO.Real(1, 10));
                yield return ($"prism of {aCurve} {i}", Shape(new N.BRepPrimAPI_MakePrism(anEdge, aV)));
            }
            else
            {
                // about the frame's Z axis, or the axis through the frame's origin along X for a segment (a cone or plane)
                Axis3D anAxis = new Axis3D(aF.Location, i % 3 == 1 ? aF.YDirection : aF.Direction);
                using N.gp_Ax1 aNAxis = Oracle.ToNative(anAxis);
                double anAngle = theO.Int(0, 1) == 0 ? 2 * Math.PI : theO.Real(0.5, 6);
                yield return ($"revolution of {aCurve} {i}", Shape(new N.BRepPrimAPI_MakeRevol(anEdge, aNAxis, anAngle)));
            }
        }
    }
}
