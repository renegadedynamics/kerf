// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeFix_Face_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's ShapeFix_FaceTest GTests, with OCCT's names. The port's faces are frozen, as a caller's are.</summary>
public class ShapeFix_Face_Test
{
    // Creates a bounded planar face suitable for testing context replacements.
    // Returns a null face if construction fails.
    private static Face makePlanarFace()
    {
        var aMakeFace = new MakeFace(new Plane3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 0.0,
                                                    1.0, 0.0, 1.0);
        if (!aMakeFace.IsDone)
        {
            return new Face();
        }
        return aMakeFace.Face();
    }

    // A closed edge belting a cone's full period (8 points sampled off the cone surface).
    private static Face MakeFaceOnPeriodicConicalSingleWire(out ConicalSurface theCone)
    {
        var anAxis = new CoordinateSystem(new Point3D(0.0, 4.0, -9.1), new Direction3D(0.0, 0.09, -1.0));
        theCone = new ConicalSurface(anAxis, 0.35, 0.0);

        const int anN = 8;
        const double aV = 0.5;
        var aPts = new NCollection_HArray1<Point3D>(1, anN);
        for (int i = 1; i <= anN; i++)
        {
            double anU = 2.0 * Math.PI * (i - 1) / anN;
            aPts.SetValue(i, theCone.Value(anU, aV));
        }

        var anInterp = new ApiInterpolate(aPts, true, 1e-6);
        anInterp.Perform();
        if (!anInterp.IsDone)
        {
            return new Face();
        }

        var aMakeEdge = new MakeEdge(anInterp.Curve());
        if (!aMakeEdge.IsDone)
        {
            return new Face();
        }

        var aMakeWire = new LibraryMakeWire();
        aMakeWire.Add(aMakeEdge.Edge());
        if (!aMakeWire.IsDone || !aMakeWire.Wire().IsClosed)
        {
            return new Face();
        }

        var aMakeFace = new MakeFace(theCone, aMakeWire.Wire(), true);
        return aMakeFace.IsDone ? aMakeFace.Face() : new Face();
    }

    [Fact]
    public void Perform_FaceReplacedByCompound_PreservesReplacement()
    {
        Face aFace = makePlanarFace();
        Assert.False(aFace.IsNull);

        var aBuilder = new BrepBuilder();
        var aCompound = new Compound();
        aBuilder.MakeCompound(ref aCompound);

        var aContext = new ReShapeBuilder();
        aContext.Replace(aFace, aCompound);

        var aFixer = new FaceFixer(aFace);
        aFixer.SetContext(aContext);

        Assert.False(aFixer.Perform());
        Assert.True(aFixer.Status(ExtendStatus.OK));
        Assert.True(aFixer.Result().IsSame(aCompound));
    }

    [Fact]
    public void Perform_RemovedFace_PreservesNullReplacement()
    {
        Face aFace = makePlanarFace();
        Assert.False(aFace.IsNull);

        var aContext = new ReShapeBuilder();
        aContext.Remove(aFace);

        var aFixer = new FaceFixer(aFace);
        aFixer.SetContext(aContext);

        Assert.False(aFixer.Perform());
        Assert.True(aFixer.Status(ExtendStatus.OK));
        Assert.True(aFixer.Result().IsNull);
    }

    // No SetContext() call (the ordinary usage) -- used to SIGSEGV in FixPeriodicDegenerated().
    [Fact]
    public void NoCrashOnPeriodicConicalSingleWire()
    {
        Face aFace = MakeFaceOnPeriodicConicalSingleWire(out _);
        Assert.False(aFace.IsNull);

        var aFixer = new FaceFixer(aFace);
        aFixer.Perform();

        Assert.False(aFixer.Face().IsNull);
    }

    // With a context supplied, the same input heals to a genuinely valid face.
    [Fact]
    public void ValidFaceOnPeriodicConicalSingleWireWithContext()
    {
        Face aFace = MakeFaceOnPeriodicConicalSingleWire(out _);
        Assert.False(aFace.IsNull);

        var aFixer = new FaceFixer(aFace);
        aFixer.SetContext(new ReShapeBuilder());
        aFixer.Perform();

        Face aFixed = aFixer.Face();
        Assert.False(aFixed.IsNull);
        Assert.True(new Analyzer(aFixed).IsValid());
    }
}
