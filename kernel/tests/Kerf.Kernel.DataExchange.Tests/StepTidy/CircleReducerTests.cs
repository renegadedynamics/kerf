// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_CircleReducer_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Tidy;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;

namespace Kerf.Kernel.DataExchange.Tests.StepTidy;

public class CircleReducerTests : BaseTestFixture
{
    // Perform removal of duplicate entities.
    HashSet<Transient> replaceDuplicateCircles()
    {
        var aReducer = new CircleReducer(myWS);
        for (int anIndex = 1; anIndex <= myWS.Model()!.EntityCount; ++anIndex)
        {
            aReducer.ProcessEntity(myWS.Model()!.Value(anIndex)!);
        }

        var aRemovedEntities = new HashSet<Transient>(ReferenceEqualityComparer.Instance);
        aReducer.Perform(aRemovedEntities);
        return aRemovedEntities;
    }

    [Fact]
    public void DifferentNames()
    {
        // Creating Circles.
        var aCircle1 = addCircle("Circle1");
        var aCircle2 = addCircle("Circle2");

        // Creating EdgeCurve containing the first Circle.
        var aFirstEdgeCurve = new StepEdgeCurve();
        aFirstEdgeCurve.Init(new TCollection_HAsciiString(),
                                                    new StepVertex(),
                                                    new StepVertex(),
                                                    aCircle1,
                                                    true);
        addToModel(aFirstEdgeCurve);

        // Creating EdgeCurve containing the second Circle.
        var aSecondEdgeCurve = new StepEdgeCurve();
        aSecondEdgeCurve.Init(new TCollection_HAsciiString(),
                                                      new StepVertex(),
                                                      new StepVertex(),
                                                      aCircle2,
                                                      true);
        addToModel(aSecondEdgeCurve);

        // Performing removal of duplicate Circles.
        HashSet<Transient> aRemovedEntities = replaceDuplicateCircles();

        // Check that nothing was removed.
        Assert.Empty(aRemovedEntities);
    }

    [Fact]
    public void EdgeCurve()
    {
        // Creating Circles.
        var aCircle1 = addCircle();
        var aCircle2 = addCircle();

        // Creating EdgeCurve containing the first Circle.
        var aFirstEdgeCurve = new StepEdgeCurve();
        aFirstEdgeCurve.Init(new TCollection_HAsciiString(),
                                                    new StepVertex(),
                                                    new StepVertex(),
                                                    aCircle1,
                                                    true);
        addToModel(aFirstEdgeCurve);

        // Creating EdgeCurve containing the second Circle.
        var aSecondEdgeCurve = new StepEdgeCurve();
        aSecondEdgeCurve.Init(new TCollection_HAsciiString(),
                                                      new StepVertex(),
                                                      new StepVertex(),
                                                      aCircle2,
                                                      true);
        addToModel(aSecondEdgeCurve);

        // Performing removal of duplicate Circles.
        HashSet<Transient> aRemovedEntities = replaceDuplicateCircles();

        // Check that one Circle was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aCircle1) || aRemovedEntities.Contains(aCircle2));
    }

    [Fact]
    public void SurfaceCurve()
    {
        // Creating Circles.
        var aCircle1 = addCircle();
        var aCircle2 = addCircle();

        // Creating SurfaceCurve containing the first Circle.
        var aFirstSurfaceCurve = new StepSurfaceCurve();
        aFirstSurfaceCurve.Init(new TCollection_HAsciiString(),
                                                          aCircle1,
                                                          new NCollection_HArray1<StepPcurveOrSurface>(),
                                                          StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aFirstSurfaceCurve);

        // Creating SurfaceCurve containing the second Circle.
        var aSecondSurfaceCurve = new StepSurfaceCurve();
        aSecondSurfaceCurve.Init(new TCollection_HAsciiString(),
                                                            aCircle2,
                                                            new NCollection_HArray1<StepPcurveOrSurface>(),
                                                            StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aSecondSurfaceCurve);

        // Performing removal of duplicate Circles.
        HashSet<Transient> aRemovedEntities = replaceDuplicateCircles();

        // Check that one Circle was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aCircle1) || aRemovedEntities.Contains(aCircle2));
    }

    [Fact]
    public void SeamCurve()
    {
        // Creating Circles.
        var aCircle1 = addCircle();
        var aCircle2 = addCircle();

        // Creating SeamCurve containing the first Circle.
        var aFirstSeamCurve = new StepSeamCurve();
        aFirstSeamCurve.Init(new TCollection_HAsciiString(),
                                                    aCircle1,
                                                    new NCollection_HArray1<StepPcurveOrSurface>(),
                                                    StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aFirstSeamCurve);

        // Creating SeamCurve containing the second Circle.
        var aSecondSeamCurve = new StepSeamCurve();
        aSecondSeamCurve.Init(new TCollection_HAsciiString(),
                                                      aCircle2,
                                                      new NCollection_HArray1<StepPcurveOrSurface>(),
                                                      StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aSecondSeamCurve);

        // Performing removal of duplicate Circles.
        HashSet<Transient> aRemovedEntities = replaceDuplicateCircles();

        // Check that one Circle was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aCircle1) || aRemovedEntities.Contains(aCircle2));
    }
}
