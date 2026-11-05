// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_LineReducer_Test.cxx
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

public class LineReducerTests : BaseTestFixture
{
    // Perform removal of duplicate entities.
    HashSet<Transient> replaceDuplicateLines()
    {
        var aReducer = new LineReducer(myWS);
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
        // Creating Lines.
        var aLine1 = addLine("Line1");
        var aLine2 = addLine("Line2");

        // Creating EdgeCurve containing the first Line.
        var aFirstEdgeCurve = new StepEdgeCurve();
        aFirstEdgeCurve.Init(new TCollection_HAsciiString(),
                                                    new StepVertex(),
                                                    new StepVertex(),
                                                    aLine1,
                                                    true);
        addToModel(aFirstEdgeCurve);

        // Creating EdgeCurve containing the second Line.
        var aSecondEdgeCurve = new StepEdgeCurve();
        aSecondEdgeCurve.Init(new TCollection_HAsciiString(),
                                                      new StepVertex(),
                                                      new StepVertex(),
                                                      aLine2,
                                                      true);
        addToModel(aSecondEdgeCurve);

        // Performing removal of duplicate Lines.
        HashSet<Transient> aRemovedEntities = replaceDuplicateLines();

        // Check that nothing was removed.
        Assert.Empty(aRemovedEntities);
    }

    [Fact]
    public void EdgeCurve()
    {
        // Creating Lines.
        var aLine1 = addLine();
        var aLine2 = addLine();

        // Creating EdgeCurve containing the first Line.
        var aFirstEdgeCurve = new StepEdgeCurve();
        aFirstEdgeCurve.Init(new TCollection_HAsciiString(),
                                                    new StepVertex(),
                                                    new StepVertex(),
                                                    aLine1,
                                                    true);
        addToModel(aFirstEdgeCurve);

        // Creating EdgeCurve containing the second Line.
        var aSecondEdgeCurve = new StepEdgeCurve();
        aSecondEdgeCurve.Init(new TCollection_HAsciiString(),
                                                      new StepVertex(),
                                                      new StepVertex(),
                                                      aLine2,
                                                      true);
        addToModel(aSecondEdgeCurve);

        // Performing removal of duplicate Lines.
        HashSet<Transient> aRemovedEntities = replaceDuplicateLines();

        // Check that one Line was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aLine1) || aRemovedEntities.Contains(aLine2));
    }

    [Fact]
    public void TrimmedCurve()
    {
        // Creating Lines.
        var aLine1 = addLine();
        var aLine2 = addLine();

        // Creating TrimmedCurve containing the first Line.
        var aFirstTrimmedCurve = new StepTrimmedCurve();
        aFirstTrimmedCurve.Init(new TCollection_HAsciiString(),
                                                          aLine1,
                                                          new NCollection_HArray1<StepTrimmingSelect>(),
                                                          new NCollection_HArray1<StepTrimmingSelect>(),
                                                          true,
                                                          StepTrimmingPreference.TpUnspecified);
        addToModel(aFirstTrimmedCurve);

        // Creating TrimmedCurve containing the second Line.
        var aSecondTrimmedCurve = new StepTrimmedCurve();
        aSecondTrimmedCurve.Init(new TCollection_HAsciiString(),
                                                            aLine2,
                                                            new NCollection_HArray1<StepTrimmingSelect>(),
                                                            new NCollection_HArray1<StepTrimmingSelect>(),
                                                            true,
                                                            StepTrimmingPreference.TpUnspecified);
        addToModel(aSecondTrimmedCurve);

        // Performing removal of duplicate Lines.
        HashSet<Transient> aRemovedEntities = replaceDuplicateLines();

        // Check that one Line was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aLine1) || aRemovedEntities.Contains(aLine2));
    }

    [Fact]
    public void SurfaceCurve()
    {
        // Creating Lines.
        var aLine1 = addLine();
        var aLine2 = addLine();

        // Creating SurfaceCurve containing the first Line.
        var aFirstSurfaceCurve = new StepSurfaceCurve();
        aFirstSurfaceCurve.Init(new TCollection_HAsciiString(),
                                                          aLine1,
                                                          new NCollection_HArray1<StepPcurveOrSurface>(),
                                                          StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aFirstSurfaceCurve);

        // Creating SurfaceCurve containing the second Line.
        var aSecondSurfaceCurve = new StepSurfaceCurve();
        aSecondSurfaceCurve.Init(new TCollection_HAsciiString(),
                                                            aLine2,
                                                            new NCollection_HArray1<StepPcurveOrSurface>(),
                                                            StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aSecondSurfaceCurve);

        // Performing removal of duplicate Lines.
        HashSet<Transient> aRemovedEntities = replaceDuplicateLines();

        // Check that one Line was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aLine1) || aRemovedEntities.Contains(aLine2));
    }

    [Fact]
    public void DefinitionalRepresentation()
    {
        // Creating Lines.
        var aLine1 = addLine();
        var aLine2 = addLine();

        // Creating DefinitionalRepresentation containing the first Line.
        var aFirstItems =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aFirstItems.SetValue(1, aLine1);
        var aFirstDefinitionalRepresentation =
            new DefinitionalRepresentation();
        aFirstDefinitionalRepresentation.Init(new TCollection_HAsciiString(),
                                                                                      aFirstItems,
                                                                                      new RepresentationContext());
        addToModel(aFirstDefinitionalRepresentation);

        // Creating DefinitionalRepresentation containing the second Line.
        var aSecondItems =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aSecondItems.SetValue(1, aLine2);
        var aSecondDefinitionalRepresentation =
            new DefinitionalRepresentation();
        aSecondDefinitionalRepresentation.Init(new TCollection_HAsciiString(),
                                                                                        aSecondItems,
                                                                                        new RepresentationContext());

        // Performing removal of duplicate Lines.
        HashSet<Transient> aRemovedEntities = replaceDuplicateLines();

        // Check that one Line was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aLine1) || aRemovedEntities.Contains(aLine2));
    }

    [Fact]
    public void SeamCurve()
    {
        // Creating Lines.
        var aLine1 = addLine();
        var aLine2 = addLine();

        // Creating SeamCurve containing the first Line.
        var aFirstSeamCurve = new StepSeamCurve();
        aFirstSeamCurve.Init(new TCollection_HAsciiString(),
                                                    aLine1,
                                                    new NCollection_HArray1<StepPcurveOrSurface>(),
                                                    StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aFirstSeamCurve);

        // Creating SeamCurve containing the second Line.
        var aSecondSeamCurve = new StepSeamCurve();
        aSecondSeamCurve.Init(new TCollection_HAsciiString(),
                                                      aLine2,
                                                      new NCollection_HArray1<StepPcurveOrSurface>(),
                                                      StepPreferredSurfaceCurveRepresentation.PscrCurve3d);
        addToModel(aSecondSeamCurve);

        // Performing removal of duplicate Lines.
        HashSet<Transient> aRemovedEntities = replaceDuplicateLines();

        // Check that one Line was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aLine1) || aRemovedEntities.Contains(aLine2));
    }
}
