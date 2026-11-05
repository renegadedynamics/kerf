// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_PlaneReducer_Test.cxx
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

public class PlaneReducerTests : BaseTestFixture
{
    // Perform removal of duplicate entities.
    HashSet<Transient> replaceDuplicatePlanes()
    {
        var aReducer = new PlaneReducer(myWS);
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
        // Creating Planes.
        var aPlane1 = addPlane("Plane1");
        var aPlane2 = addPlane("Plane2");

        // Creating StepShape_AdvancedFace containing the first Plane.
        var aFirstAdvancedFace = new StepAdvancedFace();
        aFirstAdvancedFace.Init(new TCollection_HAsciiString(),
                                                          new NCollection_HArray1<StepFaceBound>(),
                                                          aPlane1,
                                                          true);
        addToModel(aFirstAdvancedFace);

        // Creating StepShape_AdvancedFace containing the second Plane.
        var aSecondAdvancedFace = new StepAdvancedFace();
        aSecondAdvancedFace.Init(new TCollection_HAsciiString(),
                                                            new NCollection_HArray1<StepFaceBound>(),
                                                            aPlane2,
                                                            true);
        addToModel(aSecondAdvancedFace);

        // Performing removal of duplicate Planes.
        HashSet<Transient> aRemovedEntities = replaceDuplicatePlanes();

        // Check that nothing was removed.
        Assert.Empty(aRemovedEntities);
    }

    [Fact]
    public void AdvancedFace()
    {
        // Creating Planes.
        var aPlane1 = addPlane();
        var aPlane2 = addPlane();

        // Creating StepShape_AdvancedFace containing the first Plane.
        var aFirstAdvancedFace = new StepAdvancedFace();
        aFirstAdvancedFace.Init(new TCollection_HAsciiString(),
                                                          new NCollection_HArray1<StepFaceBound>(),
                                                          aPlane1,
                                                          true);
        addToModel(aFirstAdvancedFace);

        // Creating StepShape_AdvancedFace containing the second Plane.
        var aSecondAdvancedFace = new StepAdvancedFace();
        aSecondAdvancedFace.Init(new TCollection_HAsciiString(),
                                                            new NCollection_HArray1<StepFaceBound>(),
                                                            aPlane2,
                                                            true);
        addToModel(aSecondAdvancedFace);

        // Performing removal of duplicate Planes.
        HashSet<Transient> aRemovedEntities = replaceDuplicatePlanes();

        // Check that one Plane was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPlane1) || aRemovedEntities.Contains(aPlane2));
    }

    [Fact]
    public void Pcurve()
    {
        // Creating Planes.
        var aPlane1 = addPlane();
        var aPlane2 = addPlane();

        // Creating StepGeom_Pcurve containing the first Plane.
        var aFirstPcurve = new StepPcurve();
        aFirstPcurve.Init(new TCollection_HAsciiString(),
                                              aPlane1,
                                              new DefinitionalRepresentation());
        addToModel(aFirstPcurve);

        // Creating StepGeom_Pcurve containing the second Plane.
        var aSecondPcurve = new StepPcurve();
        aSecondPcurve.Init(new TCollection_HAsciiString(),
                                                aPlane2,
                                                new DefinitionalRepresentation());
        addToModel(aSecondPcurve);

        // Performing removal of duplicate Planes.
        HashSet<Transient> aRemovedEntities = replaceDuplicatePlanes();

        // Check that one Plane was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPlane1) || aRemovedEntities.Contains(aPlane2));
    }
}
