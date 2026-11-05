// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_VectorReducer_Test.cxx
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

public class VectorReducerTests : BaseTestFixture
{
    // Perform removal of duplicate entities.
    HashSet<Transient> replaceDuplicateVectors()
    {
        var aReducer = new VectorReducer(myWS);
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
        // Creating Vectors.
        var aVec1 = addVector("vec1");
        var aVec2 = addVector("vec2");

        // Creating a cartesian point for the lines.
        var aPnt = addCartesianPoint(null, new Xyz(0.0, 0.0, 0.0));

        // Creating aLine containing the first Vector.
        var aLine1 = new StepLine();
        aLine1.Init(new TCollection_HAsciiString(), aPnt, aVec1);
        addToModel(aLine1);

        // Creating aLine containing the second Vector.
        var aLine2 = new StepLine();
        aLine2.Init(new TCollection_HAsciiString(), aPnt, aVec2);
        addToModel(aLine2);

        // Performing removal of duplicate Vectors.
        HashSet<Transient> aRemovedEntities = replaceDuplicateVectors();

        // Check that nothing was removed.
        Assert.Empty(aRemovedEntities);
    }

    [Fact]
    public void Line()
    {
        // Creating Vectors.
        var aVec1 = addVector();
        var aVec2 = addVector();

        // Creating a cartesian point for the lines.
        var aPnt = addCartesianPoint(null, new Xyz(0.0, 0.0, 0.0));

        // Creating aLine containing the first Vector.
        var aLine1 = new StepLine();
        aLine1.Init(new TCollection_HAsciiString(), aPnt, aVec1);
        addToModel(aLine1);

        // Creating aLine containing the second Vector.
        var aLine2 = new StepLine();
        aLine2.Init(new TCollection_HAsciiString(), aPnt, aVec2);
        addToModel(aLine2);

        // Performing removal of duplicate Vectors.
        HashSet<Transient> aRemovedEntities = replaceDuplicateVectors();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aVec1) || aRemovedEntities.Contains(aVec2));
    }
}
