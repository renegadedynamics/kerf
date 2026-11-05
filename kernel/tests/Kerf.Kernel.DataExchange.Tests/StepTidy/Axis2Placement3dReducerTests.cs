// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_Axis2Placement3dReducer_Test.cxx
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

public class Axis2Placement3dReducerTests : BaseTestFixture
{
    // Perform removal of duplicate entities.
    HashSet<Transient> replaceDuplicateAxis2Placement3ds()
    {
        var aReducer = new Axis2Placement3dReducer(myWS);
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
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d("Axis1");
        var anAxis2 = addAxis2Placement3d("Axis2");

        // Creating a plane containing the first Axis2Placement3d.
        var aPlane1 = new StepPlane();
        aPlane1.Init(new TCollection_HAsciiString(), anAxis1);
        addToModel(aPlane1);

        // Creating a plane containing the second Axis2Placement3d.
        var aPlane2 = new StepPlane();
        aPlane2.Init(new TCollection_HAsciiString(), anAxis2);
        addToModel(aPlane2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that nothing was removed.
        Assert.Empty(aRemovedEntities);
    }

    [Fact]
    public void Plane()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a plane containing the first Axis2Placement3d.
        var aPlane1 = new StepPlane();
        aPlane1.Init(new TCollection_HAsciiString(), anAxis1);
        addToModel(aPlane1);

        // Creating a plane containing the second Axis2Placement3d.
        var aPlane2 = new StepPlane();
        aPlane2.Init(new TCollection_HAsciiString(), anAxis2);
        addToModel(aPlane2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void ItemDefinedTransformation()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();
        var anAxis3 = addAxis2Placement3d(null, new Xyz(1.0, 1.0, 1.0));

        // Creating ItemDefinedTransformation containing the first Axis2Placement3d.
        var aItem1 = new ItemDefinedTransformation();
        aItem1.Init(new TCollection_HAsciiString(), new TCollection_HAsciiString(), anAxis1, anAxis3);
        addToModel(aItem1);

        // Creating ItemDefinedTransformation containing the second Axis2Placement3d.
        var aItem2 = new ItemDefinedTransformation();
        aItem1.Init(new TCollection_HAsciiString(), new TCollection_HAsciiString(), anAxis2, anAxis3);
        addToModel(aItem2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void CylindricalSurface()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a cylindrical surface containing the first Axis2Placement3d.
        var aCylindricalSurface1 = new StepCylindricalSurface();
        aCylindricalSurface1.Init(new TCollection_HAsciiString(), anAxis1, 1.0);
        addToModel(aCylindricalSurface1);

        // Creating a cylindrical surface containing the second Axis2Placement3d.
        var aCylindricalSurface2 = new StepCylindricalSurface();
        aCylindricalSurface2.Init(new TCollection_HAsciiString(), anAxis2, 1.0);
        addToModel(aCylindricalSurface2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void ShapeRepresentation()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a shape representation containing the first Axis2Placement3d.
        var aItems1 =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aItems1.SetValue(1, anAxis1);
        var aShapeRepresentation1 =
            new StepShapeRepresentation();
        aShapeRepresentation1.Init(new TCollection_HAsciiString(),
                                                                aItems1,
                                                                new RepresentationContext());
        addToModel(aShapeRepresentation1);

        // Creating a shape representation containing the second Axis2Placement3d.
        var aItems2 =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aItems2.SetValue(1, anAxis2);
        var aShapeRepresentation2 =
            new StepShapeRepresentation();
        aShapeRepresentation2.Init(new TCollection_HAsciiString(),
                                                                aItems2,
                                                                new RepresentationContext());
        addToModel(aShapeRepresentation2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void ConstructiveGeometryRepresentation()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a constructive geometry representation containing the first Axis2Placement3d.
        var aItems1 =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aItems1.SetValue(1, anAxis1);
        var aConstructiveGeometryRepresentation1 =
            new ConstructiveGeometryRepresentation();
        aConstructiveGeometryRepresentation1.Init(new TCollection_HAsciiString(),
                                                                                              aItems1,
                                                                                              new RepresentationContext());
        addToModel(aConstructiveGeometryRepresentation1);

        // Creating a constructive geometry representation containing the second Axis2Placement3d.
        var aItems2 =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aItems2.SetValue(1, anAxis2);
        var aConstructiveGeometryRepresentation2 =
            new ConstructiveGeometryRepresentation();
        aConstructiveGeometryRepresentation2.Init(new TCollection_HAsciiString(),
                                                                                              aItems2,
                                                                                              new RepresentationContext());
        addToModel(aConstructiveGeometryRepresentation2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void Circle()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a circle containing the first Axis2Placement3d.
        var aSelector1 = new StepAxis2Placement();
        aSelector1.SetValue(anAxis1);
        var aCircle1 = new StepCircle();
        aCircle1.Init(new TCollection_HAsciiString(), aSelector1, 1.0);
        addToModel(aCircle1);

        // Creating a circle containing the second Axis2Placement3d.
        var aSelector2 = new StepAxis2Placement();
        aSelector2.SetValue(anAxis2);
        var aCircle2 = new StepCircle();
        aCircle2.Init(new TCollection_HAsciiString(), aSelector2, 1.0);
        addToModel(aCircle2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void PresentationLayerAssignment()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a presentation layer assignment containing the first Axis2Placement3d.
        var aAssignedItems1 =
            new NCollection_HArray1<LayeredItem>(1, 1);
        var aLayeredItem1 = new LayeredItem();
        aLayeredItem1.SetValue(anAxis1);
        aAssignedItems1.SetValue(1, aLayeredItem1);
        var aPresentationLayerAssignment1 =
            new PresentationLayerAssignment();
        aPresentationLayerAssignment1.Init(new TCollection_HAsciiString(),
                                                                                new TCollection_HAsciiString(),
                                                                                aAssignedItems1);
        addToModel(aPresentationLayerAssignment1);

        // Creating a presentation layer assignment containing the second Axis2Placement3d.
        var aAssignedItems2 =
            new NCollection_HArray1<LayeredItem>(1, 1);
        var aLayeredItem2 = new LayeredItem();
        aLayeredItem2.SetValue(anAxis2);
        aAssignedItems2.SetValue(1, aLayeredItem2);
        var aPresentationLayerAssignment2 =
            new PresentationLayerAssignment();
        aPresentationLayerAssignment2.Init(new TCollection_HAsciiString(),
                                                                                new TCollection_HAsciiString(),
                                                                                aAssignedItems2);
        addToModel(aPresentationLayerAssignment2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void StyledItem()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a styled item containing the first Axis2Placement3d.
        var aStiledItem1 = new StyledItem();
        aStiledItem1.Init(
            new TCollection_HAsciiString(),
            new NCollection_HArray1<PresentationStyleAssignment>(1, 1),
            anAxis1);
        addToModel(aStiledItem1);

        // Creating a styled item containing the second Axis2Placement3d.
        var aStiledItem2 = new StyledItem();
        aStiledItem2.Init(
            new TCollection_HAsciiString(),
            new NCollection_HArray1<PresentationStyleAssignment>(1, 1),
            anAxis2);
        addToModel(aStiledItem2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void Ellipse()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating an ellipse containing the first Axis2Placement3d.
        var aSelector1 = new StepAxis2Placement();
        aSelector1.SetValue(anAxis1);
        var aEllipse1 = new StepEllipse();
        aEllipse1.Init(new TCollection_HAsciiString(), aSelector1, 1.0, 2.0);
        addToModel(aEllipse1);

        // Creating an ellipse containing the second Axis2Placement3d.
        var aSelector2 = new StepAxis2Placement();
        aSelector2.SetValue(anAxis2);
        var aEllipse2 = new StepEllipse();
        aEllipse2.Init(new TCollection_HAsciiString(), aSelector2, 1.0, 2.0);
        addToModel(aEllipse2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void ConicalSurface()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a conical surface containing the first Axis2Placement3d.
        var aConicalSurface1 = new StepConicalSurface();
        aConicalSurface1.Init(new TCollection_HAsciiString(), anAxis1, 1.0, 1.0);
        addToModel(aConicalSurface1);

        // Creating a conical surface containing the second Axis2Placement3d.
        var aConicalSurface2 = new StepConicalSurface();
        aConicalSurface2.Init(new TCollection_HAsciiString(), anAxis2, 1.0, 1.0);
        addToModel(aConicalSurface2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void ToroidalSurface()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a toroidal surface containing the first Axis2Placement3d.
        var aToroidalSurface1 = new StepToroidalSurface();
        aToroidalSurface1.Init(new TCollection_HAsciiString(), anAxis1, 1.0, 1.0);
        addToModel(aToroidalSurface1);

        // Creating a toroidal surface containing the second Axis2Placement3d.
        var aToroidalSurface2 = new StepToroidalSurface();
        aToroidalSurface2.Init(new TCollection_HAsciiString(), anAxis2, 1.0, 1.0);
        addToModel(aToroidalSurface2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void AdvancedBrepShapeRepresentation()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a shape representation containing the first Axis2Placement3d.
        var aItems1 =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aItems1.SetValue(1, anAxis1);
        var aShapeRepresentation1 =
            new StepAdvancedBrepShapeRepresentation();
        aShapeRepresentation1.Init(new TCollection_HAsciiString(),
                                                                aItems1,
                                                                new RepresentationContext());
        addToModel(aShapeRepresentation1);

        // Creating a shape representation containing the second Axis2Placement3d.
        var aItems2 =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aItems2.SetValue(1, anAxis2);
        var aShapeRepresentation2 =
            new StepAdvancedBrepShapeRepresentation();
        aShapeRepresentation2.Init(new TCollection_HAsciiString(),
                                                                aItems2,
                                                                new RepresentationContext());
        addToModel(aShapeRepresentation2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }

    [Fact]
    public void SphericalSurface()
    {
        // Creating Axis2Placement3ds.
        var anAxis1 = addAxis2Placement3d();
        var anAxis2 = addAxis2Placement3d();

        // Creating a spherical surface containing the first Axis2Placement3d.
        var aSphericalSurface1 = new StepSphericalSurface();
        aSphericalSurface1.Init(new TCollection_HAsciiString(), anAxis1, 1.0);
        addToModel(aSphericalSurface1);

        // Creating a spherical surface containing the second Axis2Placement3d.
        var aSphericalSurface2 = new StepSphericalSurface();
        aSphericalSurface2.Init(new TCollection_HAsciiString(), anAxis2, 1.0);
        addToModel(aSphericalSurface2);

        // Performing removal of duplicate Axis2Placement3ds.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateAxis2Placement3ds();

        // Check that one Axis2Placement3d was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(anAxis1) || aRemovedEntities.Contains(anAxis2));
    }
}
