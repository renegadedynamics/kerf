// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Reflection;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The builder members the modelling layer called (Kerf unit 10B, where they replaced its UnsafeAccessors): the engine's
/// own since unit 10M1, which moved the layer onto the public API (decision 1565), and building a shape that stays open
/// until <see cref="TShape.FreezeGraph"/>.
/// </summary>
public class PublicBuilderTests
{
    [Theory]
    [InlineData(typeof(Builder), ".ctor")]
    [InlineData(typeof(Builder), "MakeWire")]
    [InlineData(typeof(Builder), "MakeShell")]
    [InlineData(typeof(Builder), "MakeSolid")]
    [InlineData(typeof(Builder), "MakeCompSolid")]
    [InlineData(typeof(Builder), "MakeCompound")]
    [InlineData(typeof(Builder), "Add")]
    [InlineData(typeof(Builder), "Remove")]
    [InlineData(typeof(BrepBuilder), ".ctor")]
    [InlineData(typeof(BrepBuilder), "MakeFace")]
    [InlineData(typeof(TShape), "FreezeGraph")]
    public void TheModellingLayersOldMembersAreTheEngines(Type theType, string theName)
    {
        // Internal, and the modelling layer no longer reaches them (the kernel gives it no InternalsVisibleTo)
        Assert.False(theType.IsPublic);
        Assert.DoesNotContain(theType.Assembly.GetCustomAttributes<System.Runtime.CompilerServices.InternalsVisibleToAttribute>(),
                              a => a.AssemblyName == "Kerf.Modeling");
        const BindingFlags aPublic = BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly;
        MemberInfo[] aMembers = theName == ".ctor" ? theType.GetConstructors(aPublic) : theType.GetMember(theName, aPublic);
        Assert.NotEmpty(aMembers);
    }

    [Fact]
    public void ABuiltCompoundStaysOpenUntilFrozen()
    {
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        aBuilder.Add(aCompound, aBox);
        aBuilder.Add(aCompound, new MakeBox(4, 5, 6).Shape());
        aBuilder.Remove(aCompound, aBox);
        Assert.False(aCompound.TShape()!.IsFrozen);

        aCompound.TShape()!.FreezeGraph();

        Assert.True(aCompound.TShape()!.IsFrozen);
        Assert.True(aCompound.Locked());
        Assert.Equal(1, aCompound.ChildCount);
        Assert.ThrowsAny<FailureException>(() => aBuilder.Add(aCompound, aBox));
    }

    [Fact]
    public void AFaceOfATriangulation()
    {
        var aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        aTri.SetNode(2, new Point3D(1, 0, 0));
        aTri.SetNode(3, new Point3D(0, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        Face aFace = new();
        new BrepBuilder().MakeFace(ref aFace, aTri);
        aFace.TShape()!.FreezeGraph();

        Assert.Equal(ShapeType.Face, aFace.Type);
        Assert.False(Tool.IsGeometric(aFace));
        Assert.Same(aTri, Tool.Triangulation(aFace, out _));
        Assert.True(aFace.TShape()!.IsFrozen);
    }
}
