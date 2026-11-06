// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// OCCT #1461, decision 1821: near a horn torus OCCT 8.0.1's writer picks a TOROIDAL_SURFACE or a SURFACE_OF_REVOLUTION by
/// the sign of noise, so congruent faces of one part are written as different things.
/// </summary>
public class MakeStepFaceTorusTests
{
    private static string StepText(double theMinorExcess)
    {
        var aTorus = new ToroidalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 10.0 + theMinorExcess);
        Shape aFace = new MakeFace(aTorus, 0.0, Math.PI / 2, 0.0, Math.PI / 2, 1e-7).Shape();
        var aWriter = new Writer();
        Assert.Equal(ReturnStatus.RetDone, aWriter.Transfer(aFace, ModelType.AsIs));
        using var aStream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, aWriter.WriteStream(aStream));
        return Encoding.UTF8.GetString(aStream.ToArray());
    }

    [Theory]
    [InlineData(5e-12)]
    [InlineData(-5e-12)]
    [InlineData(0.0)]
    public void Within_tolerance_of_a_horn_torus_the_surface_is_a_torus(double theMinorExcess)
    {
        string aText = StepText(theMinorExcess);
        Assert.Contains("TOROIDAL_SURFACE", aText, StringComparison.Ordinal);
        Assert.DoesNotContain("SURFACE_OF_REVOLUTION", aText, StringComparison.Ordinal);
    }

    [Fact]
    public void A_torus_that_truly_crosses_its_axis_is_a_revolution()
    {
        Assert.Contains("SURFACE_OF_REVOLUTION", StepText(1.0), StringComparison.Ordinal);
    }
}
