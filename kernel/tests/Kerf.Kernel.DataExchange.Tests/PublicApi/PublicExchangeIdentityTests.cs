// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Exchange.Stl.Api;
using Kerf.Engine.Exchange.ThreeMf;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Tools;
using Kerf.Exchange;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.PublicApi;

/// <summary>
/// The public exchange API writes the bytes the engine wrote the old way (unit 10L3): the modelling layer's glue meshed a
/// private copy of each shape in place and gave it to the engine's writers. STEP's identity is pinned by the modelling
/// layer's native references, which now write through <see cref="Step"/>.
/// </summary>
public class PublicExchangeIdentityTests
{
    private static IEnumerable<Shape> Shapes()
    {
        yield return new MakeBox(10, 20, 30).Shape();
        yield return new ApiMakeCylinder(5, 10).Shape();
        yield return new MakeSphere(7).Shape();
    }

    // The old way: a private copy meshed in place (relative deflection, in parallel), as Kerf.Modeling's MeshedCopy
    private static Shape MeshedCopy(in Shape shape, double linear, double angular)
        => new IncrementalMesh(new Copy(shape, false, false).Shape(), linear, true, angular, true).Shape();

    private static MeshOptions Options(double linear, double angular)
        => new() { LinearDeflection = linear, AngularDeflection = angular, Relative = true, Parallel = true };

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void STL_is_written_as_before(bool ascii)
    {
        foreach (Shape shape in Shapes())
        {
            var before = new MemoryStream();
            var writer = new ApiWriter();
            writer.SetASCIIMode(ascii);
            Assert.True(writer.Write(MeshedCopy(shape, 0.01, 0.2), before));

            var after = new MemoryStream();
            Assert.True(Stl.Write(Mesher.Mesh(shape, Options(0.01, 0.2)).Value, after, new StlWriteOptions { Ascii = ascii }).IsSuccess);
            Assert.Equal(before.ToArray(), after.ToArray());
        }
    }

    [Fact]
    public void GlTF_and_3MF_are_written_as_before()
    {
        foreach (Shape shape in Shapes())
        {
            Tree tree = new(new Instance(DocumentProduct.Part(MeshedCopy(shape, 0.01, 0.2).Located(new Placement()), "part"), shape.Placement));
            var glbBefore = new MemoryStream();
            Assert.True(new GltfTreeWriter(new GltfWriterOptions { InputLengthUnit = 0.001, Generator = "Kerf" }).WriteGlb(tree, glbBefore));
            var glbAfter = new MemoryStream();
            Assert.True(Gltf.Write(ModelNode.Part(shape, "part"), glbAfter, new GltfWriteOptions { Mesh = Options(0.01, 0.2) }).IsSuccess);
            Assert.Equal(glbBefore.ToArray(), glbAfter.ToArray());

            var mfBefore = new MemoryStream();
            Assert.True(new TreeWriter(new WriteOptions { InputLengthUnit = 0.001 }).Write(tree, mfBefore));
            var mfAfter = new MemoryStream();
            Assert.True(ThreeMf.Write(ModelNode.Part(shape, "part"), mfAfter, new ThreeMfWriteOptions { Mesh = Options(0.01, 0.2) }).IsSuccess);
            Assert.Equal(mfBefore.ToArray(), mfAfter.ToArray());
        }
    }

    [Fact]
    public void BREP_is_written_as_before()
    {
        foreach (Shape shape in Shapes())
        {
            var writer = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
            BrepTools.Write(shape, writer, true, false, Kerf.Engine.Exchange.Brep.FormatVersion.FormatVersion_CURRENT);
            var after = new MemoryStream();
            Assert.True(Brep.Write(shape, after).IsSuccess);
            Assert.Equal(Encoding.ASCII.GetBytes(writer.ToString()), after.ToArray());
        }
    }
}
