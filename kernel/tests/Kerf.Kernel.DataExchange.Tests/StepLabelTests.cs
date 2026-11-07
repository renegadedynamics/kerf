// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Construction;
using Kerf.Exchange;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>What isn't ASCII in a STEP file goes as the format's directives (OCCT #1686; decision 1920).</summary>
public class StepLabelTests
{
    [Theory]
    [InlineData("plain")]
    [InlineData("Øresund")]
    [InlineData("測試 part")]
    [InlineData("a 😀 b")]
    [InlineData("it's Ø")]
    public void A_name_comes_back_and_the_file_is_ascii(string name)
    {
        var file = new MemoryStream();
        Assert.True(Step.Write(ModelNode.Part(Solid.Box(1, 1, 1).Value.Shape, name), file).IsSuccess);
        Assert.DoesNotContain(file.ToArray(), b => b >= 0x80);

        file.Position = 0;
        Result<ModelNode> read = Step.Read(file);

        Assert.True(read.IsSuccess);
        Assert.Equal(name, read.Value.Name);
    }

    [Fact]
    public void Directives_are_as_the_format_has_them()
    {
        var file = new MemoryStream();
        Assert.True(Step.Write(ModelNode.Part(Solid.Box(1, 1, 1).Value.Shape, "Ø😀"), file).IsSuccess);
        string text = Encoding.ASCII.GetString(file.ToArray());

        Assert.Contains("\\X2\\00D8\\X0\\\\X4\\0001F600\\X0\\", text, StringComparison.Ordinal);
    }
}
