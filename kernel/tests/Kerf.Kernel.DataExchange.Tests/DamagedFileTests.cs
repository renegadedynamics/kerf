// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Construction;
using Kerf.Exchange;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>A number damaged in a file: a failure, or something that can be stopped, and never a hang (decisions 22502 and 22503).</summary>
public class DamagedFileTests
{
    private static string BrepOf(Shape shape)
    {
        var text = new MemoryStream();
        Assert.True(Brep.Write(shape, text).IsSuccess);
        return Encoding.ASCII.GetString(text.ToArray());
    }

    private static Result<Shape> Read(string brep) => Brep.Read(new MemoryStream(Encoding.ASCII.GetBytes(brep)));

    [Theory]
    [InlineData("nan")]
    [InlineData("inf")]
    [InlineData("-Infinity")]
    public void A_brep_with_a_number_that_is_not_finite_is_malformed(string number)
    {
        string brep = BrepOf(Solid.Box(1, 1, 1).Value.Shape);
        int at = brep.IndexOf('\n', brep.IndexOf("\nVe", StringComparison.Ordinal) + 1) + 1; // the vertex's tolerance
        string damaged = brep[..at] + number + brep[brep.IndexOf('\n', at)..];

        Result<Shape> read = Read(damaged);

        Assert.False(read.IsSuccess);
        Assert.Equal("exchange.malformed", read.Error!.Code.Value);
    }

    [Fact]
    public async Task A_mesh_of_a_range_damaged_to_1e300_can_be_stopped()
    {
        string brep = BrepOf(Solid.Cylinder(5, 10).Value.Shape);
        string damaged = brep.Replace("6.28318530717959", "1e300", StringComparison.Ordinal);
        Assert.NotEqual(brep, damaged);
        Result<Shape> read = Read(damaged);
        if (!read.IsSuccess)
        {
            return;
        }

        using var source = new CancellationTokenSource(TimeSpan.FromSeconds(2));
        Task task = Task.Run(() =>
        {
            try
            {
                _ = read.Value.Mesh(new MeshOptions { Parallel = false }, cancellationToken: source.Token);
            }
            catch (OperationCanceledException)
            {
            }
        });
        await task.WaitAsync(TimeSpan.FromSeconds(60));
    }
}
