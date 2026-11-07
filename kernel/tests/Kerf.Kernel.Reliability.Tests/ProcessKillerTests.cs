// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.Reliability.Tests;

/// <summary>What ended the process, and what it does now (docs/reliability.md; decisions 21101 and 21102).</summary>
public class ProcessKillerTests
{
    [Theory]
    [InlineData("compound-over-the-limit")]
    [InlineData("compound-far-over-the-limit")]
    [InlineData("tree-over-the-limit")]
    [InlineData("tree-far-over-the-limit")]
    public void Nesting_too_deep_is_refused(string scenario) => Assert.Equal("refused", ChildProcess.Run(scenario));

    [Fact]
    public void Nesting_up_to_the_limit_is_allowed() => Assert.Equal("built", ChildProcess.Run("compound-at-the-limit"));

    [Theory]
    [InlineData("mesh")]
    [InlineData("transform")]
    [InlineData("transform-copy")]
    [InlineData("mirror")]
    [InlineData("scale")]
    [InlineData("fuse")]
    [InlineData("cut")]
    [InlineData("fix")]
    [InlineData("unify")]
    [InlineData("validate")]
    [InlineData("brep")]
    [InlineData("step")]
    [InlineData("tree-step")]
    [InlineData("tree-gltf")]
    [InlineData("tree-3mf")]
    public void An_operation_on_the_deepest_shape_finishes_on_a_1_MB_stack(string scenario) => Assert.Equal("ok", ChildProcess.Run(scenario));

    [Fact]
    public void A_walk_of_the_deepest_shape_finds_what_is_in_it()
    {
        Assert.Equal("6", ChildProcess.Run("faces"));
        Assert.Equal("1.000000", ChildProcess.Run("volume"));
    }

    [Theory]
    [InlineData("step-file-too-deep", "exchange.limit-exceeded")]
    [InlineData("gltf-file-too-deep", "exchange.limit-exceeded")]
    [InlineData("brep-file-too-deep", "exchange.malformed")]
    public void A_file_nested_too_deep_is_a_failure(string scenario, string code) => Assert.Equal(code, ChildProcess.Run(scenario));

    // The reader treats it as any entity it can't translate: left out, and the file reads
    [Fact]
    public void A_file_of_curves_nested_too_deep_leaves_them_out() => Assert.Equal("ok", ChildProcess.Run("step-curves-too-deep"));

    [Fact]
    public void A_compound_holding_another_twice_at_each_level_is_refused() => Assert.Equal("refused", ChildProcess.Run("shared-twice-each-level", 60));

    [Fact]
    public void A_shape_can_be_moved_a_hundred_thousand_times() => Assert.Equal("1.000000", ChildProcess.Run("moved-many-times", 60));

    [Fact]
    public void Long_placements_multiply_and_raise_to_powers() => Assert.Equal("100000.000 100000.000 100000.000", ChildProcess.Run("placement-chain"));
}
