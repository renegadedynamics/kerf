// Port of: src/FoundationClasses/TKMath/GTests/MathUtils_Bracket_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class MathUtils_BracketTest
{
    private sealed class QuadraticMinimum : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = (theX - 2.0) * (theX - 2.0);
            return true;
        }
    }

    private sealed class QuadraticWithPrecomputedEndpoints : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = 0.0;
            if (Math.Abs(theX - 0.0) < 1.0e-15 || Math.Abs(theX - 1.0) < 1.0e-15)
            {
                return false;
            }
            theF = (theX - 2.0) * (theX - 2.0);
            return true;
        }
    }

    [Fact]
    public void BracketMinimum_WithLimits_Succeeds()
    {
        QuadraticMinimum aFunc = new();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.MinBracketOptions anOptions = new();
        anOptions.MaxIterations = 50;
        anOptions.UseLimits = true;
        anOptions.LeftLimit = 0.0;
        anOptions.RightLimit = 5.0;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.MinBracketResult aResult = global::Kerf.Engine.Numerics.Utilities.MathUtils.BracketMinimum(aFunc, 0.0, 1.0, anOptions);
        Assert.True(aResult.IsValid);
        Assert.True(aResult.A >= anOptions.LeftLimit);
        Assert.True(aResult.C <= anOptions.RightLimit);
        Assert.True(aResult.Fb < aResult.Fa);
        Assert.True(aResult.Fb < aResult.Fc);
    }

    [Fact]
    public void BracketMinimum_WithRestrictiveLimits_Fails()
    {
        QuadraticMinimum aFunc = new();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.MinBracketOptions anOptions = new();
        anOptions.MaxIterations = 50;
        anOptions.UseLimits = true;
        anOptions.LeftLimit = 0.0;
        anOptions.RightLimit = 1.0;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.MinBracketResult aResult = global::Kerf.Engine.Numerics.Utilities.MathUtils.BracketMinimum(aFunc, 0.0, 0.5, anOptions);
        Assert.False(aResult.IsValid);
    }

    [Fact]
    public void BracketMinimum_UsesPrecomputedEndpointValues()
    {
        QuadraticWithPrecomputedEndpoints aFunc = new();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.MinBracketOptions anOptions = new();
        anOptions.MaxIterations = 50;
        anOptions.HasFA = true;
        anOptions.HasFB = true;
        anOptions.FA = 4.0;
        anOptions.FB = 1.0;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.MinBracketResult aResult = global::Kerf.Engine.Numerics.Utilities.MathUtils.BracketMinimum(aFunc, 0.0, 1.0, anOptions);
        Assert.True(aResult.IsValid);
        Assert.True(aResult.Fb < aResult.Fa);
        Assert.True(aResult.Fb < aResult.Fc);
    }
}
