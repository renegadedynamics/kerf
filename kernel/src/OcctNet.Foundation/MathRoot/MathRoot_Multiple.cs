// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Multiple.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Roots;

/// <summary>Algorithms for finding all roots of a function in a given range.</summary>
public static partial class MathRoot
{
    /// <summary>
    /// Finds all real roots of a function within [theLower, theUpper]: uniform sampling to detect sign changes,
    /// each refined by Brent's method.
    /// </summary>
    public static MultipleResult FindAllRoots<Function>(Function func, double lower, double upper, MultipleConfig? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        config ??= new MultipleConfig();
        int nbSamples = Max(2 * config.NbSamples, 20);
        math_Vector samples = new(0, nbSamples);

        MultipleSampleValueFn<Function> sampleFn = new(func, samples, config.Offset);
        MultipleGetValueFn getValue = new(samples);
        MultipleBrentValueWrapper<Function> wrapper = new(func, config.Offset);
        MultipleGetRootValueFn<Function> getRootValue = new(func);

        return FindAllRootsImpl(lower,
                                upper,
                                config,
                                sampleFn,
                                getValue,
                                wrapper,
                                getRootValue,
                                new MultipleNoExtraHandler());
    }

    /// <summary>
    /// Finds all real roots of a function with derivative within [theLower, theUpper], including extrema where the
    /// function touches zero without crossing.
    /// </summary>
    public static MultipleResult FindAllRootsWithDerivative<Function>(Function func, double lower, double upper, MultipleConfig? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        config ??= new MultipleConfig();
        return FindAllRootsWithDerivativeImpl(func, lower, upper, config);
    }

    /// <summary>FindAllRoots with the default configuration and theNbSamples sample points.</summary>
    public static MultipleResult FindAllRoots<Function>(Function func, double lower, double upper, int nbSamples)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        MultipleConfig config = new();
        config.NbSamples = nbSamples;
        return FindAllRoots(func, lower, upper, config);
    }
}
