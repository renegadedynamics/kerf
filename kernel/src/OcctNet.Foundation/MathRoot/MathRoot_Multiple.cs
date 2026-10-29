// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Multiple.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Algorithms for finding all roots of a function in a given range.</summary>
public static partial class MathRoot
{
    /// <summary>
    /// Finds all real roots of a function within [theLower, theUpper]: uniform sampling to detect sign changes,
    /// each refined by Brent's method.
    /// </summary>
    public static MultipleResult FindAllRoots<Function>(Function theFunc, double theLower, double theUpper, MultipleConfig? theConfig = null)
        where Function : MathUtils.IScalarFunction
    {
        theConfig ??= new MultipleConfig();
        int aNbSamples = Max(2 * theConfig.NbSamples, 20);
        math_Vector aSamples = new(0, aNbSamples);

        MultipleSampleValueFn<Function> aSampleFn = new(theFunc, aSamples, theConfig.Offset);
        MultipleGetValueFn aGetValue = new(aSamples);
        MultipleBrentValueWrapper<Function> aWrapper = new(theFunc, theConfig.Offset);
        MultipleGetRootValueFn<Function> aGetRootValue = new(theFunc);

        return FindAllRootsImpl(theLower,
                                theUpper,
                                theConfig,
                                aSampleFn,
                                aGetValue,
                                aWrapper,
                                aGetRootValue,
                                new MultipleNoExtraHandler());
    }

    /// <summary>
    /// Finds all real roots of a function with derivative within [theLower, theUpper], including extrema where the
    /// function touches zero without crossing.
    /// </summary>
    public static MultipleResult FindAllRootsWithDerivative<Function>(Function theFunc, double theLower, double theUpper, MultipleConfig? theConfig = null)
        where Function : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        theConfig ??= new MultipleConfig();
        return FindAllRootsWithDerivativeImpl(theFunc, theLower, theUpper, theConfig);
    }

    /// <summary>FindAllRoots with the default configuration and theNbSamples sample points.</summary>
    public static MultipleResult FindAllRoots<Function>(Function theFunc, double theLower, double theUpper, int theNbSamples)
        where Function : MathUtils.IScalarFunction
    {
        MultipleConfig aConfig = new();
        aConfig.NbSamples = theNbSamples;
        return FindAllRoots(theFunc, theLower, theUpper, aConfig);
    }
}
