// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Tools.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Static methods on bounding boxes: conversion from Bnd boxes to BVH boxes.</summary>
public static class Bnd_Tools
{
    /// <summary>The BVH box of a 2D box (throws if the box is void, as <c>Get</c> does).</summary>
    public static BVH_Box<double, NCollection_Vec2<double>> Bnd2BVH(in Bnd_Box2d theBox)
    {
        theBox.Get(out double aXMin, out double aYMin, out double aXMax, out double aYMax);
        return new BVH_Box<double, NCollection_Vec2<double>>(new NCollection_Vec2<double>(aXMin, aYMin),
                                                             new NCollection_Vec2<double>(aXMax, aYMax));
    }

    /// <summary>The BVH box of a 3D box (throws if the box is void, as <c>Get</c> does).</summary>
    public static BVH_Box<double, NCollection_Vec3<double>> Bnd2BVH(in Bnd_Box theBox)
    {
        theBox.Get(out double aXMin, out double aYMin, out double aZMin, out double aXMax, out double aYMax, out double aZMax);
        return new BVH_Box<double, NCollection_Vec3<double>>(new NCollection_Vec3<double>(aXMin, aYMin, aZMin),
                                                             new NCollection_Vec3<double>(aXMax, aYMax, aZMax));
    }
}
