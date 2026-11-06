// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Tools.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>Static methods on bounding boxes: conversion from Bnd boxes to BVH boxes.</summary>
internal static class Tools
{
    /// <summary>The BVH box of a 2D box (throws if the box is void, as <c>Get</c> does).</summary>
    public static Box<double, NCollection_Vec2<double>> Bnd2BVH(in Box2d box)
    {
        box.Get(out double xMin, out double yMin, out double xMax, out double yMax);
        return new Box<double, NCollection_Vec2<double>>(new NCollection_Vec2<double>(xMin, yMin),
                                                             new NCollection_Vec2<double>(xMax, yMax));
    }

    /// <summary>The BVH box of a 3D box (throws if the box is void, as <c>Get</c> does).</summary>
    public static Box<double, NCollection_Vec3<double>> Bnd2BVH(in Box box)
    {
        box.Get(out double xMin, out double yMin, out double zMin, out double xMax, out double yMax, out double zMax);
        return new Box<double, NCollection_Vec3<double>>(new NCollection_Vec3<double>(xMin, yMin, zMin),
                                                             new NCollection_Vec3<double>(xMax, yMax, zMax));
    }
}
