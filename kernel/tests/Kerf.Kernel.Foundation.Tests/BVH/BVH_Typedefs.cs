// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// OCCT's BVH typedefs, for the ported tests (C# has no generic typedefs in the library itself).

global using BVH_Vec2i = Kerf.Engine.NCollection_Vec2<int>;
global using BVH_Vec3i = Kerf.Engine.NCollection_Vec3<int>;
global using BVH_Vec4i = Kerf.Engine.NCollection_Vec4<int>;
global using BVH_Vec2f = Kerf.Engine.NCollection_Vec2<float>;
global using BVH_Vec3f = Kerf.Engine.NCollection_Vec3<float>;
global using BVH_Vec4f = Kerf.Engine.NCollection_Vec4<float>;
global using BVH_Vec2d = Kerf.Engine.NCollection_Vec2<double>;
global using BVH_Vec3d = Kerf.Engine.NCollection_Vec3<double>;
global using BVH_Vec4d = Kerf.Engine.NCollection_Vec4<double>;
global using BVH_Mat4f = Kerf.Engine.NCollection_Mat4<float>;
global using BVH_Mat4d = Kerf.Engine.NCollection_Mat4<double>;
global using BVH_Array4i = Kerf.Engine.NCollection_LinearVector<Kerf.Engine.NCollection_Vec4<int>>;
