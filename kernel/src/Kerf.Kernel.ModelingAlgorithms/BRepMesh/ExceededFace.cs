// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Kerf: a face whose mesh a sphere's or a torus's guard (decisions 1929, 1953, 2056) left further from the surface than
/// the deflection it kept the face to, by the guard's own last measure (decision 26901).
/// </summary>
/// <param name="Index">The face's index in the shape's distinct faces (<see cref="MeshExtraction.Faces"/>).</param>
/// <param name="Face">The face.</param>
/// <param name="Limit">The deflection the face was kept to: the deflection asked, or at a relative deflection the face's own.</param>
/// <param name="Deviation">The largest distance from the surface the guard found, in model units.</param>
internal readonly record struct ExceededFace(int Index, Shape Face, double Limit, double Deviation);
