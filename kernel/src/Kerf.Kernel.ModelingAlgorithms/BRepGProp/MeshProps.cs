// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshProps.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The global properties of a surface mesh, as a surface (<see cref="BRepGProp_MeshObjType.Sinert"/>) or as the piece
/// of volume between the mesh and the reference point (<see cref="BRepGProp_MeshObjType.Vinert"/>).
/// </summary>
public class MeshProps : Properties
{
    /// <summary>The types of geometric objects.</summary>
    public enum BRepGProp_MeshObjType
    {
        /// <summary>The 3D region delimited by the reference point and the surface mesh.</summary>
        Vinert = 0,

        /// <summary>The surface mesh in 3D space.</summary>
        Sinert,
    }

    private readonly BRepGProp_MeshObjType _type; //!< Type of geometric object

    /// <summary>A system for meshes of the given type.</summary>
    public MeshProps(BRepGProp_MeshObjType type) => _type = type;

    /// <summary>Sets the point relative to which the calculation is done.</summary>
    public void SetLocation(in Point3D location) => _loc = location;

    /// <summary>
    /// Computes the global properties of the mesh placed by <paramref name="loc"/> (Gauss cubature over the
    /// triangles), its triangles reversed if <paramref name="ori"/> is REVERSED.
    /// </summary>
    public void Perform(Triangulation? mesh, in Placement loc, Orientation ori)
    {
        if (mesh is null || mesh.NodeCount == 0 || mesh.TriangleCount == 0)
        {
            return;
        }

        if (loc.IsIdentity)
        {
            Perform(mesh, ori);
        }
        else
        {
            Transform3D tr = loc.Transformation();
            //
            bool isToCopy = tr.ScaleFactor() * tr.HVectorialPart().Determinant() < 0.0
                            || Math.Abs(Math.Abs(tr.ScaleFactor()) - 1.0) > Constants.Resolution();
            if (isToCopy)
            {
                var copy = new Triangulation(mesh.NodeCount, mesh.TriangleCount, false);
                for (int i = 1; i <= mesh.NodeCount; ++i)
                {
                    copy.SetNode(i, mesh.Node(i).Transformed(tr));
                }

                for (int i = 1; i <= mesh.TriangleCount; ++i)
                {
                    copy.SetTriangle(i, mesh.Triangle(i));
                }

                Perform(copy, ori);
                return;
            }

            //
            Transform3D trInv = tr.Inverted();
            Point3D loc_save = _loc;
            _loc.Transform(trInv);
            Perform(mesh, ori);
            // Computes the inertia tensor at mesh gravity center
            Matrix3x3 hMat, inertia0;
            Point3D g0 = _g;
            _g.SetXYZ(_g.XYZ() + _loc.XYZ());
            if (g0.XYZ().Modulus() > Constants.Resolution())
            {
                global::Kerf.Engine.Measurement.GProp.HOperator(_g, _loc, _dim, out hMat);
                inertia0 = _inertia - hMat;
            }
            else
            {
                inertia0 = _inertia;
            }

            // Transform inertia tensor for rotation
            Matrix3x3 hVec = trInv.HVectorialPart();
            Matrix3x3 hVecT = hVec.Transposed();
            hVecT.Multiply(inertia0);
            inertia0 = hVecT.Multiplied(hVec);
            // Put gravity center in true position of mesh
            _g.Transform(tr);
            g0 = _g;
            _g.SetXYZ(_g.XYZ() - loc_save.XYZ());
            _loc = loc_save;
            //
            // Computes the inertia tensor for loc
            global::Kerf.Engine.Measurement.GProp.HOperator(g0, _loc, _dim, out hMat);
            _inertia = inertia0 + hMat;
        }
    }

    /// <summary>
    /// Computes the global properties of the mesh (Gauss cubature over the triangles), its triangles reversed if
    /// <paramref name="ori"/> is REVERSED.
    /// </summary>
    public void Perform(Triangulation? mesh, Orientation ori)
    {
        if (mesh is null || mesh.NodeCount == 0 || mesh.TriangleCount == 0)
        {
            return;
        }

        //
        // Gauss points for barycentriche coordinates
        ReadOnlySpan<double> gPtsWg =
        [
            1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0, /*3 points*/
            2.0 / 3.0, 1.0 / 6.0, 1.0 / 6.0,
            1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0,
        ];
        //
        int nbGaussPoints = 3;

        // Array to store global properties
        Span<double> gProps = stackalloc double[10];
        // aGProps[0] = Volume
        //  Static moments of inertia.
        // aGProps[1] = Ix, aGProps[2] = Iy, aGProps[3] = Iz
        // Matrix of moments of inertia.
        // aGProps[4] = Ixx, aGProps[5] = Iyy, aGProps[6] = Izz,
        // aGProps[7] = Ixy, aGProps[8] = Ixz, aGProps[9] = Iyz,

        bool isVolume = _type == BRepGProp_MeshObjType.Vinert;
        int n1, n2, n3; // node indices
        for (int i = 1; i <= mesh.TriangleCount; ++i)
        {
            Triangle1 tri = mesh.Triangle(i);
            tri.Get(out n1, out n2, out n3);
            if (ori == Reversed)
            {
                (n2, n3) = (n3, n2);
            }

            // Calculate properties of a pyramid built on face and apex
            Point3D p1 = mesh.Node(n1);
            Point3D p2 = mesh.Node(n2);
            Point3D p3 = mesh.Node(n3);
            CalculateProps(p1, p2, p3, _loc, isVolume, gProps, nbGaussPoints, gPtsWg);
        }

        _dim = gProps[0];
        if (Math.Abs(_dim) >= 1.0e-20) // To be consistent with GProp_GProps
        {
            _g.SetX(gProps[1] / _dim);
            _g.SetY(gProps[2] / _dim);
            _g.SetZ(gProps[3] / _dim);
        }
        else
        {
            _g.SetX(gProps[1]);
            _g.SetY(gProps[2]);
            _g.SetZ(gProps[3]);
        }

        _inertia.SetValue(1, 1, gProps[4]);
        _inertia.SetValue(1, 2, -gProps[7]);
        _inertia.SetValue(1, 3, -gProps[8]);
        _inertia.SetValue(2, 1, -gProps[7]);
        _inertia.SetValue(2, 2, gProps[5]);
        _inertia.SetValue(2, 3, -gProps[9]);
        _inertia.SetValue(3, 1, -gProps[8]);
        _inertia.SetValue(3, 2, -gProps[9]);
        _inertia.SetValue(3, 3, gProps[6]);
    }

    /// <summary>
    /// Adds to <paramref name="gProps"/> the global properties of the triangle {p1, p2, p3} relative to
    /// <paramref name="apex"/>: those of the pyramid on it with that apex if <paramref name="isVolume"/>, else the
    /// surface ones. <paramref name="gaussPnts"/> holds, for each of the <paramref name="nbGaussPoints"/> points, two
    /// barycentric coordinates and a weight.
    /// </summary>
    /// <remarks>
    /// <paramref name="gProps"/> holds the mass, the static moments Ix, Iy, Iz, then Ixx, Iyy, Izz, Ixy, Ixz, Iyz.
    /// </remarks>
    public static void CalculateProps(in Point3D p1,
                                      in Point3D p2,
                                      in Point3D p3,
                                      in Point3D apex,
                                      bool isVolume,
                                      Span<double> gProps,
                                      int nbGaussPoints,
                                      ReadOnlySpan<double> gaussPnts)
    {
        // GProps[0] = Volume
        //  Static moments of inertia.
        // GProps[1] = Ix, GProps[2] = Iy, GProps[3] = Iz
        // Matrix of moments of inertia.
        // GProps[4] = Ixx, GProps[5] = Iyy, GProps[6] = Izz,
        // GProps[7] = Ixy, aGProps[8] = Ixz, GProps[9] = Iyz,
        //

        // Define plane and coordinates of triangle nodes on plane
        var v12 = new Vector3D(p2, p1);
        var v23 = new Vector3D(p3, p2);
        Vector3D norm = v12 ^ v23;
        double det = norm.Magnitude();
        if (det <= Constants.Resolution())
        {
            return;
        }

        Xyz center = (p1.XYZ() + p2.XYZ() + p3.XYZ()) / 3.0;
        var pc = new Point3D(center);
        var dn = new Direction3D(norm);
        var posPln = new CoordinateSystem(pc, dn);
        // Coordinates of nodes on plane
        double x1, y1, x2, y2, x3, y3;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPln, p1, out x1, out y1);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPln, p2, out x2, out y2);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPln, p3, out x3, out y3);
        //
        double l1, l2; // barycentriche coordinates
        double x, y, z;
        double w; // weight
        int i;
        for (i = 0; i < nbGaussPoints; ++i)
        {
            int ind = 3 * i;
            l1 = gaussPnts[ind];
            l2 = gaussPnts[ind + 1];
            w = gaussPnts[ind + 2];
            w *= det;
            x = l1 * (x1 - x3) + l2 * (x2 - x3) + x3;
            y = l1 * (y1 - y3) + l2 * (y2 - y3) + y3;
            Point3D p = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(x, y, posPln);
            x = p.X() - apex.X();
            y = p.Y() - apex.Y();
            z = p.Z() - apex.Z();
            //
            if (isVolume)
            {
                double xn = dn.X() * w;
                double yn = dn.Y() * w;
                double zn = dn.Z() * w;
                double dv = x * xn + y * yn + z * zn;
                CalculateElVProps(x, y, z, dv, gProps);
            }
            else
            {
                double ds = w;
                CalculateElSProps(x, y, z, ds, gProps);
            }
        }
    }

    /// <summary>The type of mesh object.</summary>
    public BRepGProp_MeshObjType GetMeshObjType() => _type;

    // Calculate one Gauss point for surface properties of triangle p1, p2, p3 relatively point Apex
    private static void CalculateElSProps(double x, double y, double z, double ds, Span<double> gProps)
    {
        // GProps[0] = Volume
        //  Static moments of inertia.
        // GProps[1] = Ix, GProps[2] = Iy, GProps[3] = Iz
        // Matrix of moments of inertia.
        // GProps[4] = Ixx, GProps[5] = Iyy, GProps[6] = Izz,
        // GProps[7] = Ixy, aGProps[8] = Ixz, GProps[9] = Iyz,
        //
        double x2, y2, z2;
        x2 = x * x;
        y2 = y * y;
        z2 = z * z;

        gProps[0] += ds;     // Area
        gProps[1] += x * ds; // Ix
        gProps[2] += y * ds; // Iy
        gProps[3] += z * ds; // Iz
        //
        gProps[7] += x * y * ds;     // Ixy
        gProps[8] += x * z * ds;     // Ixz
        gProps[9] += y * z * ds;     // Iyz
        gProps[4] += (y2 + z2) * ds; // Ixx
        gProps[5] += (x2 + z2) * ds; // Iyy
        gProps[6] += (x2 + y2) * ds; // Izz
    }

    // Calculate one Gauss point for volume properties of pyramid, based on triangle p1, p2, p3 with apex Apex
    private static void CalculateElVProps(double x, double y, double z, double dv, Span<double> gProps)
    {
        double x2, y2, z2;
        x2 = x * x;
        y2 = y * y;
        z2 = z * z;
        gProps[0] += dv / 3.0;      // Volume
        gProps[1] += 0.25 * x * dv; // Ix
        gProps[2] += 0.25 * y * dv; // Iy
        gProps[3] += 0.25 * z * dv; // Iz
        double dv1 = 0.2 * dv;
        gProps[7] += x * y * dv1;     // Ixy
        gProps[8] += x * z * dv1;     // Ixz
        gProps[9] += y * z * dv1;     // Iyz
        gProps[4] += (y2 + z2) * dv1; // Ixx
        gProps[5] += (x2 + z2) * dv1; // Iyy
        gProps[6] += (x2 + y2) * dv1; // Izz
    }
}
