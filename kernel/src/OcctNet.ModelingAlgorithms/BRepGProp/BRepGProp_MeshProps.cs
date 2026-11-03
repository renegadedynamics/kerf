// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshProps.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// The global properties of a surface mesh, as a surface (<see cref="BRepGProp_MeshObjType.Sinert"/>) or as the piece
/// of volume between the mesh and the reference point (<see cref="BRepGProp_MeshObjType.Vinert"/>).
/// </summary>
public class BRepGProp_MeshProps : Properties
{
    /// <summary>The types of geometric objects.</summary>
    public enum BRepGProp_MeshObjType
    {
        /// <summary>The 3D region delimited by the reference point and the surface mesh.</summary>
        Vinert = 0,

        /// <summary>The surface mesh in 3D space.</summary>
        Sinert,
    }

    private readonly BRepGProp_MeshObjType myType; //!< Type of geometric object

    /// <summary>A system for meshes of the given type.</summary>
    public BRepGProp_MeshProps(BRepGProp_MeshObjType theType) => myType = theType;

    /// <summary>Sets the point relative to which the calculation is done.</summary>
    public void SetLocation(in Point3D theLocation) => _loc = theLocation;

    /// <summary>
    /// Computes the global properties of the mesh placed by <paramref name="theLoc"/> (Gauss cubature over the
    /// triangles), its triangles reversed if <paramref name="theOri"/> is REVERSED.
    /// </summary>
    public void Perform(Triangulation? theMesh, in Placement theLoc, Orientation theOri)
    {
        if (theMesh is null || theMesh.NodeCount == 0 || theMesh.TriangleCount == 0)
        {
            return;
        }

        if (theLoc.IsIdentity)
        {
            Perform(theMesh, theOri);
        }
        else
        {
            Transform3D aTr = theLoc.Transformation();
            //
            bool isToCopy = aTr.ScaleFactor() * aTr.HVectorialPart().Determinant() < 0.0
                            || Math.Abs(Math.Abs(aTr.ScaleFactor()) - 1.0) > Constants.Resolution();
            if (isToCopy)
            {
                var aCopy = new Triangulation(theMesh.NodeCount, theMesh.TriangleCount, false);
                for (int i = 1; i <= theMesh.NodeCount; ++i)
                {
                    aCopy.SetNode(i, theMesh.Node(i).Transformed(aTr));
                }

                for (int i = 1; i <= theMesh.TriangleCount; ++i)
                {
                    aCopy.SetTriangle(i, theMesh.Triangle(i));
                }

                Perform(aCopy, theOri);
                return;
            }

            //
            Transform3D aTrInv = aTr.Inverted();
            Point3D loc_save = _loc;
            _loc.Transform(aTrInv);
            Perform(theMesh, theOri);
            // Computes the inertia tensor at mesh gravity center
            Matrix3x3 HMat, inertia0;
            Point3D g0 = _g;
            _g.SetXYZ(_g.XYZ() + _loc.XYZ());
            if (g0.XYZ().Modulus() > Constants.Resolution())
            {
                global::Kerf.Engine.Measurement.GProp.HOperator(_g, _loc, _dim, out HMat);
                inertia0 = _inertia - HMat;
            }
            else
            {
                inertia0 = _inertia;
            }

            // Transform inertia tensor for rotation
            Matrix3x3 HVec = aTrInv.HVectorialPart();
            Matrix3x3 HVecT = HVec.Transposed();
            HVecT.Multiply(inertia0);
            inertia0 = HVecT.Multiplied(HVec);
            // Put gravity center in true position of mesh
            _g.Transform(aTr);
            g0 = _g;
            _g.SetXYZ(_g.XYZ() - loc_save.XYZ());
            _loc = loc_save;
            //
            // Computes the inertia tensor for loc
            global::Kerf.Engine.Measurement.GProp.HOperator(g0, _loc, _dim, out HMat);
            _inertia = inertia0 + HMat;
        }
    }

    /// <summary>
    /// Computes the global properties of the mesh (Gauss cubature over the triangles), its triangles reversed if
    /// <paramref name="theOri"/> is REVERSED.
    /// </summary>
    public void Perform(Triangulation? theMesh, Orientation theOri)
    {
        if (theMesh is null || theMesh.NodeCount == 0 || theMesh.TriangleCount == 0)
        {
            return;
        }

        //
        // Gauss points for barycentriche coordinates
        ReadOnlySpan<double> GPtsWg =
        [
            1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0, /*3 points*/
            2.0 / 3.0, 1.0 / 6.0, 1.0 / 6.0,
            1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0,
        ];
        //
        int aNbGaussPoints = 3;

        // Array to store global properties
        Span<double> aGProps = stackalloc double[10];
        // aGProps[0] = Volume
        //  Static moments of inertia.
        // aGProps[1] = Ix, aGProps[2] = Iy, aGProps[3] = Iz
        // Matrix of moments of inertia.
        // aGProps[4] = Ixx, aGProps[5] = Iyy, aGProps[6] = Izz,
        // aGProps[7] = Ixy, aGProps[8] = Ixz, aGProps[9] = Iyz,

        bool isVolume = myType == BRepGProp_MeshObjType.Vinert;
        int n1, n2, n3; // node indices
        for (int i = 1; i <= theMesh.TriangleCount; ++i)
        {
            Triangle1 aTri = theMesh.Triangle(i);
            aTri.Get(out n1, out n2, out n3);
            if (theOri == Reversed)
            {
                (n2, n3) = (n3, n2);
            }

            // Calculate properties of a pyramid built on face and apex
            Point3D p1 = theMesh.Node(n1);
            Point3D p2 = theMesh.Node(n2);
            Point3D p3 = theMesh.Node(n3);
            CalculateProps(p1, p2, p3, _loc, isVolume, aGProps, aNbGaussPoints, GPtsWg);
        }

        _dim = aGProps[0];
        if (Math.Abs(_dim) >= 1.0e-20) // To be consistent with GProp_GProps
        {
            _g.SetX(aGProps[1] / _dim);
            _g.SetY(aGProps[2] / _dim);
            _g.SetZ(aGProps[3] / _dim);
        }
        else
        {
            _g.SetX(aGProps[1]);
            _g.SetY(aGProps[2]);
            _g.SetZ(aGProps[3]);
        }

        _inertia.SetValue(1, 1, aGProps[4]);
        _inertia.SetValue(1, 2, -aGProps[7]);
        _inertia.SetValue(1, 3, -aGProps[8]);
        _inertia.SetValue(2, 1, -aGProps[7]);
        _inertia.SetValue(2, 2, aGProps[5]);
        _inertia.SetValue(2, 3, -aGProps[9]);
        _inertia.SetValue(3, 1, -aGProps[8]);
        _inertia.SetValue(3, 2, -aGProps[9]);
        _inertia.SetValue(3, 3, aGProps[6]);
    }

    /// <summary>
    /// Adds to <paramref name="GProps"/> the global properties of the triangle {p1, p2, p3} relative to
    /// <paramref name="Apex"/>: those of the pyramid on it with that apex if <paramref name="isVolume"/>, else the
    /// surface ones. <paramref name="GaussPnts"/> holds, for each of the <paramref name="NbGaussPoints"/> points, two
    /// barycentric coordinates and a weight.
    /// </summary>
    /// <remarks>
    /// <paramref name="GProps"/> holds the mass, the static moments Ix, Iy, Iz, then Ixx, Iyy, Izz, Ixy, Ixz, Iyz.
    /// </remarks>
    public static void CalculateProps(in Point3D p1,
                                      in Point3D p2,
                                      in Point3D p3,
                                      in Point3D Apex,
                                      bool isVolume,
                                      Span<double> GProps,
                                      int NbGaussPoints,
                                      ReadOnlySpan<double> GaussPnts)
    {
        // GProps[0] = Volume
        //  Static moments of inertia.
        // GProps[1] = Ix, GProps[2] = Iy, GProps[3] = Iz
        // Matrix of moments of inertia.
        // GProps[4] = Ixx, GProps[5] = Iyy, GProps[6] = Izz,
        // GProps[7] = Ixy, aGProps[8] = Ixz, GProps[9] = Iyz,
        //

        // Define plane and coordinates of triangle nodes on plane
        var aV12 = new Vector3D(p2, p1);
        var aV23 = new Vector3D(p3, p2);
        Vector3D aNorm = aV12 ^ aV23;
        double aDet = aNorm.Magnitude();
        if (aDet <= Constants.Resolution())
        {
            return;
        }

        Xyz aCenter = (p1.XYZ() + p2.XYZ() + p3.XYZ()) / 3.0;
        var aPC = new Point3D(aCenter);
        var aDN = new Direction3D(aNorm);
        var aPosPln = new CoordinateSystem(aPC, aDN);
        // Coordinates of nodes on plane
        double x1, y1, x2, y2, x3, y3;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(aPosPln, p1, out x1, out y1);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(aPosPln, p2, out x2, out y2);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(aPosPln, p3, out x3, out y3);
        //
        double l1, l2; // barycentriche coordinates
        double x, y, z;
        double w; // weight
        int i;
        for (i = 0; i < NbGaussPoints; ++i)
        {
            int ind = 3 * i;
            l1 = GaussPnts[ind];
            l2 = GaussPnts[ind + 1];
            w = GaussPnts[ind + 2];
            w *= aDet;
            x = l1 * (x1 - x3) + l2 * (x2 - x3) + x3;
            y = l1 * (y1 - y3) + l2 * (y2 - y3) + y3;
            Point3D aP = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(x, y, aPosPln);
            x = aP.X() - Apex.X();
            y = aP.Y() - Apex.Y();
            z = aP.Z() - Apex.Z();
            //
            if (isVolume)
            {
                double xn = aDN.X() * w;
                double yn = aDN.Y() * w;
                double zn = aDN.Z() * w;
                double dv = x * xn + y * yn + z * zn;
                CalculateElVProps(x, y, z, dv, GProps);
            }
            else
            {
                double ds = w;
                CalculateElSProps(x, y, z, ds, GProps);
            }
        }
    }

    /// <summary>The type of mesh object.</summary>
    public BRepGProp_MeshObjType GetMeshObjType() => myType;

    // Calculate one Gauss point for surface properties of triangle p1, p2, p3 relatively point Apex
    private static void CalculateElSProps(double x, double y, double z, double ds, Span<double> GProps)
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

        GProps[0] += ds;     // Area
        GProps[1] += x * ds; // Ix
        GProps[2] += y * ds; // Iy
        GProps[3] += z * ds; // Iz
        //
        GProps[7] += x * y * ds;     // Ixy
        GProps[8] += x * z * ds;     // Ixz
        GProps[9] += y * z * ds;     // Iyz
        GProps[4] += (y2 + z2) * ds; // Ixx
        GProps[5] += (x2 + z2) * ds; // Iyy
        GProps[6] += (x2 + y2) * ds; // Izz
    }

    // Calculate one Gauss point for volume properties of pyramid, based on triangle p1, p2, p3 with apex Apex
    private static void CalculateElVProps(double x, double y, double z, double dv, Span<double> GProps)
    {
        double x2, y2, z2;
        x2 = x * x;
        y2 = y * y;
        z2 = z * z;
        GProps[0] += dv / 3.0;      // Volume
        GProps[1] += 0.25 * x * dv; // Ix
        GProps[2] += 0.25 * y * dv; // Iy
        GProps[3] += 0.25 * z * dv; // Iz
        double dv1 = 0.2 * dv;
        GProps[7] += x * y * dv1;     // Ixy
        GProps[8] += x * z * dv1;     // Ixz
        GProps[9] += y * z * dv1;     // Iyz
        GProps[4] += (y2 + z2) * dv1; // Ixx
        GProps[5] += (x2 + z2) * dv1; // Iyy
        GProps[6] += (x2 + y2) * dv1; // Izz
    }
}
