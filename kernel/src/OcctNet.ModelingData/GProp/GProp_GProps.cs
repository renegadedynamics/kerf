// Port of: src/ModelingData/TKGeomBase/GProp/GProp_GProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_GProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Global properties of a "compound geometric system" in 3D space (mass, centre of mass, inertia), composed with
/// <see cref="Add"/> from the global properties of its components: sets of points (<see cref="GProp_PGProps"/>),
/// curves, surfaces and solids (BRepGProp, GeomGProp), or other systems.
/// </summary>
/// <remarks>
/// A working object (an accumulator): <see cref="Add"/> and the setters of the derived classes are public, as in
/// OCCT. A C++ copy (<c>GProp_GProps a = b;</c>) is <c>new GProp_GProps(b)</c>.
/// </remarks>
public class GProp_GProps
{
    /// <summary>Centre of mass, relative to <see cref="loc"/> (absolute coordinates in the elementary classes).</summary>
    protected Point3D g;

    /// <summary>Reference point used for inertia accumulation.</summary>
    protected Point3D loc;

    /// <summary>Total mass / length / area / volume.</summary>
    protected double dim;

    /// <summary>Quadratic moments of inertia at <see cref="loc"/>.</summary>
    protected Matrix3x3 inertia;

    /// <summary>An empty system; the origin is the reference point.</summary>
    public GProp_GProps()
    {
        g = Constants.Origin();
        loc = Constants.Origin();
        dim = 0.0;
        inertia = new Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>
    /// An empty system using <paramref name="SystemLocation"/> as its reference point. For accuracy, choose a point
    /// near the system's centre of mass.
    /// </summary>
    public GProp_GProps(in Point3D SystemLocation)
    {
        g = Constants.Origin();
        loc = SystemLocation;
        dim = 0.0;
        inertia = new Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>A copy of the global properties of <paramref name="theOther"/> (C++ copy construction).</summary>
    public GProp_GProps(GProp_GProps theOther)
    {
        g = theOther.g;
        loc = theOther.loc;
        dim = theOther.dim;
        inertia = theOther.inertia;
    }

    /// <summary>
    /// C++ copy assignment (<c>a = b;</c>): this system takes the global properties of <paramref name="theOther"/>.
    /// BRepGProp resets the caller's system this way (<c>Props = GProp_GProps(P);</c>).
    /// </summary>
    internal void Assign(GProp_GProps theOther)
    {
        g = theOther.g;
        loc = theOther.loc;
        dim = theOther.dim;
        inertia = theOther.inertia;
    }

    /// <summary>
    /// Adds the global properties of <paramref name="Item"/>, with density <paramref name="Density"/>, to this
    /// system. Huygens' theorem moves the inertia of Item to the reference point of this system. A density already
    /// counted when Item was built (weighted points, a composed system) must not be given again.
    /// </summary>
    /// <exception cref="DomainErrorException">Density is not greater than gp::Resolution().</exception>
    public void Add(GProp_GProps Item, double Density = 1.0)
    {
        if (Density <= Constants.Resolution())
        {
            throw new DomainErrorException();
        }
        if (loc.Distance(Item.loc) <= Constants.Resolution())
        {
            Xyz GXYZ = (Item.g.XYZ()).Multiplied(Item.dim * Density);
            g.SetXYZ(g.XYZ().Multiplied(dim));
            GXYZ.Add(g.XYZ());
            dim = dim + Item.dim * Density;
            if (Math.Abs(dim) >= 1.0e-20)
            {
                GXYZ.Divide(dim);
                g.SetXYZ(GXYZ);
            }
            else
            {
                g.SetCoord(0.0, 0.0, 0.0);
            }
            inertia = inertia + Item.inertia * Density;
        }
        else
        {
            Xyz Itemloc = loc.XYZ() - Item.loc.XYZ();
            Xyz Itemg = Item.loc.XYZ() + Item.g.XYZ();
            Xyz GXYZ = Item.g.XYZ() - Itemloc;
            GXYZ = GXYZ.Multiplied(Item.dim * Density);
            g.SetXYZ(g.XYZ().Multiplied(dim));
            GXYZ.Add(g.XYZ());
            dim = dim + Item.dim * Density;
            if (Math.Abs(dim) >= 1.0e-20)
            {
                GXYZ.Divide(dim);
                g.SetXYZ(GXYZ);
            }
            else
            {
                g.SetCoord(0.0, 0.0, 0.0);
            }
            // We have to compute the inertia of the Item at the location point
            // of the system using the Huyghens theorem
            Matrix3x3 HMat;
            Matrix3x3 ItemInertia = Item.inertia;
            if (Item.g.XYZ().Modulus() > Constants.Resolution())
            {
                // Computes the inertia of Item at its dim centre
                GProp.HOperator(new Point3D(Itemg), Item.loc, Item.dim, out HMat);
                ItemInertia = ItemInertia - HMat;
            }
            // Computes the inertia of Item at the location point of the system
            GProp.HOperator(new Point3D(Itemg), loc, Item.dim, out HMat);
            ItemInertia = ItemInertia + HMat;
            inertia = inertia + ItemInertia * Density;
        }
    }

    /// <summary>The mass of the system: its total length, area or volume when no density was given.</summary>
    public double Mass() => dim;

    /// <summary>The centre of mass, in the absolute coordinate system.</summary>
    public Point3D CentreOfMass() => new(loc.XYZ() + g.XYZ());

    /// <summary>
    /// The (symmetric) matrix of inertia in the central coordinate system: at the centre of mass, with axes parallel
    /// to the absolute ones. <see cref="GProp.HOperator"/> moves it elsewhere.
    /// </summary>
    public Matrix3x3 MatrixOfInertia()
    {
        GProp.HOperator(g, Constants.Origin(), dim, out Matrix3x3 HMat);
        return inertia - HMat;
    }

    /// <summary>The static moments of inertia about the three absolute axes.</summary>
    public void StaticMoments(out double Ix, out double Iy, out double Iz)
    {
        Xyz G = loc.XYZ() + g.XYZ();
        Ix = G.X() * dim;
        Iy = G.Y() * dim;
        Iz = G.Z() * dim;
    }

    /// <summary>The moment of inertia of the system about the axis <paramref name="A"/>.</summary>
    public double MomentOfInertia(in Axis3D A)
    {
        // Moment of inertia / axis A
        // 1] computes the math_Matrix of inertia / A.location()
        // 2] applies this math_Matrix to A.Direction()
        // 3] then computes the scalar product between this vector and
        //    A.Direction()

        if (loc.Distance(A.Location()) <= Constants.Resolution())
        {
            return (A.Direction().XYZ()).Dot((A.Direction().XYZ()).Multiplied(inertia));
        }
        else
        {
            Matrix3x3 AxisInertia = MatrixOfInertia();
            GProp.HOperator(new Point3D(loc.XYZ() + g.XYZ()), A.Location(), dim, out Matrix3x3 HMat);
            AxisInertia = AxisInertia + HMat;
            return (A.Direction().XYZ()).Dot((A.Direction().XYZ()).Multiplied(AxisInertia));
        }
    }

    /// <summary>
    /// The principal properties of inertia: the eigenvalues and eigenvectors of the matrix of inertia, with the
    /// principal radii of gyration.
    /// </summary>
    public GProp_PrincipalProps PrincipalProperties()
    {
        var DiagMat = new Matrix(1, 3, 1, 3);
        int i, j;
        Matrix3x3 AxisInertia = MatrixOfInertia();
        for (j = 1; j <= 3; j++)
        {
            for (i = 1; i <= 3; i++)
            {
                DiagMat[i, j] = AxisInertia.Value(i, j);
            }
        }
        var J = new Jacobi(DiagMat);
        double Ixx = J.Value(1);
        double Iyy = J.Value(2);
        double Izz = J.Value(3);
        DiagMat = J.Vectors();
        var Vxx = new Vector3D(DiagMat[1, 1], DiagMat[2, 1], DiagMat[3, 1]);
        var Vyy = new Vector3D(DiagMat[1, 2], DiagMat[2, 2], DiagMat[3, 2]);
        var Vzz = new Vector3D(DiagMat[1, 3], DiagMat[2, 3], DiagMat[3, 3]);
        //
        // Protection against dim == 0.0e0 in case only a single point was provided
        //
        double Rxx = 0.0e0;
        double Ryy = 0.0e0;
        double Rzz = 0.0e0;
        if (0.0e0 != dim)
        {
            Rxx = Math.Sqrt(Math.Abs(Ixx / dim));
            Ryy = Math.Sqrt(Math.Abs(Iyy / dim));
            Rzz = Math.Sqrt(Math.Abs(Izz / dim));
        }
        return new GProp_PrincipalProps(Ixx, Iyy, Izz, Rxx, Ryy, Rzz, Vxx, Vyy, Vzz, new Point3D(g.XYZ() + loc.XYZ()));
    }

    /// <summary>The radius of gyration of the system about the axis <paramref name="A"/>.</summary>
    public double RadiusOfGyration(in Axis3D A) => Math.Sqrt(MomentOfInertia(A) / dim);
}
