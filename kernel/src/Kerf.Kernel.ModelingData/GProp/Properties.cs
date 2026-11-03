// Port of: src/ModelingData/TKGeomBase/GProp/GProp_GProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_GProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Global properties of a "compound geometric system" in 3D space (mass, centre of mass, inertia), composed with
/// <see cref="Add"/> from the global properties of its components: sets of points (<see cref="PGProps"/>),
/// curves, surfaces and solids (BRepGProp, GeomGProp), or other systems.
/// </summary>
/// <remarks>
/// A working object (an accumulator): <see cref="Add"/> and the setters of the derived classes are public, as in
/// OCCT. A C++ copy (<c>GProp_GProps a = b;</c>) is <c>new GProp_GProps(b)</c>.
/// </remarks>
public class Properties
{
    /// <summary>Centre of mass, relative to <see cref="_loc"/> (absolute coordinates in the elementary classes).</summary>
    protected Point3D _g;

    /// <summary>Reference point used for inertia accumulation.</summary>
    protected Point3D _loc;

    /// <summary>Total mass / length / area / volume.</summary>
    protected double _dim;

    /// <summary>Quadratic moments of inertia at <see cref="_loc"/>.</summary>
    protected Matrix3x3 _inertia;

    /// <summary>An empty system; the origin is the reference point.</summary>
    public Properties()
    {
        _g = Constants.Origin();
        _loc = Constants.Origin();
        _dim = 0.0;
        _inertia = new Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>
    /// An empty system using <paramref name="systemLocation"/> as its reference point. For accuracy, choose a point
    /// near the system's centre of mass.
    /// </summary>
    public Properties(in Point3D systemLocation)
    {
        _g = Constants.Origin();
        _loc = systemLocation;
        _dim = 0.0;
        _inertia = new Matrix3x3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /// <summary>A copy of the global properties of <paramref name="other"/> (C++ copy construction).</summary>
    public Properties(Properties other)
    {
        _g = other._g;
        _loc = other._loc;
        _dim = other._dim;
        _inertia = other._inertia;
    }

    /// <summary>
    /// C++ copy assignment (<c>a = b;</c>): this system takes the global properties of <paramref name="other"/>.
    /// BRepGProp resets the caller's system this way (<c>Props = GProp_GProps(P);</c>).
    /// </summary>
    internal void Assign(Properties other)
    {
        _g = other._g;
        _loc = other._loc;
        _dim = other._dim;
        _inertia = other._inertia;
    }

    /// <summary>
    /// Adds the global properties of <paramref name="item"/>, with density <paramref name="density"/>, to this
    /// system. Huygens' theorem moves the inertia of Item to the reference point of this system. A density already
    /// counted when Item was built (weighted points, a composed system) must not be given again.
    /// </summary>
    /// <exception cref="DomainErrorException">Density is not greater than gp::Resolution().</exception>
    public void Add(Properties item, double density = 1.0)
    {
        if (density <= Constants.Resolution())
        {
            throw new DomainErrorException();
        }
        if (_loc.Distance(item._loc) <= Constants.Resolution())
        {
            Xyz gxyz = (item._g.XYZ()).Multiplied(item._dim * density);
            _g = _g.WithXYZ(_g.XYZ().Multiplied(_dim));
            gxyz.Add(_g.XYZ());
            _dim = _dim + item._dim * density;
            if (Math.Abs(_dim) >= 1.0e-20)
            {
                gxyz.Divide(_dim);
                _g = _g.WithXYZ(gxyz);
            }
            else
            {
                _g = _g.WithCoord(0.0, 0.0, 0.0);
            }
            _inertia = _inertia + item._inertia * density;
        }
        else
        {
            Xyz itemloc = _loc.XYZ() - item._loc.XYZ();
            Xyz itemg = item._loc.XYZ() + item._g.XYZ();
            Xyz gxyz = item._g.XYZ() - itemloc;
            gxyz = gxyz.Multiplied(item._dim * density);
            _g = _g.WithXYZ(_g.XYZ().Multiplied(_dim));
            gxyz.Add(_g.XYZ());
            _dim = _dim + item._dim * density;
            if (Math.Abs(_dim) >= 1.0e-20)
            {
                gxyz.Divide(_dim);
                _g = _g.WithXYZ(gxyz);
            }
            else
            {
                _g = _g.WithCoord(0.0, 0.0, 0.0);
            }
            // We have to compute the inertia of the Item at the location point
            // of the system using the Huyghens theorem
            Matrix3x3 hMat;
            Matrix3x3 itemInertia = item._inertia;
            if (item._g.XYZ().Modulus() > Constants.Resolution())
            {
                // Computes the inertia of Item at its dim centre
                global::Kerf.Engine.Measurement.GProp.HOperator(new Point3D(itemg), item._loc, item._dim, out hMat);
                itemInertia = itemInertia - hMat;
            }
            // Computes the inertia of Item at the location point of the system
            global::Kerf.Engine.Measurement.GProp.HOperator(new Point3D(itemg), _loc, item._dim, out hMat);
            itemInertia = itemInertia + hMat;
            _inertia = _inertia + itemInertia * density;
        }
    }

    /// <summary>The mass of the system: its total length, area or volume when no density was given.</summary>
    public double Mass() => _dim;

    /// <summary>The centre of mass, in the absolute coordinate system.</summary>
    public Point3D CentreOfMass() => new(_loc.XYZ() + _g.XYZ());

    /// <summary>
    /// The (symmetric) matrix of inertia in the central coordinate system: at the centre of mass, with axes parallel
    /// to the absolute ones. <see cref="GProp.HOperator"/> moves it elsewhere.
    /// </summary>
    public Matrix3x3 MatrixOfInertia()
    {
        global::Kerf.Engine.Measurement.GProp.HOperator(_g, Constants.Origin(), _dim, out Matrix3x3 hMat);
        return _inertia - hMat;
    }

    /// <summary>The static moments of inertia about the three absolute axes.</summary>
    public void StaticMoments(out double ix, out double iy, out double iz)
    {
        Xyz g = _loc.XYZ() + _g.XYZ();
        ix = g.X() * _dim;
        iy = g.Y() * _dim;
        iz = g.Z() * _dim;
    }

    /// <summary>The moment of inertia of the system about the axis <paramref name="a"/>.</summary>
    public double MomentOfInertia(in Axis3D a)
    {
        // Moment of inertia / axis A
        // 1] computes the math_Matrix of inertia / A.location()
        // 2] applies this math_Matrix to A.Direction()
        // 3] then computes the scalar product between this vector and
        //    A.Direction()

        if (_loc.Distance(a.Location()) <= Constants.Resolution())
        {
            return (a.Direction().XYZ()).Dot((a.Direction().XYZ()).Multiplied(_inertia));
        }
        else
        {
            Matrix3x3 axisInertia = MatrixOfInertia();
            global::Kerf.Engine.Measurement.GProp.HOperator(new Point3D(_loc.XYZ() + _g.XYZ()), a.Location(), _dim, out Matrix3x3 hMat);
            axisInertia = axisInertia + hMat;
            return (a.Direction().XYZ()).Dot((a.Direction().XYZ()).Multiplied(axisInertia));
        }
    }

    /// <summary>
    /// The principal properties of inertia: the eigenvalues and eigenvectors of the matrix of inertia, with the
    /// principal radii of gyration.
    /// </summary>
    public PrincipalProperties PrincipalProperties()
    {
        var diagMat = new Matrix(1, 3, 1, 3);
        int i, j;
        Matrix3x3 axisInertia = MatrixOfInertia();
        for (j = 1; j <= 3; j++)
        {
            for (i = 1; i <= 3; i++)
            {
                diagMat[i, j] = axisInertia.Value(i, j);
            }
        }
        var J = new Jacobi(diagMat);
        double ixx = J.Value(1);
        double iyy = J.Value(2);
        double izz = J.Value(3);
        diagMat = J.Vectors();
        var vxx = new Vector3D(diagMat[1, 1], diagMat[2, 1], diagMat[3, 1]);
        var vyy = new Vector3D(diagMat[1, 2], diagMat[2, 2], diagMat[3, 2]);
        var vzz = new Vector3D(diagMat[1, 3], diagMat[2, 3], diagMat[3, 3]);
        //
        // Protection against dim == 0.0e0 in case only a single point was provided
        //
        double rxx = 0.0e0;
        double ryy = 0.0e0;
        double rzz = 0.0e0;
        if (0.0e0 != _dim)
        {
            rxx = Math.Sqrt(Math.Abs(ixx / _dim));
            ryy = Math.Sqrt(Math.Abs(iyy / _dim));
            rzz = Math.Sqrt(Math.Abs(izz / _dim));
        }
        return new PrincipalProperties(ixx, iyy, izz, rxx, ryy, rzz, vxx, vyy, vzz, new Point3D(_g.XYZ() + _loc.XYZ()));
    }

    /// <summary>The radius of gyration of the system about the axis <paramref name="a"/>.</summary>
    public double RadiusOfGyration(in Axis3D a) => Math.Sqrt(MomentOfInertia(a) / _dim);
}
