// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Quadric.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Quadric.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Intersection;

//----------------------------------------------------------------------
//--
//-- General equation of quadrics:
//--
//--   f(x,y,z) =       CXX X**2  + CYY Y**2  + CZZ Z**2
//--
//--             + 2 (  CXY X Y   + CXZ X Z   + CYZ Y Z )
//--
//--             + 2 (  CX  X     + CY  Y     + CZ  Z)
//--
//--             + CCte
//--
//----------------------------------------------------------------------

/// <summary>A quadric (plane, sphere, cylinder or cone) described by the coefficients of its equation in the
/// global coordinate system.</summary>
/// <remarks>A working object (decision 152): <c>SetQuadric</c> is public, as in OCCT. <c>SetQuadric</c> with a cone
/// or a sphere appends to the special points without clearing them, as OCCT does.</remarks>
internal sealed class Quadric
{
    private double _cxx;
    private double _cyy;
    private double _czz;
    private double _cxy;
    private double _cxz;
    private double _cyz;
    private double _cx;
    private double _cy;
    private double _cz;
    private double _cCte;
    private readonly NCollection_List<Point3D> _specialPoints = new();

    //----------------------------------------------------------------------
    //-- Empty Quadric
    //----------------------------------------------------------------------
    public Quadric()
    {
        _cxx = _cyy = _czz = _cxy = _cxz = _cyz = _cx = _cy = _cz = 0.0;
        _cCte = 1.0;
    }

    //----------------------------------------------------------------------
    //-- Pln  -----> Quadric
    //--
    //-- Coefficients from gp_Pln return A,B,C,D
    //--              with f(x,y,z)  =    A x    + B y   +  C z   + D
    //--
    //--  which we identify as        2(  CX x   + CY y  +  CZ z ) + CCte
    //----------------------------------------------------------------------

    /// <summary>Initializes the quadric with a plane.</summary>
    public void SetQuadric(in Plane3D p)
    {
        p.Coefficients(out _cx, out _cy, out _cz, out _cCte);
        _cx *= 0.5;
        _cy *= 0.5;
        _cz *= 0.5;
        _cxx = _cyy = _czz = _cxy = _cxz = _cyz = 0.0;
    }

    /// <summary>Creates a quadric from a plane.</summary>
    public Quadric(in Plane3D p)
    {
        p.Coefficients(out _cx, out _cy, out _cz, out _cCte);
        _cx *= 0.5;
        _cy *= 0.5;
        _cz *= 0.5;
        _cxx = _cyy = _czz = _cxy = _cxz = _cyz = 0.0;
    }

    //----------------------------------------------------------------------
    //-- Cylinder -----> Quadric
    //----------------------------------------------------------------------

    /// <summary>Initializes the quadric with a cylinder.</summary>
    public void SetQuadric(in Cylinder3D cyl)
    {
        cyl.Coefficients(out _cxx, out _cyy, out _czz, out _cxy, out _cxz, out _cyz, out _cx, out _cy, out _cz, out _cCte);
    }

    /// <summary>Creates a quadric from a cylinder.</summary>
    public Quadric(in Cylinder3D cyl)
    {
        cyl.Coefficients(out _cxx, out _cyy, out _czz, out _cxy, out _cxz, out _cyz, out _cx, out _cy, out _cz, out _cCte);
    }

    //----------------------------------------------------------------------
    //-- Cone -----> Quadric
    //----------------------------------------------------------------------

    /// <summary>Creates a quadric from a cone. Its apex is a special point.</summary>
    public Quadric(in Cone3D cone)
    {
        SetQuadric(cone);
    }

    /// <summary>Initializes the quadric with a cone, and appends its apex to the special points.</summary>
    public void SetQuadric(in Cone3D cone)
    {
        cone.Coefficients(out _cxx, out _cyy, out _czz, out _cxy, out _cxz, out _cyz, out _cx, out _cy, out _cz, out _cCte);
        double vParam = -cone.RefRadius / Math.Sin(cone.SemiAngle);
        _specialPoints.Append(global::Kerf.Engine.Geometry.ElSLib.Value(0.0, vParam, cone));
    }

    //----------------------------------------------------------------------
    //-- Sphere -----> Quadric
    //----------------------------------------------------------------------

    /// <summary>Initializes the quadric with a sphere, and appends its poles to the special points.</summary>
    public void SetQuadric(in Sphere3D sph)
    {
        sph.Coefficients(out _cxx, out _cyy, out _czz, out _cxy, out _cxz, out _cyz, out _cx, out _cy, out _cz, out _cCte);
        _specialPoints.Append(global::Kerf.Engine.Geometry.ElSLib.Value(0.0, -M_PI_2, sph));
        _specialPoints.Append(global::Kerf.Engine.Geometry.ElSLib.Value(0.0, M_PI_2, sph));
    }

    /// <summary>Creates a quadric from a sphere. Its poles are special points.</summary>
    public Quadric(in Sphere3D sph)
    {
        SetQuadric(sph);
    }

    //----------------------------------------------------------------------
    //-- Returns the coefficients of the quadric
    //----------------------------------------------------------------------

    /// <summary>
    /// The coefficients of the equation of the quadric:
    /// xCXX x² + xCYY y² + xCZZ z² + 2 (xCXY x y + xCXZ x z + xCYZ y z) + 2 (xCX x + xCY y + xCZ z) + xCCte.
    /// </summary>
    public void Coefficients(out double _CXX,
                             out double _CYY,
                             out double _CZZ,
                             out double _CXY,
                             out double _CXZ,
                             out double _CYZ,
                             out double _CX,
                             out double _CY,
                             out double _CZ,
                             out double _CCte)
    {
        _CXX = _cxx;
        _CYY = _cyy;
        _CZZ = _czz;
        _CXY = _cxy;
        _CXZ = _cxz;
        _CYZ = _cyz;
        _CX = _cx;
        _CY = _cy;
        _CZ = _cz;
        _CCte = _cCte;
    }

    //----------------------------------------------------------------------
    //-- Computes the coefficients in a new coordinate system
    //----------------------------------------------------------------------

    /// <summary>
    /// Rewrites the coefficients given (of an equation in the global coordinate system) in the local coordinate
    /// system defined by Axis.
    /// </summary>
    /// <remarks>As in OCCT, the coefficients transformed are the arguments' values, not the quadric's own: callers
    /// pass what <see cref="Coefficients"/> returned.</remarks>
    public void NewCoefficients(ref double _CXX,
                                ref double _CYY,
                                ref double _CZZ,
                                ref double _CXY,
                                ref double _CXZ,
                                ref double _CYZ,
                                ref double _CX,
                                ref double _CY,
                                ref double _CZ,
                                ref double _CCte,
                                in CoordinateSystem axis)
    {
        double t11, t12, t13, t14; // x = t11 X + t12 Y + t13 Z + t14
        double t21, t22, t23, t24; // y = t21 X + t22 Y + t23 Z + t24
        double t31, t32, t33, t34; // z = t31 X + t32 Y + t33 Z + t34

        //   = X DirX + Y DirY + Z DirZ + Loc

        double cxx, cyy, czz, cxy, cxz, cyz, cx, cy, cz, ccte;

        Transform3D trans = new Transform3D().WithTransformation(axis);
        trans = trans.Inverted();
        t11 = trans.Value(1, 1);
        t12 = trans.Value(1, 2);
        t13 = trans.Value(1, 3);
        t14 = trans.Value(1, 4);
        t21 = trans.Value(2, 1);
        t22 = trans.Value(2, 2);
        t23 = trans.Value(2, 3);
        t24 = trans.Value(2, 4);
        t31 = trans.Value(3, 1);
        t32 = trans.Value(3, 2);
        t33 = trans.Value(3, 3);
        t34 = trans.Value(3, 4);

        //      P0(x,y,z)=_CXX x x + _CYY y y + ... + _CCte =0  (x,y,z "absolute" coordinates)
        // and  P1(X(x,y,z),Y(x,y,z),Z(x,y,z))=P0(x,y,z)
        //
        // with P1(X,Y,Z)= Cxx X X + Cyy Y Y + Czz Z Z + 2 Cxy X Y ... + Ccte
        //               = _CXX  x x + _CYY  y y  +  ...  +  _CCte

        double t11_P2 = t11 * t11;
        double t21_P2 = t21 * t21;
        double t31_P2 = t31 * t31;
        double t12_P2 = t12 * t12;
        double t22_P2 = t22 * t22;
        double t32_P2 = t32 * t32;
        double t13_P2 = t13 * t13;
        double t23_P2 = t23 * t23;
        double t33_P2 = t33 * t33;
        double t14_P2 = t14 * t14;
        double t24_P2 = t24 * t24;
        double t34_P2 = t34 * t34;

        ccte = ((_CCte) + t14_P2 * (_CXX) + t24_P2 * (_CYY) + t34_P2 * (_CZZ)
                + 2.0
                    * (t14 * ((_CX) + t24 * (_CXY) + t34 * (_CXZ)) + t24 * ((_CY) + t34 * (_CYZ))
                       + t34 * (_CZ)));

        cxx = (t11_P2 * (_CXX) + t21_P2 * (_CYY) + t31_P2 * (_CZZ)
               + 2.0 * (t11 * (t21 * (_CXY) + t31 * (_CXZ)) + t21 * t31 * (_CYZ)));

        cyy = (t12_P2 * (_CXX) + t22_P2 * (_CYY) + t32_P2 * (_CZZ)
               + 2.0 * (t12 * (t22 * (_CXY) + t32 * (_CXZ)) + t22 * t32 * (_CYZ)));

        czz = (t13_P2 * (_CXX) + t33_P2 * (_CZZ) + t23_P2 * (_CYY)
               + 2.0 * (t13 * (t23 * (_CXY) + t33 * (_CXZ)) + t23 * t33 * (_CYZ)));

        cz = (t13 * (_CX) + t13 * (t14 * (_CXX) + t24 * (_CXY) + t34 * (_CXZ))
              + t14 * (t23 * (_CXY) + t33 * (_CXZ)) + t23 * ((_CY) + t24 * (_CYY) + t34 * (_CYZ))
              + t33 * (t24 * (_CYZ) + (_CZ) + t34 * (_CZZ)));

        cx = (t11 * ((_CX) + t14 * (_CXX) + t24 * (_CXY) + t34 * (_CXZ))
              + t14 * (t21 * (_CXY) + t31 * (_CXZ)) + t21 * ((_CY) + t24 * (_CYY) + t34 * (_CYZ))
              + t31 * (t24 * (_CYZ) + (_CZ) + t34 * (_CZZ)));

        cxy = (t11 * (t12 * (_CXX) + t22 * (_CXY) + t32 * (_CXZ)) + t12 * (t21 * (_CXY) + t31 * (_CXZ))
               + t21 * (t22 * (_CYY) + t32 * (_CYZ)) + t31 * (t22 * (_CYZ) + t32 * (_CZZ)));

        cxz = (t11 * (t13 * (_CXX) + t23 * (_CXY) + t33 * (_CXZ)) + t13 * (t21 * (_CXY) + t31 * (_CXZ))
               + t21 * (t23 * (_CYY) + t33 * (_CYZ)) + t31 * (t23 * (_CYZ) + t33 * (_CZZ)));

        cy = (t12 * ((_CX) + t14 * (_CXX) + t24 * (_CXY) + t34 * (_CXZ))
              + t14 * (t22 * (_CXY) + t32 * (_CXZ)) + t22 * ((_CY) + t24 * (_CYY) + t34 * (_CYZ))
              + t32 * ((_CZ) + t24 * (_CYZ) + t34 * (_CZZ)));

        cyz = (t12 * (t13 * (_CXX) + t23 * (_CXY) + t33 * (_CXZ)) + t13 * (t22 * (_CXY) + t32 * (_CXZ))
               + t22 * (t23 * (_CYY) + t33 * (_CYZ)) + t32 * (t23 * (_CYZ) + t33 * (_CZZ)));

        _CXX = cxx;
        _CYY = cyy;
        _CZZ = czz;
        _CX = cx;
        _CY = cy;
        _CZ = cz;
        _CXY = cxy;
        _CXZ = cxz;
        _CYZ = cyz;
        _CCte = ccte;
    }

    /// <summary>The special points (singularities): a cone's apex, a sphere's poles. A copy.</summary>
    public IReadOnlyList<Point3D> SpecialPoints() => ImmutableArrays.Copy<Point3D>([.. _specialPoints]);

    /// <summary>The stored list of special points (OCCT's <c>SpecialPoints()</c>). Don't modify.</summary>
    internal NCollection_List<Point3D> InternalSpecialPoints() => _specialPoints;
}
