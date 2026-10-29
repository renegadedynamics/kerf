// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Quadric.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Quadric.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public sealed class IntAna_Quadric
{
    private double CXX;
    private double CYY;
    private double CZZ;
    private double CXY;
    private double CXZ;
    private double CYZ;
    private double CX;
    private double CY;
    private double CZ;
    private double CCte;
    private readonly NCollection_List<gp_Pnt> mySpecialPoints = new();

    //----------------------------------------------------------------------
    //-- Empty Quadric
    //----------------------------------------------------------------------
    public IntAna_Quadric()
    {
        CXX = CYY = CZZ = CXY = CXZ = CYZ = CX = CY = CZ = 0.0;
        CCte = 1.0;
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
    public void SetQuadric(in gp_Pln P)
    {
        P.Coefficients(out CX, out CY, out CZ, out CCte);
        CX *= 0.5;
        CY *= 0.5;
        CZ *= 0.5;
        CXX = CYY = CZZ = CXY = CXZ = CYZ = 0.0;
    }

    /// <summary>Creates a quadric from a plane.</summary>
    public IntAna_Quadric(in gp_Pln P)
    {
        P.Coefficients(out CX, out CY, out CZ, out CCte);
        CX *= 0.5;
        CY *= 0.5;
        CZ *= 0.5;
        CXX = CYY = CZZ = CXY = CXZ = CYZ = 0.0;
    }

    //----------------------------------------------------------------------
    //-- Cylinder -----> Quadric
    //----------------------------------------------------------------------

    /// <summary>Initializes the quadric with a cylinder.</summary>
    public void SetQuadric(in gp_Cylinder Cyl)
    {
        Cyl.Coefficients(out CXX, out CYY, out CZZ, out CXY, out CXZ, out CYZ, out CX, out CY, out CZ, out CCte);
    }

    /// <summary>Creates a quadric from a cylinder.</summary>
    public IntAna_Quadric(in gp_Cylinder Cyl)
    {
        Cyl.Coefficients(out CXX, out CYY, out CZZ, out CXY, out CXZ, out CYZ, out CX, out CY, out CZ, out CCte);
    }

    //----------------------------------------------------------------------
    //-- Cone -----> Quadric
    //----------------------------------------------------------------------

    /// <summary>Creates a quadric from a cone. Its apex is a special point.</summary>
    public IntAna_Quadric(in gp_Cone Cone)
    {
        SetQuadric(Cone);
    }

    /// <summary>Initializes the quadric with a cone, and appends its apex to the special points.</summary>
    public void SetQuadric(in gp_Cone Cone)
    {
        Cone.Coefficients(out CXX, out CYY, out CZZ, out CXY, out CXZ, out CYZ, out CX, out CY, out CZ, out CCte);
        double aVParam = -Cone.RefRadius() / Math.Sin(Cone.SemiAngle());
        mySpecialPoints.Append(ElSLib.Value(0.0, aVParam, Cone));
    }

    //----------------------------------------------------------------------
    //-- Sphere -----> Quadric
    //----------------------------------------------------------------------

    /// <summary>Initializes the quadric with a sphere, and appends its poles to the special points.</summary>
    public void SetQuadric(in gp_Sphere Sph)
    {
        Sph.Coefficients(out CXX, out CYY, out CZZ, out CXY, out CXZ, out CYZ, out CX, out CY, out CZ, out CCte);
        mySpecialPoints.Append(ElSLib.Value(0.0, -M_PI_2, Sph));
        mySpecialPoints.Append(ElSLib.Value(0.0, M_PI_2, Sph));
    }

    /// <summary>Creates a quadric from a sphere. Its poles are special points.</summary>
    public IntAna_Quadric(in gp_Sphere Sph)
    {
        SetQuadric(Sph);
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
        _CXX = CXX;
        _CYY = CYY;
        _CZZ = CZZ;
        _CXY = CXY;
        _CXZ = CXZ;
        _CYZ = CYZ;
        _CX = CX;
        _CY = CY;
        _CZ = CZ;
        _CCte = CCte;
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
                                in gp_Ax3 Axis)
    {
        double t11, t12, t13, t14; // x = t11 X + t12 Y + t13 Z + t14
        double t21, t22, t23, t24; // y = t21 X + t22 Y + t23 Z + t24
        double t31, t32, t33, t34; // z = t31 X + t32 Y + t33 Z + t34

        //   = X DirX + Y DirY + Z DirZ + Loc

        double Cxx, Cyy, Czz, Cxy, Cxz, Cyz, Cx, Cy, Cz, Ccte;

        gp_Trsf Trans = new();
        Trans.SetTransformation(Axis);
        Trans.Invert();
        t11 = Trans.Value(1, 1);
        t12 = Trans.Value(1, 2);
        t13 = Trans.Value(1, 3);
        t14 = Trans.Value(1, 4);
        t21 = Trans.Value(2, 1);
        t22 = Trans.Value(2, 2);
        t23 = Trans.Value(2, 3);
        t24 = Trans.Value(2, 4);
        t31 = Trans.Value(3, 1);
        t32 = Trans.Value(3, 2);
        t33 = Trans.Value(3, 3);
        t34 = Trans.Value(3, 4);

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

        Ccte = ((_CCte) + t14_P2 * (_CXX) + t24_P2 * (_CYY) + t34_P2 * (_CZZ)
                + 2.0
                    * (t14 * ((_CX) + t24 * (_CXY) + t34 * (_CXZ)) + t24 * ((_CY) + t34 * (_CYZ))
                       + t34 * (_CZ)));

        Cxx = (t11_P2 * (_CXX) + t21_P2 * (_CYY) + t31_P2 * (_CZZ)
               + 2.0 * (t11 * (t21 * (_CXY) + t31 * (_CXZ)) + t21 * t31 * (_CYZ)));

        Cyy = (t12_P2 * (_CXX) + t22_P2 * (_CYY) + t32_P2 * (_CZZ)
               + 2.0 * (t12 * (t22 * (_CXY) + t32 * (_CXZ)) + t22 * t32 * (_CYZ)));

        Czz = (t13_P2 * (_CXX) + t33_P2 * (_CZZ) + t23_P2 * (_CYY)
               + 2.0 * (t13 * (t23 * (_CXY) + t33 * (_CXZ)) + t23 * t33 * (_CYZ)));

        Cz = (t13 * (_CX) + t13 * (t14 * (_CXX) + t24 * (_CXY) + t34 * (_CXZ))
              + t14 * (t23 * (_CXY) + t33 * (_CXZ)) + t23 * ((_CY) + t24 * (_CYY) + t34 * (_CYZ))
              + t33 * (t24 * (_CYZ) + (_CZ) + t34 * (_CZZ)));

        Cx = (t11 * ((_CX) + t14 * (_CXX) + t24 * (_CXY) + t34 * (_CXZ))
              + t14 * (t21 * (_CXY) + t31 * (_CXZ)) + t21 * ((_CY) + t24 * (_CYY) + t34 * (_CYZ))
              + t31 * (t24 * (_CYZ) + (_CZ) + t34 * (_CZZ)));

        Cxy = (t11 * (t12 * (_CXX) + t22 * (_CXY) + t32 * (_CXZ)) + t12 * (t21 * (_CXY) + t31 * (_CXZ))
               + t21 * (t22 * (_CYY) + t32 * (_CYZ)) + t31 * (t22 * (_CYZ) + t32 * (_CZZ)));

        Cxz = (t11 * (t13 * (_CXX) + t23 * (_CXY) + t33 * (_CXZ)) + t13 * (t21 * (_CXY) + t31 * (_CXZ))
               + t21 * (t23 * (_CYY) + t33 * (_CYZ)) + t31 * (t23 * (_CYZ) + t33 * (_CZZ)));

        Cy = (t12 * ((_CX) + t14 * (_CXX) + t24 * (_CXY) + t34 * (_CXZ))
              + t14 * (t22 * (_CXY) + t32 * (_CXZ)) + t22 * ((_CY) + t24 * (_CYY) + t34 * (_CYZ))
              + t32 * ((_CZ) + t24 * (_CYZ) + t34 * (_CZZ)));

        Cyz = (t12 * (t13 * (_CXX) + t23 * (_CXY) + t33 * (_CXZ)) + t13 * (t22 * (_CXY) + t32 * (_CXZ))
               + t22 * (t23 * (_CYY) + t33 * (_CYZ)) + t32 * (t23 * (_CYZ) + t33 * (_CZZ)));

        _CXX = Cxx;
        _CYY = Cyy;
        _CZZ = Czz;
        _CX = Cx;
        _CY = Cy;
        _CZ = Cz;
        _CXY = Cxy;
        _CXZ = Cxz;
        _CYZ = Cyz;
        _CCte = Ccte;
    }

    /// <summary>The special points (singularities): a cone's apex, a sphere's poles. A copy.</summary>
    public PackedList<gp_Pnt> SpecialPoints() => PackedList.Create<gp_Pnt>([.. mySpecialPoints]);

    /// <summary>The stored list of special points (OCCT's <c>SpecialPoints()</c>). Don't modify.</summary>
    internal NCollection_List<gp_Pnt> InternalSpecialPoints() => mySpecialPoints;
}
