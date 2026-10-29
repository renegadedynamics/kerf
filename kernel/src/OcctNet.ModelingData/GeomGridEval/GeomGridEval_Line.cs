// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Line.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for line grid points, by the direct formula P(t) = Location + t * Direction.
/// </summary>
public sealed class GeomGridEval_Line : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_Line? myGeom;

    /// <summary>Constructor with the line to evaluate (null gives empty results).</summary>
    public GeomGridEval_Line(Geom_Line? theLine)
    {
        myGeom = theLine;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_Line? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt>();
        }

        NCollection_Array1<gp_Pnt> aResult = new(1, theParams.Length());

        // Extract line data
        gp_Lin aLin = myGeom.Lin();
        gp_Pnt aLoc = aLin.Location();
        gp_Dir aDir = aLin.Direction();

        // Pre-extract coordinates for vectorization potential
        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aDirX = aDir.X();
        double aDirY = aDir.Y();
        double aDirZ = aDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            aResult.SetValue(i - theParams.Lower() + 1,
                             new gp_Pnt(aLocX + t * aDirX, aLocY + t * aDirY, aLocZ + t * aDirZ));
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative, which is constant for a line.</summary>
    public NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD1>();
        }

        NCollection_Array1<Geom_Curve.ResD1> aResult = new(1, theParams.Length());

        gp_Lin aLin = myGeom.Lin();
        gp_Pnt aLoc = aLin.Location();
        gp_Dir aDir = aLin.Direction();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aDirX = aDir.X();
        double aDirY = aDir.Y();
        double aDirZ = aDir.Z();

        // D1 is constant for a line
        gp_Vec aD1 = new(aDirX, aDirY, aDirZ);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD1
            {
                Point = new gp_Pnt(aLocX + t * aDirX, aLocY + t * aDirY, aLocZ + t * aDirZ),
                D1 = aD1,
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives (D2 is zero).</summary>
    public NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD2>();
        }

        NCollection_Array1<Geom_Curve.ResD2> aResult = new(1, theParams.Length());

        gp_Lin aLin = myGeom.Lin();
        gp_Pnt aLoc = aLin.Location();
        gp_Dir aDir = aLin.Direction();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aDirX = aDir.X();
        double aDirY = aDir.Y();
        double aDirZ = aDir.Z();

        gp_Vec aD1 = new(aDirX, aDirY, aDirZ);
        gp_Vec aD2 = new(0, 0, 0); // Second derivative is zero for a line

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD2
            {
                Point = new gp_Pnt(aLocX + t * aDirX, aLocY + t * aDirY, aLocZ + t * aDirZ),
                D1 = aD1,
                D2 = aD2,
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third (D2 and D3 are zero).</summary>
    public NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD3>();
        }

        NCollection_Array1<Geom_Curve.ResD3> aResult = new(1, theParams.Length());

        gp_Lin aLin = myGeom.Lin();
        gp_Pnt aLoc = aLin.Location();
        gp_Dir aDir = aLin.Direction();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aDirX = aDir.X();
        double aDirY = aDir.Y();
        double aDirZ = aDir.Z();

        gp_Vec aD1 = new(aDirX, aDirY, aDirZ);
        gp_Vec aZero = new(0, 0, 0);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD3
            {
                Point = new gp_Pnt(aLocX + t * aDirX, aLocY + t * aDirY, aLocZ + t * aDirZ),
                D1 = aD1,
                D2 = aZero,
                D3 = aZero,
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative at all grid points: the direction for N = 1, zero above.</summary>
    public NCollection_Array1<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec>();
        }

        NCollection_Array1<gp_Vec> aResult = new(1, theParams.Length());

        if (theN == 1)
        {
            // D1 is constant for a line (the direction)
            gp_Dir aDir = myGeom.Lin().Direction();
            gp_Vec aD1 = new(aDir.X(), aDir.Y(), aDir.Z());
            for (int i = 1; i <= theParams.Length(); ++i)
            {
                aResult.SetValue(i, aD1);
            }
        }
        else
        {
            // All higher derivatives are zero for a line
            gp_Vec aZero = new(0, 0, 0);
            for (int i = 1; i <= theParams.Length(); ++i)
            {
                aResult.SetValue(i, aZero);
            }
        }
        return aResult;
    }
}
