// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntLinTorus.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntLinTorus.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntLinTorus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//-- IntAna_IntLinTorus.cxx
//-- lbr : the method using coefficients is catastrophic.
//--       Implementation of a proper solution.

namespace Occt;

/// <summary>Intersection between a line and a torus.</summary>
/// <remarks>An algorithm object: construct it with the line and the torus, or empty and call <c>Perform</c>.</remarks>
public class IntAna_IntLinTorus
{
    private bool done;
    private int nbpt;
    private readonly gp_Pnt[] thePoint = new gp_Pnt[4];
    private readonly double[] theParam = new double[4];
    private readonly double[] theFi = new double[4];
    private readonly double[] theTheta = new double[4];

    public IntAna_IntLinTorus()
    {
        done = false;
        nbpt = 0;
    }

    /// <summary>Creates the intersection between a line and a torus.</summary>
    public IntAna_IntLinTorus(in gp_Lin L, in gp_Torus T)
    {
        Perform(L, T);
    }

    /// <summary>Intersects a line and a torus.</summary>
    public void Perform(in gp_Lin L, in gp_Torus T)
    {
        gp_Pnt PL = L.Location();
        gp_Dir DL = L.Direction();

        // Reparametrize the line:
        // set its location as nearest to the location of torus
        gp_Pnt TorLoc = T.Location();
        double ParamOfNewPL = new gp_Vec(PL, TorLoc).Dot(new gp_Vec(DL));
        gp_Pnt NewPL = new(PL.XYZ() + ParamOfNewPL * DL.XYZ());

        //--------------------------------------------------------------
        //-- Coefficients of the line in the torus reference frame
        //--
        gp_Trsf trsf = new();
        trsf.SetTransformation(T.Position());
        NewPL.Transform(trsf);
        DL.Transform(trsf);

        double a, b, c, x1, y1, z1, x0, y0, z0;
        double a0, a1, a2, a3, a4;
        double R, r, R2, r2;

        x1 = DL.X();
        y1 = DL.Y();
        z1 = DL.Z();
        x0 = NewPL.X();
        y0 = NewPL.Y();
        z0 = NewPL.Z();
        R = T.MajorRadius();
        R2 = R * R;
        r = T.MinorRadius();
        r2 = r * r;

        a = x1 * x1 + y1 * y1 + z1 * z1;
        b = 2.0 * (x1 * x0 + y1 * y0 + z1 * z0);
        c = x0 * x0 + y0 * y0 + z0 * z0 - (R2 + r2);

        a4 = a * a;
        a3 = 2.0 * a * b;
        a2 = 2.0 * a * c + 4.0 * R2 * z1 * z1 + b * b;
        a1 = 2.0 * b * c + 8.0 * R2 * z1 * z0;
        a0 = c * c + 4.0 * R2 * (z0 * z0 - r2);

        double u, v;
        math_DirectPolynomialRoots mdpr = new(a4, a3, a2, a1, a0);
        if (mdpr.IsDone())
        {
            int nbsolvalid = 0;
            int n = mdpr.NbSolutions();
            int aNbBadSol = 0;
            for (int i = 1; i <= n; i++)
            {
                double t = mdpr.Value(i);
                t += ParamOfNewPL;
                gp_Pnt PSolL = ElCLib.Value(t, L);
                ElSLib.Parameters(T, PSolL, out u, out v);
                gp_Pnt PSolT = ElSLib.Value(u, v, T);
                a0 = PSolT.SquareDistance(PSolL);

                if (a0 > 0.0000000001)
                {
                    aNbBadSol++;
                }
                else
                {
                    theParam[nbsolvalid] = t;
                    theFi[nbsolvalid] = u;
                    theTheta[nbsolvalid] = v;
                    thePoint[nbsolvalid] = PSolL;
                    nbsolvalid++;
                }
            }
            if (n > 0 && nbsolvalid == 0 && aNbBadSol == n)
            {
                nbpt = 0;
                done = false;
            }
            else
            {
                nbpt = nbsolvalid;
                done = true;
            }
        }
        else
        {
            nbpt = 0;
            done = false;
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => done;

    /// <summary>The number of intersection points.</summary>
    /// <exception cref="StdFail_NotDone">The computation was not done.</exception>
    public int NbPoints()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return nbpt;
    }

    /// <summary>The intersection point of range Index.</summary>
    /// <exception cref="StdFail_NotDone">The computation was not done.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbPoints].</exception>
    public gp_Pnt Value(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > nbpt)
        {
            throw new Standard_OutOfRange();
        }
        return thePoint[Index - 1];
    }

    /// <summary>The parameter on the line of the intersection point of range Index.</summary>
    /// <exception cref="StdFail_NotDone">The computation was not done.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbPoints].</exception>
    public double ParamOnLine(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > nbpt)
        {
            throw new Standard_OutOfRange();
        }
        return theParam[Index - 1];
    }

    /// <summary>The parameters on the torus of the intersection point of range Index.</summary>
    /// <exception cref="StdFail_NotDone">The computation was not done.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbPoints].</exception>
    public void ParamOnTorus(int Index, out double FI, out double THETA)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > nbpt)
        {
            throw new Standard_OutOfRange();
        }
        FI = theFi[Index - 1];
        THETA = theTheta[Index - 1];
    }
}
