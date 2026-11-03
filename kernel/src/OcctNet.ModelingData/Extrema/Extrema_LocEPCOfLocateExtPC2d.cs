// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocEPCOfLocateExtPC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The extremum distance between a point and a 2D curve nearest a starting parameter: the root of
/// <see cref="Extrema_PCFOfEPCOfExtPC2d"/> found by math_FunctionRoot from U0, kept if the function is below 1e-7
/// there.
/// </summary>
/// <remarks>
/// The 2D instantiation of OCCT's template Extrema_GenLocateExtPC (Extrema_LocEPCOfLocateExtPC2d). An algorithm object
/// (decision 117). Keeps a reference to the curve, as OCCT does.
/// </remarks>
public class Extrema_LocEPCOfLocateExtPC2d
{
    private bool myDone;
    private double mytolU;
    private double myumin;
    private double myusup;
    private readonly Extrema_PCFOfEPCOfExtPC2d myF = new();

    public Extrema_LocEPCOfLocateExtPC2d()
    {
        myDone = false;
        mytolU = 0.0;
        myumin = 0.0;
        myusup = 0.0;
    }

    /// <summary>The extremum between theP and theC nearest theU0, on the curve's whole range.</summary>
    public Extrema_LocEPCOfLocateExtPC2d(in Point2D theP, Adaptor2d_Curve2d theC, double theU0, double theTolU)
    {
        Initialize(theC, Extrema_Curve2dTool.FirstParameter(theC), Extrema_Curve2dTool.LastParameter(theC), theTolU);
        Perform(theP, theU0);
    }

    /// <summary>The extremum between theP and theC nearest theU0, on [theUmin, theUsup].</summary>
    public Extrema_LocEPCOfLocateExtPC2d(in Point2D theP, Adaptor2d_Curve2d theC, double theU0, double theUmin,
        double theUsup, double theTolU)
    {
        Initialize(theC, theUmin, theUsup, theTolU);
        Perform(theP, theU0);
    }

    /// <summary>Sets the curve, the range and the parametric tolerance.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theUmin, double theUsup, double theTolU)
    {
        myDone = false;
        myF.Initialize(theC);
        myumin = theUmin;
        myusup = theUsup;
        mytolU = theTolU;
    }

    /// <summary>Searches the extremum for theP from the parameter theU0.</summary>
    public void Perform(in Point2D theP, double theU0)
    {
        myF.SetPoint(theP);
        FunctionRoot S = new(myF, theU0, mytolU, myumin, myusup);
        myDone = S.IsDone;
        if (myDone)
        {
            double uu;
            Extrema_POnCurv2d PP = Point();
            uu = PP.Parameter();
            if (myF.Value(uu, out double ff))
            {
                if (Math.Abs(ff) >= 1.0e-07)
                {
                    myDone = false;
                }
            }
            else
            {
                myDone = false;
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myF.SquareDistance(1);
    }

    /// <summary>True if the extremum is a minimum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsMin()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myF.IsMin(1);
    }

    /// <summary>The point of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Extrema_POnCurv2d Point()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myF.Point(1);
    }
}
