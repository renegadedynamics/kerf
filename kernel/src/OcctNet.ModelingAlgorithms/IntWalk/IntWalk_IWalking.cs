// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_IWalking.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_IWalking.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#pragma warning disable CS0414 // SKELETON: removed when the body lands

namespace Occt;

/// <summary>What IntWalk_IWalking reads from its function, beyond math_FunctionSetWithDerivatives (decision 607).</summary>
public interface IIntWalk_IWFunction : IIntStart_SurfFunction<Adaptor3d_Surface>
{
    /// <summary>The parametric surface.</summary>
    Adaptor3d_Surface PSurface();
}

/// <summary>
/// The walking of an implicit/parametric intersection: the polylines on a parametric surface where a function
/// F(u, v) vanishes, from start points on the surface's restrictions (open lines) and interior points (closed lines).
/// </summary>
/// <remarks>
/// OCCT's generic file IntWalk_IWalking.gxx, ported once as a generic class (decisions 607 and 5301) over the
/// function (TheIWFunction: IntPatch_TheSurfFunction, later Contap_SurfFunction) and the line it builds (TheIWLine:
/// IntPatch_TheIWLineOfTheIWalking, Contap_TheIWLineOfTheIWalking). The other macros are the same types in both of
/// OCCT's instantiations and are named in the body: the path and interior points (IntSurf_PathPoint,
/// IntSurf_InteriorPoint and their tools), the surface (Adaptor3d_Surface and Adaptor3d_HSurfaceTool).
/// </remarks>
public class IntWalk_IWalking<TheIWFunction, TheIWLine>
    where TheIWFunction : math_FunctionSetWithDerivatives, IIntWalk_IWFunction
    where TheIWLine : IntWalk_TheIWLine, new()
{
    private bool done;
    private readonly NCollection_Sequence<IntSurf_PathPoint> seqSingle = new();
    private double fleche;
    private double pas;
    private readonly math_Vector tolerance = new(1, 2);
    private double epsilon;
    private bool reversed;
    private readonly NCollection_LinearVector<IntWalk_WalkingData> wd1 = new();
    private NCollection_LinearVector<IntWalk_WalkingData> wd2 = new();
    private readonly NCollection_LinearVector<int> nbMultiplicities = new();
    private Bnd_Range mySRangeU = new(); // Estimated U-range for section curve
    private Bnd_Range mySRangeV = new(); // Estimated V-range for section curve
    private double Um;
    private double UM;
    private double Vm;
    private double VM;
    private IntSurf_PntOn2S previousPoint = new();
    private gp_Vec previousd3d = new();
    private gp_Dir2d previousd2d = new();
    private readonly NCollection_Sequence<int> seqAjout = new();
    private readonly NCollection_Sequence<int> seqAlone = new();
    private readonly NCollection_DataMap<int, NCollection_List<int>> PointLineLine = new();
    private readonly NCollection_Sequence<TheIWLine> lines = new();
    private readonly bool ToFillHoles;

    /// <summary>
    /// Deflection is the largest deflection admitted between two consecutive points of a line, Step the largest
    /// increment between two consecutive points (in the parametric space, relative to the surface's range), Epsilon
    /// the distance below which two points are confused. theToFillHoles fills possible holes between the lines
    /// (false for IntPatch's walking).
    /// </summary>
    public IntWalk_IWalking(double Epsilon, double Deflection, double Step, bool theToFillHoles = false)
    {
        done = false;
        fleche = Deflection;
        pas = Step;
        epsilon = Epsilon * Epsilon;
        reversed = false;
        Um = 0.0;
        UM = 0.0;
        Vm = 0.0;
        VM = 0.0;
        ToFillHoles = theToFillHoles;
    }

    public void SetTolerance(double Epsilon, double Deflection, double Step)
    {
        fleche = Deflection;
        pas = Step;
        epsilon = Epsilon * Epsilon;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone() => done;

    /// <summary>The number of lines. Throws StdFail_NotDone if not done.</summary>
    public int NbLines()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return lines.Length();
    }

    /// <summary>The line <paramref name="Index"/>. Throws StdFail_NotDone if not done.</summary>
    public TheIWLine Value(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return lines.Value(Index);
    }

    /// <summary>The number of the start points on which no line starts or ends. Throws StdFail_NotDone if not done.</summary>
    public int NbSinglePnts()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return seqSingle.Length();
    }

    /// <summary>The single point <paramref name="Index"/>. Throws StdFail_NotDone if not done.</summary>
    public IntSurf_PathPoint SinglePnt(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return seqSingle.Value(Index);
    }

    /// <summary>
    /// Searches the lines starting on a point of <paramref name="Pnts1"/> (open lines) or of <paramref name="Pnts2"/>
    /// (closed lines). Each point of a line verifies F(u, v) = 0.
    /// </summary>
    public void Perform(NCollection_Sequence<IntSurf_PathPoint> Pnts1, NCollection_Sequence<IntSurf_InteriorPoint> Pnts2,
                        TheIWFunction Func, Adaptor3d_Surface Caro, bool Reversed = false)
    {
        throw new NotImplementedException();
    }

    /// <summary>Searches the lines starting on a point of <paramref name="Pnts1"/>.</summary>
    public void Perform(NCollection_Sequence<IntSurf_PathPoint> Pnts1, TheIWFunction Func, Adaptor3d_Surface Caro,
                        bool Reversed = false)
    {
        throw new NotImplementedException();
    }
}
