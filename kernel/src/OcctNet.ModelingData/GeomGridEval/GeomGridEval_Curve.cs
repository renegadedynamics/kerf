// STUB of GeomGridEval_Curve for the phase2/u3a-surf branch only: point-by-point evaluation through the Geom
// curve. The real port (phase2/u3a) replaces this file; drop this commit when merging.

namespace Occt;

public sealed class GeomGridEval_Curve : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_Curve? myCurve;

    public GeomGridEval_Curve(Geom_Curve? theCurve)
    {
        myCurve = theCurve;
    }

    public NCollection_Array1<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myCurve is null || theParams.IsEmpty())
            return new NCollection_Array1<gp_Pnt>();
        NCollection_Array1<gp_Pnt> aResult = new(1, theParams.Length());
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            aResult[i - theParams.Lower() + 1] = myCurve.EvalD0(theParams.Value(i));
        return aResult;
    }

    public NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myCurve is null || theParams.IsEmpty())
            return new NCollection_Array1<Geom_Curve.ResD1>();
        NCollection_Array1<Geom_Curve.ResD1> aResult = new(1, theParams.Length());
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            aResult[i - theParams.Lower() + 1] = myCurve.EvalD1(theParams.Value(i));
        return aResult;
    }

    public NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myCurve is null || theParams.IsEmpty())
            return new NCollection_Array1<Geom_Curve.ResD2>();
        NCollection_Array1<Geom_Curve.ResD2> aResult = new(1, theParams.Length());
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            aResult[i - theParams.Lower() + 1] = myCurve.EvalD2(theParams.Value(i));
        return aResult;
    }

    public NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myCurve is null || theParams.IsEmpty())
            return new NCollection_Array1<Geom_Curve.ResD3>();
        NCollection_Array1<Geom_Curve.ResD3> aResult = new(1, theParams.Length());
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            aResult[i - theParams.Lower() + 1] = myCurve.EvalD3(theParams.Value(i));
        return aResult;
    }

    public NCollection_Array1<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myCurve is null || theParams.IsEmpty() || theN < 1)
            return new NCollection_Array1<gp_Vec>();
        NCollection_Array1<gp_Vec> aResult = new(1, theParams.Length());
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            aResult[i - theParams.Lower() + 1] = myCurve.EvalDN(theParams.Value(i), theN);
        return aResult;
    }
}
