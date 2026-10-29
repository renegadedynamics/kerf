// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_StartPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_StartPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A point of the intersection of two triangles: its 3D point, its parameters on both surfaces, the triangles, the
/// edges it lies on (by number; -1 for a triangle's vertex, -2 for unknown) with its position on them, the angle of the
/// triangles and the section line it belongs to.
/// </summary>
/// <remarks>
/// OCCT's default start point has lambdas -1, angle -2, triangles -1, edges -2 and chain list -1;
/// <c>default(IntPolyh_StartPoint)</c> is all zeros, so ported code writes <c>new IntPolyh_StartPoint()</c>.
/// </remarks>
public partial struct IntPolyh_StartPoint : IEquatable<IntPolyh_StartPoint>, IntPolyh_IDump
{
    private const double MyConfusionPrecision = 10.0e-12;

    private double x;
    private double y;
    private double z;
    private double u1;
    private double v1;
    private double u2;
    private double v2;
    private double lambda1;
    private double lambda2;
    private double angle;
    private int t1;
    private int e1;
    private int t2;
    private int e2;
    private int chainlist;

    public IntPolyh_StartPoint()
    {
        x = 0;
        y = 0;
        z = 0;
        u1 = 0;
        v1 = 0;
        u2 = 0;
        v2 = 0;
        lambda1 = -1.0;
        lambda2 = -1.0;
        angle = -2.0;
        t1 = -1;
        e1 = -2;
        t2 = -1;
        e2 = -2;
        chainlist = -1;
    }

    public IntPolyh_StartPoint(double _x, double _y, double _z, double _u1, double _v1, double _u2, double _v2,
                               int _t1, int _e1, double _lambda1, int _t2, int _e2, double _lambda2, int _chainlist)
    {
        angle = -2.0;
        x = _x;
        y = _y;
        z = _z;
        u1 = _u1;
        v1 = _v1;
        u2 = _u2;
        v2 = _v2;
        t1 = _t1;
        e1 = _e1;
        lambda1 = _lambda1;
        t2 = _t2;
        e2 = _e2;
        lambda2 = _lambda2;
        chainlist = _chainlist;
    }

    public readonly double X() => x;

    public readonly double Y() => y;

    public readonly double Z() => z;

    public readonly double U1() => u1;

    public readonly double V1() => v1;

    public readonly double U2() => u2;

    public readonly double V2() => v2;

    public readonly int T1() => t1;

    public readonly int E1() => e1;

    public readonly double Lambda1() => lambda1;

    public readonly int T2() => t2;

    public readonly int E2() => e2;

    public readonly double Lambda2() => lambda2;

    public readonly double GetAngle() => angle;

    public readonly int ChainList() => chainlist;

    /// <summary>
    /// The points of the edge the start point lies on, in the given triangle, and the triangle's third point: through
    /// E1 if it is set (returns 1), else through E2 (returns 2); 0 if neither is. The outputs are left unchanged where
    /// OCCT leaves them (an edge number other than 1 to 3).
    /// </summary>
    public readonly int GetEdgePoints(in IntPolyh_Triangle Triangle, ref int FirstEdgePoint, ref int SecondEdgePoint,
                                      ref int LastPoint)
    {
        int SurfID;
        if (e1 != -1)
        {
            if (e1 == 1)
            {
                FirstEdgePoint = Triangle.FirstPoint();
                SecondEdgePoint = Triangle.SecondPoint();
                LastPoint = Triangle.ThirdPoint();
            }
            else if (e1 == 2)
            {
                FirstEdgePoint = Triangle.SecondPoint();
                SecondEdgePoint = Triangle.ThirdPoint();
                LastPoint = Triangle.FirstPoint();
            }
            else if (e1 == 3)
            {
                FirstEdgePoint = Triangle.ThirdPoint();
                SecondEdgePoint = Triangle.FirstPoint();
                LastPoint = Triangle.SecondPoint();
            }

            SurfID = 1;
        }
        else if (e2 != -1)
        {
            if (e2 == 1)
            {
                FirstEdgePoint = Triangle.FirstPoint();
                SecondEdgePoint = Triangle.SecondPoint();
                LastPoint = Triangle.ThirdPoint();
            }
            else if (e2 == 2)
            {
                FirstEdgePoint = Triangle.SecondPoint();
                SecondEdgePoint = Triangle.ThirdPoint();
                LastPoint = Triangle.FirstPoint();
            }
            else if (e2 == 3)
            {
                FirstEdgePoint = Triangle.ThirdPoint();
                SecondEdgePoint = Triangle.FirstPoint();
                LastPoint = Triangle.SecondPoint();
            }

            SurfID = 2;
        }
        else
        {
            SurfID = 0;
        }

        return SurfID;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetXYZ(double XX, double YY, double ZZ)
    {
        x = XX;
        y = YY;
        z = ZZ;
    }

    internal void SetUV1(double UU1, double VV1)
    {
        u1 = UU1;
        v1 = VV1;
    }

    internal void SetUV2(double UU2, double VV2)
    {
        u2 = UU2;
        v2 = VV2;
    }

    internal void SetEdge1(int IE1) => e1 = IE1;

    internal void SetLambda1(double LAM1) => lambda1 = LAM1;

    internal void SetEdge2(int IE2) => e2 = IE2;

    internal void SetLambda2(double LAM2) => lambda2 = LAM2;

    internal void SetCoupleValue(int IT1, int IT2)
    {
        t1 = IT1;
        t2 = IT2;
    }

    internal void SetAngle(double Ang) => angle = Ang;

    internal void SetChainList(int ChList) => chainlist = ChList;

    public readonly IntPolyh_StartPoint WithXYZ(double XX, double YY, double ZZ)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetXYZ(XX, YY, ZZ);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithUV1(double UU1, double VV1)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetUV1(UU1, VV1);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithUV2(double UU2, double VV2)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetUV2(UU2, VV2);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithEdge1(int IE1)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetEdge1(IE1);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithLambda1(double LAM1)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetLambda1(LAM1);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithEdge2(int IE2)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetEdge2(IE2);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithLambda2(double LAM2)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetLambda2(LAM2);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithCoupleValue(int IT1, int IT2)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetCoupleValue(IT1, IT2);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithAngle(double Ang)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetAngle(Ang);
        return aCopy;
    }

    public readonly IntPolyh_StartPoint WithChainList(int ChList)
    {
        IntPolyh_StartPoint aCopy = this;
        aCopy.SetChainList(ChList);
        return aCopy;
    }

    /// <summary>1 if SP is the same start point as this one (same edge and position, or the same vertex), else 0.</summary>
    /// <remarks>
    /// When both edges of this point are unknown (-2), OCCT also prints both points and "Can't Check" to standard
    /// output; the port leaves that print out (a library writes nothing by itself), the result is the same.
    /// </remarks>
    public readonly int CheckSameSP(in IntPolyh_StartPoint SP)
    {
        // Returns 1 if this SP == SP
        int Test = 0;
        if (((e1 >= -1) && (e1 == SP.e1)) || ((e2 >= -1) && (e2 == SP.e2)))
        {
            // The edges are defined
            if (((lambda1 > -MyConfusionPrecision) && (Math.Abs(lambda1 - SP.lambda1) < MyConfusionPrecision))
                || ((lambda2 > -MyConfusionPrecision) && (Math.Abs(lambda2 - SP.lambda2) < MyConfusionPrecision)))
            {
                Test = 1;
            }
        }

        if ((Test == 0) && ((e1 == -1) || (e2 == -1)))
        {
            // this SP is a vertex
            if ((Math.Abs(SP.u1 - u1) < MyConfusionPrecision) && (Math.Abs(SP.v1 - v1) < MyConfusionPrecision))
            {
                Test = 1;
            }
        }
        else if ((e1 == -2) && (e2 == -2))
        {
            // OCCT: Dump(00200); SP.Dump(00201); printf("e1==-2 & e2==-2 Can't Check\n"); (left out, see remarks)
        }

        return Test;
    }

    /// <summary>Writes the start point as OCCT's <c>Dump()</c> prints it.</summary>
    public readonly void Dump(TextWriter theStream)
    {
        theStream.Write($"\nPoint : x={IntPolyh_Format.E(x)}g y={IntPolyh_Format.E(y)}g z={IntPolyh_Format.E(z)}g "
                        + $"u1={IntPolyh_Format.E(u1)}g v1={IntPolyh_Format.E(v1)}g u2={IntPolyh_Format.E(u2)}g "
                        + $"v2={IntPolyh_Format.E(v2)}g\n");
        DumpTail(theStream);
    }

    /// <summary>Writes the start point with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter theStream)
    {
        theStream.Write($"\nPoint({IntPolyh_Format.D(i)}) : x={IntPolyh_Format.E(x)}g y={IntPolyh_Format.E(y)}g "
                        + $"z={IntPolyh_Format.E(z)}g u1={IntPolyh_Format.E(u1)}g v1={IntPolyh_Format.E(v1)}g "
                        + $"u2={IntPolyh_Format.E(u2)}g v2={IntPolyh_Format.E(v2)}g\n");
        DumpTail(theStream);
    }

    private readonly void DumpTail(TextWriter theStream)
    {
        theStream.Write($"Triangle S1:{IntPolyh_Format.D(t1)} Edge S1:{IntPolyh_Format.D(e1)} "
                        + $"Lambda1:{IntPolyh_Format.F(lambda1)} Triangle S2:{IntPolyh_Format.D(t2)} "
                        + $"Edge S2:{IntPolyh_Format.D(e2)} Lambda2:{IntPolyh_Format.F(lambda2)}\n");
        theStream.Write($"Angle: {IntPolyh_Format.F(angle)} List Number: {IntPolyh_Format.D(chainlist)}\n");
    }

    public readonly bool Equals(IntPolyh_StartPoint other)
        => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z) && u1.Equals(other.u1) && v1.Equals(other.v1)
           && u2.Equals(other.u2) && v2.Equals(other.v2) && lambda1.Equals(other.lambda1)
           && lambda2.Equals(other.lambda2) && angle.Equals(other.angle) && t1 == other.t1 && e1 == other.e1
           && t2 == other.t2 && e2 == other.e2 && chainlist == other.chainlist;

    public override readonly bool Equals(object? obj) => obj is IntPolyh_StartPoint other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode aHash = new();
        aHash.Add(x);
        aHash.Add(y);
        aHash.Add(z);
        aHash.Add(u1);
        aHash.Add(v1);
        aHash.Add(u2);
        aHash.Add(v2);
        aHash.Add(lambda1);
        aHash.Add(lambda2);
        aHash.Add(angle);
        aHash.Add(t1);
        aHash.Add(e1);
        aHash.Add(t2);
        aHash.Add(e2);
        aHash.Add(chainlist);
        return aHash.ToHashCode();
    }

    public static bool operator ==(in IntPolyh_StartPoint theLeft, in IntPolyh_StartPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPolyh_StartPoint theLeft, in IntPolyh_StartPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntPolyh_StartPoint({gp.Format("xyz", x, y, z)}, {gp.Format("uv1", u1, v1)}, {gp.Format("uv2", u2, v2)}, "
           + $"T {t1}, {t2}; E {e1}, {e2}; {gp.Format("lambda", lambda1, lambda2)}, {gp.Format("angle", angle)}, list {chainlist})";
}
