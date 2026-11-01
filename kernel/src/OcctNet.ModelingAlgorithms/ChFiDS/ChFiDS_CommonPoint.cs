// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CommonPoint.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CommonPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The start or end point of a fillet, common to two adjacent fillets and to an edge of one of the two faces the
/// fillet is built on.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <see cref="ChFiDS_SurfData"/> holds four of them and the fillet builders change
/// them in place through its <c>Change*</c> accessors.
/// </remarks>
public partial struct ChFiDS_CommonPoint : IEquatable<ChFiDS_CommonPoint>
{
    private TopoDS_Edge arc;
    private TopoDS_Vertex vtx;
    private gp_Pnt point;
    private gp_Vec vector;
    private double tol;
    private double prmarc;
    private double prmtg;
    private TopAbs_Orientation traarc;
    private bool isonarc;
    private bool isvtx;
    private bool hasvector;

    /// <summary>Empty constructor.</summary>
    public ChFiDS_CommonPoint()
    {
        arc = new TopoDS_Edge();
        vtx = new TopoDS_Vertex();
        point = new gp_Pnt();
        vector = new gp_Vec();
        tol = 0.0;
        prmarc = 0.0;
        prmtg = 0.0;
        traarc = TopAbs_FORWARD;
        isonarc = false;
        isvtx = false;
        hasvector = false;
    }

    /// <summary>Default value for all fields.</summary>
    internal void Reset()
    {
        tol = 0.0;
        isvtx = false;
        isonarc = false;
        hasvector = false;
    }

    /// <summary>The point is the vertex <paramref name="theVertex"/> of the restriction of one of the surfaces.</summary>
    internal void SetVertex(in TopoDS_Vertex theVertex)
    {
        isvtx = true;
        vtx = theVertex;
    }

    /// <summary>The point is on the arc <paramref name="A"/>, at parameter <paramref name="Param"/>.</summary>
    internal void SetArc(double Tol, in TopoDS_Edge A, double Param, TopAbs_Orientation TArc)
    {
        isonarc = true;
        if (Tol > tol)
        {
            tol = Tol;
        }

        arc = A;
        prmarc = Param;
        traarc = TArc;
    }

    /// <summary>Sets the parameter on the spine.</summary>
    internal void SetParameter(double Param)
    {
        prmtg = Param;
    }

    /// <summary>Sets the 3d point, for a point that is neither a vertex nor on an arc.</summary>
    internal void SetPoint(in gp_Pnt thePoint)
    {
        point = thePoint;
    }

    /// <summary>Sets the output 3d vector.</summary>
    internal void SetVector(in gp_Vec theVector)
    {
        hasvector = true;
        vector = theVector;
    }

    /// <summary>Raises the fuzziness on the point to <paramref name="Tol"/>.</summary>
    internal void SetTolerance(double Tol)
    {
        if (Tol > tol)
        {
            tol = Tol;
        }
    }

    /// <summary>A copy reset to the default values.</summary>
    public readonly ChFiDS_CommonPoint Reseted()
    {
        ChFiDS_CommonPoint aCopy = this;
        aCopy.Reset();
        return aCopy;
    }

    public readonly ChFiDS_CommonPoint WithVertex(in TopoDS_Vertex theVertex)
    {
        ChFiDS_CommonPoint aCopy = this;
        aCopy.SetVertex(theVertex);
        return aCopy;
    }

    public readonly ChFiDS_CommonPoint WithArc(double Tol, in TopoDS_Edge A, double Param, TopAbs_Orientation TArc)
    {
        ChFiDS_CommonPoint aCopy = this;
        aCopy.SetArc(Tol, A, Param, TArc);
        return aCopy;
    }

    public readonly ChFiDS_CommonPoint WithParameter(double Param)
    {
        ChFiDS_CommonPoint aCopy = this;
        aCopy.SetParameter(Param);
        return aCopy;
    }

    public readonly ChFiDS_CommonPoint WithPoint(in gp_Pnt thePoint)
    {
        ChFiDS_CommonPoint aCopy = this;
        aCopy.SetPoint(thePoint);
        return aCopy;
    }

    public readonly ChFiDS_CommonPoint WithVector(in gp_Vec theVector)
    {
        ChFiDS_CommonPoint aCopy = this;
        aCopy.SetVector(theVector);
        return aCopy;
    }

    public readonly ChFiDS_CommonPoint WithTolerance(double Tol)
    {
        ChFiDS_CommonPoint aCopy = this;
        aCopy.SetTolerance(Tol);
        return aCopy;
    }

    /// <summary>The fuzziness on the point.</summary>
    public readonly double Tolerance()
    {
        return tol;
    }

    /// <summary>True if the point is a vertex of the restriction of the surface.</summary>
    public readonly bool IsVertex()
    {
        return isvtx;
    }

    /// <summary>The vertex; throws <see cref="Standard_DomainError"/> if the point is not a vertex.</summary>
    public readonly TopoDS_Vertex Vertex()
    {
        if (!isvtx)
        {
            throw new Standard_DomainError();
        }

        return vtx;
    }

    /// <summary>True if the point is on an edge of the restriction of the surface.</summary>
    public readonly bool IsOnArc()
    {
        return isonarc;
    }

    /// <summary>The arc of restriction containing the point.</summary>
    public readonly TopoDS_Edge Arc()
    {
        if (!isonarc)
        {
            throw new Standard_DomainError("CommonPoint not on Arc");
        }

        return arc;
    }

    /// <summary>The transition of the point on the arc.</summary>
    public readonly TopAbs_Orientation TransitionOnArc()
    {
        if (!isonarc)
        {
            throw new Standard_DomainError("CommonPoint not on Arc");
        }

        return traarc;
    }

    /// <summary>The parameter of the point on the arc.</summary>
    public readonly double ParameterOnArc()
    {
        if (!isonarc)
        {
            throw new Standard_DomainError("CommonPoint not on Arc");
        }

        return prmarc;
    }

    /// <summary>The parameter on the spine.</summary>
    public readonly double Parameter()
    {
        return prmtg;
    }

    /// <summary>The 3d point.</summary>
    public readonly gp_Pnt Point()
    {
        return point;
    }

    /// <summary>True if the output vector is stored.</summary>
    public readonly bool HasVector()
    {
        return hasvector;
    }

    /// <summary>The output 3d vector.</summary>
    public readonly gp_Vec Vector()
    {
        if (!hasvector)
        {
            throw new Standard_DomainError("ChFiDS_CommonPoint::Vector");
        }

        return vector;
    }

    public readonly bool Equals(ChFiDS_CommonPoint other)
        => arc.Equals(other.arc) && vtx.Equals(other.vtx) && point.Equals(other.point) && vector.Equals(other.vector)
           && tol.Equals(other.tol) && prmarc.Equals(other.prmarc) && prmtg.Equals(other.prmtg)
           && traarc == other.traarc && isonarc == other.isonarc && isvtx == other.isvtx
           && hasvector == other.hasvector;

    public override readonly bool Equals(object? obj) => obj is ChFiDS_CommonPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(arc, vtx, point, tol, prmarc, prmtg, isonarc, isvtx);

    public static bool operator ==(in ChFiDS_CommonPoint theLeft, in ChFiDS_CommonPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in ChFiDS_CommonPoint theLeft, in ChFiDS_CommonPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => gp.Format("ChFiDS_CommonPoint", point.X(), point.Y(), point.Z(), tol, prmarc, prmtg);
}
