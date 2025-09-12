using UnityEngine;

/// <summary>
/// Represents an infinite 3D line with origin m_origin and normalized direction m_dir.
/// </summary>
public class Line3d
{
    public Vector3 m_origin;
    public Vector3 m_dir;

    public Line3d() { }

    /// <summary>
    /// Creates a line defined by points A and B. The direction points from A to B.
    /// </summary>
    public Line3d(ref Vector3 a, ref Vector3 b)
    {
        ConstructLine(ref a, ref b);
    }

    /// <summary>
    /// Sets m_origin = A and m_dir = (B - A).normalized.
    /// </summary>
    public void ConstructLine(ref Vector3 a, ref Vector3 b)
    {
        m_origin = a;
        Vector3 dir = (b - a);
        float len = dir.magnitude;
        if (len > 0.0001f)
            m_dir = dir / len;
        else
            m_dir = Vector3.forward;
    }

    /// <summary>
    /// Returns the absolute angle between the directions of two lines.
    /// </summary>
    public float AbsAngleBetween(ref Line3d other)
    {
        return Mathf.Abs(Vector3.Angle(m_dir, other.m_dir));
    }

    /// <summary>
    /// Computes the smallest (signed) displacement vector from 'point' to this line.
    /// Parameter t is the projection on m_dir.
    /// </summary>
    public Vector3 ClosestVectorTo(ref Vector3 point)
    {
        Vector3 delta = point - m_origin;
        float t = Vector3.Dot(delta, m_dir);
        Vector3 closest = m_origin + m_dir * t;
        return closest - point;
    }

    /// <summary>
    /// Returns the closest point on the infinite line to the given point.
    /// </summary>
    public Vector3 ClosestPointOnLineTo(ref Vector3 point)
    {
        Vector3 vec = ClosestVectorTo(ref point);
        return point + vec;
    }
}

/// <summary>
/// Represents a finite line segment [A..B].
/// </summary>
public class LineSeg3d : Line3d
{
    public Vector3 m_end;
    public float m_length;

    public LineSeg3d() { }

    /// <summary>
    /// Creates a line segment from A to B.
    /// </summary>
    public LineSeg3d(ref Vector3 a, ref Vector3 b)
    {
        ConstructLineSeg(ref a, ref b);
    }

    /// <summary>
    /// Initializes the segment with m_origin = A, m_dir = (B - A).normalized, m_end = B, and m_length = |B - A|.
    /// </summary>
    public void ConstructLineSeg(ref Vector3 a, ref Vector3 b)
    {
        ConstructLine(ref a, ref b);
        m_end = b;
        m_length = Vector3.Distance(a, b);
    }

    public enum SegResult
    {
        OnSpan,
        LessThanOrigin,
        GreaterThanEnd,
    }

    /// <summary>
    /// Returns the point on the segment [A..B] that is closest to 'point'. Clamping is applied.
    /// </summary>
    /// <param name="point">Target point</param>
    /// <param name="res">Indicates whether the projection is on the segment, before A, or after B.</param>
    public Vector3 ClosestPointOnSegmentTo(ref Vector3 point, ref SegResult res)
    {
        Vector3 delta = point - m_origin;
        float t = Vector3.Dot(delta, m_dir);

        if (t < 0.0f)
        {
            res = SegResult.LessThanOrigin;
            return m_origin;
        }
        else if (t > m_length)
        {
            res = SegResult.GreaterThanEnd;
            return m_end;
        }
        else
        {
            res = SegResult.OnSpan;
            return m_origin + m_dir * t;
        }
    }

    /// <summary>
    /// Returns the displacement vector that must be subtracted from 'point' to project it onto the segment [A..B].
    /// </summary>
    public Vector3 ClosestVectorTo(ref Vector3 point)
    {
        SegResult dummy = SegResult.OnSpan;
        Vector3 closest = ClosestPointOnSegmentTo(ref point, ref dummy);
        return closest - point;
    }

    /// <summary>
    /// Draws the segment [A..B] as a debug line in the scene.
    /// </summary>
    public void Draw(Color c)
    {
        Debug.DrawLine(m_origin, m_end, c);
    }
}
